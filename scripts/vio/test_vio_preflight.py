# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for scripts/vio/vio_preflight.py's pure checks (no ROS graph needed)."""

import math
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import vio_preflight as vp  # noqa: E402

FIXED = 'quality=RTK_FIXED sats=24'
FLOAT = 'quality=RTK_FLOAT sats=20'


def _cdr(sec, nsec, little=True):
    head = b'\x00\x01\x00\x00' if little else b'\x00\x00\x00\x00'
    return head + struct.pack('<iI' if little else '>iI', sec, nsec) + b'\x07\x00\x00\x00frame\x00'


def test_cdr_stamp_reads_header_both_endians():
    assert vp.cdr_stamp(_cdr(1700000000, 250000000)) == 1700000000.25
    assert vp.cdr_stamp(_cdr(12, 5, little=False)) == 12 + 5e-9


def test_cdr_stamp_rejects_garbage():
    assert vp.cdr_stamp(b'\x00\x01') is None
    assert vp.cdr_stamp(b'\x01\x01\x00\x00' + b'\x00' * 8) is None          # bad encapsulation
    assert vp.cdr_stamp(_cdr(1, 1000000000)) is None                       # nanosec overflow


def test_pair_stats_exact_stamps():
    left = [100.0 + 0.1 * k for k in range(60)]
    right = list(left[:50])                    # last 10 right eyes lost
    s = vp.pair_stats(left, right, 6.0)
    assert s['pairs'] == 50
    assert abs(s['pair_hz'] - 50 / 6.0) < 1e-9
    assert abs(s['unpaired_share'] - 10 / 110) < 1e-9
    assert abs(s['max_pair_gap_s'] - 0.1) < 1e-6


def test_pair_stats_no_frames():
    s = vp.pair_stats([], [], 6.0)
    assert s['pairs'] == 0 and s['unpaired_share'] == 1.0 and math.isinf(s['max_pair_gap_s'])


def test_rtk_summary_uses_gate_rule():
    s = vp.rtk_summary([FLOAT, FIXED, FIXED, FIXED, FIXED])
    assert s['fixed_now'] and abs(s['fixed_share'] - 0.8) < 1e-9
    assert not vp.rtk_summary([FIXED, 'STALE ' + FIXED])['fixed_now']
    assert vp.rtk_summary([])['n'] == 0


def test_bag_rate_and_disk_budget():
    # two 300 KB eyes at 10 Hz + 0.25 MB/s: ~383 MB per minute
    assert abs(vp.bag_mb_per_min() - 383.6) < 0.5
    assert vp.minutes_that_fit(3.5) == 0.0                 # below the 4 GB reserve
    assert abs(vp.minutes_that_fit(12.0) - 8000 / 383.64) < 0.1


def _obs(**kw):
    w = 6.0
    pairs = [1000.0 + 0.1 * k for k in range(60)]
    o = {'window_s': w, 'left': pairs, 'right': list(pairs), 'stereo_imu': 1200,
         'wit_imu': 600, 'odom': 300, 'fix': 60, 'fix_status': [FIXED] * 12}
    o.update(kw)
    return o


def _by_name(checks):
    return {c.name: c for c in checks}


def test_evaluate_all_good():
    checks = vp.evaluate(_obs(), minutes=20, free_gb=14.0)
    assert all(c.ok for c in checks), [c for c in checks if not c.ok]
    assert vp.report(checks) == 0


def test_evaluate_split_pairs_and_slow_imu_fail():
    left = [1000.0 + 0.1 * k for k in range(60)]
    c = _by_name(vp.evaluate(_obs(right=left[::3], stereo_imu=600)))
    assert not c['stereo pair'].ok and 'setup_stereo_host' in c['stereo pair'].fix
    assert not c['stereo IMU'].ok
    assert vp.report(list(c.values())) == 1


def test_evaluate_rtk_float_now_fails_even_if_mostly_fixed():
    c = _by_name(vp.evaluate(_obs(fix_status=[FIXED] * 11 + [FLOAT])))
    assert not c['RTK fixed'].ok


def test_evaluate_wit_imu_is_only_a_warning():
    checks = vp.evaluate(_obs(wit_imu=0))
    c = _by_name(checks)
    assert not c['WIT IMU'].ok and not c['WIT IMU'].required
    assert vp.report(checks) == 0


def test_evaluate_disk_suggests_shorter_recording():
    c = _by_name(vp.evaluate(_obs(), minutes=30, free_gb=10.0))
    assert not c['disk'].ok
    assert '--minutes 15' in c['disk'].fix          # (10 - 4) GB / 384 MB/min = 15.6


def _idle_obs(activity):
    # the 2026-11-14 docked check: 1.2 Hz pairs, stereo IMU 100 Hz
    pairs = [1000.0 + 0.85 * k for k in range(7)]
    return _obs(left=pairs, right=list(pairs), stereo_imu=600, activity=activity)


def test_evaluate_idle_cap_passes_the_stereo_checks_with_a_note():
    c = _by_name(vp.evaluate(_idle_obs('docked_idle')))
    assert c['stereo pair'].ok and c['stereo IMU'].ok
    assert 'idle cap' in c['stereo pair'].detail and 'manual mode' in c['stereo pair'].detail


def test_evaluate_same_rates_fail_when_not_idle():
    for activity in ('manual', 'mowing', None):
        c = _by_name(vp.evaluate(_idle_obs(activity)))
        assert not c['stereo pair'].ok and not c['stereo IMU'].ok, activity


def test_evaluate_idle_cap_still_fails_without_frames_or_with_split_pairs():
    left = [1000.0 + 0.85 * k for k in range(7)]
    c = _by_name(vp.evaluate(_obs(left=[], right=[], stereo_imu=600, activity='idle')))
    assert not c['stereo pair'].ok
    c = _by_name(vp.evaluate(_obs(left=left, right=left[::2], stereo_imu=600, activity='idle')))
    assert not c['stereo pair'].ok


def test_idle_capped_values():
    assert vp.idle_capped('docked_idle') and vp.idle_capped('idle')
    assert not vp.idle_capped('manual') and not vp.idle_capped(None)
