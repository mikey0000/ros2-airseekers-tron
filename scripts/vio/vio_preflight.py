#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Preflight for ``scripts/vio/record_vio_bag.sh`` (2026-11-14): are the VIO inputs live?

Runs INSIDE the mower_jazzy container (rclpy, the stack's DDS settings) for a few seconds and
checks what the offline VIO evaluation (docs/vio.md section 8) needs from the recording:

* the stereo pair ``/vio/{left,right}/image_raw`` at ~10 Hz with matching stamps (stereo_cam
  publishes both eyes with one stamp; split pairs mean the host socket buffers are small,
  ``scripts/setup_stereo_host.sh``);
* the Metoak IMU ``/stereo_imu/data`` at ~200 Hz (needs the IIO driver mode, same script);
* the WIT IMU, ``/odom``, ``/fix`` present; RTK FIXED now (``/fix_status``, the same test as
  ``vio_gate``) so the bag starts with truth;
* enough free space on /userdata for the requested minutes, keeping bag_recorder's 3 GB floor.

Subscriptions are raw (no deserialisation of the 300 KB images): the header stamp is read
straight from the CDR bytes. Exit 0 when every required check passes, 1 otherwise (the
report says which and what to do), 2 on usage errors.

    python3 scripts/vio/vio_preflight.py --seconds 6 --free-gb 12.5 --minutes 20
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(STACK, 'src', 'mower_localization'))

from mower_localization.vio_gate_logic import rtk_is_fixed  # noqa: E402

LEFT, RIGHT = '/vio/left/image_raw', '/vio/right/image_raw'
STEREO_IMU, WIT_IMU = '/stereo_imu/data', '/imu/data'
ACTIVITY = '/mission/activity'   # latched; idle values cap the camera rates

PAIR_HZ_MIN = 9.0           # stereo_cam delivers 10.0 Hz exact pairs with 8 MiB buffers
UNPAIRED_MAX = 0.05         # share of frames without a partner (2.9 Hz pairs before the fix)
STEREO_IMU_HZ_MIN = 190.0   # 200.35 Hz measured (docs/vio.md 7.2)
WIT_IMU_HZ_MIN = 80.0       # 100 Hz nominal
ODOM_HZ_MIN = 20.0          # mcu_node 50 Hz
FIX_HZ_MIN = 2.0            # um960 10 Hz; only "is it alive"
RTK_FIXED_SHARE_MIN = 0.8   # of the /fix_status lines in the listen window

# Bag size: two mono8 640x480 eyes at the pair rate dominate; everything else (both IMUs at
# 300 Hz together, odometry, tf, fix) adds about 0.25 MB/s. sqlite3 overhead is ~1 %.
EYE_BYTES = 640 * 480
OTHER_MB_S = 0.25
RESERVE_GB = 3.0 + 1.0      # bag_recorder's min_free_gb floor + 1 GB margin


def cdr_stamp(raw: bytes) -> Optional[float]:
    """Header stamp (s) of a serialized message whose first field is a std_msgs/Header.

    CDR: 4-byte encapsulation (byte 1: 0x01 little endian, 0x00 big endian), then
    builtin_interfaces/Time {int32 sec, uint32 nanosec}. None if too short / unknown."""
    if len(raw) < 12 or raw[0] != 0 or raw[1] not in (0, 1):
        return None
    sec, nsec = struct.unpack_from('<iI' if raw[1] == 1 else '>iI', raw, 4)
    if nsec >= 1000000000:
        return None
    return sec + nsec * 1e-9


def rate_hz(n: int, window_s: float) -> float:
    """Messages per second over the listen window."""
    return n / window_s if window_s > 0 else 0.0


def max_gap(times: Sequence[float]) -> float:
    """Largest interval between consecutive (sorted) times; inf with fewer than two."""
    ts = sorted(times)
    if len(ts) < 2:
        return math.inf
    return max(b - a for a, b in zip(ts, ts[1:]))


def pair_stats(left: Sequence[float], right: Sequence[float], window_s: float) -> Dict:
    """Exact-stamp stereo pairing (stamps rounded to 1 us): pair rate, unpaired share and the
    largest gap between pairs."""
    lk = {round(t * 1e6) for t in left}
    rk = {round(t * 1e6) for t in right}
    both = sorted(lk & rk)
    frames = len(lk) + len(rk)
    unpaired = len(lk ^ rk)
    return {'pairs': len(both), 'pair_hz': rate_hz(len(both), window_s),
            'unpaired_share': unpaired / frames if frames else 1.0,
            'max_pair_gap_s': max_gap([k * 1e-6 for k in both])}


def rtk_summary(status_lines: Sequence[str]) -> Dict:
    """Share of /fix_status lines that say RTK FIXED, and whether the last one does."""
    if not status_lines:
        return {'n': 0, 'fixed_share': 0.0, 'fixed_now': False}
    fixed = [rtk_is_fixed(s) for s in status_lines]
    return {'n': len(fixed), 'fixed_share': sum(fixed) / len(fixed), 'fixed_now': fixed[-1]}


def bag_mb_per_min(pair_hz: float = 10.0) -> float:
    """Expected recording rate in MB (1e6 B) per minute for the record_vio_bag.sh topic set."""
    return (2 * EYE_BYTES * pair_hz / 1e6 + OTHER_MB_S) * 60.0


def minutes_that_fit(free_gb: float, pair_hz: float = 10.0, reserve_gb: float = RESERVE_GB) -> float:
    """Recording minutes that fit in ``free_gb`` while keeping ``reserve_gb`` free."""
    return max(0.0, (free_gb - reserve_gb) * 1000.0 / bag_mb_per_min(pair_hz))


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    required: bool = True
    fix: str = ''


IDLE_ACTIVITIES = frozenset(('docked_idle', 'idle'))
IDLE_PAIR_HZ_MIN = 0.8
IDLE_STEREO_IMU_HZ_MIN = 90.0

SETUP_HINT = ('on the mower host: sudo /userdata/ros2_stack/scripts/setup_stereo_host.sh, '
              'then restart the stack (docker restart mower_jazzy)')


def idle_capped(activity: Optional[str]) -> bool:
    """True while /mission/activity lets the cameras run at their idle rate (docked or parked:
    1 stereo pair/s, stereo IMU 100 Hz; mower_cameras/activity_gate.py). No message = full rate."""
    return activity in IDLE_ACTIVITIES


def evaluate(obs: Dict, minutes: Optional[float] = None,
             free_gb: Optional[float] = None) -> List[Check]:
    """Checks from observations ``obs``: window_s, left / right stamp lists, counts per topic
    (stereo_imu, wit_imu, odom, fix), fix_status lines and the latest /mission/activity.

    While the activity is idle the camera nodes run their idle cap on purpose, so the stereo
    checks then only test that frames flow and pair (the full rate starts when the mower drives
    in manual mode or mows)."""
    w = obs['window_s']
    ps = pair_stats(obs['left'], obs['right'], w)
    idle = idle_capped(obs.get('activity'))
    pair_min, imu_min = (IDLE_PAIR_HZ_MIN, IDLE_STEREO_IMU_HZ_MIN) if idle else (
        PAIR_HZ_MIN, STEREO_IMU_HZ_MIN)
    note = (' [idle cap: activity %s; full rate once it drives in manual mode or mows]'
            % obs.get('activity')) if idle else ''
    out = [Check('stereo pair', ps['pair_hz'] >= pair_min
                 and ps['unpaired_share'] <= UNPAIRED_MAX,
                 '%.1f Hz exact pairs, %.0f %% unpaired, max gap %.2f s (need >= %.1f Hz, '
                 '<= %.0f %%)%s' % (ps['pair_hz'], 100 * ps['unpaired_share'],
                                     ps['max_pair_gap_s'], pair_min, 100 * UNPAIRED_MAX, note),
                 fix='no frames: is the stack up with stereo:=true? split pairs: ' + SETUP_HINT)]
    hz = rate_hz(obs['stereo_imu'], w)
    out.append(Check('stereo IMU', hz >= imu_min,
                     '%s %.0f Hz (need >= %.0f)%s' % (STEREO_IMU, hz, imu_min, note),
                     fix=SETUP_HINT))
    hz = rate_hz(obs['wit_imu'], w)
    out.append(Check('WIT IMU', hz >= WIT_IMU_HZ_MIN,
                     '%s %.0f Hz (need >= %.0f)' % (WIT_IMU, hz, WIT_IMU_HZ_MIN),
                     required=False, fix='wit_imu_driver not publishing: check the stack log'))
    hz = rate_hz(obs['odom'], w)
    out.append(Check('wheel odometry', hz >= ODOM_HZ_MIN, '/odom %.0f Hz (need >= %.0f)'
                     % (hz, ODOM_HZ_MIN), fix='mcu_node not publishing: check the stack log'))
    hz = rate_hz(obs['fix'], w)
    out.append(Check('GNSS', hz >= FIX_HZ_MIN, '/fix %.1f Hz' % hz,
                     fix='um960 driver not publishing: check the stack log'))
    rs = rtk_summary(obs['fix_status'])
    out.append(Check('RTK fixed', rs['fixed_now'] and rs['fixed_share'] >= RTK_FIXED_SHARE_MIN,
                     'now %s, %.0f %% of %d status lines fixed' % (
                         'FIXED' if rs['fixed_now'] else 'not fixed', 100 * rs['fixed_share'],
                         rs['n']),
                     fix='start in open sky and wait for RTK FIXED (GUI), then run again'))
    if free_gb is not None and minutes is not None:
        fit = minutes_that_fit(free_gb, max(ps['pair_hz'], 10.0))
        out.append(Check('disk', fit >= minutes,
                         '/userdata %.1f GB free: %.0f min fit at %.0f MB/min (asked %.0f min, '
                         'keeping %.0f GB free)' % (free_gb, fit, bag_mb_per_min(), minutes,
                                                    RESERVE_GB),
                         fix='record fewer minutes (--minutes %d) or copy off / delete old '
                             'vio bags in /userdata/ros2/vio_bags' % int(fit)))
    return out


def report(checks: Sequence[Check]) -> int:
    """Print the checks; 0 if every required one passed, else 1."""
    bad = 0
    for c in checks:
        tag = 'ok  ' if c.ok else ('FAIL' if c.required else 'warn')
        print('  %s %-15s %s' % (tag, c.name, c.detail))
        if not c.ok:
            print('       -> %s' % c.fix)
            bad += c.required
    return 1 if bad else 0


def listen(seconds: float) -> Dict:
    """Subscribe raw to the checked topics for ``seconds``; return the observations."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, Imu, NavSatFix
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('vio_preflight')
    be = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST)
    obs = {'left': [], 'right': [], 'stereo_imu': 0, 'wit_imu': 0, 'odom': 0, 'fix': 0,
           'fix_status': [], 'activity': None}

    def stamps(key):
        def cb(raw):
            t = cdr_stamp(raw)
            if t is not None:
                obs[key].append(t)
        return cb

    def count(key):
        def cb(_raw):
            obs[key] += 1
        return cb

    node.create_subscription(Image, LEFT, stamps('left'), be, raw=True)
    node.create_subscription(Image, RIGHT, stamps('right'), be, raw=True)
    node.create_subscription(Imu, STEREO_IMU, count('stereo_imu'), be, raw=True)
    node.create_subscription(Imu, WIT_IMU, count('wit_imu'), be, raw=True)
    node.create_subscription(Odometry, '/odom', count('odom'), be, raw=True)
    node.create_subscription(NavSatFix, '/fix', count('fix'), be, raw=True)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, ACTIVITY, lambda m: obs.__setitem__('activity', m.data),
                             latched)
    node.create_subscription(String, '/fix_status',
                             lambda m: obs['fix_status'].append(m.data), 50)
    # discovery + stereo_cam's on-demand start, then the counted window
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for k in ('left', 'right', 'fix_status'):  # activity is latched: keep it
        obs[k] = []
    for k in ('stereo_imu', 'wit_imu', 'odom', 'fix'):
        obs[k] = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    obs['window_s'] = time.monotonic() - t0
    node.destroy_node()
    rclpy.shutdown()
    return obs


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--minutes', type=float, default=None, help='planned recording length')
    ap.add_argument('--free-gb', type=float, default=None, help='free space on /userdata')
    a = ap.parse_args(argv)
    obs = listen(a.seconds)
    print('VIO preflight (%.1f s listen, activity %s):' % (obs['window_s'], obs['activity']))
    return report(evaluate(obs, a.minutes, a.free_gb))


if __name__ == '__main__':
    sys.exit(main())
