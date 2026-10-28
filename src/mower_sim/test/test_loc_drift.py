# SPDX-License-Identifier: Apache-2.0
"""ROS-free tests for mower_sim.loc_drift (field-like drifting map -> odom)."""

import math
import statistics

import pytest

from mower_sim.loc_drift import (LocDrift, LocDriftParams, compose, field_params, inverse,
                                 params_for)

DT = 0.02


def _drive(ld, seconds=120.0, v=0.2):
    """Back-and-forth swaths (8 m, pivot 180 deg) at v; yields (truth, odom, sample)."""
    x, y, th = 0.0, 0.0, 0.0
    t, leg, out = 0.0, 0.0, []
    turning = 0.0
    while t < seconds:
        if turning > 0.0:
            dth = min(0.3 * DT, turning)
            th += dth
            turning -= dth
        else:
            x += v * DT * math.cos(th)
            y += v * DT * math.sin(th)
            leg += v * DT
            if leg >= 8.0:
                leg, turning = 0.0, math.pi
        t += DT
        truth = (x, y, math.atan2(math.sin(th), math.cos(th)))
        odom = ld.step(truth, DT)
        s = ld.sample(truth) if ld.due() else None
        out.append((truth, odom, s))
    return out


def test_disabled_is_perfect_localisation():
    ld = LocDrift(LocDriftParams(enabled=False), (1.0, 2.0, 0.5))
    for truth, odom, s in _drive(ld, 20.0):
        assert odom == truth
        if s is not None:
            assert s[0] == truth and s[1] == (0.0, 0.0, 0.0)


def test_map_odom_composes_to_the_map_estimate_and_stays_near_the_truth():
    ld = LocDrift(LocDriftParams(enabled=True, seed=3), (0.0, 0.0, 0.0))
    worst = 0.0
    for truth, odom, s in _drive(ld, 120.0):
        if s is None:
            continue
        est, m2o = s
        back = compose(m2o, odom)
        assert math.hypot(back[0] - est[0], back[1] - est[1]) < 1e-9
        assert abs(math.atan2(math.sin(back[2] - est[2]), math.cos(back[2] - est[2]))) < 1e-9
        worst = max(worst, math.hypot(est[0] - truth[0], est[1] - truth[1]))
    assert worst < 0.2          # the RTK map estimate stays a few cm off the truth


def test_odom_is_smooth_but_drifts_away_from_the_truth():
    ld = LocDrift(LocDriftParams(enabled=True, seed=1), (0.0, 0.0, 0.0))
    run = _drive(ld, 240.0)
    prev = None
    for truth, odom, _ in run:
        if prev is not None:
            # never a jump: at most the true step (0.004 m at 0.2 m/s) + its scale error
            assert math.hypot(odom[0] - prev[0], odom[1] - prev[1]) < 0.0045
        prev = odom
    truth, odom, _ = run[-1]
    assert math.hypot(odom[0] - truth[0], odom[1] - truth[1]) > 0.2


def test_map_odom_steps_match_the_field_while_driving():
    # run 6 (2026-10-16), map -> odom at 10 Hz while driving: median 5.4-5.7 mm, p90 13 mm,
    # 97 % of the steps > 1 mm
    ld = LocDrift(LocDriftParams(enabled=True, seed=2), (0.0, 0.0, 0.0))
    m2o = [s[1] for _, _, s in _drive(ld, 300.0) if s is not None]
    steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(m2o, m2o[1:])]
    med = statistics.median(steps)
    assert 0.003 < med < 0.010
    assert sum(1 for s in steps if s > 0.001) / len(steps) > 0.9


def test_seeded_and_repeatable():
    a = [s for _, _, s in _drive(LocDrift(LocDriftParams(enabled=True, seed=5), (0, 0, 0)), 30)]
    b = [s for _, _, s in _drive(LocDrift(LocDriftParams(enabled=True, seed=5), (0, 0, 0)), 30)]
    c = [s for _, _, s in _drive(LocDrift(LocDriftParams(enabled=True, seed=6), (0, 0, 0)), 30)]
    assert a == b
    assert a != c


def test_inverse_compose_identity():
    p = (1.5, -2.0, 2.8)
    q = compose(p, inverse(p))
    assert all(abs(v) < 1e-12 for v in q)


# ------------------------------------------------------------ field model (2026-10-29)

def test_legacy_model_is_the_dataclass_default():
    assert params_for('legacy') == LocDriftParams()
    assert params_for('legacy', seed=4).seed == 4
    with pytest.raises(ValueError):
        params_for('nope')


def test_legacy_samples_are_unchanged_by_the_field_hooks():
    # legacy: no latency, no lever: the map estimate is the truth plus noise, as before
    a = LocDrift(LocDriftParams(enabled=True, seed=5), (0.0, 0.0, 0.0))
    b = LocDrift(params_for('legacy', enabled=True, seed=5), (0.0, 0.0, 0.0))
    ra, rb = _drive(a, 30.0), _drive(b, 30.0)
    assert [r[2] for r in ra] == [r[2] for r in rb]


def test_field_model_has_no_jumps_and_latency():
    p = field_params()
    assert p.model == 'field' and p.jump_rate_hz == 0.0
    assert p.fix_latency_s == pytest.approx(0.1)
    assert p.antenna_x == p.navsat_lever_x       # no static lever error at rest
    assert not p.odom_lateral


def _quiet_field(**kw):
    return field_params(enabled=True, map_err_std=0.0, jitter_std=0.0, yaw_jitter_std=0.0, **kw)


def test_field_map_estimate_lags_the_truth_by_the_fix_latency():
    ld = LocDrift(_quiet_field(), (0.0, 0.0, 0.0))
    v, x = 0.2, 0.0
    for _ in range(100):
        x += v * DT
        ld.step((x, 0.0, 0.0), DT)
    est, _ = ld.sample((x, 0.0, 0.0))
    assert est[0] == pytest.approx(x - v * 0.1, abs=1e-6)
    assert est[1] == pytest.approx(0.0, abs=1e-9)


def test_field_standstill_has_no_offset():
    ld = LocDrift(_quiet_field(), (1.0, 2.0, 0.7))
    for _ in range(20):
        ld.step((1.0, 2.0, 0.7), DT)
    est, _ = ld.sample((1.0, 2.0, 0.7))
    assert est[0] == pytest.approx(1.0) and est[1] == pytest.approx(2.0)


def test_field_lever_with_latency_in_a_pivot():
    # pivot at 0.3 rad/s about base_link: the delayed antenna minus the lever along the
    # current yaw is off by about lever x yaw change over the latency
    ld = LocDrift(_quiet_field(), (0.0, 0.0, 0.0))
    th = 0.0
    for _ in range(50):
        th += 0.3 * DT
        ld.step((0.0, 0.0, th), DT)
    est, _ = ld.sample((0.0, 0.0, th))
    assert math.hypot(est[0], est[1]) == pytest.approx(0.098 * 0.3 * 0.1, rel=0.05)


def test_field_odom_ignores_lateral_motion():
    ld = LocDrift(field_params(enabled=True, odom_scale_err=0.0, odom_yaw_scale_err=0.0,
                               odom_yaw_bias_walk=0.0), (0.0, 0.0, 0.0))
    y = 0.0
    for _ in range(50):
        y += 0.01
        odom = ld.step((0.0, y, 0.0), DT)
    assert odom[1] == pytest.approx(0.0, abs=1e-12)
