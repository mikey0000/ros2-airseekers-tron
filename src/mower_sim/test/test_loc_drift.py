# SPDX-License-Identifier: Apache-2.0
"""ROS-free tests for mower_sim.loc_drift (field-like drifting map -> odom)."""

import math
import statistics

from mower_sim.loc_drift import LocDrift, LocDriftParams, compose, inverse

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
