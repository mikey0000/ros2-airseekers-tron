"""Dock yaw re-measurement from the forward undock chord (mower_docking/dock_yaw.py)."""
import math
import random

from mower_docking import dock_yaw as dy


def track(yaw_deg, n=40, length=0.8, noise=0.005, turn_deg=0.0, seed=1, start=(0.4, -1.3),
          fixed=True, odom_yaw0=0.3):
    r = random.Random(seed)
    a = math.radians(yaw_deg)
    out = []
    for k in range(n):
        f = k / (n - 1)
        d = f * length
        turn = math.radians(turn_deg) * f
        out.append((start[0] + d * math.cos(a) + r.gauss(0, noise),
                    start[1] + d * math.sin(a) + r.gauss(0, noise), odom_yaw0 + turn, fixed))
    return out, odom_yaw0


def test_straight_undock_measures_the_dock_yaw():
    s, y0 = track(104.0)
    res = dy.undock_chord_yaw(s, y0)
    assert res.ok, res.reason
    assert abs(math.degrees(res.yaw) - 104.0) < 1.0
    assert res.span_m > 0.6 and res.rms_m < 0.01


def test_docked_point_is_skipped():
    # the first (docked) sample is gauge-reset onto the stored dock pose: 5 cm sideways
    s, y0 = track(100.0)
    s[0] = (s[0][0] + 0.05, s[0][1], s[0][2], True)
    res = dy.undock_chord_yaw(s, y0)
    assert res.ok and abs(math.degrees(res.yaw) - 100.0) < 1.0


def test_gates():
    assert not dy.undock_chord_yaw(*track(100.0, fixed=False)).ok
    assert 'turned' in dy.undock_chord_yaw(*track(100.0, turn_deg=6.0)).reason
    assert 'straight' in dy.undock_chord_yaw(*track(100.0, noise=0.04)).reason
    assert 'span' in dy.undock_chord_yaw(*track(100.0, length=0.35)).reason


def test_small_turn_is_compensated():
    # heading hold let 2 deg through: the line direction is the mean heading, not the start
    s, y0 = track(100.0, turn_deg=2.0)
    res = dy.undock_chord_yaw(s, y0)
    assert res.ok and abs(math.degrees(res.yaw) - 100.0) < 1.6


def test_apply_decision():
    res = dy.undock_chord_yaw(*track(104.0))
    ok, why = dy.apply_decision(res, math.radians(100.7))
    assert ok and '+3' in why
    ok, why = dy.apply_decision(res, math.radians(70.0))
    assert not ok and 'not applied' in why
