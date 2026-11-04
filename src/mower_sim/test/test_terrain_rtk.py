# SPDX-License-Identifier: Apache-2.0
"""ROS-free tests for the slope / slip (terrain.py) and RTK degradation (rtk.py) models,
calibrated against the 2026-11-02 side-slope field evidence (docs/simulation.md "Terrain")."""

import math

import pytest

from mower_sim import world as W
from mower_sim.kinematics import DiffDrive
from mower_sim.rtk import DROPOUT, FIXED, FLOAT, RtkModel, parse_spec, rtk_from_dict
from mower_sim.terrain import terrain_from_dict

DT = 0.02
SLOPE = {'regions': [{'name': 'west', 'polygon': [[-50, -50], [50, -50], [50, 50], [-50, 50]],
                      'slope_deg': 10.0, 'downhill_deg': 180.0, 'fade_m': 0.0}]}


def _drive(terrain=SLOPE, yaw=math.pi / 2):
    doc = {'lawn': [[-60, -60], [60, -60], [60, 60], [-60, 60]],
           'dock': {'x': -500.0, 'y': 0.0, 'yaw': 0.0},
           'start': {'x': 0.0, 'y': 0.0, 'yaw': yaw}}
    if terrain is not None:
        doc['terrain'] = terrain
    return DiffDrive(W.world_from_dict(doc))


def _run(d, v, w, seconds):
    t = 0.0
    for _ in range(int(round(seconds / DT))):
        d.command(v, w)
        d.step(DT, t)
        t += DT


def test_flat_world_has_no_terrain_and_wheel_odom_matches_truth():
    d = _drive(terrain=None)
    assert d.terrain is None
    _run(d, 0.2, 0.1, 10.0)
    assert d.wheel_odom.x == pytest.approx(d.pose.x, abs=1e-6)
    assert d.wheel_odom.y == pytest.approx(d.pose.y, abs=1e-6)
    assert d.creep == (0.0, 0.0)


def test_roll_pitch_signs():
    t = terrain_from_dict(SLOPE)          # downhill = -x (west)
    # heading north: west (downhill) is on the left -> left side down -> roll < 0
    roll, pitch = t.roll_pitch(0.0, 0.0, math.pi / 2)
    assert math.degrees(roll) == pytest.approx(-10.0, abs=0.05)
    assert abs(pitch) < 1e-9
    # heading west: nose downhill -> pitch > 0
    roll, pitch = t.roll_pitch(0.0, 0.0, math.pi)
    assert math.degrees(pitch) == pytest.approx(10.0, abs=0.05)
    assert abs(roll) < 1e-9
    assert t.roll_pitch(100.0, 0.0, 0.0) == (0.0, 0.0)       # outside the region


def test_fade_and_onset():
    t = terrain_from_dict({'regions': [{'polygon': [[0, 0], [4, 0], [4, 4], [0, 4]],
                                        'slope_deg': 10.0, 'fade_m': 1.0}]})
    assert math.degrees(t.gradient(0.5, 2.0)[0]) == pytest.approx(5.0, abs=0.01)
    assert math.degrees(t.gradient(2.0, 2.0)[0]) == pytest.approx(10.0)
    assert t.slip.factor(math.radians(2.0)) == 0.0
    assert t.slip.factor(math.radians(12.0)) == 1.0


def test_reverse_on_10deg_side_slope_matches_field_ratio():
    """Field 2026-11-02: 3.8 m wheel odom vs 0.4 m real at 8-12 deg roll, downhill creep."""
    d = _drive()
    x0, y0 = d.pose.x, d.pose.y
    _run(d, -0.1, 0.0, 38.0)
    real = math.hypot(d.pose.x - x0, d.pose.y - y0)
    assert d.wheel_dist == pytest.approx(3.8, rel=0.05)
    ratio = d.wheel_dist / real
    assert 6.0 < ratio < 14.0, ratio
    assert d.pose.x - x0 < -0.3           # crept downhill (west) ~1 cm/s
    assert (x0 - d.pose.x) / 38.0 == pytest.approx(0.010, abs=0.002)


def test_forward_mowing_still_makes_progress():
    d = _drive()
    _run(d, 0.2, 0.0, 10.0)
    real = d.pose.y
    assert 1.3 < real < 1.7               # 25 % slip at 0.2 m/s
    assert d.wheel_dist / math.hypot(d.pose.x, d.pose.y) < 1.5   # no slip-check trip (3x)


def test_pivot_on_slope_slides():
    d = _drive()
    _run(d, 0.0, 0.3, 6.5)                # ~90 deg pivot (yaw loss included)
    slide = math.hypot(d.pose.x, d.pose.y)
    assert 0.15 <= slide <= 0.30, slide
    assert d.pose.x < 0.0                 # downhill
    flat = _drive(terrain=None)
    _run(flat, 0.0, 0.3, 6.5)
    assert math.hypot(flat.pose.x, flat.pose.y) < 1e-6


def test_standing_still_does_not_creep():
    d = _drive()
    _run(d, 0.0, 0.0, 5.0)
    assert (d.pose.x, d.pose.y) == (0.0, 0.0)


def test_terrain_rejects_unknown_slip_param():
    with pytest.raises(ValueError):
        terrain_from_dict(dict(SLOPE, slip={'nope': 1}))
    assert terrain_from_dict({'regions': []}) is None


def _steps(m, t0, t1, dt=0.1, docked=False, cutting=True, xy=(0.0, 0.0)):
    out = []
    for i in range(int(round((t1 - t0) / dt))):
        t = round(t0 + i * dt, 6)
        m.notify(t, docked, cutting)
        out.append((t,) + m.step(t, dt, *xy))
    return out


def test_rtk_events_anchor_on_blade():
    m = RtkModel(rtk_from_dict({'events': [{'t0': 10, 'dur': 5, 'kind': 'float'},
                                           {'t0': 20, 'dur': 3, 'kind': 'dropout'}]}))
    assert [r[1] for r in _steps(m, 0.0, 50.0, cutting=False)] == [FIXED] * 500
    rows = _steps(m, 50.0, 80.0)          # blade on at 50 -> events at 60-65, 70-73
    modes = {round(r[0], 1): r[1] for r in rows}
    assert modes[59.9] == FIXED and modes[60.0] == FLOAT and modes[64.9] == FLOAT
    assert modes[65.0] == FIXED and modes[70.0] == DROPOUT and modes[73.0] == FIXED
    off = [math.hypot(r[2], r[3]) for r in rows if r[1] == FLOAT]
    assert min(off) >= 0.3 - 1e-9 and max(off) <= 1.0 + 1e-9
    assert all(r[2] == 0.0 and r[3] == 0.0 for r in rows if r[1] != FLOAT)


def test_rtk_float_is_a_random_walk_and_seeded():
    spec = {'anchor': 'sim', 'events': [{'t0': 0, 'dur': 100, 'kind': 'float'}], 'seed': 4}
    a = _steps(RtkModel(rtk_from_dict(spec)), 0.0, 100.0)
    b = _steps(RtkModel(rtk_from_dict(spec)), 0.0, 100.0)
    assert a == b
    steps = [math.hypot(a[i + 1][2] - a[i][2], a[i + 1][3] - a[i][3]) for i in range(len(a) - 1)]
    assert sum(steps) / len(steps) < 0.08   # continuous, not white noise
    assert max(math.hypot(r[2], r[3]) for r in a) > 0.45


def test_rtk_undock_anchor_and_multipath():
    m = RtkModel(rtk_from_dict({'anchor': 'undock', 'events': [{'t0': 1, 'dur': 1}],
                                'multipath': {'polygon': [[0, 0], [2, 0], [2, 2], [0, 2]],
                                              'range_m': 3.0, 'bias_m': 0.3,
                                              'float_within_m': 1.0}}))
    assert m.event_mode(5.0) == FIXED     # never docked: no anchor
    m.notify(0.0, True, False)
    m.notify(1.0, False, False)
    assert m.event_mode(2.5) == FLOAT     # t0 1 s after leaving the dock (default float)
    mode, dx, dy = m.step(10.0, 0.1, 6.0, 1.0)     # 4 m east of the house: no bias
    assert (mode, dx, dy) == (FIXED, 0.0, 0.0)
    mode, dx, dy = m.step(10.1, 0.1, 3.5, 1.0)     # 1.5 m: biased east, still fixed
    assert mode == FIXED and dx == pytest.approx(0.15) and abs(dy) < 1e-9
    mode, _dx, _dy = m.step(10.2, 0.1, 2.5, 1.0)   # 0.5 m: float
    assert mode == FLOAT


def test_parse_spec():
    e = parse_spec('120:60:float')
    assert (e.t0, e.dur, e.kind) == (120.0, 60.0, FLOAT)
    assert parse_spec('5:2').kind == DROPOUT
    with pytest.raises(ValueError):
        parse_spec('5:2:bad')
    assert rtk_from_dict({}) is None


def test_rtk_events_anchor_on_the_first_bladed_in_place_pivot():
    """2026-11-04 scenario ii: RTK float during a coverage pivot (anchor 'pivot')."""
    m = RtkModel(rtk_from_dict({'anchor': 'pivot',
                                'events': [{'t0': 0.0, 'dur': 5, 'kind': 'float'}]}))
    m.notify(0.0, False, False, 0.0, 0.5)          # blade off: a transit pivot, no anchor
    m.notify(1.0, False, False, 0.0, 0.5)
    assert m.anchor_t is None
    m.notify(2.0, False, True, 0.2, 0.0)           # mowing straight
    m.notify(3.0, False, True, 0.0, 0.5)           # pivot starts
    m.notify(3.2, False, True, 0.0, 0.5)
    assert m.anchor_t is None and m.event_mode(3.2) == FIXED
    m.notify(3.5, False, True, 0.0, 0.5)           # held 0.5 s >= PIVOT_MIN_S
    assert m.anchor_t == 3.5 and m.event_mode(4.0) == FLOAT and m.event_mode(9.0) == FIXED
