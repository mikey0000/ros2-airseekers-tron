# SPDX-License-Identifier: GPL-3.0-or-later
"""Rear camera logic: reverse corridor, dock awareness, rear_policy, RearRelevance (pure)."""
import math

import numpy as np
import pytest

from mower_vision.guard_logic import (
    Box, MotionState, Persistence, RearConfig, RearRelevance, dock_foot_xy, rear_corridor,
    rear_policy)

CAM_XYZ = (-0.201, 0.0, 0.25)
CAM_RPY = (-1.5707963, 0.0, 1.5707963)
W, H = 1280.0, 720.0


def inside(x, y, dock=None, cfg=None):
    return rear_corridor(x, y, cfg or RearConfig(), dock)


# --------------------------------------------------------------------------- corridor
@pytest.mark.parametrize('pt, expect', [
    ((-0.8, 0.0), True),
    ((-0.8, 0.6), False),     # lateral
    ((-1.6, 0.0), False),     # along 1.35 > 1.2
    ((0.0, 0.0), False),      # beside the robot
    ((-0.3, 0.4), True),
])
def test_corridor_without_dock(pt, expect):
    assert inside(*pt)[0] is expect


def test_corridor_reasons():
    assert 'side' in inside(-0.8, 0.6)[1]
    assert 'beside' in inside(0.0, 0.0)[1]
    assert 'far' in inside(-1.6, 0.0)[1]


DOCK = (-1.45, 0.0)


def test_owner_scenario_behind_dock():
    ok, why = inside(-1.75, 0.0, DOCK)
    assert not ok and 'dock' in why


def test_owner_scenario_beside_dock():
    assert not inside(-1.4, 0.8, DOCK)[0]


def test_owner_scenario_between_robot_and_dock():
    assert inside(-0.8, 0.05, DOCK)[0]


def test_owner_scenario_at_dock():
    assert not inside(-1.35, 0.0, DOCK)[0]


def test_dock_off_axis_corridor_points_at_dock():
    dock = (-1.4, 0.3)
    assert inside(-1.0, 0.2, dock)[0]
    assert not inside(-1.0, -0.4, dock)[0]


def test_dock_closer_than_margin_nothing_inside():
    for pt in ((-0.3, 0.0), (-0.25, 0.0), (-0.35, 0.1)):
        assert not inside(*pt, (-0.4, 0.0))[0]


# --------------------------------------------------------------------------- dock_foot_xy
def _rot(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def test_dock_foot_unknown_height_returns_marker_xy():
    assert dock_foot_xy((-1.4, 0.2, 0.15), CAM_XYZ, CAM_RPY, -1.0) == (-1.4, 0.2)


@pytest.mark.parametrize('xy', [(-1.4, 0.0), (-1.2, 0.3), (-1.8, -0.2)])
def test_dock_foot_nominal_geometry_identity(xy):
    foot = dock_foot_xy((xy[0], xy[1], 0.15), CAM_XYZ, CAM_RPY, 0.15)
    assert foot[0] == pytest.approx(xy[0], abs=1e-3)
    assert foot[1] == pytest.approx(xy[1], abs=1e-3)


def _nominal_ground(ray_c, r_nom, t):
    """det_range: pixel ray (camera frame) -> ground point through the NOMINAL pose."""
    d = r_nom @ ray_c
    s = -t[2] / d[2]
    return t + s * d


def _scenario(delta_deg):
    """What the system sees with a camera rotated delta_deg about base_link y (true pose)."""
    t = np.array(CAM_XYZ)
    r_nom = _rot(*CAM_RPY)
    ry = _rot(0.0, math.radians(delta_deg), 0.0)
    r_true = ry @ r_nom

    def to_cam(p):
        return r_true.T @ (np.array(p) - t)

    marker_nom = t + r_nom @ to_cam((-1.4, 0.0, 0.15))    # PnP -> nominal base_link
    est = lambda p: _nominal_ground(to_cam(p), r_nom, t)[:2]   # noqa: E731
    return marker_nom, est


@pytest.mark.parametrize('delta', [1.0, -1.0])
def test_pitch_bias_dock_foot_tracks_detections(delta):
    marker_nom, est = _scenario(delta)
    behind, front = est((-1.7, 0.0, 0.0)), est((-1.0, 0.0, 0.0))
    dock = dock_foot_xy(marker_nom, CAM_XYZ, CAM_RPY, 0.15)
    ok, why = rear_corridor(behind[0], behind[1], RearConfig(), dock)
    assert not ok, (behind, dock, why)
    assert rear_corridor(front[0], front[1], RearConfig(), dock)[0], (front, dock)


@pytest.mark.parametrize('delta', [1.0, -1.0])
def test_pitch_bias_known_height_beats_unknown(delta):
    marker_nom, est = _scenario(delta)
    at_dock = est((-1.4, 0.0, 0.0))        # person standing on the true dock foot
    known = dock_foot_xy(marker_nom, CAM_XYZ, CAM_RPY, 0.15)
    unknown = dock_foot_xy(marker_nom, CAM_XYZ, CAM_RPY, -1.0)
    err_known = abs(at_dock[0] - known[0])
    err_unknown = abs(at_dock[0] - unknown[0])
    assert err_known < err_unknown
    assert err_known < 0.01


# --------------------------------------------------------------------------- rear_policy
def box(label='person', x=-0.8, y=0.0, score=0.9, rng='auto', cx=640.0, h=400.0, cy=500.0):
    ranged = x is not None
    r = (math.hypot(x, y) if rng == 'auto' else rng) if ranged else None
    return Box(label, score, cx, cy, 100.0, h, range_m=r, x_m=x, y_m=y if ranged else None)


def pol(boxes, cfg=None, **kw):
    return rear_policy(boxes, cfg or RearConfig(), W, H, **kw)


@pytest.mark.parametrize('label', ['person', 'dog', 'cat', 'hedgehog', 'rabbit'])
def test_living_classes_dynamic_rear(label):
    p = pol([box(label)])
    assert p['kind'] == 'dynamic' and p['camera'] == 'rear' and p['class'] == label


def test_stone_ignored_by_default_static_when_opted_in():
    assert pol([box('stone')])['kind'] == 'none'
    p = pol([box('stone')], RearConfig(static_classes=['stone']))
    assert p['kind'] == 'static' and p['camera'] == 'rear'


def test_low_score_ignored():
    assert pol([box(score=0.3)])['kind'] == 'none'


def test_ranged_outside_corridor_ignored_with_reason():
    ign = []
    assert pol([box(x=-0.8, y=0.9)], ignored=ign)['kind'] == 'none'
    assert len(ign) == 1 and ign[0][1]


def test_unranged_central_tall_counts_without_dock_only():
    b = box(x=None)
    assert pol([b])['kind'] == 'dynamic'
    ign = []
    assert pol([b], dock=DOCK, ignored=ign)['kind'] == 'none' and ign


def test_unranged_offcentre_or_short_does_not_count():
    assert pol([box(x=None, cx=100.0)])['kind'] == 'none'
    assert pol([box(x=None, h=100.0)])['kind'] == 'none'


def test_persistence_two_frames_and_reset():
    per, cfg = Persistence(), RearConfig(persist_frames=2)
    assert pol([box()], cfg, persistence=per)['kind'] == 'none'
    assert pol([box()], cfg, persistence=per)['kind'] == 'dynamic'
    assert pol([], cfg, persistence=per)['kind'] == 'none'
    assert pol([box()], cfg, persistence=per)['kind'] == 'none'
    assert pol([box()], cfg, persistence=per)['kind'] == 'dynamic'


def test_disabled():
    assert pol([box()], RearConfig(enabled=False))['kind'] == 'none'


def test_dynamic_beats_static_and_closest_wins():
    cfg = RearConfig(static_classes=['stone'])
    assert pol([box('stone', x=-0.4), box('person', x=-1.0)], cfg)['kind'] == 'dynamic'
    p = pol([box('person', x=-1.0), box('dog', x=-0.5)])
    assert p['class'] == 'dog'


# --------------------------------------------------------------------------- relevance
def moving(linear_x, now=0.0):
    m = MotionState()
    m.update(linear_x, 0.0, now)
    return m


def test_motion_state_accessors():
    m = moving(-0.2, 10.0)
    assert m.reversing(10.3) and not m.forward(10.3)
    assert not m.reversing(11.0)                      # hold_s 0.6 elapsed
    m = moving(0.2, 10.0)
    assert m.forward(10.3) and not m.reversing(10.3)
    m = moving(0.01, 10.0)                            # inside the deadband
    assert not m.forward(10.0) and not m.reversing(10.0)


def test_relevance_active_when_reversing():
    assert RearRelevance().active(True, False, 0.0)
    assert not RearRelevance().active(False, False, 0.0)


def test_relevance_docking_fresh_vs_stale():
    r = RearRelevance(dock_state_timeout_s=1.0)
    r.update_dock_state('DOCKING', 10.0)
    assert r.active(False, False, 10.5)
    assert not r.active(False, False, 11.5)


def test_relevance_aligning_watch_only():
    r = RearRelevance()
    r.update_dock_state('ALIGNING', 10.0)
    assert not r.active(False, False, 10.1)
    assert r.watch(False, False, 10.1)
    assert not r.watch(False, False, 12.0)            # stale


def test_relevance_sticky_hit():
    r = RearRelevance(sticky_s=20.0)
    r.note_hit(100.0)
    assert r.active(False, False, 110.0)
    assert not r.active(False, False, 121.0)
    assert not r.active(False, True, 110.0)           # driving forward
    r.clear_hits()
    assert not r.active(False, False, 101.0)


def test_relevance_watch_hold_after_reverse():
    r = RearRelevance(watch_hold_s=5.0)
    assert not r.watch(False, False, 100.0)
    r.note_reverse(100.0)
    assert r.watch(False, False, 104.0)
    assert not r.watch(False, False, 106.0)
