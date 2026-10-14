"""Task 0.4 (d) fix (2026-10-15 sim on the real map): route corners are split robustly.

The run 4 split measured single 3-pose chords with "> route_corner_turn_rad" (pi/2), so the
dock path's drawn 90 deg corners were never split, and after a cut scanning resumed 6 poses
later, so a second corner within ~0.5 m was skipped. Real geometry: test/data/
real_dock_path_routes.json (the run 3 areas.dat Dock path channel and the two routes
plan_route made on it in the sim)."""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import geometry as geo   # noqa: E402
from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import Harness, _answer_route, _route_h, _route_resp  # noqa: E402

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data',
                    'real_dock_path_routes.json')
TH = 1.38                       # mission.yaml route_corner_turn_rad


def real():
    with open(DATA) as fh:
        d = json.load(fh)
    out = {}
    for k in ('route_from_dock', 'route_to_dock'):
        p = [tuple(q) for q in d[k]]
        out[k] = [(x, y, math.atan2(p[i + 1][1] - y, p[i + 1][0] - x) if i + 1 < len(p) else 0.0)
                  for i, (x, y) in enumerate(p)]
    out['channel'] = [tuple(q) for q in d['dock_path_channel']]
    return out


def resample(verts, step):
    """Polyline through ``verts`` resampled at <= ``step`` (as route_graph does)."""
    out = [(float(verts[0][0]), float(verts[0][1]))]
    for a, b in zip(verts, verts[1:]):
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                for k in range(1, n + 1)]
    return [(x, y, 0.0) for x, y in out]


def polyline(start, legs):
    """Vertices from ``start`` along [(heading_deg, length_m)]."""
    v = [start]
    for hd, ln in legs:
        x, y = v[-1]
        v.append((x + ln * math.cos(math.radians(hd)), y + ln * math.sin(math.radians(hd))))
    return v


def corners_xy(poses, th=TH):
    return [(poses[i][0], poses[i][1]) for i, _t in geo.route_corners(
        poses, th, tol_rad=f.ROUTE_CORNER_TOL_RAD)]


def near(p, q, tol=0.05):
    return math.hypot(p[0] - q[0], p[1] - q[1]) <= tol


# --- real dock path ------------------------------------------------------------------

def test_real_route_from_dock_splits_the_90_and_the_111_deg_corner_on_their_vertices():
    r = real()['route_from_dock']
    c = corners_xy(r)
    assert len(c) == 2
    assert near(c[0], (-0.140, -0.165))         # drawn 90 deg (dock path junction)
    assert near(c[1], (-3.248, 1.119))          # -111 deg into Area 1
    turns = [math.degrees(t) for _i, t in geo.route_corners(r, TH)]
    assert abs(abs(turns[0]) - 90.0) < 1.0 and abs(abs(turns[1]) - 111.0) < 1.5


def test_real_route_to_dock_splits_both_corners_on_their_vertices():
    r = real()['route_to_dock']
    c = corners_xy(r)
    assert len(c) == 2
    assert near(c[0], (-3.248, 1.119))          # +107 deg out of Area 1
    assert near(c[1], (-0.133, -0.175))         # 90 deg, 4 poses before the approach pose


def test_real_routes_through_the_fsm_end_each_leg_on_the_corner():
    r = real()['route_from_dock']
    h = _route_h()
    legs = h.fsm._split_route(r)
    assert len(legs) == 3
    assert near(legs[0][-1], (-0.140, -0.165)) and near(legs[1][-1], (-3.248, 1.119))
    assert legs[1][0][:2] == legs[0][-1][:2] and legs[2][0][:2] == legs[1][-1][:2]
    # the leg arrives along its last run (not already turned to the next one)
    assert abs(geo.wrap_angle(legs[0][-1][2] - math.radians(-146.4))) < 0.02
    assert abs(geo.wrap_angle(legs[1][-1][2] - math.radians(137.2))) < 0.02
    assert sum(len(lg) for lg in legs) == len(r) + 2


def test_legacy_split_missed_the_real_90_deg_corners():
    """Documents the bug: the run 4 rule (and the revert flag) leaves the 90 deg corner in."""
    r = real()['route_from_dock']
    legs = _route_h(route_corner_turn_rad=0.0).fsm._split_route_legacy(r)
    assert not any(near(lg[-1], (-0.140, -0.165)) for lg in legs)


def test_dock_path_channel_corners_do_not_depend_on_the_pose_spacing():
    ch = real()['channel']
    ref = None
    for step in (0.03, 0.07, 0.1, 0.17, 0.25):
        got = corners_xy(resample(ch, step), th=math.radians(35.0))
        if ref is None:
            ref = got
            assert len(ref) >= 2
        assert len(got) == len(ref) and all(near(a, b, 0.01) for a, b in zip(got, ref))


# --- synthetic -------------------------------------------------------------------------

def test_exactly_90_deg_is_split_at_any_spacing_and_on_the_vertex():
    for step in (0.03, 0.05, 0.096, 0.1, 0.2):
        p = resample(polyline((0.0, 0.0), [(0.0, 2.0), (90.0, 2.0)]), step)
        assert corners_xy(p) == [(2.0, 0.0)] or (
            len(corners_xy(p)) == 1 and near(corners_xy(p)[0], (2.0, 0.0), 1e-6))
        p = resample(polyline((0.0, 0.0), [(-146.4, 0.4), (123.6, 2.0)]), step)   # real one
        assert len(corners_xy(p)) == 1


def test_80_deg_is_split_at_the_default_threshold_and_not_at_the_run4_one():
    p = resample(polyline((0.0, 0.0), [(0.0, 2.0), (80.0, 2.0)]), 0.1)
    assert len(corners_xy(p, TH)) == 1
    assert corners_xy(p, 1.5708) == []
    p = resample(polyline((0.0, 0.0), [(0.0, 2.0), (75.0, 2.0)]), 0.1)
    assert corners_xy(p, TH) == []


def test_two_sharp_corners_0_3_m_apart_are_both_split():
    for second in (90.0, -90.0):                  # U (both left) and S (left then right)
        v = polyline((0.0, 0.0), [(0.0, 2.0), (90.0, 0.3), (90.0 + second, 2.0)])
        for step in (0.05, 0.1):
            c = corners_xy(resample(v, step))
            assert len(c) == 2, (second, step, c)
            assert near(c[0], v[1]) and near(c[1], v[2])


def test_the_real_cusp_then_90_deg_corner_4_poses_later_are_both_split():
    """Approach pose cusp (~150 deg) then the 90 deg junction 0.38 m on (sim route shape)."""
    v = polyline((0.6, 0.0), [(180.0, 0.4), (30.0, 0.38), (120.0, 2.0)])
    c = corners_xy(resample(v, 0.096))
    assert len(c) == 2 and near(c[0], v[1]) and near(c[1], v[2])


def test_straight_path_with_a_5_deg_wobble_is_not_split():
    for seg in (0.1, 0.3, 0.6):
        legs = [((5.0 if k % 2 else -5.0), seg) for k in range(int(6.0 / seg))]
        p = resample(polyline((0.0, 0.0), legs), 0.1)
        assert corners_xy(p) == [], seg
    # a slow same-side drift of 5 deg per 0.3 m is a gentle arc (r ~3.4 m), not a corner
    p = resample(polyline((0.0, 0.0), [(5.0 * k, 0.3) for k in range(18)]), 0.1)
    assert corners_xy(p) == []


def test_hand_drawn_corner_with_wobble_is_one_split_near_the_corner():
    """~90 deg drawn as two clicks (40 + 48 deg, 0.12 m apart) between wobbly arms."""
    arm1 = [((3.0 if k % 2 else -3.0), 0.25) for k in range(8)]
    arm2 = [((88.0 + (3.0 if k % 2 else -3.0)), 0.25) for k in range(8)]
    v = polyline((0.0, 0.0), arm1 + [(40.0, 0.12)] + arm2)
    c = corners_xy(resample(v, 0.1))
    assert len(c) == 1
    assert math.hypot(c[0][0] - v[8][0], c[0][1] - v[8][1]) <= 0.13


def test_corner_close_to_the_route_end_is_not_split():
    p = resample(polyline((0.0, 0.0), [(0.0, 2.0), (90.0, 0.1)]), 0.05)
    assert corners_xy(p) == []


# --- leg goal checker ------------------------------------------------------------------

def _corner(h):
    tgt = h.fsm.mission.transit_target
    a = [(0.1 * i, 0.0, 0.0) for i in range(21)]
    b = [(2.0, 0.1 * i, math.pi / 2) for i in range(1, 20)]
    _answer_route(h, resp=_route_resp(a + b + [tuple(tgt)]))


def test_corner_leg_uses_the_leg_goal_checker_and_the_last_leg_the_route_one():
    h = _route_h()
    _corner(h)
    g = h.goal(f.ACT_FOLLOW)
    assert g['goal_checker_id'] == 'coverage_leg_goal_checker'
    assert g['poses'][-1][:2] == (2.0, 0.0) and g['poses'][-1][2] == 0.0
    h.fsm.inputs.pose = (2.0, 0.0, 0.0)
    h.finish(f.ACT_FOLLOW)
    h.fsm.inputs.pose = (2.0, 0.0, math.pi / 2)
    h.finish(f.ACT_SPIN)
    assert h.goal(f.ACT_FOLLOW)['goal_checker_id'] == 'general_goal_checker'


def test_corner_goal_checker_revert_flag():
    h = _route_h(route_corner_goal_checker_id='')
    _corner(h)
    assert h.goal(f.ACT_FOLLOW)['goal_checker_id'] == 'general_goal_checker'


def test_dock_route_corner_leg_uses_the_leg_goal_checker():
    h = Harness(route_transits=True)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    assert h.cmd(f.CMD_HOME)
    a = [(0.1 * i, 0.0, 0.0) for i in range(21)]
    b = [(2.0, 0.1 * i, math.pi / 2) for i in range(1, 20)]
    _answer_route(h, resp=_route_resp(a + b))
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'dock_route'
    assert h.goal(f.ACT_FOLLOW)['goal_checker_id'] == 'coverage_leg_goal_checker'
