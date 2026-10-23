"""Step 3 round 2 (2026-10-21): body-aware boundary-out check with the stock recovery
(reverse 0.3 m straight, resume), the outside_m jump filter, Spin drift, repeated bumper
hits."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import body_boundary as bb   # noqa: E402
from mower_mission import mission_fsm as f      # noqa: E402
from test_mission_fsm import Harness, line, plan, square, start_until_planning  # noqa: E402

SQ = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]
SOFT = bb.limits_for('soft')
HARD = bb.limits_for('hard')


# ---------------------------------------------------------------- pure geometry

def test_soft_lap0_on_the_line_does_not_trip():
    ev = bb.evaluate((2.0, 0.0, 0.0), [SQ], SOFT)        # base_link on the line, heading along
    assert abs(ev['out'] - 0.235) < 1e-9 and ev['excess'] < -0.1


def test_soft_lap0_drift_trips_on_a_side_corner():
    ev = bb.evaluate((2.0, -0.13, 0.0), [SQ], SOFT)      # 0.13 drift: corners 0.365 out
    assert ev['excess'] > 0.0 and ev['point'] == 'rear_right' and ev['side_only']


def test_soft_row_end_nose_and_pivot_sweep_do_not_trip():
    # rows end 0.45 inside: nose 0.15 past; a pivot there sweeps the front corners <= 0.195
    for deg in range(0, 361, 5):
        ev = bb.evaluate((4.55, 2.0, math.radians(deg)), [SQ], SOFT)
        assert ev['excess'] < -0.1, deg


def test_hard_trips_beyond_0_10_and_rear_points_recover_forward():
    assert bb.evaluate((2.0, 0.34, 0.0), [SQ], HARD)['excess'] < 0.0     # corners 0.105 in
    ev = bb.evaluate((0.02, 2.0, math.radians(67)), [SQ], HARD)          # rear-left 0.235 out
    assert ev['excess'] > 0.0 and ev['point'] == 'rear_left' and ev['rear']


def test_union_of_area_and_path_counts_as_inside():
    path = [(5.0, 1.8), (9.0, 1.8), (9.0, 2.2), (5.0, 2.2)]
    ev = bb.evaluate((4.8, 2.0, 0.0), [SQ, path], SOFT)                 # nose on the path
    assert ev['outs']['nose'] == 0.0


# ---------------------------------------------------------------- mission integration

def feed(h, pose, out=0.0):
    i = h.fsm.inputs
    i.pose = tuple(pose)
    i.outside_m = out
    i.outside_stamp = h.t


def mowing_h(subpaths, pose, **params):
    params.setdefault('hard_boundary_body', True)
    h = Harness(**params)
    h.areas = [square(0, 0, 5)]
    feed(h, pose)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan(subpaths))
    h.fsm.inputs.is_cutting = True
    feed(h, pose)
    h.tick()
    return h


def integrate(h, max_ticks=200):
    """Integrate /cmd_vel_emergency DriveCmds while the boundary recovery runs."""
    cmds = []
    for _ in range(max_ticks):
        if h.fsm._hb is None:
            break
        m = h.mark()
        h.tick()
        dc = h.since(m, f.DriveCmd)
        x, y, yaw = h.fsm.inputs.pose
        if dc:
            v, w = dc[-1].linear, dc[-1].angular
            cmds.append((v, w))
            yaw += w * 0.1
            x += v * 0.1 * math.cos(yaw)
            y += v * 0.1 * math.sin(yaw)
        feed(h, (x, y, yaw))
    return cmds


def test_nose_trip_reverses_0_3_straight_slowly_then_resumes():
    h = mowing_h([line(1, 2, 4.9, 2, n=41), line(1, 3, 4, 3)], (1.0, 2.0, 0.0))
    assert h.name == 'MOWING' and h.pending_action(f.ACT_FOLLOW) and h.blade
    m = h.mark()
    x = 1.0
    while h.fsm._hb is None and x < 4.9:
        x += 0.05
        feed(h, (x, 2.0, 0.0))
        h.tick()
    assert h.fsm._hb is not None and h.fsm._hb['mode'] == 'soft' and not h.blade
    assert x > 4.75                  # nose 0.40 past at x 4.80: no trip before the limit
    x0 = x
    cmds = integrate(h)
    moving = [c for c in cmds if c[0] or c[1]]
    assert moving and all(v < 0.0 and abs(v) <= 0.10 + 1e-9 and w == 0.0 for v, w in moving)
    assert abs((x0 - h.fsm.inputs.pose[0]) - 0.30) < 0.06          # 0.3 m straight back
    assert h.fsm._hb_count == 1
    h.tick(n=3)
    assert h.name == 'MOWING' and h.fsm.mission.sub_i == 0         # resumed the same lap
    assert [e.kind for e in h.since(m, f.RecordIncident)][:1] == ['boundary']


def test_side_drift_not_fixed_by_the_straight_move_falls_back_to_the_retrace():
    h = mowing_h([line(1, 0, 4, 0), line(1, 3, 4, 3)], (1.0, 0.0, 0.0))
    for k in range(1, 30):           # drift out sideways while driving east
        feed(h, (1.0 + 0.05 * k, -0.01 * k, 0.0))     # lateral slide
        h.tick()
        if h.fsm._hb is not None:
            break
    assert h.fsm._hb is not None
    for _ in range(60):
        if h.fsm._hb is None or h.fsm._hb['stage'] == 'drive':
            break
        integrate(h, max_ticks=1)
    assert h.fsm._hb is not None and h.fsm._hb['stage'] == 'drive'   # retrace fallback


def test_hard_rear_trip_moves_forward():
    h = mowing_h([line(1, 2, 4, 2)], (1.0, 2.0, 0.0))
    h.fsm.mission.settings['edge_mode'] = 'hard'
    feed(h, (0.02, 2.0, math.radians(67)))
    h.tick()
    k = h.fsm._hb
    assert k is not None and k['mode'] == 'hard' and k['dir'] > 0
    cmds = integrate(h, max_ticks=10)
    assert any(v > 0.0 for v, _ in cmds)


def test_outside_jump_is_ignored_and_not_counted():
    h = mowing_h([line(1, 2, 4, 2)], (1.0, 2.0, 0.0), hard_boundary_body=False)
    feed(h, (1.2, 2.0, 0.0), out=0.0)
    h.tick()
    feed(h, (1.25, 2.0, 0.0), out=23.37)      # GUI map reload mid-way (field 2026-10-21)
    h.tick(n=5)
    assert h.fsm._hb is None and h.fsm._hb_count == 0
    feed(h, (1.3, 2.0, 0.0), out=0.0)
    h.tick()
    assert h.fsm._hb is None and h.name == 'MOWING'


def test_spin_drift_inside_goes_on_beyond_the_hard_limit_pivots_again_no_skip():
    # field 2026-10-23 afternoon: 47 % of wet-turf pivots walked ~0.14 m along the axis, well
    # inside the area; stopping them caused transit aborts and skips. Now: inside -> the pivot
    # goes on; beyond spin_drift_hard_m (or near the body limit) -> stop and pivot again from
    # where the robot is (no transit, no skip)
    h = mowing_h([line(1, 2, 4, 2), line(1, 3, 4, 3)], (1.0, 2.0, math.pi),
                 path_conditioning=True)
    a = h.pending_action(f.ACT_SPIN)
    assert a.purpose == 'cov_pivot'
    m = h.mark()
    feed(h, (1.1, 1.85, 2.0))                 # 0.18 m, body well inside the 5 m square
    h.tick()
    assert not h.since(m, f.CancelActions) and h.fsm._action.name == f.ACT_SPIN
    m = h.mark()
    feed(h, (1.2, 1.70, 1.5))                 # 0.36 m > spin_drift_hard_m
    h.tick()
    assert h.since(m, f.CancelActions)
    assert h.name == 'MOWING' and h.pending_action(f.ACT_SPIN)      # pivoting again
    assert 'spin_drift' in [e.kind for e in h.since(m, f.RecordIncident)]
    assert h.fsm.mission.sub_i == 0 and not h.fsm.mission.blocked


def test_spin_drift_near_the_body_limit_stops_the_pivot():
    h = mowing_h([line(4.6, 2, 4.6, 4.5), line(1, 3, 4, 3)], (4.6, 2.0, math.pi),
                 path_conditioning=True)
    h.pending_action(f.ACT_SPIN)
    m = h.mark()
    feed(h, (4.76, 2.02, 0.0))                # 0.16 m toward the east edge: nose 0.36 (limit 0.40)
    h.tick()
    assert h.since(m, f.CancelActions)



def test_three_bumper_hits_within_a_metre_skip_the_stretch():
    h = mowing_h([line(0.5, 2, 4.5, 2), line(0.5, 3, 4.5, 3)], (0.5, 2.0, 0.0))
    assert h.pending_action(f.ACT_FOLLOW)
    feed(h, (4.0, 2.0, 0.0))
    for k in range(2):
        h.fsm.inputs.bumper_hits += 1
        feed(h, (4.0 + 0.1 * k, 2.0, 0.0))
        h.tick()
        assert h.fsm.mission.sub_i == 0 and h.fsm._action is not None
    m = h.mark()
    h.fsm.inputs.bumper_hits += 1
    h.tick()
    assert 'bumper_skip' in [e.kind for e in h.since(m, f.RecordIncident)]
    assert h.fsm.mission.sub_i == 1              # the rest of the row skipped


def test_side_trip_reverses_onto_the_planned_leg_toward_the_interior():
    h = mowing_h([line(0.5, 0, 4.5, 0, n=41), line(1, 3, 4, 3)], (0.5, 0.0, 0.0))
    x, y = 0.5, 0.0
    while h.fsm._hb is None and x < 3.0:      # lap 0 on the line, sliding out sideways
        x, y = x + 0.05, y - 0.01
        feed(h, (x, y, 0.0))
        h.tick()
    k = h.fsm._hb
    assert k is not None and k['side'] and k['mode'] == 'soft'
    y0 = h.fsm.inputs.pose[1]
    cmds = integrate(h, max_ticks=150)
    assert h.fsm._hb is None and h.name == 'MOWING'
    assert all(v <= 0.0 for v, _ in cmds)                     # reverse only
    assert h.fsm.inputs.pose[1] > y0 + 0.02                   # moved toward the interior


def test_soft_tight_convex_bend_on_lap0_does_not_trip_but_drift_does():
    # owner_far lap 0 follows drawn convex bulges of radius ~1.1 m: base_link on the line,
    # front corners sqrt((R + 0.235)^2 + 0.6^2) - R = 0.36 out by geometry alone
    R = 1.1
    circle = [(R * math.cos(2 * math.pi * k / 180), R * math.sin(2 * math.pi * k / 180))
              for k in range(180)]
    on = bb.evaluate((R, 0.0, math.pi / 2), [circle], SOFT)       # tangent, CCW
    assert 0.35 < on['outs']['front_right'] < 0.40 and on['excess'] < 0.0
    drift = bb.evaluate((R + 0.13, 0.0, math.pi / 2), [circle], SOFT)
    assert drift['excess'] > 0.0                                   # lateral drift still trips


def test_soft_driven_arc_r044_stays_on_the_line_without_a_trip():
    # owner option (c): Area 6's driven arcs (R 0.44) put the front corners 0.46 out by
    # geometry alone; lap 0 stays on the line, the soft front-corner trip is 0.55
    R = 0.44
    circle = [(R * math.cos(2 * math.pi * k / 360), R * math.sin(2 * math.pi * k / 360))
              for k in range(360)]
    ev = bb.evaluate((R, 0.0, math.pi / 2), [circle], SOFT)
    assert 0.44 < ev['outs']['front_right'] < 0.48 and ev['excess'] < -0.05


def test_leg_start_pivot_turns_toward_the_interior_even_the_long_way():
    # sim 2026-10-22: transit ended facing the line (NE) near a lap-0 start heading S; the
    # shorter -135 deg pivot swings the nose out over the east edge, +225 deg stays inside
    h = mowing_h([line(5, 4, 5, 1, n=31), line(1, 3, 4, 3)], (4.8, 4.0, math.radians(45)),
                 path_conditioning=True)
    a = h.pending_action(f.ACT_SPIN)
    delta = h.goal(f.ACT_SPIN)['angle']
    assert delta > math.pi                       # CCW the long way, through the interior


# ---- field 2026-10-22 morning: retrace cap, START while outside, Spin re-send back-off ----

def test_retrace_ending_still_outside_moves_inward_instead_of_needs_help():
    # the robot rode the line ~0.07 m out for 2 m, then tripped at 0.11: the 1.0 m retrace
    # along that track ends still 0.07 out -> bounded inward moves, then resume
    h = mowing_h([line(0.5, 0, 4.5, 0, n=41), line(1, 3, 4, 3)], (0.5, 0.0, 0.0),
                 hard_boundary_body=False, hard_boundary_snapshot=True)
    x = 0.5
    while x < 3.0:
        x += 0.05
        feed(h, (x, -0.07, 0.0))
        h.tick()
    assert h.fsm._hb is None
    feed(h, (x + 0.05, -0.11, 0.0))
    h.tick()
    assert h.fsm._hb is not None and h.fsm._hb_count == 1
    integrate(h, max_ticks=400)
    assert h.fsm._hb is None and h.name == 'MOWING'
    assert bb.point_outside(h.fsm.inputs.pose[0], h.fsm.inputs.pose[1], [SQ]) == 0.0
    assert h.fsm._hb_count == 1


def test_start_while_outside_moves_inward_first_uncounted_no_pivot():
    h = mowing_h([line(2, 0, 4.5, 0, n=26), line(1, 3, 4, 3)], (2.0, -0.07, 0.4),
                 hard_boundary_body=False, hard_boundary_snapshot=True, path_conditioning=True)
    k = h.fsm._hb
    assert k is not None and k['entry'] and not k['counted'] and h.fsm._hb_count == 0
    assert h.fsm._action is None or h.fsm._action.name != f.ACT_SPIN
    cmds = integrate(h, max_ticks=200)
    assert all(not (v == 0.0 and w != 0.0) for v, w in cmds)      # no pivot while outside
    assert h.fsm._hb is None and h.fsm._hb_count == 0
    assert h.name == 'MOWING'


def test_refused_pivot_is_resent_after_a_real_pause():
    h = mowing_h([line(1, 2, 4, 2), line(1, 3, 4, 3)], (1.0, 2.0, math.pi),
                 path_conditioning=True, blocked_pivot_policy='retry', pivot_retry_pause_s=2.0)
    h.pending_action(f.ACT_SPIN)
    h.finish(f.ACT_SPIN, f.ABORTED)
    assert h.fsm._action is None                 # not re-sent in the same second
    h.tick(n=10)
    assert h.fsm._action is None
    h.tick(n=11)
    assert h.pending_action(f.ACT_SPIN)


def test_transit_to_an_edge_lap_start_ends_inside_beside_it():
    # sim 2026-10-22: transits arriving square to the line at a lap-0 start put the nose
    # 0.40 m out; the transit now ends 0.30 m inside, then a shallow join (look-ahead 3 x
    # the lateral offset)
    h = mowing_h([line(1, 3, 4, 3), line(1, 0, 4.5, 0, n=36)], (1.0, 3.0, 0.0),
                 path_conditioning=True)
    h.fsm.mission.sub_i = 1
    h.fsm.mission.start_local = 0
    h.fsm._dispatch(force_transit=True)
    tx, ty, tyaw = h.fsm.mission.transit_target
    assert abs(tx - 1.0) < 1e-6 and abs(ty - 0.30) < 1e-6 and abs(tyaw) < 1e-6
    hd = f.MissionFSM._join_heading(line(1, 0, 4.5, 0, n=36), (tx, ty, 0.0))
    assert abs(math.degrees(hd)) < 20.0


def test_soft_trip_is_plan_relative_near_a_corner_planned_out():
    # sim 2026-10-22 run5: lap 0's first bend is planned with the body 0.41 m out; the nose
    # tripped at 0.40 there on every lap start. Near such a corner the soft limit is the
    # planned body + 0.10; elsewhere the static limits stay
    h = mowing_h([line(1, 2, 4, 2), line(1, 3, 4, 3)], (1.0, 2.0, 0.0))
    h.fsm.mission.corner_body = [(4.8, 2.0, 0.41)]
    near = h.fsm._hb_body_eval((4.85, 2.0, 0.0), 'soft')     # nose 0.45 out
    assert near['excess'] < 0.0
    h.fsm.mission.corner_body = [(1.0, 4.0, 0.41)]
    far = h.fsm._hb_body_eval((4.85, 2.0, 0.0), 'soft')
    assert far['excess'] > 0.0


def test_edge_start_join_merges_tangentially_onto_an_arc():
    # round 4: owner_far's lap 0 starts on a drawn arc (no straight stretch): from the
    # approach point 0.30 m inside, the leg's first 1.5 m fade from the inset onto the lap
    R = 2.0
    arc = [(2.5 + R * math.cos(-math.pi / 2 + a / 40.0), 2.5 + R * math.sin(-math.pi / 2 + a / 40.0),
            a / 40.0) for a in range(60)]
    h = mowing_h([line(1, 3, 4, 3), arc], (1.0, 3.0, 0.0), path_conditioning=True,
                 edge_approach_inset_m=0.30)
    m = h.fsm.mission
    h.fsm._region_areas = {0: {'outer': [(2.5 + R * math.cos(t / 80.0), 2.5 + R * math.sin(t / 80.0))
                                         for t in range(503)], 'is_navigation_area': False}}
    m.sub_i, m.start_local = 1, 0
    h.fsm._begin(h.t)
    h.fsm._dispatch(force_transit=True)
    h._apply(h.fsm._end())
    assert m.join is not None
    tx, ty, _ = m.transit_target
    legs = h.fsm._join_leg([tuple(q) for q in arc], 1, 0)
    assert math.hypot(legs[0][0] - tx, legs[0][1] - ty) < 0.02      # starts at the approach point
    cum = 0.0
    for a, b, q in zip(legs, legs[1:], arc[1:]):
        cum += math.hypot(b[0] - a[0], b[1] - a[1])
        if cum > 1.6:
            assert math.hypot(b[0] - q[0], b[1] - q[1]) < 1e-6         # on the lap after 1.5 m
    turns = [abs(math.remainder(legs[i + 1][2] - legs[i][2], 2 * math.pi)) for i in range(len(legs) - 1)]
    assert max(turns) < math.radians(8)                                # smooth, no kink


# ---- owner 2026-10-23: blade stays on for obstacle avoidance while inside the area ----

def _fx_call(h, fn):
    h.fsm._begin(h.t)
    fn()
    h._apply(h.fsm._end())


def test_bumper_skip_inside_the_area_keeps_the_blade_on():
    h = mowing_h([line(0.5, 2, 4.5, 2), line(0.5, 3, 4.5, 3)], (0.5, 2.0, 0.0))
    assert h.blade and h.pending_action(f.ACT_FOLLOW)
    feed(h, (2.0, 2.0, 0.0))
    for _ in range(3):
        h.fsm.inputs.bumper_hits += 1
        h.tick()
    assert h.fsm.mission.sub_i == 1 or h.fsm.mission.step in ('transit', 'follow', 'spinup')
    assert h.blade                                   # skipped, blade kept on
    h2 = mowing_h([line(0.5, 2, 4.5, 2), line(0.5, 3, 4.5, 3)], (0.5, 2.0, 0.0),
                  blade_keep_on_obstacles=False)
    feed(h2, (2.0, 2.0, 0.0))
    for _ in range(3):
        h2.fsm.inputs.bumper_hits += 1
        h2.tick()
    assert not h2.blade                              # revert flag: the old blade-off


def test_detour_inside_keeps_the_blade_on_and_one_leaving_turns_it_off():
    h = mowing_h([line(0.5, 2, 4.5, 2), line(0.5, 3, 4.5, 3)], (0.5, 2.0, 0.0))
    feed(h, (2.0, 2.0, 0.0))
    h.tick()
    assert h.blade
    _fx_call(h, lambda: h.fsm._transit_blade((3.5, 2.5), 'detour', force_off=True))
    assert h.blade
    _fx_call(h, lambda: h.fsm._transit_blade((5.8, 2.0), 'detour', force_off=True))
    assert not h.blade                               # the path leaves the area


def test_boundary_violation_lift_and_tilt_still_turn_the_blade_off():
    for setup in ('boundary', 'lift', 'tilt'):
        h = mowing_h([line(0.5, 2, 4.5, 2), line(0.5, 3, 4.5, 3)], (0.5, 2.0, 0.0))
        feed(h, (2.0, 2.0, 0.0))
        h.tick()
        assert h.blade
        if setup == 'boundary':
            h.fsm.inputs.boundary_violation = True
            _fx_call(h, lambda: h.fsm._transit_blade((3.5, 2.5), 'detour', force_off=True))
        elif setup == 'lift':
            h.fsm.inputs.lift = True
            h.tick()
        else:
            i = h.fsm.inputs
            i.tilt_band, i.tilt_roll_deg, i.tilt_pitch_deg, i.tilt_axis = 'critical', 31.0, 0.0, 'roll'
            i.tilt_stamp = h.t
            h.tick()
        assert not h.blade, setup


# ---- field 2026-10-23 afternoon: stretch retries, off-network transit goals ----

from test_mission_fsm import _answer_route, _route_h, _route_resp   # noqa: E402


def test_offgraph_goal_is_re_asked_inside_then_skipped_never_the_costmap_planner():
    h = _route_h(edge_approach_inset_m=0.0)
    goal0 = tuple(h.fsm.mission.transit_target)
    fail = _route_resp(ok=False, message='goal (5.00, 5.00) is off the route graph')
    _answer_route(h, resp=fail)
    req = _answer_route(h, resp=fail)                 # 1st re-ask (0.3 m inside) failed too
    assert math.hypot(req['goal'][0] - goal0[0], req['goal'][1] - goal0[1]) > 0.29
    m = h.mark()
    _answer_route(h, resp=fail)                       # 0.6 m inside failed: skip it
    assert all(e.name != f.ACT_NAV for e in h.since(m, f.StartAction))
    assert any('skipping it' in e.text for e in h.since(m, f.Log))


def test_short_and_unbypassable_stretches_are_not_re_tried():
    h = mowing_h([line(0.5, 2, 4.5, 2, n=41), line(0.5, 3, 4.5, 3)], (0.5, 2.0, 0.0))
    m = h.fsm.mission
    m.unbypassable = [(2.0, 2.0)]
    m.requeue = [(5, 8), (20, 25), (30, 38)]          # 0.3 m; next to (2, 2); 0.8 m clear
    h.fsm._begin(h.t)
    h.fsm._area_done()
    h._apply(h.fsm._end())
    assert (0, 5, 8) in m.blocked and (0, 20, 25) in m.blocked
    assert m.stretch == (30, 38)                      # the clear one is driven
