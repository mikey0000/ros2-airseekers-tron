"""Fix 10 (turn-leg completion cap, leg loop guard) and fix 11 (route cusp about-turn, bounded
costmap fallback) from the 2026-10-14 run 2 field test (docs/wheel_smoothness_field_tests.md)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import geometry as geo   # noqa: E402
from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import (Harness, _answer_route, _pending_route_request,  # noqa: E402
                              _route_h, _route_resp, mowing, plan, square,
                              start_until_planning)


def logs(h, mark=0):
    return [e.text for e in h.since(mark, f.Log)]


def starts(h, mark, name):
    return [e for e in h.since(mark, f.StartAction) if e.name == name]


# =====================================================================
# fix 10: turn-leg completion cap
# =====================================================================
RYAW = math.atan2(-0.35, 0.1)          # reverse leg poses face against its travel


def _k_turn_on_rev(h, fwd2=None):
    """swath ending at a cusp -> reverse leg -> forward leg (+ a last swath). Returns with
    the reverse leg's follow_path running and the robot at its start."""
    start_until_planning(h)
    fwd = [(0.2, 0.0, 0.0)] + [(1.0 + 0.1 * k, 0.0, 0.0) for k in range(11)]   # ends (2.0, 0)
    rev = [(2.0, 0.0, RYAW), (1.95, 0.175, RYAW), (1.9, 0.35, RYAW)]
    fwd2 = fwd2 or [(1.9, 0.35, math.pi), (0.0, 0.36, math.pi)]
    last = [(0.0, 1.0, 0.0), (4.0, 1.0, 0.0)]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([fwd, rev, fwd2, last]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.goal(f.ACT_FOLLOW)['poses'][-1] == fwd[-1]
    h.fsm.inputs.pose = (2.0, 0.0, 0.0)          # at the cusp: the leg goal checker ends it
    h.finish(f.ACT_FOLLOW, arrive=False)
    assert h.goal(f.ACT_FOLLOW)['poses'] == rev
    return fwd, rev, fwd2


def test_turn_leg_end_reached_with_yaw_off_completes_after_the_time_cap():
    h = Harness()
    _fwd, rev, fwd2 = _k_turn_on_rev(h)
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.fsm.inputs.pose = (1.9, 0.35, RYAW + 1.0)  # at the end in xy, 1.0 rad off: no skip
    m = h.mark()
    h.tick(dt=0.1, n=199)                        # 19.9 s, yaw constant (no rotation)
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    assert not h.since(m, f.CancelActions)
    h.tick(dt=0.1, n=2)
    g = h.goal(f.ACT_FOLLOW)
    assert g['poses'] == fwd2 and h.name == 'MOWING'
    assert 1 in h.fsm.cursor.areas[0].completed
    done = [t for t in logs(h, m) if 'leg done' in t]
    assert len(done) == 1 and 'time cap' in done[0] and 'sub-path 1' in done[0]
    assert h.since(m, f.CancelActions)


def test_turn_leg_end_reached_completes_after_one_full_rotation():
    """The run 2 loop: pose index at the leg's last pose, the robot looping 0.4 m away
    (outside turn_leg_skip_m, so the yaw-pass skip can never fire) and turning. That
    progress_local rule is the fix 10 one, kept behind turn_leg_end_by_distance: false
    (task 0.4 f; see test_turn_leg_end_by_distance_rotation_cap for the default)."""
    h = Harness(turn_leg_end_by_distance=False)
    _fwd, rev, fwd2 = _k_turn_on_rev(h)
    h.fsm.inputs.pose = (1.9, 0.35, RYAW + 1.0)
    h.tick()                                     # end reached (pose index at the last pose)
    m = h.mark()
    yaw = RYAW + 1.0
    for k in range(70):                          # 0.1 rad per 0.1 s tick, 0.4 m off the end
        yaw += 0.1
        h.fsm.inputs.pose = (1.9, 0.75, yaw)
        h.tick()
        if h.since(m, f.CancelActions):
            break
    assert 60 <= k < 66                          # ~6.28 rad, after ~6.3 s (< the 20 s cap)
    assert h.goal(f.ACT_FOLLOW)['poses'] == fwd2
    done = [t for t in logs(h, m) if 'leg done' in t]
    assert len(done) == 1 and 'rotation cap' in done[0]


def test_turn_leg_end_with_yaw_met_is_unchanged():
    """Yaw within tolerance at the end: the leg goal checker ends it (0.2 s later) and the
    caps stay out of it; an abort there is skipped on the re-send as before (turn_leg_skip)."""
    h = Harness()
    _fwd, rev, fwd2 = _k_turn_on_rev(h)
    h.fsm.inputs.pose = (1.92, 0.33, RYAW + 0.2)
    m = h.mark()
    h.tick(dt=0.1, n=2)                          # 0.2 s at the end
    assert h.goal(f.ACT_FOLLOW)['poses'] == rev and not h.since(m, f.CancelActions)
    h.finish(f.ACT_FOLLOW, arrive=False)
    assert h.goal(f.ACT_FOLLOW)['poses'] == fwd2
    assert not [t for t in logs(h, m) if 'leg done' in t]

    h2 = Harness(transit_retry_delay_s=0.1)
    _k_turn_on_rev(h2)
    h2.fsm.inputs.pose = (1.95, 0.2, RYAW + 0.2)    # 0.16 m short of the end (pose 1)
    h2.tick(dt=0.1, n=2)
    m = h2.mark()
    h2.finish(f.ACT_FOLLOW, f.ABORTED)
    h2.tick(dt=0.1, n=2)                         # retry after 0.1 s: turn_leg_skip, next leg
    assert h2.goal(f.ACT_FOLLOW)['poses'][0] == (1.9, 0.35, math.pi)
    done = [t for t in logs(h2, m) if 'leg done' in t]
    assert len(done) == 1 and 'no re-send' in done[0] and 'cap' not in done[0]


def test_turn_leg_caps_off_never_end_the_leg():
    h = Harness(turn_leg_done_after_s=0.0, turn_leg_done_after_rot_rad=0.0,
                leg_loop_abort_s=0.0)
    _k_turn_on_rev(h)
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.fsm.inputs.pose = (1.9, 0.35, RYAW + 1.0)
    h.tick(dt=1.0, n=120)
    assert h.pending_action(f.ACT_FOLLOW).token == tok


def test_turn_leg_cap_resets_per_leg():
    """The reverse leg ends on its time cap; the next (short forward) leg reaches its end at
    once with the yaw off and gets its own 20 s from then."""
    fwd2 = [(1.9, 0.35, math.pi), (1.0, 0.36, math.pi)]      # 0.9 m <= turn_leg_max_m
    h = Harness()
    _k_turn_on_rev(h, fwd2=fwd2)
    h.fsm.inputs.pose = (1.9, 0.35, RYAW + 1.0)
    h.tick(dt=0.1, n=201)
    assert h.goal(f.ACT_FOLLOW)['poses'] == fwd2
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.fsm.inputs.pose = (1.0, 0.36, math.pi + 1.0)
    m = h.mark()
    h.tick(dt=0.1, n=199)
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    h.tick(dt=0.1, n=2)
    assert 2 in h.fsm.cursor.areas[0].completed
    assert h.name == 'TRANSIT' and h.fsm.mission.sub_i == 3     # on to the last swath
    done = [t for t in logs(h, m) if 'leg done' in t]
    assert len(done) == 1 and 'sub-path 2' in done[0] and 'time cap' in done[0]


# =====================================================================
# fix 10: leg loop guard
# =====================================================================
def test_follow_without_pose_index_progress_is_aborted_and_retried():
    h = Harness(transit_retry_delay_s=3.0)
    mowing(h)                                    # line (0, 0) -> (4, 0), 0.4 m poses
    for k in range(1, 4):                        # drives 1.2 m, then loops in place
        h.fsm.inputs.pose = (0.4 * k, 0.0, 0.0)
        h.tick(dt=1.0)
    tok = h.pending_action(f.ACT_FOLLOW).token
    assert h.fsm.mission.progress_local == 3
    m = h.mark()
    h.tick(dt=1.0, n=89)
    assert h.pending_action(f.ACT_FOLLOW).token == tok and not h.since(m, f.CancelActions)
    h.tick(dt=1.0)
    assert h.fsm._action is None and h.since(m, f.CancelActions)
    assert h.fsm.mission.step == 'retry_wait' and h.fsm.mission.follow_fails == 1
    assert 'retry 1/3' in h.fsm.sub_state
    assert any('leg loop guard' in t for t in logs(h, m))
    assert any(t.startswith('follow_path finished: aborted') for t in logs(h, m))
    h.tick(dt=1.0, n=4)
    g = h.goal(f.ACT_FOLLOW)                     # the existing retry: from the reached pose
    assert abs(g['poses'][0][0] - 1.2) < 1e-6


def test_loop_guard_restarts_with_every_new_goal_and_does_not_fire_in_a_hold():
    """Localization hold for 100 s (the follow is cancelled): no loop abort while held; the
    resumed goal gets its own leg_loop_abort_s."""
    h = Harness()
    mowing(h)
    h.tick(dt=1.0, n=50)
    m = h.mark()
    i = h.fsm.inputs
    for _ in range(100):
        i.loc_state, i.loc_reason, i.loc_stamp = 'degraded', 'rtk float', h.t
        h.tick(dt=1.0)
    assert h.fsm.mission.step == 'loc_hold'
    for _ in range(5):
        i.loc_state, i.loc_stamp = 'ok', h.t
        h.tick(dt=1.0)
    assert h.fsm.mission.step == 'follow'
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.tick(dt=1.0, n=85)
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    assert not any('leg loop guard' in t for t in logs(h, m))
    h.tick(dt=1.0, n=6)
    assert any('leg loop guard' in t for t in logs(h, m))


def test_loop_guard_does_not_fire_in_a_dynamic_obstacle_wait():
    h = Harness(dynamic_wait_s=120.0)
    mowing(h)
    h.tick(dt=1.0, n=80)
    m = h.mark()
    i = h.fsm.inputs
    for _ in range(40):                          # person right in front for 40 s
        i.obstacle_kind, i.obstacle_class = 'dynamic', 'person'
        i.obstacle_distance, i.obstacle_bearing, i.obstacle_stamp = 0.8, 0.0, h.t
        h.tick(dt=1.0)
    assert h.fsm.mission.step == 'dyn_wait'
    assert not any('leg loop guard' in t for t in logs(h, m))


def test_loop_guard_does_not_fire_in_a_stuck_escape():
    h = Harness()
    mowing(h)
    h.fsm.inputs.pose = (1.0, 0.0, 0.0)
    h.tick(dt=1.0, n=80)                         # 80 s at pose index 2..3
    m = h.mark()
    h.fsm.inputs.stuck = True
    h.tick()
    h.fsm.inputs.stuck = False
    assert h.fsm._stk is not None
    h.tick(n=10)
    for k in range(1, 40):                       # the escape reverses slowly (~20 s)
        h.fsm.inputs.pose = (1.0 - 0.01 * k, 0.0, 0.0)
        h.tick(dt=0.5)
        if h.fsm._stk is None:
            break
    assert h.fsm._stk is None and h.name == 'MOWING'
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW)
    assert not any('leg loop guard' in t for t in logs(h, m))
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.tick(dt=1.0, n=80)                         # the resumed goal starts its own timer
    assert h.pending_action(f.ACT_FOLLOW).token == tok


# =====================================================================
# fix 11: route cusps
# =====================================================================
def _hairpin170(n1=20, n2=20):
    """East 2 m, then back at 170 deg (a hairpin at (2, 0)), 0.1 m poses."""
    a = [(0.1 * i, 0.0, 0.0) for i in range(n1 + 1)]
    h = math.radians(170.0)
    b = [(2.0 + 0.1 * i * math.cos(h), 0.1 * i * math.sin(h), h) for i in range(1, n2 + 1)]
    return a + b


def test_route_cusps_hairpin_yes_right_angle_no():
    c = geo.route_cusps(_hairpin170(), 2.0)
    assert len(c) == 1
    i, x, y, after = c[0]
    assert i == 20 and abs(x - 2.0) < 1e-9 and abs(y) < 1e-9
    assert abs(geo.wrap_angle(after - math.radians(170.0))) < 0.05
    corner = [(0.1 * i, 0.0, 0.0) for i in range(21)] + \
        [(2.0, 0.1 * i, 1.57) for i in range(1, 21)]
    assert geo.route_cusps(corner, 2.0) == []
    straight = [(0.1 * i, 0.0, 0.0) for i in range(30)]
    assert geo.route_cusps(straight, 2.0) == []
    # filleted hairpin (r 0.1 m arc instead of a vertex): one cusp, on the arc
    arc = [(2.0 + 0.1 * math.sin(t), 0.1 - 0.1 * math.cos(t), t)
           for t in [math.pi * k / 6 for k in range(1, 6)]]
    back = [(2.0 - 0.1 * i, 0.2, math.pi) for i in range(0, 21)]
    fil = geo.route_cusps(_hairpin170(n2=0) + arc + back, 2.0)
    assert len(fil) == 1 and abs(fil[0][1] - 2.05) < 0.1


def test_route_cusps_of_a_planned_route_include_sharp_leg_junctions():
    h = _route_h()
    route = _hairpin170()
    _answer_route(h, resp=_route_resp(route))
    cusps = h.fsm.mission.route_cusps
    assert len(cusps) == 1 and abs(cusps[0][1] - 2.0) < 1e-6
    assert len(h.fsm._route_legs['legs']) == 2


def _abort_at(h, pose, outcome=f.ABORTED):
    h.fsm.inputs.pose = pose
    a = h.pending_action(f.ACT_FOLLOW)
    assert a.purpose == 'route'
    h._apply(h.fsm.on_action_result(a.token, outcome, {}, h.t))


def test_route_abort_at_a_hairpin_turns_about_and_resumes_the_route():
    h = _route_h()
    route = _hairpin170()
    _answer_route(h, resp=_route_resp(route))
    m = h.mark()
    _abort_at(h, (1.95, 0.02, 0.0))              # stalled at the junction facing the old leg
    spin = h.pending_action(f.ACT_SPIN)
    assert spin.purpose == 'route_cusp'
    g = h.goal(f.ACT_SPIN)
    assert abs(g['angle'] - math.radians(170.0)) < 0.05 and g['time_allowance_s'] > 10.0
    assert any('route cusp at (2.00, 0.00): about-turn 1/2 then resume route' in t
               for t in logs(h, m))
    assert 'about-turn 1/2' in h.fsm.sub_state and h.name == 'TRANSIT'
    h.fsm.inputs.pose = (1.95, 0.02, math.radians(170.0))
    h.finish(f.ACT_SPIN)
    a = h.pending_action(f.ACT_FOLLOW)
    assert a.purpose == 'route'
    poses = h.goal(f.ACT_FOLLOW)['poses']
    assert poses[0] == route[20] and poses[-1] == route[-1]
    assert h.goal(f.ACT_FOLLOW)['controller_id'] == 'FollowPath'
    assert not starts(h, m, f.ACT_NAV) and h.fsm.mission.transit_fails == 0

    # second abort at the same cusp: second about-turn
    m = h.mark()
    _abort_at(h, (1.97, 0.0, 0.1))
    assert h.pending_action(f.ACT_SPIN)
    assert any('about-turn 2/2' in t for t in logs(h, m))
    h.finish(f.ACT_SPIN, f.ABORTED)              # even a failed spin: the route is the judge
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'route'
    assert h.goal(f.ACT_FOLLOW)['poses'] == route[20:]
    assert not starts(h, m, f.ACT_NAV)

    # third: about-turns used up -> the transit abort / retry path, still no costmap planner
    m = h.mark()
    _abort_at(h, (1.97, 0.0, 0.1))
    assert h.fsm._action is None and not starts(h, m, f.ACT_NAV)
    mi = h.fsm.mission
    assert mi.transit_fails == 1 and mi.step == 'retry_wait'
    assert 'at a route cusp, retry 1/3' in h.fsm.sub_state
    assert [e.kind for e in h.since(m, f.RecordIncident)] == ['transit_abort']


def test_route_cusp_about_turn_skipped_when_already_facing_the_route():
    h = _route_h()
    route = _hairpin170()
    _answer_route(h, resp=_route_resp(route))
    m = h.mark()
    _abort_at(h, (2.0, 0.05, math.radians(165.0)))
    assert not starts(h, m, f.ACT_SPIN)
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'route'
    assert h.goal(f.ACT_FOLLOW)['poses'][0] == route[20]


def test_route_abort_away_from_any_cusp_falls_back_to_the_costmap_planner():
    h = _route_h()
    route = _hairpin170()
    _answer_route(h, resp=_route_resp(route))
    target = h.fsm.mission.transit_target
    m = h.mark()
    _abort_at(h, (0.0, 0.0, 0.0))                # 2 m from the hairpin
    assert not starts(h, m, f.ACT_SPIN)
    assert h.goal(f.ACT_NAV)['pose'] == target and h.fsm.mission.route_fallback
    assert h.fsm.mission.fallback_ref == [tuple(p) for p in route]


def test_route_cusp_retries_zero_restores_the_old_fallback():
    h = _route_h(route_cusp_retries=0)
    _answer_route(h, resp=_route_resp(_hairpin170()))
    m = h.mark()
    _abort_at(h, (1.95, 0.02, 0.0))
    assert not starts(h, m, f.ACT_SPIN) and h.pending_action(f.ACT_NAV)


def test_off_route_graph_start_falls_back_to_the_costmap_planner():
    h = _route_h()
    target = h.fsm.mission.transit_target
    _answer_route(h, ok=False, resp=_route_resp(ok=False, message='start (0.00, 0.00) is off '
                                                                   'the route graph'))
    assert h.goal(f.ACT_NAV)['pose'] == target
    h.tick(n=20)                                 # inside the mowing area: bound never fires
    assert h.pending_action(f.ACT_NAV)


# ---- the corridor bound of the costmap fallback ----------------------
def _outside_route_h(**params):
    """Robot at (0, 0) outside the only mowing area (4..10, 4..10); sub-path from (5, 5)."""
    h = Harness(route_transits=True, **params)
    h.areas = [square(4, 4, 6)]
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([[(5.0 + 0.5 * k, 5.0, 0.0) for k in range(9)]]))
    assert h.name == 'TRANSIT'
    return h


def _diag():
    return [(0.1 * k, 0.1 * k, math.pi / 4) for k in range(51)]   # (0, 0) -> (5, 5)


def test_fallback_leaving_the_corridor_is_cancelled_as_a_transit_abort():
    h = _outside_route_h()
    _answer_route(h, resp=_route_resp(_diag()))
    _abort_at(h, (1.0, 1.0, 0.0))                # no cusp on a straight route: fallback
    nav = h.pending_action(f.ACT_NAV)
    h.tick()
    h.fsm.inputs.pose = (1.2, 0.85, 0.0)         # 0.25 m off the route: fine
    h.tick(n=5)
    assert h.fsm._action is nav
    m = h.mark()
    h.fsm.inputs.pose = (1.3, 0.7, -0.5)         # 0.42 m off
    h.tick()
    assert h.fsm._action is None and h.since(m, f.CancelActions)
    assert any('transit fallback left the corridor by 0.42 m: abort' in t for t in logs(h, m))
    mi = h.fsm.mission
    assert mi.transit_fails == 1 and mi.step == 'retry_wait'
    assert [e.kind for e in h.since(m, f.RecordIncident)] == ['transit_abort']
    h.tick(dt=1.0, n=4)                          # the retry asks for the route again
    assert h.fsm._service is not None and h.fsm._service.name == f.SRV_PLAN_ROUTE


def test_fallback_bound_off_keeps_the_fallback():
    h = _outside_route_h(transit_fallback_max_offpath_m=0.0)
    _answer_route(h, resp=_route_resp(_diag()))
    _abort_at(h, (1.0, 1.0, 0.0))
    nav = h.pending_action(f.ACT_NAV)
    h.fsm.inputs.pose = (1.6, 0.4, 0.0)
    h.tick(n=5)
    assert h.fsm._action is nav


def test_off_graph_fallback_is_bounded_once_back_in_the_corridor():
    """No route (start off the graph, 1.4 m from the area): the bound arms only once the
    robot is within it (inside the area) or when it moves another 0.35 m farther away."""
    h = _outside_route_h()
    h.fsm.inputs.pose = (3.0, 3.0, 0.0)          # 1.41 m from the area corner
    _answer_route(h, ok=False)
    nav = h.pending_action(f.ACT_NAV)
    h.tick()
    h.fsm.inputs.pose = (3.5, 3.5, 0.0)          # approaching: fine
    h.tick(n=3)
    h.fsm.inputs.pose = (4.5, 4.5, 0.0)          # inside the area: armed
    h.tick(n=3)
    assert h.fsm._action is nav and h.fsm.mission.fallback_armed
    m = h.mark()
    h.fsm.inputs.pose = (4.5, 3.5, 0.0)          # 0.5 m outside the area again
    h.tick()
    assert h.fsm._action is None and h.fsm.mission.transit_fails == 1
    assert any('left the corridor by 0.50 m' in t for t in logs(h, m))


def test_off_graph_fallback_moving_away_is_cancelled():
    h = _outside_route_h()
    h.fsm.inputs.pose = (3.0, 3.0, 0.0)
    _answer_route(h, ok=False)
    h.tick()
    h.fsm.inputs.pose = (2.7, 2.7, 0.0)          # 1.84 m off > 1.41 + 0.35: cancelled
    h.tick()
    assert h.fsm._action is None and h.fsm.mission.transit_fails == 1


# =====================================================================
# task 0.4 (f), run 3: "end reached" by distance only, capped by own length only
# =====================================================================
def test_turn_leg_end_by_distance_rotation_cap():
    """Default rule: the rotation cap still fires once the robot is within turn_leg_skip_m
    of the end (yaw off, so the yaw-pass skip does not end it)."""
    h = Harness()
    _fwd, rev, fwd2 = _k_turn_on_rev(h)
    h.fsm.inputs.pose = (1.9, 0.5, RYAW + 1.0)           # 0.15 m from the end
    h.tick()
    m = h.mark()
    yaw = RYAW + 1.0
    for k in range(70):
        yaw += 0.1
        h.fsm.inputs.pose = (1.9, 0.5, yaw)
        h.tick()
        if h.since(m, f.CancelActions):
            break
    assert 60 <= k < 66
    assert h.goal(f.ACT_FOLLOW)['poses'] == fwd2


def test_progress_index_at_the_last_pose_far_from_the_end_does_not_end_the_leg():
    """progress_local can jump to the last pose (run 3); only the distance counts."""
    h = Harness()
    _k_turn_on_rev(h)
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.fsm.inputs.pose = (1.9, 0.75, RYAW + 1.0)          # 0.4 m from the end
    h.fsm.mission.progress_local = 2                     # the last pose of the reverse leg
    h.tick(dt=0.1, n=300)                                # 30 s > the 20 s cap
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    assert not [t for t in logs(h) if 'leg done' in t]


def _ring_then_cusp(h):
    """A 49 m outer ring that ends at the cusp of a short reverse leg (run 3)."""
    h.areas = [square(-2, -2, 20)]
    start_until_planning(h)
    ring = []
    for (x0, y0), (x1, y1) in zip([(0, 0), (12, 0), (12, 12), (0, 12)],
                                  [(12, 0), (12, 12), (0, 12), (0, 0.25)]):
        n = 40
        for k in range(n):
            ring.append((x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, 0.0))
    ring.append((0.0, 0.25, -math.pi / 2))
    rev = [(0.0, 0.25, RYAW), (0.05, 0.6, RYAW), (0.1, 0.95, RYAW)]
    last = [(0.1, 0.95, 0.0), (4.0, 1.0, 0.0)]
    assert geo.path_length(ring) > 47.0
    h.fsm.inputs.pose = ring[0]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([ring, rev, last]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW)
    return ring


def test_long_ring_ending_at_a_cusp_is_not_a_turn_leg_and_not_capped():
    h = Harness(follow_chunk_m=0.0)
    ring = _ring_then_cusp(h)
    tok = h.pending_action(f.ACT_FOLLOW).token
    # 4.6 m before the end, on the last side; the nearest-index search matched the ring's
    # own end (its start/end are 0.25 m apart): progress_local at the last pose
    h.fsm.inputs.pose = (0.0, 4.6, -math.pi / 2)
    h.fsm.mission.progress_local = len(ring) - 1
    h.tick(dt=0.1, n=300)                                # 30 s > the 20 s cap
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    assert not h.fsm.mission.leg_watch['capped']
    assert 0 not in h.fsm.cursor.areas[0].completed
    assert not [t for t in logs(h) if 'leg done' in t]


def test_long_ring_old_rule_capped_it_revert_flag():
    h = Harness(follow_chunk_m=0.0, turn_leg_end_by_distance=False)
    ring = _ring_then_cusp(h)
    h.fsm.inputs.pose = (0.0, 4.6, -math.pi / 2)
    h.fsm.mission.progress_local = len(ring) - 1
    m = h.mark()
    h.tick(dt=0.1, n=210)
    done = [t for t in logs(h, m) if 'leg done' in t]
    assert len(done) == 1 and 'time cap' in done[0]


# =====================================================================
# task 0.4 (d): about-turn at every route corner > 90 deg
# =====================================================================
def _corner_route(deg, tgt):
    """East 2 m from (0, 0), then turn ``deg`` (left) and run 2 m; ends at ``tgt``."""
    a = [(0.1 * i, 0.0, 0.0) for i in range(21)]
    hd = math.radians(deg)
    b = [(2.0 + 0.1 * i * math.cos(hd), 0.1 * i * math.sin(hd), hd) for i in range(1, 20)]
    return a + b + [tuple(tgt)]


def test_route_corner_100_deg_gets_an_about_turn_before_the_next_leg():
    h = _route_h()
    assert h.fsm.p.route_cusp_radius_m == 1.0           # 0.5 -> 1.0 (run 3)
    route = _corner_route(100.0, h.fsm.mission.transit_target)
    _answer_route(h, resp=_route_resp(route))
    assert len(h.fsm._route_legs['legs']) == 2          # split at the 100 deg corner
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'route'   # robot faces leg 1: no pivot
    m = h.mark()
    h.finish(f.ACT_FOLLOW)                               # stops at the corner (2, 0), yaw 0
    spin = h.pending_action(f.ACT_SPIN)
    assert spin.purpose == 'route_corner' and h.name == 'TRANSIT'
    assert abs(h.goal(f.ACT_SPIN)['angle'] - math.radians(100.0)) < 0.1
    assert any('route corner at (2.00, 0.00)' in t and 'pivot' in t for t in logs(h, m))
    assert not starts(h, m, f.ACT_FOLLOW)               # no follow before the pivot
    h.fsm.inputs.pose = (2.0, 0.0, math.radians(100.0))
    h.finish(f.ACT_SPIN)
    a = h.pending_action(f.ACT_FOLLOW)
    assert a.purpose == 'route' and h.goal(f.ACT_FOLLOW)['poses'][0][:2] == (2.0, 0.0)
    assert len(starts(h, m, f.ACT_SPIN)) == 1 and not starts(h, m, f.ACT_NAV)


def test_route_corner_80_deg_is_split_at_the_default_threshold():
    """2026-10-15: route_corner_turn_rad 1.38 (79 deg): an 80 deg corner stops and pivots."""
    h = _route_h()
    assert abs(h.fsm.p.route_corner_turn_rad - 1.38) < 1e-9
    _answer_route(h, resp=_route_resp(_corner_route(80.0, h.fsm.mission.transit_target)))
    assert len(h.fsm._route_legs['legs']) == 2
    assert h.fsm._route_legs['legs'][0][-1][:2] == (2.0, 0.0)
    h.finish(f.ACT_FOLLOW)
    assert abs(h.goal(f.ACT_SPIN)['angle'] - math.radians(80.0)) < 0.1


def test_route_corner_80_deg_is_driven_without_an_about_turn_at_the_run4_threshold():
    h = _route_h(route_corner_turn_rad=1.5708)
    route = _corner_route(80.0, h.fsm.mission.transit_target)
    _answer_route(h, resp=_route_resp(route))
    assert len(h.fsm._route_legs['legs']) == 1
    m = h.mark()
    h.fsm.inputs.pose = (2.0, 0.0, 0.0)                  # at the corner, facing the old leg
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'route'
    assert not starts(h, m, f.ACT_SPIN)


def test_failed_corner_pivot_still_follows_the_leg_and_pivots_only_once():
    h = _route_h()
    route = _corner_route(120.0, h.fsm.mission.transit_target)
    _answer_route(h, resp=_route_resp(route))
    h.finish(f.ACT_FOLLOW)
    m = h.mark()
    h.finish(f.ACT_SPIN, f.ABORTED)                      # yaw unchanged
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'route'
    assert not starts(h, m, f.ACT_SPIN)


def test_route_corner_turn_off_restores_the_controller_turn():
    h = _route_h(route_corner_turn_rad=0.0)
    route = _corner_route(95.0, h.fsm.mission.transit_target)
    _answer_route(h, resp=_route_resp(route))
    assert len(h.fsm._route_legs['legs']) == 1           # 95 < route_split_turn_deg 100
    h2 = _route_h(route_corner_turn_rad=0.0)
    _answer_route(h2, resp=_route_resp(_corner_route(120.0, h2.fsm.mission.transit_target)))
    m = h2.mark()
    h2.finish(f.ACT_FOLLOW)
    assert h2.pending_action(f.ACT_FOLLOW).purpose == 'route' and not starts(h2, m, f.ACT_SPIN)


def test_dock_route_corner_gets_the_about_turn_too():
    h = Harness(route_transits=True)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    assert h.cmd(f.CMD_HOME)
    _answer_route(h, resp=_route_resp(_corner_route(110.0, (0.5, 2.0, 0.0))))
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'dock_route'
    h.finish(f.ACT_FOLLOW)
    assert h.pending_action(f.ACT_SPIN).purpose == 'route_corner'
    h.finish(f.ACT_SPIN)
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'dock_route'


# =====================================================================
# task 0.4 (e): the docking server navigates only near the approach pose
# =====================================================================
DOCK = (0.0, 0.0, 0.0)               # approach pose = (0.8, 0)


def _home_far(h):
    h.fsm.inputs.dock_pose = DOCK
    h.fsm.inputs.pose = (1.5, -5.6, 1.5)
    assert h.cmd(f.CMD_HOME)
    assert h.name == 'RETURNING_HOME'


def _dock_route():
    return [(1.5, -5.6, 1.5), (1.2, -3.0, 1.6), (0.8, 0.0, math.pi)]


def test_dock_route_abort_far_from_the_approach_pose_never_starts_the_docking_server():
    h = Harness(route_transits=True)
    _home_far(h)
    m = h.mark()
    for k in range(3):                                  # the route + 2 re-sends
        _answer_route(h, resp=_route_resp(_dock_route()))
        assert h.pending_action(f.ACT_FOLLOW).purpose == 'dock_route'
        h.fsm.inputs.pose = (1.3, -3.5, 1.6)            # aborted 3.6 m from the approach pose
        h.finish(f.ACT_FOLLOW, f.ABORTED)
    assert not starts(h, m, f.ACT_DOCK) and not starts(h, m, f.ACT_NAV)
    assert h.name == 'NAV_TO_DOCK_FAILED'
    assert 'not crossing the lawn' in h.fsm.sub_state
    assert sum('driving the dock route again' in t for t in logs(h, m)) == 2


def test_dock_route_resend_reaches_the_approach_then_docking_server():
    h = Harness(route_transits=True)
    _home_far(h)
    _answer_route(h, resp=_route_resp(_dock_route()))
    h.fsm.inputs.pose = (1.3, -3.5, 1.6)
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    req = _pending_route_request(h)                     # re-planned from where it stopped
    assert req['to_dock'] and req['start'][:2] == (1.3, -3.5)
    _answer_route(h, resp=_route_resp([(1.3, -3.5, 1.6), (0.8, 0.0, math.pi)]))
    h.finish(f.ACT_FOLLOW)                              # drives to (0.8, 0)
    assert h.pending_action(f.ACT_DOCK)


def test_no_dock_route_far_away_fails_instead_of_navigating():
    h = Harness(route_transits=True)
    _home_far(h)
    m = h.mark()
    for k in range(3):
        _answer_route(h, ok=False)
    assert h.name == 'NAV_TO_DOCK_FAILED'
    assert not starts(h, m, f.ACT_DOCK) and not starts(h, m, f.ACT_NAV)


def test_near_the_approach_pose_the_docking_server_takes_over():
    h = Harness(route_transits=True)
    _home_far(h)
    _answer_route(h, resp=_route_resp(_dock_route()))
    h.fsm.inputs.pose = (0.9, -0.3, 1.6)                # 0.32 m from (0.8, 0)
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    assert h.pending_action(f.ACT_DOCK)


def test_dock_server_nav_max_zero_restores_the_old_hand_over():
    h = Harness(route_transits=True, dock_server_nav_max_m=0.0)
    _home_far(h)
    _answer_route(h, resp=_route_resp(_dock_route()))
    h.fsm.inputs.pose = (1.3, -3.5, 1.6)
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    assert h.pending_action(f.ACT_DOCK)
