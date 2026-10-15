"""2026-10-15 field undock incident follow-ups (roadmap task 0.4).

After undocking the start route was split at (0.14, -0.28): the first leg was the 0.17 m
connector from the robot to the dock path (heading -146 deg), which then turns 90 deg onto
the path at (0.0, -0.38). The robot pivoted 111 deg to the stub, MPPI got a path that hooks
90 deg 0.17 m later, and it circled for 82 s (|v| <= 0.03, |w| 0.05-0.3) until STOP: the
rotation-counting progress checker never aborted it.

1. Route stubs: a corner within route_stub_trim_m of a route end cuts the stub off and the
   first leg joins where the robot projects onto the route (route_start_join_m).
2. Spinning is not progress on a route leg: transit_progress_checker per goal, plus the
   mission's route progress watch; a corner leg aborting next to its corner counts as reached.

Real geometry: test/data/undock_spin_2026-10-15_plan.json (the field /plan) and
test/data/real_dock_path_routes.json."""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import geometry as geo   # noqa: E402
from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import Harness, _answer_route, _route_h, _route_resp  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
TH = 1.38                                   # mission.yaml route_corner_turn_rad


def _with_yaw(pts):
    p = [tuple(q[:2]) for q in pts]
    return [(x, y, math.atan2(p[i + 1][1] - y, p[i + 1][0] - x) if i + 1 < len(p) else 0.0)
            for i, (x, y) in enumerate(p)]


def incident():
    with open(os.path.join(HERE, 'data', 'undock_spin_2026-10-15_plan.json')) as fh:
        d = json.load(fh)
    return _with_yaw(d['plan']), tuple(d['pose_at_plan']), tuple(d['pose_after_pivot'])


def real_routes():
    with open(os.path.join(HERE, 'data', 'real_dock_path_routes.json')) as fh:
        d = json.load(fh)
    return _with_yaw(d['route_from_dock']), _with_yaw(d['route_to_dock'])


def logs(h, m):
    return [e.text for e in h.since(m, f.Log)]


def sharp_turn_within(leg, within=0.3, th=TH):
    """Corners of at least ``th`` (inclusive, 1 deg) within ``within`` m of the leg start."""
    cum = geo.cumulative_lengths(leg)
    return [(i, t) for i, t in geo.route_corners(leg, th, min_arm_m=0.0,
                                                  tol_rad=f.ROUTE_CORNER_TOL_RAD)
            if cum[i] <= within]


def start_heading(leg):
    return f.MissionFSM._leg_heading(leg)


def answer(h, pose, route):
    h.fsm.inputs.pose = pose
    _answer_route(h, resp=_route_resp(route))


def first_motion(h):
    """(spin angle or None, first FollowPath goal) of the route just answered."""
    a = h.pending_action()
    spin = None
    if a.name == f.ACT_SPIN:
        spin = h.goal(f.ACT_SPIN)['angle']
        h.finish(f.ACT_SPIN)
    return spin, h.goal(f.ACT_FOLLOW)


# --- follow-up 2: the start stub ----------------------------------------------------------

def test_incident_plan_has_the_hook_and_the_old_split_made_the_stub_a_leg():
    route, pose, _ = incident()
    c = geo.route_corners(route, TH, tol_rad=f.ROUTE_CORNER_TOL_RAD)
    assert c and c[0][0] == 2 and abs(abs(c[0][1]) - math.pi / 2) < 0.02    # (0.0, -0.38)
    h = _route_h(route_stub_trim_m=0.0, route_start_join_m=0.0)          # revert flags
    answer(h, pose, route)
    spin, g = first_motion(h)
    assert spin is not None and abs(math.degrees(spin) - 111.0) < 2.0   # the field log
    assert geo.path_length(g['poses']) < 0.2                            # the 0.17 m stub


def test_incident_plan_first_leg_starts_at_the_corner_with_no_hook():
    route, pose, _ = incident()
    h = _route_h()
    m = h.mark()
    answer(h, pose, route)
    assert any('stub before the corner at (0.00, -0.38)' in t for t in logs(h, m))
    spin, g = first_motion(h)
    leg = g['poses']
    assert geo.dist(leg[0], (0.0015, -0.3783)) < 0.03         # starts on the corner
    assert geo.path_length(leg) > 1.0                         # a real leg, not a stub
    assert sharp_turn_within(leg) == []                       # no >= 79 deg turn in 0.3 m
    # heading AFTER the corner (the dock path, 123.6 deg); the robot (102 deg) is within
    # 79 deg of it, so no pivot at all (was: 111 deg to the stub)
    assert abs(math.degrees(start_heading(leg)) - 123.6) < 2.0
    assert spin is None
    assert geo.dist(leg[0], pose) < 0.2                       # joined next to the robot
    assert g['progress_checker_id'] == 'transit_progress_checker'


def test_incident_plan_after_the_pivot_overshoot_still_no_hook():
    """Replanned from where the robot stood after the field pivot (0.14, -0.42, -178 deg)."""
    route, _, pose = incident()
    h = _route_h()
    answer(h, pose, route)
    spin, g = first_motion(h)
    leg = g['poses']
    assert sharp_turn_within(leg) == []
    assert abs(math.degrees(start_heading(leg)) - 123.6) < 2.0
    target = start_heading(leg)
    if spin is not None:                                      # pivot targets the run heading
        assert abs(geo.wrap_angle(pose[2] + spin - target)) < 0.02


def test_real_route_from_dock_keeps_its_legs_and_has_no_start_hook():
    """The sim route from the approach pose: its first corner is 0.4 m on, not a stub."""
    route, _ = real_routes()
    pose = (route[0][0], route[0][1], math.radians(-146.4))
    h = _route_h()
    answer(h, pose, route)
    legs = h.fsm._route_legs['legs']
    assert len(legs) == 3
    assert legs[0][0] == route[0] and legs[-1][-1] == route[-1]       # nothing trimmed
    for lg in legs:
        assert sharp_turn_within(lg) == []
    spin, g = first_motion(h)
    assert spin is None and abs(math.degrees(start_heading(g['poses'])) + 146.4) < 1.0


def test_real_route_from_dock_with_the_robot_off_the_start_joins_ahead():
    """Undock leaves the robot ~0.12 m along / beside the route start: the first leg starts at
    its projection, never at the start behind it."""
    route, _ = real_routes()
    hd = math.atan2(route[1][1] - route[0][1], route[1][0] - route[0][0])
    along, side = 0.12, 0.06
    pose = (route[0][0] + along * math.cos(hd) - side * math.sin(hd),
            route[0][1] + along * math.sin(hd) + side * math.cos(hd), hd)
    h = _route_h()
    m = h.mark()
    answer(h, pose, route)
    assert any('first leg joins at' in t for t in logs(h, m))
    spin, g = first_motion(h)
    leg = g['poses']
    s0 = geo.path_projection(route, leg[0])[0]
    assert abs(s0 - along) < 0.02 and geo.dist(leg[0], pose) < side + 0.01
    assert spin is None and sharp_turn_within(leg) == []
    assert abs(geo.wrap_angle(leg[0][2] - hd)) < 0.02


def test_real_route_to_dock_end_corner_is_not_a_stub():
    """RETURNING_HOME: the 90 deg corner 0.38 m before the approach pose stays a split."""
    _, route = real_routes()
    out = geo.trim_route_stubs(route, TH, 0.3, tol_rad=f.ROUTE_CORNER_TOL_RAD)
    assert out == route


def test_end_stub_is_trimmed_and_still_arrives():
    """A 90 deg hook 0.2 m before the target: the route ends at the corner (0.2 m from the
    target, inside route_end_tolerance_m), no pivot into a 0.2 m leg."""
    a = [(0.1 * i, 0.0, 0.0) for i in range(51)]
    b = [(5.0, 0.1 * i, math.pi / 2) for i in range(1, 3)]
    out = geo.trim_route_stubs(a + b, TH, 0.3, tol_rad=f.ROUTE_CORNER_TOL_RAD)
    assert out == a
    h = _route_h()
    answer(h, (0.0, 0.0, 0.0), a + b)
    assert len(h.fsm._route_legs['legs']) == 1


def test_stub_trim_needs_a_sharp_corner_and_keeps_short_routes():
    a = [(0.1 * i, 0.0, 0.0) for i in range(3)]
    b = [(0.2 + 0.1 * i * math.cos(0.9), 0.1 * i * math.sin(0.9), 0.9) for i in range(1, 20)]
    assert geo.trim_route_stubs(a + b, TH, 0.3) == a + b                # 52 deg: kept
    c = [(0.2, 0.1 * i, math.pi / 2) for i in range(1, 3)]
    assert geo.trim_route_stubs(a + c, TH, 0.3) == a + c                # nothing left


def test_join_never_moves_the_start_backwards_or_far():
    route = [(0.1 * i, 0.0, 0.0) for i in range(30)]
    assert geo.join_route_from(route, (-0.1, 0.05), 0.5) == route       # behind the start
    assert geo.join_route_from(route, (0.3, 0.6), 0.5) == route         # too far off
    assert geo.join_route_from(route, (1.5, 0.0), 0.5) == route         # past the window
    out = geo.join_route_from(route, (0.25, 0.05), 0.5)
    assert abs(out[0][0] - 0.25) < 1e-9 and out[1] == route[3]


# --- follow-up 1: spinning in place is not progress -------------------------------------

def _long_route(h, pose=(0.0, 0.0, 0.0)):
    route = [(0.1 * i, 0.0, 0.0) for i in range(41)]
    answer(h, pose, route)
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'route'
    return route


def test_route_legs_and_coverage_follows_name_their_progress_checker():
    h = _route_h()
    _long_route(h)
    assert h.goal(f.ACT_FOLLOW)['progress_checker_id'] == 'transit_progress_checker'
    h2 = _route_h(route_progress_checker_id='progress_checker')          # revert
    _long_route(h2)
    assert h2.goal(f.ACT_FOLLOW)['progress_checker_id'] == 'progress_checker'
    h3 = Harness()
    from test_mission_fsm import start_until_planning, plan, line
    start_until_planning(h3)
    h3.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0)]))
    for _ in range(100):
        a = h3.fsm._action
        if a is not None and a.name == f.ACT_FOLLOW:
            break
        h3.fsm.inputs.is_cutting = True
        h3.tick()
    assert h3.goal(f.ACT_FOLLOW)['progress_checker_id'] == 'progress_checker'


def _spin_on_the_spot(h, seconds, x=0.05, y=0.04, r=0.12):
    """The incident: circling ~0.12 m around a point near the leg start, yaw turning."""
    for k in range(int(seconds * 10)):
        a = 0.2 * k * 0.1
        h.fsm.inputs.pose = (x + r * math.cos(a), y + r * math.sin(a), geo.wrap_angle(a))
        h.tick(dt=0.1)


def test_circling_on_a_route_leg_is_aborted_by_the_progress_watch():
    h = _route_h()
    _long_route(h)
    tok = h.pending_action(f.ACT_FOLLOW).token
    m = h.mark()
    _spin_on_the_spot(h, 19.0)
    assert h.pending_action(f.ACT_FOLLOW).token == tok        # not before the window
    _spin_on_the_spot(h, 2.0)
    assert any('route progress watch' in t for t in logs(h, m))
    assert [e for e in h.since(m, f.CancelActions)]
    a = h.fsm._action
    assert a is None or a.token != tok                       # handled as a route abort


def test_progress_watch_leaves_a_driving_leg_and_a_slow_one_alone():
    h = _route_h()
    _long_route(h)
    tok = h.pending_action(f.ACT_FOLLOW).token
    m = h.mark()
    for k in range(300):                                      # 0.012 m/s: 0.24 m per 20 s
        h.fsm.inputs.pose = (0.0012 * k, 0.03, 0.0)
        h.tick(dt=0.1)
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    assert not any('route progress watch' in t for t in logs(h, m))


def test_progress_watch_revert_flag():
    h = _route_h(route_progress_window_s=0.0)
    _long_route(h)
    tok = h.pending_action(f.ACT_FOLLOW).token
    m = h.mark()
    _spin_on_the_spot(h, 60.0)
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    assert not any('route progress watch' in t for t in logs(h, m))


def test_progress_watch_does_not_count_the_corner_pivot():
    """The pre-leg pivot is a Nav2 Spin: the watch only runs on the FollowPath leg."""
    h = _route_h()
    answer(h, (0.0, 0.0, math.pi), [(0.1 * i, 0.0, 0.0) for i in range(41)])
    assert h.pending_action(f.ACT_SPIN).purpose == 'route_corner'
    m = h.mark()
    _spin_on_the_spot(h, 30.0, x=0.0, y=0.0, r=0.0)
    assert h.pending_action(f.ACT_SPIN)
    assert not any('route progress watch' in t for t in logs(h, m))


def _corner_route():
    a = [(0.1 * i, 0.0, 0.0) for i in range(21)]
    b = [(2.0, 0.1 * i, math.pi / 2) for i in range(1, 20)]
    return a + b


def test_corner_leg_abort_next_to_its_corner_counts_as_reached():
    h = _route_h()
    answer(h, (0.0, 0.0, 0.0), _corner_route())
    m = h.mark()
    a = h.pending_action(f.ACT_FOLLOW)
    h.fsm.inputs.pose = (1.85, 0.05, 0.0)
    h._apply(h.fsm.on_action_result(a.token, f.ABORTED, {}, h.t))
    assert any('counted as reached' in t for t in logs(h, m))
    assert h.pending_action(f.ACT_SPIN).purpose == 'route_corner'      # next leg's pivot
    assert h.fsm._route_legs['i'] == 1 and not h.fsm.mission.route_fallback


def test_corner_leg_abort_far_from_the_corner_or_reverted_is_a_route_abort():
    for params, pose in (({}, (1.0, 0.05, 0.0)), ({'route_leg_end_accept_m': 0.0},
                                                  (1.85, 0.05, 0.3))):
        h = _route_h(**params)
        answer(h, (0.0, 0.0, 0.0), _corner_route())
        m = h.mark()
        a = h.pending_action(f.ACT_FOLLOW)
        h.fsm.inputs.pose = pose
        h._apply(h.fsm.on_action_result(a.token, f.ABORTED, {}, h.t))
        assert not any('counted as reached' in t for t in logs(h, m))
        assert h.fsm._route_legs is None or h.fsm._route_legs['i'] == 0


def test_dock_route_leg_is_watched_too():
    h = Harness(route_transits=True)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    assert h.cmd(f.CMD_HOME)
    _answer_route(h, resp=_route_resp([(0.1 * i, 0.0, 0.0) for i in range(41)]))
    a = h.pending_action(f.ACT_FOLLOW)
    assert a.purpose == 'dock_route'
    assert h.goal(f.ACT_FOLLOW)['progress_checker_id'] == 'transit_progress_checker'
    m = h.mark()
    _spin_on_the_spot(h, 21.0)
    assert any('route progress watch' in t for t in logs(h, m))


# --- run 4 (mow-040433): STUCK at split corners ------------------------------------------

def run4():
    with open(os.path.join(HERE, 'data', 'run4_2026-10-15_plans.json')) as fh:
        return json.load(fh)


def test_run4_plans_first_legs_have_no_hook_and_corner_legs_end_on_corners():
    """Every real /plan of the run, from the robot pose it was planned at: no leg begins with
    a >= 79 deg turn within 0.3 m, and every leg but the last ends on a split corner."""
    for k, pl in enumerate(run4()['plans']):
        h = _route_h()
        answer(h, tuple(pl['pose']), _with_yaw(pl['poses']))
        legs = h.fsm._route_legs['legs']
        for i, lg in enumerate(legs):
            assert sharp_turn_within(lg) == [], (k, i)
        corners = [(r[0], r[1]) for r in (_with_yaw(pl['poses'])[j] for j, _t in
                   geo.route_corners(_with_yaw(pl['poses']), TH, min_arm_m=0.0,
                                     tol_rad=f.ROUTE_CORNER_TOL_RAD))]
        for lg in legs[:-1]:
            assert any(geo.dist(lg[-1], c) < 1e-6 for c in corners), k


def test_run4_retry_plan_start_stub_is_trimmed():
    """plans[0] (the retry after STUCK1) starts with a 0.06 m stub then a 90 deg turn onto
    the dock path: the route starts on the dock path."""
    pl = run4()['plans'][0]
    h = _route_h()
    answer(h, tuple(pl['pose']), _with_yaw(pl['poses']))
    leg = h.fsm._route_legs['legs'][0]
    assert geo.dist(leg[0], (-2.61, 0.58)) < 0.03
    assert abs(math.degrees(leg[0][2]) - 148.4) < 1.0


def _drive_to(h, pose, seconds, dt=0.1):
    for _ in range(int(round(seconds / dt))):
        h.fsm.inputs.pose = pose
        h.tick(dt=dt)


def test_run4_stuck2_dock_route_corner_leg_stalled_short_of_the_corner_ends_and_pivots():
    """STUCK2: dock route plans[6], leg 1 into the (-3.25, 1.12) corner; the robot stopped
    0.12 m short at -147 deg and MPPI dithered. Now: stalled within 0.15 m for 2 s -> leg
    done -> Spin to the dock path (about +106 deg) -> leg 2."""
    d = run4()
    pl = d['plans'][6]
    route = _with_yaw(pl['poses'])
    h = Harness(route_transits=True)
    h.fsm.inputs.pose = tuple(pl['pose'])
    assert h.cmd(f.CMD_HOME)
    _answer_route(h, resp=_route_resp(route))
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'dock_route'
    legs = h.fsm._route_legs['legs']
    assert len(legs) == 2 and geo.dist(legs[0][-1], (-3.248, 1.119)) < 0.01
    stuck = tuple(d['stuck2_dither_pose'])
    assert 0.08 < geo.dist(stuck, legs[0][-1]) <= 0.15
    m = h.mark()
    tok = h.pending_action(f.ACT_FOLLOW).token
    _drive_to(h, stuck, 1.5)
    assert h.pending_action(f.ACT_FOLLOW).token == tok        # not before the stall time
    _drive_to(h, stuck, 1.0)
    assert any('corner (-3.25, 1.12) reached' in t for t in logs(h, m))
    spin = h.pending_action(f.ACT_SPIN)
    assert spin.purpose == 'route_corner'
    after = start_heading(legs[1])
    assert abs(geo.wrap_angle(stuck[2] + h.goal(f.ACT_SPIN)['angle'] - after)) < 0.02
    assert 95.0 < math.degrees(h.goal(f.ACT_SPIN)['angle']) < 115.0
    h.finish(f.ACT_SPIN)
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'dock_route'
    assert h.fsm._route_legs['i'] == 1


def test_run4_stuck1_connector_leg_reached_off_yaw_ends_and_pivots():
    """STUCK1 (route not in the bag; rebuilt as plan_route makes it: the robot's connector to
    the dock path, 0.36 m at ~47 deg, then 90 deg onto the path): the robot got within 0.07 m
    of the join 39 deg off the connector's yaw, the leg checker never fired. Now: within
    route_corner_xy_m -> leg done at once -> Spin ~51 deg (>= route_leg_pivot_min_rad)."""
    d = run4()
    start = tuple(d['stuck1_start_pose'])
    path = _with_yaw(d['plans'][0]['poses'])[5:]               # dock path from (-2.86, 0.76)
    join = (-2.834, 0.736)
    n = 4
    conn = [(start[0] + (join[0] - start[0]) * k / n, start[1] + (join[1] - start[1]) * k / n)
            for k in range(n + 1)]
    route = _with_yaw(conn + [p[:2] for p in path])
    h = _route_h()
    answer(h, start, route)
    legs = h.fsm._route_legs['legs']
    assert len(legs) >= 2 and geo.dist(legs[0][-1], join) < 1e-6
    m = h.mark()
    _drive_to(h, tuple(d['stuck1_dither_pose']), 0.2)
    assert any('corner (-2.83, 0.74) reached' in t for t in logs(h, m))
    assert h.pending_action(f.ACT_SPIN).purpose == 'route_corner'
    assert 40.0 < math.degrees(h.goal(f.ACT_SPIN)['angle']) < 60.0


def test_corner_stall_revert_flags():
    """route_corner_stall_s 0: the leg goal checker alone (the run 4 behaviour);
    route_leg_pivot_min_rad 0: no pivot under 79 deg at a corner."""
    d = run4()
    pl = d['plans'][6]
    h = Harness(route_transits=True, route_corner_stall_s=0.0, route_progress_window_s=0.0)
    h.fsm.inputs.pose = tuple(pl['pose'])
    assert h.cmd(f.CMD_HOME)
    _answer_route(h, resp=_route_resp(_with_yaw(pl['poses'])))
    tok = h.pending_action(f.ACT_FOLLOW).token
    _drive_to(h, tuple(d['stuck2_dither_pose']), 30.0)
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    h2 = _route_h(route_leg_pivot_min_rad=0.0)
    a = [(0.1 * i, 0.0, 0.0) for i in range(21)]
    b = [(2.0 + 0.1 * i * math.cos(1.5), 0.1 * i * math.sin(1.5), 1.5) for i in range(1, 20)]
    answer(h2, (0.0, 0.0, 0.0), a + b)
    _drive_to(h2, (1.97, 0.0, 0.7), 0.2)                 # turned 40 deg already: no pivot
    assert h2.pending_action(f.ACT_FOLLOW).purpose == 'route'      # MPPI turns in, no Spin
    assert h2.fsm._route_legs['i'] == 1


def test_costmap_fallback_spinning_in_place_is_a_transit_abort():
    """Sim 2026-10-15 (forced spin): after the route leg aborted (transit_progress_checker),
    the /navigate_to_pose fallback spun in place for minutes. Now: no 0.15 m closer to the
    target in 20 s -> cancel, transit abort (retry)."""
    h = _route_h()
    _answer_route(h, ok=False)                     # no route: costmap planner transit
    a = h.pending_action(f.ACT_NAV)
    m = h.mark()
    _spin_on_the_spot(h, 19.0, x=0.0, y=0.0, r=0.05)
    assert h.pending_action(f.ACT_NAV).token == a.token
    _spin_on_the_spot(h, 2.0, x=0.0, y=0.0, r=0.05)
    assert any('(progress watch): abort' in t for t in logs(h, m))
    assert h.fsm.mission.transit_fails == 1


def test_costmap_fallback_driving_toward_the_target_is_left_alone():
    h = _route_h()
    _answer_route(h, ok=False)
    tok = h.pending_action(f.ACT_NAV).token
    for k in range(400):                                 # 0.1 m/s toward (5, 5)
        h.fsm.inputs.pose = (0.007 * k, 0.007 * k, 0.785)
        h.tick(dt=0.1)
    assert h.pending_action(f.ACT_NAV).token == tok
