"""Task 0.4 (a): the boundary is a hard rule in every autonomous driving phase (2026-10-14
run 3: seven transit excursions up to 0.85 m outside the areas / paths;
docs/wheel_smoothness_field_tests.md, docs/ROADMAP_2026-10.md)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import Harness, line, plan, square, start_until_planning  # noqa: E402

EDGE = 1.8          # the allowed region ends at x = 1.8 (areas U paths, as ~/outside_m sees it)


def outside(p):
    return max(0.0, p[0] - EDGE)


def feed(h, pose, out=None):
    i = h.fsm.inputs
    i.pose = tuple(pose)
    i.outside_m = outside(pose) if out is None else out
    i.outside_stamp = h.t


def transit_h(**params):
    """TRANSIT (Nav2 goal running) with the robot at (1.0, 0) facing east."""
    h = Harness(**params)
    h.areas = [square(-1, -1, 12)]
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(5, 5, 9, 5)]))
    assert h.name == 'TRANSIT' and h.pending_action(f.ACT_NAV)
    feed(h, (1.0, 0.0, 0.0))
    h.tick()
    return h


def drive_east(h, x_to, step=0.05):
    """Drive the pose east until x_to (inclusive) or until the hard boundary takes over."""
    x = h.fsm.inputs.pose[0]
    while x < x_to - 1e-9 and h.fsm._hb is None:
        x = round(min(x_to, x + step), 6)
        feed(h, (x, 0.0, 0.0))
        h.tick()


def run_retrace(h, max_ticks=300):
    """Integrate the retrace's /cmd_vel_emergency commands (unicycle, 0.1 s ticks)."""
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


def assert_slow_two_wheel_reverse(cmds):
    half = 0.24
    moving = [(v, w) for v, w in cmds if v or w]
    assert moving
    for v, w in moving:
        assert v <= 0.0 and abs(v) <= 0.10 + 1e-9               # slow, never forward
        if v == 0.0:
            assert abs(w) >= 0.25 - 1e-9                       # proper counter-rotating pivot
        else:
            assert abs(v) - abs(w) * half >= 0.06 - 1e-9        # both wheels roll backwards


def test_transit_outside_by_0_11_cancels_retraces_and_counts_a_transit_abort():
    h = transit_h()
    m = h.mark()
    drive_east(h, 1.91)
    assert h.fsm.inputs.pose[0] == 1.91 and h.fsm._hb is not None
    assert h.since(m, f.CancelActions) and h.since(m, f.ZeroBurst)
    assert h.since(m, f.BladeOff) and not h.blade
    assert [e.kind for e in h.since(m, f.RecordIncident)] == ['boundary']
    assert any('HARD BOUNDARY: 0.11 m outside' in e.text for e in h.since(m, f.Log))
    assert h.fsm._action is None and h.name == 'TRANSIT'
    cmds = run_retrace(h)
    assert_slow_two_wheel_reverse(cmds)
    x = h.fsm.inputs.pose[0]
    assert x <= EDGE - 0.09                                     # back inside + 0.10 m
    mi = h.fsm.mission
    assert mi.transit_fails == 1 and mi.step == 'retry_wait'    # counted as a transit abort
    assert [e.kind for e in h.since(m, f.RecordIncident)] == ['boundary', 'transit_abort']
    assert not [e for e in h.since(m, f.StartAction)]           # no goal during the retrace
    h.tick(dt=0.5, n=8)                                         # the transit retry
    assert h.name == 'TRANSIT' and h.pending_action(f.ACT_NAV)


def test_second_excursion_in_a_mission_ends_in_stuck_needs_help():
    h = transit_h()
    drive_east(h, 1.91)
    run_retrace(h)
    h.tick(dt=0.5, n=8)
    assert h.pending_action(f.ACT_NAV)
    m = h.mark()
    drive_east(h, 1.95)
    assert h.fsm._hb is not None
    run_retrace(h)
    assert h.name == 'STUCK_NEEDS_HELP' and 'boundary: needs help' in h.fsm.sub_state
    assert any('left the boundary 2 times' in e.text for e in h.since(m, f.Log))
    assert h.fsm.mission is None and h.fsm.cursor.available     # resume cursor kept
    assert not h.fsm.motion_enabled()
    assert h.cmd(f.CMD_STOP)                                    # operator clears it
    assert h.fsm._hb_count == 0


def test_outside_by_0_09_does_nothing():
    h = transit_h()
    tok = h.pending_action(f.ACT_NAV).token
    m = h.mark()
    drive_east(h, 1.89)
    h.tick(n=20)
    assert h.fsm._hb is None and h.pending_action(f.ACT_NAV).token == tok
    assert not h.since(m, f.CancelActions) and not h.since(m, f.DriveCmd)


def test_inside_a_path_corridor_does_nothing():
    """Off the mowing area square, on a drawn path: map_server's ~/outside_m is 0 there
    (paths and the dock corridor count as inside; mower_map test_areas covers that)."""
    h = transit_h()
    tok = h.pending_action(f.ACT_NAV).token
    m = h.mark()
    for k in range(30):
        feed(h, (11.5 + 0.1 * k, 0.0, 0.0), out=0.0)          # beyond the square's x = 11
        h.tick()
    assert h.fsm._hb is None and h.pending_action(f.ACT_NAV).token == tok
    assert not h.since(m, f.CancelActions)


def test_stale_or_missing_outside_distance_does_nothing():
    h = transit_h()
    m = h.mark()
    feed(h, (2.5, 0.0, 0.0))
    h.fsm.inputs.outside_stamp = h.t - 5.0                      # older than 1 s
    h.tick(n=5)
    h.fsm.inputs.outside_m = None
    h.tick(n=5)
    assert h.fsm._hb is None and not h.since(m, f.CancelActions)


def test_hard_boundary_off_is_the_revert_flag():
    h = transit_h(hard_boundary=False)
    m = h.mark()
    drive_east(h, 2.5)
    h.tick(n=5)
    assert h.fsm._hb is None and not h.since(m, f.CancelActions)


def test_mowing_excursion_retraces_then_resumes_the_sub_path():
    h = Harness()
    h.areas = [square(-1, -1, 12)]
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0), line(0, 2, 4, 2)]))
    h.fsm.inputs.is_cutting = True
    feed(h, (0.0, 0.0, 0.0))
    h.tick()
    assert h.name == 'MOWING' and h.pending_action(f.ACT_FOLLOW) and h.blade
    m = h.mark()
    drive_east(h, 1.95)
    assert h.fsm._hb is not None and not h.blade and h.name == 'MOWING'
    cmds = run_retrace(h)
    assert_slow_two_wheel_reverse(cmds)
    assert h.fsm._hb_count == 1
    h.tick(n=3)
    assert h.name == 'MOWING' and h.pending_action(f.ACT_FOLLOW)
    poses = h.goal(f.ACT_FOLLOW)['poses']
    assert poses[0][0] >= 1.5 and poses[-1][0] == 4.0           # resumed, not restarted
    assert [e.kind for e in h.since(m, f.RecordIncident)][:1] == ['boundary']


def test_returning_home_excursion_retraces_then_reruns_the_dock():
    h = Harness(route_transits=True)
    feed(h, (1.0, 0.0, 0.0))
    assert h.cmd(f.CMD_HOME)
    assert h.name == 'RETURNING_HOME'
    m = h.mark()
    drive_east(h, 1.95)
    assert h.fsm._hb is not None and h.fsm._service is None and h.fsm._action is None
    run_retrace(h)
    assert h.name == 'RETURNING_HOME'
    s = h.fsm._service
    assert s is not None and s.name == f.SRV_PLAN_ROUTE         # a fresh dock route
    assert h.since(m, f.CancelActions)


def test_heading_calibration_drive_is_stopped_not_retraced():
    h = Harness()
    h.fsm._hcal = {'dir': 1, 'start': (1.0, 0.0), 'yaw': 0.0, 'done_at': None,
                   'outcome': None}
    h.fsm.phase = 'WAITING_FOR_RTK'
    h.fsm._hcal_wanted = True
    m = h.mark()
    feed(h, (2.0, 0.0, 0.0))
    h.tick()
    assert h.fsm._hb is None and h.fsm._hcal is None
    assert h.since(m, f.ZeroBurst) and not h.since(m, f.DriveCmd)
    assert h.name == 'IDLE'


def test_emergency_aborts_the_retrace_with_a_stop():
    h = transit_h()
    drive_east(h, 1.95)
    h.tick(n=8)                                                 # past the pause, reversing
    m = h.mark()
    h.fsm.inputs.lift = True
    h.tick()
    assert h.fsm._hb is None and h.name == 'EMERGENCY'
    dc = h.since(m, f.DriveCmd)
    assert dc and (dc[-1].linear, dc[-1].angular) == (0.0, 0.0)


def test_retrace_without_progress_needs_help():
    h = transit_h()
    drive_east(h, 1.95)
    h.tick(n=60)                                                # pose never moves
    assert h.name == 'STUCK_NEEDS_HELP' and h.fsm._hb is None


def test_retrace_follows_a_curved_track_with_both_wheels():
    """Excursion after a left-hand bend: the retrace reverses back round the bend (arcs with
    both wheels rolling backwards, or a pivot), ending inside."""
    def out_y(p):
        return max(0.0, p[1] - 0.6)

    h = transit_h()
    pts = [(1.0 + 0.05 * k, 0.0, 0.0) for k in range(1, 11)]            # to (1.5, 0)
    pts += [(1.5 + 0.5 * math.sin(a), 0.5 - 0.5 * math.cos(a), a)
            for a in [math.pi / 2 * k / 10 for k in range(1, 11)]]      # to (2.0, 0.5), north
    pts += [(2.0, 0.5 + 0.05 * k, math.pi / 2) for k in range(1, 6)]    # to (2.0, 0.75)
    for p in pts:
        if h.fsm._hb is not None:
            break
        h.fsm.inputs.pose = p
        h.fsm.inputs.outside_m = out_y(p)
        h.fsm.inputs.outside_stamp = h.t
        h.tick()
    assert h.fsm._hb is not None
    cmds = []
    for _ in range(300):
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
        h.fsm.inputs.pose = (x, y, yaw)
        h.fsm.inputs.outside_m = out_y((x, y))
        h.fsm.inputs.outside_stamp = h.t
    assert_slow_two_wheel_reverse(cmds)
    assert h.fsm.inputs.pose[1] < 0.6 - 0.05
    assert h.name == 'TRANSIT' and h.fsm.mission.transit_fails == 1
