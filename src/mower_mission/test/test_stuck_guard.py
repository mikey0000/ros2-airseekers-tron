"""Stuck guard in the mission FSM: escape sequence, terrain incident, STUCK_NEEDS_HELP."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import Harness, mowing  # noqa: E402


def drives(h, m):
    return [(e.linear, e.angular) for e in h.since(m, f.DriveCmd)]


def stuck(h, pose=(1.0, 0.0, 0.0)):
    h.fsm.inputs.pose = pose
    h.fsm.inputs.stuck = True
    m = h.mark()
    h.tick()
    return m


def test_stuck_while_mowing_escapes_marks_incident_and_resumes():
    h = Harness()
    mowing(h)
    m = stuck(h)
    assert h.name == 'MOWING' and not h.blade
    assert h.since(m, f.CancelActions) and h.since(m, f.ZeroBurst)
    assert [e.active for e in h.since(m, f.SuppressStuckGuard)] == [True]
    assert not drives(h, m)
    h.fsm.inputs.stuck = False                  # detector clears while suppressed
    h.tick(n=9)                                 # 1 s pause: no drive yet
    assert not drives(h, m)
    h.tick()
    assert drives(h, m)[-1] == (-0.1, 0.0)      # step 1: straight reverse
    assert 'escape 1/3' in h.fsm.sub_state
    for k in range(1, 31):                      # reverse at 0.1 m/s
        h.fsm.inputs.pose = (1.0 - 0.01 * k, 0.0, 0.0)
        h.tick()
        if h.since(m, f.RecordIncident):
            break
    inc = h.since(m, f.RecordIncident)
    assert inc and inc[-1].kind == 'stuck' and (inc[-1].x, inc[-1].y) == (1.0, 0.0)
    assert [e.active for e in h.since(m, f.SuppressStuckGuard)] == [True, False]
    assert any(e.name == f.SRV_CLEAR_COSTMAPS for e in h.since(m, f.CallService))
    assert drives(h, m)[-1] == (0.0, 0.0)
    assert h.name == 'MOWING'                   # re-dispatched: spin-up / follow again
    assert h.fsm._stk is None and len(h.fsm._stuck_hist) == 1


def test_step_without_progress_moves_to_next_step_then_needs_help():
    h = Harness()
    mowing(h)
    m = stuck(h)
    h.fsm.inputs.stuck = False
    h.tick(n=10)
    h.tick(dt=0.5, n=8)                         # no EKF progress for > 3 s
    d = drives(h, m)
    assert (-0.1, 0.0) in d
    fwd = [x for x in d if x[0] > 0]
    assert fwd and fwd[0][1] > 0.1              # step 2: forward 30 deg arc
    h.tick(dt=0.5, n=8)
    rev = [x for x in drives(h, m) if x[0] < 0 and x[1] < 0]
    assert rev                                  # step 3: reverse, opposite offset
    h.tick(dt=0.5, n=8)
    assert h.name == 'STUCK_NEEDS_HELP' and not h.blade
    assert h.fsm.sub_state == 'stuck: needs help at (1.00, 0.00)'
    assert not h.fsm.motion_enabled() and h.fsm.mission is None
    assert h.fsm.cursor.available             # resume cursor kept
    m2 = h.mark()
    h.tick(n=20)
    assert h.name == 'STUCK_NEEDS_HELP' and not drives(h, m2)
    h.cmd(f.CMD_STOP)                           # operator clears it
    assert h.name in f.IDLE_NAMES and h.since(m2, f.ResetStuckGuard)
    assert h.fsm._stuck_hist == []


def _escape(h, x0):
    stuck(h, (x0, 0.0, 0.0))
    h.fsm.inputs.stuck = False
    h.tick(n=10)
    for k in range(1, 40):
        h.fsm.inputs.pose = (x0 - 0.01 * k, 0.0, 0.0)
        h.tick()
        if h.fsm._stk is None:
            return
    raise AssertionError('no escape')


def test_max_recoveries_within_period_needs_help():
    h = Harness()
    mowing(h)
    for x0 in (1.0, 2.0, 3.0):
        _escape(h, x0)
        h.tick(n=11)                            # past stuck_ignore_after_s
        assert h.name != 'STUCK_NEEDS_HELP'
    stuck(h, (3.6, 1.0, 0.0))                   # 4th within 2 min
    assert h.name == 'STUCK_NEEDS_HELP'
    assert h.fsm.sub_state == 'stuck: needs help at (3.60, 1.00)'


def test_stuck_again_at_same_spot_needs_help_without_pushing():
    h = Harness()
    mowing(h)
    _escape(h, 1.0)
    h.tick(n=11)
    m = stuck(h, (1.1, 0.1, 0.0))
    assert h.name == 'STUCK_NEEDS_HELP' and not drives(h, m)


def test_stuck_near_an_old_spot_after_driving_away_escapes_again():
    """2026-10-08: after an escape the robot mowed on (> 1 m away) and came back down the
    next lane within 0.5 m of the spot: a fresh event, escape again instead of NEEDS_HELP."""
    h = Harness()
    mowing(h)
    _escape(h, 1.0)
    h.tick(n=11)
    for x in (1.5, 2.2, 2.5, 1.6):              # drives on, then returns along a lane
        h.fsm.inputs.pose = (x, 0.2, 0.0)
        h.tick()
    stuck(h, (1.1, 0.2, 0.0))
    assert h.name != 'STUCK_NEEDS_HELP' and h.fsm._stk is not None


def test_stuck_ignored_outside_stuck_phases_and_when_disabled():
    h = Harness()
    h.fsm.inputs.stuck = True
    h.tick(n=5)
    assert h.name in f.IDLE_NAMES and h.fsm._stk is None
    h = Harness(stuck_guard=False)
    mowing(h)
    stuck(h)
    assert h.fsm._stk is None and h.name == 'MOWING'


def test_stuck_while_docking_resumes_docking():
    h = Harness()
    h.fsm.inputs.pose = (3.0, 3.0, 0.0)
    h.cmd(f.CMD_HOME)
    assert h.name == 'RETURNING_HOME' and h.pending_action(f.ACT_DOCK)
    m = stuck(h, (3.0, 3.0, 0.0))
    assert h.name == 'RETURNING_HOME' and h.fsm._action is None
    h.fsm.inputs.stuck = False
    h.tick(n=10)
    for k in range(1, 40):
        h.fsm.inputs.pose = (3.0 - 0.01 * k, 3.0, 0.0)
        h.tick()
        if h.fsm._stk is None:
            break
    assert h.name == 'RETURNING_HOME' and h.pending_action(f.ACT_DOCK)
    assert h.since(m, f.RecordIncident)[-1].kind == 'stuck'


def test_emergency_aborts_escape():
    h = Harness()
    mowing(h)
    stuck(h)
    h.fsm.inputs.stuck = False
    h.tick(n=11)
    m = h.mark()
    h.fsm.inputs.lift = True
    h.tick()
    assert h.name == 'EMERGENCY' and h.fsm._stk is None
    assert [e.active for e in h.since(m, f.SuppressStuckGuard)] == [False]
    assert drives(h, m) == [(0.0, 0.0)]
