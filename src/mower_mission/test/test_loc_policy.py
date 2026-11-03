"""Graded RTK-loss policy (docs/REVIEW_2026-11-02.md step 7, 2026-11-03): degraded speed
cap + finish the straight swath, lost hold, retrace toward the last fix, dock / alert after
loc_lost_dock_s, no retry budget charged, dock exemptions."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import Harness, mowing, square  # noqa: E402
from test_loc_hold import held, run, set_loc, transit  # noqa: E402


def big_mow(**params):
    """Mowing line (0,0)->(4,0) inside a 20 m square: 5 m from every edge."""
    h = Harness(**params)
    h.areas = [square(-5, -5, 20)]
    mowing(h)
    h.tick()
    assert h.fsm.mission.step == 'follow'
    return h


def speed_calls(h, mark):
    return [e.request['params'] for e in h.since(mark, f.CallService)
            if e.name == f.SRV_SET_PARAMS and e.request.get('node') == f.PARAM_NODE_CONTROLLER]


def at(h, x, y=0.0, yaw=0.0):
    h.fsm.inputs.pose = (x, y, yaw)


def drive_cmds(h, mark):
    return [e for e in h.since(mark, f.DriveCmd)]


def follow_retrace(h, state='lost', secs=60.0, dt=0.2):
    """Tick while applying the mission's DriveCmd to the pose (unicycle)."""
    for _ in range(int(round(secs / dt))):
        m = h.mark()
        set_loc(h, state)
        h.tick(dt=dt)
        cmds = drive_cmds(h, m)
        if cmds:
            v, w = cmds[-1].linear, cmds[-1].angular
            x, y, th = h.fsm.inputs.pose
            h.fsm.inputs.pose = (x + v * dt * math.cos(th), y + v * dt * math.sin(th),
                                 th + w * dt)
        if h.fsm._loc is None or (h.fsm._loc['retraced'] and h.fsm._loc['rt'] is None):
            return


# ---- degraded ------------------------------------------------------------------------------
def test_degraded_caps_speed_and_finishes_the_swath_then_holds():
    h = big_mow()
    m = h.mark()
    run(h, 'degraded', 1.0, est=0.4)
    assert h.fsm.mission.step == 'follow' and h.fsm._loc is None and h.blade
    caps = speed_calls(h, m)
    assert caps and all(v == 0.15 for v in caps[-1].values()), caps
    assert any('finishing the current swath' in e.text for e in h.since(m, f.Log))
    at(h, 2.0)
    run(h, 'degraded', 1.0, est=0.45)
    assert h.fsm.mission.step == 'follow'
    at(h, 4.0)                          # swath end
    run(h, 'degraded', 0.5, est=0.5)
    assert held(h) and not h.blade
    assert h.fsm.mission.start_local == 10


def test_degraded_drift_over_edge_clearance_holds_at_once():
    h = big_mow()
    run(h, 'degraded', 0.5, est=4.8)    # 4.8 + 0.3 > 5 m clearance
    assert held(h)


def test_degraded_without_drift_estimate_holds():
    h = big_mow()
    run(h, 'degraded', 0.5, est=None)
    assert held(h)


def test_degraded_growing_drift_stops_the_swath():
    h = big_mow()
    run(h, 'degraded', 0.5, est=0.4)
    assert not held(h)
    at(h, 1.0)
    run(h, 'degraded', 0.5, est=4.9)
    assert held(h)


def test_lost_while_finishing_the_swath_holds():
    h = big_mow()
    run(h, 'degraded', 0.5, est=0.4)
    run(h, 'lost', 0.5, est=1.2)
    assert held(h) and h.fsm.sub_state.startswith('localization lost')


def test_degraded_on_the_edge_holds():
    """The toy mowing line lies on the area edge: no clearance, no swath finish."""
    h = Harness()
    mowing(h)
    h.tick()
    run(h, 'degraded', 0.5, est=0.05)
    assert held(h)


def test_degraded_transit_holds():
    h = Harness()
    transit(h)
    run(h, 'degraded', 0.5, est=0.4)
    assert held(h) and h.name == 'TRANSIT'


def test_speed_cap_lifted_when_ok():
    h = big_mow()
    run(h, 'degraded', 0.5, est=0.4)
    m = h.mark()
    run(h, 'ok', 0.5)
    caps = speed_calls(h, m)
    assert caps and all(v != 0.15 for v in caps[-1].values()), caps
    assert not h.fsm._loc_capped


def test_tilt_cap_survives_loc_restore():
    """Both caps share the controller: lifting one keeps the other."""
    h = big_mow()
    h.fsm._tilt_capped = True
    run(h, 'degraded', 0.5, est=0.4)
    m = h.mark()
    run(h, 'ok', 0.5)
    caps = speed_calls(h, m)
    assert caps and all(v == h.fsm.p.tilt_caution_speed_mps for v in caps[-1].values())


# ---- lost: retrace -------------------------------------------------------------------------
def _trail_then_lost(h, until_x, step=0.1):
    run(h, 'ok', 0.2)                   # fixed at (0, 0): the trail starts here
    x = 0.0
    while x < until_x - 1e-9:
        x += step
        at(h, x)
        run(h, 'degraded', 0.1, est=0.4)
    run(h, 'lost', 0.5, est=1.1)
    assert held(h)


def test_lost_retraces_toward_the_last_fix_then_waits_and_resumes():
    h = big_mow()
    _trail_then_lost(h, 2.0)
    m = h.mark()
    run(h, 'lost', 110.0, dt=5.0)
    assert not drive_cmds(h, m) and h.fsm._loc['rt'] is None    # < loc_lost_retrace_s
    run(h, 'lost', 15.0, dt=5.0)
    assert h.fsm._loc['rt'] is not None
    assert h.since(m, f.SuppressStuckGuard)[-1].active is True
    follow_retrace(h)
    x, y, _ = h.fsm.inputs.pose
    assert math.hypot(x, y) < 0.35, h.fsm.inputs.pose         # back near the fix
    assert h.fsm._loc['retraced'] and h.fsm._loc['rt'] is None and not h.blade
    assert held(h)
    m = h.mark()
    run(h, 'lost', 30.0, dt=5.0)        # waits, no second retrace
    assert not [c for c in drive_cmds(h, m) if c.linear != 0.0]
    run(h, 'ok', 3.5)                   # fixed again: resumes the mow
    assert h.fsm._loc is None and h.fsm.mission is not None
    # 2 m back from the held progress: transit to it, then the follow resumes there
    assert h.name == 'TRANSIT' and h.pending_action(f.ACT_NAV)
    assert h.fsm.mission.start_local == 5


def test_retrace_is_capped_at_loc_lost_retrace_max_m():
    h = big_mow()
    h.fsm.mission.subpaths[0] = [(0.4 * k, 0.0, 0.0) for k in range(21)]   # 8 m swath
    _trail_then_lost(h, 6.0)
    run(h, 'lost', 125.0, dt=5.0)
    assert h.fsm._loc['rt'] is not None
    follow_retrace(h)
    x = h.fsm.inputs.pose[0]
    assert 2.8 <= 6.0 - x <= 3.2, x


def test_ok_during_retrace_stops_it_and_resumes():
    h = big_mow()
    _trail_then_lost(h, 2.0)
    run(h, 'lost', 125.0, dt=5.0)
    assert h.fsm._loc['rt'] is not None
    m = h.mark()
    run(h, 'ok', 0.5)
    assert h.fsm._loc['rt'] is None and h.since(m, f.DriveCmd)[-1].linear == 0.0
    run(h, 'ok', 3.0)
    assert h.fsm._loc is None


# ---- lost: dock / alert after loc_lost_dock_s --------------------------------------------
def test_timeout_with_rtk_fixed_docks_as_incomplete():
    h = big_mow()
    run(h, 'lost', 1.0)
    m = h.mark()
    # receiver fixed but the EKF sigma gate keeps the monitor at degraded
    run(h, 'degraded', 605.0, dt=5.0, rtk='fixed', reason='ekf_map sigma 0.30 m > 0.15 m')
    names = h.statuses(m)
    assert 'MOWING_INCOMPLETE' in names, names
    assert h.name == 'RETURNING_HOME' and h.pending_action(f.ACT_DOCK)
    assert 'localization not ok' in h.fsm._incomplete_text


def test_timeout_with_rtk_fixed_stays_without_return_home():
    h = big_mow(return_home_on_incomplete=False)
    run(h, 'lost', 1.0)
    run(h, 'degraded', 605.0, dt=5.0, rtk='fixed')
    assert h.name == 'MOWING_INCOMPLETE' and h.fsm._action is None


# ---- budgets -------------------------------------------------------------------------------
def test_hold_never_charges_retry_budgets():
    h = big_mow()
    mm = h.fsm.mission
    mm.follow_fails, mm.transit_fails = 1, 2
    run(h, 'lost', 1.0)
    assert held(h)
    # anything the stop / cancel would have charged is given back on resume
    mm.follow_fails, mm.transit_fails, mm.route_plan_fails, mm.detours = 5, 5, 5, 5
    h.fsm._route_watch = {'token': -1, 's': 0.0, 't': 0.0}
    run(h, 'ok', 3.5)
    assert h.fsm._loc is None
    assert (mm.follow_fails, mm.transit_fails, mm.route_plan_fails, mm.detours) == (1, 2, 0, 0)
    assert h.fsm._route_watch is None or h.fsm._route_watch['token'] != -1


def test_transit_hold_keeps_transit_fails():
    h = Harness()
    transit(h)
    n = h.fsm.mission.transit_fails
    run(h, 'lost', 200.0, dt=5.0)
    run(h, 'ok', 3.5)
    assert h.fsm.mission.transit_fails == n and h.fsm.mission.step == 'transit'


# ---- dock exemptions -----------------------------------------------------------------------
def _home(h):
    h.fsm.inputs.pose = (3.0, 3.0, 0.0)
    h.cmd(f.CMD_HOME)
    assert h.name == 'RETURNING_HOME' and h.pending_action(f.ACT_DOCK)


def test_dock_exempt_while_degraded():
    h = Harness()
    _home(h)
    run(h, 'degraded', 300.0, dt=2.0)
    assert h.fsm._loc is None and h.pending_action(f.ACT_DOCK)


def test_dock_held_when_lost_beyond_exempt_max_then_redocks():
    h = Harness()
    _home(h)
    run(h, 'lost', 55.0, dt=2.0)
    assert h.fsm._loc is None and h.pending_action(f.ACT_DOCK)
    m = h.mark()
    run(h, 'lost', 10.0, dt=2.0)        # > loc_hold_dock_exempt_max_s (60 s)
    assert h.fsm._loc is not None and h.fsm._loc['ctx'] == 'dock'
    assert h.fsm._action is None and h.since(m, f.CancelActions)
    assert h.name == 'RETURNING_HOME'
    run(h, 'ok', 3.5)
    assert h.fsm._loc is None and h.pending_action(f.ACT_DOCK)


def test_dock_hold_timeout_without_fix_fails_docking():
    h = Harness()
    _home(h)
    run(h, 'lost', 70.0, dt=2.0)
    assert h.fsm._loc is not None
    run(h, 'lost', 605.0, dt=2.0)
    assert h.fsm._loc is None and h.name in ('NAV_TO_DOCK_FAILED',) + f.IDLE_NAMES
    assert any('no RTK fixed' in e.text for e in h.fx if isinstance(e, f.Log))


def test_camera_and_loc_caps_lift_independently():
    """All caps go through _push_speed_caps: the lowest wins, each lifts on its own."""
    h = big_mow(camera_blocked_speed_mps=0.2, loc_degraded_speed_mps=0.1)
    h.fsm.inputs.camera_blocked = True
    run(h, 'degraded', 0.5, est=0.4)
    m = h.mark()
    h.fsm.inputs.camera_blocked = False
    run(h, 'degraded', 0.5, est=0.4)
    assert all(v == 0.1 for v in speed_calls(h, m)[-1].values())
    h.fsm.inputs.camera_blocked = True
    run(h, 'degraded', 0.5, est=0.4)
    m = h.mark()
    run(h, 'ok', 0.5)                   # loc lifts, camera cap stays
    assert all(v == 0.2 for v in speed_calls(h, m)[-1].values())


def test_trail_is_continuous_so_a_late_loss_still_retraces():
    """RTK-fixed all the way (the trail used to restart on every fixed tick), the mower
    stops, then the status drops to lost: the ring still holds the driven track."""
    h = big_mow()
    h.fsm.mission.subpaths[0] = [(0.4 * k, 0.0, 0.0) for k in range(21)]   # 8 m swath
    x = 0.0
    while x < 6.0 - 1e-9:
        x += 0.1
        at(h, x)
        run(h, 'ok', 0.1, dt=0.1)
    run(h, 'lost', 0.5, est=0.3)
    assert held(h)
    run(h, 'lost', 125.0, dt=5.0)
    assert h.fsm._loc['rt'] is not None
    follow_retrace(h)
    assert 2.8 <= 6.0 - h.fsm.inputs.pose[0] <= 3.2, h.fsm.inputs.pose


def test_trail_ring_is_capped_at_loc_trail_len_m():
    h = big_mow(loc_trail_len_m=5.0)
    x = 0.0
    while x < 12.0 - 1e-9:
        x += 0.1
        at(h, x)
        run(h, 'ok', 0.1, dt=0.1)
    tr = h.fsm._loc_trail
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(tr, tr[1:]))
    assert 5.0 <= length <= 5.3, length
