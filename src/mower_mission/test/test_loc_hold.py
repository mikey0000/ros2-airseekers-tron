"""Localization hold in the mission FSM (/localization/status degraded|lost -> stop, blade off)."""

import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import (Harness, mowing, start_until_planning, plan, line,  # noqa: E402
                              square)

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'config', 'mission.yaml')


def set_loc(h, state, reason='rtk float', rtk=None, est=None):
    """Fresh /localization/status message received now."""
    i = h.fsm.inputs
    i.loc_state, i.loc_reason, i.loc_stamp = state, reason, h.t
    i.loc_rtk = rtk if rtk is not None else ('fixed' if state == 'ok' else 'float')
    i.loc_est_drift = est


def run(h, state, secs, dt=0.5, reason='rtk float', rtk=None, est=None):
    """Tick for `secs` while the monitor keeps publishing `state` (status stays fresh)."""
    for _ in range(int(round(secs / dt))):
        set_loc(h, state, reason, rtk, est)
        h.tick(dt=dt)


def hold(h, state='degraded'):
    set_loc(h, state)
    h.tick()


def mow_follow(h):
    mowing(h)
    h.tick()
    assert h.fsm.mission.step == 'follow' and h.name == 'MOWING'


def transit(h):
    h.areas = [square(-1, -1, 12)]
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(5, 5, 9, 5)]))
    assert h.name == 'TRANSIT' and h.fsm.mission.step == 'transit'
    return h.goal(f.ACT_NAV)['pose']


def held(h):
    return h.fsm._loc is not None and h.fsm.mission.step == 'loc_hold'


# ---- 1, 2: entering the hold ---------------------------------------------------------------
def test_degraded_while_following_holds():
    h = Harness()
    mow_follow(h)
    assert h.blade
    m = h.mark()
    hold(h, 'degraded')
    assert h.since(m, f.CancelActions) and h.since(m, f.ZeroBurst) and h.since(m, f.BladeOff)
    assert h.fsm.mission.step == 'loc_hold' and h.name == 'MOWING'
    assert h.fsm.sub_state.startswith('localization degraded')
    assert h.fsm._action is None and not h.blade


def test_lost_holds():
    h = Harness()
    mow_follow(h)
    hold(h, 'lost')
    assert held(h) and h.fsm.sub_state.startswith('localization lost')


def test_ok_and_empty_do_not_hold():
    for state in ('ok', ''):
        h = Harness()
        mow_follow(h)
        m = h.mark()
        run(h, state, 2.0)
        assert h.fsm.mission.step == 'follow' and h.fsm._loc is None
        assert not h.since(m, f.CancelActions) and not h.since(m, f.BladeOff)


# ---- 3, 4: stale / never received / disabled -----------------------------------------------
def test_stale_status_is_degraded():
    """Review 2026-11-02 H3a: a monitor that stops publishing fails closed."""
    h = Harness()
    mow_follow(h)
    m = h.mark()
    set_loc(h, 'ok')
    h.tick(dt=4.5)                      # still fresh (loc_status_stale_s 5 s): no hold
    assert h.fsm.mission.step == 'follow' and h.fsm._loc is None
    h.tick(dt=1.0)                      # 5.5 s old: degraded -> stop, blade off
    assert h.fsm.mission.step == 'loc_hold' and h.fsm._loc is not None
    assert h.since(m, f.BladeOff)
    assert 'stale' in h.fsm.sub_state or 'degraded' in h.fsm.sub_state
    run(h, 'ok', 4.0)                   # the monitor is back and ok: resumes
    assert h.fsm.mission.step == 'follow' and h.fsm._loc is None


def test_never_received_does_not_hold():
    h = Harness()
    mow_follow(h)
    h.fsm.inputs.loc_state = 'degraded'
    assert h.fsm.inputs.loc_stamp is None
    h.tick(n=5)
    assert h.fsm.mission.step == 'follow' and h.fsm._loc is None


def test_disabled_param_does_not_hold():
    h = Harness(loc_hold=False)
    mow_follow(h)
    run(h, 'lost', 3.0)
    assert h.fsm.mission.step == 'follow' and h.fsm._loc is None and h.blade


# ---- 5: resume -----------------------------------------------------------------------------
def test_resume_needs_continuous_ok_for_loc_resume_s():
    h = Harness()
    mow_follow(h)
    hold(h)
    run(h, 'ok', 2.0)                   # < 3 s
    assert held(h)
    m = h.mark()
    run(h, 'ok', 1.5)                   # >= 3 s in total
    assert h.fsm._loc is None and h.fsm.mission.step != 'loc_hold'
    assert h.name == 'MOWING'
    h.tick()
    starts = [e.name for e in h.since(m, f.StartAction)]
    assert f.ACT_FOLLOW in starts, starts
    assert h.fsm._action is not None and h.blade


def test_degraded_interrupts_ok_and_resets_resume_timer():
    h = Harness()
    mow_follow(h)
    hold(h)
    run(h, 'ok', 2.5)
    assert held(h)
    run(h, 'degraded', 0.5)             # resets
    run(h, 'ok', 2.5)                   # 2.5 s again: still < 3 s
    assert held(h)
    run(h, 'ok', 1.0)
    assert not held(h) and h.fsm._loc is None


def test_stale_ok_does_not_count_towards_resume():
    h = Harness()
    mow_follow(h)
    hold(h)
    set_loc(h, 'ok')
    h.tick(dt=6.0)                      # status went stale: no resume
    h.tick(dt=6.0)
    assert held(h)


# ---- 6: transit ----------------------------------------------------------------------------
def test_transit_hold_cancels_nav_and_resends_goal_on_resume():
    h = Harness()
    target = transit(h)
    m = h.mark()
    hold(h, 'degraded')
    assert h.name == 'TRANSIT' and held(h) and h.fsm._action is None
    assert h.since(m, f.CancelActions) and h.since(m, f.ZeroBurst)
    assert h.fsm.sub_state.startswith('localization degraded')
    run(h, 'ok', 3.5)
    assert h.fsm._loc is None and h.fsm.mission.step == 'transit'
    assert h.name == 'TRANSIT' and h.goal(f.ACT_NAV)['pose'] == target


# ---- 7: timeout (graded policy 2026-11-03: loc_lost_dock_s) ---------------------------------
def test_hold_timeout_without_fix_latches_incomplete_and_stays():
    h = Harness()
    mow_follow(h)
    hold(h, 'lost')
    run(h, 'lost', 590.0, dt=5.0)
    assert held(h)
    m = h.mark()
    run(h, 'lost', 15.0, dt=5.0)        # past loc_lost_dock_s (600 s), rtk not fixed
    assert h.name == 'MOWING_INCOMPLETE' and h.fsm.mission is None and h.fsm._loc is None
    assert h.since(m, f.BladeOff) and h.since(m, f.ZeroBurst)
    errs = [e.text for e in h.since(m, f.Log) if e.level == 'error']
    assert any('localization not ok' in t for t in errs), errs
    assert 'localization not ok' in h.fsm.sub_state
    assert h.fsm._action is None and not h.blade     # no dock: stays put
    h.tick(n=20)                        # stays stopped, no re-hold on a dead mission
    assert h.fsm._loc is None and h.fsm._action is None
    assert h.fsm.cursor.current_command == f.CMD_START       # START resumes

# ---- 8: another guard takes over -----------------------------------------------------------
def test_emergency_during_hold_clears_it_and_no_later_resume():
    h = Harness()
    mow_follow(h)
    hold(h)
    assert held(h)
    h.fsm.inputs.lift = True
    set_loc(h, 'degraded')
    h.tick()
    assert h.name == 'EMERGENCY' and h.fsm.mission is None
    h.fsm.inputs.lift = False
    m = h.mark()
    # _run_phase is skipped during EMERGENCY, so the stale context is dropped on the first
    # phase tick afterwards (mission is already gone: nothing can resume).
    run(h, 'degraded', 3.0)
    assert h.fsm._loc is None and h.name != 'MOWING'
    run(h, 'ok', 10.0)
    assert h.fsm._loc is None and h.name != 'MOWING'
    assert not h.since(m, f.StartAction) or \
        f.ACT_FOLLOW not in [e.name for e in h.since(m, f.StartAction)]
    assert not h.blade


# ---- 9: docking is never held --------------------------------------------------------------
def test_docking_is_not_held():
    h = Harness()
    h.fsm.inputs.pose = (3.0, 3.0, 0.0)
    h.cmd(f.CMD_HOME)
    assert h.name == 'RETURNING_HOME' and h.pending_action(f.ACT_DOCK)
    m = h.mark()
    run(h, 'lost', 3.0)
    assert h.fsm._loc is None and h.name == 'RETURNING_HOME'
    assert h.pending_action(f.ACT_DOCK)
    assert not h.since(m, f.CancelActions)


# ---- 10: param plumbing --------------------------------------------------------------------
LOC_DEFAULTS = (('loc_status_stale_s', 5.0), ('loc_resume_s', 3.0),
                ('caution_speed_mps', 0.15), ('loc_degraded_edge_margin_m', 0.3),
                ('loc_lost_retrace_s', 120.0), ('loc_lost_retrace_max_m', 3.0),
                ('loc_lost_dock_s', 600.0), ('loc_hold_dock_exempt_max_s', 60.0))


def test_param_defaults_and_yaml_keys():
    p = f.Params()
    assert p.loc_hold is True
    for k, v in LOC_DEFAULTS:
        assert getattr(p, k) == v, k
    with open(CONFIG) as fh:
        cfg = yaml.safe_load(fh)['behavior_tree_node']['ros__parameters']
    assert cfg['loc_hold'] is True
    for k, v in LOC_DEFAULTS:
        assert isinstance(cfg[k], float) and cfg[k] == v, k
    q = f.Params.from_dict({k: cfg[k] for k, _ in LOC_DEFAULTS})
    assert q.loc_lost_dock_s == 600.0
