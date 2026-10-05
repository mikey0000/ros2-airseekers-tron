"""Unit tests for the stub high-level state machine and the blade interlock (no ROS)."""

import os
import sys

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from mower_gui_bridge import state_machine as sm  # noqa: E402


def _machine(state=None):
    m = sm.HighLevelStateMachine()
    if state == sm.STATE_MANUAL_MOWING:
        m.command(sm.CMD_MANUAL_MOW)
    elif state == sm.STATE_RECORDING:
        m.command(sm.CMD_RECORD_AREA)
    return m


def test_initial_state_is_idle():
    m = sm.HighLevelStateMachine()
    assert m.snapshot() == (sm.STATE_IDLE, 'IDLE')


def test_idle_docked_name():
    m = sm.HighLevelStateMachine()
    r = m.set_docked(True)
    assert r.changed
    assert m.snapshot() == (sm.STATE_IDLE, 'IDLE_DOCKED')
    assert not m.set_docked(True).changed


def test_record_area_enters_recording_without_blade():
    m = sm.HighLevelStateMachine()
    r = m.command(sm.CMD_RECORD_AREA)
    assert r.success and r.changed
    assert m.snapshot() == (sm.STATE_RECORDING, 'RECORDING')
    assert sm.ACTION_CUTTER_ON not in r.actions


def test_record_finish_and_cancel_return_to_idle():
    for cmd in (sm.CMD_RECORD_FINISH, sm.CMD_RECORD_CANCEL):
        m = _machine(sm.STATE_RECORDING)
        r = m.command(cmd)
        assert r.success
        assert m.state == sm.STATE_IDLE
        assert 'not implemented' in r.log


def test_manual_mow_enters_state_and_requests_cutter_on():
    m = sm.HighLevelStateMachine()
    r = m.command(sm.CMD_MANUAL_MOW)
    assert r.success and r.changed
    assert m.snapshot() == (sm.STATE_MANUAL_MOWING, 'MANUAL_MOWING')
    assert r.actions == [sm.ACTION_CUTTER_ON]


def test_stop_from_manual_mowing_turns_cutter_off_and_zero_twist():
    m = _machine(sm.STATE_MANUAL_MOWING)
    r = m.command(sm.CMD_STOP)
    assert r.success and r.changed
    assert m.state == sm.STATE_IDLE
    assert r.actions.count(sm.ACTION_CUTTER_OFF) == 1
    assert sm.ACTION_ZERO_TWIST in r.actions
    assert r.actions.index(sm.ACTION_CUTTER_OFF) < r.actions.index(sm.ACTION_ZERO_TWIST)


def test_leaving_manual_mowing_via_record_turns_cutter_off():
    m = _machine(sm.STATE_MANUAL_MOWING)
    r = m.command(sm.CMD_RECORD_AREA)
    assert m.state == sm.STATE_RECORDING
    assert r.actions[0] == sm.ACTION_CUTTER_OFF


def test_start_and_home_refused():
    for cmd in (sm.CMD_START, sm.CMD_HOME):
        m = sm.HighLevelStateMachine()
        r = m.command(cmd)
        assert not r.success
        assert 'no mission layer' in r.log
        assert r.actions == [] and m.state == sm.STATE_IDLE


def test_unknown_commands_refused():
    for cmd in (sm.CMD_S2, sm.CMD_DELETE_MAPS, 42):
        m = sm.HighLevelStateMachine()
        r = m.command(cmd)
        assert not r.success and not r.changed and r.actions == []


def test_emergency_forces_state_and_cutter_off():
    m = _machine(sm.STATE_MANUAL_MOWING)
    r = m.set_emergency(True)
    assert r.changed
    assert m.snapshot() == (sm.STATE_NULL, 'EMERGENCY')
    assert r.actions == [sm.ACTION_CUTTER_OFF]
    # repeated "still active" updates are no-ops
    assert m.set_emergency(True).actions == []


def test_emergency_clear_returns_to_idle_not_blade_state():
    m = _machine(sm.STATE_MANUAL_MOWING)
    m.set_emergency(True)
    r = m.set_emergency(False)
    assert r.changed
    assert m.state == sm.STATE_IDLE
    assert sm.ACTION_CUTTER_ON not in r.actions


def test_commands_refused_during_emergency():
    m = sm.HighLevelStateMachine()
    m.set_emergency(True)
    for cmd in (sm.CMD_MANUAL_MOW, sm.CMD_RECORD_AREA):
        r = m.command(cmd)
        assert not r.success
        assert m.state == sm.STATE_NULL


def test_stop_during_emergency_keeps_emergency():
    m = sm.HighLevelStateMachine()
    m.set_emergency(True)
    r = m.command(sm.CMD_STOP)
    assert r.success
    assert m.state == sm.STATE_NULL
    assert sm.ACTION_CUTTER_OFF in r.actions and sm.ACTION_ZERO_TWIST in r.actions


def test_reset_emergency_requests_clear_but_waits_for_inputs():
    m = sm.HighLevelStateMachine()
    m.set_emergency(True)
    r = m.command(sm.CMD_RESET_EMERGENCY)
    assert r.success and r.actions == [sm.ACTION_CLEAR_ESTOP]
    assert m.state == sm.STATE_NULL     # stays until the inputs actually clear
    m.set_emergency(False)
    assert m.state == sm.STATE_IDLE


def test_interlock_allows_off_always():
    for state in (sm.STATE_NULL, sm.STATE_IDLE, sm.STATE_RECORDING, sm.STATE_MANUAL_MOWING):
        for em in (False, True):
            assert sm.cutter_request_allowed(False, state, em)[0]


def test_interlock_on_only_in_blade_states():
    assert sm.cutter_request_allowed(True, sm.STATE_MANUAL_MOWING, False)[0]
    assert sm.cutter_request_allowed(True, sm.STATE_AUTONOMOUS, False)[0]
    for state in (sm.STATE_NULL, sm.STATE_IDLE, sm.STATE_RECORDING):
        ok, reason = sm.cutter_request_allowed(True, state, False)
        assert not ok and 'refused' in reason


def test_interlock_on_refused_in_emergency():
    ok, reason = sm.cutter_request_allowed(True, sm.STATE_MANUAL_MOWING, True)
    assert not ok and 'emergency' in reason


def test_emergency_summary():
    assert sm.emergency_summary(False, False, False) == (False, False, False, '')
    active, latched, lift, reason = sm.emergency_summary(True, True, True)
    assert active and latched and lift
    assert 'estop' in reason and 'stop button' in reason and 'lift' in reason
    active, latched, lift, reason = sm.emergency_summary(False, False, True)
    assert active and not latched and lift
    active, latched, _, reason = sm.emergency_summary(False, False, False, mcu_stale=True)
    assert active and not latched and 'stale' in reason


def test_battery_percent_scaling():
    assert sm.battery_percent(0.5) == 50.0
    assert sm.battery_percent(1.0) == 100.0
    assert sm.battery_percent(73.0) == 73.0
    assert sm.battery_percent(float('nan')) == 0.0
    assert sm.battery_percent(None) == 0.0
