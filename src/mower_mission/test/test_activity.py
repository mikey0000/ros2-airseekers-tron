# SPDX-License-Identifier: GPL-3.0-or-later
from mower_mission import activity as a
from mower_mission import mission_fsm as f


def act(phase, docked=False, charging=False):
    return a.activity_for(phase, docked, charging, f.MOTION_PHASES)


def test_idle_phases():
    for ph in f.IDLE_NAMES:
        assert act(ph) == a.IDLE
        assert act(ph, docked=True) == a.DOCKED_IDLE
        assert act(ph, charging=True) == a.DOCKED_IDLE


def test_motion_phases_never_low_power():
    for ph in f.MOTION_PHASES:
        for d in (False, True):
            assert not a.is_low_power(act(ph, docked=d, charging=d)), ph


def test_specific_modes():
    assert act('MANUAL_MOWING', docked=True) == a.MANUAL
    assert act('RECORDING') == a.MANUAL
    for ph in f.DOCK_PHASES:
        assert act(ph, docked=True) == a.DOCKING
    assert act('UNDOCKING', docked=True) == a.MOWING
    assert act('MOWING') == a.MOWING
    assert act('TRANSIT') == a.MOWING


def test_stopped_states_are_idle():
    for ph in ('EMERGENCY', 'MOWING_COMPLETE', 'STUCK_NEEDS_HELP', 'NAV_TO_DOCK_FAILED', ''):
        assert act(ph) == a.IDLE


def test_unknown_activity_is_active():
    assert a.is_low_power(a.IDLE) and a.is_low_power(a.DOCKED_IDLE)
    for x in (None, '', 'garbage', a.MANUAL, a.MOWING, a.DOCKING):
        assert not a.is_low_power(x)
