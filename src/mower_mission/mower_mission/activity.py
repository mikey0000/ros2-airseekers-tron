# SPDX-License-Identifier: GPL-3.0-or-later
"""Activity mode (``/mission/activity``, latched std_msgs/String).

A coarse summary of what the robot is doing, so that perception / logging nodes can
drop to an idle duty cycle while the robot sits still and come back as soon as it
moves:  ``docked_idle | idle | manual | mowing | docking``.

Consumers MUST treat anything they do not recognise (including no message yet) as
active: only ``docked_idle`` and ``idle`` allow throttling (see ``is_low_power``).
"""

DOCKED_IDLE = 'docked_idle'
IDLE = 'idle'
MANUAL = 'manual'
MOWING = 'mowing'
DOCKING = 'docking'
ALL = (DOCKED_IDLE, IDLE, MANUAL, MOWING, DOCKING)
LOW_POWER = frozenset((DOCKED_IDLE, IDLE))

_MANUAL_PHASES = ('MANUAL_MOWING', 'RECORDING')
_DOCK_PHASES = ('RETURNING_HOME', 'LOW_BATTERY_DOCKING', 'RAIN_DETECTED_DOCKING',
                'COVERAGE_FAILED_DOCKING')


def activity_for(phase, docked=False, charging=False, motion_phases=()):
    """Mission phase (+ dock contact) -> activity mode (pure)."""
    phase = str(phase or '')
    if phase in _MANUAL_PHASES:
        return MANUAL
    if phase in _DOCK_PHASES:
        return DOCKING
    # PREFLIGHT_CHECK wakes the sensors too: area enumeration + planning give the stereo
    # depth / cameras time to reopen and publish before the first Nav2 goal.
    if phase in motion_phases or phase in ('UNDOCKING', 'PREFLIGHT_CHECK'):
        return MOWING
    return DOCKED_IDLE if (docked or charging) else IDLE


def is_low_power(activity):
    """True only for the explicit idle modes; unknown / missing -> False (stay active)."""
    return activity in LOW_POWER
