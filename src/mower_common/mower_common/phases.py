# SPDX-License-Identifier: GPL-3.0-or-later
"""Mission phases that gate wheel motion: the single source of truth.

mower_mission.mission_fsm publishes them (latched /motion_enabled) and
mower_control.cmd_vel_slew enforces them in its motion gate; both import them from here so
the interlock cannot drift (code review 2026-10-23: cmd_vel_slew kept a hand copy).
"""

DOCK_PHASES = ('RETURNING_HOME', 'LOW_BATTERY_DOCKING', 'RAIN_DETECTED_DOCKING',
               'COVERAGE_FAILED_DOCKING')
# Phases in which the wheels may move. Everything else -- IDLE*, CHARGING, EMERGENCY,
# BOUNDARY_EMERGENCY_STOP, MOWING_COMPLETE/INCOMPLETE, ... -- keeps them stopped.
MOTION_PHASES = ('MANUAL_MOWING', 'RECORDING', 'UNDOCKING', 'WAITING_FOR_RTK',
                 'PLANNING', 'TRANSIT', 'MOWING', 'AREA_UNREACHABLE',
                 'BOUNDARY_PAUSED') + DOCK_PHASES
# Phases allowed to move while the base reports docked/charging (explicit undock;
# docking itself ends on the contacts; manual: the operator may drive off the dock).
DOCKED_MOTION_PHASES = ('UNDOCKING', 'MANUAL_MOWING') + DOCK_PHASES
