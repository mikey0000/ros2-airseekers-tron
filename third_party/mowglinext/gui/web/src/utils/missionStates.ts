// Single source of truth for the high-level state -> dashboard control matrix.
//
// Covers both upstream MowgliNext BT state names (IDLE, IDLE_DOCKED, CHARGING,
// CRITICAL_BATTERY_CHARGING, RECORDING, MANUAL_MOWING, EMERGENCY, ...) and the
// Airseekers mission FSM names (mower_mission/mission_fsm.py STATE_CODES).
// Stock robots keep their previous behaviour: Start only from IDLE/IDLE_DOCKED,
// Home everywhere except IDLE_DOCKED.

export const HL_STATE_NULL = 0;
export const HL_STATE_IDLE = 1;
export const HL_STATE_AUTONOMOUS = 2;
export const HL_STATE_RECORDING = 3;
export const HL_STATE_MANUAL_MOWING = 4;

/** high_level_control Command values used by the GUI. */
export const CMD_START = 1;
export const CMD_HOME = 2;
export const CMD_STOP = 8;
export const CMD_RESET_EMERGENCY = 254;

/** Mission-in-progress phases published by mission_fsm. */
export const MISSION_PHASES = [
    "PREFLIGHT_CHECK", "UNDOCKING", "WAITING_FOR_RTK", "PLANNING", "MOWING",
    "TRANSIT", "AREA_UNREACHABLE", "BOUNDARY_PAUSED", "MOWING_COMPLETE",
] as const;

/** Autonomous drive-to-dock phases. */
export const DOCK_PHASES = [
    "RETURNING_HOME", "LOW_BATTERY_DOCKING", "RAIN_DETECTED_DOCKING",
    "COVERAGE_FAILED_DOCKING",
] as const;

/** Faults that need operator attention; RESET (254) is offered for these. */
export const LATCHED_FAULTS = [
    "BOUNDARY_EMERGENCY_STOP", "NAV_TO_DOCK_FAILED", "EMERGENCY",
] as const;

const ACTIVE_STATES = new Set([HL_STATE_AUTONOMOUS, HL_STATE_RECORDING, HL_STATE_MANUAL_MOWING]);
const ACTIVE_NAMES = new Set<string>([...MISSION_PHASES, ...DOCK_PHASES, "RECORDING", "MANUAL_MOWING"]);
const IDLE_START_NAMES = new Set(["IDLE", "IDLE_DOCKED"]);

type S = number | undefined | null;
type N = string | undefined | null;

/** A mow / recording / dock drive is in progress. */
export const isMissionActive = (state: S, name: N): boolean =>
    (state != null && ACTIVE_STATES.has(state)) || (!!name && ACTIVE_NAMES.has(name));

export const isLatchedFault = (name: N): boolean =>
    !!name && (LATCHED_FAULTS as readonly string[]).includes(name);

/** "Stop mowing" (Command 8) is offered. STOP is the one command the
 *  BOUNDARY_EMERGENCY_STOP latch accepts besides RESET. */
export const canStop = (state: S, name: N): boolean =>
    isMissionActive(state, name) || name === "BOUNDARY_EMERGENCY_STOP";

/** Reset (Command 254) is offered. */
export const canReset = (name: N): boolean => isLatchedFault(name);

export const canStart = (state: S, name: N): boolean =>
    state !== HL_STATE_AUTONOMOUS && !!name && IDLE_START_NAMES.has(name);

/** Home is hidden when already docked and while the boundary latch refuses it. */
export const canHome = (_state: S, name: N): boolean =>
    name !== "IDLE_DOCKED" && name !== "BOUNDARY_EMERGENCY_STOP";

/** STOP asks for a confirm only when the blade is actually spinning on the lawn. */
export const stopNeedsConfirm = (name: N, bladeOn: boolean | undefined): boolean =>
    !!bladeOn && (name === "MOWING" || name === "MANUAL_MOWING");
