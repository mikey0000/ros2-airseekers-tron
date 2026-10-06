import {describe, expect, it} from "vitest";
import {
    CMD_UNDOCK, recordingKind, canHome, canReset, canUndock, isDocked, canStart, canStop, isLatchedFault, isMissionActive, isNotice, stopNeedsConfirm,
} from "./missionStates.ts";

// [state_name, numeric state, active, latched, stop, reset, start, home]
const MATRIX: [string, number, boolean, boolean, boolean, boolean, boolean, boolean][] = [
    // upstream / shared idle names
    ["IDLE", 1, false, false, false, false, true, true],
    ["IDLE_DOCKED", 1, false, false, false, false, true, false],
    ["CHARGING", 1, false, false, false, false, false, true],
    ["CRITICAL_BATTERY_CHARGING", 1, false, false, false, false, false, true],
    ["RECORDING", 3, true, false, true, false, false, true],
    ["MANUAL_MOWING", 4, true, false, true, false, false, true],
    ["EMERGENCY", 0, false, true, false, true, false, true],
    // mission FSM
    ["PREFLIGHT_CHECK", 2, true, false, true, false, false, true],
    ["UNDOCKING", 2, true, false, true, false, false, true],
    ["WAITING_FOR_RTK", 2, true, false, true, false, false, true],
    ["PLANNING", 2, true, false, true, false, false, true],
    ["TRANSIT", 2, true, false, true, false, false, true],
    ["MOWING", 2, true, false, true, false, false, true],
    ["BOUNDARY_PAUSED", 2, true, false, true, false, false, true],
    ["MOWING_COMPLETE", 2, true, false, true, false, false, true],
    ["RETURNING_HOME", 2, true, false, true, false, false, true],
    ["LOW_BATTERY_DOCKING", 2, true, false, true, false, false, true],
    ["RAIN_DETECTED_DOCKING", 2, true, false, true, false, false, true],
    ["COVERAGE_FAILED_DOCKING", 2, true, false, true, false, false, true],
    ["BOUNDARY_EMERGENCY_STOP", 0, false, true, true, true, false, false],
    ["NAV_TO_DOCK_FAILED", 0, false, true, false, true, false, true],
    ["MOWING_INCOMPLETE", 1, false, false, false, false, true, true],
];

describe("mission state matrix", () => {
    it.each(MATRIX)("%s", (name, state, active, latched, stop, reset, start, home) => {
        expect(isMissionActive(state, name)).toBe(active);
        expect(isLatchedFault(name)).toBe(latched);
        expect(canStop(state, name)).toBe(stop);
        expect(canReset(name)).toBe(reset);
        expect(canStart(state, name)).toBe(start);
        expect(canHome(state, name)).toBe(home);
    });

    it("treats an unknown name as active when the numeric state is autonomous", () => {
        expect(isMissionActive(2, "OBSTACLE_BACKOFF")).toBe(true);
        expect(canStop(2, "OBSTACLE_BACKOFF")).toBe(true);
        expect(canStart(2, "IDLE")).toBe(false);
    });

    it("flags MOWING_INCOMPLETE as a notice, not a fault", () => {
        expect(isNotice("MOWING_INCOMPLETE")).toBe(true);
        expect(isNotice("MOWING_COMPLETE")).toBe(false);
        expect(isNotice("NAV_TO_DOCK_FAILED")).toBe(false);
        expect(isNotice(undefined)).toBe(false);
    });

    it("handles missing status", () => {
        expect(isMissionActive(undefined, undefined)).toBe(false);
        expect(canStop(undefined, undefined)).toBe(false);
        expect(canStart(undefined, undefined)).toBe(false);
    });

    it("confirms stop only while mowing with blade on", () => {
        expect(stopNeedsConfirm("MOWING", true)).toBe(true);
        expect(stopNeedsConfirm("MANUAL_MOWING", true)).toBe(true);
        expect(stopNeedsConfirm("MOWING", false)).toBe(false);
        expect(stopNeedsConfirm("TRANSIT", true)).toBe(false);
        expect(stopNeedsConfirm("BOUNDARY_EMERGENCY_STOP", undefined)).toBe(false);
    });
});

describe("canUndock", () => {
    it("is offered only while docked", () => {
        expect(canUndock("IDLE_DOCKED")).toBe(true);
        expect(canUndock("CHARGING")).toBe(true);
        for (const n of ["IDLE", "MOWING", "UNDOCKING", "RETURNING_HOME", "EMERGENCY", "RECORDING", undefined, null, ""]) {
            expect(canUndock(n)).toBe(false);
        }
        expect(isDocked("CHARGING")).toBe(true);
        expect(CMD_UNDOCK).toBe(9);
    });
});

describe('recordingKind', () => {
    it('reads the record kind from the sub_state', () => {
        expect(recordingKind('RECORDING', 'drive the path with the joystick')).toBe('path');
        expect(recordingKind('RECORDING', 'saving path')).toBe('path');
        expect(recordingKind('RECORDING', 'drive the boundary with the joystick')).toBe('area');
        expect(recordingKind('IDLE', 'path')).toBeNull();
    });
});
