import {describe, expect, it} from "vitest";
import {
    BATTERY_DEFAULTS,
    computeBatteryPercent,
    computeBatteryPercentOrNull,
    resolveBatteryThresholds,
} from "./battery.ts";
import {AIRSEEKERS_TRON_PROFILE_ID, getRobotProfile} from "../constants/robotProfiles.ts";

describe("computeBatteryPercentOrNull", () => {
    it("is unknown (null) until high-level status arrives, not 0 %", () => {
        expect(computeBatteryPercentOrNull({}, 21.6, {})).toBeNull();
        expect(computeBatteryPercentOrNull(undefined, 21.6, {})).toBeNull();
    });

    it("uses reported percent once status arrived", () => {
        expect(computeBatteryPercentOrNull({battery_percent: 52.4, state_name: "IDLE"}, 21.6, {})).toBe(52);
    });

    it("falls back to voltage after arrival when percent is missing", () => {
        expect(computeBatteryPercentOrNull({state_name: "IDLE"}, 26, {})).toBe(50);
    });
});

describe("battery thresholds from the robot profile", () => {
    const stock = getRobotProfile("YardForce500").battery;
    const tron = getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID).battery;
    const settings = {battery_full_voltage: 28, battery_empty_voltage: 24};

    it("a stock profile behaves exactly as without a profile", () => {
        for (const [pct, v] of [[52.4, 21.6], [0, 26], [undefined, 27], [null, 0], [101, 25]] as const) {
            expect(computeBatteryPercent(pct, v, settings, stock)).toBe(computeBatteryPercent(pct, v, settings));
            expect(computeBatteryPercentOrNull({battery_percent: pct, state_name: "IDLE"}, v, settings, stock))
                .toBe(computeBatteryPercentOrNull({battery_percent: pct, state_name: "IDLE"}, v, settings));
        }
    });

    it("preferReportedPercent trusts a reported 0 %", () => {
        expect(computeBatteryPercentOrNull({battery_percent: 0, state_name: "IDLE"}, 26, settings, tron)).toBe(0);
        // Without the profile 0 means "not reported" and the voltage decides.
        expect(computeBatteryPercentOrNull({battery_percent: 0, state_name: "IDLE"}, 26, settings)).toBe(50);
    });

    it("does not guess from another robot's voltage window", () => {
        expect(computeBatteryPercentOrNull({state_name: "IDLE"}, 26, settings, tron)).toBeNull();
        expect(computeBatteryPercent(undefined, 26, settings, tron)).toBe(0);
    });

    it("uses the profile's own window when it has one", () => {
        const battery = {preferReportedPercent: true, fullV: 30, emptyV: 20};
        expect(computeBatteryPercentOrNull({state_name: "IDLE"}, 25, settings, battery)).toBe(50);
    });

    it("resolves thresholds yaml > profile > defaults", () => {
        expect(resolveBatteryThresholds({battery_full_voltage: "29"}, {preferReportedPercent: false, fullV: 30, emptyV: 20}))
            .toEqual({fullV: 29, emptyV: 20, criticalV: BATTERY_DEFAULTS.criticalVoltage});
        expect(resolveBatteryThresholds({})).toEqual({
            fullV: BATTERY_DEFAULTS.fullVoltage,
            emptyV: BATTERY_DEFAULTS.emptyVoltage,
            criticalV: BATTERY_DEFAULTS.criticalVoltage,
        });
    });
});
