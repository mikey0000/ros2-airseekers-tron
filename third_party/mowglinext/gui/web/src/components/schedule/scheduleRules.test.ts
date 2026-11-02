import {describe, expect, it} from "vitest";
import {scheduleRules} from "./scheduleRules.ts";

describe("scheduleRules", () => {
    it("reads rain mode and the low-battery action instead of always saying ON", () => {
        const [rain, battery] = scheduleRules({rain_mode: 0, battery_low_percent: "25", battery_low_action: "stop"});
        expect(rain).toMatchObject({on: false, hintKey: "settingsRain.rainModeIgnoreLabel"});
        expect(battery).toMatchObject({on: false, hintKey: "schedulePage.ruleStopLowHintPct", hintParams: {pct: 25}});
    });

    it("falls back to the template defaults", () => {
        const [rain, battery] = scheduleRules({});
        expect(rain).toMatchObject({on: true, hintKey: "settingsRain.rainModeDockUntilDryLabel"});
        expect(battery).toMatchObject({on: true, hintKey: "schedulePage.ruleAutoDockLowHintPct", hintParams: {pct: 20}});
    });
});
