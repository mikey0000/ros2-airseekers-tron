import {describe, expect, it} from "vitest";
import {
    AREA_SETTINGS_DEFAULTS, angleFromPoint, effectiveAreaSettings, invalidAreaSettingKey,
    normaliseAngle, parseStringMsgJson, sanitizeAreaSettings,
} from "./areaSettings.ts";

describe("areaSettings", () => {
    it("layers built-in < robot defaults < area overrides", () => {
        const eff = effectiveAreaSettings({cutter_height_mm: 60, repeat: 2}, {cutter_height_mm: 40});
        expect(eff.cutter_height_mm).toBe(40);
        expect(eff.repeat).toBe(2);
        expect(eff.path_mode).toBe(AREA_SETTINGS_DEFAULTS.path_mode);
    });

    it("drops unknown keys and wrong types", () => {
        expect(sanitizeAreaSettings({bogus: 1, repeat: "3", path_mode: "random", edge_first: false}))
            .toEqual({edge_first: false});
        expect(sanitizeAreaSettings(null)).toEqual({});
    });

    it("validates ranges like the backend", () => {
        expect(invalidAreaSettingKey({...AREA_SETTINGS_DEFAULTS})).toBeNull();
        expect(invalidAreaSettingKey({cutter_height_mm: 95})).toBe("cutter_height_mm");
        expect(invalidAreaSettingKey({cutter_height_mm: 45.5})).toBe("cutter_height_mm");
        expect(invalidAreaSettingKey({mow_angle_deg: -0.5})).toBe("mow_angle_deg");
        expect(invalidAreaSettingKey({mow_angle_deg: 135})).toBeNull();
        expect(invalidAreaSettingKey({repeat: 0})).toBe("repeat");
        expect(invalidAreaSettingKey({cut_speed_mps: 0.25})).toBeNull();
    });

    it("parses std_msgs/String JSON", () => {
        expect(parseStringMsgJson({data: '{"a":1}'})).toEqual({a: 1});
        expect(parseStringMsgJson({data: "nope"})).toBeUndefined();
        expect(parseStringMsgJson(undefined)).toBeUndefined();
    });

    it("maps pointer offsets to compass angles", () => {
        expect(angleFromPoint(0, -1)).toBe(0);
        expect(angleFromPoint(1, 0)).toBe(90);
        expect(angleFromPoint(0, 1)).toBe(180);
        expect(angleFromPoint(-1, 0)).toBe(270);
        expect(normaliseAngle(-30)).toBe(330);
    });
});

import {buildAreaPatch, overriddenKeys} from "./areaSettings.ts";

describe("area patches", () => {
    it("overrides changed keys and resets the rest", () => {
        const defaults = {...AREA_SETTINGS_DEFAULTS};
        const patch = buildAreaPatch({...defaults, cutter_height_mm: 70}, defaults);
        expect(patch.cutter_height_mm).toBe(70);
        expect(patch.repeat).toBeNull();
    });
    it("prefers the topic's override list", () => {
        const d = {...AREA_SETTINGS_DEFAULTS};
        expect([...overriddenKeys({...d, repeat: 2}, d)]).toEqual(["repeat"]);
        expect([...overriddenKeys({...d, repeat: 2}, d, {cutter_height_mm: 50})]).toEqual(["cutter_height_mm"]);
    });
});
