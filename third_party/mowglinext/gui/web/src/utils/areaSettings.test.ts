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

import {extractSlopeDerived} from "./areaSettings.ts";

describe("blade_policy", () => {
    it("accepts continuous/conservative and rejects others", () => {
        expect(sanitizeAreaSettings({blade_policy: "conservative"})).toEqual({blade_policy: "conservative"});
        expect(sanitizeAreaSettings({blade_policy: "always" as never})).toEqual({});
        expect(invalidAreaSettingKey({blade_policy: "continuous"})).toBeNull();
        expect(invalidAreaSettingKey({blade_policy: "always" as never})).toBe("blade_policy");
    });
});

describe("transit_variation", () => {
    it("accepts none/lanes/perimeter/mixed and rejects others", () => {
        expect(sanitizeAreaSettings({transit_variation: "perimeter"})).toEqual({transit_variation: "perimeter"});
        expect(sanitizeAreaSettings({transit_variation: "sometimes" as never})).toEqual({});
        expect(invalidAreaSettingKey({transit_variation: "mixed"})).toBeNull();
        expect(invalidAreaSettingKey({transit_variation: "sometimes" as never})).toBe("transit_variation");
    });
});

describe("slope settings", () => {
    it("accepts slope_mode / slope_contour_above_deg and drops derived keys", () => {
        expect(sanitizeAreaSettings({slope_mode: "contour", slope_contour_above_deg: 12,
            slope_mow_angle_deg: 30, slope_angle_why: "x"}))
            .toEqual({slope_mode: "contour", slope_contour_above_deg: 12});
        expect(sanitizeAreaSettings({slope_mode: "diagonal"})).toEqual({});
        expect(invalidAreaSettingKey({slope_mode: "updown"})).toBeNull();
        expect(invalidAreaSettingKey({slope_contour_above_deg: 1})).toBe("slope_contour_above_deg");
        expect(invalidAreaSettingKey({slope_contour_above_deg: 31})).toBe("slope_contour_above_deg");
    });
    it("patches never carry the derived keys", () => {
        const d = {...AREA_SETTINGS_DEFAULTS};
        const patch = buildAreaPatch({...d, slope_mode: "auto"}, d) as Record<string, unknown>;
        expect(patch.slope_mode).toBe("auto");
        expect(patch.slope_contour_above_deg).toBeNull();
        expect("slope_mow_angle_deg" in patch).toBe(false);
        expect("slope_angle_why" in patch).toBe(false);
    });
    it("extracts the read-only slope angle", () => {
        expect(extractSlopeDerived({slope_mow_angle_deg: 80, slope_angle_why: "contour"}))
            .toEqual({slope_mow_angle_deg: 80, slope_angle_why: "contour"});
        expect(extractSlopeDerived({slope_mow_angle_deg: null, slope_angle_why: "no data"}))
            .toEqual({slope_mow_angle_deg: null, slope_angle_why: "no data"});
        expect(extractSlopeDerived({repeat: 1})).toBeNull();
    });
});

import {AREA_SETTINGS_DEFAULTS as D, buildDefaultsBody} from "./areaSettings.ts";

describe("buildDefaultsBody", () => {
    it("leaves an unchanged swath width out (never pins the disc-derived standard)", () => {
        const body = buildDefaultsBody({...D, swath_width_m: 0.29, cutter_height_mm: 60}, {...D, swath_width_m: 0.29}, 0.29);
        expect("swath_width_m" in body).toBe(false);
        expect(body.cutter_height_mm).toBe(60);
    });
    it("sends a changed swath width, null when set back to the standard", () => {
        expect(buildDefaultsBody({...D, swath_width_m: 0.2}, {...D, swath_width_m: 0.29}, 0.29).swath_width_m).toBe(0.2);
        expect(buildDefaultsBody({...D, swath_width_m: 0.29}, {...D, swath_width_m: 0.2}, 0.29).swath_width_m).toBeNull();
        expect(buildDefaultsBody({...D, swath_width_m: 0.29}, {...D, swath_width_m: 0.2}).swath_width_m).toBe(0.29);
    });
});
