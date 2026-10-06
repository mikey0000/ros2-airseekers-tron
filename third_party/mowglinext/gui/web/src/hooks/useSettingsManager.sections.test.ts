import {describe, expect, it} from "vitest";
import {AIRSEEKERS_TRON_PROFILE_ID, getRobotProfile} from "../constants/robotProfiles.ts";
import {SECTION_DEFINITIONS, visibleSections} from "./useSettingsManager.ts";

const ids = (sections: {id: string}[]) => sections.map((s) => s.id);

describe("Settings sections per robot profile", () => {
    it("YardForce500 lists every section, as before profiles", () => {
        expect(ids(visibleSections(getRobotProfile("YardForce500")))).toEqual(ids(SECTION_DEFINITIONS));
    });

    it("Tron hides today's trim plus the LED section", () => {
        const hidden = ids(SECTION_DEFINITIONS).filter(
            (id) => !ids(visibleSections(getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID))).includes(id),
        );
        expect(hidden).toEqual(["updates", "drive_motor", "leds", "remote_access"]);
    });

    it("keeps rain and MQTT on Tron", () => {
        const tron = ids(visibleSections(getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID)));
        expect(tron).toContain("rain");
        expect(tron).toContain("mqtt");
    });

    it("lists no gated section while the profile is loading", () => {
        expect(ids(visibleSections(null))).not.toContain("updates");
        expect(ids(visibleSections(null))).toContain("hardware");
    });
});
