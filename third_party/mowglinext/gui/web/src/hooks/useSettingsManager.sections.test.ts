import {describe, expect, it} from "vitest";
import {AIRSEEKERS_TRON_PROFILE_ID, getRobotProfile} from "../constants/robotProfiles.ts";
import {SECTION_DEFINITIONS, visibleSections} from "./useSettingsManager.ts";

const ids = (sections: {id: string}[]) => sections.map((s) => s.id);

describe("Settings sections per robot profile", () => {
    it("YardForce500 lists every section, as before profiles", () => {
        // Additive Tron-only sections (cmd_vel_slew straight-line driving, host Wi-Fi setup).
        expect(ids(visibleSections(getRobotProfile("YardForce500")))).toEqual(
            ids(SECTION_DEFINITIONS).filter((id) => id !== "straight_driving" && id !== "network"));
    });

    it("shows straight driving and network on Tron only", () => {
        expect(ids(visibleSections(getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID)))).toContain("straight_driving");
        expect(ids(visibleSections(getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID)))).toContain("network");
    });

    it("Tron hides today's trim, the LED section and the fields-only navigation section", () => {
        const hidden = ids(SECTION_DEFINITIONS).filter(
            (id) => !ids(visibleSections(getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID))).includes(id),
        );
        // Remote access stays: its HTTPS card works everywhere (Tailscale part gated).
        // Navigation renders only its six fields and Tron reads none of them.
        expect(hidden).toEqual(["updates", "drive_motor", "navigation", "leds"]);
    });

    it("keeps a fields-only section while the profile is loading", () => {
        expect(ids(visibleSections(null))).toContain("navigation");
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
