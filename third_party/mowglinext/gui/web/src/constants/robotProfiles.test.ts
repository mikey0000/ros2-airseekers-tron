import {describe, expect, it} from "vitest";
import {MOWER_MODELS} from "./mowerModels.ts";
import {
    AIRSEEKERS_TRON_PROFILE_ID,
    DEFAULT_ROBOT_PROFILE,
    FEATURE_IDS,
    ROBOT_PROFILES,
    UPSTREAM_FEATURES,
    findRobotProfile,
    getRobotProfile,
    hasFeature,
    isSettingKeyHidden,
} from "./robotProfiles.ts";
import en from "../i18n/locales/en.json";

describe("robot profiles", () => {
    it("wraps every MowerModel preset without changing it", () => {
        for (const model of MOWER_MODELS) {
            const profile = findRobotProfile(model.value)!;
            expect(profile).toBeDefined();
            expect(profile.label).toBe(model.label);
            expect(profile.defaults).toEqual(model.defaults);
        }
    });

    it("gives every stock model the upstream UI", () => {
        for (const model of MOWER_MODELS.filter((m) => m.value !== AIRSEEKERS_TRON_PROFILE_ID)) {
            const profile = getRobotProfile(model.value);
            expect(profile.features).toEqual(UPSTREAM_FEATURES);
            expect(profile.perception).toBe("lidar");
            expect(profile.docking).toBe("charger_contacts");
            expect(profile.cameras).toEqual([]);
            expect(profile.hiddenSettingKeys).toEqual([]);
        }
    });

    it("keeps the stock features on and the additive ones off", () => {
        const yf = getRobotProfile("YardForce500");
        expect(hasFeature(yf, "stm32_firmware")).toBe(true);
        expect(hasFeature(yf, "docker_host")).toBe(true);
        expect(hasFeature(yf, "cameras")).toBe(false);
        expect(hasFeature(yf, "lora_corrections")).toBe(false);
        expect(hasFeature(yf, "area_settings")).toBe(false);
        expect(hasFeature(yf, "manual_blade_two_step")).toBe(false);
        expect(hasFeature(getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID), "manual_blade_two_step")).toBe(true);
    });

    it("takes battery thresholds from the preset", () => {
        expect(getRobotProfile("Sabo").battery).toEqual({
            fullV: 28.5, emptyV: 21.0, criticalV: 20.0, preferReportedPercent: false,
        });
    });

    it("falls back to the stock profile for unknown or empty ids", () => {
        expect(findRobotProfile("NoSuchRobot")).toBeUndefined();
        expect(findRobotProfile("")).toBeUndefined();
        expect(getRobotProfile("NoSuchRobot")).toBe(DEFAULT_ROBOT_PROFILE);
        expect(getRobotProfile(undefined).value).toBe("YardForce500");
    });

    it("describes the Airseekers Tron", () => {
        const tron = getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID);
        expect(tron.value).toBe("AirseekersTron");
        expect(MOWER_MODELS.map((m) => m.value)).toContain("AirseekersTron");
        expect(tron.defaults).toBe(MOWER_MODELS.find((m) => m.value === "AirseekersTron")!.defaults);
        expect(tron.label).toBe("mowerModels.AirseekersTron.label");
        for (const off of ["stm32_firmware", "gnss_sidecar", "docker_admin", "drive_tuning", "lidar",
            "dock_calibration", "fusion_graph", "imu_yaw_calibration", "status_leds"] as const) {
            expect(hasFeature(tron, off)).toBe(false);
        }
        expect(hasFeature(tron, "docker_host")).toBe(true);
        expect(hasFeature(tron, "cameras")).toBe(true);
        expect(hasFeature(tron, "lora_corrections")).toBe(true);
        expect(hasFeature(tron, "area_settings")).toBe(true);
        expect(hasFeature(tron, "straight_driving")).toBe(true);
        expect(tron.perception).toBe("camera");
        expect(tron.docking).toBe("vision_marker");
        expect(tron.battery.preferReportedPercent).toBe(true);
        expect(tron.battery.fullV).toBeUndefined();
        expect(tron.cameras.map((c) => c.topic)).toEqual([
            "/left_oa_camera/image_raw", "/right_oa_camera/image_raw", "/rear_camera/image_raw",
            "/vio/left/image_raw", "/vio/right/image_raw",
        ]);
        expect(tron.cameras[0].annotatedTopic).toBe("/left_oa_camera/image_annotated");
        expect(tron.defaults.wheel_track).toBe(0.48);
        expect(tron.defaults.tool_width).toBe(0.2);
    });

    it("lists the Status fields a robot cannot measure", () => {
        expect(getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID).unmeasuredStatusFields).toEqual(
            expect.arrayContaining(["mower_esc_temperature", "raspberry_pi_power"]));
        expect(getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID).unmeasuredStatusFields)
            .not.toContain("mower_motor_temperature");
        for (const profile of ROBOT_PROFILES.filter((p) => p.value !== AIRSEEKERS_TRON_PROFILE_ID)) {
            expect(profile.unmeasuredStatusFields).toEqual([]);
        }
    });

    it("defines every feature on every profile", () => {
        for (const profile of ROBOT_PROFILES) {
            expect(Object.keys(profile.features).sort()).toEqual([...FEATURE_IDS].sort());
        }
    });

    it("matches hidden setting keys exactly or by prefix", () => {
        const tron = getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID);
        expect(isSettingKeyHidden(tron, "ticks_per_meter")).toBe(true);
        expect(isSettingKeyHidden(tron, "wheel_pid_kp")).toBe(true);
        expect(isSettingKeyHidden(tron, "caster_track")).toBe(true);
        expect(isSettingKeyHidden(tron, "wheel_radius")).toBe(false);
        expect(isSettingKeyHidden(DEFAULT_ROBOT_PROFILE, "ticks_per_meter")).toBe(false);
    });

    it("has translations for every profile label and camera", () => {
        const lookup = (key: string) =>
            key.split(".").reduce<unknown>((acc, part) => (acc as Record<string, unknown>)?.[part], en);
        for (const profile of ROBOT_PROFILES) {
            expect(typeof lookup(profile.label)).toBe("string");
            expect(typeof lookup(profile.description)).toBe("string");
            for (const camera of profile.cameras) {
                expect(typeof lookup(camera.label)).toBe("string");
            }
        }
    });
});
