import {describe, expect, it} from "vitest";
import {MOWER_MODELS} from "./mowerModels.ts";
import {
    AIRSEEKERS_TRON_PROFILE_ID,
    FEATURE_IDS,
    getRobotProfile,
} from "./robotProfiles.ts";
import {
    GATE_IDS,
    PROFILE_GATES,
    defaultMqttTopicPrefix,
    hiddenGates,
    isGateId,
    isGateVisible,
    isProfileSettingKeyHidden,
} from "./profileGates.ts";
import {SECTION_DEFINITIONS} from "../hooks/useSettingsManager.ts";

// The build-time list this module replaced (web/src/tronFeatures.ts), minus
// "/onboarding": the wizard is profile-aware now and shows for every robot.
const TRON_HIDDEN_PAGES_BEFORE = [
    "settings:updates",
    "settings:remote_access",
    "settings:drive_motor",
    "feature:firmware_flash",
    "feature:gnss_configurator",
    "feature:host_updater",
    "feature:rosbag",
];

// Gates added with the profile (T23/T25/T37/T41): everything Tron lacks
// beyond the old trim.
const NEW_TRON_GATES = [
    "settings:leds",
    "feature:firmware_debug",
    "feature:containers",
    "feature:restart_ros2",
    "feature:lidar",
    "feature:fusion_graph",
    "feature:imu_yaw_calibration",
    "feature:dock_calibration",
];

// Gates for UI no stock robot has ever shown (additive features).
const ADDITIVE_GATES = ["/perception"];

const STOCK_MODELS = MOWER_MODELS.filter((m) => m.value !== AIRSEEKERS_TRON_PROFILE_ID);
const tron = getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID);

describe("profile gates", () => {
    it("only reference known features", () => {
        for (const needs of Object.values(PROFILE_GATES)) {
            for (const f of needs) expect(FEATURE_IDS).toContain(f);
        }
    });

    it("hide nothing a stock robot shows today", () => {
        for (const model of STOCK_MODELS) {
            expect(hiddenGates(getRobotProfile(model.value))).toEqual(ADDITIVE_GATES);
        }
    });

    it("reproduce today's Tron trim plus the new gates", () => {
        expect(new Set(hiddenGates(tron))).toEqual(new Set([...TRON_HIDDEN_PAGES_BEFORE, ...NEW_TRON_GATES]));
        expect(isGateVisible(tron, "/perception")).toBe(true);
    });

    it("keeps every old Tron id as a gate", () => {
        for (const id of TRON_HIDDEN_PAGES_BEFORE) expect(isGateId(id)).toBe(true);
        expect(GATE_IDS.length).toBe(Object.keys(PROFILE_GATES).length);
    });

    it("treats unknown ids as visible", () => {
        expect(isGateVisible(tron, "/map")).toBe(true);
        expect(isGateId("/map")).toBe(false);
    });
});

describe("profile setting keys", () => {
    const allSectionKeys = SECTION_DEFINITIONS.flatMap((s) => s.keys);

    it("hides no settings field on a stock robot", () => {
        for (const model of STOCK_MODELS) {
            const p = getRobotProfile(model.value);
            expect(allSectionKeys.filter((k) => isProfileSettingKeyHidden(p, k))).toEqual([]);
        }
    });

    it("hides LiDAR and STM32-odometry keys on Tron", () => {
        for (const key of [
            "lidar_enabled", "lidar_x", "lidar_yaw", "use_lidar_map_anchor", "lidar_anchor_shadow_mode",
            "ticks_per_meter", "deadband_pwm", "wheel_pid_kp", "caster_radius", "caster_track", "chassis_mass_kg",
        ]) {
            expect(isProfileSettingKeyHidden(tron, key), key).toBe(true);
        }
        for (const key of ["wheel_radius", "wheel_track", "imu_yaw", "gps_x", "rain_mode", "mqtt_topic_prefix"]) {
            expect(isProfileSettingKeyHidden(tron, key), key).toBe(false);
        }
    });
});

describe("defaultMqttTopicPrefix", () => {
    it("keeps the upstream prefix for stock robots", () => {
        for (const model of STOCK_MODELS) {
            expect(defaultMqttTopicPrefix(getRobotProfile(model.value))).toBe("mowgli");
        }
    });

    it("uses the profile id elsewhere", () => {
        expect(defaultMqttTopicPrefix(tron)).toBe("airseekerstron");
    });
});
