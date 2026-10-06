import {describe, it, expect} from "vitest";
import {
    PARAM_BINDING_PROFILES,
    hasParamBindings,
    isKeyBound,
    isKeyUsedByRobot,
    paramApplyResults,
    summarizeParamApply,
} from "./paramBindings.ts";
import {ROBOT_PROFILES, getRobotProfile} from "./robotProfiles.ts";

describe("paramBindings", () => {
    it("stock profiles have no table and use every key", () => {
        for (const p of ROBOT_PROFILES.filter((x) => x.value !== "AirseekersTron")) {
            expect(hasParamBindings(p)).toBe(false);
            expect(isKeyBound(p, "mowing_speed")).toBe(false);
            expect(isKeyUsedByRobot(p, "wheel_track")).toBe(true);
        }
        expect(isKeyUsedByRobot(undefined, "anything")).toBe(true);
    });

    it("every table belongs to a known profile", () => {
        for (const id of Object.keys(PARAM_BINDING_PROFILES)) {
            expect(getRobotProfile(id).value).toBe(id);
        }
    });

    it("Tron binds mission, nav, docking and NTRIP keys", () => {
        const tron = getRobotProfile("AirseekersTron");
        for (const key of ["tool_width", "mowing_speed", "transit_speed", "undock_distance",
            "battery_low_percent", "rain_mode", "mow_angle_deg", "dock_approach_distance", "ntrip_host", "ntrip_enabled"]) {
            expect(isKeyBound(tron, key)).toBe(true);
            expect(isKeyUsedByRobot(tron, key)).toBe(true);
        }
        // Accepts a bare id too.
        expect(isKeyBound("AirseekersTron", "mowing_speed")).toBe(true);
    });

    it("Tron flags keys nothing reads, but not GUI keys", () => {
        const tron = getRobotProfile("AirseekersTron");
        expect(isKeyUsedByRobot(tron, "wheel_track")).toBe(false);
        expect(isKeyUsedByRobot(tron, "dock_charging_threshold")).toBe(false);
        expect(isKeyUsedByRobot(tron, "dock_use_charger_detection")).toBe(false);
        expect(isKeyUsedByRobot(tron, "datum_lat")).toBe(true);
        expect(isKeyUsedByRobot(tron, "dock_pose_yaw")).toBe(true);
        expect(isKeyBound(tron, "datum_lat")).toBe(false);
    });

    it("reads and groups the save response", () => {
        expect(paramApplyResults({})).toEqual([]);
        expect(paramApplyResults(null)).toEqual([]);
        const results = paramApplyResults({
            param_bindings: [
                {key: "mowing_speed", param: "/controller_server.FollowCoveragePath.desired_linear_vel", status: "applied", value: 0.3},
                {key: "battery_low_percent", param: "/behavior_tree_node.battery_low_percent", status: "restart_required"},
                {key: "undock_distance", param: "/behavior_tree_node.undock_distance_m", status: "reset"},
                {key: "ntrip_host", param: "/um960_gps_driver.ntrip_host", status: "unavailable", message: "not connected"},
            ],
        });
        const summary = summarizeParamApply(results);
        expect(summary.applied.map((r) => r.key)).toEqual(["mowing_speed"]);
        expect(summary.onRestart.map((r) => r.key)).toEqual(["battery_low_percent", "undock_distance"]);
        expect(summary.failed.map((r) => r.key)).toEqual(["ntrip_host"]);
    });
});
