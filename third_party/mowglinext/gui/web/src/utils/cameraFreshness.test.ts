import {describe, expect, it} from "vitest";
import {AIRSEEKERS_TRON_PROFILE_ID, getRobotProfile} from "../constants/robotProfiles.ts";
import {CAMERA_FRESH_MS, cameraFreshness} from "./cameraFreshness.ts";

const cams = getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID).cameras;
const now = 1_000_000;

describe("cameraFreshness", () => {
    it("is undefined for a robot without cameras", () => {
        expect(cameraFreshness(getRobotProfile("YardForce500").cameras, [], now)).toBeUndefined();
    });

    it("reports unknown (nothing reporting) without diagnostics", () => {
        expect(cameraFreshness(cams, undefined, now)).toEqual({total: 3, live: 0, reporting: 0});
    });

    it("counts topic statuses that are OK and recent", () => {
        const statuses = [
            {name: "cams: /left_oa_camera/image_raw topic status", level: 0, receivedAt: now - 1000},
            {name: "cams: /right_oa_camera/image_raw topic status", level: 2, receivedAt: now - 1000},
            {name: "x", hardware_id: "rear", level: 1, receivedAt: now - CAMERA_FRESH_MS - 1},
        ];
        expect(cameraFreshness(cams, statuses, now)).toEqual({total: 3, live: 1, reporting: 3});
    });
});
