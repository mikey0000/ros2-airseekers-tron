import {describe, expect, it} from "vitest";
import type {CameraInfo} from "../../hooks/useCameras.ts";
import {buildStreamUrl, cameraForFrame} from "./streamUrl.ts";

const defaults = {quality: 50, fps: 5, maxFps: 10};
const cam = (o: Partial<CameraInfo> = {}): CameraInfo => ({
    id: "left_oa", label: "Left", topic: "/left_oa_camera/image_raw",
    streamUrl: "/api/cameras/left_oa/stream", snapshotUrl: "/api/cameras/left_oa/snapshot", ...o,
});

describe("buildStreamUrl", () => {
    it("appends quality and fps", () => {
        expect(buildStreamUrl(cam(), {variant: "raw", quality: 50, fps: 5}, defaults))
            .toBe("/api/cameras/left_oa/stream?quality=50&fps=5");
    });
    it("clamps fps to maxFps and quality to 1..100", () => {
        const u = buildStreamUrl(cam(), {variant: "raw", quality: 500, fps: 99}, defaults);
        expect(u).toContain("fps=10");
        expect(u).toContain("quality=100");
    });
    it("only sends variant=annotated when an annotated topic exists", () => {
        expect(buildStreamUrl(cam(), {variant: "annotated", quality: 50, fps: 5}, defaults)).not.toContain("variant");
        expect(buildStreamUrl(cam({annotatedTopic: "/x"}), {variant: "annotated", quality: 50, fps: 5}, defaults))
            .toContain("variant=annotated");
    });
});

describe("cameraForFrame", () => {
    it("matches the frame_id against the topic namespace", () => {
        const cams = [cam(), cam({id: "rear", topic: "/rear_camera/image_raw"})];
        expect(cameraForFrame(cams, "rear_camera")?.id).toBe("rear");
        expect(cameraForFrame(cams, "nope")).toBeUndefined();
        expect(cameraForFrame(cams, undefined)).toBeUndefined();
    });
});
