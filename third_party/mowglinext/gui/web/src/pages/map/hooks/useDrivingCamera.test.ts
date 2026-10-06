import {describe, it, expect, beforeEach} from "vitest";
import {act, renderHook} from "@testing-library/react";
import type {CameraInfo} from "../../../hooks/useCameras.ts";
import {
    chooseCamera, defaultDrivingCamera, DEFAULT_PREFS, DRIVE_PIP_STORAGE_KEY, isReversing, loadPrefs, useDrivePipPrefs,
} from "./useDrivingCamera.ts";
import {getRobotProfile, AIRSEEKERS_TRON_PROFILE_ID} from "../../../constants/robotProfiles.ts";

const cam = (id: string): CameraInfo => ({id, label: id, topic: `/${id}`, streamUrl: `/s/${id}`, snapshotUrl: `/f/${id}`});
const CAMS = ["left_oa", "right_oa", "rear", "front_left", "front_right"].map(cam);

describe("driving camera choice", () => {
    it("defaults to the profile's driving camera (Tron: right_oa)", () => {
        const tron = getRobotProfile(AIRSEEKERS_TRON_PROFILE_ID);
        expect(tron.drivingCamera).toBe("right_oa");
        expect(defaultDrivingCamera(CAMS, tron.drivingCamera)?.id).toBe("right_oa");
    });
    it("falls back to a front camera, then the first; none without cameras", () => {
        expect(defaultDrivingCamera(CAMS)?.id).toBe("front_left");
        expect(defaultDrivingCamera([cam("a"), cam("b")])?.id).toBe("a");
        expect(defaultDrivingCamera([])).toBeUndefined();
    });
    it("honours the operator's pick and ignores a stale one", () => {
        const base = {drivingCamera: "front_right", reversing: false, autoReverse: true};
        expect(chooseCamera(CAMS, {...base, selectedId: "left_oa"})?.id).toBe("left_oa");
        expect(chooseCamera(CAMS, {...base, selectedId: "gone"})?.id).toBe("front_right");
    });
    it("auto-switches to rear while reversing and back going forward", () => {
        const base = {selectedId: "left_oa", drivingCamera: "front_right", autoReverse: true};
        expect(chooseCamera(CAMS, {...base, reversing: true})?.id).toBe("rear");
        expect(chooseCamera(CAMS, {...base, reversing: false})?.id).toBe("left_oa");
        expect(chooseCamera(CAMS, {...base, reversing: true, autoReverse: false})?.id).toBe("left_oa");
        expect(chooseCamera(CAMS.filter((c) => c.id !== "rear"), {...base, reversing: true})?.id).toBe("left_oa");
    });
    it("treats only clear backward stick as reversing", () => {
        expect(isReversing(-0.5)).toBe(true);
        expect(isReversing(-0.05)).toBe(false);
        expect(isReversing(0.8)).toBe(false);
        expect(isReversing(null)).toBe(false);
    });
});

describe("PiP prefs persistence", () => {
    beforeEach(() => localStorage.clear());
    it("defaults when empty or corrupt", () => {
        expect(loadPrefs()).toEqual(DEFAULT_PREFS);
        localStorage.setItem(DRIVE_PIP_STORAGE_KEY, "{nope");
        expect(loadPrefs()).toEqual(DEFAULT_PREFS);
    });
    it("round-trips camera, toggles and rect through localStorage", () => {
        const {result, unmount} = renderHook(() => useDrivePipPrefs());
        act(() => result.current.update({cameraId: "rear", annotated: true, rect: {x: 5, y: 6, w: 300, h: 200}}));
        unmount();
        const again = renderHook(() => useDrivePipPrefs());
        expect(again.result.current.prefs).toMatchObject({cameraId: "rear", annotated: true, autoReverse: true,
            rect: {x: 5, y: 6, w: 300, h: 200}});
    });
});
