import {describe, it, expect, vi, beforeEach} from "vitest";
import {render, screen} from "@testing-library/react";
import {DrivingCameraPip} from "./DrivingCameraPip.tsx";
import {DEFAULT_PREFS} from "../hooks/useDrivingCamera.ts";
import type {CamerasResponse} from "../../../hooks/useCameras.ts";

const cams = vi.hoisted(() => ({data: null as CamerasResponse | null}));
vi.mock("../../../hooks/useCameras.ts", async (orig) => ({
    ...(await orig<typeof import("../../../hooks/useCameras.ts")>()),
    useCameras: () => ({data: cams.data, loading: false, error: false, reload: () => undefined}),
}));
vi.mock("../../../hooks/useCameraHealth.ts", () => ({useCameraHealth: () => undefined}));

const mk = (id: string) => ({id, label: id, topic: `/${id}`, streamUrl: `/api/cameras/${id}/stream`, snapshotUrl: `/x`});
const RESP: CamerasResponse = {
    available: true, defaults: {quality: 50, fps: 5, maxFps: 15},
    cameras: [mk("left_oa"), mk("rear"), mk("front_right")],
};

const renderPip = (over: Partial<Parameters<typeof DrivingCameraPip>[0]> = {}) => render(
    <DrivingCameraPip manualMode={true} reversing={false} drivingCamera="front_right"
                      prefs={DEFAULT_PREFS} onPrefsChange={vi.fn()} {...over}/>);

describe("DrivingCameraPip", () => {
    beforeEach(() => { cams.data = RESP; });

    it("renders nothing outside manual mode", () => {
        renderPip({manualMode: false});
        expect(screen.queryByTestId("drive-camera-pip")).toBeNull();
    });
    it("streams one MJPEG of the driving camera in manual mode", () => {
        renderPip();
        const imgs = screen.getAllByTestId("drive-pip-stream");
        expect(imgs).toHaveLength(1);
        expect(imgs[0].getAttribute("src")).toMatch(/^\/api\/cameras\/front_right\/stream\?/);
    });
    it("switches to rear while reversing", () => {
        renderPip({reversing: true});
        expect(screen.getByTestId("drive-pip-stream").getAttribute("src")).toContain("/rear/");
    });
    it("closes the stream when collapsed", () => {
        renderPip({prefs: {...DEFAULT_PREFS, collapsed: true}});
        expect(screen.getByTestId("drive-camera-pip")).toBeTruthy();
        expect(screen.queryByTestId("drive-pip-stream")).toBeNull();
    });
    it("renders nothing on a robot without cameras", () => {
        cams.data = {...RESP, cameras: []};
        renderPip();
        expect(screen.queryByTestId("drive-camera-pip")).toBeNull();
    });
});
