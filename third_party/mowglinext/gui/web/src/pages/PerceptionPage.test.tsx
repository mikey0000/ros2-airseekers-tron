import {beforeEach, describe, expect, it, vi} from "vitest";
import {fireEvent, render, screen} from "@testing-library/react";

const cameras = vi.hoisted(() => ({value: null as unknown}));
const detections = vi.hoisted(() => ({
    value: {data: {count: 0, classes: [] as string[], max_score: 0} as Record<string, unknown>, lastMessageAt: null as number | null},
    close: false,
    enabledArgs: [] as boolean[],
}));
const mobile = vi.hoisted(() => ({value: false}));

vi.mock("../hooks/useCameras.ts", () => ({
    useCameras: () => ({data: cameras.value, loading: false, error: false, reload: vi.fn()}),
}));
vi.mock("../hooks/useDetections.ts", () => ({
    useDetections: (en: boolean) => {
        detections.enabledArgs.push(en);
        return detections.value;
    },
    useVisionObstacleClose: () => detections.close,
}));
vi.mock("../hooks/useIsMobile.ts", () => ({useIsMobile: () => mobile.value}));

import PerceptionPage from "./PerceptionPage.tsx";

const mk = (id: string, extra: object = {}) => ({
    id, label: id, topic: `/${id}_camera/image_raw`,
    streamUrl: `/api/cameras/${id}/stream`, snapshotUrl: `/api/cameras/${id}/snapshot`, ...extra,
});
const ok = () => ({
    available: true,
    cameras: [mk("left_oa", {annotatedTopic: "/ai/ann"}), mk("right_oa"), mk("rear")],
    defaults: {quality: 50, fps: 5, maxFps: 10},
});

describe("PerceptionPage", () => {
    beforeEach(() => {
        cameras.value = ok();
        mobile.value = false;
        detections.close = false;
        detections.value = {data: {count: 0, classes: [], max_score: 0}, lastMessageAt: null};
        detections.enabledArgs = [];
    });

    it("shows the hint and no images when unavailable", () => {
        cameras.value = {...ok(), available: false, hint: "start web_video_server"};
        render(<PerceptionPage/>);
        expect(screen.getByTestId("perception-hint")).toHaveTextContent("start web_video_server");
        expect(screen.queryByRole("img", {name: "left_oa"})).not.toBeInTheDocument();
        expect(detections.enabledArgs.every((e) => e === false)).toBe(true);
    });

    it("streams all three tiles with default fps/quality on desktop", () => {
        render(<PerceptionPage/>);
        const img = screen.getByAltText("left_oa") as HTMLImageElement;
        expect(img.getAttribute("src")).toBe("/api/cameras/left_oa/stream?quality=50&fps=5");
        expect(screen.getAllByRole("img").filter((i) => i.tagName === "IMG")).toHaveLength(3);
    });

    it("streams one camera on mobile", () => {
        mobile.value = true;
        render(<PerceptionPage/>);
        expect(document.querySelectorAll("img")).toHaveLength(1);
    });

    it("pause removes every <img>", () => {
        render(<PerceptionPage/>);
        fireEvent.click(screen.getByRole("switch", {name: "Pause streams"}));
        expect(document.querySelectorAll("img")).toHaveLength(0);
    });

    it("switching to annotated changes the stream URL", () => {
        render(<PerceptionPage/>);
        fireEvent.click(screen.getByText("Annotated"));
        expect(screen.getByAltText("left_oa").getAttribute("src")).toContain("variant=annotated");
    });

    it("renders detection summary and obstacle badge", () => {
        detections.close = true;
        detections.value = {data: {count: 2, classes: ["person"], max_score: 0.81, frame_id: "rear_camera"}, lastMessageAt: 1};
        render(<PerceptionPage/>);
        expect(screen.getByTestId("obstacle-close-badge")).toBeInTheDocument();
        expect(screen.getByTestId("det-count")).toHaveTextContent("2");
        expect(screen.getByTestId("det-score")).toHaveTextContent("81%");
        expect(screen.getByTestId("det-camera")).toHaveTextContent("rear");
    });
});
