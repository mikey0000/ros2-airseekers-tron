import {beforeEach, describe, expect, it, vi} from "vitest";
import {act, fireEvent, render, screen} from "@testing-library/react";

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
    cameras: [mk("left_oa", {annotatedTopic: "/ai/ann", sourceWidth: 960, sourceHeight: 540}), mk("right_oa"), mk("rear")],
    defaults: {quality: 50, fps: 5, maxFps: 10},
});

describe("PerceptionPage", () => {
    beforeEach(() => {
        cameras.value = ok();
        mobile.value = false;
        detections.close = false;
        detections.value = {data: {count: 0, classes: [], max_score: 0}, lastMessageAt: null};
        detections.enabledArgs = [];
        window.localStorage.clear();
    });

    const startAll = () => {
        for (const b of screen.queryAllByRole("button", {name: "Start this stream"})) fireEvent.click(b);
    };

    it("shows the hint and no images when unavailable", () => {
        cameras.value = {...ok(), available: false, hint: "start web_video_server"};
        render(<PerceptionPage/>);
        expect(screen.getByTestId("perception-hint")).toHaveTextContent("start web_video_server");
        expect(screen.queryByRole("img", {name: "left_oa"})).not.toBeInTheDocument();
        expect(detections.enabledArgs.every((e) => e === false)).toBe(true);
    });

    it("streams only the first camera on open; others start on demand", () => {
        render(<PerceptionPage/>);
        expect(document.querySelectorAll("img")).toHaveLength(1);
        expect(screen.getByAltText("left_oa")).toBeInTheDocument();
        expect(screen.getAllByRole("button", {name: "Start this stream"})).toHaveLength(2);
        fireEvent.click(screen.getAllByRole("button", {name: "Start this stream"})[1]);  // rear
        expect(document.querySelectorAll("img")).toHaveLength(2);
        expect(screen.getByAltText("rear").getAttribute("src")).toBe("/api/cameras/rear/stream?quality=50&fps=5");
    });

    it("remembers the started cameras across visits", () => {
        const {unmount} = render(<PerceptionPage/>);
        fireEvent.click(screen.getByRole("button", {name: "Stop this stream"}));          // left_oa off
        fireEvent.click(screen.getAllByRole("button", {name: "Start this stream"})[1]); // right_oa on
        unmount();
        render(<PerceptionPage/>);
        expect([...document.querySelectorAll("img")].map((i) => i.getAttribute("alt"))).toEqual(["right_oa"]);
    });

    it("streams every started tile, annotated by default when available", () => {
        render(<PerceptionPage/>);
        startAll();
        const img = screen.getByAltText("left_oa") as HTMLImageElement;
        expect(img.getAttribute("src")).toBe("/api/cameras/left_oa/stream?variant=annotated&quality=50&fps=5");
        expect(screen.getByAltText("right_oa").getAttribute("src")).toBe("/api/cameras/right_oa/stream?quality=50&fps=5");
        expect(screen.getAllByRole("img").filter((i) => i.tagName === "IMG")).toHaveLength(3);
    });

    it("stops every stream in a hidden (background) tab on desktop and resumes when shown", () => {
        const hidden = vi.spyOn(document, "hidden", "get").mockReturnValue(false);
        try {
            render(<PerceptionPage/>);
            startAll();
            expect(document.querySelectorAll("img")).toHaveLength(3);
            hidden.mockReturnValue(true);
            act(() => { document.dispatchEvent(new Event("visibilitychange")); });
            expect(document.querySelectorAll("img")).toHaveLength(0);
            expect(detections.enabledArgs[detections.enabledArgs.length - 1]).toBe(false);
            hidden.mockReturnValue(false);
            act(() => { document.dispatchEvent(new Event("visibilitychange")); });
            expect(document.querySelectorAll("img")).toHaveLength(3);
        } finally {
            hidden.mockRestore();
        }
    });

    it("releases the MJPEG connection when a tile stops", () => {
        render(<PerceptionPage/>);
        const img = screen.getByAltText("left_oa") as HTMLImageElement;
        fireEvent.click(screen.getByRole("button", {name: "Stop this stream"}));
        expect(img.isConnected).toBe(false);
        expect(img.getAttribute("src")).toMatch(/^data:image\/gif/);   // request aborted
    });

    it("pauses a hidden tab on mobile", () => {
        mobile.value = true;
        const hidden = vi.spyOn(document, "hidden", "get").mockReturnValue(true);
        try {
            render(<PerceptionPage/>);
            expect(document.querySelectorAll("img")).toHaveLength(0);
        } finally {
            hidden.mockRestore();
        }
    });

    it("per-tile stop removes only that stream", () => {
        render(<PerceptionPage/>);
        startAll();
        fireEvent.click(screen.getAllByRole("button", {name: "Stop this stream"})[1]);
        expect(document.querySelectorAll("img")).toHaveLength(2);
        expect(screen.queryByAltText("right_oa")).not.toBeInTheDocument();
    });

    it("streams at most three MJPEG tiles and polls snapshots for the rest", () => {
        const fetchSpy = vi.spyOn(globalThis, "fetch").mockImplementation(() => new Promise(() => undefined));
        cameras.value = {...ok(), cameras: [...ok().cameras, mk("front_left"), mk("front_right")]};
        render(<PerceptionPage/>);
        startAll();
        const srcs = [...document.querySelectorAll("img")].map((i) => i.getAttribute("src") ?? "");
        expect(srcs.filter((u) => u.includes("/stream?"))).toHaveLength(3);
        expect(screen.queryByAltText("front_right")).not.toBeInTheDocument(); // until the first snapshot arrives
        expect(fetchSpy.mock.calls.map((c) => String(c[0]))).toContain("/api/cameras/front_right/snapshot?quality=50");
        fetchSpy.mockRestore();
    });

    it("streams one camera on mobile", () => {
        mobile.value = true;
        render(<PerceptionPage/>);
        expect(document.querySelectorAll("img")).toHaveLength(1);
    });

    it("pause removes every <img>", () => {
        render(<PerceptionPage/>);
        startAll();
        fireEvent.click(screen.getByRole("switch", {name: "Pause streams"}));
        expect(document.querySelectorAll("img")).toHaveLength(0);
    });

    it("switching to raw drops the variant and draws client-side boxes", () => {
        detections.value = {
            data: {count: 1, classes: ["person"], max_score: 0.81, frame_id: "left_oa_camera",
                boxes: [{class: "person", score: 0.81, x: 10, y: 20, w: 30, h: 40}]},
            lastMessageAt: Date.now(),
        };
        render(<PerceptionPage/>);
        expect(screen.queryByTestId("tile-boxes-left_oa")).not.toBeInTheDocument(); // annotated: drawn server-side
        fireEvent.click(screen.getByText("Raw"));
        expect(screen.getByAltText("left_oa").getAttribute("src")).not.toContain("variant");
        const svg = screen.getByTestId("tile-boxes-left_oa");
        expect(svg.querySelector("rect")?.getAttribute("width")).toBe("30");
        expect(svg).toHaveTextContent("person 81%");
    });

    it("lists the latest detections with class, score and camera, plus a histogram", () => {
        detections.close = true;
        detections.value = {
            data: {count: 2, classes: ["person"], max_score: 0.81, frame_id: "rear_camera",
                boxes: [{class: "person", score: 0.81, x: 0, y: 0, w: 1, h: 1}, {class: "person", score: 0.4, x: 0, y: 0, w: 1, h: 1}]},
            lastMessageAt: 1,
        };
        render(<PerceptionPage/>);
        expect(screen.getByTestId("obstacle-close-badge")).toBeInTheDocument();
        const table = screen.getByTestId("det-table");
        expect(table).toHaveTextContent("person");
        expect(table).toHaveTextContent("81%");
        expect(table).toHaveTextContent("rear");
        expect(screen.getByTestId("det-histogram")).toHaveTextContent("person2");
    });

    it("shows an honest empty state when frames arrive without objects", () => {
        detections.value = {data: {count: 0, classes: [], max_score: 0, frame_id: "left_oa_camera", boxes: []}, lastMessageAt: 1};
        render(<PerceptionPage/>);
        expect(screen.getByTestId("det-none")).toBeInTheDocument();
    });
});
