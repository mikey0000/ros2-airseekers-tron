import {beforeEach, describe, expect, it, vi} from "vitest";
import {act, fireEvent, render, screen} from "@testing-library/react";
import {App} from "antd";
import {ThemeProvider} from "../theme/ThemeContext.tsx";
import {TimeFormatProvider} from "../hooks/useTimeFormat.tsx";
import type {RosoutRecord} from "./rosoutLog.ts";

// A robot without a Docker host (profile feature docker_host off): the page
// must not touch /containers and must stream /rosout instead.
type StreamHandler = (data: string, first?: boolean) => void;
let pushFrame: StreamHandler = () => { /* replaced on render */ };
const start = vi.fn();
const containersList = vi.fn();
let dockerHost = false;

vi.mock("../hooks/useWS.ts", () => ({
    useWS: (_onError: unknown, _onInfo: unknown, onData: StreamHandler) => {
        pushFrame = onData;
        return {start, stop: vi.fn()};
    },
}));

vi.mock("../hooks/useRobotProfile.ts", () => ({
    useFeature: (feature: string) => feature === "docker_host" ? dockerHost : true,
}));

vi.mock("../hooks/useApi.ts", () => ({
    useApi: () => ({containers: {containersList, containersCreate: vi.fn()}}),
}));

import LogsPage from "./LogsPage.tsx";

const rec = (seq: number, node: string, level: RosoutRecord["level"], msg: string): string => JSON.stringify({
    boot: 1, seq, stamp_ms: 1747087353123, level, node, msg,
} satisfies RosoutRecord);

async function renderPage() {
    render(
        <ThemeProvider>
            <TimeFormatProvider>
                <App>
                    <LogsPage/>
                </App>
            </TimeFormatProvider>
        </ThemeProvider>,
    );
    await act(async () => {
        await new Promise((resolve) => setTimeout(resolve, 0));
    });
}

async function push(frames: string[]) {
    await act(async () => {
        frames.forEach((f) => pushFrame(f));
        await new Promise((resolve) => setTimeout(resolve, 200));
    });
}

describe("LogsPage /rosout source", () => {
    beforeEach(() => {
        start.mockReset();
        containersList.mockReset();
        dockerHost = false;
        const storage = new Map<string, string>();
        Object.defineProperty(window, "localStorage", {
            configurable: true,
            value: {
                getItem: (key: string) => storage.get(key) ?? null,
                setItem: (key: string, value: string) => storage.set(key, value),
                removeItem: (key: string) => storage.delete(key),
                clear: () => storage.clear(),
            },
        });
    });

    it("streams /rosout without asking for containers when the robot has no Docker host", async () => {
        await renderPage();

        expect(containersList).not.toHaveBeenCalled();
        expect(start).toHaveBeenCalledWith("/api/rosout/stream");
        expect(screen.queryByText("Containers")).not.toBeInTheDocument();
    });

    it("renders node and level, and drops a replayed record", async () => {
        await renderPage();

        await push([rec(1, "map_server_node", "WARN", "dock pose unset"), rec(1, "map_server_node", "WARN", "dock pose unset")]);

        expect(screen.getAllByTestId("log-line")).toHaveLength(1);
        expect(screen.getByText("[map_server_node]")).toBeInTheDocument();
        expect(screen.getByText("dock pose unset")).toBeInTheDocument();
    });

    it("holds lines back while paused and appends them on resume", async () => {
        await renderPage();
        await push([rec(1, "a", "INFO", "first")]);

        await act(async () => {
            fireEvent.click(screen.getByRole("button", {name: "Pause"}));
            await Promise.resolve();
        });
        await push([rec(2, "a", "INFO", "second")]);
        expect(screen.queryByText("second")).not.toBeInTheDocument();

        await act(async () => {
            fireEvent.click(screen.getByRole("button", {name: /Resume \(1 new\)/}));
            await Promise.resolve();
        });
        expect(screen.getByText("second")).toBeInTheDocument();
    });

    it("falls back to /rosout when the backend reports no Docker daemon", async () => {
        dockerHost = true;
        containersList.mockResolvedValue({data: {available: false, containers: []}});

        await renderPage();

        expect(containersList).toHaveBeenCalled();
        expect(start).toHaveBeenLastCalledWith("/api/rosout/stream");
        expect(screen.getByText(/No Docker daemon/i)).toBeInTheDocument();
    });

    it("offers a source switch when containers are available", async () => {
        dockerHost = true;
        containersList.mockResolvedValue({
            data: {available: true, containers: [{id: "c1", names: ["/mowgli-ros2"], state: "running", labels: {}}]},
        });

        await renderPage();
        expect(start).toHaveBeenLastCalledWith("/api/containers/c1/logs");

        await act(async () => {
            fireEvent.click(screen.getByText("ROS log"));
            await Promise.resolve();
        });
        expect(start).toHaveBeenLastCalledWith("/api/rosout/stream");
    });
});
