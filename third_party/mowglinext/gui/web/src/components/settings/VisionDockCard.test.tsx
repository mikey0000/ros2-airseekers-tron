import {beforeEach, describe, expect, it, vi} from "vitest";
import {act, fireEvent, render, screen, waitFor} from "@testing-library/react";
import {App} from "antd";
import {MemoryRouter} from "react-router-dom";
import {VisionDockCard} from "./VisionDockCard.tsx";
import {resolveDockPose} from "./visionDockPose.ts";

const mapDockingCreate = vi.fn();
const request = vi.fn();

vi.mock("../../hooks/useApi.ts", () => ({
    useApi: () => ({mowglinext: {mapDockingCreate}, request}),
}));

let dockStatus: Record<string, unknown> = {present: false};
vi.mock("../../hooks/useCalibrationStatus.ts", () => ({
    useCalibrationStatus: () => ({status: {dock: dockStatus}, error: null, refresh: vi.fn()}),
}));

describe("resolveDockPose", () => {
    it("prefers the persisted pose, then the settings form, and treats (0, 0) as unset", () => {
        expect(resolveDockPose({present: true, dock_pose_x: 1, dock_pose_y: 2, dock_pose_yaw_rad: 0.5}, {dock_pose_x: 9, dock_pose_y: 9}))
            .toEqual({x: 1, y: 2, yawRad: 0.5});
        expect(resolveDockPose({present: false}, {dock_pose_x: 3, dock_pose_y: 4, dock_pose_yaw: 1}))
            .toEqual({x: 3, y: 4, yawRad: 1});
        expect(resolveDockPose(undefined, {dock_pose_x: 0, dock_pose_y: 0})).toBeNull();
        expect(resolveDockPose(undefined, undefined)).toBeNull();
    });
});

function renderCard(onChange = vi.fn()) {
    render(
        <MemoryRouter>
            <App>
                <VisionDockCard values={{}} onChange={onChange}/>
            </App>
        </MemoryRouter>,
    );
    return onChange;
}

describe("VisionDockCard", () => {
    beforeEach(() => {
        mapDockingCreate.mockReset();
        request.mockReset();
        dockStatus = {present: false};
    });

    it("shows the stored dock pose", () => {
        dockStatus = {present: true, dock_pose_x: 1.234, dock_pose_y: -5.5, dock_pose_yaw_rad: Math.PI / 2};
        renderCard();
        expect(screen.getByText("1.23 m")).toBeInTheDocument();
        expect(screen.getByText("-5.50 m")).toBeInTheDocument();
        expect(screen.getByText("90.0°")).toBeInTheDocument();
    });

    it("sets the dock from the robot pose with use_gps_position and keeps the heading", async () => {
        mapDockingCreate.mockResolvedValue({data: {}});
        request.mockResolvedValue({data: {dock: {present: true, dock_pose_x: 2, dock_pose_y: 3, dock_pose_yaw_rad: 0.25}}});
        const onChange = renderCard();

        await act(async () => {
            fireEvent.click(screen.getByRole("button", {name: /Set dock = current robot pose/}));
            await Promise.resolve();
        });
        // Popconfirm: confirm with its OK button (same label).
        const confirm = await screen.findAllByRole("button", {name: /Set dock = current robot pose/});
        await act(async () => {
            fireEvent.click(confirm[confirm.length - 1]);
            await Promise.resolve();
        });

        await waitFor(() => expect(mapDockingCreate).toHaveBeenCalledTimes(1));
        expect(mapDockingCreate.mock.calls[0][0]).toMatchObject({use_gps_position: true, yaw_source: 0});
        await waitFor(() => expect(onChange).toHaveBeenCalledWith("dock_pose_x", 2));
        expect(onChange).toHaveBeenCalledWith("dock_pose_y", 3);
        expect(onChange).toHaveBeenCalledWith("dock_pose_yaw", 0.25);
    });
});
