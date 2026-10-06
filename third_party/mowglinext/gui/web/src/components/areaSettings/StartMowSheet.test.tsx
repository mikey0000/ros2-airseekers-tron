import {fireEvent, render, screen, waitFor} from "@testing-library/react";
import {beforeEach, describe, expect, it, vi} from "vitest";
import {StartMowSheet} from "./StartMowSheet.tsx";

const mocks = vi.hoisted(() => ({
    callCreate: vi.fn(),
    request: vi.fn(),
    resume: {value: false},
    notifyError: vi.fn(),
}));

const api = {mowglinext: {callCreate: mocks.callCreate}, request: mocks.request};
vi.mock("../../hooks/useApi.ts", () => ({useApi: () => api}));
vi.mock("../../hooks/useCoverageResumeAvailable.ts", () => ({useCoverageResumeAvailable: () => mocks.resume.value}));
vi.mock("../../hooks/useIsMobile.ts", () => ({useIsMobile: () => false}));
vi.mock("react-i18next", () => ({useTranslation: () => ({t: (k: string) => k})}));
vi.mock("antd", async (orig) => ({
    ...(await orig<typeof import("antd")>()),
    App: {useApp: () => ({notification: {error: mocks.notifyError, success: vi.fn()}})},
}));

const areas = [{index: 0, name: "Front"}, {index: 3, name: "Back"}];

function mockGet(byPath: Record<string, object>) {
    mocks.request.mockImplementation(async ({path, method}: {path: string; method: string}) => {
        if (method === "PUT") return {data: {supported: true}};
        const settings = byPath[path];
        if (!settings) throw {status: 501, data: {supported: false}};
        return {data: {supported: true, settings}};
    });
}

describe("StartMowSheet", () => {
    beforeEach(() => {
        vi.clearAllMocks();
        mocks.resume.value = false;
        mocks.callCreate.mockResolvedValue({data: {}, error: undefined});
    });

    it("starts all areas with plain START and shows the defaults", async () => {
        mockGet({"/mowglinext/areas/defaults/settings": {cutter_height_mm: 60}});
        render(<StartMowSheet open areas={areas} onClose={vi.fn()}/>);
        await screen.findByTestId("area-settings-summary");
        expect(screen.getByText("areaSettings.chipHeight")).toBeTruthy();
        fireEvent.click(screen.getByTestId("start-confirm"));
        await waitFor(() => expect(mocks.callCreate).toHaveBeenCalledTimes(1));
        expect(mocks.callCreate.mock.calls[0]).toEqual(["high_level_control", {Command: 1}]);
        expect(mocks.request.mock.calls.some(([a]) => a.method === "PUT")).toBe(false);
    });

    it("starts one area with start_in_area", async () => {
        mockGet({
            "/mowglinext/areas/defaults/settings": {cutter_height_mm: 50},
            "/mowglinext/areas/3/settings": {cutter_height_mm: 70},
        });
        render(<StartMowSheet open areas={areas} initialSelection={3} onClose={vi.fn()}/>);
        await screen.findByTestId("area-settings-summary");
        fireEvent.click(screen.getByTestId("start-confirm"));
        await waitFor(() => expect(mocks.callCreate).toHaveBeenCalled());
        expect(mocks.callCreate.mock.calls[0]).toEqual(["start_in_area", {area: 3}]);
    });

    it("clears the resume file first when Start fresh is chosen", async () => {
        mocks.resume.value = true;
        mockGet({"/mowglinext/areas/defaults/settings": {}});
        render(<StartMowSheet open areas={areas} onClose={vi.fn()}/>);
        await screen.findByTestId("area-settings-summary");
        fireEvent.click(screen.getByTestId("start-fresh"));
        fireEvent.click(screen.getByTestId("start-confirm"));
        await waitFor(() => expect(mocks.callCreate).toHaveBeenCalledTimes(2));
        expect(mocks.callCreate.mock.calls[0]).toEqual(["coverage_clear_resume", {}]);
        expect(mocks.callCreate.mock.calls[1]).toEqual(["high_level_control", {Command: 1}]);
    });

    it("PUTs only the changed run overrides before starting", async () => {
        mockGet({"/mowglinext/areas/defaults/settings": {cutter_height_mm: 50, repeat: 1}});
        render(<StartMowSheet open areas={areas} onClose={vi.fn()}/>);
        await screen.findByTestId("area-settings-summary");
        const slider = document.querySelector(".ant-slider-handle") as HTMLElement;
        slider.focus();
        fireEvent.keyDown(slider, {key: "ArrowRight", keyCode: 39});
        fireEvent.click(screen.getByTestId("start-confirm"));
        await waitFor(() => expect(mocks.callCreate).toHaveBeenCalled());
        const put = mocks.request.mock.calls.find(([a]) => a.method === "PUT")![0];
        expect(put.path).toBe("/mowglinext/areas/defaults/settings");
        expect(put.body).toEqual({settings: {cutter_height_mm: 55}});
    });

    it("degrades when the robot has no area settings service", async () => {
        mockGet({});
        render(<StartMowSheet open areas={areas} onClose={vi.fn()}/>);
        expect(await screen.findByText("areaSettings.notSupported")).toBeTruthy();
        fireEvent.click(screen.getByTestId("start-confirm"));
        await waitFor(() => expect(mocks.callCreate).toHaveBeenCalledWith("high_level_control", {Command: 1}));
    });
});
