import {fireEvent, render, screen, waitFor} from "@testing-library/react";
import {beforeEach, describe, expect, it, vi} from "vitest";
import {AreaSettingsPanel} from "./AreaSettingsPanel.tsx";

const mocks = vi.hoisted(() => ({request: vi.fn(), success: vi.fn(), error: vi.fn()}));


const api = {request: mocks.request};
vi.mock("../../hooks/useApi.ts", () => ({useApi: () => api}));
vi.mock("../../hooks/useTopic.ts", () => ({useTopic: (_: string, initial: unknown) => ({data: initial})}));
vi.mock("react-i18next", () => ({useTranslation: () => ({t: (k: string) => k})}));
vi.mock("../../theme/ThemeContext.tsx", () => ({useThemeMode: () => ({colors: {}})}));
vi.mock("antd", async (orig) => ({
    ...(await orig<typeof import("antd")>()),
    App: {useApp: () => ({notification: {error: mocks.error, success: mocks.success}})},
}));

const putBodies = () => mocks.request.mock.calls.filter(([a]) => a.method === "PUT").map(([a]) => a);

describe("AreaSettingsPanel", () => {
    beforeEach(() => vi.clearAllMocks());

    it("shows the unsupported notice on 501", async () => {
        mocks.request.mockRejectedValue({status: 501});
        render(<AreaSettingsPanel target={2}/>);
        expect(await screen.findByTestId("area-settings-unsupported")).toBeTruthy();
    });

    it("saves a path mode change as the area override", async () => {
        mocks.request.mockImplementation(async ({path, method}: {path: string; method: string}) => {
            if (method === "PUT") return {data: {supported: true}};
            return {data: {supported: true, settings: path.includes("defaults") ? {cutter_height_mm: 55} : {}}};
        });
        render(<AreaSettingsPanel target={2}/>);
        await screen.findByTestId("area-settings-form");
        // Area starts on defaults: switch that off, pick spiral, save.
        fireEvent.click(screen.getByTestId("use-defaults"));
        fireEvent.click(screen.getByTestId("path-mode-spiral"));
        fireEvent.click(screen.getByTestId("area-settings-save"));
        await waitFor(() => expect(putBodies()).toHaveLength(1));
        const put = putBodies()[0];
        expect(put.path).toBe("/mowglinext/areas/2/settings");
        expect(put.body.settings.path_mode).toBe("spiral");
        // Equal to the defaults -> reset (null) so the area keeps following them.
        expect(put.body.settings.cutter_height_mm).toBeNull();
    });

    it("sends use_defaults when the toggle is turned back on", async () => {
        mocks.request.mockImplementation(async ({path, method}: {path: string; method: string}) => {
            if (method === "PUT") return {data: {supported: true}};
            return {data: {supported: true, settings: path.includes("defaults") ? {} : {repeat: 3}}};
        });
        render(<AreaSettingsPanel target={1}/>);
        await screen.findByTestId("area-settings-form");
        fireEvent.click(screen.getByTestId("use-defaults"));
        fireEvent.click(screen.getByTestId("area-settings-save"));
        await waitFor(() => expect(putBodies()).toHaveLength(1));
        expect(putBodies()[0].body).toEqual({use_defaults: true});
    });

    it("edits the defaults without a use-defaults toggle", async () => {
        mocks.request.mockResolvedValue({data: {supported: true, settings: {}}});
        render(<AreaSettingsPanel target="defaults"/>);
        await screen.findByTestId("area-settings-form");
        expect(screen.queryByTestId("use-defaults")).toBeNull();
    });
});
