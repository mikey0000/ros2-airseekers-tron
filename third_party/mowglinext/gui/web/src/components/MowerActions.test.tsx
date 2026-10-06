import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import { MowerActions } from "./MowerActions.tsx";

const mocks = vi.hoisted(() => ({
    callCreate: vi.fn(),
    resume: { value: true },
}));

vi.mock("../hooks/useApi.ts", () => ({ useApi: () => ({ mowglinext: { callCreate: mocks.callCreate } }) }));
vi.mock("../hooks/useCoverageResumeAvailable.ts", () => ({ useCoverageResumeAvailable: () => mocks.resume.value }));
vi.mock("../hooks/useHighLevelStatus.ts", () => ({
    useHighLevelStatus: () => ({ highLevelStatus: { state: 1, state_name: "IDLE_DOCKED", emergency: false } }),
}));
vi.mock("../theme/ThemeContext.tsx", () => ({ useThemeMode: () => ({ colors: {} }) }));
vi.mock("react-i18next", () => ({ useTranslation: () => ({ t: (k: string) => k }) }));
vi.mock("antd", async (orig) => ({
    ...(await orig<typeof import("antd")>()),
    App: { useApp: () => ({ modal: { confirm: vi.fn() }, notification: {} }) },
}));

describe("MowerActions coverage resume", () => {
    beforeEach(() => {
        vi.clearAllMocks();
        mocks.callCreate.mockResolvedValue({ data: {}, error: undefined });
    });

    it("offers Resume and Start fresh when a resume is available", () => {
        mocks.resume.value = true;
        render(<MowerActions bare />);
        expect(screen.getByText("mowerActions.resume")).toBeTruthy();
        expect(screen.getByText("mowerActions.startFresh")).toBeTruthy();
    });

    it("hides Start fresh when there is nothing to resume", () => {
        mocks.resume.value = false;
        render(<MowerActions bare />);
        expect(screen.getByText("mowerActions.start")).toBeTruthy();
        expect(screen.queryByText("mowerActions.startFresh")).toBeNull();
    });

    it("Start fresh clears the resume file before sending START", async () => {
        mocks.resume.value = true;
        render(<MowerActions bare />);
        fireEvent.click(screen.getByText("mowerActions.startFresh"));
        await waitFor(() => expect(mocks.callCreate).toHaveBeenCalledTimes(2));
        expect(mocks.callCreate.mock.calls[0]).toEqual(["coverage_clear_resume", {}]);
        expect(mocks.callCreate.mock.calls[1]).toEqual(["high_level_control", { Command: 1 }]);
    });
});
