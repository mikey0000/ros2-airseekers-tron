import { render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { beforeEach, describe, expect, it, vi } from "vitest";
import { RecorderCard } from "./RecorderCard.tsx";

const callCreateMock = vi.fn();

// Stable object: the card refreshes in an effect keyed on the api handle.
const apiMock = { mowglinext: { callCreate: callCreateMock } };

vi.mock("../../hooks/useApi.ts", () => ({
    useApi: () => apiMock,
}));

vi.mock("react-i18next", () => ({
    useTranslation: () => ({
        t: (key: string, opts?: { gb?: string }) => (opts?.gb ? `${key}:${opts.gb}` : key),
    }),
}));

const reply = (status: Record<string, unknown>) => ({
    data: { success: true, message: JSON.stringify(status) },
    error: null,
});

const ON = {
    enabled: true, recording: true, paused_low_disk: false, session: "s",
    ring_bytes: 2_500_000_000, free_bytes: 9_876_543_210, mow: "20261009-104315",
};
const OFF = { ...ON, enabled: false, recording: false, mow: null };

describe("RecorderCard", () => {
    beforeEach(() => {
        callCreateMock.mockReset();
        Object.defineProperty(window, "matchMedia", {
            writable: true,
            value: vi.fn().mockImplementation((query: string) => ({
                matches: false, media: query, onchange: null,
                addListener: vi.fn(), removeListener: vi.fn(),
                addEventListener: vi.fn(), removeEventListener: vi.fn(), dispatchEvent: vi.fn(),
            })),
        });
    });

    it("renders the state fetched on mount", async () => {
        callCreateMock.mockResolvedValue(reply(ON));
        render(<RecorderCard />);

        expect(await screen.findByText("settingsRecorder.recording")).toBeTruthy();
        expect(callCreateMock).toHaveBeenCalledWith("bag_recorder_state", {});
        expect(screen.getByText("settingsRecorder.ring:2.5")).toBeTruthy();
        expect(screen.getByText("settingsRecorder.free:9.9")).toBeTruthy();
        expect(screen.getByText("settingsRecorder.mowRecording")).toBeTruthy();
        expect(screen.getByRole("switch").getAttribute("aria-checked")).toBe("true");
    });

    it("toggling the switch calls bag_recorder and shows the reply state", async () => {
        callCreateMock.mockResolvedValueOnce(reply(ON)).mockResolvedValueOnce(reply(OFF));
        const user = userEvent.setup();
        render(<RecorderCard />);

        await screen.findByText("settingsRecorder.recording");
        await user.click(screen.getByRole("switch"));

        await waitFor(() => expect(callCreateMock).toHaveBeenCalledWith("bag_recorder", { enabled: false }));
        expect(await screen.findByText("settingsRecorder.off")).toBeTruthy();
        expect(screen.queryByText("settingsRecorder.mowRecording")).toBeNull();
        expect(screen.getByRole("switch").getAttribute("aria-checked")).toBe("false");
    });

    it("shows an alert when the recorder is unreachable", async () => {
        callCreateMock.mockResolvedValue({ data: null, error: { error: "service not available" } });
        render(<RecorderCard />);

        expect(await screen.findByText("settingsRecorder.unreachable")).toBeTruthy();
        expect(screen.getByText("service not available")).toBeTruthy();
    });
});
