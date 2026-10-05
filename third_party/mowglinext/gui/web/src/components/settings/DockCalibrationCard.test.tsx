import { App } from "antd";
import { render, screen } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import { DockCalibrationCard } from "./DockCalibrationCard.tsx";
import type { DockCalibrationStatus } from "../../hooks/useDockCalibration.ts";

// Return value of useDockCalibration, overridden per test.
interface UseDockCalibrationValue {
    status: DockCalibrationStatus | null;
    start: () => void;
    starting: boolean;
    startError: string | null;
    running: boolean;
    done: boolean;
}

const hookValue: UseDockCalibrationValue = {
    status: null,
    start: vi.fn(),
    starting: false,
    startError: null,
    running: false,
    done: false,
};

vi.mock("../../hooks/useDockCalibration.ts", async (importOriginal) => {
    const actual = await importOriginal<typeof import("../../hooks/useDockCalibration.ts")>();
    return {
        ...actual,
        useDockCalibration: () => hookValue,
    };
});

vi.mock("../../theme/ThemeContext.tsx", () => ({
    useThemeMode: () => ({ colors: { text: "#000", primary: "#1677ff" } }),
}));

const renderCard = () =>
    render(
        <App>
            <DockCalibrationCard />
        </App>,
    );

describe("DockCalibrationCard", () => {
    beforeEach(() => {
        hookValue.status = null;
        hookValue.starting = false;
        hookValue.startError = null;
        hookValue.running = false;
        hookValue.done = false;
        Object.defineProperty(window, "matchMedia", {
            writable: true,
            value: vi.fn().mockImplementation((query: string) => ({
                matches: false,
                media: query,
                onchange: null,
                addListener: vi.fn(),
                removeListener: vi.fn(),
                addEventListener: vi.fn(),
                removeEventListener: vi.fn(),
                dispatchEvent: vi.fn(),
            })),
        });
    });

    it("surfaces the start-rejection reason returned by the ROS service", () => {
        // Regression for issue #412: a rejected start (e.g. robot not on the
        // dock) publishes no status message, so startError is the only signal —
        // the card must render it instead of silently doing nothing.
        hookValue.startError = "Robot is not on the dock (not charging). Dock it, then retry.";
        renderCard();

        expect(screen.getByText("Could not start dock calibration")).toBeInTheDocument();
        expect(
            screen.getByText("Robot is not on the dock (not charging). Dock it, then retry."),
        ).toBeInTheDocument();
    });

    it("shows no error alert when the start was accepted", () => {
        renderCard();

        expect(screen.queryByText("Could not start dock calibration")).not.toBeInTheDocument();
    });
});
