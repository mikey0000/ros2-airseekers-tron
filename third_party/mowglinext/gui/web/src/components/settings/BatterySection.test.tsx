import { render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import { ThemeProvider } from "../../theme/ThemeContext.tsx";
import { AIRSEEKERS_TRON_PROFILE_ID, getRobotProfile } from "../../constants/robotProfiles.ts";
import { BatterySection } from "./BatterySection.tsx";
import { SettingsPreview } from "./SettingsPreview.tsx";

// Active robot profile for useProfileGates (stock unless a test switches it).
const active = vi.hoisted(() => ({ id: "YardForce500" }));
vi.mock("../../hooks/useRobotProfile.ts", async (importOriginal) => ({
    ...(await importOriginal<typeof import("../../hooks/useRobotProfile.ts")>()),
    useRobotProfile: () => ({ profile: getRobotProfile(active.id), source: "settings", loading: false }),
}));
vi.mock("../../hooks/usePower.ts", () => ({ usePower: () => ({}) }));
afterEach(() => { active.id = "YardForce500"; });

const values = {
    battery_full_voltage: 28, battery_empty_voltage: 24, battery_critical_voltage: 23,
    battery_full_percent: 95, battery_low_percent: 20, battery_critical_percent: 10,
};

const renderSection = () => render(
    <ThemeProvider><BatterySection values={values} onChange={vi.fn()} /></ThemeProvider>,
);
const renderPreview = () => render(
    <ThemeProvider><SettingsPreview values={values} section="battery" /></ThemeProvider>,
);

describe("BatterySection profile gating", () => {
    it("shows voltage and critical thresholds on a stock robot", () => {
        renderSection();
        expect(screen.getByText("Full Voltage")).toBeInTheDocument();
        expect(screen.getByText("Critical Voltage")).toBeInTheDocument();
        expect(screen.getByText("Resume Above")).toBeInTheDocument();
    });

    it("on Tron hides the voltage card and unused percentages instead of tagging them", () => {
        active.id = AIRSEEKERS_TRON_PROFILE_ID;
        renderSection();
        expect(screen.queryByText("Full Voltage")).toBeNull();
        expect(screen.queryByText("Critical Voltage")).toBeNull();
        expect(screen.queryByText("Not used by this robot")).toBeNull();
        // Bound keys stay.
        expect(screen.getByText("Resume Above")).toBeInTheDocument();
        expect(screen.getByText("Max charge level")).toBeInTheDocument();
    });
});

describe("SettingsPreview battery", () => {
    it("prints voltages and the critical level on a stock robot", () => {
        renderPreview();
        expect(screen.getByText(/Empty 24\.00 V/)).toBeInTheDocument();
        expect(screen.getByText(/critical at 10 %/)).toBeInTheDocument();
    });

    it("on Tron leaves out the voltage line and the critical level", () => {
        active.id = AIRSEEKERS_TRON_PROFILE_ID;
        renderPreview();
        expect(screen.queryByText(/Empty 24\.00 V/)).toBeNull();
        expect(screen.queryByText(/critical/i)).toBeNull();
        expect(screen.getByText("Returns home at 20 %, resumes at 95 %")).toBeInTheDocument();
    });
});
