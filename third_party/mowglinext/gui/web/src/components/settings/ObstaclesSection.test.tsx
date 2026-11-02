import { fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import { ObstaclesSection } from "./ObstaclesSection.tsx";
import { AIRSEEKERS_TRON_PROFILE_ID, getRobotProfile } from "../../constants/robotProfiles.ts";

// Active robot profile for useProfileGates (stock unless a test switches it).
const active = vi.hoisted(() => ({ id: "YardForce500" }));
vi.mock("../../hooks/useRobotProfile.ts", async (importOriginal) => ({
    ...(await importOriginal<typeof import("../../hooks/useRobotProfile.ts")>()),
    useRobotProfile: () => ({ profile: getRobotProfile(active.id), source: "settings", loading: false }),
}));
afterEach(() => { active.id = "YardForce500"; });

describe("dig obstacle setting", () => {
    it.each([true, false, "false"])("renders %s and changes the map setting only", (value) => {
        const onChange = vi.fn();
        render(<ObstaclesSection values={{ dig_obstacle_enabled: value }} onChange={onChange} />);
        const toggle = screen.getByRole("switch", { name: "Automatically create dig no-go zones" });
        const enabled = value === true;
        expect(toggle).toHaveAttribute("aria-checked", String(enabled));
        fireEvent.click(toggle);
        expect(onChange).toHaveBeenCalledExactlyOnceWith("dig_obstacle_enabled", !enabled);
        expect(screen.getByText(/dig detection, stopping and recovery remain active/)).toBeInTheDocument();
    });
});

describe("auto-mark obstacle setting", () => {
    it.each([true, false, "false"])("renders %s and toggles the live setting", (value) => {
        const onChange = vi.fn();
        render(<ObstaclesSection values={{ obstacle_auto_promote: value }} onChange={onChange} />);
        const toggle = screen.getByRole("switch", { name: "Automatically mark obstacles" });
        const enabled = value === true;
        expect(toggle).toHaveAttribute("aria-checked", String(enabled));
        fireEvent.click(toggle);
        expect(onChange).toHaveBeenCalledExactlyOnceWith("obstacle_auto_promote", !enabled);
    });
});

describe("sparse yaml: unset switches show the schema default", () => {
    it.each([
        ["obstacle_auto_promote", "Automatically mark obstacles"],
        ["dig_obstacle_enabled", "Automatically create dig no-go zones"],
    ])("%s absent from values falls back to defaults", (key, name) => {
        render(<ObstaclesSection values={{}} defaults={{ [key]: true }} onChange={vi.fn()} />);
        expect(screen.getByRole("switch", { name })).toHaveAttribute("aria-checked", "true");
    });

    it("an explicit value wins over the default", () => {
        render(<ObstaclesSection values={{ obstacle_auto_promote: false }} defaults={{ obstacle_auto_promote: true }} onChange={vi.fn()} />);
        expect(screen.getByRole("switch", { name: "Automatically mark obstacles" })).toHaveAttribute("aria-checked", "false");
    });
});

describe("profile gating", () => {
    it("shows every card and the LiDAR tip on a stock robot", () => {
        render(<ObstaclesSection values={{}} onChange={vi.fn()} />);
        expect(screen.getByText(/2D LiDAR cannot see roots/)).toBeInTheDocument();
        expect(screen.getByText("Avoidance Margins")).toBeInTheDocument();
        expect(screen.getByText("No-go Zone Margin")).toBeInTheDocument();
        expect(screen.getByText("Approach Slowdown")).toBeInTheDocument();
        expect(screen.getByRole("switch", { name: "Automatically create dig no-go zones" })).toBeInTheDocument();
    });

    it("on Tron hides unused fields and empty cards instead of tagging them", () => {
        active.id = AIRSEEKERS_TRON_PROFILE_ID;
        render(<ObstaclesSection values={{}} onChange={vi.fn()} />);
        expect(screen.getByRole("switch", { name: "Automatically mark obstacles" })).toBeInTheDocument();
        expect(screen.queryByRole("switch", { name: "Automatically create dig no-go zones" })).toBeNull();
        expect(screen.queryByText("Avoidance Margins")).toBeNull();
        expect(screen.queryByText("Approach Slowdown")).toBeNull();
        expect(screen.queryByText("Not used by this robot")).toBeNull();
    });

    it("on Tron the root-zone tip names neither the LiDAR nor a hidden margin", () => {
        active.id = AIRSEEKERS_TRON_PROFILE_ID;
        render(<ObstaclesSection values={{}} onChange={vi.fn()} />);
        expect(screen.queryByText(/LiDAR/)).toBeNull();
        expect(screen.queryByText(/margin/i)).toBeNull();
        expect(screen.getByText(/Draw the root zone as a no-go zone/)).toBeInTheDocument();
    });
});
