import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import { ObstaclesSection } from "./ObstaclesSection.tsx";

describe("dig obstacle setting", () => {
    it.each([true, false, "false"])("renders %s and changes the map setting only", (value) => {
        const onChange = vi.fn();
        render(<ObstaclesSection values={{ dig_obstacle_enabled: value }} onChange={onChange} />);
        const toggle = screen.getByRole("switch", { name: "Automatically create dig keepouts" });
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
        ["dig_obstacle_enabled", "Automatically create dig keepouts"],
    ])("%s absent from values falls back to defaults", (key, name) => {
        render(<ObstaclesSection values={{}} defaults={{ [key]: true }} onChange={vi.fn()} />);
        expect(screen.getByRole("switch", { name })).toHaveAttribute("aria-checked", "true");
    });

    it("an explicit value wins over the default", () => {
        render(<ObstaclesSection values={{ obstacle_auto_promote: false }} defaults={{ obstacle_auto_promote: true }} onChange={vi.fn()} />);
        expect(screen.getByRole("switch", { name: "Automatically mark obstacles" })).toHaveAttribute("aria-checked", "false");
    });
});
