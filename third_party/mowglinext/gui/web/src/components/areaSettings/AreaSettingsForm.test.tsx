import {fireEvent, render, screen} from "@testing-library/react";
import {describe, expect, it, vi} from "vitest";
import {AreaSettingsForm} from "./AreaSettingsForm.tsx";
import {AREA_SETTINGS_DEFAULTS, type AreaSettings} from "../../utils/areaSettings.ts";
import en from "../../i18n/locales/en.json";

vi.mock("../../theme/ThemeContext.tsx", () => ({useThemeMode: () => ({colors: {}})}));

const renderForm = (patch: Partial<AreaSettings>) => {
    const onChange = vi.fn();
    render(<AreaSettingsForm value={{...AREA_SETTINGS_DEFAULTS, ...patch}} onChange={onChange}/>);
    return onChange;
};

describe("AreaSettingsForm cross angle", () => {
    it("labels the start direction and the cross angle in cross mode", () => {
        renderForm({path_mode: "cross", mow_angle_deg: 105});
        expect(screen.getByText(en.areaSettings.startAngle)).toBeInTheDocument();
        expect(screen.getByText(en.areaSettings.crossAngle)).toBeInTheDocument();
        expect(screen.getByText(en.areaSettings.crossAngleHint)).toBeInTheDocument();
        expect(screen.queryByText(en.areaSettings.mowAngle)).toBeNull();
        expect(screen.queryByText(en.areaSettings.alternateOffset)).toBeNull();
    });

    it("keeps the alternate wording on the same key", () => {
        renderForm({path_mode: "alternate", alternate_angle_offset_deg: 45});
        expect(screen.getByText(en.areaSettings.mowAngle)).toBeInTheDocument();
        expect(screen.getByText(en.areaSettings.alternateOffset)).toBeInTheDocument();
        expect(screen.getByText("45°")).toBeInTheDocument();
        expect(screen.queryByText(en.areaSettings.crossAngle)).toBeNull();
    });

    it("hides the offset for zigzag", () => {
        renderForm({path_mode: "zigzag"});
        expect(screen.queryByText(en.areaSettings.crossAngle)).toBeNull();
        expect(screen.queryByText(en.areaSettings.alternateOffset)).toBeNull();
    });

    it("starts a cross at 90° when the alternate step would repeat the rows", () => {
        const onChange = renderForm({path_mode: "alternate", alternate_angle_offset_deg: 180});
        fireEvent.click(screen.getByTestId("path-mode-cross"));
        expect(onChange).toHaveBeenLastCalledWith(expect.objectContaining(
            {path_mode: "cross", alternate_angle_offset_deg: 90}));
    });

    it("keeps a usable step when switching to cross", () => {
        const onChange = renderForm({path_mode: "alternate", alternate_angle_offset_deg: 45});
        fireEvent.click(screen.getByTestId("path-mode-cross"));
        expect(onChange).toHaveBeenLastCalledWith(expect.objectContaining(
            {path_mode: "cross", alternate_angle_offset_deg: 45}));
    });
});
