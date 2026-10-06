import React from "react";
import {useTranslation} from "react-i18next";
import type {AreaSettings} from "../../utils/areaSettings.ts";

/** Read-only chip list of the settings that matter at a glance. */
export const AreaSettingsSummary: React.FC<{
    settings: Partial<AreaSettings>;
    usesDefaults?: boolean;
    tone?: "default" | "hero";
}> = ({settings, usesDefaults, tone = "default"}) => {
    const {t} = useTranslation();
    const chips: string[] = [];
    if (settings.cutter_height_mm !== undefined) chips.push(t("areaSettings.chipHeight", {value: settings.cutter_height_mm}));
    if (settings.path_mode) chips.push(t(`areaSettings.pathModes.${settings.path_mode}`));
    if (settings.mow_angle_deg !== undefined) {
        chips.push(settings.mow_angle_deg < 0 ? t("areaSettings.chipAngleAuto") : t("areaSettings.chipAngle", {value: Math.round(settings.mow_angle_deg)}));
    }
    if (settings.perimeter_laps !== undefined) chips.push(t("areaSettings.chipLaps", {count: settings.perimeter_laps}));
    if (settings.cut_speed_mps !== undefined) chips.push(t("areaSettings.mps", {value: settings.cut_speed_mps.toFixed(2)}));
    if (settings.repeat !== undefined && settings.repeat > 1) chips.push(t("areaSettings.times", {count: settings.repeat}));
    if (usesDefaults) chips.push(t("areaSettings.chipDefaults"));
    const hero = tone === "hero";
    return (
        <div data-testid="area-settings-summary" style={{display: "flex", flexWrap: "wrap", gap: 6, marginTop: 6}}>
            {chips.map((c) => (
                <span key={c} style={{
                    fontSize: 11, fontWeight: 600, padding: "3px 9px", borderRadius: 999,
                    background: hero ? "rgba(124,255,178,0.12)" : "rgba(127,127,127,0.12)",
                    color: hero ? "var(--lime, #7CFFB2)" : undefined,
                    border: hero ? "1px solid rgba(124,255,178,0.25)" : "1px solid rgba(127,127,127,0.2)",
                }}>{c}</span>
            ))}
        </div>
    );
};
