import React from "react";
import {InputNumber, Select, Slider, Switch, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {
    AREA_SETTINGS_RANGES,
    type AreaSettings,
    MOW_ANGLE_AUTO,
    ROUTE_ORDERS,
    TURN_TYPES,
} from "../../utils/areaSettings.ts";
import {PathModeCards} from "./PathModeCards.tsx";
import {AnglePicker} from "./AnglePicker.tsx";

const {Text} = Typography;

const Field: React.FC<React.PropsWithChildren<{
    label: string; value?: React.ReactNode; hint?: string;
    /** undefined = no indicator; true = area override; false = inherited. */
    custom?: boolean;
}>> = ({label, value, hint, custom, children}) => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();
    return (
        <div style={{marginBottom: 16}}>
            <div style={{display: "flex", justifyContent: "space-between", alignItems: "baseline", marginBottom: 4}}>
                <Text strong style={{fontSize: 13}}>
                    {label}
                    {custom !== undefined && (
                        <span data-testid={custom ? "field-custom" : "field-inherited"} style={{
                            marginLeft: 6, fontSize: 10, fontWeight: 600, padding: "1px 6px", borderRadius: 999,
                            border: `1px solid ${custom ? colors.primary : colors.borderSubtle}`,
                            color: custom ? colors.primary : colors.muted,
                        }}>{custom ? t("areaSettings.custom") : t("areaSettings.inherited")}</span>
                    )}
                </Text>
                {value !== undefined && <Text style={{fontSize: 13, color: colors.primary, fontWeight: 600}}>{value}</Text>}
            </div>
            {children}
            {hint && <div style={{fontSize: 11, color: colors.muted, marginTop: 2}}>{hint}</div>}
        </div>
    );
};

/** Controlled editor for one complete AreaSettings object. */
export const AreaSettingsForm: React.FC<{
    value: AreaSettings;
    onChange: (next: AreaSettings) => void;
    disabled?: boolean;
    /** Keys this area overrides; when given, each field shows custom vs inherited. */
    overridden?: Set<keyof AreaSettings>;
}> = ({value, onChange, disabled, overridden}) => {
    const {t} = useTranslation();
    const set = <K extends keyof AreaSettings>(k: K, v: AreaSettings[K]) => onChange({...value, [k]: v});
    const R = AREA_SETTINGS_RANGES;
    const auto = value.mow_angle_deg < 0;
    const c = (k: keyof AreaSettings) => (overridden ? overridden.has(k) : undefined);

    return (
        <div data-testid="area-settings-form">
            <Field label={t("areaSettings.cutterHeight")} custom={c("cutter_height_mm")} value={t("areaSettings.mm", {value: value.cutter_height_mm})}>
                <Slider min={R.cutter_height_mm.min} max={R.cutter_height_mm.max} step={R.cutter_height_mm.step}
                        disabled={disabled} value={value.cutter_height_mm}
                        marks={{30: "30", 50: "50", 70: "70", 90: "90"}}
                        tooltip={{formatter: (v) => `${v} mm`}}
                        onChange={(v: number) => set("cutter_height_mm", v)}/>
            </Field>

            <Field label={t("areaSettings.pathMode")} custom={c("path_mode")}>
                <PathModeCards value={value.path_mode} disabled={disabled}
                               onChange={(m) => set("path_mode", m)}/>
            </Field>

            <Field label={t("areaSettings.mowAngle")} custom={c("mow_angle_deg")}
                   value={auto ? t("areaSettings.auto") : `${Math.round(value.mow_angle_deg)}°`}
                   hint={t("areaSettings.mowAngleHint")}>
                <div style={{display: "flex", alignItems: "center", gap: 16, flexWrap: "wrap"}}>
                    <AnglePicker value={auto ? 0 : value.mow_angle_deg} disabled={disabled || auto}
                                 onChange={(deg) => set("mow_angle_deg", deg)}/>
                    <div style={{display: "flex", flexDirection: "column", gap: 8}}>
                        <label style={{display: "flex", alignItems: "center", gap: 8}}>
                            <Switch size="small" checked={auto} disabled={disabled} data-testid="mow-angle-auto"
                                    onChange={(on) => set("mow_angle_deg", on ? MOW_ANGLE_AUTO : 0)}/>
                            {t("areaSettings.auto")}
                        </label>
                        <InputNumber min={0} max={359} step={5} size="small" addonAfter="°"
                                     disabled={disabled || auto} style={{width: 110}}
                                     value={auto ? undefined : value.mow_angle_deg}
                                     onChange={(v) => typeof v === "number" && set("mow_angle_deg", v)}/>
                    </div>
                </div>
            </Field>

            {value.path_mode === "alternate" && (
                <Field label={t("areaSettings.alternateOffset")} custom={c("alternate_angle_offset_deg")} value={`${value.alternate_angle_offset_deg}°`}
                       hint={t("areaSettings.alternateOffsetHint")}>
                    <Slider min={R.alternate_angle_offset_deg.min} max={R.alternate_angle_offset_deg.max}
                            step={R.alternate_angle_offset_deg.step} disabled={disabled}
                            value={value.alternate_angle_offset_deg}
                            onChange={(v: number) => set("alternate_angle_offset_deg", v)}/>
                </Field>
            )}

            <Field label={t("areaSettings.perimeterLaps")} custom={c("perimeter_laps")} value={value.perimeter_laps}>
                <Slider min={R.perimeter_laps.min} max={R.perimeter_laps.max} step={1} dots disabled={disabled}
                        value={value.perimeter_laps} onChange={(v: number) => set("perimeter_laps", v)}/>
            </Field>

            <Field label={t("areaSettings.edgeFirst")} custom={c("edge_first")} hint={t("areaSettings.edgeFirstHint")}>
                <Switch checked={value.edge_first} disabled={disabled}
                        onChange={(v) => set("edge_first", v)}/>
            </Field>

            {value.path_mode !== "spiral" && value.path_mode !== "contour_only" && (<>
                <Field label={t("areaSettings.routeOrder")} custom={c("route_order")} hint={t("areaSettings.routeOrderHint")}>
                    <Select data-testid="route-order" style={{width: "100%"}} disabled={disabled}
                            value={value.route_order} onChange={(v) => set("route_order", v)}
                            options={ROUTE_ORDERS.map((o) => ({
                                value: o,
                                label: t(`areaSettings.routeOrder${o[0].toUpperCase()}${o.slice(1)}`),
                            }))}/>
                </Field>
                {value.route_order === "spiral" && (
                    <Field label={t("areaSettings.routeSpiralSize")} custom={c("route_spiral_size")}>
                        <InputNumber min={R.route_spiral_size.min} max={R.route_spiral_size.max} step={1}
                                     precision={0} size="small" disabled={disabled} style={{width: 110}}
                                     value={value.route_spiral_size}
                                     onChange={(v) => typeof v === "number" && set("route_spiral_size", v)}/>
                    </Field>
                )}
                <Field label={t("areaSettings.turnType")} custom={c("turn_type")} hint={t("areaSettings.turnTypeHint")}>
                    <Select data-testid="turn-type" style={{width: "100%"}} disabled={disabled}
                            value={value.turn_type} onChange={(v) => set("turn_type", v)}
                            options={TURN_TYPES.map((o) => ({
                                value: o,
                                label: t(`areaSettings.turnType${o[0].toUpperCase()}${o.slice(1)}`),
                            }))}/>
                </Field>
                <Field label={t("areaSettings.minTurnRadius")} custom={c("min_turn_radius_m")}
                       hint={t("areaSettings.minTurnRadiusHint")}>
                    <InputNumber min={R.min_turn_radius_m.min} max={R.min_turn_radius_m.max}
                                 step={R.min_turn_radius_m.step} precision={2} size="small" addonAfter="m"
                                 disabled={disabled} style={{width: 130}} value={value.min_turn_radius_m}
                                 onChange={(v) => typeof v === "number" && set("min_turn_radius_m", v)}/>
                </Field>
            </>)}

            <Field label={t("areaSettings.cutSpeed")} custom={c("cut_speed_mps")} value={t("areaSettings.mps", {value: value.cut_speed_mps.toFixed(2)})}>
                <Slider min={R.cut_speed_mps.min} max={R.cut_speed_mps.max} step={R.cut_speed_mps.step}
                        disabled={disabled} value={value.cut_speed_mps}
                        onChange={(v: number) => set("cut_speed_mps", Math.round(v * 100) / 100)}/>
            </Field>

            <Field label={t("areaSettings.swathOverlap")} custom={c("swath_overlap_m")}
                   value={t("areaSettings.cm", {value: Math.round(value.swath_overlap_m * 100)})}>
                <Slider min={0} max={10} step={1} disabled={disabled}
                        value={Math.round(value.swath_overlap_m * 100)}
                        tooltip={{formatter: (v) => `${v} cm`}}
                        onChange={(v: number) => set("swath_overlap_m", v / 100)}/>
            </Field>

            <Field label={t("areaSettings.repeat")} custom={c("repeat")} value={t("areaSettings.times", {count: value.repeat})}>
                <Slider min={R.repeat.min} max={R.repeat.max} step={1} dots disabled={disabled}
                        value={value.repeat} onChange={(v: number) => set("repeat", v)}/>
            </Field>
        </div>
    );
};
