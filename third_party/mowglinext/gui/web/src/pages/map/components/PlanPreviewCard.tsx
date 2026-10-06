import {Button, Spin} from "antd";
import {CloseOutlined} from "@ant-design/icons";
import React from "react";
import {useTranslation} from "react-i18next";
import type {PlanPreviewSummary} from "../../../hooks/usePlanPreview.ts";
import AsyncButton from "../../../components/AsyncButton.tsx";

export interface PlanPreviewColors {
    ring: string;
    swath: string;
    transit: string;
}

interface Props {
    summary: PlanPreviewSummary;
    colors: PlanPreviewColors;
    onClear: () => Promise<void>;
    onDismiss: () => void;
    style?: React.CSSProperties;
}

const Swatch = ({color, dashed}: {color: string; dashed?: boolean}) => (
    <span style={{display: "inline-block", width: 16, height: 0, borderTop: `3px ${dashed ? "dashed" : "solid"} ${color}`, verticalAlign: "middle", marginRight: 4}}/>
);

const fmtM = (v: number | null | undefined) => (v === null || v === undefined ? "–" : `${v.toFixed(2)} m`);

/** Summary of the last plan preview: counts, length, boundary inset, legend. */
export const PlanPreviewCard: React.FC<Props> = ({summary, colors, onClear, onDismiss, style}) => {
    const {t} = useTranslation();
    const title = summary.area < 0
        ? t("planPreview.titleAll")
        : t("planPreview.titleArea", {name: summary.areas[0]?.name || `#${summary.area}`});
    return (
        <div data-testid="plan-preview-card" style={{fontSize: 13, lineHeight: 1.5, ...style}}>
            <div style={{display: "flex", alignItems: "center", gap: 8, marginBottom: 4}}>
                <strong style={{flex: 1}}>{title}</strong>
                <Button size="small" type="text" icon={<CloseOutlined/>} aria-label={t("planPreview.hide")} onClick={onDismiss}/>
            </div>
            {summary.status === "planning" && (
                <div><Spin size="small"/> {t("planPreview.planning")}</div>
            )}
            {summary.status === "failed" && (
                <div style={{color: "#ff6b6b"}}>{t("planPreview.failed", {message: summary.message})}</div>
            )}
            {summary.status === "ok" && (
                <>
                    <table style={{borderSpacing: "8px 0", marginLeft: -8}}>
                        <tbody>
                        <tr><td><Swatch color={colors.ring}/>{t("planPreview.rings")}</td><td data-testid="plan-preview-rings">{summary.rings}</td></tr>
                        <tr><td><Swatch color={colors.swath}/>{t("planPreview.swaths")}</td><td data-testid="plan-preview-swaths">{summary.swaths}</td></tr>
                        <tr><td><Swatch color={colors.transit} dashed/>{t("planPreview.transits")}</td><td>{summary.transits.length}</td></tr>
                        <tr><td>{t("planPreview.length")}</td><td data-testid="plan-preview-length">{summary.length_m.toFixed(1)} m</td></tr>
                        <tr><td>{t("planPreview.subPaths")}</td><td>{summary.sub_paths}</td></tr>
                        <tr><td>{t("planPreview.inset")}</td><td data-testid="plan-preview-inset">{fmtM(summary.inset_m)}</td></tr>
                        </tbody>
                    </table>
                    {summary.areas.length > 1 && (
                        <ul style={{margin: "4px 0 0", paddingLeft: 16}}>
                            {summary.areas.map((a) => (
                                <li key={a.index}>
                                    {a.name || `#${a.index}`}: {a.error
                                        ? t("planPreview.areaFailed", {message: a.error})
                                        : t("planPreview.areaLine", {rings: a.rings, swaths: a.swaths, length: a.length_m.toFixed(1), mode: a.path_mode})}
                                </li>
                            ))}
                        </ul>
                    )}
                    {summary.areas.length === 1 && summary.areas[0].path_mode && (
                        <div style={{opacity: 0.75}}>{t("planPreview.mode", {mode: summary.areas[0].path_mode})}</div>
                    )}
                </>
            )}
            {summary.status !== "planning" && (
                <AsyncButton size="small" style={{marginTop: 6}} onAsyncClick={onClear}>
                    {t("planPreview.clear")}
                </AsyncButton>
            )}
        </div>
    );
};
