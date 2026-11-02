import {App, Button, Space, Tag} from "antd";
import {CloseOutlined} from "@ant-design/icons";
import React from "react";
import {formatAge, TERRAIN_STATUS_COLORS, type TerrainArea, type TerrainCluster, type TerrainSummary} from "../../../utils/terrain.ts";
import type {TerrainAction} from "../../../hooks/useTerrain.ts";
import {useDismissOnOutside} from "../hooks/useDismissOnOutside.ts";

interface Props {
    summary: TerrainSummary;
    selected: {area: number; id: number} | null;
    onSelect: (sel: {area: number; id: number} | null) => void;
    /** Resolves on success, rejects with a readable Error. */
    onAction: (action: TerrainAction, areaIndex: number, clusterId?: number) => Promise<void>;
    onDismiss: () => void;
    style?: React.CSSProperties;
}

const fmtDeg = (v: number | null | undefined, digits = 1) =>
    v === null || v === undefined || !isFinite(v) ? "–" : `${v.toFixed(digits)}°`;

/** Plain words for the incident kinds the mission and map server record. */
const KIND_TEXT: Record<string, string> = {
    follow_abort: "couldn't follow the path",
    subpath_failed: "strip failed",
    transit_abort: "drive between strips failed",
    blocked_pivot: "blocked while turning",
    nogo_pivot: "turn blocked by a no-go zone",
    nogo_replan: "re-routed around a no-go zone",
    spin_drift: "wheels slipped while turning",
    bumper_skip: "bumper hit, skipped",
    sweep_blocked_skip: "blocked, skipped",
    stretch_blocked_static: "blocked by an obstacle",
    edge_blocked: "edge blocked",
    edge_blocked_total: "edge blocked",
    edge_detour: "detour along the edge",
    detour: "detour",
    leg_loop: "went round in circles",
    boundary: "touched the boundary",
    boundary_lethal: "crossed the boundary",
    marks_excluded: "marked spot left out",
};

export const terrainKindText = (kind: string): string => KIND_TEXT[kind] ?? kind.replace(/_/g, " ");

const kindsText = (kinds: Record<string, number>) =>
    Object.entries(kinds).map(([k, n]) => (n > 1 ? `${terrainKindText(k)} (${n}×)` : terrainKindText(k))).join(", ");

const incidentsText = (n: number) => (n === 1 ? "1 incident" : `${n} incidents`);

/** Terrain memory: incident clusters (keep-out / confirm / dismiss) and per-area slope info. */
export const TerrainCard: React.FC<Props> = ({summary, selected, onSelect, onAction, onDismiss, style}) => {
    const {modal, message} = App.useApp();
    const [busy, setBusy] = React.useState(false);
    const rootRef = React.useRef<HTMLDivElement>(null);
    // Escape or a tap on the map closes the card (it floats over the map).
    useDismissOnOutside(rootRef, onDismiss);

    const run = async (action: TerrainAction, area: number, id?: number) => {
        setBusy(true);
        try {
            await onAction(action, area, id);
            if (action !== "confirm" && action !== "clear") onSelect(null);
        } catch (e) {
            message.error((e as Error).message);
        } finally {
            setBusy(false);
        }
    };

    const keepout = (area: number, c: TerrainCluster) => modal.confirm({
        title: "No-go zone",
        content: "Make this spot a no-go zone? The mower will no longer mow there.",
        okText: "Create no-go zone",
        okButtonProps: {danger: true},
        onOk: () => run("keepout", area, c.id),
    });

    const clearAll = () => modal.confirm({
        title: "Clear terrain memory",
        content: "Forget all recorded traction problems, incident clusters and slope data?",
        okText: "Clear",
        okButtonProps: {danger: true},
        onOk: () => run("clear", 255),
    });

    const areas = summary.areas ?? [];
    const clusterRows = areas.flatMap((a) => (a.clusters ?? []).map((c) => ({a, c})));

    return (
        <div ref={rootRef} data-testid="terrain-card" role="dialog" aria-label="Terrain"
             style={{fontSize: 13, lineHeight: 1.5, maxHeight: '50vh', overflowY: 'auto', ...style}}>
            <div style={{display: "flex", alignItems: "center", gap: 8, marginBottom: 4}}>
                <strong style={{flex: 1}}>Terrain</strong>
                <Button type="text" icon={<CloseOutlined/>} aria-label="Hide terrain card" onClick={onDismiss}
                        style={{width: 36, height: 36}}/>
            </div>
            {clusterRows.length === 0 && <div style={{opacity: 0.7}}>No problem spots recorded.</div>}
            {clusterRows.map(({a, c}) => {
                const isSel = selected?.area === a.area_index && selected?.id === c.id;
                return (
                    <div key={`${a.area_index}-${c.id}`}
                         data-testid="terrain-cluster"
                         onClick={() => onSelect(isSel ? null : {area: a.area_index, id: c.id})}
                         style={{cursor: "pointer", padding: "4px 6px", marginBottom: 4, borderRadius: 8,
                             border: `1px solid ${isSel ? TERRAIN_STATUS_COLORS[c.status] ?? "#999" : "transparent"}`,
                             background: isSel ? "rgba(255,255,255,0.06)" : undefined}}>
                        <div>
                            <Tag color={TERRAIN_STATUS_COLORS[c.status] ?? "default"}>{c.status}</Tag>
                            <strong>{a.area || `Area ${a.area_index + 1}`}</strong> · {incidentsText(c.n)}{c.last_t ? ` · ${formatAge(c.last_t)} ago` : ""}
                        </div>
                        <div style={{opacity: 0.8}}>{kindsText(c.kinds)}</div>
                        {isSel && (
                            <Space size={4} style={{marginTop: 4}} onClick={(e) => e.stopPropagation()}>
                                <Button size="small" danger disabled={busy} onClick={() => { keepout(a.area_index, c); }}>No-go zone</Button>
                                {c.status !== "confirmed" && (
                                    <Button size="small" disabled={busy} onClick={() => void run("confirm", a.area_index, c.id)}>Confirm</Button>
                                )}
                                <Button size="small" disabled={busy} onClick={() => void run("dismiss", a.area_index, c.id)}>Dismiss</Button>
                            </Space>
                        )}
                    </div>
                );
            })}
            {areas.some((a) => a.slope) && (
                <table style={{borderSpacing: "8px 0", marginLeft: -8, marginTop: 4}}>
                    <tbody>
                    {areas.filter((a): a is TerrainArea & {slope: NonNullable<TerrainArea["slope"]>} => !!a.slope).map((a) => (
                        <tr key={a.area_index}>
                            <td>{a.area || `#${a.area_index}`}</td>
                            <td>slope {fmtDeg(a.slope.slope_deg)} @ {fmtDeg(a.slope.axis_deg, 0)}</td>
                            <td>{a.slope_mode ?? "off"}</td>
                            <td title={a.slope_angle_why}>
                                {a.slope_mow_angle_deg != null ? `→ ${fmtDeg(a.slope_mow_angle_deg, 0)}` : "–"}
                                {a.slope_angle_why ? ` (${a.slope_angle_why})` : ""}
                            </td>
                        </tr>
                    ))}
                    </tbody>
                </table>
            )}
            <div style={{marginTop: 6}}>
                <Button size="small" disabled={busy} onClick={() => { clearAll(); }}>Clear memory</Button>
            </div>
        </div>
    );
};
