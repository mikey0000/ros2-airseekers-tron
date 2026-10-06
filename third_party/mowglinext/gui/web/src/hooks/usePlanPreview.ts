// Plan preview: the mission plans an area (or all areas) with its current
// per-area settings, exactly like a mission's PLANNING step, WITHOUT mowing.
// POST /mowglinext/plan/preview?area=N only starts it; the plan arrives on the
// "path" topic (/coverage/full_plan) and this summary on "previewSummary".
import {Api} from "../api/Api.ts";
import {useTopic} from "./useTopic.ts";
import {parseStringMsgJson} from "../utils/areaSettings.ts";

export type PlanPreviewStatus = "planning" | "ok" | "failed" | "cleared";

export interface PlanPreviewArea {
    index: number;
    name: string;
    path_mode: string;
    headland_rings: number;
    rings: number;
    swaths: number;
    length_m: number;
    sub_paths: number;
    inset_m: number | null;
    mow_angle_deg?: number;
    runs?: number;
    error?: string;
}

export interface PlanPreviewSegment {
    type: "ring" | "swath";
    points: [number, number][];
}

export interface PlanPreviewSummary {
    id: number;
    status: PlanPreviewStatus;
    area: number;
    message: string;
    areas: PlanPreviewArea[];
    rings: number;
    swaths: number;
    length_m: number;
    sub_paths: number;
    inset_m: number | null;
    segments: PlanPreviewSegment[];
    transits: [number, number][][];
}

const num = (v: unknown, d = 0): number => (typeof v === "number" && Number.isFinite(v) ? v : d);
const numOrNull = (v: unknown): number | null => (typeof v === "number" && Number.isFinite(v) ? v : null);

/** Decode the ~/preview_summary JSON (std_msgs/String) defensively. */
export function parsePlanPreview(raw: unknown): PlanPreviewSummary | null {
    const d = parseStringMsgJson(raw);
    if (!d) return null;
    const status = d.status;
    if (status !== "planning" && status !== "ok" && status !== "failed" && status !== "cleared") return null;
    const arr = (v: unknown): unknown[] => (Array.isArray(v) ? v : []);
    return {
        id: num(d.id),
        status,
        area: num(d.area, -1),
        message: typeof d.message === "string" ? d.message : "",
        areas: arr(d.areas).filter((a): a is Record<string, unknown> => !!a && typeof a === "object").map((a) => ({
            index: num(a.index),
            name: typeof a.name === "string" ? a.name : "",
            path_mode: typeof a.path_mode === "string" ? a.path_mode : "",
            headland_rings: num(a.headland_rings),
            rings: num(a.rings),
            swaths: num(a.swaths),
            length_m: num(a.length_m),
            sub_paths: num(a.sub_paths),
            inset_m: numOrNull(a.inset_m),
            mow_angle_deg: numOrNull(a.mow_angle_deg) ?? undefined,
            runs: numOrNull(a.runs) ?? undefined,
            error: typeof a.error === "string" ? a.error : undefined,
        })),
        rings: num(d.rings),
        swaths: num(d.swaths),
        length_m: num(d.length_m),
        sub_paths: num(d.sub_paths),
        inset_m: numOrNull(d.inset_m),
        segments: arr(d.segments).filter((s): s is PlanPreviewSegment =>
            !!s && typeof s === "object" && Array.isArray((s as PlanPreviewSegment).points)
            && ((s as PlanPreviewSegment).type === "ring" || (s as PlanPreviewSegment).type === "swath")),
        transits: arr(d.transits).filter((t): t is [number, number][] => Array.isArray(t) && t.length >= 2),
    };
}

/** Latest plan preview summary (null until one was published). */
export function usePlanPreview(enabled = true): PlanPreviewSummary | null {
    const {data} = useTopic<PlanPreviewSummary | null>("previewSummary", null, {
        select: (raw) => parsePlanPreview(raw) ?? undefined,
        enabled,
    });
    return enabled ? data : null;
}

/** Mission states in which the mission accepts a preview (mission_fsm PREVIEW_PHASES). */
export const PREVIEW_STATES = ["IDLE", "IDLE_DOCKED", "CHARGING", "MOWING_COMPLETE", "MOWING_INCOMPLETE"];

export function canPreviewPlan(stateName?: string): boolean {
    return !!stateName && PREVIEW_STATES.includes(stateName);
}

interface HttpLike {status?: number; error?: {error?: string; message?: string}; data?: {message?: string}}

function previewError(e: unknown): Error {
    const h = e as HttpLike;
    return new Error(h?.error?.message || h?.error?.error || h?.data?.message || (e as Error)?.message || String(e));
}

/** Start a preview; area -1 = all areas. Rejects with the mission's refusal reason. */
export async function requestPlanPreview(api: Api<unknown>, area: number): Promise<void> {
    try {
        await api.request({path: `/mowglinext/plan/preview?area=${area}`, method: "POST", format: "json"});
    } catch (e) {
        throw previewError(e);
    }
}

/** Clear the drawn preview. */
export async function clearPlanPreview(api: Api<unknown>): Promise<void> {
    try {
        await api.request({path: "/mowglinext/plan/preview", method: "DELETE", format: "json"});
    } catch (e) {
        throw previewError(e);
    }
}
