// Live mow progress (mower_mission/mow_progress.py): the mission publishes the
// current area's drivable sub-paths once per plan ("missionPlan",
// /behavior_tree_node/mow_plan) and a 1 Hz cursor ("missionProgress",
// /behavior_tree_node/mow_progress). Pose indices in the progress are absolute
// indices into the concatenation of the plan's sub-paths. Pure helpers only:
// parsing, the plan -> coloured stretches mapping and formatting.
import {parseStringMsgJson} from "./areaSettings.ts";

export type XY = [number, number];

export interface MissionPlan {
    plan_id: string;
    area: number;
    subpaths: XY[][];
}

export interface ObstaclePolicy {
    kind?: string;
    class?: string;
    distance_m?: number | null;
    bearing_deg?: number | null;
    wall_time?: number;
    [k: string]: unknown;
}

export interface WhyStopped {
    obstacle?: ObstaclePolicy | null;
    stereo_stale_s?: number | null;
    boundary_violation?: boolean;
    emergency?: boolean;
    lift?: boolean;
    stop_button?: boolean;
    rain?: boolean;
    docked?: boolean;
    critical_nodes_down?: string;
}

export interface MissionProgress {
    plan_id: string | null;
    area: number;
    state: string;
    sub_state: string;
    sub_path: number;
    sub_paths: number;
    pose_index: number;
    total_poses: number;
    mowed_segments: XY[];
    current_segment: XY | null;
    skipped: XY[];
    blade_on: boolean;
    percent: number;
    mowed_m: number;
    remaining_m: number;
    elapsed_s: number | null;
    eta_s: number | null;
    why: WhyStopped;
}

const num = (v: unknown, d = 0): number => (typeof v === "number" && Number.isFinite(v) ? v : d);
const numOrNull = (v: unknown): number | null => (typeof v === "number" && Number.isFinite(v) ? v : null);
const isXY = (p: unknown): p is XY =>
    Array.isArray(p) && p.length >= 2 && Number.isFinite(p[0]) && Number.isFinite(p[1]);
const ranges = (v: unknown): XY[] =>
    (Array.isArray(v) ? v : []).filter(isXY).map(([a, b]) => [a, b] as XY);

export function parseMissionPlan(raw: unknown): MissionPlan | null {
    const d = parseStringMsgJson(raw);
    if (!d || typeof d.plan_id !== "string" || !Array.isArray(d.subpaths)) return null;
    return {
        plan_id: d.plan_id,
        area: num(d.area, -1),
        subpaths: d.subpaths.map((sp: unknown) => (Array.isArray(sp) ? sp.filter(isXY).map(([x, y]) => [x, y] as XY) : [])),
    };
}

export function parseMissionProgress(raw: unknown): MissionProgress | null {
    const d = parseStringMsgJson(raw);
    if (!d || typeof d.state !== "string") return null;
    const why = d.why && typeof d.why === "object" ? d.why as WhyStopped : {};
    return {
        plan_id: typeof d.plan_id === "string" ? d.plan_id : null,
        area: num(d.area, -1),
        state: d.state,
        sub_state: typeof d.sub_state === "string" ? d.sub_state : "",
        sub_path: num(d.sub_path, -1),
        sub_paths: num(d.sub_paths),
        pose_index: num(d.pose_index, -1),
        total_poses: num(d.total_poses),
        mowed_segments: ranges(d.mowed_segments),
        current_segment: isXY(d.current_segment) ? [d.current_segment[0], d.current_segment[1]] : null,
        skipped: ranges(d.skipped),
        blade_on: d.blade_on === true,
        percent: num(d.percent),
        mowed_m: num(d.mowed_m),
        remaining_m: num(d.remaining_m),
        elapsed_s: numOrNull(d.elapsed_s),
        eta_s: numOrNull(d.eta_s),
        why,
    };
}

export type StretchKind = "mowed" | "current" | "remaining" | "skipped";

export interface Stretch {
    kind: StretchKind;
    points: XY[];
}

const inRanges = (i: number, rs: XY[]) => rs.some(([a, b]) => i >= a && i <= b);

/**
 * Split the plan into coloured stretches. Each edge (pose j -> j+1 of one
 * sub-path) is classified by its absolute index: skipped wins over mowed
 * (both ends in a mowed range), then the current segment, then remaining.
 * Consecutive edges of one kind become one polyline; sub-paths are never
 * joined (the jump between them is a Nav2 transit, not a driven line).
 * Returns [] when the progress belongs to another plan.
 */
export function planStretches(plan: MissionPlan | null, progress: MissionProgress | null): Stretch[] {
    if (!plan || !progress || progress.plan_id !== plan.plan_id) return [];
    const out: Stretch[] = [];
    const cur = progress.current_segment;
    let off = 0;
    for (const sp of plan.subpaths) {
        let run: Stretch | null = null;
        for (let j = 0; j + 1 < sp.length; j++) {
            const a = off + j, b = a + 1;
            let kind: StretchKind;
            if (inRanges(a, progress.skipped) && inRanges(b, progress.skipped)) kind = "skipped";
            else if (progress.mowed_segments.some(([s, e]) => a >= s && b <= e)) kind = "mowed";
            else if (cur && a >= cur[0] && b <= cur[1]) kind = "current";
            else kind = "remaining";
            if (run && run.kind === kind) {
                run.points.push(sp[j + 1]);
            } else {
                run = {kind, points: [sp[j], sp[j + 1]]};
                out.push(run);
            }
        }
        off += sp.length;
    }
    return out;
}

/** "1 h 05 min", "12 min", "45 s"; "–" for null. */
export function formatDuration(s: number | null | undefined): string {
    if (s == null || !Number.isFinite(s) || s < 0) return "–";
    const sec = Math.round(s);
    if (sec < 60) return `${sec} s`;
    const min = Math.floor(sec / 60);
    if (min < 60) return `${min} min`;
    return `${Math.floor(min / 60)} h ${String(min % 60).padStart(2, "0")} min`;
}

export interface TrackPoint {
    x: number;
    y: number;
    t: number;
}

/**
 * Append a pose to the robot track: drops points closer than `minStepM` to
 * the last one and everything older than `windowMs`. Returns a new array only
 * when something changed (so it can drive React state).
 */
export function appendTrack(track: TrackPoint[], x: number, y: number, t: number,
                            windowMs = 5 * 60_000, minStepM = 0.05): TrackPoint[] {
    if (!Number.isFinite(x) || !Number.isFinite(y)) return track;
    const last = track[track.length - 1];
    const cutoff = t - windowMs;
    const stale = track.length > 0 && track[0].t < cutoff;
    if (last && Math.hypot(x - last.x, y - last.y) < minStepM && !stale) return track;
    const kept = stale ? track.filter((p) => p.t >= cutoff) : track.slice();
    if (!last || Math.hypot(x - last.x, y - last.y) >= minStepM) kept.push({x, y, t});
    return kept;
}

/** Freshness of an obstacle-policy sample (wall_time in s) in seconds, or null. */
export function ageSeconds(wallTime: unknown, nowMs = Date.now()): number | null {
    return typeof wallTime === "number" && Number.isFinite(wallTime) ? Math.max(0, nowMs / 1000 - wallTime) : null;
}
