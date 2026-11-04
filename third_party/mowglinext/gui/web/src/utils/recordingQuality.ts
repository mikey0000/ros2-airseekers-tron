// RTK quality of recorded boundary vertices (mower_mission recording phase, served by
// GET /mowglinext/recording-quality): each recorded area, no-go zone or path keeps the
// GNSS fix type and hdop of every saved vertex, and the map colours the vertices so the
// owner can see which corners were recorded without RTK fixed.
import type {Feature, FeatureCollection, Position} from "geojson";
import type {Map as MapType} from "../types/ros.ts";

export interface RecordingQualityVertex {
    x: number;
    y: number;
    /** GnssStatus FIX_TYPE_*: 3 RTK fixed, 2 RTK float, 0/1/4 worse; null = no sample. */
    fix_type: number | null;
    hdop: number | null;
    fixed: boolean;
}

export interface RecordingQuality {
    name: string;
    kind: string;
    saved: number;
    require_fixed: boolean;
    not_fixed: number;
    vertices: RecordingQualityVertex[];
}

export type VertexQualityLevel = "fixed" | "float" | "none";

const FIX_TYPE_RTK_FLOAT = 2;

/** Vertex colours: RTK fixed, RTK float, anything worse (or no sample). */
export const VERTEX_QUALITY_COLORS: Record<VertexQualityLevel, string> = {
    fixed: "#2f9e44",
    float: "#f59f00",
    none: "#e03131",
};

export function vertexQualityLevel(v: Pick<RecordingQualityVertex, "fixed" | "fix_type">): VertexQualityLevel {
    if (v.fixed) return "fixed";
    return v.fix_type === FIX_TYPE_RTK_FLOAT ? "float" : "none";
}

/** Response body -> recordings; malformed entries and vertices are dropped. */
export function parseRecordingQuality(raw: unknown): RecordingQuality[] {
    const list = (raw as {recordings?: unknown} | null | undefined)?.recordings;
    if (!Array.isArray(list)) return [];
    const finite = (n: unknown): n is number => typeof n === "number" && Number.isFinite(n);
    return list.flatMap((r): RecordingQuality[] => {
        if (!r || typeof r !== "object" || typeof (r as {name?: unknown}).name !== "string") return [];
        const o = r as Record<string, unknown>;
        const vertices = (Array.isArray(o.vertices) ? o.vertices : []).flatMap((v): RecordingQualityVertex[] => {
            const p = v as Record<string, unknown> | null;
            if (!p || !finite(p.x) || !finite(p.y)) return [];
            return [{
                x: p.x, y: p.y,
                fix_type: finite(p.fix_type) ? p.fix_type : null,
                hdop: finite(p.hdop) ? p.hdop : null,
                fixed: p.fixed === true,
            }];
        });
        return [{
            name: o.name as string,
            kind: typeof o.kind === "string" ? o.kind : "",
            saved: finite(o.saved) ? o.saved : 0,
            require_fixed: o.require_fixed === true,
            not_fixed: finite(o.not_fixed) ? o.not_fixed : vertices.filter((v) => !v.fixed).length,
            vertices,
        }];
    });
}

/** Names of the areas, no-go zones and paths in the map (stale recordings are not drawn). */
export function mapItemNames(map: MapType | undefined): Set<string> {
    const names = new Set<string>();
    const add = (n: unknown) => { if (typeof n === "string" && n.trim()) names.add(n.trim()); };
    for (const a of [...(map?.working_area ?? []), ...(map?.navigation_areas ?? [])]) {
        add(a.name);
        for (const o of a.obstacle_info ?? []) add(o.name);
    }
    return names;
}

/**
 * Point features for the recorded vertices of the items still in the map. With
 * `showFixed` false (view mode) only the vertices without RTK fixed are drawn.
 * `toLonLat` converts map-frame metres to display lon/lat.
 */
export function recordingQualityFeatures(
    recordings: RecordingQuality[],
    names: Set<string>,
    toLonLat: (x: number, y: number) => Position,
    showFixed: boolean,
): FeatureCollection {
    const features: Feature[] = [];
    const seen = new Set<string>();
    // Newest first: an item recorded twice under one name shows its latest recording.
    for (const r of [...recordings].sort((a, b) => b.saved - a.saved)) {
        if (!names.has(r.name.trim()) || seen.has(r.name.trim())) continue;
        seen.add(r.name.trim());
        r.vertices.forEach((v, i) => {
            const level = vertexQualityLevel(v);
            if (level === "fixed" && !showFixed) return;
            features.push({
                type: "Feature",
                id: `rq-${r.name}-${i}`,
                properties: {level, color: VERTEX_QUALITY_COLORS[level], name: r.name,
                    hdop: v.hdop ?? -1},
                geometry: {type: "Point", coordinates: toLonLat(v.x, v.y)},
            });
        });
    }
    return {type: "FeatureCollection", features};
}
