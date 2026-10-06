import {OccupancyGrid} from "../types/ros.ts";
import {RasterizedGrid, Rgba, rasterizeOccupancyGrid} from "./occupancyGrid.ts";

/** Scores below this are treated as "fine" and left transparent. */
/** Cluster marker colour by status. */
export const TERRAIN_STATUS_COLORS: Record<string, string> = {open: "#ff9f1a", confirmed: "#ff4d4f"};

export const TERRAIN_MIN_SCORE = 10;

/**
 * Traction score (0..100, 100 = worst / stuck; -1 = no data) -> RGBA.
 * Yellow -> orange -> red ramp, alpha rising with the score.
 */
export function terrainPaint(value: number): Rgba | null {
    if (!(value >= TERRAIN_MIN_SCORE)) return null;
    const t = Math.min(1, (value - TERRAIN_MIN_SCORE) / (100 - TERRAIN_MIN_SCORE));
    // yellow (255,230,0) -> orange (255,140,0) at t=0.5 -> red (230,20,20)
    let r: number, g: number, b: number;
    if (t < 0.5) {
        const k = t / 0.5;
        r = 255; g = 230 - 90 * k; b = 0;
    } else {
        const k = (t - 0.5) / 0.5;
        r = 255 - 25 * k; g = 140 - 120 * k; b = 20 * k;
    }
    const a = 70 + 170 * t;
    return [Math.round(r), Math.round(g), Math.round(b), Math.round(a)];
}

export function rasterizeTerrain(grid: OccupancyGrid): RasterizedGrid | null {
    return rasterizeOccupancyGrid(grid, terrainPaint);
}

export interface TerrainSlope {
    axis_deg: number;
    slope_deg: number;
    p90_slope_deg?: number;
    anisotropy?: number;
    cells?: number;
    coverage?: number;
}

export interface TerrainCluster {
    id: number;
    ids?: number[];
    kinds: Record<string, number>;
    n: number;
    weight: number;
    x: number;
    y: number;
    hull?: [number, number][];
    /** "open" | "confirmed" */
    status: string;
    last_t?: number;
}

export interface TerrainArea {
    area: string;
    area_index: number;
    slope: TerrainSlope | null;
    max_slope_deg?: number;
    slope_mode?: string;
    slope_mow_angle_deg?: number | null;
    slope_angle_why?: string;
    traction_cells_bad?: number;
    traction_max?: number;
    incidents?: number;
    clusters?: TerrainCluster[];
}

export interface TerrainSummary {
    version?: number;
    stamp?: number;
    areas: TerrainArea[];
}

/** Parse the latched terrain_summary std_msgs/String payload; null if unusable. */
export function parseTerrainSummary(raw: string | undefined | null): TerrainSummary | null {
    if (!raw) return null;
    try {
        const v = JSON.parse(raw) as {areas?: unknown} | null;
        if (!v || !Array.isArray(v.areas)) return null;
        return v as TerrainSummary;
    } catch {
        return null;
    }
}

/** Human age of a cluster's last incident ("3 min", "2 h", "4 d"). */
export function formatAge(lastT: number | undefined, nowS: number = Date.now() / 1000): string {
    if (lastT == null || !isFinite(lastT)) return "—";
    const d = Math.max(0, nowS - lastT);
    if (d < 60) return `${Math.round(d)} s`;
    if (d < 3600) return `${Math.round(d / 60)} min`;
    if (d < 86400) return `${Math.round(d / 3600)} h`;
    return `${Math.round(d / 86400)} d`;
}
