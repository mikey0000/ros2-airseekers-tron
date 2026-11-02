import type {Position} from "geojson";
import {
    BoundaryLineFeature,
    MowingAreaFeature,
    NavigationFeature,
    ObstacleFeature,
    type MowingFeature,
    type MowingFeatureBase,
} from "../../../types/map.ts";
import {pointInPolygon} from "../../../utils/map.tsx";

// Same spherical constant as utils/map.tsx (transpose / itranspose).
const METERS_PER_DEG = 6378137.0 * Math.PI / 180.0;

/**
 * What a view-mode map click selects, most specific first: a boundary line
 * (within the click tolerance), a no-go zone, a navigation area / path, a
 * mowing area. A no-go zone always wins over the mowing area it sits in.
 */
export type MapItemKind = "boundary" | "nogo" | "nav" | "area";

export interface MapItem {
    kind: MapItemKind;
    id: string;
}

/** Ground metres per CSS pixel of a Mapbox GL map (512 px tiles) at `lat`, `zoom`. */
export function metresPerPixel(lat: number, zoom: number): number {
    return 40075016.686 * Math.cos(lat * Math.PI / 180) / (512 * Math.pow(2, zoom));
}

/** Local east / north metres of `p` relative to `origin` (both [lon, lat]). */
function localMetres(origin: Position, p: Position): [number, number] {
    const cosLat = Math.cos(origin[1] * Math.PI / 180);
    return [(p[0] - origin[0]) * cosLat * METERS_PER_DEG, (p[1] - origin[1]) * METERS_PER_DEG];
}

/** Ground distance in metres between two [lon, lat] points. */
export function distanceM(a: Position, b: Position): number {
    const [x, y] = localMetres(a, b);
    return Math.hypot(x, y);
}

/** Shortest ground distance in metres from `pt` to the polyline `line` ([lon, lat]). */
export function distanceToPolylineM(pt: Position, line: Position[]): number {
    if (line.length === 0) return Infinity;
    if (line.length === 1) return distanceM(pt, line[0]);
    let best = Infinity;
    for (let i = 1; i < line.length; i++) {
        const [ax, ay] = localMetres(pt, line[i - 1]);
        const [bx, by] = localMetres(pt, line[i]);
        const dx = bx - ax, dy = by - ay;
        const len2 = dx * dx + dy * dy;
        // Projection of the origin (the click) onto segment a-b, clamped to it.
        const u = len2 > 0 ? Math.min(1, Math.max(0, -(ax * dx + ay * dy) / len2)) : 0;
        best = Math.min(best, Math.hypot(ax + u * dx, ay + u * dy));
    }
    return best;
}

/** Planar ring area in degrees² — only used to rank nested polygons. */
function ringSize(ring: Position[]): number {
    let s = 0;
    for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
        s += (ring[j][0] + ring[i][0]) * (ring[j][1] - ring[i][1]);
    }
    return Math.abs(s / 2);
}

/** The smallest polygon of `candidates` containing `pt` (nested areas: the inner one). */
function smallestContaining<T extends MowingFeatureBase>(pt: [number, number], candidates: T[]): T | undefined {
    let best: T | undefined;
    let bestSize = Infinity;
    for (const f of candidates) {
        if (!pointInPolygon(pt, f.geometry.coordinates)) continue;
        const size = ringSize(f.geometry.coordinates[0] ?? []);
        if (size < bestSize) {
            best = f;
            bestSize = size;
        }
    }
    return best;
}

/**
 * Resolve a map click at `pt` ([lon, lat]) to the most specific map item:
 * boundary line within `toleranceM` of the click (nearest wins) > no-go zone
 * > navigation area / path > mowing area > null.
 */
export function hitTestMap(
    pt: [number, number],
    features: Record<string, MowingFeature>,
    toleranceM: number,
): MapItem | null {
    const all = Object.values(features);

    let line: BoundaryLineFeature | undefined;
    let lineDist = Infinity;
    for (const f of all) {
        if (!(f instanceof BoundaryLineFeature)) continue;
        const d = distanceToPolylineM(pt, f.getCoordinates());
        if (d <= toleranceM && d < lineDist) {
            line = f;
            lineDist = d;
        }
    }
    if (line) return {kind: "boundary", id: line.id};

    const nogo = smallestContaining(pt, all.filter((f): f is ObstacleFeature => f instanceof ObstacleFeature));
    if (nogo) return {kind: "nogo", id: nogo.id};
    const nav = smallestContaining(pt, all.filter((f): f is NavigationFeature => f instanceof NavigationFeature));
    if (nav) return {kind: "nav", id: nav.id};
    const area = smallestContaining(pt, all.filter((f): f is MowingAreaFeature => f instanceof MowingAreaFeature));
    if (area) return {kind: "area", id: area.id};
    return null;
}
