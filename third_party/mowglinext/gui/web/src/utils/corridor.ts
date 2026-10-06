/**
 * Corridor ("path" / vendor "channel") geometry for the map editor.
 *
 * A path is a polyline the operator clicks on the map; it is saved as a
 * navigation area (is_navigation_area=true) by buffering the polyline into a
 * polygon of the chosen width. The stack frees navigation areas in the Nav2
 * navigation mask, so the robot may transit through the corridor but never
 * mows it.
 *
 * All buffering happens in local metres (x east, y north) using the same
 * equirectangular projection as the rest of the map (utils/map.tsx), so the
 * width is true metres regardless of latitude. The buffer is the union of one
 * rectangle per segment (flat end caps) plus a join piece at every interior
 * vertex — a disc for "round" joins, a bevel triangle for "flat" joins —
 * which is robust for any turn angle, unlike offset-curve construction.
 */
import union from "@turf/union";
import {featureCollection, polygon as turfPolygon} from "@turf/helpers";
import type {Feature, MultiPolygon, Polygon, Position} from "geojson";
import {itranspose, transpose} from "./map.tsx";

export type XY = [number, number];
export type CorridorJoin = "round" | "flat";

export interface CorridorOptions {
    /** Join style at interior vertices. Default "round". */
    join?: CorridorJoin;
    /** Segments used to approximate a full circle for round joins. Default 24. */
    arcSegments?: number;
    /**
     * End caps of an open path. "square" (default, like the vendor import in
     * mower_map) extends each end by half the width, so an end placed on an
     * area outline overlaps the area by half a width; "flat" stops at the end.
     */
    cap?: "square" | "flat";
}

/** Extend the first and last vertex outward along their segment by `d`. */
export function extendEnds(points: XY[], d: number): XY[] {
    if (points.length < 2 || !(d > 0)) return points;
    const out = points.map((p) => [p[0], p[1]] as XY);
    const [a, b] = [out[0], out[1]];
    const l0 = dist(a, b) || 1;
    out[0] = [a[0] - (b[0] - a[0]) / l0 * d, a[1] - (b[1] - a[1]) / l0 * d];
    const n = out.length;
    const [y, z] = [out[n - 2], out[n - 1]];
    const l1 = dist(y, z) || 1;
    out[n - 1] = [z[0] + (z[0] - y[0]) / l1 * d, z[1] + (z[1] - y[1]) / l1 * d];
    return out;
}

/** Default corridor width (m) — a bit wider than the robot footprint. */
export const DEFAULT_PATH_WIDTH_M = 0.7;   // 2 x 0.35 m, the vendor channel half-width
export const MIN_PATH_WIDTH_M = 0.5;
export const MAX_PATH_WIDTH_M = 3.0;
/** An end within this distance of an area outline / the dock is snapped to it. */
export const PATH_SNAP_TOLERANCE_M = 0.5;
/** Same value as mower_docking `approach_distance` / vendor `undock_point`. */
export const DOCK_APPROACH_DISTANCE_M = 0.8;

const EPS = 1e-6;

const dist = (a: XY, b: XY) => Math.hypot(a[0] - b[0], a[1] - b[1]);

/** Drop consecutive duplicate vertices (double clicks produce them). */
export function dedupeXY(points: XY[], eps = 1e-3): XY[] {
    const out: XY[] = [];
    for (const p of points) {
        if (!Number.isFinite(p[0]) || !Number.isFinite(p[1])) continue;
        if (out.length === 0 || dist(out[out.length - 1], p) > eps) out.push([p[0], p[1]]);
    }
    return out;
}

function closeRing(ring: XY[]): XY[] {
    return [...ring, ring[0]];
}

function segmentRect(a: XY, b: XY, half: number): XY[] {
    const len = dist(a, b);
    const nx = -(b[1] - a[1]) / len * half;
    const ny = (b[0] - a[0]) / len * half;
    // Counter-clockwise when walking a->b with the left normal first.
    return closeRing([
        [a[0] - nx, a[1] - ny],
        [b[0] - nx, b[1] - ny],
        [b[0] + nx, b[1] + ny],
        [a[0] + nx, a[1] + ny],
    ]);
}

function disc(c: XY, half: number, segments: number): XY[] {
    const ring: XY[] = [];
    for (let i = 0; i < segments; i++) {
        const a = (2 * Math.PI * i) / segments;
        ring.push([c[0] + half * Math.cos(a), c[1] + half * Math.sin(a)]);
    }
    return closeRing(ring);
}

/** Bevel (flat) join: fills the outer wedge between two segment rectangles. */
function bevel(prev: XY, v: XY, next: XY, half: number): XY[] | null {
    const d1: XY = [(v[0] - prev[0]) / dist(prev, v), (v[1] - prev[1]) / dist(prev, v)];
    const d2: XY = [(next[0] - v[0]) / dist(v, next), (next[1] - v[1]) / dist(v, next)];
    const cross = d1[0] * d2[1] - d1[1] * d2[0];
    if (Math.abs(cross) < EPS) return null; // straight through: rectangles already meet
    // Outer side is to the right of a left turn (cross > 0) and vice versa.
    const s = cross > 0 ? -1 : 1;
    const n1: XY = [-d1[1] * half * s, d1[0] * half * s];
    const n2: XY = [-d2[1] * half * s, d2[0] * half * s];
    return closeRing([
        [v[0], v[1]],
        [v[0] + n1[0], v[1] + n1[1]],
        [v[0] + n2[0], v[1] + n2[1]],
    ]);
}

function unionRings(pieces: XY[][]): XY[][][] {
    if (pieces.length === 0) return [];
    const feats = pieces.map((r) => turfPolygon([r as Position[]]));
    const merged: Feature<Polygon | MultiPolygon> | null =
        feats.length === 1 ? feats[0] : union(featureCollection(feats));
    if (!merged) return [];
    const polys = merged.geometry.type === "Polygon"
        ? [merged.geometry.coordinates]
        : merged.geometry.coordinates;
    return polys.map((rings) => rings.map((r) => r.map((p) => [p[0], p[1]] as XY)));
}

function bufferOpen(points: XY[], half: number, join: CorridorJoin, arcSegments: number,
                    endDiscs: boolean): XY[][][] {
    const pieces: XY[][] = [];
    for (let i = 0; i + 1 < points.length; i++) pieces.push(segmentRect(points[i], points[i + 1], half));
    for (let i = 1; i + 1 < points.length; i++) {
        if (join === "round") pieces.push(disc(points[i], half, arcSegments));
        else {
            const b = bevel(points[i - 1], points[i], points[i + 1], half);
            if (b) pieces.push(b);
        }
    }
    if (endDiscs) {
        pieces.push(disc(points[0], half, arcSegments));
        pieces.push(disc(points[points.length - 1], half, arcSegments));
    }
    return unionRings(pieces);
}

/**
 * Buffer a polyline (local metres) into corridor polygons of total `width`.
 *
 * Returns hole-free polygons (outer rings only, closed, counter-clockwise or
 * as produced by the union). Map-server navigation areas carry no holes, so:
 *  - a closed loop (last vertex within half a width of the first) is split
 *    into two overlapping open halves whose union is the ring corridor;
 *  - any hole an open path still encloses (a U that touches itself) is
 *    filled, i.e. the corridor errs on the side of more drivable space.
 * Returns [] for fewer than two distinct vertices or a non-positive width.
 */
export function bufferPolyline(input: XY[], width: number, opts: CorridorOptions = {}): XY[][][] {
    const join = opts.join ?? "round";
    const arcSegments = Math.max(8, opts.arcSegments ?? 24);
    const half = width / 2;
    let points = dedupeXY(input);
    if (!(half > 0) || points.length < 2) return [];

    const closed = points.length >= 4 && dist(points[0], points[points.length - 1]) <= half;
    let parts: XY[][];
    if (closed) {
        points = [...points.slice(0, -1), points[0]];
        const k = Math.floor((points.length - 1) / 2);
        parts = [points.slice(0, k + 1), points.slice(k)];
    } else {
        parts = [(opts.cap ?? "square") === "square" ? extendEnds(points, half) : points];
    }
    const out: XY[][][] = [];
    for (const part of parts) {
        // Seams of a split loop get discs so the two halves join seamlessly.
        for (const poly of bufferOpen(part, half, join, arcSegments, closed)) out.push([poly[0]]);
    }
    return out;
}

/** Signed-area magnitude of a closed ring (m²). */
export function ringArea(ring: XY[]): number {
    let a = 0;
    for (let i = 0; i + 1 < ring.length; i++) a += ring[i][0] * ring[i + 1][1] - ring[i + 1][0] * ring[i][1];
    return Math.abs(a) / 2;
}

// ---------------------------------------------------------------------------
// Dock snapping
// ---------------------------------------------------------------------------

export interface DockPose { x: number; y: number; heading: number }

/** Dock approach point: `distance` metres along the dock yaw (mower_docking convention). */
export function dockApproachPoint(dock: DockPose, distance = DOCK_APPROACH_DISTANCE_M): XY {
    return [dock.x + distance * Math.cos(dock.heading), dock.y + distance * Math.sin(dock.heading)];
}

/**
 * Snap one end of a path to the dock: the end nearer the dock gets the
 * approach point and then the dock itself, so the corridor arrives straight
 * along the dock axis. Points already at the approach/dock are not duplicated.
 */
export function snapPathToDock(points: XY[], dock: DockPose, distance = DOCK_APPROACH_DISTANCE_M): XY[] {
    const pts = dedupeXY(points);
    if (pts.length === 0) return pts;
    const d: XY = [dock.x, dock.y];
    const approach = dockApproachPoint(dock, distance);
    const reversed = dist(pts[0], d) < dist(pts[pts.length - 1], d);
    const path = reversed ? [...pts].reverse() : [...pts];
    while (path.length > 1 && (dist(path[path.length - 1], d) < 0.05 || dist(path[path.length - 1], approach) < 0.05)) {
        path.pop();
    }
    path.push(approach, d);
    const result = dedupeXY(path);
    return reversed ? result.reverse() : result;
}

/** Closest point to `p` on the boundary of `ring` (closed or open ring). */
export function closestPointOnRing(p: XY, ring: XY[]): { point: XY; distance: number } | null {
    if (ring.length < 2) return null;
    let best: { point: XY; distance: number } | null = null;
    const n = ring.length;
    for (let i = 0; i < n; i++) {
        const a = ring[i];
        const b = ring[(i + 1) % n];
        const abx = b[0] - a[0], aby = b[1] - a[1];
        const len2 = abx * abx + aby * aby;
        const t = len2 < EPS ? 0 : Math.max(0, Math.min(1, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / len2));
        const q: XY = [a[0] + t * abx, a[1] + t * aby];
        const dq = dist(p, q);
        if (!best || dq < best.distance) best = {point: q, distance: dq};
    }
    return best;
}

function insideRing(p: XY, ring: XY[]): boolean {
    let inside = false;
    for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
        const [xi, yi] = ring[i];
        const [xj, yj] = ring[j];
        if ((yi > p[1]) !== (yj > p[1]) && p[0] < ((xj - xi) * (p[1] - yi)) / (yj - yi + 1e-12) + xi) inside = !inside;
    }
    return inside;
}

/**
 * Polyline for "connect nearest area to dock": dock -> approach point ->
 * closest point on the nearest area outline, extended `overlap` metres into
 * the area so the corridor overlaps it instead of touching along a hairline.
 * Returns null when there are no areas or the approach point already lies
 * inside one (nothing to connect).
 */
export function dockConnectorPolyline(dock: DockPose, areaRings: XY[][], overlap = 0.3,
                                      distance = DOCK_APPROACH_DISTANCE_M): { points: XY[]; areaIndex: number; gap: number } | null {
    const approach = dockApproachPoint(dock, distance);
    let best: { point: XY; distance: number; index: number } | null = null;
    areaRings.forEach((ring, index) => {
        if (ring.length < 3) return;
        const c = closestPointOnRing(approach, ring);
        if (c && (!best || c.distance < best.distance)) best = {...c, index};
    });
    if (!best) return null;
    const b = best as { point: XY; distance: number; index: number };
    if (insideRing(approach, areaRings[b.index])) return null;
    const ux = (b.point[0] - approach[0]) / (b.distance || 1);
    const uy = (b.point[1] - approach[1]) / (b.distance || 1);
    const end: XY = [b.point[0] + ux * overlap, b.point[1] + uy * overlap];
    return {points: dedupeXY([[dock.x, dock.y], approach, end]), areaIndex: b.index, gap: b.distance};
}

// ---------------------------------------------------------------------------
// End snapping (areas / dock approach pose)
// ---------------------------------------------------------------------------

/** How one end of a path is connected. "none" = touches nothing (hint shown). */
export type PathEndSnap = "dock" | "area" | "inside" | "none";

export interface SnappedPath {
    points: XY[];
    start: PathEndSnap;
    end: PathEndSnap;
}

/** True when both ends reach an area or the dock. */
export const isPathConnected = (s: Pick<SnappedPath, "start" | "end">) => s.start !== "none" && s.end !== "none";

function snapEnd(p: XY, areaRings: XY[][], dock: DockPose | null, tol: number): { point: XY; snap: PathEndSnap } {
    if (dock) {
        const approach = dockApproachPoint(dock);
        if (dist(p, approach) <= tol || dist(p, [dock.x, dock.y]) <= tol) return {point: approach, snap: "dock"};
    }
    let best: { point: XY; distance: number } | null = null;
    for (const ring of areaRings) {
        if (ring.length < 3) continue;
        if (insideRing(p, ring)) return {point: p, snap: "inside"};
        const c = closestPointOnRing(p, ring);
        if (c && (!best || c.distance < best.distance)) best = c;
    }
    if (best && best.distance <= tol) return {point: best.point, snap: "area"};
    return {point: p, snap: "none"};
}

/**
 * Snap both ends of a path (local metres): an end within `tol` of the dock
 * (or its approach point) moves to the dock approach pose; otherwise an end
 * within `tol` of an area outline moves onto the outline (the square cap then
 * overlaps the area by half a width). An end already inside an area stays.
 */
export function snapPathEnds(input: XY[], areaRings: XY[][], dock: DockPose | null,
                             tol = PATH_SNAP_TOLERANCE_M): SnappedPath {
    const points = dedupeXY(input);
    if (points.length < 2) return {points, start: "none", end: "none"};
    const s = snapEnd(points[0], areaRings, dock, tol);
    const e = snapEnd(points[points.length - 1], areaRings, dock, tol);
    const out = [...points];
    out[0] = s.point;
    out[out.length - 1] = e.point;
    return {points: dedupeXY(out), start: s.snap, end: e.snap};
}

// ---------------------------------------------------------------------------
// lon/lat <-> local metres (display offset cancels: both directions use 0)
// ---------------------------------------------------------------------------

export function lonLatToLocal(datum: [number, number, number], p: Position): XY {
    return itranspose(0, 0, datum, p[1], p[0]);
}

export function localToLonLat(datum: [number, number, number], p: XY): Position {
    return transpose(0, 0, datum, p[1], p[0]);
}

/** Buffer a lon/lat polyline into lon/lat corridor polygons (GeoJSON rings). */
export function bufferLonLatPolyline(datum: [number, number, number], coords: Position[], width: number,
                                     opts?: CorridorOptions): Polygon[] {
    const local = coords.map((c) => lonLatToLocal(datum, c));
    return bufferPolyline(local, width, opts).map((rings) => ({
        type: "Polygon",
        coordinates: rings.map((r) => r.map((p) => localToLonLat(datum, p))),
    }));
}
