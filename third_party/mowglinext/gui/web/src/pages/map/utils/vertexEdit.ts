/**
 * Pure vertex editing helpers for the map editor: merge, delete, insert,
 * Douglas–Peucker simplify and a self-intersection check. Points are
 * [lon, lat]; distances are ground metres on a local equirectangular plane.
 *
 * "Open" vertex lists never repeat the first point. GeoJSON polygon rings
 * are closed (last == first); use openRing / closeRing at the boundary.
 */
import type {Position} from "geojson";

const M_PER_DEG_LAT = 110_540;
const M_PER_DEG_LON_EQ = 111_320;

export const MERGE_DEFAULT_M = 0.15;
export const SIMPLIFY_DEFAULT_M = 0.05;
/** Drag-merge snap radius in screen pixels (desktop / phone). */
export const SNAP_PX_DESKTOP = 12;
export const SNAP_PX_PHONE = 16;

export type VertexEditFailure = "tooFew" | "selfIntersection";
export type VertexEditResult =
    | { ok: true; points: Position[]; removed: number }
    | { ok: false; reason: VertexEditFailure };

export const minVertices = (closed: boolean): number => (closed ? 3 : 2);

/** Drop the closing duplicate of a GeoJSON ring. */
export function openRing(ring: Position[]): Position[] {
    if (ring.length < 2) return ring.slice();
    const a = ring[0], b = ring[ring.length - 1];
    return a[0] === b[0] && a[1] === b[1] ? ring.slice(0, -1) : ring.slice();
}

/** Repeat the first point at the end (GeoJSON polygon ring). */
export function closeRing(points: Position[]): Position[] {
    return points.length === 0 ? [] : [...points, points[0]];
}

/** Project [lon, lat] points to metres around the first point. */
export function toLocalMetres(points: Position[]): [number, number][] {
    if (points.length === 0) return [];
    const [lon0, lat0] = points[0];
    const kx = M_PER_DEG_LON_EQ * Math.cos((lat0 * Math.PI) / 180);
    return points.map((p) => [(p[0] - lon0) * kx, (p[1] - lat0) * M_PER_DEG_LAT]);
}

const dist = (a: [number, number], b: [number, number]) => Math.hypot(a[0] - b[0], a[1] - b[1]);

const cross = (o: [number, number], a: [number, number], b: [number, number]) =>
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);

const onSegment = (p: [number, number], a: [number, number], b: [number, number]) =>
    Math.min(a[0], b[0]) - 1e-9 <= p[0] && p[0] <= Math.max(a[0], b[0]) + 1e-9 &&
    Math.min(a[1], b[1]) - 1e-9 <= p[1] && p[1] <= Math.max(a[1], b[1]) + 1e-9;

function segmentsIntersect(a: [number, number], b: [number, number], c: [number, number], d: [number, number]): boolean {
    const eps = 1e-9;
    const d1 = cross(c, d, a), d2 = cross(c, d, b), d3 = cross(a, b, c), d4 = cross(a, b, d);
    if (((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
        ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))) return true;
    if (Math.abs(d1) <= eps && onSegment(a, c, d)) return true;
    if (Math.abs(d2) <= eps && onSegment(b, c, d)) return true;
    if (Math.abs(d3) <= eps && onSegment(c, a, b)) return true;
    if (Math.abs(d4) <= eps && onSegment(d, a, b)) return true;
    return false;
}

/**
 * True when two non-adjacent edges of the outline touch or cross (or two
 * vertices coincide). `points` is an open vertex list.
 */
export function selfIntersects(points: Position[], closed: boolean): boolean {
    const p = toLocalMetres(points);
    const n = p.length;
    const edges = closed ? n : n - 1;
    if (edges < 2) return false;
    for (let i = 0; i < edges; i++) {
        const a = p[i], b = p[(i + 1) % n];
        for (let j = i + 1; j < edges; j++) {
            const adjacent = j === i + 1 || (closed && i === 0 && j === edges - 1);
            const c = p[j], d = p[(j + 1) % n];
            if (adjacent) {
                // Adjacent edges share one vertex; they only overlap if folded back.
                const shared = j === i + 1 ? b : a;
                const other1 = j === i + 1 ? a : b;
                const other2 = j === i + 1 ? d : c;
                if (Math.abs(cross(shared, other1, other2)) <= 1e-9 &&
                    (other1[0] - shared[0]) * (other2[0] - shared[0]) + (other1[1] - shared[1]) * (other2[1] - shared[1]) > 0) {
                    return true;
                }
                continue;
            }
            if (segmentsIntersect(a, b, c, d)) return true;
        }
    }
    return false;
}

function finish(points: Position[], closed: boolean, removed: number): VertexEditResult {
    if (points.length < minVertices(closed)) return {ok: false, reason: "tooFew"};
    if (selfIntersects(points, closed)) return {ok: false, reason: "selfIntersection"};
    return {ok: true, points, removed};
}

/** Remove vertex `from`, keeping its neighbour `to` where it is (drag-merge). */
export function mergeVertexInto(points: Position[], from: number, closed: boolean): VertexEditResult {
    return deleteVertex(points, from, closed);
}

/** Delete vertex `index`. */
export function deleteVertex(points: Position[], index: number, closed: boolean): VertexEditResult {
    if (index < 0 || index >= points.length) return {ok: true, points: points.slice(), removed: 0};
    return finish(points.filter((_, i) => i !== index), closed, 1);
}

/** Insert the midpoint of edge `edgeStart` → `edgeStart + 1` (wrapping on a ring). */
export function insertMidpoint(points: Position[], edgeStart: number, closed: boolean): Position[] {
    const n = points.length;
    const next = edgeStart + 1;
    if (edgeStart < 0 || (!closed && next >= n) || n < 2) return points.slice();
    const a = points[edgeStart], b = points[next % n];
    const mid: Position = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    return [...points.slice(0, next), mid, ...points.slice(next)];
}

/**
 * Merge consecutive vertices closer than `distM`: the later vertex of a pair
 * is dropped, so the kept one stays where it was. On a ring the last vertex
 * is also compared with the first. Never goes below the minimum count.
 */
export function mergeNearbyVertices(points: Position[], distM: number, closed: boolean): VertexEditResult {
    const local = toLocalMetres(points);
    const min = minVertices(closed);
    const keep: number[] = [];
    for (let i = 0; i < points.length; i++) {
        const prev = keep[keep.length - 1];
        const remaining = keep.length + (points.length - i);
        if (prev !== undefined && dist(local[prev], local[i]) < distM && remaining > min) continue;
        keep.push(i);
    }
    // Open line: keep the true end point, drop the kept one before it instead.
    if (!closed && keep[keep.length - 1] !== points.length - 1 && keep.length >= 2) {
        keep[keep.length - 1] = points.length - 1;
    }
    if (closed && keep.length > min && dist(local[keep[keep.length - 1]], local[keep[0]]) < distM) keep.pop();
    const merged = keep.map((i) => points[i]);
    return finish(merged, closed, points.length - merged.length);
}

function dpKeep(p: [number, number][], first: number, last: number, tol: number, keep: boolean[]) {
    let maxD = -1, idx = -1;
    const a = p[first], b = p[last];
    const len = dist(a, b);
    for (let i = first + 1; i < last; i++) {
        const d = len === 0 ? dist(p[i], a) : Math.abs(cross(a, b, p[i])) / len;
        if (d > maxD) { maxD = d; idx = i; }
    }
    if (idx >= 0 && maxD > tol) {
        keep[idx] = true;
        dpKeep(p, first, idx, tol, keep);
        dpKeep(p, idx, last, tol, keep);
    }
}

/** Douglas–Peucker with tolerance `tolM` metres; rejects too few vertices or a self-crossing result. */
export function simplifyVertices(points: Position[], tolM: number, closed: boolean): VertexEditResult {
    const n = points.length;
    if (n <= minVertices(closed)) return {ok: true, points: points.slice(), removed: 0};
    const p = toLocalMetres(points);
    const keep = new Array<boolean>(n).fill(false);
    keep[0] = true;
    if (closed) {
        // Split the ring at vertex 0 and the vertex farthest from it.
        let far = 1;
        for (let i = 1; i < n; i++) if (dist(p[i], p[0]) > dist(p[far], p[0])) far = i;
        keep[far] = true;
        const ring = [...p, p[0]];
        dpKeep(ring, 0, far, tolM, keep);
        const tail = new Array<boolean>(n + 1).fill(false);
        dpKeep(ring, far, n, tolM, tail);
        for (let i = far + 1; i < n; i++) if (tail[i]) keep[i] = true;
    } else {
        keep[n - 1] = true;
        dpKeep(p, 0, n - 1, tolM, keep);
    }
    const out = points.filter((_, i) => keep[i]);
    return finish(out, closed, n - out.length);
}

/**
 * Drag-merge target: the neighbour of `index` within `snapPx` screen pixels
 * of it, or -1. `px` are the vertices in screen pixels.
 */
export function findSnapNeighbour(px: [number, number][], index: number, closed: boolean, snapPx: number): number {
    const n = px.length;
    const candidates = [index - 1, index + 1]
        .map((i) => (closed ? (i + n) % n : i))
        .filter((i) => i >= 0 && i < n && i !== index);
    let best = -1, bestD = snapPx;
    for (const i of candidates) {
        const d = dist(px[i], px[index]);
        if (d <= bestD) { best = i; bestD = d; }
    }
    return best;
}

/** Apply an open-list operation to every ring of a polygon (outer + holes). */
export function mapPolygonRings(
    rings: Position[][],
    op: (points: Position[]) => VertexEditResult,
): VertexEditResult & { rings?: Position[][] } {
    const out: Position[][] = [];
    let removed = 0;
    for (const ring of rings) {
        const r = op(openRing(ring));
        if (!r.ok) return r;
        removed += r.removed;
        out.push(closeRing(r.points));
    }
    return {ok: true, points: out[0] ? openRing(out[0]) : [], removed, rings: out};
}

/** Number of distinct vertices of a polygon (all rings) or line. */
export function countVertices(geometry: { type: string; coordinates: unknown }): number {
    if (geometry.type === "Polygon") {
        return (geometry.coordinates as Position[][]).reduce((s, r) => s + openRing(r).length, 0);
    }
    if (geometry.type === "LineString") return (geometry.coordinates as Position[]).length;
    return 0;
}

export type VertexHit =
    | { kind: "vertex"; ring: number; index: number }
    | { kind: "edge"; ring: number; edgeStart: number }
    | null;

/**
 * What a right-click / long-press at `pt` (screen px) is on: the nearest
 * vertex within `radiusPx`, else the nearest edge within `radiusPx`.
 * `rings` are open vertex lists in screen px.
 */
export function hitVertexOrEdge(rings: [number, number][][], pt: [number, number], radiusPx: number, closed = true): VertexHit {
    let best: VertexHit = null, bestD = radiusPx;
    rings.forEach((ring, r) => ring.forEach((p, i) => {
        const d = dist(p, pt);
        if (d <= bestD) { best = {kind: "vertex", ring: r, index: i}; bestD = d; }
    }));
    if (best) return best;
    bestD = radiusPx;
    rings.forEach((ring, r) => {
        const edges = closed ? ring.length : ring.length - 1;
        for (let i = 0; i < edges; i++) {
            const a = ring[i], b = ring[(i + 1) % ring.length];
            const dx = b[0] - a[0], dy = b[1] - a[1];
            const len2 = dx * dx + dy * dy;
            const t = len2 === 0 ? 0 : Math.max(0, Math.min(1, ((pt[0] - a[0]) * dx + (pt[1] - a[1]) * dy) / len2));
            const d = dist([a[0] + t * dx, a[1] + t * dy], pt);
            if (d <= bestD) { best = {kind: "edge", ring: r, edgeStart: i}; bestD = d; }
        }
    });
    return best;
}
