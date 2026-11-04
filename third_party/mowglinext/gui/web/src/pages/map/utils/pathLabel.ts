// Where a navigation area / path label goes, and how long a path is.
//
// F14: labels sat on the polygon centroid, which for a bent corridor lies
// outside it, often on the neighbouring corridor. A path is labelled at the
// middle of its centreline; a plain navigation polygon at its centroid when
// that is inside, else at its vertex nearest the centroid.
import type {Polygon, Position} from "geojson";
import {itranspose} from "../../../utils/map.tsx";

type Datum = [number, number, number];

function pointInRing(p: Position, ring: Position[]): boolean {
    let inside = false;
    for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
        const [xi, yi] = ring[i];
        const [xj, yj] = ring[j];
        if ((yi > p[1]) !== (yj > p[1]) && p[0] < ((xj - xi) * (p[1] - yi)) / (yj - yi + 1e-18) + xi) {
            inside = !inside;
        }
    }
    return inside;
}

/** The point halfway along a polyline (planar, fine at path scale). */
export function polylineMidpoint(points: Position[]): Position | null {
    if (points.length === 0) return null;
    if (points.length === 1) return points[0];
    const seg: number[] = [];
    let total = 0;
    for (let i = 1; i < points.length; i++) {
        const d = Math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]);
        seg.push(d);
        total += d;
    }
    if (total === 0) return points[0];
    let left = total / 2;
    for (let i = 0; i < seg.length; i++) {
        if (left <= seg[i] || i === seg.length - 1) {
            const f = seg[i] > 0 ? Math.min(1, left / seg[i]) : 0;
            const a = points[i], b = points[i + 1];
            return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
        }
        left -= seg[i];
    }
    return points[points.length - 1];
}

/** Label point of a navigation area: centreline middle, else an inside point. */
export function navigationLabelPoint(geometry: Polygon, centreline: Position[] | null): Position | null {
    if (centreline && centreline.length >= 2) return polylineMidpoint(centreline);
    const ring = geometry.coordinates?.[0] ?? [];
    if (ring.length === 0) return null;
    const open = ring.length > 1 && ring[0][0] === ring[ring.length - 1][0] && ring[0][1] === ring[ring.length - 1][1]
        ? ring.slice(0, -1) : ring;
    const c: Position = [
        open.reduce((s, p) => s + p[0], 0) / open.length,
        open.reduce((s, p) => s + p[1], 0) / open.length,
    ];
    if (pointInRing(c, ring)) return c;
    let best = open[0];
    let bestD = Infinity;
    for (const p of open) {
        const d = Math.hypot(p[0] - c[0], p[1] - c[1]);
        if (d < bestD) { best = p; bestD = d; }
    }
    return best;
}

/** Length (m) of a lon/lat polyline in the robot's map frame. */
export function pathLengthM(points: Position[], offsetX: number, offsetY: number, datum: Datum): number {
    const pts = points.map((c) => itranspose(offsetX, offsetY, datum, c[1], c[0]));
    let total = 0;
    for (let i = 1; i < pts.length; i++) total += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
    return total;
}
