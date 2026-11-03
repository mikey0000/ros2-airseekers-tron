import {describe, expect, it} from "vitest";
import type {Position} from "geojson";
import {
    closeRing,
    deleteVertex,
    findSnapNeighbour,
    hitVertexOrEdge,
    insertMidpoint,
    mapPolygonRings,
    mergeNearbyVertices,
    mergeVertexInto,
    openRing,
    selfIntersects,
    simplifyVertices,
    toLocalMetres,
} from "./vertexEdit.ts";

const LAT0 = 48, LON0 = 2;
/** [x, y] metres east / north of a fixed origin to [lon, lat]. */
const m = (x: number, y: number): Position => [
    LON0 + x / (111_320 * Math.cos((LAT0 * Math.PI) / 180)),
    LAT0 + y / 110_540,
];
const square = [m(0, 0), m(10, 0), m(10, 10), m(0, 10)];

describe("ring helpers", () => {
    it("opens and closes rings", () => {
        const closed = closeRing(square);
        expect(closed).toHaveLength(5);
        expect(openRing(closed)).toEqual(square);
        expect(openRing(square)).toEqual(square);
    });
    it("projects to local metres", () => {
        const p = toLocalMetres(square);
        expect(p[2][0]).toBeCloseTo(10, 6);
        expect(p[2][1]).toBeCloseTo(10, 6);
    });
});

describe("selfIntersects", () => {
    it("accepts a simple square and a line", () => {
        expect(selfIntersects(square, true)).toBe(false);
        expect(selfIntersects([m(0, 0), m(5, 0), m(5, 5)], false)).toBe(false);
    });
    it("detects a bow tie", () => {
        expect(selfIntersects([m(0, 0), m(10, 10), m(10, 0), m(0, 10)], true)).toBe(true);
    });
    it("detects a crossing line and a folded-back edge", () => {
        expect(selfIntersects([m(0, 0), m(10, 0), m(10, 5), m(5, -5)], false)).toBe(true);
        expect(selfIntersects([m(0, 0), m(10, 0), m(5, 0)], false)).toBe(true);
    });
});

describe("merge", () => {
    it("drag-merge removes the dragged vertex", () => {
        const pts = [m(0, 0), m(10, 0), m(10, 0.05), m(10, 10), m(0, 10)];
        const r = mergeVertexInto(pts, 2, true);
        expect(r.ok && r.points).toEqual([pts[0], pts[1], pts[3], pts[4]]);
    });
    it("refuses to drop a triangle below 3 vertices", () => {
        const r = mergeVertexInto([m(0, 0), m(1, 0), m(0, 1)], 1, true);
        expect(r).toEqual({ok: false, reason: "tooFew"});
    });
    it("merges consecutive close vertices of a ring, including last-first", () => {
        const pts = [m(0, 0), m(0.1, 0), m(10, 0), m(10, 10), m(0, 10), m(0, 0.05)];
        const r = mergeNearbyVertices(pts, 0.15, true);
        expect(r.ok).toBe(true);
        if (!r.ok) return;
        expect(r.removed).toBe(2);
        expect(r.points).toEqual([pts[0], pts[2], pts[3], pts[4]]);
    });
    it("keeps the end points of a line", () => {
        const pts = [m(0, 0), m(5, 0), m(9.95, 0), m(10, 0)];
        const r = mergeNearbyVertices(pts, 0.15, false);
        expect(r.ok && r.points).toEqual([pts[0], pts[1], pts[3]]);
    });
    it("leaves well-spaced vertices alone", () => {
        const r = mergeNearbyVertices(square, 0.15, true);
        expect(r.ok && r.removed).toBe(0);
    });
    it("never goes below the minimum", () => {
        const r = mergeNearbyVertices([m(0, 0), m(0.01, 0), m(0, 0.01), m(0.01, 0.01)], 1, true);
        expect(r.ok && r.points.length).toBe(3);
    });
});

describe("delete / insert", () => {
    it("deletes a vertex", () => {
        const r = deleteVertex([...square, m(5, 12)], 4, true);
        expect(r.ok && r.points).toEqual(square);
    });
    it("deletes the notch tip of a concave outline", () => {
        const pts = [m(0, 0), m(10, 0), m(10, 10), m(5, 1), m(0, 10)];
        expect(deleteVertex(pts, 3, true).ok).toBe(true);
    });
    it("inserts an edge midpoint, wrapping on a ring", () => {
        const pts = insertMidpoint(square, 3, true);
        expect(pts).toHaveLength(5);
        expect(pts[4][0]).toBeCloseTo(m(0, 5)[0], 9);
        expect(pts[4][1]).toBeCloseTo(m(0, 5)[1], 9);
        expect(insertMidpoint([m(0, 0), m(2, 0)], 1, false)).toHaveLength(2);
    });
});

describe("simplify", () => {
    it("drops near-collinear vertices of a ring", () => {
        const pts = [m(0, 0), m(5, 0.02), m(10, 0), m(10, 5), m(10.03, 7), m(10, 10), m(0, 10)];
        const r = simplifyVertices(pts, 0.05, true);
        expect(r.ok).toBe(true);
        if (!r.ok) return;
        expect(r.points).toEqual([pts[0], pts[2], pts[5], pts[6]]);
        expect(r.removed).toBe(3);
    });
    it("keeps corners above the tolerance", () => {
        const pts = [m(0, 0), m(5, 0.2), m(10, 0), m(10, 10), m(0, 10)];
        const r = simplifyVertices(pts, 0.05, true);
        expect(r.ok && r.points.length).toBe(5);
    });
    it("rejects too few vertices", () => {
        const pts = [m(0, 0), m(5, 0.01), m(10, 0), m(5, 0.02)];
        expect(simplifyVertices(pts, 1, true)).toEqual({ok: false, reason: "tooFew"});
    });
    it("keeps both ends of a line", () => {
        const pts = [m(0, 0), m(3, 0.01), m(6, -0.01), m(10, 0)];
        const r = simplifyVertices(pts, 0.05, false);
        expect(r.ok && r.points).toEqual([pts[0], pts[3]]);
    });
    it("rejects a result that crosses itself", () => {
        // A thin U whose arms are 0.3 m apart: a coarse tolerance cuts across the gap.
        const pts = [m(0, 0), m(10, 0), m(10, 0.1), m(0.3, 0.1), m(0.3, 5), m(0, 5)];
        const zig = [m(0, 0), m(4, 0), m(4, 1), m(2, 0.2), m(0.1, 1), m(0.1, 3), m(-1, 3)];
        expect(simplifyVertices(pts, 0.05, true).ok).toBe(true);
        const r = simplifyVertices(zig, 1.2, false);
        expect(r.ok === false ? r.reason : r.points.length).toBe("selfIntersection");
    });
});

describe("findSnapNeighbour", () => {
    const px: [number, number][] = [[0, 0], [100, 0], [108, 2], [100, 100]];
    it("finds a neighbour within the snap radius", () => {
        expect(findSnapNeighbour(px, 2, true, 12)).toBe(1);
        expect(findSnapNeighbour(px, 2, true, 5)).toBe(-1);
    });
    it("wraps on a ring only", () => {
        const p: [number, number][] = [[0, 0], [50, 0], [50, 50], [3, 3]];
        expect(findSnapNeighbour(p, 3, true, 12)).toBe(0);
        expect(findSnapNeighbour(p, 0, false, 12)).toBe(-1);
    });
});

describe("mapPolygonRings", () => {
    it("applies to holes and re-closes rings", () => {
        const hole = [m(2, 2), m(2.05, 2), m(4, 2), m(4, 4), m(2, 4)];
        const r = mapPolygonRings([closeRing(square), closeRing(hole)], (p) => mergeNearbyVertices(p, 0.15, true));
        expect(r.ok && r.removed).toBe(1);
        expect(r.ok && r.rings?.[1]).toHaveLength(5);
    });
});

describe("hitVertexOrEdge", () => {
    const ring: [number, number][] = [[0, 0], [100, 0], [100, 100], [0, 100]];
    it("prefers a vertex, then an edge, else null", () => {
        expect(hitVertexOrEdge([ring], [98, 3], 12)).toEqual({kind: "vertex", ring: 0, index: 1});
        expect(hitVertexOrEdge([ring], [50, 5], 12)).toEqual({kind: "edge", ring: 0, edgeStart: 0});
        expect(hitVertexOrEdge([ring], [5, 50], 12)).toEqual({kind: "edge", ring: 0, edgeStart: 3});
        expect(hitVertexOrEdge([ring], [50, 50], 12)).toBeNull();
    });
});
