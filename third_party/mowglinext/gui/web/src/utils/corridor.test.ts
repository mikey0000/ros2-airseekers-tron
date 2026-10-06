import union from "@turf/union";
import {featureCollection, polygon as turfPolygon} from "@turf/helpers";
import {describe, expect, it} from "vitest";
import {
    bufferLonLatPolyline, bufferPolyline, closestPointOnRing, dockApproachPoint, dockConnectorPolyline,
    lonLatToLocal, ringArea, snapPathToDock, type XY,
    DEFAULT_PATH_WIDTH_M, MAX_PATH_WIDTH_M, MIN_PATH_WIDTH_M, extendEnds, isPathConnected, snapPathEnds,
} from "./corridor.ts";

const totalArea = (polys: XY[][][]) => polys.reduce((a, p) => a + ringArea(p[0]), 0);
const bbox = (ring: XY[]) => ({
    minX: Math.min(...ring.map((p) => p[0])), maxX: Math.max(...ring.map((p) => p[0])),
    minY: Math.min(...ring.map((p) => p[1])), maxY: Math.max(...ring.map((p) => p[1])),
});

describe("bufferPolyline", () => {
    it("buffers a straight segment to exactly length x width with flat caps", () => {
        const polys = bufferPolyline([[0, 0], [10, 0]], 0.8, {cap: "flat"});
        expect(polys).toHaveLength(1);
        expect(polys[0]).toHaveLength(1);
        const b = bbox(polys[0][0]);
        expect(b.minX).toBeCloseTo(0, 6);
        expect(b.maxX).toBeCloseTo(10, 6);
        expect(b.minY).toBeCloseTo(-0.4, 6);
        expect(b.maxY).toBeCloseTo(0.4, 6);
        expect(totalArea(polys)).toBeCloseTo(8, 6);
    });

    it("keeps the width constant on a diagonal (metres, not degrees)", () => {
        const polys = bufferPolyline([[0, 0], [3, 4]], 1.0, {cap: "flat"});
        expect(totalArea(polys)).toBeCloseTo(5, 6);
    });

    it("fills a 90 degree corner with a round join", () => {
        const w = 1;
        const polys = bufferPolyline([[0, 0], [10, 0], [10, 10]], w, {arcSegments: 256, cap: "flat"});
        expect(polys).toHaveLength(1);
        // Two 10x1 rectangles overlap in a 0.5x0.5 square at the corner; the
        // round join adds the outer quarter disc (r=0.5).
        const expected = 2 * 10 * w - 0.25 + Math.PI * 0.25 / 4;
        expect(totalArea(polys)).toBeCloseTo(expected, 2);
        // Outer corner is rounded: no vertex reaches (10.5, -0.5).
        expect(polys[0][0].some(([x, y]) => x > 10.49 && y < -0.49)).toBe(false);
    });

    it("fills a 90 degree corner with a flat (bevel) join", () => {
        const polys = bufferPolyline([[0, 0], [10, 0], [10, 10]], 1, {join: "flat", cap: "flat"});
        expect(polys).toHaveLength(1);
        // Bevel triangle = half of the 0.5x0.5 outer square.
        expect(totalArea(polys)).toBeCloseTo(20 - 0.25 + 0.125, 6);
    });

    it("splits a closed loop into two hole-free halves covering the ring", () => {
        const loop: XY[] = [[0, 0], [10, 0], [10, 10], [0, 10], [0, 0]];
        const polys = bufferPolyline(loop, 1);
        expect(polys).toHaveLength(2);
        for (const p of polys) expect(p).toHaveLength(1); // no holes
        // Together the halves cover exactly the ring corridor: outer 11x11
        // with rounded corners minus the inner 9x9.
        const merged = union(featureCollection(polys.map((p) => turfPolygon(p))))!;
        expect(merged.geometry.type).toBe("Polygon");
        const rings = merged.geometry.coordinates as XY[][];
        expect(rings).toHaveLength(2); // outer + the loop's interior hole
        const ringCorridor = 11 * 11 - 9 * 9 - (4 - Math.PI) * 0.25;
        expect(ringArea(rings[0]) - ringArea(rings[1])).toBeCloseTo(ringCorridor, 1);
    });

    it("rejects degenerate input", () => {
        expect(bufferPolyline([[1, 1]], 1)).toEqual([]);
        expect(bufferPolyline([[1, 1], [1, 1]], 1)).toEqual([]);
        expect(bufferPolyline([[0, 0], [1, 0]], 0)).toEqual([]);
    });
});

describe("dock snapping", () => {
    const dock = {x: 5, y: 5, heading: Math.PI / 2}; // facing north

    it("puts the approach point 0.8 m along the dock yaw", () => {
        const [x, y] = dockApproachPoint(dock);
        expect(x).toBeCloseTo(5, 9);
        expect(y).toBeCloseTo(5.8, 9);
    });

    it("appends approach then dock to the end nearer the dock", () => {
        const out = snapPathToDock([[0, 20], [5, 10]], dock);
        expect(out).toHaveLength(4);
        expect(out[2][0]).toBeCloseTo(5);
        expect(out[2][1]).toBeCloseTo(5.8);
        expect(out[3]).toEqual([5, 5]);
    });

    it("prepends when the path was drawn starting at the dock", () => {
        const out = snapPathToDock([[5, 6.5], [0, 20]], dock);
        expect(out[0]).toEqual([5, 5]);
        expect(out[1][1]).toBeCloseTo(5.8);
        expect(out[out.length - 1]).toEqual([0, 20]);
    });

    it("does not duplicate a vertex already on the approach point", () => {
        const out = snapPathToDock([[0, 20], [5, 5.8]], dock);
        expect(out).toHaveLength(3);
    });
});

describe("dockConnectorPolyline", () => {
    const square: XY[] = [[10, -5], [20, -5], [20, 5], [10, 5], [10, -5]];
    const far: XY[] = [[100, 100], [110, 100], [110, 110], [100, 110], [100, 100]];

    it("connects dock -> approach -> nearest area, overlapping into it", () => {
        const res = dockConnectorPolyline({x: 0, y: 0, heading: 0}, [far, square], 0.3);
        expect(res).not.toBeNull();
        expect(res!.areaIndex).toBe(1);
        expect(res!.gap).toBeCloseTo(10 - 0.8, 6);
        expect(res!.points[0]).toEqual([0, 0]);
        expect(res!.points[1][0]).toBeCloseTo(0.8);
        expect(res!.points[2][0]).toBeCloseTo(10.3);
        expect(res!.points[2][1]).toBeCloseTo(0);
    });

    it("returns null when the approach point is already inside an area", () => {
        expect(dockConnectorPolyline({x: 9.5, y: 0, heading: 0}, [square])).toBeNull();
        expect(dockConnectorPolyline({x: 0, y: 0, heading: 0}, [])).toBeNull();
    });

    it("finds the closest boundary point", () => {
        const c = closestPointOnRing([15, -9], square)!;
        expect(c.point[0]).toBeCloseTo(15);
        expect(c.point[1]).toBeCloseTo(-5);
        expect(c.distance).toBeCloseTo(4);
    });
});

describe("bufferLonLatPolyline", () => {
    it("produces a corridor whose width is metres at the datum latitude", () => {
        const datum: [number, number, number] = [60, 10, 0]; // high latitude: lon degrees are short
        const a = [10, 60];
        const b = [10 + 20 / (Math.cos(60 * Math.PI / 180) * 6378137 * Math.PI / 180), 60]; // 20 m east
        const [poly] = bufferLonLatPolyline(datum, [a, b], 1, {cap: "flat"});
        const local = poly.coordinates[0].map((p) => lonLatToLocal(datum, p));
        const bb = bbox(local);
        expect(bb.maxX - bb.minX).toBeCloseTo(20, 4);
        expect(bb.maxY - bb.minY).toBeCloseTo(1, 4);
    });
});

describe("square caps (default)", () => {
    it("extends each end by half the width, like the vendor channel import", () => {
        const polys = bufferPolyline([[0, 0], [10, 0]], 0.7);
        const b = bbox(polys[0][0]);
        expect(b.minX).toBeCloseTo(-0.35, 6);
        expect(b.maxX).toBeCloseTo(10.35, 6);
        expect(b.maxY - b.minY).toBeCloseTo(0.7, 6);
        expect(totalArea(polys)).toBeCloseTo(10.7 * 0.7, 6);
    });

    it("extendEnds leaves interior vertices alone", () => {
        expect(extendEnds([[0, 0], [5, 0], [5, 5]], 1)).toEqual([[-1, 0], [5, 0], [5, 6]]);
        expect(extendEnds([[0, 0]], 1)).toEqual([[0, 0]]);
    });

    it("width defaults and limits match the spec (0.7 m, 0.5..3.0)", () => {
        expect(DEFAULT_PATH_WIDTH_M).toBe(0.7);
        expect(MIN_PATH_WIDTH_M).toBe(0.5);
        expect(MAX_PATH_WIDTH_M).toBe(3.0);
    });
});

describe("snapPathEnds", () => {
    const area: XY[] = [[0, 0], [10, 0], [10, 10], [0, 10]];
    const dock = {x: 20, y: 5, heading: Math.PI}; // approach point at (19.2, 5)

    it("snaps an end within 0.5 m onto the area outline and the other onto the dock approach pose", () => {
        const r = snapPathEnds([[10.4, 5], [15, 5], [19.5, 5.2]], [area], dock);
        expect(r.start).toBe("area");
        expect(r.points[0][0]).toBeCloseTo(10, 6);
        expect(r.points[0][1]).toBeCloseTo(5, 6);
        expect(r.end).toBe("dock");
        expect(r.points[r.points.length - 1][0]).toBeCloseTo(19.2, 6);
        expect(r.points[r.points.length - 1][1]).toBeCloseTo(5, 6);
        expect(isPathConnected(r)).toBe(true);
    });

    it("keeps an end inside an area and reports far ends as not connected", () => {
        const r = snapPathEnds([[5, 5], [14, 5]], [area], dock);
        expect(r.start).toBe("inside");
        expect(r.points[0]).toEqual([5, 5]);
        expect(r.end).toBe("none");
        expect(r.points[1]).toEqual([14, 5]);
        expect(isPathConnected(r)).toBe(false);
    });

    it("snaps to the dock when the click is on the dock itself", () => {
        const r = snapPathEnds([[20.1, 5], [12, 5]], [area], dock);
        expect(r.start).toBe("dock");
        expect(r.points[0][0]).toBeCloseTo(19.2, 6);
    });

    it("works without a dock pose and with degenerate input", () => {
        expect(snapPathEnds([[10.3, 2], [12, 2]], [area], null).start).toBe("area");
        expect(snapPathEnds([[1, 1]], [area], null)).toEqual({points: [[1, 1]], start: "none", end: "none"});
    });

    it("a snapped band overlaps the area by half a width (square cap)", () => {
        const r = snapPathEnds([[10.4, 5], [15, 5]], [area], null);
        const b = bbox(bufferPolyline(r.points, 0.7)[0][0]);
        expect(b.minX).toBeCloseTo(10 - 0.35, 6);
    });
});
