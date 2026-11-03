import {describe, expect, it} from "vitest";
import type {LineString, Polygon, Position} from "geojson";
import {
    TOLERANCE_MAX_M,
    TOLERANCE_MIN_M,
    initialVertexToolState,
    previewVertexTool,
    vertexToolReducer,
} from "./vertexToolState.ts";
import {closeRing} from "../utils/vertexEdit.ts";

const m = (x: number, y: number): Position => [2 + x / (111_320 * Math.cos((48 * Math.PI) / 180)), 48 + y / 110_540];

describe("vertexToolReducer", () => {
    it("opens with per-tool defaults", () => {
        const s = vertexToolReducer(initialVertexToolState(), {type: "open", tool: "merge", featureId: "a"});
        expect(s.tool).toBe("merge");
        expect(s.featureId).toBe("a");
        expect(s.tolerance).toEqual({merge: 0.15, simplify: 0.05});
    });
    it("edits only the open tool's tolerance, clamped, and remembers it", () => {
        let s = vertexToolReducer(initialVertexToolState(), {type: "open", tool: "simplify", featureId: "a"});
        s = vertexToolReducer(s, {type: "tolerance", value: 0.2});
        expect(s.tolerance).toEqual({merge: 0.15, simplify: 0.2});
        s = vertexToolReducer(s, {type: "tolerance", value: 99});
        expect(s.tolerance.simplify).toBe(TOLERANCE_MAX_M);
        s = vertexToolReducer(s, {type: "tolerance", value: 0});
        expect(s.tolerance.simplify).toBe(TOLERANCE_MIN_M);
        s = vertexToolReducer(s, {type: "tolerance", value: null});
        expect(s.tolerance.simplify).toBe(TOLERANCE_MIN_M);
        s = vertexToolReducer(s, {type: "close"});
        expect(s.tool).toBeNull();
        expect(s.tolerance.simplify).toBe(TOLERANCE_MIN_M);
    });
    it("ignores tolerance while closed", () => {
        const s0 = initialVertexToolState();
        expect(vertexToolReducer(s0, {type: "tolerance", value: 1})).toBe(s0);
    });
});

describe("previewVertexTool", () => {
    const poly: Polygon = {type: "Polygon", coordinates: [closeRing([m(0, 0), m(0.1, 0), m(5, 0.02), m(10, 0), m(10, 10), m(0, 10)])]};
    it("returns null while closed", () => {
        expect(previewVertexTool(initialVertexToolState(), poly)).toBeNull();
    });
    it("previews a polygon merge", () => {
        const s = vertexToolReducer(initialVertexToolState(), {type: "open", tool: "merge", featureId: "a"});
        const p = previewVertexTool(s, poly);
        expect(p).toMatchObject({ok: true, before: 6, after: 5});
        if (p?.ok) expect(p.geometry.coordinates[0]).toHaveLength(6);
    });
    it("previews a polygon simplify", () => {
        const s = vertexToolReducer(initialVertexToolState(), {type: "open", tool: "simplify", featureId: "a"});
        expect(previewVertexTool(s, poly)).toMatchObject({ok: true, before: 6, after: 4});
    });
    it("previews a line and reports a rejection", () => {
        const line: LineString = {type: "LineString", coordinates: [m(0, 0), m(4, 0), m(4, 1), m(2, 0.2), m(0.1, 1), m(0.1, 3), m(-1, 3)]};
        let s = vertexToolReducer(initialVertexToolState(), {type: "open", tool: "simplify", featureId: "l"});
        s = vertexToolReducer(s, {type: "tolerance", value: 1.2});
        expect(previewVertexTool(s, line)).toEqual({ok: false, before: 7, reason: "selfIntersection"});
    });
});
