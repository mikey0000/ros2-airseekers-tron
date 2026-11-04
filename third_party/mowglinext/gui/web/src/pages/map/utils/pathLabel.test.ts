import {describe, expect, it} from "vitest";
import {navigationLabelPoint, polylineMidpoint} from "./pathLabel.ts";

describe("path labels (F14)", () => {
    it("labels a path at the middle of its centreline, not its centroid", () => {
        // An L-shaped path: the centroid of its band would sit off the path.
        const line = [[0, 0], [10, 0], [10, 10]];
        expect(polylineMidpoint(line)).toEqual([10, 0]);
        const band = {type: "Polygon" as const, coordinates: [[[0, -1], [11, -1], [11, 10], [9, 10], [9, 1], [0, 1], [0, -1]]]};
        expect(navigationLabelPoint(band, line)).toEqual([10, 0]);
    });

    it("keeps the centroid of a convex navigation polygon", () => {
        const square = {type: "Polygon" as const, coordinates: [[[0, 0], [2, 0], [2, 2], [0, 2], [0, 0]]]};
        expect(navigationLabelPoint(square, null)).toEqual([1, 1]);
    });

    it("moves the label onto a bent polygon when its centroid is outside", () => {
        // U shape: vertex average (1.5, 1.5) is in the notch.
        const u = {type: "Polygon" as const, coordinates: [[[0, 0], [3, 0], [3, 3], [2, 3], [2, 1], [1, 1], [1, 3], [0, 3], [0, 0]]]};
        const p = navigationLabelPoint(u, null)!;
        expect([[2, 1], [1, 1]]).toContainEqual(p);
    });

    it("handles degenerate input", () => {
        expect(polylineMidpoint([])).toBeNull();
        expect(polylineMidpoint([[1, 2]])).toEqual([1, 2]);
        expect(navigationLabelPoint({type: "Polygon", coordinates: []}, null)).toBeNull();
    });
});
