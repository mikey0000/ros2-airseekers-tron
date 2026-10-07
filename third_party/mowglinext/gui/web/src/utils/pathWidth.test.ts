import {describe, expect, it} from "vitest";
import {deriveDefaultSwathWidth as d} from "./pathWidth.ts";

describe("deriveDefaultSwathWidth", () => {
    it("derives disc - overlap", () => {
        expect(d(220, 0.04)).toBeCloseTo(0.18);
        expect(d(330, 0.04)).toBeCloseTo(0.29);
        expect(d(220, 0)).toBeCloseTo(0.22);
    });
    it("falls back on missing / unknown values", () => {
        expect(d(undefined, undefined)).toBeCloseTo(0.18);
        expect(d(250, 0.04)).toBeCloseTo(0.18);
        expect(d(330, "x")).toBeCloseTo(0.29);
    });
    it("clamps the overlap and the result", () => {
        expect(d(330, 0.5)).toBeCloseTo(0.18);
        expect(d(220, -1)).toBeCloseTo(0.22);
        expect(d(220, 0.15)).toBeCloseTo(0.10);
    });
    it("an explicit default width wins, 0 = derived", () => {
        expect(d(330, 0.04, 0.25)).toBeCloseTo(0.25);
        expect(d(330, 0.04, 0)).toBeCloseTo(0.29);
        expect(d(220, 0.04, 9)).toBeCloseTo(0.40);
    });
});
