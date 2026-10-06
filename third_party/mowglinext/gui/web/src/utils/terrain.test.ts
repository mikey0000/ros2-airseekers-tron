import {describe, expect, it} from "vitest";
import {formatAge, parseTerrainSummary, terrainPaint} from "./terrain.ts";

describe("terrainPaint", () => {
    it("leaves no-data and low scores transparent", () => {
        expect(terrainPaint(-1)).toBeNull();
        expect(terrainPaint(0)).toBeNull();
        expect(terrainPaint(9)).toBeNull();
    });
    it("ramps yellow -> red with rising alpha", () => {
        const lo = terrainPaint(10)!;
        const hi = terrainPaint(100)!;
        expect(lo[1]).toBeGreaterThan(hi[1]); // green drops
        expect(hi[0]).toBeGreaterThan(200);
        expect(hi[3]).toBeGreaterThan(lo[3]);
        expect(terrainPaint(150)).toEqual(hi);
    });
});

describe("parseTerrainSummary", () => {
    it("parses valid JSON and rejects junk", () => {
        expect(parseTerrainSummary('{"areas":[]}')).toEqual({areas: []});
        expect(parseTerrainSummary("nope")).toBeNull();
        expect(parseTerrainSummary('{"x":1}')).toBeNull();
        expect(parseTerrainSummary(undefined)).toBeNull();
    });
});

describe("formatAge", () => {
    it("formats seconds/minutes/hours/days", () => {
        expect(formatAge(100, 130)).toBe("30 s");
        expect(formatAge(0, 600)).toBe("10 min");
        expect(formatAge(0, 7200)).toBe("2 h");
        expect(formatAge(0, 172800)).toBe("2 d");
        expect(formatAge(undefined)).toBe("—");
    });
});
