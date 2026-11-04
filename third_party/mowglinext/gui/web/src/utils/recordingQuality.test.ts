import {describe, expect, it} from "vitest";
import {
    mapItemNames, parseRecordingQuality, recordingQualityFeatures, VERTEX_QUALITY_COLORS, vertexQualityLevel,
} from "./recordingQuality.ts";
import type {Map as MapType} from "../types/ros.ts";

const body = {recordings: [
    {name: "Front lawn", kind: "area", saved: 200, require_fixed: true, not_fixed: 2, vertices: [
        {x: 1, y: 2, fix_type: 3, hdop: 0.6, fixed: true},
        {x: 3, y: 4, fix_type: 2, hdop: 1.4, fixed: false},
        {x: 5, y: 6, fix_type: null, hdop: null, fixed: false},
        {x: "bad", y: 0},
    ]},
    {name: "Front lawn", kind: "area", saved: 100, vertices: [{x: 9, y: 9, fix_type: 1, hdop: 3, fixed: false}]},
    {name: "Gone", kind: "obstacle", saved: 300, vertices: [{x: 0, y: 0, fix_type: 0, hdop: null, fixed: false}]},
    {kind: "no name"},
]};

const tp = (x: number, y: number) => [x * 10, y * 10];

describe("recording quality", () => {
    it("parses the backend body and drops malformed entries", () => {
        const r = parseRecordingQuality(body);
        expect(r.map((x) => x.name)).toEqual(["Front lawn", "Front lawn", "Gone"]);
        expect(r[0].vertices).toHaveLength(3);
        expect(r[1].not_fixed).toBe(1);
        expect(parseRecordingQuality(null)).toEqual([]);
        expect(parseRecordingQuality({recordings: "x"})).toEqual([]);
    });

    it("grades fixed, float and anything worse", () => {
        expect(vertexQualityLevel({fixed: true, fix_type: 3})).toBe("fixed");
        expect(vertexQualityLevel({fixed: false, fix_type: 2})).toBe("float");
        expect(vertexQualityLevel({fixed: false, fix_type: 1})).toBe("none");
        expect(vertexQualityLevel({fixed: false, fix_type: null})).toBe("none");
    });

    it("collects area, no-go zone and path names from the map", () => {
        const map = {
            working_area: [{name: "Front lawn", obstacle_info: [{name: "Tree", source: 0, pending: false, id: 0}]}],
            navigation_areas: [{name: "Side path"}, {name: ""}],
        } as unknown as MapType;
        expect([...mapItemNames(map)].sort()).toEqual(["Front lawn", "Side path", "Tree"]);
        expect(mapItemNames(undefined).size).toBe(0);
    });

    it("draws the newest recording of items still in the map", () => {
        const recs = parseRecordingQuality(body);
        const all = recordingQualityFeatures(recs, new Set(["Front lawn"]), tp, true);
        expect(all.features.map((f) => f.properties?.level)).toEqual(["fixed", "float", "none"]);
        expect(all.features[1].properties?.color).toBe(VERTEX_QUALITY_COLORS.float);
        expect(all.features[1].geometry).toEqual({type: "Point", coordinates: [30, 40]});
    });

    it("view mode shows only the vertices without RTK fixed", () => {
        const recs = parseRecordingQuality(body);
        const view = recordingQualityFeatures(recs, new Set(["Front lawn"]), tp, false);
        expect(view.features.map((f) => f.properties?.level)).toEqual(["float", "none"]);
    });
});
