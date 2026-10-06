import {describe, expect, it} from "vitest";
import {markerCornersLocal, resolveAnchor, selectDockMarker, selectRobotMarker} from "./mapMarker.ts";
import {MOWER_MODELS} from "../constants/mowerModels.ts";
import {findRobotProfile} from "../constants/robotProfiles.ts";

const close = (a: number[], b: number[]) => a.forEach((v, i) => expect(v).toBeCloseTo(b[i], 9));

describe("map marker selection", () => {
    it("stock presets keep the silhouette and dot", () => {
        for (const m of MOWER_MODELS.filter((m) => m.value !== "AirseekersTron")) {
            expect(selectRobotMarker(m)).toBeUndefined();
            expect(selectDockMarker(m)).toBeUndefined();
        }
        expect(selectRobotMarker(undefined)).toBeUndefined();
    });
    it("Airseekers Tron profile carries the vendor images", () => {
        const p = findRobotProfile("AirseekersTron")!;
        expect(selectRobotMarker(p)?.src).toBe("/robots/airseekers_tron_top.png");
        expect(selectRobotMarker(p)?.anchor).toEqual({x: 0.5, y: 0.86});
        // the vendor dock asset is a glyph, not a top-down view: stock dock marker is kept
        expect(selectDockMarker(p)).toBeUndefined();
    });
    it("rejects malformed markers", () => {
        expect(selectRobotMarker({mapMarker: {src: "", widthM: 1, lengthM: 1}})).toBeUndefined();
        expect(selectRobotMarker({mapMarker: {src: "a.png", widthM: 0, lengthM: 1}})).toBeUndefined();
    });
});

describe("anchor math", () => {
    it("derives base_link from chassis_center_x when no anchor is given", () => {
        expect(resolveAnchor({src: "a", widthM: 0.5, lengthM: 0.8}, {chassis_center_x: 0.2})).toEqual({x: 0.5, y: 0.75});
        expect(resolveAnchor({src: "a", widthM: 0.5, lengthM: 0.8})).toEqual({x: 0.5, y: 0.5});
        expect(resolveAnchor({src: "a", widthM: 0.5, lengthM: 0.8, anchor: {x: 0.4, y: 0.9}}, {chassis_center_x: 0.2}))
            .toEqual({x: 0.4, y: 0.9});
    });
    const m = {src: "a", widthM: 0.4, lengthM: 1.0, anchor: {x: 0.5, y: 0.75}};
    it("heading 0 (east): image top points east, anchor at pose", () => {
        const [tl, tr, br, bl] = markerCornersLocal(m, 10, 20, 0);
        close(tl, [10.75, 20.2]); // front-left
        close(tr, [10.75, 19.8]); // front-right
        close(br, [9.75, 19.8]);
        close(bl, [9.75, 20.2]);
    });
    it("rotates with heading (90 deg = north)", () => {
        const [tl, , br] = markerCornersLocal(m, 0, 0, Math.PI / 2);
        close(tl, [-0.2, 0.75]);
        close(br, [0.2, -0.25]);
    });
    it("applies headingOffsetDeg", () => {
        const a = markerCornersLocal({...m, headingOffsetDeg: 90}, 0, 0, 0);
        const b = markerCornersLocal(m, 0, 0, Math.PI / 2);
        a.forEach((p, i) => close(p, b[i]));
    });
});
