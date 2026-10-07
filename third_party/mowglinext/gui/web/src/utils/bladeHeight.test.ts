import {describe, expect, it} from "vitest";
import {parseCutterHeightMsg, snapBladeHeight, stepBladeHeight} from "./bladeHeight.ts";

describe("bladeHeight", () => {
    it("snaps to 5 mm and clamps to 30-90", () => {
        expect(snapBladeHeight(52)).toBe(50);
        expect(snapBladeHeight(53)).toBe(55);
        expect(snapBladeHeight(10)).toBe(30);
        expect(snapBladeHeight(120)).toBe(90);
    });
    it("steps within range", () => {
        expect(stepBladeHeight(50, 1)).toBe(55);
        expect(stepBladeHeight(50, -1)).toBe(45);
        expect(stepBladeHeight(90, 1)).toBe(90);
        expect(stepBladeHeight(30, -1)).toBe(30);
        expect(stepBladeHeight(62, 1)).toBe(65);
    });
    it("parses std_msgs/Int16", () => {
        expect(parseCutterHeightMsg({data: 60})).toBe(60);
        expect(parseCutterHeightMsg({data: 0})).toBeUndefined();
        expect(parseCutterHeightMsg(null)).toBeUndefined();
        expect(parseCutterHeightMsg({data: "x"})).toBeUndefined();
    });
});
