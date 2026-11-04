import {describe, expect, it} from "vitest";
import {dockMoveNeedsRecalibration, selectDockMoved} from "./dockMoved.ts";

const msg = (o: unknown) => ({data: JSON.stringify(o)});
const moved = {moved: true, distance_m: 0.8, docked: [1.5, 2], stored: [0.7, 2], warn_m: 0.5};

describe("selectDockMoved", () => {
    it("parses the docking node's JSON", () => {
        expect(selectDockMoved(msg(moved))).toEqual(moved);
    });

    it("rejects malformed payloads", () => {
        expect(selectDockMoved({data: "not json"})).toBeUndefined();
        expect(selectDockMoved(msg({moved: true}))).toBeUndefined();
        expect(selectDockMoved(msg({...moved, docked: [1]}))).toBeUndefined();
        expect(selectDockMoved(null)).toBeUndefined();
    });

    it("defaults warn_m to the docking default", () => {
        const noWarn = {moved: true, distance_m: 0.8, docked: [1.5, 2], stored: [0.7, 2]};
        expect(selectDockMoved(msg(noWarn))?.warn_m).toBe(0.5);
    });
});

describe("dockMoveNeedsRecalibration", () => {
    const result = selectDockMoved(msg(moved));

    it("offers it while the stored pose is still the old one", () => {
        expect(dockMoveNeedsRecalibration(result, {x: 0.7, y: 2})).toBe(true);
        expect(dockMoveNeedsRecalibration(result, null)).toBe(true);
    });

    it("stops offering once the dock was re-set near the docked pose", () => {
        expect(dockMoveNeedsRecalibration(result, {x: 1.45, y: 2.05})).toBe(false);
    });

    it("never offers it when the charger did not move", () => {
        expect(dockMoveNeedsRecalibration(selectDockMoved(msg({...moved, moved: false})), {x: 0, y: 0})).toBe(false);
        expect(dockMoveNeedsRecalibration(null, null)).toBe(false);
    });
});
