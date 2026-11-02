import {describe, expect, it} from "vitest";
import {
    attentionTitleKey, dockFailReason, incompletePhrases, isHeadingHint, parseIncomplete, plainSubState, skippedLengthM,
    stuckReason,
} from "./missionPresentation.ts";

describe("missionPresentation", () => {
    it("parses the mission's not-mowed text", () => {
        expect(parseIncomplete("2 of 7 sub-paths not mowed, 1 area(s) not planned, 3 stretches blocked by obstacles: follow aborted: path blocked"))
            .toEqual({skippedStrips: 2, plannedStrips: 7, unplannedAreas: 1, blockedStretches: 3, reason: "path blocked"});
        expect(parseIncomplete("1 stretch blocked by obstacles")).toMatchObject({skippedStrips: null, blockedStretches: 1, reason: null});
        expect(incompletePhrases("2 of 7 sub-paths not mowed: x", 0.2).map((p) => p.key))
            .toEqual(["missionText.incompleteStrips", "missionText.incompleteReason"]);
    });

    it("names the stuck cause and the dock failure in plain words", () => {
        expect(stuckReason("stuck: needs help at (1.00, 2.00)").key).toBe("missionText.stuckWheels");
        expect(stuckReason("steep slope: needs help at (1.00, 2.00)").key).toBe("missionText.stuckSlope");
        expect(stuckReason("boundary: needs help at (1.00, 2.00)").key).toBe("missionText.stuckBoundary");
        expect(dockFailReason("docking failed: marker not found after 3 attempts").key).toBe("missionText.dockReasonMarker");
        expect(dockFailReason("dock route: start (1.00, 2.00) is off the route graph and no ...").key).toBe("missionText.dockReasonRoute");
        expect(dockFailReason("no docking server and no dock_pose configured").key).toBe("missionText.dockReasonNoDock");
        expect(dockFailReason("dock join transit aborted: Nav2 down").key).toBe("missionText.dockReasonNav");
        expect(dockFailReason("").key).toBe("missionText.dockReasonGeneric");
    });

    it("detects the heading hint and the attention states", () => {
        expect(isHeadingHint("heading not aligned: in manual, drive straight 1 m")).toBe(true);
        expect(isHeadingHint("area 0 sub-path 1/7")).toBe(false);
        expect(attentionTitleKey("IDLE", "heading not aligned: x")).toBe("missionText.headingTitle");
        expect(attentionTitleKey("STUCK_NEEDS_HELP", "")).toBe("missionText.stuckTitle");
        expect(attentionTitleKey("IDLE", "")).toBeNull();
        expect(plainSubState("MOWING", "area 0 sub-path 1/7: pivot -61 deg")).toBeNull();
        expect(plainSubState("IDLE", "heading not aligned: x")?.key).toBe("missionText.headingShort");
    });

    it("measures skipped stretches without the transit between sub-paths", () => {
        const plan = {plan_id: "p", subpaths: [[[0, 0], [3, 0], [3, 4]], [[10, 0], [10, 2]]] as [number, number][][]};
        // poses: 0 (0,0) 1 (3,0) 2 (3,4) | 3 (10,0) 4 (10,2); skip 1..4 = 4 m + 2 m, not the 2->3 jump
        expect(skippedLengthM(plan, {plan_id: "p", skipped: [[1, 4]]})).toBeCloseTo(6);
        expect(skippedLengthM(plan, {plan_id: "other", skipped: [[1, 4]]})).toBeNull();
        expect(skippedLengthM(plan, {plan_id: "p", skipped: []})).toBeNull();
    });
});
