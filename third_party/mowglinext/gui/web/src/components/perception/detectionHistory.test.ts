import {describe, expect, it} from "vitest";
import {addDetections, EMPTY_HISTORY, sortedHistogram} from "./detectionHistory.ts";

const box = (c: string, score = 0.5) => ({class: c, score, x: 0, y: 0, w: 1, h: 1});

describe("detection history", () => {
    it("keeps the latest message per camera and a running class histogram", () => {
        let h = addDetections(EMPTY_HISTORY, {count: 2, classes: ["person"], max_score: 0.9, frame_id: "left_oa_camera",
            boxes: [box("person", 0.9), box("person")]}, 1);
        h = addDetections(h, {count: 1, classes: ["chair"], max_score: 0.5, frame_id: "right_oa_camera",
            boxes: [box("chair")]}, 2);
        h = addDetections(h, {count: 0, classes: [], max_score: 0, frame_id: "left_oa_camera", boxes: []}, 3);
        expect(Object.keys(h.byFrame).sort()).toEqual(["left_oa_camera", "right_oa_camera"]);
        expect(h.byFrame.left_oa_camera.summary.count).toBe(0);
        expect(h.byFrame.left_oa_camera.at).toBe(3);
        expect(h.histogram).toEqual({person: 2, chair: 1});
        expect(h.messages).toBe(3);
        expect(sortedHistogram(h.histogram)).toEqual([["person", 2], ["chair", 1]]);
    });

    it("falls back to classes when the backend sends no boxes", () => {
        const h = addDetections(EMPTY_HISTORY, {count: 3, classes: ["dog"], max_score: 0.4}, 1);
        expect(h.histogram).toEqual({dog: 1});
        expect(Object.keys(h.byFrame)).toEqual(["?"]);
    });
});
