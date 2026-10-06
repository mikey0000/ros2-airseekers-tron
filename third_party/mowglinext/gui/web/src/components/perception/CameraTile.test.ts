import {describe, expect, it} from "vitest";
import {tileHealth} from "./CameraTile.tsx";

const h = (age: number | null) => ({topic: "/t", fps: 5, lastFrameAgeMs: age, viewers: 1});
const st = (state: "publishing" | "not_publishing" | "listed") => ({state, listed: state !== "not_publishing", annotatedListed: false});

describe("tileHealth", () => {
    it("is live for a fresh frame and stale after 3 s", () => {
        expect(tileHealth(true, st("publishing"), h(200))).toBe("live");
        expect(tileHealth(true, st("publishing"), h(3500))).toBe("stale");
    });
    it("waits before the first frame and flags absent publishers", () => {
        expect(tileHealth(true, st("listed"), h(null))).toBe("waiting");
        expect(tileHealth(true, st("not_publishing"), undefined)).toBe("absent");
        expect(tileHealth(false, st("listed"), undefined)).toBe("waiting");
    });
});
