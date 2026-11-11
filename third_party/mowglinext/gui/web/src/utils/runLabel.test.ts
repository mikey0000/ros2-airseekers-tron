import {describe, expect, it} from "vitest";
import i18n from "../i18n";
import {formatRunLabel} from "./runLabel.ts";
import {parseActiveAreaSettings} from "../hooks/useAreaSettings.ts";

const t = i18n.t.bind(i18n);

describe("formatRunLabel", () => {
    it("names the run, its rows and the next run's rows", () => {
        expect(formatRunLabel(t, {run: 1, runs: 2, runLineAnglesDeg: [105, 15]}))
            .toBe("Run 1/2 at 105°, next at 15°");
        expect(formatRunLabel(t, {run: 2, runs: 2, runLineAnglesDeg: [105, 15]}))
            .toBe("Run 2/2 at 15°");
    });

    it("leaves out angles that are not known yet (auto, not planned)", () => {
        expect(formatRunLabel(t, {run: 1, runs: 2, runLineAnglesDeg: [null, null]})).toBe("Run 1/2");
        expect(formatRunLabel(t, {run: 1, runs: 2})).toBe("Run 1/2");
        expect(formatRunLabel(t, {run: 1, runs: 2, runLineAnglesDeg: [104.6, null]})).toBe("Run 1/2 at 105°");
    });

    it("is null without a run", () => {
        expect(formatRunLabel(t, null)).toBeNull();
        expect(formatRunLabel(t, {})).toBeNull();
        expect(formatRunLabel(t, {run: 1, runs: 0})).toBeNull();
    });

    it("reads run_line_angles_deg from the active settings topic", () => {
        const a = parseActiveAreaSettings({path_mode: "cross", run: 1, runs: 2, run_mow_angle_deg: -1,
            run_line_angles_deg: [105, null, "x"]});
        expect(a?.runLineAnglesDeg).toEqual([105, null, null]);
        expect(formatRunLabel(t, a)).toBe("Run 1/2 at 105°");
    });
});
