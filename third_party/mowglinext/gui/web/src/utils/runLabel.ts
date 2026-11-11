// Which run of a multi-run area (cross: two passes; repeat; alternate) the
// mission is mowing, from /behavior_tree_node/active_area_settings: "Run 1/2
// at 105°, next at 15°". Pure formatting; the angles are the planned row
// directions (run_line_angles_deg; null = auto, not planned yet).
import type {TFunction} from "i18next";

export interface RunInfo {
    run?: number;
    runs?: number;
    runLineAnglesDeg?: (number | null)[];
}

/** Row angle of run `run` (1-based), or null when unknown. */
function lineAngle(info: RunInfo, run: number): number | null {
    const a = info.runLineAnglesDeg?.[run - 1];
    return typeof a === "number" && Number.isFinite(a) ? Math.round(a) % 180 : null;
}

/**
 * "Run 1/2 at 105°, next at 15°" (parts left out when unknown), or null when
 * the active settings name no run.
 */
export function formatRunLabel(t: TFunction, info: RunInfo | null | undefined): string | null {
    if (!info || info.run === undefined || info.runs === undefined || info.runs < 1) return null;
    const angle = lineAngle(info, info.run);
    let label = angle === null
        ? t("areaSettings.run", {run: info.run, runs: info.runs})
        : t("areaSettings.runAt", {run: info.run, runs: info.runs, angle});
    if (info.run < info.runs) {
        const next = lineAngle(info, info.run + 1);
        if (next !== null) label += t("areaSettings.runNextAt", {angle: next});
    }
    return label;
}
