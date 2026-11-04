// Charger-moved check (mower_docking dock_moved.py): after each RTK-fixed dock the
// docking node compares the docked pose with the stored dock pose and latches the
// result on /mower_docking/dock_moved (topic key `dockMoved`). The dock card offers
// the re-calibration while the charger looks moved.
import {parseStringMsgJson} from "./areaSettings.ts";

export interface DockMovedResult {
    moved: boolean;
    /** Docked pose to stored dock pose, metres. */
    distance_m: number;
    /** Map-frame x, y where the robot docked (the charger's position now). */
    docked: [number, number];
    /** Map-frame x, y of the stored dock pose at the time of the check. */
    stored: [number, number];
    warn_m: number;
}

const isXY = (v: unknown): v is [number, number] =>
    Array.isArray(v) && v.length >= 2 && v.every((n) => typeof n === "number" && Number.isFinite(n));

/** std_msgs/String JSON -> result; undefined for anything malformed. */
export function selectDockMoved(raw: unknown): DockMovedResult | undefined {
    const d = parseStringMsgJson(raw);
    if (!d || typeof d.moved !== "boolean" || typeof d.distance_m !== "number"
        || !isXY(d.docked) || !isXY(d.stored)) return undefined;
    return {
        moved: d.moved,
        distance_m: d.distance_m,
        docked: [d.docked[0], d.docked[1]],
        stored: [d.stored[0], d.stored[1]],
        warn_m: typeof d.warn_m === "number" ? d.warn_m : 0.5,
    };
}

/**
 * True while the re-calibration should be offered: the last check found the
 * charger moved and the dock pose stored now is still not where the robot docked
 * (once the owner re-sets the dock, the latched result is stale and the offer goes).
 */
export function dockMoveNeedsRecalibration(
    result: DockMovedResult | null | undefined,
    storedNow: {x: number; y: number} | null | undefined,
): boolean {
    if (!result?.moved) return false;
    if (!storedNow) return true;
    const d = Math.hypot(storedNow.x - result.docked[0], storedNow.y - result.docked[1]);
    return d > result.warn_m;
}
