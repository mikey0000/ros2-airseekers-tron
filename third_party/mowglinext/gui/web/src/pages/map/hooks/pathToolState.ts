/**
 * Pure state machine of the map editor's path tool (usePathTool drives it).
 *
 *   idle --start--> drawing --lineFinished--> editing --saved/cancel--> idle
 *   idle --editExisting--> editing
 *   drawing --aborted (Escape / tool switch)--> idle
 *
 * While editing, `lineId` is the temporary LineString in the MapboxDraw store
 * whose vertices the operator drags; `coords` mirrors its coordinates (raw,
 * before end snapping). `editFeatureId` is the navigation area being
 * re-edited (null for a new path).
 */
import type {Position} from "geojson";
import {DEFAULT_PATH_WIDTH_M, MAX_PATH_WIDTH_M, MIN_PATH_WIDTH_M} from "../../../utils/corridor.ts";

export type PathPhase = "idle" | "drawing" | "editing";

export interface PathToolState {
    phase: PathPhase;
    /** The line starts at the dock approach pose ("Path to dock"). */
    fromDock: boolean;
    lineId: string | null;
    editFeatureId: string | null;
    coords: Position[] | null;
    name: string;
    width: number;
}

export type PathToolAction =
    | { type: "start"; fromDock?: boolean; lineId?: string | null }
    | { type: "aborted" }
    | { type: "lineFinished"; lineId: string; coords: Position[]; defaultName: string }
    | { type: "editExisting"; featureId: string; lineId: string; coords: Position[]; name: string; width: number }
    | { type: "coords"; coords: Position[] }
    | { type: "name"; name: string }
    | { type: "width"; width: number }
    | { type: "cancel" }
    | { type: "saved" };

export const clampWidth = (w: number) =>
    Number.isFinite(w) ? Math.min(MAX_PATH_WIDTH_M, Math.max(MIN_PATH_WIDTH_M, w)) : DEFAULT_PATH_WIDTH_M;

export function initialPathToolState(width = DEFAULT_PATH_WIDTH_M): PathToolState {
    return {phase: "idle", fromDock: false, lineId: null, editFeatureId: null, coords: null, name: "", width: clampWidth(width)};
}

export function pathToolReducer(s: PathToolState, a: PathToolAction): PathToolState {
    switch (a.type) {
        case "start":
            return {...s, phase: "drawing", fromDock: !!a.fromDock, lineId: a.lineId ?? null,
                editFeatureId: null, coords: null, name: ""};
        case "aborted":
            return s.phase === "drawing" ? {...s, phase: "idle", lineId: null, coords: null, fromDock: false} : s;
        case "lineFinished":
            if (s.phase !== "drawing") return s;
            return {...s, phase: "editing", lineId: a.lineId, coords: a.coords, name: a.defaultName};
        case "editExisting":
            return {...s, phase: "editing", fromDock: false, lineId: a.lineId, editFeatureId: a.featureId,
                coords: a.coords, name: a.name, width: clampWidth(a.width)};
        case "coords":
            return s.phase === "idle" ? s : {...s, coords: a.coords};
        case "name":
            return {...s, name: a.name};
        case "width":
            return {...s, width: clampWidth(a.width)};
        case "cancel":
        case "saved":
            return {...initialPathToolState(s.width)};
    }
}
