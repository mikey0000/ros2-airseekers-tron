/**
 * Pure state machine of the map editor's boundary line tool
 * (useBoundaryLineTool drives it).
 *
 *   idle --start--> drawing --finish (>= 2 points)--> idle
 *   drawing --cancel (Escape / Cancel / tool switch)--> idle
 *
 * While drawing, `points` are the placed vertices ([lon, lat]) and `cursor`
 * the pointer position for the rubber-band preview segment.
 */
import type {Position} from "geojson";

export const BOUNDARY_LINE_MIN_POINTS = 2;

export interface BoundaryLineToolState {
    phase: "idle" | "drawing";
    points: Position[];
    cursor: Position | null;
}

export type BoundaryLineToolAction =
    | { type: "start" }
    | { type: "add"; point: Position }
    | { type: "undo" }
    | { type: "cursor"; point: Position | null }
    | { type: "finish" }
    | { type: "cancel" };

export const initialBoundaryLineToolState = (): BoundaryLineToolState =>
    ({phase: "idle", points: [], cursor: null});

export const canFinishBoundaryLine = (s: BoundaryLineToolState) =>
    s.phase === "drawing" && s.points.length >= BOUNDARY_LINE_MIN_POINTS;

export function boundaryLineToolReducer(s: BoundaryLineToolState, a: BoundaryLineToolAction): BoundaryLineToolState {
    switch (a.type) {
        case "start":
            return {phase: "drawing", points: [], cursor: null};
        case "add": {
            if (s.phase !== "drawing") return s;
            // The two clicks of a double-click land on the same spot: one vertex.
            const last = s.points[s.points.length - 1];
            if (last && last[0] === a.point[0] && last[1] === a.point[1]) return s;
            return {...s, points: [...s.points, [a.point[0], a.point[1]]]};
        }
        case "undo":
            return s.phase === "drawing" && s.points.length > 0 ? {...s, points: s.points.slice(0, -1)} : s;
        case "cursor":
            return s.phase === "drawing" ? {...s, cursor: a.point} : s;
        case "finish":
            return canFinishBoundaryLine(s) ? initialBoundaryLineToolState() : s;
        case "cancel":
            return initialBoundaryLineToolState();
    }
}
