/**
 * Pure state of the map editor's "Merge nearby points" / "Simplify" dialog
 * (useVertexTools drives it). The dialog edits one feature at a time:
 *
 *   closed --open(tool, feature)--> open --tolerance--> open (preview updates)
 *   open --confirm / cancel--> closed
 *
 * The preview is derived (previewVertexTool), never stored, so it always
 * matches the feature geometry and the current tolerance.
 */
import type {LineString, Polygon, Position} from "geojson";
import {
    MERGE_DEFAULT_M,
    SIMPLIFY_DEFAULT_M,
    countVertices,
    mapPolygonRings,
    mergeNearbyVertices,
    simplifyVertices,
    type VertexEditFailure,
} from "../utils/vertexEdit.ts";

export type VertexTool = "merge" | "simplify";

export const TOLERANCE_MIN_M = 0.01;
export const TOLERANCE_MAX_M = 5;

export interface VertexToolState {
    tool: VertexTool | null;
    featureId: string | null;
    /** Last tolerance per tool, kept across dialog openings in a session. */
    tolerance: Record<VertexTool, number>;
}

export type VertexToolAction =
    | { type: "open"; tool: VertexTool; featureId: string }
    | { type: "tolerance"; value: number | null }
    | { type: "close" };

export const initialVertexToolState = (): VertexToolState => ({
    tool: null,
    featureId: null,
    tolerance: {merge: MERGE_DEFAULT_M, simplify: SIMPLIFY_DEFAULT_M},
});

export const clampTolerance = (v: number): number =>
    Math.min(TOLERANCE_MAX_M, Math.max(TOLERANCE_MIN_M, v));

export function vertexToolReducer(state: VertexToolState, action: VertexToolAction): VertexToolState {
    switch (action.type) {
        case "open":
            return {...state, tool: action.tool, featureId: action.featureId};
        case "tolerance":
            if (!state.tool || action.value === null || !Number.isFinite(action.value)) return state;
            return {...state, tolerance: {...state.tolerance, [state.tool]: clampTolerance(action.value)}};
        case "close":
            return {...state, tool: null, featureId: null};
    }
}

export type EditableGeometry = Polygon | LineString;

export type VertexToolPreview =
    | { ok: true; before: number; after: number; geometry: EditableGeometry }
    | { ok: false; before: number; reason: VertexEditFailure };

/** Run the open tool on `geometry` at the current tolerance. */
export function previewVertexTool(state: VertexToolState, geometry: EditableGeometry): VertexToolPreview | null {
    if (!state.tool) return null;
    const tol = state.tolerance[state.tool];
    const before = countVertices(geometry);
    const op = (closed: boolean) => (pts: Position[]) =>
        state.tool === "merge" ? mergeNearbyVertices(pts, tol, closed) : simplifyVertices(pts, tol, closed);
    if (geometry.type === "Polygon") {
        const r = mapPolygonRings(geometry.coordinates, op(true));
        if (!r.ok) return {ok: false, before, reason: r.reason};
        return {ok: true, before, after: before - r.removed, geometry: {type: "Polygon", coordinates: r.rings ?? []}};
    }
    const r = op(false)(geometry.coordinates);
    if (!r.ok) return {ok: false, before, reason: r.reason};
    return {ok: true, before, after: r.points.length, geometry: {type: "LineString", coordinates: r.points}};
}
