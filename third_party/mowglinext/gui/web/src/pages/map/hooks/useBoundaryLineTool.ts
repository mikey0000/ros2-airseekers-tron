import {useCallback, useEffect, useMemo, useReducer} from "react";
import type MapboxDraw from "@mapbox/mapbox-gl-draw";
import type {Map as MapboxMap} from "mapbox-gl";
import type {FeatureCollection, Position} from "geojson";
import {
    boundaryLineToolReducer,
    canFinishBoundaryLine,
    initialBoundaryLineToolState,
} from "./boundaryLineToolState.ts";
import {distanceM} from "../utils/mapHitTest.ts";

interface UseBoundaryLineToolOptions {
    drawRef: React.RefObject<MapboxDraw | null>;
    mapInstanceRef: React.RefObject<MapboxMap | null>;
    /** Called with the placed vertices ([lon, lat], >= 2) when the line is finished. */
    onFinish: (points: Position[]) => void;
}

/**
 * Boundary line drawing: map clicks add vertices (MapPage routes them here
 * while `drawing`), double-click / Enter / Finish ends the line, Escape /
 * Cancel abandons it, and switching to another mapbox-gl-draw tool cancels it.
 */
export function useBoundaryLineTool({drawRef, mapInstanceRef, onFinish}: UseBoundaryLineToolOptions) {
    const [state, dispatch] = useReducer(boundaryLineToolReducer, undefined, initialBoundaryLineToolState);
    const drawing = state.phase === "drawing";

    const start = useCallback(() => {
        drawRef.current?.changeMode("simple_select", {featureIds: []});
        dispatch({type: "start"});
    }, [drawRef]);
    const cancel = useCallback(() => dispatch({type: "cancel"}), []);
    const undoVertex = useCallback(() => dispatch({type: "undo"}), []);
    const setCursor = useCallback((point: Position | null) => dispatch({type: "cursor", point}), []);

    /** Add a vertex; one closer than `minSpacingM` to the previous one is ignored (double-tap jitter). */
    const addVertex = useCallback((point: Position, minSpacingM = 0) => {
        const last = state.points[state.points.length - 1];
        if (last && minSpacingM > 0 && distanceM(last, point) < minSpacingM) return;
        dispatch({type: "add", point});
    }, [state.points]);

    const finish = useCallback(() => {
        if (!canFinishBoundaryLine(state)) return;
        onFinish(state.points);
        dispatch({type: "finish"});
    }, [state, onFinish]);

    // Escape cancels, Enter finishes.
    useEffect(() => {
        if (!drawing) return;
        const onKeyDown = (e: KeyboardEvent) => {
            if (e.key === "Escape") cancel();
            else if (e.key === "Enter") finish();
        };
        window.addEventListener("keydown", onKeyDown);
        return () => window.removeEventListener("keydown", onKeyDown);
    }, [drawing, cancel, finish]);

    // Another editor tool (draw polygon, path, shape...) switches the draw
    // mode away from simple_select: abandon the boundary line.
    useEffect(() => {
        const map = mapInstanceRef.current;
        if (!drawing || !map) return;
        const onRender = () => {
            const mode = drawRef.current?.getMode();
            if (mode && mode !== "simple_select") cancel();
        };
        map.on("draw.render" as never, onRender);
        return () => {
            map.off("draw.render" as never, onRender);
        };
    }, [drawing, drawRef, mapInstanceRef, cancel]);

    const previewCollection = useMemo<FeatureCollection>(() => {
        if (!drawing) return {type: "FeatureCollection", features: []};
        const line = state.cursor && state.points.length > 0 ? [...state.points, state.cursor] : state.points;
        return {
            type: "FeatureCollection",
            features: [
                ...(line.length >= 2 ? [{
                    type: "Feature" as const, id: "boundary-preview-line", properties: {kind: "line"},
                    geometry: {type: "LineString" as const, coordinates: line},
                }] : []),
                ...state.points.map((p, i) => ({
                    type: "Feature" as const, id: `boundary-preview-vertex-${i}`, properties: {kind: "vertex"},
                    geometry: {type: "Point" as const, coordinates: p},
                })),
            ],
        };
    }, [drawing, state.points, state.cursor]);

    return {
        drawing,
        pointCount: state.points.length,
        canFinish: canFinishBoundaryLine(state),
        start, cancel, undoVertex, addVertex, finish, setCursor,
        previewCollection,
    };
}
