import {useCallback, useEffect, useMemo, useReducer, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import type MapboxDraw from "@mapbox/mapbox-gl-draw";
import type {Map as MapboxMap} from "mapbox-gl";
import type {Feature, FeatureCollection, Polygon, Position} from "geojson";
import type {NotificationInstance} from "antd/es/notification/interface";
import {
    DEFAULT_PATH_WIDTH_M, MAX_PATH_WIDTH_M, MIN_PATH_WIDTH_M,
    bufferPolyline, dockApproachPoint, dockConnectorPolyline, localToLonLat, lonLatToLocal, snapPathEnds,
    type DockPose, type PathEndSnap, type XY,
} from "../../../utils/corridor.ts";
import {initialPathToolState, pathToolReducer} from "./pathToolState.ts";
import type {PathMeta} from "./useMapEditing.ts";
import type {NavigationFeature} from "../../../types/map.ts";

const WIDTH_KEY = "mowgli.map.pathWidthM";
const EXTEND_KEY = "mowgli.map.pathExtendToDock";

// Per-viewer conveniences only; storage may be unavailable (private mode).
function readNumber(key: string, fallback: number): number {
    try {
        const v = parseFloat(window.localStorage.getItem(key) ?? "");
        return Number.isFinite(v) ? Math.min(MAX_PATH_WIDTH_M, Math.max(MIN_PATH_WIDTH_M, v)) : fallback;
    } catch {
        return fallback;
    }
}

function readBool(key: string, fallback: boolean): boolean {
    try {
        const v = window.localStorage.getItem(key);
        return v === null ? fallback : v === "true";
    } catch {
        return fallback;
    }
}

function write(key: string, value: string) {
    try {
        window.localStorage.setItem(key, value);
    } catch {
        // ignore — width just won't be remembered
    }
}

export interface PathDockInput {
    /** Dock position as displayed ([lon, lat]). */
    lonLat: Position;
    heading: number;
}

export interface CorridorResult {
    /** Snapped centreline (lon/lat) — stored as the path's channel metadata. */
    centerline: Position[];
    polygons: Polygon[];
    start: PathEndSnap;
    end: PathEndSnap;
}

/**
 * Pure: snap the ends (area outline / dock approach pose, 0.5 m), optionally
 * continue a dock-snapped end into the dock itself, and buffer the line into
 * band polygons (square caps) in lon/lat.
 */
export function computeCorridor(
    datum: [number, number, number],
    coords: Position[],
    width: number,
    dock: PathDockInput | null,
    extendToDock: boolean,
    areaRings: Position[][] = [],
): CorridorResult {
    const local: XY[] = coords.map((c) => lonLatToLocal(datum, c));
    let pose: DockPose | null = null;
    if (dock) {
        const [dx, dy] = lonLatToLocal(datum, dock.lonLat);
        pose = {x: dx, y: dy, heading: dock.heading};
    }
    const rings = areaRings.map((r) => r.map((p) => lonLatToLocal(datum, p)));
    const snapped = snapPathEnds(local, rings, pose);
    let pts = snapped.points;
    if (extendToDock && pose && pts.length >= 2) {
        if (snapped.end === "dock") pts = [...pts, [pose.x, pose.y]];
        if (snapped.start === "dock") pts = [[pose.x, pose.y], ...pts];
    }
    const polygons: Polygon[] = bufferPolyline(pts, width).map((rings2) => ({
        type: "Polygon",
        coordinates: rings2.map((r) => r.map((p) => localToLonLat(datum, p))),
    }));
    return {centerline: pts.map((p) => localToLonLat(datum, p)), polygons, start: snapped.start, end: snapped.end};
}

interface UsePathToolOptions {
    drawRef: React.RefObject<MapboxDraw | null>;
    mapInstanceRef: React.RefObject<MapboxMap | null>;
    datum: [number, number, number];
    dock: PathDockInput | null;
    /** Work-area outer rings ([lon, lat]): snapping targets and "connect nearest area to dock". */
    workAreaRings: Position[][];
    navigationCount: number;
    addNavigationAreas: (geometries: Polygon[], name: string, path?: PathMeta) => string[];
    replacePathArea: (id: string, geometries: Polygon[], name: string, path?: PathMeta) => void;
    deleteFeature: (id: string) => void;
    notification: NotificationInstance;
}

export function usePathTool({
    drawRef, mapInstanceRef, datum, dock, workAreaRings, navigationCount, addNavigationAreas,
    replacePathArea, deleteFeature, notification,
}: UsePathToolOptions) {
    const {t} = useTranslation();
    const [state, dispatch] = useReducer(pathToolReducer, undefined,
        () => initialPathToolState(readNumber(WIDTH_KEY, DEFAULT_PATH_WIDTH_M)));
    const [extendToDock, setExtendState] = useState(() => readBool(EXTEND_KEY, false));
    const stateRef = useRef(state);
    useEffect(() => {
        stateRef.current = state;
    }, [state]);

    const setWidth = useCallback((v: number) => {
        const w = Math.min(MAX_PATH_WIDTH_M, Math.max(MIN_PATH_WIDTH_M, v));
        dispatch({type: "width", width: w});
        write(WIDTH_KEY, String(w));
    }, []);
    const setName = useCallback((name: string) => dispatch({type: "name", name}), []);
    const setExtendToDock = useCallback((v: boolean) => {
        setExtendState(v);
        write(EXTEND_KEY, String(v));
    }, []);

    const removeLine = useCallback(() => {
        const id = stateRef.current.lineId;
        const draw = drawRef.current;
        if (draw && id && draw.get(id)) draw.delete(id);
    }, [drawRef]);

    const dockApproachLonLat = useCallback((): Position | null => {
        if (!dock || datum[0] === 0) return null;
        const [dx, dy] = lonLatToLocal(datum, dock.lonLat);
        return localToLonLat(datum, dockApproachPoint({x: dx, y: dy, heading: dock.heading}));
    }, [dock, datum]);

    /** Arm line drawing. `fromDock`: the line starts at the dock approach pose ("Path to dock"). */
    const startPathImpl = useCallback((fromDock: boolean) => {
        const draw = drawRef.current;
        if (!draw) return;
        removeLine();
        if (fromDock) {
            const ap = dockApproachLonLat();
            if (!ap) {
                notification.info({message: t("mapPath.noDock")});
                return;
            }
            // A 2-vertex seed line [ap, ap] continued from its end: the first
            // click adds the second real vertex. Finishing fires draw.create.
            const [id] = draw.add({type: "Feature", properties: {}, geometry: {type: "LineString", coordinates: [ap, ap]}});
            dispatch({type: "start", fromDock: true, lineId: id});
            stateRef.current = {...stateRef.current, phase: "drawing", lineId: id};
            draw.changeMode("draw_line_string", {featureId: id, from: ap} as never);
            return;
        }
        dispatch({type: "start"});
        stateRef.current = {...stateRef.current, phase: "drawing", lineId: null};
        draw.changeMode("draw_line_string");
    }, [drawRef, removeLine, dockApproachLonLat, notification, t]);

    const startPath = useCallback(() => startPathImpl(false), [startPathImpl]);
    const startPathToDock = useCallback(() => startPathImpl(true), [startPathImpl]);

    /** Re-open a saved path as an editable line (drag vertices / width / name / delete). */
    const editPath = useCallback((feature: NavigationFeature) => {
        const draw = drawRef.current;
        const ch = feature.getChannel();
        if (!draw || !ch) return;
        removeLine();
        const [lineId] = draw.add({type: "Feature", properties: {}, geometry: {type: "LineString", coordinates: ch.points}});
        dispatch({type: "editExisting", featureId: feature.id, lineId, coords: ch.points,
            name: feature.getName(), width: ch.widthM});
        setTimeout(() => draw.changeMode("direct_select", {featureId: lineId}), 0);
    }, [drawRef, removeLine]);

    // Follow the line in the draw store: live preview while clicking (the
    // vertex following the cursor included) and while dragging vertices in
    // direct_select. rAF-coalesced: at most one buffer per frame.
    const active = state.phase !== "idle";
    useEffect(() => {
        const map = mapInstanceRef.current;
        if (!active || !map) return;
        let frame = 0;
        const onRender = () => {
            if (frame) return;
            frame = requestAnimationFrame(() => {
                frame = 0;
                const draw = drawRef.current;
                if (!draw) return;
                const s = stateRef.current;
                let line = s.lineId ? draw.get(s.lineId) : undefined;
                if (!line && s.phase === "drawing" && draw.getMode() === "draw_line_string") {
                    line = draw.getAll().features.find((f) => f.geometry.type === "LineString");
                }
                if (!line || line.geometry.type !== "LineString") return;
                const coords = (line.geometry).coordinates;
                if (coords.length >= 2) dispatch({type: "coords", coords});
            });
        };
        // Leaving draw_line_string without finishing (Escape, tool switch)
        // aborts; finishing goes through onLineCreated first.
        const onModeChange = (e: { mode?: string }) => {
            if (e.mode !== "draw_line_string" && stateRef.current.phase === "drawing") {
                removeLine();
                dispatch({type: "aborted"});
            }
        };
        map.on("draw.render" as never, onRender);
        map.on("draw.modechange" as never, onModeChange as never);
        return () => {
            if (frame) cancelAnimationFrame(frame);
            map.off("draw.render" as never, onRender);
            map.off("draw.modechange" as never, onModeChange as never);
        };
    }, [active, drawRef, mapInstanceRef, removeLine]);

    /**
     * Wraps the editor's draw.create handler: while the path tool is drawing
     * a finished LineString becomes the path being edited — it stays in the
     * draw store (direct_select) so its vertices can be dragged, and the path
     * panel opens. Returns true when it consumed the event.
     */
    const onLineCreated = useCallback((features: Feature[]): boolean => {
        if (stateRef.current.phase !== "drawing") return false;
        const line = features.find((f) => f.geometry?.type === "LineString");
        if (!line) return false;
        const lineId = String(line.id);
        const coords = (line.geometry as GeoJSON.LineString).coordinates;
        dispatch({type: "lineFinished", lineId, coords,
            defaultName: stateRef.current.fromDock ? t("mapPath.dockPathName")
                : t("mapPath.defaultName", {index: navigationCount + 1})});
        stateRef.current = {...stateRef.current, phase: "editing", lineId};
        setTimeout(() => {
            const draw = drawRef.current;
            if (draw?.get(lineId)) draw.changeMode("direct_select", {featureId: lineId});
        }, 0);
        return true;
    }, [drawRef, navigationCount, t]);

    const corridor = useMemo(() => {
        if (!state.coords || state.coords.length < 2 || datum[0] === 0) return null;
        return computeCorridor(datum, state.coords, state.width, dock, extendToDock, workAreaRings);
    }, [state.coords, state.width, datum, dock, extendToDock, workAreaRings]);

    const previewCollection = useMemo<FeatureCollection>(() => ({
        type: "FeatureCollection",
        features: corridor ? [
            ...corridor.polygons.map((g, i) => ({
                type: "Feature" as const, id: `path-preview-${i}`, geometry: g, properties: {kind: "corridor"},
            })),
            {
                type: "Feature" as const, id: "path-preview-line",
                geometry: {type: "LineString" as const, coordinates: corridor.centerline},
                properties: {kind: "centerline"},
            },
        ] : [],
    }), [corridor]);

    const cancel = useCallback(() => {
        const wasDrawing = stateRef.current.phase === "drawing";
        removeLine();
        dispatch({type: "cancel"});
        stateRef.current = {...stateRef.current, phase: "idle", lineId: null};
        const draw = drawRef.current;
        if (draw && (wasDrawing || draw.getMode() === "direct_select")) draw.changeMode("simple_select");
    }, [drawRef, removeLine]);

    const save = useCallback(() => {
        if (!corridor || corridor.polygons.length === 0) return;
        const meta: PathMeta = {centerline: corridor.centerline, widthM: state.width};
        if (state.editFeatureId) replacePathArea(state.editFeatureId, corridor.polygons, state.name, meta);
        else addNavigationAreas(corridor.polygons, state.name, meta);
        cancel();
    }, [corridor, state.width, state.name, state.editFeatureId, replacePathArea, addNavigationAreas, cancel]);

    const remove = useCallback(() => {
        const id = stateRef.current.editFeatureId;
        cancel();
        if (id) deleteFeature(id);
    }, [cancel, deleteFeature]);

    const connectNearestAreaToDock = useCallback(() => {
        if (!dock || datum[0] === 0) {
            notification.info({message: t("mapPath.noDock")});
            return;
        }
        const [dx, dy] = lonLatToLocal(datum, dock.lonLat);
        const pose: DockPose = {x: dx, y: dy, heading: dock.heading};
        const rings = workAreaRings.map((r) => r.map((p) => lonLatToLocal(datum, p)));
        const conn = dockConnectorPolyline(pose, rings, state.width / 2);
        if (!conn) {
            notification.info({message: t(rings.length ? "mapPath.alreadyConnected" : "mapPath.noWorkArea")});
            return;
        }
        const polygons: Polygon[] = bufferPolyline(conn.points, state.width).map((rs) => ({
            type: "Polygon",
            coordinates: rs.map((r) => r.map((p) => localToLonLat(datum, p))),
        }));
        addNavigationAreas(polygons, t("mapPath.dockConnectorName"),
            {centerline: conn.points.map((p) => localToLonLat(datum, p)), widthM: state.width});
        notification.success({
            message: t("mapPath.dockConnected", {gap: conn.gap.toFixed(1)}),
            description: t("mapPath.saveReminder"),
        });
    }, [dock, datum, workAreaRings, state.width, addNavigationAreas, notification, t]);

    return {
        phase: state.phase,
        drawing: state.phase === "drawing",
        editing: state.phase === "editing",
        editingExisting: state.editFeatureId !== null,
        name: state.name, setName, width: state.width, setWidth,
        extendToDock, setExtendToDock,
        dockAvailable: dock !== null,
        start: corridor?.start ?? "none",
        end: corridor?.end ?? "none",
        invalid: state.phase === "editing" && (!corridor || corridor.polygons.length === 0),
        previewCollection,
        startPath, startPathToDock, editPath, onLineCreated, cancel, save, remove, connectNearestAreaToDock,
    };
}
