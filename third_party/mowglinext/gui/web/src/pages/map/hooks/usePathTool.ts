import {useCallback, useEffect, useMemo, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import type MapboxDraw from "@mapbox/mapbox-gl-draw";
import type {Map as MapboxMap} from "mapbox-gl";
import type {Feature, FeatureCollection, Polygon, Position} from "geojson";
import type {NotificationInstance} from "antd/es/notification/interface";
import {
    DEFAULT_PATH_WIDTH_M, MAX_PATH_WIDTH_M, MIN_PATH_WIDTH_M,
    bufferPolyline, dockConnectorPolyline, localToLonLat, lonLatToLocal, snapPathToDock,
    type DockPose, type XY,
} from "../../../utils/corridor.ts";

const WIDTH_KEY = "mowgli.map.pathWidthM";
const SNAP_KEY = "mowgli.map.pathSnapToDock";

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

/** Pure: centreline (with optional dock snap) and corridor polygons in lon/lat. */
export function computeCorridor(
    datum: [number, number, number],
    coords: Position[],
    width: number,
    dock: PathDockInput | null,
    snap: boolean,
): { centerline: Position[]; polygons: Polygon[] } {
    let local: XY[] = coords.map((c) => lonLatToLocal(datum, c));
    if (snap && dock) {
        const [dx, dy] = lonLatToLocal(datum, dock.lonLat);
        local = snapPathToDock(local, {x: dx, y: dy, heading: dock.heading});
    }
    const polygons: Polygon[] = bufferPolyline(local, width).map((rings) => ({
        type: "Polygon",
        coordinates: rings.map((r) => r.map((p) => localToLonLat(datum, p))),
    }));
    return {centerline: local.map((p) => localToLonLat(datum, p)), polygons};
}

interface UsePathToolOptions {
    drawRef: React.RefObject<MapboxDraw | null>;
    mapInstanceRef: React.RefObject<MapboxMap | null>;
    datum: [number, number, number];
    dock: PathDockInput | null;
    /** Work-area outer rings ([lon, lat]) for "connect nearest area to dock". */
    workAreaRings: Position[][];
    navigationCount: number;
    addNavigationAreas: (geometries: Polygon[], name: string) => string[];
    notification: NotificationInstance;
}

export function usePathTool({
    drawRef, mapInstanceRef, datum, dock, workAreaRings, navigationCount, addNavigationAreas, notification,
}: UsePathToolOptions) {
    const {t} = useTranslation();
    const [drawing, setDrawing] = useState(false);
    const [liveCoords, setLiveCoords] = useState<Position[] | null>(null);
    const [draft, setDraft] = useState<Position[] | null>(null);
    const [name, setName] = useState("");
    const [width, setWidthState] = useState(() => readNumber(WIDTH_KEY, DEFAULT_PATH_WIDTH_M));
    const [snapToDock, setSnapState] = useState(() => readBool(SNAP_KEY, true));
    const drawingRef = useRef(false);
    useEffect(() => {
        drawingRef.current = drawing;
    }, [drawing]);

    const setWidth = useCallback((v: number) => {
        const w = Math.min(MAX_PATH_WIDTH_M, Math.max(MIN_PATH_WIDTH_M, v));
        setWidthState(w);
        write(WIDTH_KEY, String(w));
    }, []);
    const setSnapToDock = useCallback((v: boolean) => {
        setSnapState(v);
        write(SNAP_KEY, String(v));
    }, []);

    const startPath = useCallback(() => {
        setDraft(null);
        setLiveCoords(null);
        drawingRef.current = true;
        setDrawing(true);
        drawRef.current?.changeMode("draw_line_string");
    }, [drawRef]);

    // Live preview while clicking: mapbox-gl-draw keeps the in-progress line
    // (including the vertex following the cursor) in its store and fires
    // draw.render on every change. rAF-coalesced so a mousemove storm costs
    // at most one buffer per frame.
    useEffect(() => {
        const map = mapInstanceRef.current;
        if (!drawing || !map) return;
        let frame = 0;
        const onRender = () => {
            if (frame) return;
            frame = requestAnimationFrame(() => {
                frame = 0;
                const draw = drawRef.current;
                if (!draw || draw.getMode() !== "draw_line_string") return;
                const line = draw.getAll().features.find((f) => f.geometry.type === "LineString");
                const coords = line ? (line.geometry as GeoJSON.LineString).coordinates : [];
                setLiveCoords(coords.length >= 2 ? coords : null);
            });
        };
        // Leaving draw_line_string without finishing (Escape, tool switch)
        // ends the path tool; finishing goes through onLineCreated first.
        const onModeChange = (e: { mode?: string }) => {
            if (e.mode !== "draw_line_string" && drawingRef.current) {
                drawingRef.current = false;
                setDrawing(false);
                setLiveCoords(null);
            }
        };
        map.on("draw.render" as never, onRender);
        map.on("draw.modechange" as never, onModeChange as never);
        return () => {
            if (frame) cancelAnimationFrame(frame);
            map.off("draw.render" as never, onRender);
            map.off("draw.modechange" as never, onModeChange as never);
        };
    }, [drawing, drawRef, mapInstanceRef]);

    /**
     * Wraps the editor's draw.create handler: while the path tool is armed a
     * finished LineString becomes the path draft (removed from the draw store
     * — it is only a centreline) and opens the dialog. Returns true when it
     * consumed the event.
     */
    const onLineCreated = useCallback((features: Feature[]): boolean => {
        if (!drawingRef.current) return false;
        const line = features.find((f) => f.geometry?.type === "LineString");
        if (!line) return false;
        drawRef.current?.delete(String(line.id));
        drawingRef.current = false;
        setDrawing(false);
        setLiveCoords(null);
        setDraft((line.geometry as GeoJSON.LineString).coordinates);
        setName(t("mapPath.defaultName", {index: navigationCount + 1}));
        return true;
    }, [drawRef, navigationCount, t]);

    const corridor = useMemo(() => {
        const coords = draft ?? liveCoords;
        if (!coords || datum[0] === 0) return null;
        return computeCorridor(datum, coords, width, dock, snapToDock);
    }, [draft, liveCoords, datum, width, dock, snapToDock]);

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
        setDraft(null);
        setLiveCoords(null);
        if (drawingRef.current) drawRef.current?.changeMode("simple_select");
        drawingRef.current = false;
        setDrawing(false);
    }, [drawRef]);

    const save = useCallback(() => {
        if (!corridor || corridor.polygons.length === 0) return;
        addNavigationAreas(corridor.polygons, name);
        setDraft(null);
        setLiveCoords(null);
    }, [corridor, addNavigationAreas, name]);

    const connectNearestAreaToDock = useCallback(() => {
        if (!dock || datum[0] === 0) {
            notification.info({message: t("mapPath.noDock")});
            return;
        }
        const [dx, dy] = lonLatToLocal(datum, dock.lonLat);
        const pose: DockPose = {x: dx, y: dy, heading: dock.heading};
        const rings = workAreaRings.map((r) => r.map((p) => lonLatToLocal(datum, p)));
        const conn = dockConnectorPolyline(pose, rings, width / 2);
        if (!conn) {
            notification.info({message: t(rings.length ? "mapPath.alreadyConnected" : "mapPath.noWorkArea")});
            return;
        }
        const polygons: Polygon[] = bufferPolyline(conn.points, width).map((rs) => ({
            type: "Polygon",
            coordinates: rs.map((r) => r.map((p) => localToLonLat(datum, p))),
        }));
        addNavigationAreas(polygons, t("mapPath.dockConnectorName"));
        notification.success({
            message: t("mapPath.dockConnected", {gap: conn.gap.toFixed(1)}),
            description: t("mapPath.saveReminder"),
        });
    }, [dock, datum, workAreaRings, width, addNavigationAreas, notification, t]);

    return {
        drawing, draft, name, setName, width, setWidth, snapToDock, setSnapToDock,
        dockAvailable: dock !== null,
        invalid: !!draft && (!corridor || corridor.polygons.length === 0),
        previewCollection,
        startPath, onLineCreated, cancel, save, connectNearestAreaToDock,
    };
}
