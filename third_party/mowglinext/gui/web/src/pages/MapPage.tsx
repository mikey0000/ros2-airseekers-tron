import {mowingAreaIndex} from "../utils/mapAreaIndex.ts";
import "@mapbox/mapbox-gl-draw/dist/mapbox-gl-draw.css";
import {useApi} from "../hooks/useApi.ts";
import {App, Button} from "antd";
import {useNavigate} from "react-router-dom";
import {datumErrorDetail, requestDatumFromGps} from "../utils/datumGps.ts";
import turfArea from "@turf/area";
import React, {useCallback, useEffect, useMemo, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import {MapArea, Map as MapType} from "../types/ros.ts";
import type {AreaChannel, MapWithChannels} from "../types/map.ts";
import DrawControl from "../components/DrawControl.tsx";
import Map, {Layer, Popup, Source} from 'react-map-gl/mapbox';
import turfCentroid from "@turf/centroid";
import type {Map as MapboxMap} from 'mapbox-gl';
import type {Feature} from 'geojson';
import {FeatureCollection, Position} from "geojson";
import {useMowerAction} from "../components/MowerActions.tsx";
import {MapStyle} from "./MapStyle.tsx";
import {drawLine, itranspose, transpose} from "../utils/map.tsx";
import {useSettings} from "../hooks/useSettings.ts";
import {useConfig} from "../hooks/useConfig.tsx";
import {useEnv} from "../hooks/useEnv.tsx";
import {Spinner} from "../components/Spinner.tsx";
import {MowingFeature, MowingAreaFeature, DockFeatureBase, MowingFeatureBase, NavigationFeature, ObstacleFeature, ActivePathFeature, PathFeature} from "../types/map.ts";
import {useMapEditHistory} from "./map/hooks/useMapEditHistory.ts";
import {useMapOffset} from "./map/hooks/useMapOffset.ts";
import {useMapBearing} from "./map/hooks/useMapBearing.ts";
import {useMapBearingCamera} from "./map/hooks/useMapBearingCamera.ts";
import {useManualMode} from "./map/hooks/useManualMode.ts";
import {useMapEditing} from "./map/hooks/useMapEditing.ts";
import {useMapStreams} from "./map/hooks/useMapStreams.ts";
import {useMapFiles, type ImportOpenMowerSummary} from "./map/hooks/useMapFiles.ts";
import {useResetMowingProgress} from "./map/hooks/useResetMowingProgress.tsx";
import {ImportOpenMowerModal} from "./map/components/ImportOpenMowerModal.tsx";
import {NewAreaModal} from "./map/components/NewAreaModal.tsx";
import {EditAreaModal} from "./map/components/EditAreaModal.tsx";
import {AreasListPanel} from "./map/components/AreasListPanel.tsx";
import {TrackedObstaclesPanel} from "./map/components/TrackedObstaclesPanel.tsx";
import {MapOffsetPanel} from "./map/components/MapOffsetPanel.tsx";
import {MapToolbar} from "./map/components/MapToolbar.tsx";
import {MapToolbarMobile} from "./map/components/MapToolbarMobile.tsx";
import {MapEditorToolbar} from "./map/components/MapEditorToolbar.tsx";
import {JoystickOverlay} from "./map/components/JoystickOverlay.tsx";
import {useIsMobile} from "../hooks/useIsMobile.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useAreaSettingsSupport} from "../hooks/useAreaSettings.ts";
import {MissionStopControls} from "../components/MissionStopControls.tsx";
import {StartMowSheet, type StartSelection} from "../components/areaSettings/StartMowSheet.tsx";
import {AreaSettingsDrawer} from "../components/areaSettings/AreaSettingsDrawer.tsx";
import {mowingAreaChoices} from "../utils/mapAreaIndex.ts";
import {pointInPolygon} from "../utils/map.tsx";
import {MapImageMarker} from "./map/components/MapImageMarker.tsx";
import {markerCorners, selectDockMarker} from "../utils/mapMarker.ts";
import {useRobotProfile} from "../hooks/useRobotProfile.ts";
import {hasFeature} from "../constants/robotProfiles.ts";
import {ManualBladeControl} from "./map/components/ManualBladeControl.tsx";
import {DrivingCameraPip} from "./map/components/DrivingCameraPip.tsx";
import {isReversing, useDrivePipPrefs} from "./map/hooks/useDrivingCamera.ts";
import {PathModal} from "./map/components/PathModal.tsx";
import {DockHeadingPanel} from "./map/components/DockHeadingPanel.tsx";
import {CorridorHatchPattern, CORRIDOR_HATCH_IMAGE} from "./map/components/CorridorHatchPattern.tsx";
import {usePathTool, type PathDockInput} from "./map/hooks/usePathTool.ts";
import {useDockCorridor} from "../hooks/useDockCorridor.ts";
import {clearPlanPreview, requestPlanPreview, usePlanPreview} from "../hooks/usePlanPreview.ts";
import {PlanPreviewCard} from "./map/components/PlanPreviewCard.tsx";
import {PREVIEW_ALL_KEY} from "./map/components/MapToolbar.tsx";
import {useMissionPlan, useMissionProgress} from "../hooks/useMissionProgress.ts";
import {appendTrack, planStretches, type TrackPoint} from "../utils/missionProgress.ts";
import {hasLiveProgress, MowProgressCard} from "./map/components/MowProgressCard.tsx";
import {MissionStatusLine} from "./map/components/MissionStatusLine.tsx";
import {useTopic} from "../hooks/useTopic.ts";
import {CMD_RECORD_PATH, recordingKind} from "../utils/missionStates.ts";
import type {AbsolutePose} from "../types/ros.ts";
import {postTerrainAction, useTerrainSummary} from "../hooks/useTerrain.ts";
import {TerrainCard} from "./map/components/TerrainCard.tsx";
import {TERRAIN_STATUS_COLORS} from "../utils/terrain.ts";


// Mapbox access token comes from the build env only — no hardcoded fallback.
// When it is missing the page renders a clear error panel instead of a broken
// (blank) map, so the misconfiguration is obvious rather than silent.
const MAPBOX_TOKEN = import.meta.env.VITE_MAPBOX_TOKEN as string | undefined || "REDACTED_MAPBOX_TOKEN";

// Layers the full map queries on hover to drive the two-way obstacle
// highlight (map polygon → panel row). Module-level so the array identity is
// stable across renders and react-map-gl does not re-bind the query on every
// render.
const DYN_OBSTACLE_INTERACTIVE_LAYERS = ['dyn-obstacle-fill', 'dock-corridor-fill', 'terrain-cluster-circle'];

/** Drive view: the map shrinks to an inset (top-right, clear of both joystick corners). */
const DRIVE_VIEW_MAP_INSET: React.CSSProperties = {
    position: 'absolute', top: 44, right: 12, width: '28%', height: '28%', minWidth: 140, minHeight: 110,
    zIndex: 45, borderRadius: 8, overflow: 'hidden', border: '1px solid rgba(255,255,255,0.35)',
    boxShadow: '0 10px 30px -10px rgba(0,0,0,0.7)',
};

export const MapPage: React.FC<{compact?: boolean}> = ({compact = false}) => {
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const navigate = useNavigate();
    const [datumBusy, setDatumBusy] = useState(false);
    const {colors, displayMode} = useThemeMode();
    const isMobile = useIsMobile();
    const mowerAction = useMowerAction()
    const resetMowingProgress = useResetMowingProgress()

    // Brand tokens for the Mapbox display-only layers (dock, mower, lidar).
    // Shared between the compact and full render branches so the two stay in
    // lockstep — never re-hardcode these hex values inline below.
    const LAYER_COLORS = useMemo(() => ({
        dock: colors.auroraViolet,           // dock marker + label
        dockHeading: colors.primary,         // dock heading line (lime)
        mower: colors.info,                  // mower center point (aurora-cyan)
        mowerOutline: colors.emeraldDeep,    // mower footprint outline (deep accent)
        lidarHit: colors.danger,             // lidar hit points (rose)
        lidarMiss: colors.amber,             // lidar miss points (amber)
        coveragePath: colors.mint,           // full F2C coverage plan line (mint)
        previewRing: colors.amber,           // plan preview: headland rings
        previewSwath: colors.mint,           // plan preview: swaths
        previewTransit: colors.auroraViolet, // plan preview: blade-off transits between sub-paths
        liveMowed: colors.emeraldDeep,       // live progress: plan stretches already mowed
        liveCurrent: colors.info,            // live progress: the sub-path being mowed now
        liveRemaining: colors.mint,          // live progress: still to mow (= preview swath colour)
        liveSkipped: '#FF7A1A',              // live progress: skipped stretches (orange; no theme token)
        track: colors.text,                  // robot's actual track (last 5 min)
        halo: colors.text,                   // white halo → ink
        labelText: colors.text,              // symbol label text → ink
        labelHalo: colors.bgBase,            // symbol label halo → deep bg
    }), [colors]);

    const {settings} = useSettings()
    const [labelsCollection, setLabelsCollection] = useState<FeatureCollection>({
        type: "FeatureCollection",
        features: []
    })
    const {config, setConfig} = useConfig(["gui.map.offset.x", "gui.map.offset.y", "gui.map.display.bearing"])
    const envs = useEnv()
    const guiApi = useApi()
    const [tileUri, setTileUri] = useState<string | undefined>()
    const [editMap, setEditMap] = useState<boolean>(false)
    // Shared hover/selection link between the tracked-obstacles panel and the
    // map. Hovering a panel row (or a map polygon) sets this id; both the
    // Mapbox highlight layer (via its filter) and the panel read it, so the
    // operator sees exactly which obstacle they're about to promote. null =
    // nothing highlighted.
    const [selectedObstacleId, setSelectedObstacleId] = useState<number | null>(null);
    const [features, setFeatures] = useState<Record<string, MowingFeature>>({});
    const [dockPlacementMode, setDockPlacementMode] = useState<boolean>(false);
    // OpenMower import preview — populated by handleImportOpenMower after
    // the file is uploaded + parsed server-side. Modal renders when set.
    const [importPreview, setImportPreview] = useState<ImportOpenMowerSummary | null>(null);
    // Verbatim text of the imported map.json. Stashed at preview time so
    // the apply step can re-POST byte-identical content (the server then
    // runs the same parse + validate flow before writing).
    const [importFileText, setImportFileText] = useState<string | null>(null);
    // Track whether the user actually moved the dock during this edit
    // session. The dock feature is rebuilt from the live /map topic on
    // every render, and saving without this flag would clobber the
    // persisted dock pose with a stale value (e.g. when dock_pose was
    // updated by the calibration service but the /map topic hasn't
    // re-emitted yet).
    const [dockDirty, setDockDirty] = useState<boolean>(false);
    const [mapKey, setMapKey] = useState<string>("origin")
    const [useSatellite, setUseSatellite] = useState(true)
    const [pitched, setPitched] = useState(false)
    const togglePitch = useCallback(() => {
        setPitched(prev => {
            const next = !prev;
            const m = mapInstanceRef.current;
            if (m) m.easeTo({pitch: next ? 50 : 0, duration: 600});
            return next;
        });
    }, [])
    const robotPoseRef = useRef<{ x: number; y: number; heading: number } | null>(null)
    const mapInstanceRef = useRef<MapboxMap | null>(null)
    const drawRef = useRef<import('@mapbox/mapbox-gl-draw').default | null>(null);

    // Only include editable polygon features for DrawControl — exclude mower,
    // paths, and other display-only features so that frequent pose updates don't
    // trigger DrawControl to deleteAll() + re-add, which wipes out selection state.
    // Stable ref ensures the array identity only changes when mowing areas actually
    // change, not on every pose update, preventing DrawControl sync timer thrashing.
    const prevMowingRef = useRef<GeoJSON.Feature[]>([]);
    const drawableFeatures = useMemo(() => {
        const next = Object.values(features).filter(f => f instanceof MowingFeatureBase) as GeoJSON.Feature[];
        const prev = prevMowingRef.current;
        const unchanged =
            next.length === prev.length &&
            next.every((f, i) => f.id === prev[i]?.id && JSON.stringify(f.geometry) === JSON.stringify(prev[i]?.geometry));
        if (unchanged) return prev;
        prevMowingRef.current = next;
        return next;
    }, [features]);

    // Extracted hooks
    const {offsetX, offsetY, handleOffsetX, handleOffsetY} = useMapOffset({config, setConfig, notification});
    const {bearing, handleBearing} = useMapBearing({config, setConfig, notification});

    const onMapLoad = useMapBearingCamera({mapInstanceRef, bearing, onBearingChange: handleBearing, interactive: !compact});

    const _datumLon = parseFloat(settings["datum_lon"] ?? 0)
    const _datumLat = parseFloat(settings["datum_lat"] ?? 0)

    // Datum as [lat, lon, 0] for equirectangular projection
    // (matches the navsat_to_absolute_pose ROS node projection)
    const datum = useMemo<[number, number, number]>(() => {
        if (_datumLon == 0 || _datumLat == 0) {
            return [0, 0, 0]
        }
        return [_datumLat, _datumLon, 0]
    }, [_datumLat, _datumLon])

    // Display-only features (mower, dock, heading, paths) rendered as separate layers
    const displayFeatures = useMemo<GeoJSON.FeatureCollection>(() => {
        const feats = Object.values(features)
            .filter(f => !(f instanceof MowingFeatureBase))
            .map(f => ({
                type: "Feature" as const,
                id: f.id,
                geometry: f.geometry,
                properties: f.properties,
            }));

        // Add dock heading direction line (longer, with contrasting color)
        const dock = features["dock"];
        if (dock instanceof DockFeatureBase) {
            const coords = dock.getCoordinates();
            const rosCoords = datum[0] !== 0 ? itranspose(offsetX, offsetY, datum, coords[1], coords[0]) : [0, 0];
            const endPoint = drawLine(offsetX, offsetY, datum, rosCoords[1], rosCoords[0], dock.getHeading());
            feats.push({
                type: "Feature" as const,
                id: "dock-heading",
                geometry: {type: "LineString", coordinates: [coords, endPoint]},
                properties: {color: LAYER_COLORS.dockHeading, width: 3, feature_type: "dock-heading"},
            });
        }

        return {type: "FeatureCollection", features: feats};
    }, [features, offsetX, offsetY, datum, LAYER_COLORS]);

    // Optional profile dock image (MowerModel.dockMarker); absent -> stock dot.
    const {profile: robotProfile} = useRobotProfile();
    const dockMarker = useMemo(() => selectDockMarker(robotProfile), [robotProfile]);
    const dockMarkerCorners = useMemo(() => {
        const dock = features["dock"];
        if (!dockMarker || !(dock instanceof DockFeatureBase) || datum[0] === 0) return null;
        const c = dock.getCoordinates();
        const [dx, dy] = itranspose(offsetX, offsetY, datum, c[1], c[0]);
        return markerCorners(offsetX, offsetY, datum, dockMarker, dx, dy, dock.getHeading());
    }, [dockMarker, features, offsetX, offsetY, datum]);
    // Remount the image markers when another image layer appears so they stay on top.

    // Layers for the persistent tracked-obstacle polygons (feature_type
    // 'dyn-obstacle', carried in the same display-features source). Rendered as
    // a translucent fill + rose outline + an id label, so the map and the
    // TrackedObstaclesPanel share one visible identifier. `withHighlight` adds
    // an amber outline on the currently selected/hovered obstacle — its filter
    // reads selectedObstacleId, so a hover only re-binds this one layer's
    // filter (no feature rebuild). Only the full map passes withHighlight; the
    // compact overview just shows the fill/outline/label.
    const renderDynObstacleLayers = (withHighlight: boolean) => (
        [
            <Layer key={"dyn-obstacle-fill"} type={"fill"} id={"dyn-obstacle-fill"}
                filter={['==', ['get', 'feature_type'], 'dyn-obstacle']}
                paint={{'fill-color': ['get', 'color']}}/>,
            <Layer key={"dyn-obstacle-outline"} type={"line"} id={"dyn-obstacle-outline"}
                filter={['==', ['get', 'feature_type'], 'dyn-obstacle']}
                paint={{'line-color': LAYER_COLORS.lidarHit, 'line-width': 2}}/>,
            ...(withHighlight ? [
                <Layer key={"dyn-obstacle-highlight"} type={"line"} id={"dyn-obstacle-highlight"}
                    filter={['all',
                        ['==', ['get', 'feature_type'], 'dyn-obstacle'],
                        ['==', ['get', 'obs_id'], selectedObstacleId ?? -1]]}
                    paint={{'line-color': LAYER_COLORS.lidarMiss, 'line-width': 4}}/>,
            ] : []),
            <Layer key={"dyn-obstacle-label"} type={"symbol"} id={"dyn-obstacle-label"}
                filter={['==', ['get', 'feature_type'], 'dyn-obstacle']}
                layout={{
                    'text-field': ['concat', '#', ['get', 'obs_label']],
                    'text-size': 13,
                    'text-font': ['Open Sans Bold'],
                    'text-allow-overlap': true,
                }}
                paint={{
                    'text-color': LAYER_COLORS.labelText,
                    'text-halo-color': LAYER_COLORS.labelHalo,
                    'text-halo-width': 1.5,
                }}/>,
        ]
    );

    const [mowingAreas, setMowingAreas] = useState<{ key: string, label: string, feat: Feature }[]>([])

    const {map, setMap, path, plan, lidarCollection, mowProgressImage, terrainImage, lidarMapImage, highLevelStatus, joyStream, dynamicObstacles, robotMarkerCorners, robotMarker} = useMapStreams({
        editMap,
        settings,
        offsetX,
        offsetY,
        datum,
        setFeatures,
        setEditMap,
        setMapKey,
        mapInstanceRef,
        robotPoseRef,
    });

    // Compute map bounds for the Mapbox viewport — depends on map data for centering
    const [map_ne, map_sw] = useMemo<[[number, number], [number, number]]>(() => {
        if (_datumLon == 0 || _datumLat == 0) {
            return [[0, 0], [0, 0]]
        }
        const map_center = (map && map.map_center_y && map.map_center_x) ? transpose(offsetX, offsetY, datum, map.map_center_y, map.map_center_x) : [_datumLon, _datumLat]
        // Use map center as datum for bounds calculation
        const centerDatum: [number, number, number] = [map_center[1], map_center[0], 0]
        const map_sw = transpose(0, 0, centerDatum, -((map?.map_height ?? 10) / 2), -((map?.map_width ?? 10) / 2))
        const map_ne = transpose(0, 0, centerDatum, ((map?.map_height ?? 10) / 2), ((map?.map_width ?? 10) / 2))
        return [map_ne, map_sw]
    }, [_datumLat, _datumLon, map, offsetX, offsetY, datum])

    const {
        hasUnsavedChanges, setHasUnsavedChanges, handleEditMap,
        handleUndo, handleRedo, historyIndex, editHistory,
    } = useMapEditHistory({features, setFeatures, editMap, setEditMap});

    useEffect(() => {
        if (envs) {
            setTileUri(envs.tileUri)
        }
    }, [envs]);

    const {
        modalOpen,
        areaModelOpen,
        newAreaName, setNewAreaName,
        newAreaType, setNewAreaType,
        curMowingAreaFeature, setCurMowingAreaFeature,
        selectedFeatureIds,
        buildLabels,
        onCreate, onUpdate, onCombine, onDelete, onSelectionChange, onOpenDetails,
        handleEditSelectedFeature, handleDrawPolygon, handleDrawShape, handleDrawEmoji,
        handleTrash, handleCombine,
        handleAreaSelect, handleSubtract, handleSplit,
        handleSaveNewArea, updateMowingArea, cancelAreaModal, deleteFeature,
        addNavigationAreas, replacePathArea,
    } = useMapEditing({
        features,
        setFeatures,
        editMap,
        mowingAreas,
        drawRef,
        notification,
        mapInstanceRef,
    });

    // Path tool (drive-only corridors, optionally snapped to the dock).
    const pathDock = useMemo<PathDockInput | null>(() => {
        const dock = features["dock"];
        if (!(dock instanceof DockFeatureBase)) return null;
        return {lonLat: dock.getCoordinates(), heading: dock.getHeading()};
    }, [features]);
    const workAreaRings = useMemo(() => Object.values(features)
        .filter((f): f is MowingAreaFeature => f instanceof MowingAreaFeature)
        .map((f) => f.geometry.coordinates[0] ?? [])
        .filter((r) => r.length >= 4), [features]);
    const navigationCount = useMemo(
        () => Object.values(features).filter((f) => f instanceof NavigationFeature).length, [features]);
    const deletePathFeature = useCallback((id: string) => {
        setFeatures((curr) => {
            const next = {...curr};
            delete next[id];
            return next;
        });
    }, [setFeatures]);
    const pathTool = usePathTool({
        drawRef, mapInstanceRef, datum, dock: pathDock, workAreaRings, navigationCount,
        addNavigationAreas, replacePathArea, deleteFeature: deletePathFeature, notification,
    });
    // "Edit path" is offered when exactly one saved path (navigation area
    // with centreline metadata) is selected.
    const selectedPath = useMemo(() => {
        if (selectedFeatureIds.length !== 1) return null;
        const f = features[selectedFeatureIds[0]];
        return f instanceof NavigationFeature && f.isPath() ? f : null;
    }, [selectedFeatureIds, features]);
    const {editPath} = pathTool;
    const handleEditPath = useCallback(() => {
        if (selectedPath) editPath(selectedPath);
    }, [selectedPath, editPath]);
    // Path recorded by driving (CMD_RECORD_PATH): when the mission reports
    // RECORDING_COMPLETE with the new path's name, wait for it in the polled map,
    // enter edit mode and open the path panel (name / width / connections) on it.
    const hlState = highLevelStatus.highLevelStatus.state_name;
    const hlSub = highLevelStatus.highLevelStatus.sub_state_name;
    const recordKind = recordingKind(hlState, hlSub);
    const lastRecordKind = useRef<"area" | "path" | null>(null);
    const [pendingPath, setPendingPath] = useState<{name: string; since: number} | null>(null);
    useEffect(() => {
        if (recordKind) lastRecordKind.current = recordKind;
        else if (hlState === "RECORDING_COMPLETE" && lastRecordKind.current === "path" && hlSub) {
            lastRecordKind.current = null;
            setPendingPath({name: hlSub, since: Date.now()});
        }
    }, [recordKind, hlState, hlSub]);
    useEffect(() => {
        if (!pendingPath || compact) return;
        const f = Object.values(features).find((x): x is NavigationFeature =>
            x instanceof NavigationFeature && x.isPath() && x.getName() === pendingPath.name);
        if (!f) {
            if (Date.now() - pendingPath.since > 20_000) setPendingPath(null);
            return;
        }
        if (!editMap) {
            handleEditMap();
            return;
        }
        const timer = setTimeout(() => { editPath(f); setPendingPath(null); }, 300);
        return () => clearTimeout(timer);
    }, [pendingPath, features, editMap, handleEditMap, editPath, compact]);
    // Saved paths: translucent band + dashed centreline, drawn above the
    // editor polygons so they read differently from mowing areas.
    const pathsCollection = useMemo<FeatureCollection>(() => ({
        type: "FeatureCollection",
        features: Object.values(features).flatMap((f) => {
            if (!(f instanceof NavigationFeature)) return [];
            const ch = f.getChannel();
            if (!ch || !f.geometry?.coordinates?.[0]?.length) return [];
            return [
                {type: "Feature" as const, id: `${f.id}-band`, geometry: f.geometry, properties: {kind: "band"}},
                {type: "Feature" as const, id: `${f.id}-line`, properties: {kind: "centerline"},
                    geometry: {type: "LineString" as const, coordinates: ch.points}},
            ];
        }),
    }), [features]);
    const {onLineCreated: pathOnLineCreated, cancel: pathCancel} = pathTool;
    const onCreateWithPath = useCallback((e: {features: Feature[]}) => {
        if (pathOnLineCreated(e.features)) return;
        onCreate(e);
    }, [pathOnLineCreated, onCreate]);
    // Leaving edit mode abandons an in-progress path.
    useEffect(() => {
        if (!editMap) pathCancel();
    }, [editMap, pathCancel]);

    // Automatic dock corridor published by the map server (latched; absent on
    // map servers that do not implement it -> nothing drawn).
    const dockCorridorPts = useDockCorridor(!compact);
    // Plan preview (POST /mowglinext/plan/preview): typed rings/swaths/transits
    // replace the plain coverage path while a preview is shown.
    const planPreview = usePlanPreview(!compact);
    const [previewHiddenId, setPreviewHiddenId] = useState<number | null>(null);
    // Live mow progress (mower_mission mow_progress.py): plan stretches coloured
    // mowed / current / remaining / skipped, plus the robot's actual track.
    const missionPlan = useMissionPlan(!compact);
    const missionProgress = useMissionProgress(!compact);
    const liveProgress = hasLiveProgress(missionProgress) ? missionProgress : null;
    const liveStretches = useMemo(() => planStretches(missionPlan, liveProgress), [missionPlan, liveProgress]);
    const liveDrawn = liveStretches.length > 0;
    const trackPose = useTopic<AbsolutePose>("pose", {}, {throttleMs: 1000, enabled: !compact}).data;
    const [track, setTrack] = useState<TrackPoint[]>([]);
    useEffect(() => {
        const x = trackPose.pose?.pose?.position?.x;
        const y = trackPose.pose?.pose?.position?.y;
        if (typeof x === "number" && typeof y === "number") setTrack((tr) => appendTrack(tr, x, y, Date.now()));
    }, [trackPose]);
    const previewDrawn = planPreview?.status === "ok" && planPreview.segments.length > 0
        && planPreview.id !== previewHiddenId ? planPreview : null;
    // "Hide" on the card hides the drawn preview too (the latched /coverage/full_plan
    // still holds it until the next preview / mission / Clear).
    const previewHidden = planPreview?.status === "ok" && planPreview.id === previewHiddenId;
    const dockCorridorCollection = useMemo<FeatureCollection>(() => {
        if (dockCorridorPts.length < 3 || datum[0] === 0) return {type: "FeatureCollection", features: []};
        const ring = dockCorridorPts.map((p) => transpose(offsetX, offsetY, datum, p.y, p.x));
        ring.push(ring[0]);
        return {type: "FeatureCollection", features: [{
            type: "Feature", id: "dock-corridor", properties: {feature_type: "dock-corridor"},
            geometry: {type: "Polygon", coordinates: [ring]},
        }]};
    }, [dockCorridorPts, offsetX, offsetY, datum]);
    const [corridorHover, setCorridorHover] = useState<{lng: number; lat: number} | null>(null);
    // Terrain memory (map server): incident clusters + per-area slope axis.
    const terrainSummary = useTerrainSummary(!compact);
    const [terrainSel, setTerrainSel] = useState<{area: number; id: number} | null>(null);
    const [terrainCardHidden, setTerrainCardHidden] = useState(false);
    const terrainCollection = useMemo<FeatureCollection>(() => {
        const features: Feature[] = [];
        if (!terrainSummary || datum[0] === 0) return {type: "FeatureCollection", features};
        const tp = (x: number, y: number) => transpose(offsetX, offsetY, datum, y, x);
        for (const a of terrainSummary.areas) {
            for (const c of a.clusters ?? []) {
                const selected = terrainSel?.area === a.area_index && terrainSel?.id === c.id;
                const color = TERRAIN_STATUS_COLORS[c.status] ?? "#ff9f1a";
                features.push({type: "Feature", properties: {kind: "cluster", area_index: a.area_index, cluster_id: c.id,
                    color, selected: selected ? 1 : 0, label: String(c.n)},
                    geometry: {type: "Point", coordinates: tp(c.x, c.y)}});
                if (c.hull && c.hull.length >= 3) {
                    const ring = c.hull.map(([hx, hy]) => tp(hx, hy));
                    ring.push(ring[0]);
                    features.push({type: "Feature", properties: {kind: "hull", color, selected: selected ? 1 : 0},
                        geometry: {type: "LineString", coordinates: ring}});
                }
            }
            if (a.slope && Number.isFinite(a.slope.axis_deg)) {
                const pos = (map?.working_area ?? []).findIndex((_, i) => mowingAreaIndex(map, i) === a.area_index);
                const pts = pos >= 0 ? map?.working_area?.[pos]?.area?.points ?? [] : [];
                if (pts.length < 3) continue;
                const cx = pts.reduce((sum, p) => sum + (p.x ?? 0), 0) / pts.length;
                const cy = pts.reduce((sum, p) => sum + (p.y ?? 0), 0) / pts.length;
                // axis_deg: map-frame (ENU, CCW from +x) downhill/uphill axis folded to [0, 180).
                const r = a.slope.axis_deg * Math.PI / 180, L = 3;
                const dx = Math.cos(r) * L, dy = Math.sin(r) * L;
                features.push({type: "Feature", properties: {kind: "slope", label: `${a.slope.slope_deg.toFixed(1)}°`},
                    geometry: {type: "LineString", coordinates: [tp(cx - dx, cy - dy), tp(cx + dx, cy + dy)]}});
            }
        }
        return {type: "FeatureCollection", features};
    }, [terrainSummary, terrainSel, map, offsetX, offsetY, datum]);
    const terrainHasData = !!terrainSummary && terrainSummary.areas.some((a) => (a.clusters?.length ?? 0) > 0 || a.slope);

    useEffect(() => {
        // Don't rebuild features from stream data while in edit mode —
        // path/plan becoming undefined when streams stop would wipe user edits.
        if (editMap) return;

        let newFeatures: Record<string, MowingFeature> = {}
        if (map) {
            const workingAreas = buildFeatures(map.working_area??[], "area", true)
            const navigationAreas = buildFeatures(map.navigation_areas??[], "navigation", false, (map as MapWithChannels).navigation_channels)
            newFeatures = {...workingAreas, ...navigationAreas}

            // dock_x/dock_y are optional on the wire. `map?.dock_y!` claimed
            // otherwise and pushed `undefined` into transpose(), painting the
            // dock at NaN; skip the marker instead when the pose is absent.
            if (map.dock_x !== undefined && map.dock_y !== undefined) {
                const dock_lonlat = transpose(offsetX, offsetY, datum, map.dock_y, map.dock_x)
                newFeatures["dock"] = new DockFeatureBase(dock_lonlat, map.dock_heading ?? 0);
            }
        }
        if (previewDrawn) {
            const toLonLat = (pts: [number, number][]) =>
                pts.map(([x, y]) => transpose(offsetX, offsetY, datum, y, x));
            previewDrawn.segments.forEach((seg, i) => {
                const f = new PathFeature(`preview-${seg.type}-${i}`, toLonLat(seg.points),
                    seg.type === "ring" ? LAYER_COLORS.previewRing : LAYER_COLORS.previewSwath, 2);
                newFeatures[f.id] = f;
            });
            previewDrawn.transits.forEach((tr, i) => {
                const f = new PathFeature(`preview-transit-${i}`, toLonLat(tr), LAYER_COLORS.previewTransit, 1);
                newFeatures[f.id] = f;
            });
        } else if (path?.poses && !previewHidden && !liveDrawn) {
            // Coverage plan: the full F2C route (headland rings + every swath)
            // for the current area (/coverage/full_plan, a nav_msgs/Path).
            // Execution is swath-by-swath, but this shows the whole plan.
            // Rendered green so it reads distinctly from the transit plan below.
            //
            // full_path is the CONCATENATION of the drivable sub-paths; the
            // jump between two sub-paths is never driven directly (the BT
            // bridges it with an obstacle-avoiding Nav2 transit), so break the
            // polyline at large gaps — drawing them as one line paints fake
            // straight "routes" through the very obstacles the sub-path split
            // exists to avoid.
            const SUBPATH_GAP_M = 0.75;
            let segment: Position[] = [];
            let segmentIdx = 0;
            let prev: { x: number; y: number } | null = null;
            const flushSegment = () => {
                if (segment.length > 1) {
                    const feature = new PathFeature(
                        `coverage-path-${segmentIdx}`, segment, LAYER_COLORS.coveragePath, 2);
                    newFeatures[feature.id] = feature
                    segmentIdx += 1;
                }
                segment = [];
            };
            for (const pose of path.poses) {
                const x = pose.pose?.position?.x;
                const y = pose.pose?.position?.y;
                // A pose without coordinates cannot be drawn — break the
                // polyline there rather than feeding NaN into transpose().
                if (x === undefined || y === undefined) {
                    flushSegment();
                    prev = null;
                    continue;
                }
                if (prev && Math.hypot(x - prev.x, y - prev.y) > SUBPATH_GAP_M) {
                    flushSegment();
                }
                segment.push(transpose(offsetX, offsetY, datum, y, x));
                prev = { x, y };
            }
            flushSegment();
        }
        if (plan?.poses) {
            const coordinates = plan.poses.flatMap((pose) => {
                const x = pose.pose?.position?.x;
                const y = pose.pose?.position?.y;
                if (x === undefined || y === undefined) return [];
                return [transpose(offsetX, offsetY, datum, y, x)];
            });
            const feature = new ActivePathFeature("plan", coordinates);
            newFeatures[feature.id] = feature
        }
        // Preserve the live robot features — they are owned by the pose stream
        // (useMapStreams merges mower/mower-* in) and must survive this
        // map/path/plan-driven rebuild. Replacing the record wholesale wiped
        // the mower on every path update, so the robot only stayed visible
        // while the path overlays were NOT being streamed.
        setFeatures((old) => ({
            ...newFeatures,
            ...Object.fromEntries(
                Object.entries(old).filter(([k]) => k === "mower" || k.startsWith("mower-"))
            ),
        }))
    }, [map, path, plan, previewDrawn, previewHidden, liveDrawn, offsetX, offsetY, datum, editMap, LAYER_COLORS]);

    const liveCollection = useMemo<FeatureCollection>(() => {
        if (datum[0] === 0) return {type: "FeatureCollection", features: []};
        const toLonLat = (pts: [number, number][]) => pts.map(([x, y]) => transpose(offsetX, offsetY, datum, y, x));
        const features: Feature[] = liveStretches.map((st, i) => ({
            type: "Feature", id: `live-${i}`, properties: {kind: st.kind},
            geometry: {type: "LineString", coordinates: toLonLat(st.points)},
        }));
        if (track.length > 1) {
            features.push({type: "Feature", id: "track", properties: {kind: "track"},
                geometry: {type: "LineString", coordinates: toLonLat(track.map((p) => [p.x, p.y]))}});
        }
        return {type: "FeatureCollection", features};
    }, [liveStretches, track, offsetX, offsetY, datum]);

    // Labels for navigation areas / paths, kept out of labelsCollection so
    // they never show up as mowable areas (mowingAreas is derived from it).
    const navLabelsCollection = useMemo<FeatureCollection>(() => {
        const navs = Object.values(features).filter((f): f is NavigationFeature => f instanceof NavigationFeature);
        return {
            type: "FeatureCollection",
            features: navs.flatMap((f, i) => {
                if (!f.geometry?.coordinates?.[0]?.length) return [];
                const c = turfCentroid(f);
                c.properties = {title: f.getName() || t('mapAreasList.navigationArea', {index: i + 1})};
                return [c];
            }),
        };
    }, [features, t]);

    useEffect(() => {
        const labels = buildLabels(Object.values(features))
        setLabelsCollection({
            type: "FeatureCollection",
            features: labels
        });
        setMowingAreas(labels.flatMap(feat => {
            if (feat.properties?.title == undefined) {
                return []
            }
            return [{
                key: feat.id as string,
                label: feat.properties.title,
                feat: feat
            }]
        }))
    }, [features]);

    // For each tracked obstacle (transient /obstacle_tracker/obstacles
    // observation), figure out which mowing area's polygon contains its
    // centroid. The result is the area_index map_server expects when we
    // promote the obstacle — the position of the matching area in
    // map_server's areas_ vector. Mowing order and the filtered working-area
    // position are not ROS IDs; resolve the original ID from map metadata.
    // Returns null when no workarea contains the centroid → promote button
    // is disabled because we'd have nowhere to attach it.
    const obstacleAreaIndex = useMemo(() => {
        const result: Record<number, number | null> = {};
        const workareas = Object.values(features)
            .filter((f): f is MowingAreaFeature => f instanceof MowingAreaFeature)
            .sort((a, b) => (a.getMowingOrder() ?? 9999) - (b.getMowingOrder() ?? 9999));
        for (const obs of dynamicObstacles) {
            const id = obs.id ?? 0;
            // Compute centroid of the obstacle polygon in ROS coords. The
            // tracker publishes points in ROS map frame (x/y metres).
            const pts = obs.polygon?.points ?? [];
            if (pts.length < 3) {
                result[id] = null;
                continue;
            }
            let cxRos = 0;
            let cyRos = 0;
            for (const p of pts) {
                cxRos += p.x ?? 0;
                cyRos += p.y ?? 0;
            }
            cxRos /= pts.length;
            cyRos /= pts.length;
            // Transpose to lng/lat so we can use the GeoJSON polygons.
            const [cLon, cLat] = transpose(offsetX, offsetY, datum, cyRos, cxRos);
            // Find the first workarea whose polygon contains the centroid.
            // Manual ray-casting (point-in-polygon) — avoids pulling in
            // turf-boolean-point-in-polygon for one call.
            let matchedIdx: number | null = null;
            for (let i = 0; i < workareas.length; ++i) {
                const ring = workareas[i].geometry.coordinates[0] ?? [];
                let inside = false;
                for (let j = 0, k = ring.length - 1; j < ring.length; k = j++) {
                    const xj = ring[j][0];
                    const yj = ring[j][1];
                    const xk = ring[k][0];
                    const yk = ring[k][1];
                    const intersect =
                        (yj > cLat) !== (yk > cLat) &&
                        cLon < ((xk - xj) * (cLat - yj)) / (yk - yj + 1e-12) + xj;
                    if (intersect) inside = !inside;
                }
                if (inside) {
                    matchedIdx = mowingAreaIndex(map, workareas[i].properties.source_working_area_index) ?? null;
                    break;
                }
            }
            result[id] = matchedIdx;
        }
        return result;
    }, [dynamicObstacles, features, offsetX, offsetY, datum, map]);

    const obstacleAreaNames = useMemo(() => {
        const names: Record<number, string> = {};
        const workareas = Object.values(features)
            .filter((f): f is MowingAreaFeature => f instanceof MowingAreaFeature)
            .sort((a, b) => (a.getMowingOrder() ?? 9999) - (b.getMowingOrder() ?? 9999));
        for (let i = 0; i < workareas.length; ++i) {
            const areaIndex = mowingAreaIndex(map, workareas[i].properties.source_working_area_index);
            if (areaIndex === undefined) continue;
            names[areaIndex] = workareas[i].getLabel(
                t('mapAreasList.unnamedArea', {order: workareas[i].getMowingOrder()})
            );
        }
        return names;
    }, [features, t, map]);

    // Build the areas list for the sidebar panel
    const areasList = useMemo(() => {
        const polygons = Object.values(features).filter(
            (f): f is MowingFeatureBase => f instanceof MowingFeatureBase
        );
        return polygons
            .sort((a, b) => {
                // workareas first, then navigation, then obstacles
                const typeOrder: Record<string, number> = { workarea: 0, navigation: 1, obstacle: 2 };
                const ta = typeOrder[a.properties.feature_type] ?? 3;
                const tb = typeOrder[b.properties.feature_type] ?? 3;
                if (ta !== tb) return ta - tb;
                return (a.properties.mowing_order ?? 0) - (b.properties.mowing_order ?? 0);
            })
            .map((f, i, arr) => {
                const areaSqm = turfArea(f);
                const areaLabel = areaSqm >= 10000
                    ? `${(areaSqm / 10000).toFixed(2)} ha`
                    : `${areaSqm.toFixed(0)} m²`;
                const ftype = f.properties.feature_type;
                let name = '';
                if (f instanceof MowingAreaFeature) {
                    name = f.getLabel(t('mapAreasList.unnamedArea', {order: f.getMowingOrder()}));
                } else if (f instanceof NavigationFeature) {
                    // Short 1-based ordinal within its own type, not the raw id.
                    const navIdx = arr.slice(0, i).filter(x => x instanceof NavigationFeature).length + 1;
                    name = f.getName() || t('mapAreasList.navigationArea', {index: navIdx});
                } else if (f instanceof ObstacleFeature) {
                    const obsIdx = arr.slice(0, i).filter(x => x instanceof ObstacleFeature).length + 1;
                    name = t('mapAreasList.obstacleArea', {index: obsIdx});
                }
                const mowingOrder = f instanceof MowingAreaFeature ? f.getMowingOrder() : undefined;
                return { id: f.id, name, ftype, areaLabel, mowingOrder };
            });
    }, [features, t]);

    const handleReorder = useCallback((id: string, direction: 'up' | 'down') => {
        setFeatures((curr) => {
            const next = {...curr};
            const target = next[id];
            if (!(target instanceof MowingAreaFeature)) return curr;
            const targetOrder = target.getMowingOrder();
            const swapOrder = direction === 'up' ? targetOrder - 1 : targetOrder + 1;
            const swapFeat = Object.values(next).find(
                (f): f is MowingAreaFeature =>
                    f instanceof MowingAreaFeature && f.getMowingOrder() === swapOrder
            );
            if (!swapFeat) return curr;
            target.setMowingOrder(swapOrder);
            swapFeat.setMowingOrder(targetOrder);
            return next;
        });
    }, []);

    function buildFeatures(areas: MapArea[], type: string, fromLiveMap = false,
                           channels?: (AreaChannel | null)[]) : Record<string, MowingFeatureBase> {


        return areas?.flatMap((area, index) : MowingFeatureBase[] => {
            if (!area.area?.points?.length) {
                return []
            }

            const nfeat = type=="area" ? new MowingAreaFeature(type + "-" + index.toString() + "-area-0", index+1)
                : new NavigationFeature(type + "-" + index.toString() + "-area-0");//, offsetX, offsetY, datum.
            nfeat.setArea(area, offsetX, offsetY, datum);
            const ch = channels?.[index];
            if (nfeat instanceof NavigationFeature && ch && ch.points?.length >= 2) {
                nfeat.setChannel(ch.points.map(([x, y]) => transpose(offsetX, offsetY, datum, y, x)), ch.width_m);
            }
            // Preserve source identity separately from editable mowing order.
            // Restored/imported maps cannot target live ROS areas until saved.
            if (fromLiveMap) nfeat.properties.source_working_area_index = index;

            let obstacles:  ObstacleFeature[] = [];

            if ((nfeat instanceof MowingAreaFeature) && (area.obstacles))
                obstacles = area.obstacles.map((obstacle, oindex) => {
                const nobst =  new ObstacleFeature(
                    type + "-" + index.toString() + "-obstacle-" + oindex.toString(),
                    nfeat
                );
                
                if (obstacle.points)
                    nobst.transpose(obstacle.points, offsetX, offsetY, datum);

                return nobst;

            })
            return [nfeat, ...obstacles ]
        }).reduce((acc, val) :Record<string, MowingFeatureBase> => {
            if (val.id == undefined) {
                return acc
            }
            acc[val.id] = val;
            return acc;
        }, {} as Record<string, MowingFeatureBase>);
    }

    // Build the full editable feature set (areas, obstacles and dock) from a
    // Map message. The stream-driven effect above does the same thing but is
    // skipped while editMap is true, so map restore (which enters edit mode)
    // calls this directly to populate the features it will save.
    function buildFeaturesFromMap(m: MapType): Record<string, MowingFeature> {
        const newFeatures: Record<string, MowingFeature> = {
            ...buildFeatures(m.working_area ?? [], "area"),
            ...buildFeatures(m.navigation_areas ?? [], "navigation", false, (m as MapWithChannels).navigation_channels),
        };
        const dockLonLat = transpose(offsetX, offsetY, datum, m.dock_y ?? 0, m.dock_x ?? 0);
        newFeatures["dock"] = new DockFeatureBase(dockLonLat, m.dock_heading ?? 0);
        return newFeatures;
    }

    const {
        handleSaveMap,
        handleApplyDockPose,
        handleBackupMap,
        handleRestoreMap,
        handleDownloadGeoJSON,
        handleUploadGeoJSON,
        handleImportOpenMower,
        handleReprojectOpenMowerPreview,
        handleApplyOpenMowerImport,
    } = useMapFiles({
        features,
        setFeatures,
        map,
        setMap,
        editMap,
        setEditMap,
        setHasUnsavedChanges,
        offsetX,
        offsetY,
        datum,
        notification,
        guiApi,
        dockDirty,
        setDockDirty,
        buildFeaturesFromMap,
    });


    const bladeTwoStep = hasFeature(robotProfile, "manual_blade_two_step");
    const {
        manualMode, handleManualMode, handleStopManualMode, handleJoyMove, handleJoyStop,
        bladeOn, canStartBlade, handleBladeStart, handleBladeStop,
    } = useManualMode({
        mowerAction, joyStream,
        stateName: highLevelStatus.highLevelStatus.state_name,
        subStateName: highLevelStatus.highLevelStatus.sub_state_name,
        bladeTwoStep,
    });
    // Manual-drive camera PiP (camera robots only): direction drives the
    // rear-camera auto switch; drive view swaps camera and map.
    const hasDriveCamera = hasFeature(robotProfile, "cameras");
    const {prefs: drivePip, update: updateDrivePip} = useDrivePipPrefs();
    const [reversing, setReversing] = useState(false);
    const onJoyMove = useCallback((event: Parameters<typeof handleJoyMove>[0]) => {
        setReversing(isReversing(event.y));
        handleJoyMove(event);
    }, [handleJoyMove]);
    const onJoyStop = useCallback(() => {
        setReversing(false);
        handleJoyStop();
    }, [handleJoyStop]);
    const driveView = hasDriveCamera && manualMode && drivePip.driveView && !drivePip.collapsed;


    // Toggle dock placement mode: re-pressing the button (or pressing Escape)
    // cancels it, so the crosshair cursor is not a one-way trap.
    const handleDockPlacement = useCallback(() => {
        setDockPlacementMode(prev => !prev);
    }, []);

    // Escape cancels an armed dock placement.
    useEffect(() => {
        if (!dockPlacementMode) return;
        const onKeyDown = (e: KeyboardEvent) => {
            if (e.key === "Escape") setDockPlacementMode(false);
        };
        window.addEventListener("keydown", onKeyDown);
        return () => window.removeEventListener("keydown", onKeyDown);
    }, [dockPlacementMode]);


    // Area settings (robots whose map server implements them): clicking a
    // mowing area in view mode — on the map or in the sidebar list — opens its
    // "Mow settings" panel, and Start opens the Start sheet.
    const areaSettings = useAreaSettingsSupport();
    const [settingsArea, setSettingsArea] = useState<{index: number | "defaults"; name?: string; areaName?: string} | null>(null);
    // Navigation area / path selected in view mode: the settings panel opens
    // disabled with a hint, since drive-only areas are never mowed.
    const [settingsNavName, setSettingsNavName] = useState<string | null>(null);
    const [startSheet, setStartSheet] = useState<{open: boolean; selection: StartSelection}>({open: false, selection: "all"});
    const areaChoices = useMemo(
        () => mowingAreaChoices(map, (order) => t('mapAreasList.unnamedArea', {order})),
        [map, t],
    );
    const openAreaSettingsForFeature = useCallback((f: MowingFeature | undefined) => {
        if (f instanceof NavigationFeature) {
            const navIdx = Object.values(features).filter((x) => x instanceof NavigationFeature).indexOf(f) + 1;
            setSettingsNavName(f.getName() || t('mapAreasList.navigationArea', {index: navIdx}));
            return true;
        }
        if (!(f instanceof MowingAreaFeature)) return false;
        const index = mowingAreaIndex(map, f.properties.source_working_area_index);
        if (index === undefined) return false;
        setSettingsArea({
            index,
            name: f.getLabel(t('mapAreasList.unnamedArea', {order: f.getMowingOrder()})),
            areaName: map?.working_area?.[f.properties.source_working_area_index ?? -1]?.name,
        });
        return true;
    }, [map, t, features]);
    const openAreaSettingsById = useCallback(
        (id: string) => { openAreaSettingsForFeature(features[id]); },
        [features, openAreaSettingsForFeature],
    );

    const handleMapClick = useCallback((e: {lngLat: {lng: number; lat: number}; features?: {properties?: Record<string, unknown> | null}[]}) => {
        const tc = e.features?.find((f) => f.properties?.kind === "cluster");
        if (!dockPlacementMode && !editMap && tc) {
            setTerrainSel({area: Number(tc.properties?.area_index), id: Number(tc.properties?.cluster_id)});
            setTerrainCardHidden(false);
            return;
        }
        if (!dockPlacementMode) {
            if (editMap || !areaSettings.enabled) return;
            const pt: [number, number] = [e.lngLat.lng, e.lngLat.lat];
            const hit = Object.values(features).find((f) =>
                f instanceof MowingAreaFeature && pointInPolygon(pt, f.geometry.coordinates))
                ?? Object.values(features).find((f) =>
                    f instanceof NavigationFeature && pointInPolygon(pt, f.geometry.coordinates));
            openAreaSettingsForFeature(hit);
            return;
        }
        setDockPlacementMode(false);
        const coord: [number, number] = [e.lngLat.lng, e.lngLat.lat];
        setFeatures(prev => {
            const existingDock = prev["dock"];
            const heading = existingDock instanceof DockFeatureBase ? existingDock.getHeading() : 0;
            return {...prev, dock: new DockFeatureBase(coord, heading)};
        });
        setHasUnsavedChanges(true);
        setDockDirty(true);
    }, [dockPlacementMode, setHasUnsavedChanges, editMap, areaSettings.enabled, features, openAreaSettingsForFeature]);

    // Dock heading panel: rotate the dock marker in place (position kept).
    const handleDockHeadingChange = useCallback((heading: number) => {
        setFeatures(prev => {
            const dock = prev["dock"];
            if (!(dock instanceof DockFeatureBase)) return prev;
            return {...prev, dock: new DockFeatureBase(dock.getCoordinates(), heading)};
        });
        setHasUnsavedChanges(true);
        setDockDirty(true);
    }, [setHasUnsavedChanges]);

    // Map → panel side of the two-way obstacle highlight: while the cursor is
    // over a tracked-obstacle polygon, mirror its id into selectedObstacleId so
    // the matching panel row lights up. e.features only carries the interactive
    // layers (DYN_OBSTACLE_INTERACTIVE_LAYERS); when the cursor leaves every
    // obstacle it is empty → clears the highlight. Hovering the overlay panel
    // does not reach the map canvas, so panel-driven highlights are never
    // clobbered here.
    const handleMapMouseMove = useCallback((e: {features?: Array<{properties?: Record<string, unknown> | null}>; lngLat?: {lng: number; lat: number}}) => {
        const hit = e.features?.find(f => f.properties?.feature_type === 'dyn-obstacle');
        const id = hit ? (hit.properties?.obs_id as number) : null;
        setSelectedObstacleId(prev => (prev === id ? prev : id));
        const corridor = e.features?.some(f => f.properties?.feature_type === 'dock-corridor');
        setCorridorHover(corridor && e.lngLat ? {lng: e.lngLat.lng, lat: e.lngLat.lat} : null);
    }, []);

    // Belt-and-suspenders: any time dockDirty flips to true, ensure
    // hasUnsavedChanges is also true so the Save Map button glows. The
    // inline setHasUnsavedChanges(true) above + the useMapEditHistory
    // features-watcher should already cover this, but a stale closure
    // or a state-batching race used to leave the dock-only case where
    // the user pinned a new dock pose but the save toolbar stayed
    // calm. This effect makes the dirty signal sticky.
    useEffect(() => {
        if (dockDirty) setHasUnsavedChanges(true);
    }, [dockDirty, setHasUnsavedChanges]);

    // Mower action callbacks shared between desktop and mobile toolbars
    const useStartSheet = areaSettings.enabled && areaSettings.supported === true;
    const startSelectedArea = (key: string) => {
        const item = mowingAreas.find(item => item.key == key);
        const index = mowingAreaIndex(map, item?.feat?.properties?.index);
        if (index === undefined) return Promise.reject(new Error(t("crossHatch.areaUnavailable")));
        if (useStartSheet) {
            setStartSheet({open: true, selection: index});
            return Promise.resolve();
        }
        return mowerAction("start_in_area", {area: index})();
    };

    const previewSelectedPlan = async (key: string) => {
        let area = -1;
        if (key !== PREVIEW_ALL_KEY) {
            const item = mowingAreas.find(item => item.key == key);
            const index = mowingAreaIndex(map, item?.feat?.properties?.index);
            if (index === undefined) throw new Error(t("crossHatch.areaUnavailable"));
            area = index;
        }
        setPreviewHiddenId(null);
        await requestPlanPreview(guiApi, area);
    };
    const progressColors = {mowed: LAYER_COLORS.liveMowed, current: LAYER_COLORS.liveCurrent,
        remaining: LAYER_COLORS.liveRemaining, skipped: LAYER_COLORS.liveSkipped, track: LAYER_COLORS.track};
    const liveAreaName = liveProgress
        ? (areaChoices.find((a) => a.index === liveProgress.area)?.name ?? undefined) : undefined;
    const showPreviewCard = !compact && !editMap && !liveProgress && planPreview !== null
        && planPreview.status !== "cleared" && planPreview.id !== previewHiddenId;
    const previewCard = showPreviewCard && planPreview ? (
        <div style={{position: 'absolute', zIndex: 15, ...(isMobile ? {top: 64, left: 12} : {bottom: 84, left: 16}), maxWidth: 300, background: colors.glassBackground, border: colors.glassBorder, boxShadow: colors.glassShadow, borderRadius: 14, padding: '8px 12px'}}>
            <PlanPreviewCard
                summary={planPreview}
                colors={{ring: LAYER_COLORS.previewRing, swath: LAYER_COLORS.previewSwath, transit: LAYER_COLORS.previewTransit}}
                onClear={() => clearPlanPreview(guiApi)}
                onDismiss={() => setPreviewHiddenId(planPreview.id)}
            />
        </div>
    ) : null;

    const mowerActions = useMemo(() => ({
        onStart: useStartSheet
            ? () => { setStartSheet({open: true, selection: "all"}); return Promise.resolve(); }
            : mowerAction("high_level_control", {Command: 1}),
        onHome: mowerAction("high_level_control", {Command: 2}),
        // Go provider "undock" route -> high_level_control Command 9 (mission_fsm CMD_UNDOCK).
        onUndock: mowerAction("undock"),
        onEmergencyOn: mowerAction("emergency", {Emergency: 1}),
        onEmergencyOff: mowerAction("emergency", {Emergency: 0}),
        onAreaRecording: mowerAction("high_level_control", {Command: 3}),
        onPathRecording: mowerAction("high_level_control", {Command: CMD_RECORD_PATH}),
        onMowNextArea: mowerAction("high_level_control", {Command: 4}),
        // Match MapToolbar's isIdle: the BT publishes IDLE_DOCKED as the
        // primary resting state; "IDLE" without a suffix only appears as the
        // manual-mow fallthrough. There is no "pause flag" in the stack (the
        // old mower_logic/manual_pause_mowing OpenMower command does not exist
        // here and returned HTTP 500, which also broke Continue-from-idle by
        // rejecting before the START fired). Use real HighLevelControl commands:
        // Continue = START (mow_progress persists, so it resumes where it left
        // off); Pause = STOP (COMMAND_STOP=8 → StopHoldSequence: mower off, halt
        // in place, Nav2 left up so the mission can resume, no dock drive).
        onContinueOrPause:
            highLevelStatus.highLevelStatus.state_name === "IDLE_DOCKED" ||
            highLevelStatus.highLevelStatus.state_name === "IDLE"
                ? mowerAction("high_level_control", {Command: 1})
                : mowerAction("high_level_control", {Command: 8}),
        onBladeForward: mowerAction("mow_enabled", {mow_enabled: 1, mow_direction: 0}),
        onBladeBackward: mowerAction("mow_enabled", {mow_enabled: 1, mow_direction: 1}),
        onBladeOff: mowerAction("mow_enabled", {mow_enabled: 0, mow_direction: 0}),
        onRecordFinish: mowerAction("high_level_control", {Command: 5}),
        onRecordCancel: mowerAction("high_level_control", {Command: 6}),
    }), [mowerAction, highLevelStatus.highLevelStatus.state_name, useStartSheet]);

    // Centered message panel used for the missing-token and missing-datum
    // states — a plain, translated explanation instead of an eternal spinner
    // or a broken map.
    // Set the datum from the current GPS fix (same set_datum flow as Settings ->
    // Positioning), persist it to the robot config, then reload so the map
    // picks up the new origin.
    const setDatumFromGps = async () => {
        setDatumBusy(true);
        try {
            const {lat, lon} = await requestDatumFromGps(guiApi);
            const saved = await guiApi.settings.yamlCreate({datum_lat: lat, datum_lon: lon});
            if (saved.error) throw new Error((saved.error as any).error);
            notification.success({message: t('datum.setFromGpsSuccess', {lat: lat.toFixed(7), lon: lon.toFixed(7)})});
            window.location.reload();
        } catch (e: unknown) {
            notification.error({
                message: t('settingsPositioning.datumGpsFailed'),
                description: datumErrorDetail(e, (raw) => t('datum.unparseableReply', {message: raw || '-'})),
            });
            setDatumBusy(false);
        }
    };

    const CenteredMessage: React.FC<{title: string; detail?: string; children?: React.ReactNode}> = ({title, detail, children}) => (
        <div style={{
            width: '100%',
            height: '100%',
            minHeight: compact ? undefined : 240,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            gap: 8,
            textAlign: 'center',
            padding: 24,
            color: colors.textSecondary,
            background: colors.bgCard,
            borderRadius: 12,
        }}>
            <div style={{fontSize: compact ? 14 : 16, fontWeight: 600, color: colors.text}}>{title}</div>
            {detail && <div style={{fontSize: compact ? 12 : 14}}>{detail}</div>}
            {children}
        </div>
    );

    if (!MAPBOX_TOKEN) {
        return <CenteredMessage
            title={t('mapPage.mapboxTokenMissingTitle')}
            detail={t('mapPage.mapboxTokenMissingDetail')}
        />;
    }
    if (_datumLon == 0 || _datumLat == 0) {
        return <CenteredMessage
            title={t('mapPage.noDatumTitle')}
            detail={t('mapPage.noDatumDetail')}
        >
            <div style={{display: 'flex', gap: 8, flexWrap: 'wrap', justifyContent: 'center', marginTop: 8}}>
                <Button type="primary" size={compact ? 'small' : 'middle'} loading={datumBusy}
                        onClick={setDatumFromGps}>{t('datum.setFromGps')}</Button>
                <Button size={compact ? 'small' : 'middle'}
                        onClick={() => navigate('/settings?section=positioning')}>{t('datum.openPositioning')}</Button>
            </div>
        </CenteredMessage>;
    }
    if (compact) {
        return (
            <div style={{width: '100%', height: '100%', position: 'relative'}}>
                {map_sw?.length && map_ne?.length ? <Map key={mapKey}
                                                         reuseMaps
                                                         antialias
                                                         projection={{
                                                             name: "globe"
                                                         }}
                                                         mapboxAccessToken={MAPBOX_TOKEN}
                                                         initialViewState={{
                                                             bounds: [{lng: map_sw[0], lat: map_sw[1]}, {lng: map_ne[0], lat: map_ne[1]}],
                                                             bearing,
                                                         }}
                                                         style={{width: '100%', height: '100%'}}
                                                         mapStyle={useSatellite ? "mapbox://styles/mapbox/satellite-streets-v12" : "mapbox://styles/mapbox/dark-v11"}
                                                         interactive={false}
                                                         onLoad={onMapLoad}
                                                         attributionControl={false}
                >
                    {tileUri ? <Source type={"raster"} id={"custom-raster"} tiles={[tileUri]} tileSize={256}/> : null}
                    {tileUri ? <Layer type={"raster"} source={"custom-raster"} id={"custom-layer"}/> : null}
                    <Source type={"geojson"} id={"labels"} data={labelsCollection}/>
                    <Layer type={"symbol"} id={"mower"} source={"labels"} layout={{
                        "text-field": ['get', 'title'],
                        "text-rotation-alignment": "auto",
                        "text-allow-overlap": true,
                        "text-anchor": "top"
                    }} paint={{
                        "text-color": LAYER_COLORS.labelText,
                        "text-halo-color": LAYER_COLORS.labelHalo,
                        "text-halo-width": 1.5,
                    }}/>
                    <DrawControl
                        drawRef={drawRef}
                        styles={MapStyle}
                        userProperties={true}
                        features={drawableFeatures}
                        position="top-left"
                        displayControlsDefault={false}
                        editMode={false}
                        controls={{}}
                        defaultMode="simple_select"
                        onCreate={() => {}}
                        onUpdate={() => {}}
                        onCombine={() => {}}
                        onDelete={() => {}}
                        onSelectionChange={() => {}}
                        onOpenDetails={() => {}}
                    />
                    <Source type={"geojson"} id={"display-features"} data={displayFeatures}>
                        <Layer type={"line"} id={"display-lines"} filter={['==', ['geometry-type'], 'LineString']}
                            layout={{'line-cap': 'round', 'line-join': 'round'}}
                            paint={{
                                'line-color': ['get', 'color'],
                                'line-width': ['get', 'width'],
                            }}/>
                        {/* Dock marker */}
                        <Layer type={"circle"} id={"dock-halo"} layout={{visibility: dockMarker ? "none" : "visible"}}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            paint={{
                                'circle-radius': 12,
                                'circle-color': LAYER_COLORS.halo,
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"dock-point"} layout={{visibility: dockMarker ? "none" : "visible"}}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            paint={{
                                'circle-radius': 9,
                                'circle-color': LAYER_COLORS.dock,
                                'circle-stroke-color': LAYER_COLORS.halo,
                                'circle-stroke-width': 2,
                            }}/>
                        <Layer type={"symbol"} id={"dock-label"}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            layout={{
                                'text-field': 'DOCK',
                                'text-size': 10,
                                'text-font': ['Open Sans Bold'],
                                'text-offset': [0, 1.8],
                                'text-anchor': 'top',
                            }}
                            paint={{
                                'text-color': LAYER_COLORS.dock,
                                'text-halo-color': LAYER_COLORS.halo,
                                'text-halo-width': 1.5,
                            }}/>
                        {/* Mower footprint (robot shape from URDF) */}
                        <Layer type={"fill"} id={"mower-footprint-fill"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'fill-color': ['get', 'color'],
                                'fill-opacity': 0.55,
                            }}/>
                        <Layer type={"line"} id={"mower-footprint-outline"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'line-color': LAYER_COLORS.mowerOutline,
                                'line-width': 2,
                            }}/>
                        {/* Mower center point */}
                        <Layer type={"circle"} id={"mower-point"}
                            filter={['==', ['get', 'feature_type'], 'mower']}
                            paint={{
                                'circle-radius': 4,
                                'circle-color': LAYER_COLORS.mower,
                                'circle-stroke-color': LAYER_COLORS.halo,
                                'circle-stroke-width': 1.5,
                            }}/>
                        {/* Other display points (Point geometry only — exclude polygon/line vertices) */}
                        <Layer type={"circle"} id={"display-points-halo"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 8,
                                'circle-color': LAYER_COLORS.halo,
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"display-points"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 5,
                                'circle-color': ['get', 'color'],
                            }}/>
                        {/* Persistent tracked-obstacle polygons + id labels (compact overview: no highlight) */}
                        {renderDynObstacleLayers(false)}
                    </Source>
                    {/* Profile image markers (dock, robot) on top of every other layer. */}
                    <MapImageMarker key={`dock-img-${!!mowProgressImage}-${!!lidarMapImage}`} id={"dock-image"}
                        src={dockMarker?.src ?? ""} corners={dockMarker ? dockMarkerCorners : null}/>
                    <MapImageMarker key={`robot-img-${!!mowProgressImage}-${!!lidarMapImage}`} id={"robot-image"}
                        src={robotMarker?.src ?? ""} corners={robotMarkerCorners}/>
                </Map> : <Spinner/>}
            </div>
        );
    }

    return (
        <div style={{
            // Full-bleed the map across the AppShell's main padding.
            // Desktop main padding is 24px top / 32px horizontal / 48px bottom.
            // Mobile main padding includes the bottom safe area as well.
            position: 'relative',
            height: isMobile ? 'calc(100% + 122px + env(safe-area-inset-bottom, 0px))' : 'calc(100% + 72px)',
            width:  isMobile ? 'calc(100% + 28px)'  : 'calc(100% + 64px)',
            margin: isMobile ? '-12px -14px calc(-110px - env(safe-area-inset-bottom, 0px))' : '-24px -32px -48px',
        }}>
            <NewAreaModal
                open={modalOpen}
                areaType={newAreaType}
                areaName={newAreaName}
                onAreaTypeChange={setNewAreaType}
                onAreaNameChange={setNewAreaName}
                onSave={handleSaveNewArea}
                onCancel={deleteFeature}
            />
            <PathModal
                open={pathTool.editing}
                editing={pathTool.editingExisting}
                name={pathTool.name}
                width={pathTool.width}
                start={pathTool.start}
                end={pathTool.end}
                extendToDock={pathTool.extendToDock}
                invalid={pathTool.invalid}
                onNameChange={pathTool.setName}
                onWidthChange={pathTool.setWidth}
                onExtendToDockChange={pathTool.setExtendToDock}
                onSave={pathTool.save}
                onCancel={pathTool.cancel}
                onDelete={pathTool.remove}
                mobile={isMobile}
            />
            <EditAreaModal
                open={areaModelOpen}
                area={curMowingAreaFeature}
                onChange={setCurMowingAreaFeature}
                onSave={updateMowingArea}
                onCancel={cancelAreaModal}
            />

            <div style={{height: '100%', position: 'relative'}}>
                {map_sw?.length && map_ne?.length ? <Map key={mapKey}
                                                         reuseMaps
                                                         antialias
                                                         projection={{
                                                             name: "globe"
                                                         }}
                                                         mapboxAccessToken={MAPBOX_TOKEN}
                                                         initialViewState={{
                                                             bounds: [{lng: map_sw[0], lat: map_sw[1]}, {lng: map_ne[0], lat: map_ne[1]}],
                                                             bearing,
                                                         }}
                                                         style={driveView ? DRIVE_VIEW_MAP_INSET : {width: '100%', height: '100%'}}
                                                         mapStyle={useSatellite ? "mapbox://styles/mapbox/satellite-streets-v12" : "mapbox://styles/mapbox/dark-v11"}
                                                         onLoad={onMapLoad}
                                                         onClick={handleMapClick}
                                                         interactiveLayerIds={DYN_OBSTACLE_INTERACTIVE_LAYERS}
                                                         onMouseMove={handleMapMouseMove}
                                                         cursor={dockPlacementMode ? 'crosshair' : undefined}
                >
                    {tileUri ? <Source type={"raster"} id={"custom-raster"} tiles={[tileUri]} tileSize={256}/> : null}
                    {tileUri ? <Layer type={"raster"} source={"custom-raster"} id={"custom-layer"}/> : null}
                    <Source type={"geojson"} id={"labels"} data={labelsCollection}/>
                    <Layer type={"symbol"} id={"mower"} source={"labels"} layout={{
                        "text-field": ['get', 'title'],
                        "text-rotation-alignment": "auto",
                        "text-allow-overlap": true,
                        "text-anchor": "top"
                    }} paint={{
                        "text-color": LAYER_COLORS.labelText,
                        "text-halo-color": LAYER_COLORS.labelHalo,
                        "text-halo-width": 1.5,
                    }}/>
                    <DrawControl
                        drawRef={drawRef}
                        styles={MapStyle}
                        userProperties={true}
                        features={drawableFeatures}
                        position="top-left"
                        displayControlsDefault={false}
                        editMode={editMap}
                        controls={{}}
                        defaultMode="simple_select"
                        onCreate={onCreateWithPath}
                        onUpdate={onUpdate}
                        onCombine={onCombine}
                        onDelete={onDelete}
                        onSelectionChange={onSelectionChange}
                        onOpenDetails={onOpenDetails}
                    />
                    {/* Display-only features: mower, dock, heading, paths */}
                    <Source type={"geojson"} id={"display-features"} data={displayFeatures}>
                        <Layer type={"line"} id={"display-lines"} filter={['==', ['geometry-type'], 'LineString']}
                            layout={{'line-cap': 'round', 'line-join': 'round'}}
                            paint={{
                                'line-color': ['get', 'color'],
                                'line-width': ['get', 'width'],
                            }}/>
                        {/* Dock marker */}
                        <Layer type={"circle"} id={"dock-halo"} layout={{visibility: dockMarker ? "none" : "visible"}}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            paint={{
                                'circle-radius': 12,
                                'circle-color': LAYER_COLORS.halo,
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"dock-point"} layout={{visibility: dockMarker ? "none" : "visible"}}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            paint={{
                                'circle-radius': 9,
                                'circle-color': LAYER_COLORS.dock,
                                'circle-stroke-color': LAYER_COLORS.halo,
                                'circle-stroke-width': 2,
                            }}/>
                        <Layer type={"symbol"} id={"dock-label"}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            layout={{
                                'text-field': 'DOCK',
                                'text-size': 10,
                                'text-font': ['Open Sans Bold'],
                                'text-offset': [0, 1.8],
                                'text-anchor': 'top',
                            }}
                            paint={{
                                'text-color': LAYER_COLORS.dock,
                                'text-halo-color': LAYER_COLORS.halo,
                                'text-halo-width': 1.5,
                            }}/>
                        {/* Mower footprint (robot shape from URDF) */}
                        <Layer type={"fill"} id={"mower-footprint-fill"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'fill-color': ['get', 'color'],
                                'fill-opacity': 0.55,
                            }}/>
                        <Layer type={"line"} id={"mower-footprint-outline"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'line-color': LAYER_COLORS.mowerOutline,
                                'line-width': 2,
                            }}/>
                        {/* Mower center point */}
                        <Layer type={"circle"} id={"mower-point"}
                            filter={['==', ['get', 'feature_type'], 'mower']}
                            paint={{
                                'circle-radius': 4,
                                'circle-color': LAYER_COLORS.mower,
                                'circle-stroke-color': LAYER_COLORS.halo,
                                'circle-stroke-width': 1.5,
                            }}/>
                        {/* Other display points (Point geometry only — exclude polygon/line vertices) */}
                        <Layer type={"circle"} id={"display-points-halo"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 8,
                                'circle-color': LAYER_COLORS.halo,
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"display-points"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 5,
                                'circle-color': ['get', 'color'],
                            }}/>
                        {/* Persistent tracked-obstacle polygons + id labels + hover/select highlight */}
                        {renderDynObstacleLayers(true)}
                    </Source>
                    {/* Navigation area / path names (dimmer than mowing-area labels). */}
                    <Source type={"geojson"} id={"nav-labels"} data={navLabelsCollection}>
                        <Layer type={"symbol"} id={"nav-label"} layout={{
                            "text-field": ['get', 'title'],
                            "text-size": 11,
                            "text-allow-overlap": false,
                        }} paint={{
                            "text-color": LAYER_COLORS.labelText,
                            "text-opacity": 0.75,
                            "text-halo-color": LAYER_COLORS.labelHalo,
                            "text-halo-width": 1.5,
                        }}/>
                    </Source>
                    {/* Automatic dock corridor from the map server: faint hatch + thin outline. */}
                    <CorridorHatchPattern color={LAYER_COLORS.labelText}/>
                    <Source type={"geojson"} id={"dock-corridor"} data={dockCorridorCollection}>
                        <Layer type={"fill"} id={"dock-corridor-fill"} paint={{
                            "fill-pattern": CORRIDOR_HATCH_IMAGE,
                            "fill-opacity": 0.5,
                        }}/>
                        <Layer type={"line"} id={"dock-corridor-outline"} paint={{
                            "line-color": LAYER_COLORS.labelText,
                            "line-opacity": 0.35,
                            "line-width": 1,
                        }}/>
                    </Source>
                    {corridorHover && (
                        <Popup longitude={corridorHover.lng} latitude={corridorHover.lat}
                               closeButton={false} closeOnClick={false} anchor="bottom" offset={8}>
                            {pathsCollection.features.length > 0
                                ? t('mapPath.autoDockCorridorFallback')
                                : t('mapPath.autoDockCorridor')}
                        </Popup>
                    )}
                    {/* Saved paths (navigation areas drawn as a line): band + centreline. */}
                    <Source type={"geojson"} id={"saved-paths"} data={pathsCollection}>
                        <Layer type={"fill"} id={"saved-path-band"}
                            filter={['==', ['get', 'kind'], 'band']}
                            paint={{"fill-color": LAYER_COLORS.dockHeading, "fill-opacity": 0.18}}/>
                        <Layer type={"line"} id={"saved-path-centerline"}
                            filter={['==', ['get', 'kind'], 'centerline']}
                            paint={{"line-color": LAYER_COLORS.dockHeading, "line-width": 2, "line-opacity": 0.9, "line-dasharray": [3, 2]}}/>
                    </Source>
                    {/* Path tool live preview: buffered corridor + centreline. */}
                    <Source type={"geojson"} id={"path-preview"} data={pathTool.previewCollection}>
                        <Layer type={"fill"} id={"path-preview-fill"}
                            filter={['==', ['get', 'kind'], 'corridor']}
                            paint={{"fill-color": LAYER_COLORS.dockHeading, "fill-opacity": 0.25}}/>
                        <Layer type={"line"} id={"path-preview-outline"}
                            filter={['==', ['get', 'kind'], 'corridor']}
                            paint={{"line-color": LAYER_COLORS.dockHeading, "line-width": 2, "line-dasharray": [2, 2]}}/>
                        <Layer type={"line"} id={"path-preview-centerline"}
                            filter={['==', ['get', 'kind'], 'centerline']}
                            paint={{"line-color": LAYER_COLORS.dockHeading, "line-width": 1, "line-opacity": 0.8}}/>
                    </Source>
                    {/* fusion_graph's LiDAR anchor map (walls as ink, scanned ground as a faint wash). */}
                    {lidarMapImage && (
                        <Source type={"image"} id={"lidar-map"} url={lidarMapImage.url} coordinates={lidarMapImage.coordinates}>
                            <Layer type={"raster"} id={"lidar-map-layer"} paint={{
                                "raster-opacity": 0.85,
                                "raster-fade-duration": 0,
                                "raster-resampling": "nearest",
                            }}/>
                        </Source>
                    )}
                    {/* Terrain memory: traction score per cell (yellow -> red), just below mow progress. */}
                    {terrainImage && (
                        <Source type={"image"} id={"terrain-grid"} url={terrainImage.url} coordinates={terrainImage.coordinates}>
                            <Layer type={"raster"} id={"terrain-grid-layer"} paint={{
                                "raster-opacity": 0.75,
                                "raster-fade-duration": 0,
                                "raster-resampling": "nearest",
                            }}/>
                        </Source>
                    )}
                    {mowProgressImage && (
                        <Source type={"image"} id={"mow-progress"} url={mowProgressImage.url} coordinates={mowProgressImage.coordinates}>
                            <Layer type={"raster"} id={"mow-progress-layer"} paint={{
                                "raster-opacity": 0.7,
                                "raster-fade-duration": 0,
                            }}/>
                        </Source>
                    )}
                    {/* Terrain memory: incident cluster hulls/markers + per-area slope axis. */}
                    <Source type={"geojson"} id={"terrain-features"} data={terrainCollection}>
                        <Layer type={"line"} id={"terrain-cluster-hull"} filter={['==', ['get', 'kind'], 'hull']}
                            paint={{'line-color': ['get', 'color'], 'line-width': ['case', ['==', ['get', 'selected'], 1], 3, 1.5], 'line-dasharray': [2, 2]}}/>
                        <Layer type={"line"} id={"terrain-slope-line"} filter={['==', ['get', 'kind'], 'slope']}
                            layout={{'line-cap': 'round'}}
                            paint={{'line-color': '#4dabf7', 'line-width': 2.5}}/>
                        <Layer type={"symbol"} id={"terrain-slope-label"} filter={['==', ['get', 'kind'], 'slope']}
                            layout={{'text-field': ['get', 'label'], 'symbol-placement': 'line-center', 'text-size': 12, 'text-allow-overlap': true}}
                            paint={{'text-color': '#ffffff', 'text-halo-color': '#1c7ed6', 'text-halo-width': 1.5}}/>
                        <Layer type={"circle"} id={"terrain-cluster-circle"} filter={['==', ['get', 'kind'], 'cluster']}
                            paint={{'circle-radius': ['case', ['==', ['get', 'selected'], 1], 9, 7], 'circle-color': ['get', 'color'],
                                'circle-opacity': 0.9, 'circle-stroke-color': '#ffffff', 'circle-stroke-width': ['case', ['==', ['get', 'selected'], 1], 2.5, 1]}}/>
                        <Layer type={"symbol"} id={"terrain-cluster-label"} filter={['==', ['get', 'kind'], 'cluster']}
                            layout={{'text-field': ['get', 'label'], 'text-size': 10, 'text-allow-overlap': true}}
                            paint={{'text-color': '#000000'}}/>
                    </Source>
                    {/* Live mow progress on the plan + the robot's actual track (last 5 min). */}
                    <Source type={"geojson"} id={"mow-live"} data={liveCollection}>
                        <Layer type={"line"} id={"mow-live-remaining"} filter={['==', ['get', 'kind'], 'remaining']}
                            layout={{'line-cap': 'round', 'line-join': 'round'}}
                            paint={{'line-color': LAYER_COLORS.liveRemaining, 'line-width': 2, 'line-opacity': 0.8}}/>
                        <Layer type={"line"} id={"mow-live-mowed"} filter={['==', ['get', 'kind'], 'mowed']}
                            layout={{'line-cap': 'round', 'line-join': 'round'}}
                            paint={{'line-color': LAYER_COLORS.liveMowed, 'line-width': 4}}/>
                        <Layer type={"line"} id={"mow-live-skipped"} filter={['==', ['get', 'kind'], 'skipped']}
                            layout={{'line-cap': 'round', 'line-join': 'round'}}
                            paint={{'line-color': LAYER_COLORS.liveSkipped, 'line-width': 4}}/>
                        <Layer type={"line"} id={"mow-live-current"} filter={['==', ['get', 'kind'], 'current']}
                            layout={{'line-cap': 'round', 'line-join': 'round'}}
                            paint={{'line-color': LAYER_COLORS.liveCurrent, 'line-width': 4}}/>
                        <Layer type={"line"} id={"mow-live-track"} filter={['==', ['get', 'kind'], 'track']}
                            layout={{'line-cap': 'round', 'line-join': 'round'}}
                            paint={{'line-color': LAYER_COLORS.track, 'line-width': 1.5, 'line-opacity': 0.7, 'line-dasharray': [1, 1.5]}}/>
                    </Source>
                    {/* Raw scan points only until the LiDAR map exists — then the map replaces them. */}
                    {!lidarMapImage && (
                        <Source type={"geojson"} id={"lidar"} data={lidarCollection}>
                            <Layer type={"circle"} id={"lidar-points"} paint={{
                                "circle-radius": 3,
                                "circle-color": [
                                    "case",
                                    ["==", ["get", "intensity"], "hit"],
                                    LAYER_COLORS.lidarHit,
                                    LAYER_COLORS.lidarMiss
                                ],
                                "circle-stroke-width": 0,
                            }}/>
                        </Source>
                    )}
                    {/* Profile image markers (dock, robot) on top of every other layer. */}
                    <MapImageMarker key={`dock-img-${!!mowProgressImage}-${!!lidarMapImage}`} id={"dock-image"}
                        src={dockMarker?.src ?? ""} corners={dockMarker ? dockMarkerCorners : null}/>
                    <MapImageMarker key={`robot-img-${!!mowProgressImage}-${!!lidarMapImage}`} id={"robot-image"}
                        src={robotMarker?.src ?? ""} corners={robotMarkerCorners}/>
                </Map> : <Spinner/>}
                {hasDriveCamera && (
                    <DrivingCameraPip
                        manualMode={manualMode}
                        mobile={isMobile}
                        reversing={reversing}
                        drivingCamera={robotProfile.drivingCamera}
                        reverseCamera={robotProfile.reverseCamera}
                        prefs={drivePip}
                        onPrefsChange={updateDrivePip}
                    />
                )}
                <JoystickOverlay
                    visible={highLevelStatus.highLevelStatus.state_name === "RECORDING" || highLevelStatus.highLevelStatus.state_name === "MANUAL_MOWING" || manualMode}
                    isRecording={highLevelStatus.highLevelStatus.state_name === "RECORDING"}
                    recordKind={recordKind}
                    mobile={isMobile}
                    onMove={onJoyMove}
                    onStop={onJoyStop}
                    onFinishRecording={mowerActions.onRecordFinish}
                    onCancelRecording={mowerActions.onRecordCancel}
                    onHome={mowerActions.onHome}
                    linkDown={joyStream.status === "reconnecting" || joyStream.status === "connecting"}
                    sideControls={bladeTwoStep && manualMode ? (
                        <ManualBladeControl
                            bladeOn={bladeOn}
                            canStart={canStartBlade}
                            onStart={handleBladeStart}
                            onStop={handleBladeStop}
                        />
                    ) : undefined}
                />
                {/* One top stack (no more overlap between the Stop/fault banner and
                    the sub-state pill): banner, then the always-visible state line
                    with "Why stopped?" (and, on a phone, the compact progress bar).
                    Desktop leaves the right 272 px to the areas panel. */}
                <div style={{position: 'absolute', top: 12, left: isMobile ? 12 : 16, right: isMobile ? 12 : 272,
                    zIndex: 20, display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 6, pointerEvents: 'none'}}>
                    <div style={{pointerEvents: 'auto', maxWidth: '100%'}}>
                        <MissionStopControls
                            state={highLevelStatus.highLevelStatus.state}
                            stateName={highLevelStatus.highLevelStatus.state_name}
                            subStateName={highLevelStatus.highLevelStatus.sub_state_name}
                            onStart={() => { void mowerActions.onStart(); }}
                        />
                    </div>
                    {!editMap && (
                        <MissionStatusLine
                            stateName={highLevelStatus.highLevelStatus.state_name}
                            subStateName={highLevelStatus.highLevelStatus.sub_state_name}
                            why={missionProgress?.why}
                            style={{width: isMobile ? '100%' : 'min(560px, 100%)'}}
                        >
                            {isMobile && liveProgress && (
                                <MowProgressCard compact progress={liveProgress} colors={progressColors}/>
                            )}
                        </MissionStatusLine>
                    )}
                </div>
                {!isMobile && !editMap && liveProgress && (
                    <div style={{position: 'absolute', zIndex: 15, bottom: 84, left: 16, maxWidth: 300, background: colors.glassBackground, border: colors.glassBorder, boxShadow: colors.glassShadow, borderRadius: 14, padding: '8px 12px'}}>
                        <MowProgressCard progress={liveProgress} colors={progressColors}
                                         areaName={liveAreaName}/>
                    </div>
                )}
                {previewCard}
                {!compact && !editMap && terrainSummary && terrainHasData && (terrainCardHidden ? (
                    <Button size="small" style={{position: 'absolute', zIndex: 15, ...(isMobile ? {top: 64, right: 12} : {top: 72, left: 16})}}
                            onClick={() => setTerrainCardHidden(false)}>Terrain</Button>
                ) : (
                    <div style={{position: 'absolute', zIndex: 15, ...(isMobile ? {top: 64, right: 12} : {top: 72, left: 16}), maxWidth: 340, background: colors.glassBackground, border: colors.glassBorder, boxShadow: colors.glassShadow, borderRadius: 14, padding: '8px 12px'}}>
                        <TerrainCard
                            summary={terrainSummary}
                            selected={terrainSel}
                            onSelect={setTerrainSel}
                            onAction={(action, area, id) => postTerrainAction(guiApi, action, area, id)}
                            onDismiss={() => setTerrainCardHidden(true)}
                        />
                    </div>
                ))}
                {editMap && !pathTool.editing && features["dock"] instanceof DockFeatureBase && (
                    <DockHeadingPanel
                        heading={(features["dock"] as DockFeatureBase).getHeading()}
                        onChange={handleDockHeadingChange}
                        onApply={handleApplyDockPose}
                        mobile={isMobile}
                    />
                )}
                {isMobile && (
                    <MapToolbarMobile
                        editMap={editMap}
                        hasUnsavedChanges={hasUnsavedChanges}
                        manualMode={manualMode}
                        useSatellite={useSatellite}
                        historyIndex={historyIndex}
                        editHistoryLength={editHistory.length}
                        mowingAreas={mowingAreas}
                        settingsAreas={areaSettings.enabled
                            ? areasList.filter((a) => a.ftype !== 'obstacle').map((a) => ({key: String(a.id), label: a.name}))
                            : undefined}
                        onAreaSettings={areaSettings.enabled ? openAreaSettingsById : undefined}
                        selectedFeatureCount={selectedFeatureIds.length}
                        onEditMap={handleEditMap}
                        onEditSelectedFeature={handleEditSelectedFeature}
                        onDrawPolygon={handleDrawPolygon}
                        onDrawShape={handleDrawShape}
                        onDrawEmoji={handleDrawEmoji}
                        onTrash={handleTrash}
                        onCombine={handleCombine}
                        onSubtract={handleSubtract}
                        onSplit={handleSplit}
                        onPlaceDock={handleDockPlacement}
                        dockPlacementMode={dockPlacementMode}
                        onDrawPath={pathTool.startPath}
                        onDrawPathToDock={pathTool.startPathToDock}
                        onEditPath={handleEditPath}
                        editPathEnabled={selectedPath !== null}
                        onConnectDock={pathTool.connectNearestAreaToDock}
                        dockAvailable={pathTool.dockAvailable}
                        onSaveMap={handleSaveMap}
                        onUndo={handleUndo}
                        onRedo={handleRedo}
                        onToggleSatellite={() => setUseSatellite(!useSatellite)}
                        onManualMode={handleManualMode}
                        onStopManualMode={handleStopManualMode}
                        onBackupMap={handleBackupMap}
                        onRestoreMap={handleRestoreMap}
                        onDownloadGeoJSON={handleDownloadGeoJSON}
                        onUploadGeoJSON={handleUploadGeoJSON}
                        onImportOpenMower={() => handleImportOpenMower(setImportPreview, setImportFileText)}
                        onMowArea={startSelectedArea}
                        onPreviewPlan={previewSelectedPlan}
                        stateName={highLevelStatus.highLevelStatus.state_name}
                        highLevelState={highLevelStatus.highLevelStatus.state}
                        emergency={highLevelStatus.highLevelStatus.emergency}
                        onResetMowingProgress={resetMowingProgress}
                        {...mowerActions}
                    />
                )}
                {/* Desktop: Edit mode — left vertical toolbar */}
                {!isMobile && editMap && (
                    <MapEditorToolbar
                        hasUnsavedChanges={hasUnsavedChanges}
                        historyIndex={historyIndex}
                        editHistoryLength={editHistory.length}
                        selectedFeatureCount={selectedFeatureIds.length}
                        onSaveMap={handleSaveMap}
                        onCancel={handleEditMap}
                        onUndo={handleUndo}
                        onRedo={handleRedo}
                        onDrawPolygon={handleDrawPolygon}
                        onDrawShape={handleDrawShape}
                        onDrawEmoji={handleDrawEmoji}
                        onTrash={handleTrash}
                        onCombine={handleCombine}
                        onSubtract={handleSubtract}
                        onSplit={handleSplit}
                        onEditSelectedFeature={handleEditSelectedFeature}
                        onPlaceDock={handleDockPlacement}
                        dockPlacementMode={dockPlacementMode}
                        onDrawPath={pathTool.startPath}
                        onDrawPathToDock={pathTool.startPathToDock}
                        onEditPath={handleEditPath}
                        editPathEnabled={selectedPath !== null}
                        pathDrawing={pathTool.drawing}
                        onConnectDock={pathTool.connectNearestAreaToDock}
                        dockAvailable={pathTool.dockAvailable}
                    />
                )}
                {/* Desktop: View mode — bottom glass toolbar */}
                {!isMobile && !editMap && (
                    <div style={{position: 'absolute', bottom: 12, left: 16, right: 16, zIndex: 10, background: colors.glassBackground, backdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined, WebkitBackdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined, borderRadius: 18, border: colors.glassBorder, boxShadow: colors.glassShadow, padding: '10px 14px'}}>
                        <MapToolbar
                            manualMode={manualMode}
                            useSatellite={useSatellite}
                            mowingAreas={mowingAreas}
                            stateName={highLevelStatus.highLevelStatus.state_name}
                            highLevelState={highLevelStatus.highLevelStatus.state}
                            emergency={highLevelStatus.highLevelStatus.emergency}
                            onResetMowingProgress={resetMowingProgress}
                            pitched={pitched}
                            onTogglePitch={togglePitch}
                            onEditMap={handleEditMap}
                            onToggleSatellite={() => setUseSatellite(!useSatellite)}
                            onManualMode={handleManualMode}
                            onStopManualMode={handleStopManualMode}
                            onBackupMap={handleBackupMap}
                            onRestoreMap={handleRestoreMap}
                            onDownloadGeoJSON={handleDownloadGeoJSON}
                            onImportOpenMower={() => handleImportOpenMower(setImportPreview, setImportFileText)}
                            onMowArea={startSelectedArea}
                            onPreviewPlan={previewSelectedPlan}
                            settingsAreas={areaSettings.enabled
                                ? areasList.filter((a) => a.ftype !== 'obstacle').map((a) => ({key: String(a.id), label: a.name}))
                                : undefined}
                            onAreaSettings={areaSettings.enabled ? openAreaSettingsById : undefined}
                            {...mowerActions}
                        />
                    </div>
                )}
                {/* Desktop: Right panel — areas list + offset */}
                {!isMobile && (
                    <div style={{position: 'absolute', top: 12, right: 16, zIndex: 10, display: 'flex', flexDirection: 'column', gap: 0, width: 240, maxHeight: 'calc(100% - 32px)', background: colors.glassBackground, backdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined, WebkitBackdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined, borderRadius: 18, border: colors.glassBorder, boxShadow: colors.glassShadow, overflow: 'hidden'}}>
                        <AreasListPanel
                            areas={areasList}
                            onAreaClick={editMap ? handleAreaSelect : (areaSettings.enabled ? openAreaSettingsById : undefined)}
                            onReorder={editMap ? handleReorder : undefined}
                            selectedId={editMap ? selectedFeatureIds[0] : undefined}
                        />
                        {dynamicObstacles.length > 0 && (
                            <div style={{borderTop: `1px solid ${colors.borderSubtle}`}}>
                                <TrackedObstaclesPanel
                                    obstacles={dynamicObstacles}
                                    obstacleAreaIndex={obstacleAreaIndex}
                                    areaNames={obstacleAreaNames}
                                    selectedObstacleId={selectedObstacleId}
                                    onHoverObstacle={setSelectedObstacleId}
                                />
                            </div>
                        )}
                        <div style={{borderTop: `1px solid ${colors.borderSubtle}`, padding: 8}}>
                            <MapOffsetPanel
                                offsetX={offsetX}
                                offsetY={offsetY}
                                bearing={bearing}
                                onChangeX={handleOffsetX}
                                onChangeY={handleOffsetY}
                                onChangeBearing={handleBearing}
                            />
                        </div>
                    </div>
                )}
            </div>
            {areaSettings.enabled && (
                <>
                    <AreaSettingsDrawer
                        target={settingsArea?.index ?? null}
                        title={settingsNavName ?? settingsArea?.name}
                        areaName={settingsArea?.areaName}
                        disabledHint={settingsNavName !== null ? t('mapPath.notMowedHint') : undefined}
                        onClose={() => { setSettingsArea(null); setSettingsNavName(null); }}
                    />
                    <StartMowSheet
                        open={startSheet.open}
                        initialSelection={startSheet.selection}
                        areas={areaChoices}
                        onClose={() => setStartSheet((s) => ({...s, open: false}))}
                        onEdit={(target) => setSettingsArea({
                            index: target,
                            name: target === "defaults" ? t('areaSettings.defaultsTitle') : areaChoices.find((a) => a.index === target)?.name,
                            areaName: target === "defaults" ? undefined
                                : map?.working_area?.[map?.working_area_indices?.indexOf(target) ?? -1]?.name,
                        })}
                    />
                </>
            )}
            <ImportOpenMowerModal
                preview={importPreview}
                onApply={async (omDatumLat, omDatumLon, importDatum) => {
                    if (!importFileText) {
                        throw new Error("No imported map text in memory — re-select the file.");
                    }
                    await handleApplyOpenMowerImport(importFileText, omDatumLat, omDatumLon, importDatum);
                }}
                onReproject={async (omDatumLat, omDatumLon) => {
                    if (!importFileText) {
                        throw new Error("No imported map text in memory — re-select the file.");
                    }
                    const summary = await handleReprojectOpenMowerPreview(importFileText, omDatumLat, omDatumLon);
                    setImportPreview(summary);
                    return summary;
                }}
                onClose={() => {
                    setImportPreview(null);
                    setImportFileText(null);
                }}
            />
        </div>
    );
}

//MapPage.whyDidYouRender = true

export default MapPage;
