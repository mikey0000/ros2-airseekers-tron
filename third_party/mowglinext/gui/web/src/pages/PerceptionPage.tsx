import {type ComponentProps, useEffect, useMemo, useState} from "react";
import type {CameraInfo} from "../hooks/useCameras.ts";
import type {DetectionBox} from "../hooks/useDetections.ts";
import {useCameraHealth} from "../hooks/useCameraHealth.ts";
import {useDetectionHistory} from "../components/perception/detectionHistory.ts";
import {useTranslation} from "react-i18next";
import {Alert, Button, Card, Empty, InputNumber, Select, Skeleton, Slider, Switch} from "antd";
import {ReloadOutlined} from "@ant-design/icons";
import {useCameras} from "../hooks/useCameras.ts";
import {useDetections, useVisionObstacleClose} from "../hooks/useDetections.ts";
import {useIsMobile} from "../hooks/useIsMobile.ts";
import {usePageVisible} from "../hooks/usePageVisible.ts";
import {CameraTile} from "../components/perception/CameraTile.tsx";
import {DetectionPanel} from "../components/perception/DetectionPanel.tsx";
import {buildSnapshotUrl, buildStreamUrl, cameraForFrame, clamp, type StreamVariant} from "../components/perception/streamUrl.ts";

const MAX_TILES = 6;
/**
 * Browsers allow 6 concurrent HTTP/1.1 connections per host; every MJPEG tile
 * holds one for good. The first MAX_MJPEG streaming tiles get a live stream,
 * the rest poll snapshots, leaving room for API calls and health polls.
 */
const MAX_MJPEG = 3;
const SNAPSHOT_INTERVAL_MS = 1000;
/** Client-side boxes older than this are not drawn. */
const BOX_TTL_MS = 1500;

function useNow(periodMs: number, enabled: boolean): number {
    const [now, setNow] = useState(() => Date.now());
    useEffect(() => {
        if (!enabled) return;
        const id = window.setInterval(() => setNow(Date.now()), periodMs);
        return () => window.clearInterval(id);
    }, [periodMs, enabled]);
    return now;
}

/** CameraTile + its health poll (one hook per tile). */
function LiveTile(props: Omit<ComponentProps<typeof CameraTile>, "status"> & {cam: CameraInfo}) {
    const {cam, ...rest} = props;
    const status = useCameraHealth(cam.healthUrl, rest.streaming, cam.status);
    return <CameraTile {...rest} status={status}/>;
}

/** Desktop tiles the user started (camera id -> streaming), remembered per browser. */
const STARTED_KEY = "perception.startedCameras";

function loadStarted(): Record<string, boolean> | null {
    try {
        const raw = window.localStorage.getItem(STARTED_KEY);
        const v: unknown = raw ? JSON.parse(raw) : null;
        return v && typeof v === "object" && !Array.isArray(v) ? v as Record<string, boolean> : null;
    } catch {
        return null;
    }
}

function saveStarted(v: Record<string, boolean>): void {
    try {
        window.localStorage.setItem(STARTED_KEY, JSON.stringify(v));
    } catch {
        // private mode / storage blocked: the choice just is not remembered
    }
}

export default function PerceptionPage() {
    const {t} = useTranslation();
    // A zero-width viewport (minimised / background automation window) is not
    // a phone: keep the desktop behaviour (all tiles, keep streaming) there.
    const isMobile = useIsMobile() && (typeof window === "undefined" || window.innerWidth > 0);
    const pageVisible = usePageVisible();
    const {data, loading, error, reload} = useCameras();

    const [paused, setPaused] = useState(false);
    const [quality, setQuality] = useState<number | null>(null);
    const [fps, setFps] = useState<number | null>(null);
    const [variants, setVariants] = useState<Record<string, StreamVariant>>({});
    // Tiles the user started (desktop; null = default: only the first camera) /
    // the single selected tile (mobile). Every streaming tile makes the robot
    // capture, convert and JPEG-encode that camera, so nothing streams unless
    // it was asked for.
    const [started, setStarted] = useState<Record<string, boolean> | null>(loadStarted);
    const [selectedId, setSelectedId] = useState<string | null>(null);

    const cameras = useMemo(() => (data?.cameras ?? []).slice(0, MAX_TILES), [data]);
    const available = !!data?.available && cameras.length > 0;
    const defaults = data?.defaults;
    const effQuality = quality ?? defaults?.quality ?? 50;
    const effFps = clamp(fps ?? defaults?.fps ?? 5, 1, defaults?.maxFps ?? 15);

    // Only the started camera(s) stream, and nothing streams while the tab is
    // hidden (desktop too): web_video_server then drops its subscription and
    // the camera driver closes the device, so a forgotten background tab costs
    // the robot nothing. Streams resume when the tab is shown again.
    const live = available && pageVisible;
    const detections = useDetections(live);
    const obstacleClose = useVisionObstacleClose(live);
    const history = useDetectionHistory(detections.data, detections.lastMessageAt);
    const now = useNow(500, live);

    const activeSelected = selectedId && cameras.some((c) => c.id === selectedId) ? selectedId : cameras[0]?.id;

    if (loading && !data) return <Skeleton active style={{padding: 24}}/>;

    const header = (
        <div style={{display: "flex", justifyContent: "space-between", alignItems: "center", flexWrap: "wrap", gap: 8}}>
            <h2 style={{margin: 0}}>{t("perception.title")}</h2>
            <Button icon={<ReloadOutlined/>} onClick={reload} loading={loading}>{t("perception.recheck")}</Button>
        </div>
    );

    if (error || !data || !available) {
        return (
            <div style={{padding: 16, display: "flex", flexDirection: "column", gap: 16}}>
                {header}
                <Card>
                    <Empty
                        data-testid="perception-empty"
                        description={
                            <div>
                                <div>{error ? t("perception.loadFailed") : cameras.length === 0 && data?.available ? t("perception.noCameras") : t("perception.unavailable")}</div>
                                {data?.hint && <div data-testid="perception-hint" style={{marginTop: 8, opacity: 0.75}}>{data.hint}</div>}
                            </div>
                        }
                    />
                </Card>
            </div>
        );
    }

    const startedNow = started ?? (cameras[0] ? {[cameras[0].id]: true} : {});
    const isStreaming = (id: string) =>
        live && !paused && (isMobile ? id === activeSelected : !!startedNow[id]);
    const toggleStarted = (id: string) => {
        const next = {...startedNow, [id]: !startedNow[id]};
        setStarted(next);
        saveStarted(next);
    };

    return (
        <div style={{padding: 16, display: "flex", flexDirection: "column", gap: 16}}>
            {header}
            <Card size="small">
                <div style={{display: "flex", flexWrap: "wrap", gap: 20, alignItems: "center"}}>
                    <label style={{display: "flex", gap: 8, alignItems: "center"}}>
                        <Switch checked={paused} onChange={setPaused} aria-label={t("perception.pause")}/>
                        {t("perception.pause")}
                    </label>
                    <label style={{display: "flex", gap: 8, alignItems: "center"}}>
                        {t("perception.fps")}
                        <InputNumber
                            min={1}
                            max={defaults?.maxFps ?? 15}
                            step={1}
                            value={effFps}
                            onChange={(v) => v != null && setFps(v)}
                            aria-label={t("perception.fps")}
                        />
                    </label>
                    <label style={{display: "flex", gap: 8, alignItems: "center", minWidth: 220}}>
                        {t("perception.quality")}
                        <Slider min={10} max={100} step={5} value={effQuality} onChange={setQuality} style={{flex: 1, minWidth: 120}}/>
                    </label>
                </div>
                {isMobile && cameras.length > 1 && (
                    <div style={{marginTop: 12}}>
                        <Select
                            style={{width: "100%"}}
                            value={activeSelected}
                            onChange={setSelectedId}
                            options={cameras.map((c) => ({value: c.id, label: c.label}))}
                            aria-label={t("perception.camera")}
                        />
                        <div style={{marginTop: 6, fontSize: 12, opacity: 0.7}}>{t("perception.oneAtATime")}</div>
                    </div>
                )}
            </Card>
            <div style={{
                display: "grid", gap: 12,
                gridTemplateColumns: isMobile ? "1fr" : "repeat(auto-fit, minmax(320px, 1fr))",
            }}>
                {(() => { let mjpeg = 0; return cameras.filter((c) => !isMobile || c.id === activeSelected).map((cam) => {
                    // Annotated (detector overlay) by default when the camera has one.
                    const variant: StreamVariant = cam.annotatedTopic ? (variants[cam.id] ?? "annotated") : "raw";
                    const streaming = isStreaming(cam.id);
                    const useMjpeg = streaming && mjpeg < MAX_MJPEG;
                    if (useMjpeg) mjpeg += 1;
                    return (
                        <LiveTile
                            key={`${cam.id}:${variant}`}
                            cam={cam}
                            camera={cam}
                            boxes={freshBoxes(cam, cameras, history.byFrame, now)}
                            src={useMjpeg ? buildStreamUrl(cam, {variant, quality: effQuality, fps: effFps}, data.defaults) : ""}
                            snapshotSrc={streaming && !useMjpeg ? buildSnapshotUrl(cam, {variant, quality: effQuality}) : undefined}
                            snapshotIntervalMs={SNAPSHOT_INTERVAL_MS}
                            streaming={streaming}
                            variant={variant}
                            onVariantChange={(v) => setVariants((s) => ({...s, [cam.id]: v}))}
                            onToggle={() => isMobile ? setPaused((p) => !p) : toggleStarted(cam.id)}
                            highlight={obstacleClose && cameraForFrame(cameras, detections.data.frame_id)?.id === cam.id}
                        />
                    );
                }); })()}
            </div>
            <Card size="small">
                <DetectionPanel
                    history={history}
                    obstacleClose={obstacleClose}
                    cameras={data.cameras}
                    now={now}
                />
            </Card>
            {!pageVisible && <Alert type="info" showIcon message={t("perception.hiddenPaused")}/>}
        </div>
    );
}

/** Boxes of the newest detection message from `cam`, if recent enough. */
function freshBoxes(cam: CameraInfo, cameras: CameraInfo[],
                    byFrame: ReturnType<typeof useDetectionHistory>["byFrame"], now: number): DetectionBox[] | undefined {
    for (const [frame, {summary, at}] of Object.entries(byFrame)) {
        if (now - at <= BOX_TTL_MS && cameraForFrame(cameras, frame)?.id === cam.id) return summary.boxes;
    }
    return undefined;
}
