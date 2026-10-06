import {useEffect, useMemo, useState} from "react";
import {useTranslation} from "react-i18next";
import {Alert, Button, Card, Empty, InputNumber, Select, Skeleton, Slider, Switch} from "antd";
import {ReloadOutlined} from "@ant-design/icons";
import {useCameras} from "../hooks/useCameras.ts";
import {useDetections, useVisionObstacleClose} from "../hooks/useDetections.ts";
import {useIsMobile} from "../hooks/useIsMobile.ts";
import {CameraTile} from "../components/perception/CameraTile.tsx";
import {DetectionPanel} from "../components/perception/DetectionPanel.tsx";
import {buildStreamUrl, cameraForFrame, clamp, type StreamVariant} from "../components/perception/streamUrl.ts";

const MAX_TILES = 3;

function usePageVisible(): boolean {
    const [visible, setVisible] = useState(typeof document === "undefined" || !document.hidden);
    useEffect(() => {
        const h = () => setVisible(!document.hidden);
        document.addEventListener("visibilitychange", h);
        return () => document.removeEventListener("visibilitychange", h);
    }, []);
    return visible;
}

export default function PerceptionPage() {
    const {t} = useTranslation();
    const isMobile = useIsMobile();
    const pageVisible = usePageVisible();
    const {data, loading, error, reload} = useCameras();

    const [paused, setPaused] = useState(false);
    const [quality, setQuality] = useState<number | null>(null);
    const [fps, setFps] = useState<number | null>(null);
    const [variants, setVariants] = useState<Record<string, StreamVariant>>({});
    // Tiles the user stopped individually (desktop) / the single selected tile (mobile).
    const [stopped, setStopped] = useState<Record<string, boolean>>({});
    const [selectedId, setSelectedId] = useState<string | null>(null);

    const cameras = useMemo(() => (data?.cameras ?? []).slice(0, MAX_TILES), [data]);
    const available = !!data?.available && cameras.length > 0;
    const defaults = data?.defaults;
    const effQuality = quality ?? defaults?.quality ?? 50;
    const effFps = clamp(fps ?? defaults?.fps ?? 5, 1, defaults?.maxFps ?? 15);

    const live = available && pageVisible;
    const detections = useDetections(live);
    const obstacleClose = useVisionObstacleClose(live);

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

    const isStreaming = (id: string) =>
        live && !paused && (isMobile ? id === activeSelected : !stopped[id]);

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
                {cameras.filter((c) => !isMobile || c.id === activeSelected).map((cam) => {
                    const variant: StreamVariant = cam.annotatedTopic ? (variants[cam.id] ?? "raw") : "raw";
                    const streaming = isStreaming(cam.id);
                    return (
                        <CameraTile
                            key={`${cam.id}:${variant}`}
                            camera={cam}
                            src={streaming ? buildStreamUrl(cam, {variant, quality: effQuality, fps: effFps}, data.defaults) : ""}
                            streaming={streaming}
                            variant={variant}
                            onVariantChange={(v) => setVariants((s) => ({...s, [cam.id]: v}))}
                            onToggle={() => isMobile ? setPaused((p) => !p) : setStopped((s) => ({...s, [cam.id]: !s[cam.id]}))}
                            highlight={obstacleClose && cameraForFrame(cameras, detections.data.frame_id)?.id === cam.id}
                        />
                    );
                })}
            </div>
            <Card size="small">
                <DetectionPanel
                    detections={detections.data}
                    lastMessageAt={detections.lastMessageAt}
                    obstacleClose={obstacleClose}
                    cameras={data.cameras}
                />
            </Card>
            {!pageVisible && <Alert type="info" showIcon message={t("perception.hiddenPaused")}/>}
        </div>
    );
}
