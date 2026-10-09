import {Button, Segmented, Tag, Tooltip} from "antd";
import {PauseCircleOutlined, PlayCircleOutlined} from "@ant-design/icons";
import {useEffect, useState} from "react";
import {useTranslation} from "react-i18next";
import type {CameraInfo, CameraStatus, TopicHealth} from "../../hooks/useCameras.ts";
import type {DetectionBox} from "../../hooks/useDetections.ts";
import type {StreamVariant} from "./streamUrl.ts";
import {MjpegImg} from "./MjpegImg.tsx";

/** A tile is "stale" when its newest frame is older than this. */
export const STALE_AFTER_MS = 3000;

interface Props {
    camera: CameraInfo;
    /** Fully built MJPEG URL; only dereferenced while `streaming`. */
    src: string;
    streaming: boolean;
    variant: StreamVariant;
    onVariantChange: (v: StreamVariant) => void;
    onToggle: () => void;
    highlight?: boolean;
    /** Fresh detections of this camera, drawn client-side over the raw stream. */
    boxes?: DetectionBox[];
    /** Publisher state + proxy-measured fps / last-frame age. */
    status?: CameraStatus;
    /**
     * Poll single JPEG frames from this URL instead of holding an MJPEG
     * stream open (browsers allow only 6 HTTP/1.1 connections per host, so
     * tiles beyond the first few would otherwise never connect).
     */
    snapshotSrc?: string;
    snapshotIntervalMs?: number;
}

export type TileHealth = "live" | "stale" | "waiting" | "absent";

/** Health word for the tile header (pure; exported for tests). */
export function tileHealth(streaming: boolean, status: CameraStatus | undefined, h: TopicHealth | undefined): TileHealth {
    if (status?.state === "not_publishing") return "absent";
    if (!streaming) return status?.state === "publishing" ? "live" : "waiting";
    const age = h?.lastFrameAgeMs;
    if (age == null) return "waiting";
    return age > STALE_AFTER_MS ? "stale" : "live";
}

const HEALTH_COLOR: Record<TileHealth, string> = {live: "green", stale: "red", waiting: "default", absent: "red"};

function boxColor(score: number): string {
    return score >= 0.6 ? "#52c41a" : score >= 0.4 ? "#faad14" : "#ff7a45";
}

/**
 * One camera tile. The <img> exists only while `streaming`: unmounting it
 * closes the MJPEG connection (MjpegImg aborts it explicitly), which is what
 * actually stops the CPU cost on the robot: web_video_server subscribes and
 * encodes per connected viewer, and the camera driver closes the device once
 * the topic has no subscriber.
 */
export function CameraTile({camera, src, streaming, variant, onVariantChange, onToggle, highlight, boxes, status,
                               snapshotSrc, snapshotIntervalMs = 1000}: Props) {
    const {t} = useTranslation();
    const [failed, setFailed] = useState(false);
    const snapshot = useSnapshotSrc(snapshotSrc, streaming, snapshotIntervalMs);
    const imgSrc = snapshotSrc ? snapshot.src : src;
    const [natural, setNatural] = useState<{ w: number; h: number } | null>(null);
    const topicHealth = variant === "annotated" && camera.annotatedTopic ? status?.annotated : status?.raw;
    const health = tileHealth(streaming, status, topicHealth);
    const age = topicHealth?.lastFrameAgeMs;

    // Box coordinates are in source pixels; the SVG viewBox maps them onto the
    // (letterboxed, object-fit: contain) stream.
    const srcW = camera.sourceWidth || natural?.w || camera.width;
    const srcH = camera.sourceHeight || natural?.h || camera.height;
    const drawBoxes = streaming && !failed && variant === "raw" && !!srcW && !!srcH && (boxes?.length ?? 0) > 0;
    const resolution = natural ? `${natural.w}×${natural.h}` : camera.width && camera.height ? `${camera.width}×${camera.height}` : "–";

    return (
        <div data-testid={`camera-tile-${camera.id}`} style={{
            border: `1px solid ${highlight ? "#faad14" : health === "stale" ? "#ff4d4f" : "var(--ant-color-border, #d9d9d9)"}`,
            borderRadius: 8, overflow: "hidden", display: "flex", flexDirection: "column",
        }}>
            <div style={{display: "flex", justifyContent: "space-between", alignItems: "center", gap: 8, padding: "6px 10px", flexWrap: "wrap"}}>
                <div style={{display: "flex", flexDirection: "column", minWidth: 0}}>
                    <strong>{camera.label}</strong>
                    <span data-testid={`tile-stats-${camera.id}`} style={{fontSize: 12, opacity: 0.8, display: "flex", gap: 8, alignItems: "center", flexWrap: "wrap"}}>
                        <Tag color={HEALTH_COLOR[health]} style={{marginInlineEnd: 0}} data-testid={`tile-health-${camera.id}`}>
                            {t(`perception.health.${health}`)}
                        </Tag>
                        <span>{resolution}</span>
                        {snapshotSrc && <span title={t("perception.snapshotModeHint")}>{t("perception.snapshotMode")}</span>}
                        <span data-testid={`tile-fps-${camera.id}`}>
                            {topicHealth && topicHealth.fps > 0 ? `${topicHealth.fps.toFixed(1)} ${t("perception.fpsUnit")}` : `– ${t("perception.fpsUnit")}`}
                        </span>
                        <span data-testid={`tile-age-${camera.id}`} style={{color: health === "stale" ? "#ff4d4f" : undefined}}>
                            {age != null ? t("perception.frameAge", {age: (age / 1000).toFixed(1)}) : ""}
                        </span>
                    </span>
                </div>
                <div style={{display: "flex", gap: 8, alignItems: "center"}}>
                    {camera.annotatedTopic && (
                        <Segmented
                            size="small"
                            value={variant}
                            onChange={(v) => {
                                setFailed(false);
                                setNatural(null);
                                onVariantChange(v as StreamVariant);
                            }}
                            options={[
                                {label: t("perception.annotated"), value: "annotated"},
                                {label: t("perception.raw"), value: "raw"},
                            ]}
                        />
                    )}
                    <Tooltip title={streaming ? t("perception.stopTile") : t("perception.startTile")}>
                        <Button
                            size="small"
                            type="text"
                            aria-label={streaming ? t("perception.stopTile") : t("perception.startTile")}
                            icon={streaming ? <PauseCircleOutlined/> : <PlayCircleOutlined/>}
                            onClick={() => {
                                setFailed(false);
                                onToggle();
                            }}
                        />
                    </Tooltip>
                </div>
            </div>
            <div style={{position: "relative", aspectRatio: "16 / 9", background: "#111", display: "flex", alignItems: "center", justifyContent: "center", color: "#aaa", fontSize: 13}}>
                {streaming && !failed && imgSrc ? (() => {
                    const imgProps = {
                        src: imgSrc,
                        alt: camera.label,
                        style: {width: "100%", height: "100%", objectFit: "contain" as const},
                        onLoad: (e: React.SyntheticEvent<HTMLImageElement>) => {
                            const im = e.currentTarget;
                            if (im.naturalWidth) setNatural({w: im.naturalWidth, h: im.naturalHeight});
                            snapshot.onSettled();
                        },
                        onError: () => snapshotSrc ? snapshot.onSettled() : setFailed(true),
                    };
                    // MJPEG: MjpegImg aborts the connection on unmount / URL change.
                    return snapshotSrc ? <img {...imgProps}/> : <MjpegImg {...imgProps}/>;
                })() : (
                    <span>{failed ? t("perception.streamError") : t("perception.tilePaused")}</span>
                )}
                {drawBoxes && (
                    <svg
                        data-testid={`tile-boxes-${camera.id}`}
                        viewBox={`0 0 ${srcW} ${srcH}`}
                        preserveAspectRatio="xMidYMid meet"
                        style={{position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none"}}
                    >
                        {boxes!.map((b, i) => (
                            <g key={i}>
                                <rect x={b.x} y={b.y} width={b.w} height={b.h} fill="none"
                                      stroke={boxColor(b.score)} strokeWidth={Math.max(2, srcW / 320)}/>
                                <text x={b.x + 2} y={Math.max(srcH / 30, b.y - 4)} fill={boxColor(b.score)}
                                      fontSize={Math.max(12, srcH / 24)} fontWeight={600}
                                      stroke="#000" strokeWidth={0.6} paintOrder="stroke">
                                    {`${b.class} ${Math.round(b.score * 100)}%`}
                                </text>
                            </g>
                        ))}
                    </svg>
                )}
            </div>
        </div>
    );
}

/**
 * Snapshot polling via fetch(): the next frame is requested `intervalMs`
 * after the previous one arrived, at most one request in flight, and shown
 * as a blob: URL. fetch() rather than <img src=...?n>: image loads are
 * low-priority and Chrome queues them behind the open MJPEG connections,
 * whereas fetch() gets a slot.
 */
function useSnapshotSrc(base: string | undefined, enabled: boolean, intervalMs: number) {
    const [src, setSrc] = useState("");
    const active = !!base && enabled;
    useEffect(() => {
        if (!active || !base) {
            setSrc("");
            return;
        }
        let cancelled = false;
        let timer: number | undefined;
        let url = "";
        const ctrl = new AbortController();
        const tick = async () => {
            try {
                const res = await fetch(base, {signal: ctrl.signal, cache: "no-store"});
                if (res.ok) {
                    const blob = await res.blob();
                    if (cancelled) return;
                    const next = URL.createObjectURL(blob);
                    if (url) URL.revokeObjectURL(url);
                    url = next;
                    setSrc(next);
                }
            } catch {
                // keep the last frame; retry below
            }
            if (!cancelled) timer = window.setTimeout(tick, intervalMs);
        };
        void tick();
        return () => {
            cancelled = true;
            ctrl.abort();
            window.clearTimeout(timer);
            if (url) URL.revokeObjectURL(url);
        };
    }, [base, active, intervalMs]);
    return {src, onSettled: () => undefined};
}
