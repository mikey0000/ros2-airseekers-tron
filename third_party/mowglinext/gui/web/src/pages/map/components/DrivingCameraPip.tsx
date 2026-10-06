import {Button, Select, Switch, Tag, Tooltip} from "antd";
import {CompressOutlined, DownOutlined, ExpandOutlined, UpOutlined} from "@ant-design/icons";
import {useEffect, useLayoutEffect, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import {useCameras, FALLBACK_DEFAULTS} from "../../../hooks/useCameras.ts";
import {useCameraHealth} from "../../../hooks/useCameraHealth.ts";
import {tileHealth} from "../../../components/perception/CameraTile.tsx";
import {buildStreamUrl} from "../../../components/perception/streamUrl.ts";
import {chooseCamera, type DrivePipPrefs} from "../hooks/useDrivingCamera.ts";

const HEADER_PX = 34;
const MIN_W = 180;
/** Driving wants the freshest frame, not the best one. */
const DRIVE_FPS = 15;

const HEALTH_COLOR = {live: "green", stale: "red", waiting: "default", absent: "red"} as const;

interface Props {
    /** Only mounted while manual mode is active; renders nothing otherwise. */
    manualMode: boolean;
    mobile?: boolean;
    reversing: boolean;
    drivingCamera?: string;
    reverseCamera?: string;
    prefs: DrivePipPrefs;
    onPrefsChange: (patch: Partial<DrivePipPrefs>) => void;
}

type Rect = NonNullable<DrivePipPrefs["rect"]>;

export function defaultRect(cw: number, ch: number, mobile: boolean): Rect {
    const w = Math.max(MIN_W, Math.round(cw * (mobile ? 0.45 : 0.35)));
    const h = Math.round(w * 9 / 16) + HEADER_PX;
    // Mobile: the joystick owns the bottom-left thumb zone, so sit above it.
    const bottom = mobile ? 340 : 16;
    return {x: 12, y: Math.max(56, ch - h - bottom), w, h};
}

function clampRect(r: Rect, cw: number, ch: number): Rect {
    if (!cw || !ch) return r;
    const w = Math.min(Math.max(MIN_W, r.w), cw);
    const h = Math.min(Math.max(HEADER_PX + 60, r.h), ch);
    return {w, h, x: Math.min(Math.max(0, r.x), cw - w), y: Math.min(Math.max(0, r.y), ch - h)};
}

/**
 * Camera picture-in-picture for manual driving on the Map page: one live
 * MJPEG stream (the <img> only exists while expanded, so collapsing or
 * leaving manual mode closes the connection), draggable + resizable, with a
 * full-size "drive view" where the map becomes the inset.
 */
export function DrivingCameraPip(props: Props) {
    if (!props.manualMode) return null;
    return <PipInner {...props}/>;
}

function PipInner({mobile = false, reversing, drivingCamera, reverseCamera, prefs, onPrefsChange}: Props) {
    const {t} = useTranslation();
    const {data} = useCameras();
    const cameras = data?.available ? data.cameras : [];
    const defaults = data?.defaults ?? FALLBACK_DEFAULTS;
    const cam = chooseCamera(cameras, {
        selectedId: prefs.cameraId, drivingCamera, reverseCamera, reversing, autoReverse: prefs.autoReverse,
    });
    const autoSwitched = !!cam && reversing && prefs.autoReverse && cam.id !== prefs.cameraId &&
        cam.id === (reverseCamera ?? "rear");

    const rootRef = useRef<HTMLDivElement>(null);
    const [container, setContainer] = useState({w: 0, h: 0});
    useLayoutEffect(() => {
        const parent = rootRef.current?.parentElement;
        if (!parent) return;
        const measure = () => setContainer({w: parent.clientWidth, h: parent.clientHeight});
        measure();
        const ro = typeof ResizeObserver !== "undefined" ? new ResizeObserver(measure) : undefined;
        ro?.observe(parent);
        return () => ro?.disconnect();
    }, []);
    const [dragRect, setDragRect] = useState<Rect | null>(null);
    const baseRect = prefs.rect ?? defaultRect(container.w, container.h, mobile);
    const rect = clampRect(dragRect ?? baseRect, container.w, container.h);

    const streaming = !prefs.collapsed && !!cam;
    const variant = prefs.annotated && cam?.annotatedTopic ? "annotated" : "raw";
    const status = useCameraHealth(cam?.healthUrl, streaming, cam?.status);
    const topicHealth = variant === "annotated" ? status?.annotated : status?.raw;
    const health = tileHealth(streaming, status, topicHealth);
    const age = topicHealth?.lastFrameAgeMs;
    const [failed, setFailed] = useState(false);
    useEffect(() => setFailed(false), [cam?.id, variant]);

    // Drag (header) and resize (corner) via pointer capture.
    const startPointer = (mode: "move" | "resize") => (e: React.PointerEvent) => {
        if (prefs.driveView || (e.target as HTMLElement).closest("button,.ant-select,.ant-switch")) return;
        e.preventDefault();
        const el = e.currentTarget as HTMLElement;
        el.setPointerCapture?.(e.pointerId);
        const sx = e.clientX, sy = e.clientY, r0 = rect;
        let last = r0;
        const move = (ev: PointerEvent) => {
            const dx = ev.clientX - sx, dy = ev.clientY - sy;
            last = clampRect(mode === "move" ? {...r0, x: r0.x + dx, y: r0.y + dy}
                : {...r0, w: r0.w + dx, h: r0.h + dy}, container.w, container.h);
            setDragRect(last);
        };
        const up = () => {
            el.removeEventListener("pointermove", move);
            el.removeEventListener("pointerup", up);
            el.removeEventListener("pointercancel", up);
            setDragRect(null);
            onPrefsChange({rect: last});
        };
        el.addEventListener("pointermove", move);
        el.addEventListener("pointerup", up);
        el.addEventListener("pointercancel", up);
    };

    if (data && cameras.length === 0) return null;

    const full = prefs.driveView && !prefs.collapsed;
    const box: React.CSSProperties = full
        ? {left: 0, top: 0, width: "100%", height: "100%", zIndex: 40, borderRadius: 0}
        : {left: rect.x, top: rect.y, width: rect.w, height: prefs.collapsed ? HEADER_PX : rect.h, zIndex: 50, borderRadius: 8};

    return (
        <div ref={rootRef} data-testid="drive-camera-pip" style={{
            // overflow stays visible here so the camera Select's dropdown (rendered inside this
            // panel, see getPopupContainer) is not clipped; the video wrapper clips instead.
            position: "absolute", ...box, display: "flex", flexDirection: "column",
            background: "#111", border: `1px solid ${health === "stale" ? "#ff4d4f" : "rgba(255,255,255,0.25)"}`,
            boxShadow: "0 10px 30px -10px rgba(0,0,0,0.7)", color: "#ddd",
        }}>
            <div onPointerDown={startPointer("move")} style={{
                height: HEADER_PX, flex: "0 0 auto", display: "flex", alignItems: "center", gap: 6, padding: "0 6px",
                background: "rgba(0,0,0,0.75)", cursor: full ? "default" : "move", touchAction: "none", fontSize: 12,
            }}>
                <div onPointerDown={(e) => e.stopPropagation()} style={{minWidth: 0, flex: "1 1 auto", maxWidth: 170}}>
                <Select
                    size="small"
                    aria-label={t("drivePip.camera")}
                    value={cam?.id}
                    onChange={(id: string) => onPrefsChange({cameraId: id})}
                    options={cameras.map((c) => ({value: c.id, label: c.label}))}
                    style={{width: "100%"}}
                    popupMatchSelectWidth={false}
                    // Render the dropdown inside the panel: the map page's overlays (joystick,
                    // fullscreen drive view) sit above a body-level popup and it was unreachable.
                    getPopupContainer={() => rootRef.current ?? document.body}
                    dropdownStyle={{zIndex: 3000}}
                />
                </div>
                {autoSwitched && <Tag color="orange" style={{marginInlineEnd: 0}}>{t("drivePip.reverse")}</Tag>}
                <Tag color={HEALTH_COLOR[health]} style={{marginInlineEnd: 0}} data-testid="drive-pip-health">
                    {t(`perception.health.${health}`)}
                </Tag>
                {streaming && age != null && (
                    <span data-testid="drive-pip-age" style={{color: health === "stale" ? "#ff4d4f" : undefined, whiteSpace: "nowrap"}}>
                        {t("perception.frameAge", {age: (age / 1000).toFixed(1)})}
                    </span>
                )}
                <span style={{flex: 1}}/>
                <Tooltip title={t("drivePip.autoReverseHint")}>
                    <Switch size="small" checked={prefs.autoReverse} aria-label={t("drivePip.autoReverse")}
                            checkedChildren="R" unCheckedChildren="R"
                            onChange={(v) => onPrefsChange({autoReverse: v})}/>
                </Tooltip>
                {cam?.annotatedTopic && (
                    <Tooltip title={t("drivePip.yoloHint")}>
                        <Switch size="small" checked={prefs.annotated} aria-label={t("drivePip.yolo")}
                                checkedChildren="AI" unCheckedChildren="AI"
                                onChange={(v) => onPrefsChange({annotated: v})}/>
                    </Tooltip>
                )}
                {!prefs.collapsed && (
                    <Tooltip title={prefs.driveView ? t("drivePip.exitDriveView") : t("drivePip.driveView")}>
                        <Button size="small" type="text" style={{color: "#ddd"}}
                                aria-label={prefs.driveView ? t("drivePip.exitDriveView") : t("drivePip.driveView")}
                                icon={prefs.driveView ? <CompressOutlined/> : <ExpandOutlined/>}
                                onClick={() => onPrefsChange({driveView: !prefs.driveView})}/>
                    </Tooltip>
                )}
                <Tooltip title={prefs.collapsed ? t("drivePip.expand") : t("drivePip.collapse")}>
                    <Button size="small" type="text" style={{color: "#ddd"}}
                            aria-label={prefs.collapsed ? t("drivePip.expand") : t("drivePip.collapse")}
                            icon={prefs.collapsed ? <UpOutlined/> : <DownOutlined/>}
                            onClick={() => onPrefsChange({collapsed: !prefs.collapsed, driveView: false})}/>
                </Tooltip>
            </div>
            {!prefs.collapsed && (
                <div style={{position: "relative", flex: "1 1 auto", minHeight: 0, display: "flex", overflow: "hidden",
                    alignItems: "center", justifyContent: "center", fontSize: 13, color: "#aaa"}}>
                    {streaming && cam && !failed ? (
                        <img
                            key={`${cam.id}-${variant}`}
                            data-testid="drive-pip-stream"
                            src={buildStreamUrl(cam, {variant, quality: defaults.quality, fps: Math.min(DRIVE_FPS, defaults.maxFps)}, defaults)}
                            alt={cam.label}
                            draggable={false}
                            style={{width: "100%", height: "100%", objectFit: "contain", pointerEvents: "none"}}
                            onError={() => setFailed(true)}
                        />
                    ) : (
                        <span>{failed ? t("perception.streamError") : t("drivePip.noCamera")}</span>
                    )}
                    {!full && (
                        <div onPointerDown={startPointer("resize")} aria-label={t("drivePip.resize")} style={{
                            position: "absolute", right: 0, bottom: 0, width: 22, height: 22, cursor: "nwse-resize",
                            touchAction: "none", background: "linear-gradient(135deg, transparent 50%, rgba(255,255,255,0.5) 50%)",
                        }}/>
                    )}
                </div>
            )}
        </div>
    );
}
