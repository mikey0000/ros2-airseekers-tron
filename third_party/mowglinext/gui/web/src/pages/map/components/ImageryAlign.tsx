import {useCallback, useMemo} from "react";
import {Button, InputNumber, Slider, Space, Tooltip, Typography} from "antd";
import {AimOutlined, CheckOutlined, CloseOutlined, EnvironmentOutlined, PictureOutlined, RobotOutlined} from "@ant-design/icons";
import {Marker} from "react-map-gl/mapbox";
import {useTranslation} from "react-i18next";
import {transpose} from "../../../utils/map.tsx";
import {imageToMap, mapToImage, normalizeDeg, placementFromCornerDrag, type XY} from "../../../utils/imagery.ts";
import {isCpComplete, type AlignDeps, type ImageryAlignApi} from "../hooks/useImageryAlign.ts";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";

const handleStyle = (bg: string): React.CSSProperties => ({
    width: 18, height: 18, borderRadius: 9, background: bg, border: "2px solid #fff",
    boxShadow: "0 0 4px rgba(0,0,0,0.6)", cursor: "grab",
});
const cpStyle = (bg: string, square: boolean): React.CSSProperties => ({
    width: 20, height: 20, borderRadius: square ? 3 : 10, background: bg, border: "2px solid #fff",
    color: "#02110D", fontSize: 11, fontWeight: 700, display: "flex", alignItems: "center", justifyContent: "center",
    boxShadow: "0 0 4px rgba(0,0,0,0.6)", cursor: "grab",
});

/** Handles drawn on the map during alignment (inside <Map>). */
export function ImageryAlignMarkers({align, offsetX, offsetY, datum}: {align: ImageryAlignApi} & Omit<AlignDeps, "update">) {
    const {session, setPlacement, setCp, toMap} = align;
    const ll = useCallback((xy: XY) => transpose(offsetX, offsetY, datum, xy[1], xy[0]), [offsetX, offsetY, datum]);
    if (!session) return null;
    const {placement: p, width: w, height: h} = session;
    const center = ll([p.centerX, p.centerY]);
    const corner = ll(imageToMap(p, w, h, 1, 0));
    return (
        <>
            <Marker longitude={center[0]} latitude={center[1]} draggable
                    onDrag={(e) => { const [x, y] = toMap(e.lngLat.lng, e.lngLat.lat); setPlacement({centerX: x, centerY: y}); }}>
                <div style={handleStyle("#F3A85C")} title="move"/>
            </Marker>
            <Marker longitude={corner[0]} latitude={corner[1]} draggable
                    onDrag={(e) => {
                        const t = toMap(e.lngLat.lng, e.lngLat.lat);
                        setPlacement(placementFromCornerDrag(p, w, h, t));
                    }}>
                <div style={{...handleStyle("#7CFFB2"), borderRadius: 3}} title="rotate / scale"/>
            </Marker>
            {session.cps.map((c, i) => {
                const label = i === 0 ? "A" : "B";
                const out = [];
                if (typeof c.u === "number" && typeof c.v === "number") {
                    const pos = ll(imageToMap(p, w, h, c.u, c.v));
                    out.push(
                        <Marker key={`img-${i}`} longitude={pos[0]} latitude={pos[1]} draggable
                                onDragEnd={(e) => {
                                    const [x, y] = toMap(e.lngLat.lng, e.lngLat.lat);
                                    const [u, v] = mapToImage(p, w, h, x, y);
                                    setCp(i as 0 | 1, {u, v});
                                }}>
                            <div style={cpStyle("#F3A85C", true)}>{label}</div>
                        </Marker>,
                    );
                }
                if (typeof c.x === "number" && typeof c.y === "number") {
                    const pos = ll([c.x, c.y]);
                    out.push(
                        <Marker key={`tgt-${i}`} longitude={pos[0]} latitude={pos[1]} draggable
                                onDragEnd={(e) => {
                                    const [x, y] = toMap(e.lngLat.lng, e.lngLat.lat);
                                    setCp(i as 0 | 1, {x, y});
                                }}>
                            <div style={cpStyle("#7CFFB2", false)}>{label}</div>
                        </Marker>,
                    );
                }
                return out;
            })}
        </>
    );
}

interface PanelProps {
    align: ImageryAlignApi;
    label: string;
    robotXY: XY | null;
    robotFixOk: boolean;
    mobile?: boolean;
}

/** Floating alignment controls (outside <Map>). */
export function ImageryAlignPanel({align, label, robotXY, robotFixOk, mobile}: PanelProps) {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const {session, pick, setPick, setCp, setPlacement, setOpacity, save, cancel, saving, clearCps} = align;
    const solved = useMemo(() => !!session && isCpComplete(session.cps[0]) && isCpComplete(session.cps[1]), [session]);
    if (!session) return null;
    const p = session.placement;
    const cpRow = (i: 0 | 1) => {
        const c = session.cps[i];
        const hasImg = typeof c.u === "number";
        const hasTgt = typeof c.x === "number";
        const picking = pick?.index === i;
        return (
            <div key={i} style={{display: "flex", alignItems: "center", gap: 6, flexWrap: "wrap"}}>
                <b style={{width: 14}}>{i === 0 ? "A" : "B"}</b>
                <Tooltip title={t("imagery.align.pickImageHint")}>
                    <Button size="small" icon={<PictureOutlined/>} type={picking && pick?.what === "image" ? "primary" : hasImg ? "default" : "dashed"}
                            onClick={() => setPick(picking && pick?.what === "image" ? null : {index: i, what: "image"})}>
                        {t("imagery.align.pickImage")}{hasImg ? " ✓" : ""}
                    </Button>
                </Tooltip>
                <Tooltip title={t("imagery.align.useRobotHint")}>
                    <Button size="small" icon={<RobotOutlined/>} disabled={!robotXY}
                            type={hasTgt ? "default" : "dashed"}
                            onClick={() => robotXY && setCp(i, {x: robotXY[0], y: robotXY[1]})}>
                        {t("imagery.align.useRobot")}
                    </Button>
                </Tooltip>
                <Tooltip title={t("imagery.align.pickMapHint")}>
                    <Button size="small" icon={<EnvironmentOutlined/>}
                            type={picking && pick?.what === "target" ? "primary" : "default"}
                            onClick={() => setPick(picking && pick?.what === "target" ? null : {index: i, what: "target"})}/>
                </Tooltip>
                {hasTgt && <Typography.Text type="secondary" style={{fontSize: 11}}>
                    ({c.x!.toFixed(2)}, {c.y!.toFixed(2)}) m
                </Typography.Text>}
            </div>
        );
    };
    return (
        <div style={{
            position: "absolute", zIndex: 25,
            ...(mobile ? {left: 8, right: 8, bottom: 96} : {left: 16, bottom: 84, width: 400}),
            background: colors.glassBackground, border: colors.glassBorder, boxShadow: colors.glassShadow,
            borderRadius: 14, padding: "10px 12px", maxHeight: mobile ? "55vh" : "70vh", overflowY: "auto",
        }}>
            <div style={{display: "flex", alignItems: "center", gap: 8, marginBottom: 6}}>
                <AimOutlined/>
                <b style={{flex: 1, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap"}}>{t("imagery.align.title", {label})}</b>
                <Button size="small" icon={<CloseOutlined/>} onClick={cancel}>{t("imagery.align.cancel")}</Button>
                <Button size="small" type="primary" icon={<CheckOutlined/>} loading={saving} onClick={() => { void save(); }}>{t("imagery.align.save")}</Button>
            </div>
            <Typography.Paragraph type="secondary" style={{fontSize: 12, marginBottom: 6}}>
                {pick
                    ? (pick.what === "image" ? t("imagery.align.clickImage", {cp: pick.index === 0 ? "A" : "B"}) : t("imagery.align.clickMap", {cp: pick.index === 0 ? "A" : "B"}))
                    : t("imagery.align.intro")}
            </Typography.Paragraph>
            {!robotFixOk && robotXY && (
                <Typography.Paragraph type="warning" style={{fontSize: 12, marginBottom: 6}}>{t("imagery.align.noRtk")}</Typography.Paragraph>
            )}
            <Space direction="vertical" size={6} style={{width: "100%"}}>
                {cpRow(0)}
                {cpRow(1)}
                <div style={{display: "flex", gap: 8, alignItems: "center", fontSize: 12}}>
                    <span style={{color: solved ? colors.primary : colors.textSecondary, flex: 1}}>
                        {solved ? t("imagery.align.solved") : t("imagery.align.notSolved")}
                    </span>
                    <Button size="small" type="link" onClick={clearCps}>{t("imagery.align.clearPoints")}</Button>
                </div>
                <div style={{fontSize: 12, color: colors.textSecondary}}>{t("imagery.align.freeform")}</div>
                <div style={{display: "grid", gridTemplateColumns: "auto 1fr 84px", gap: "4px 8px", alignItems: "center", fontSize: 12}}>
                    <span>{t("imagery.align.rotation")}</span>
                    <Slider min={-180} max={180} step={0.1} value={p.rotationDeg} onChange={(v) => setPlacement({rotationDeg: v})} style={{margin: 0}}/>
                    <InputNumber size="small" value={Number(p.rotationDeg.toFixed(2))} step={0.1} addonAfter="°"
                                 onChange={(v) => v !== null && setPlacement({rotationDeg: normalizeDeg(v)})}/>
                    <span>{t("imagery.align.resolution")}</span>
                    <Slider min={0.2} max={20} step={0.01} value={p.metersPerPixel * 100} onChange={(v) => setPlacement({metersPerPixel: v / 100})} style={{margin: 0}}/>
                    <InputNumber size="small" value={Number((p.metersPerPixel * 100).toFixed(3))} step={0.01} min={0.01} addonAfter="cm"
                                 onChange={(v) => v && v > 0 && setPlacement({metersPerPixel: v / 100})}/>
                    <span>X / Y</span>
                    <InputNumber size="small" value={Number(p.centerX.toFixed(2))} step={0.05} addonAfter="m"
                                 onChange={(v) => v !== null && setPlacement({centerX: v})}/>
                    <InputNumber size="small" value={Number(p.centerY.toFixed(2))} step={0.05} addonAfter="m"
                                 onChange={(v) => v !== null && setPlacement({centerY: v})}/>
                    <span>{t("imagery.opacity")}</span>
                    <Slider min={0} max={1} step={0.05} value={session.opacity} onChange={setOpacity} style={{margin: 0}}/>
                    <span>{Math.round(session.opacity * 100)}%</span>
                </div>
            </Space>
        </div>
    );
}
