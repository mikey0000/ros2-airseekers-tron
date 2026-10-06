import {useState} from "react";
import {App, Button, Popconfirm, Progress, Select, Slider, Switch, Tag, Tooltip, Upload} from "antd";
import {AimOutlined, ArrowDownOutlined, ArrowUpOutlined, DeleteOutlined, InboxOutlined, ZoomInOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import type {ImageryApi} from "../../../hooks/useImagery.ts";
import {IMAGERY_ACCEPT, IMAGERY_MAX_UPLOAD_BYTES, moveOverlay, validateUpload, type ImageryOverlay} from "../../../utils/imagery.ts";

interface Props {
    imagery: ImageryApi;
    onAlign: (o: ImageryOverlay) => void;
    onZoomTo: (o: ImageryOverlay) => void;
    /** Hide the section header (the mobile drawer has its own title). */
    bare?: boolean;
}

const sectionTitle = (color: string): React.CSSProperties => ({
    fontSize: 12, fontWeight: 600, color, textTransform: "uppercase", letterSpacing: "0.05em", marginBottom: 6,
});

/** "Imagery" section of the Map settings card: upload + per-overlay controls. */
export function ImageryPanel({imagery, onAlign, onZoomTo, bare}: Props) {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const {notification} = App.useApp();
    const {overlays, update, reorder, remove, uploadFile, upload, cancelUpload} = imagery;
    const [scheme, setScheme] = useState("auto");

    const doUpload = async (file: File) => {
        const bad = validateUpload(file);
        if (bad) {
            notification.error({message: bad === "size"
                ? t("imagery.tooLarge", {mb: IMAGERY_MAX_UPLOAD_BYTES / 1024 / 1024})
                : t("imagery.badType")});
            return;
        }
        try {
            const o = await uploadFile(file, {scheme: file.name.toLowerCase().endsWith(".zip") ? scheme : undefined});
            notification.success({message: t("imagery.uploaded", {label: o.label}), description: t("imagery.processingHint")});
        } catch (e) {
            const msg = e instanceof Error ? e.message : String(e);
            if (msg !== "cancelled") notification.error({message: t("imagery.uploadFailed"), description: msg});
        }
    };

    const names = overlays.map((o) => o.name);
    const err = (e: unknown) => notification.error({message: t("imagery.saveFailed"), description: e instanceof Error ? e.message : String(e)});

    return (
        <div>
            {!bare && <div style={sectionTitle(colors.muted)}>{t("imagery.title")}</div>}
            {upload ? (
                <div style={{fontSize: 12, marginBottom: 8}}>
                    <div style={{overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap"}}>{t("imagery.uploading", {file: upload.fileName})}</div>
                    <div style={{display: "flex", alignItems: "center", gap: 6}}>
                        <Progress percent={Math.round(upload.progress * 100)} size="small" style={{flex: 1, margin: 0}}/>
                        <Button size="small" type="link" onClick={cancelUpload}>{t("imagery.cancel")}</Button>
                    </div>
                </div>
            ) : (
                <Upload.Dragger accept={IMAGERY_ACCEPT} multiple={false} showUploadList={false}
                                beforeUpload={(file) => { void doUpload(file as File); return false; }}
                                style={{padding: 0, marginBottom: 6}}>
                    <div style={{padding: "4px 6px", fontSize: 12}}>
                        <InboxOutlined style={{fontSize: 20, color: colors.primary}}/>
                        <div>{t("imagery.drop")}</div>
                        <div style={{color: colors.textSecondary, fontSize: 11}}>{t("imagery.dropHint", {mb: IMAGERY_MAX_UPLOAD_BYTES / 1024 / 1024})}</div>
                    </div>
                </Upload.Dragger>
            )}
            <div style={{display: "flex", alignItems: "center", gap: 6, fontSize: 11, color: colors.textSecondary, marginBottom: 6}}>
                <span>{t("imagery.zipScheme")}</span>
                <Select size="small" value={scheme} onChange={setScheme} style={{flex: 1}}
                        options={[{value: "auto", label: t("imagery.schemeAuto")}, {value: "xyz", label: "XYZ"}, {value: "tms", label: "TMS"}]}/>
            </div>
            {overlays.length === 0 && <div style={{fontSize: 12, color: colors.textSecondary}}>{t("imagery.empty")}</div>}
            {/* Top of the stack first. */}
            {[...overlays].reverse().map((o) => {
                const idx = names.indexOf(o.name);
                const unplaced = o.kind === "image" && o.status === "ready" && !o.placement;
                return (
                    <div key={o.name} style={{borderTop: `1px solid ${colors.borderSubtle}`, padding: "6px 0", fontSize: 12}}>
                        <div style={{display: "flex", alignItems: "center", gap: 4}}>
                            <Switch size="small" checked={o.visible} disabled={o.status !== "ready"}
                                    onChange={(v) => { update(o.name, {visible: v}).catch(err); }}/>
                            <span style={{flex: 1, fontWeight: 600, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap"}} title={o.label}>{o.label}</span>
                            <Tag style={{marginInlineEnd: 0, fontSize: 10, lineHeight: "16px"}}>{t(`imagery.kind.${o.kind}`)}</Tag>
                        </div>
                        {o.status === "processing" && <Progress percent={Math.round(o.progress * 100)} size="small" status="active" format={() => t("imagery.processing")}/>}
                        {o.status === "error" && <div style={{color: colors.danger, wordBreak: "break-word"}}>{o.error}</div>}
                        {unplaced && <div style={{color: colors.amber}}>{t("imagery.notAligned")}</div>}
                        {o.status === "ready" && (
                            <div style={{display: "flex", alignItems: "center", gap: 6}}>
                                <Slider min={0} max={1} step={0.05} value={o.opacity} style={{flex: 1, margin: "4px 0"}}
                                        tooltip={{formatter: (v) => `${Math.round((v ?? 0) * 100)}%`}}
                                        onChange={(v) => { update(o.name, {opacity: v}).catch(err); }}/>
                            </div>
                        )}
                        <div style={{display: "flex", gap: 2, justifyContent: "flex-end"}}>
                            {o.kind === "image" && o.status === "ready" && (
                                <Tooltip title={t("imagery.align.button")}>
                                    <Button size="small" type={unplaced ? "primary" : "text"} icon={<AimOutlined/>} onClick={() => onAlign(o)}/>
                                </Tooltip>
                            )}
                            {o.status === "ready" && (
                                <Tooltip title={t("imagery.zoomTo")}>
                                    <Button size="small" type="text" icon={<ZoomInOutlined/>} disabled={unplaced} onClick={() => onZoomTo(o)}/>
                                </Tooltip>
                            )}
                            <Tooltip title={t("imagery.moveUp")}>
                                <Button size="small" type="text" icon={<ArrowUpOutlined/>} disabled={idx >= names.length - 1}
                                        onClick={() => { reorder(moveOverlay(names, o.name, "up")).catch(err); }}/>
                            </Tooltip>
                            <Tooltip title={t("imagery.moveDown")}>
                                <Button size="small" type="text" icon={<ArrowDownOutlined/>} disabled={idx <= 0}
                                        onClick={() => { reorder(moveOverlay(names, o.name, "down")).catch(err); }}/>
                            </Tooltip>
                            <Popconfirm title={t("imagery.deleteConfirm", {label: o.label})} okText={t("imagery.delete")} okButtonProps={{danger: true}}
                                        onConfirm={() => { remove(o.name).catch(err); }}>
                                <Button size="small" type="text" danger icon={<DeleteOutlined/>}/>
                            </Popconfirm>
                        </div>
                    </div>
                );
            })}
        </div>
    );
}
