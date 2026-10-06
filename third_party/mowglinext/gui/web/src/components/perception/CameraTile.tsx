import {Button, Segmented, Tooltip} from "antd";
import {PauseCircleOutlined, PlayCircleOutlined} from "@ant-design/icons";
import {useState} from "react";
import {useTranslation} from "react-i18next";
import type {CameraInfo} from "../../hooks/useCameras.ts";
import type {StreamVariant} from "./streamUrl.ts";

interface Props {
    camera: CameraInfo;
    /** Fully built MJPEG URL; only dereferenced while `streaming`. */
    src: string;
    streaming: boolean;
    variant: StreamVariant;
    onVariantChange: (v: StreamVariant) => void;
    onToggle: () => void;
    highlight?: boolean;
}

/**
 * One camera tile. The <img> exists only while `streaming`: unmounting it
 * closes the MJPEG connection, which is what actually stops the CPU cost on
 * the robot (web_video_server encodes per connected viewer).
 */
export function CameraTile({camera, src, streaming, variant, onVariantChange, onToggle, highlight}: Props) {
    const {t} = useTranslation();
    const [failed, setFailed] = useState(false);
    return (
        <div data-testid={`camera-tile-${camera.id}`} style={{
            border: `1px solid ${highlight ? "#faad14" : "var(--ant-color-border, #d9d9d9)"}`,
            borderRadius: 8, overflow: "hidden", display: "flex", flexDirection: "column",
        }}>
            <div style={{display: "flex", justifyContent: "space-between", alignItems: "center", gap: 8, padding: "6px 10px", flexWrap: "wrap"}}>
                <strong>{camera.label}</strong>
                <div style={{display: "flex", gap: 8, alignItems: "center"}}>
                    {camera.annotatedTopic && (
                        <Segmented
                            size="small"
                            value={variant}
                            onChange={(v) => {
                                setFailed(false);
                                onVariantChange(v as StreamVariant);
                            }}
                            options={[
                                {label: t("perception.raw"), value: "raw"},
                                {label: t("perception.annotated"), value: "annotated"},
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
            <div style={{aspectRatio: "16 / 9", background: "#111", display: "flex", alignItems: "center", justifyContent: "center", color: "#aaa", fontSize: 13}}>
                {streaming && !failed ? (
                    <img
                        src={src}
                        alt={camera.label}
                        style={{width: "100%", height: "100%", objectFit: "contain"}}
                        onError={() => setFailed(true)}
                    />
                ) : (
                    <span>{failed ? t("perception.streamError") : t("perception.tilePaused")}</span>
                )}
            </div>
        </div>
    );
}
