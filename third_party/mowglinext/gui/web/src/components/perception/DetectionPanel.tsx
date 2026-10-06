import {Empty, Tag} from "antd";
import {useTranslation} from "react-i18next";
import type {CameraInfo} from "../../hooks/useCameras.ts";
import type {DetectionSummary} from "../../hooks/useDetections.ts";
import {cameraForFrame} from "./streamUrl.ts";

interface Props {
    detections: DetectionSummary;
    /** null until the first message arrives. */
    lastMessageAt: number | null;
    obstacleClose: boolean;
    cameras: CameraInfo[];
}

export function DetectionPanel({detections, lastMessageAt, obstacleClose, cameras}: Props) {
    const {t} = useTranslation();
    const cam = cameraForFrame(cameras, detections.frame_id);
    const cameraName = cam?.label ?? detections.frame_id;
    return (
        <div data-testid="detection-panel" style={{display: "flex", flexDirection: "column", gap: 10}}>
            <div style={{display: "flex", alignItems: "center", gap: 8}}>
                <strong>{t("perception.detections")}</strong>
                {obstacleClose
                    ? <Tag color="red" data-testid="obstacle-close-badge">{t("perception.obstacleClose")}</Tag>
                    : <Tag color="green" data-testid="obstacle-clear-badge">{t("perception.obstacleClear")}</Tag>}
            </div>
            {lastMessageAt === null ? (
                <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description={t("perception.noDetections")}/>
            ) : (
                <dl style={{display: "grid", gridTemplateColumns: "auto 1fr", gap: "4px 12px", margin: 0}}>
                    <dt>{t("perception.count")}</dt>
                    <dd style={{margin: 0}} data-testid="det-count">{detections.count}</dd>
                    <dt>{t("perception.classes")}</dt>
                    <dd style={{margin: 0}} data-testid="det-classes">
                        {detections.classes.length
                            ? detections.classes.map((c) => <Tag key={c}>{c}</Tag>)
                            : "-"}
                    </dd>
                    <dt>{t("perception.maxScore")}</dt>
                    <dd style={{margin: 0}} data-testid="det-score">
                        {detections.count > 0 ? `${Math.round(detections.max_score * 100)}%` : "-"}
                    </dd>
                    <dt>{t("perception.camera")}</dt>
                    <dd style={{margin: 0}} data-testid="det-camera">{cameraName ?? "-"}</dd>
                </dl>
            )}
        </div>
    );
}
