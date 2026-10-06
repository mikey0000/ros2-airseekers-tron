import {Empty, Table, Tag} from "antd";
import {useTranslation} from "react-i18next";
import type {CameraInfo} from "../../hooks/useCameras.ts";
import {cameraForFrame} from "./streamUrl.ts";
import {type DetectionHistory, sortedHistogram} from "./detectionHistory.ts";

interface Props {
    history: DetectionHistory;
    obstacleClose: boolean;
    cameras: CameraInfo[];
    /** Date.now() reference for ages (injectable for tests). */
    now?: number;
}

interface Row {
    key: string;
    cls: string;
    score: number;
    camera: string;
    age: number;
}

/**
 * Latest detections per camera (class, score, camera) plus a running class
 * histogram since the page was opened.
 */
export function DetectionPanel({history, obstacleClose, cameras, now = Date.now()}: Props) {
    const {t} = useTranslation();
    const frames = Object.entries(history.byFrame);
    const rows: Row[] = [];
    for (const [frame, {summary, at}] of frames) {
        const camName = cameraForFrame(cameras, frame)?.label ?? frame;
        (summary.boxes ?? []).forEach((b, i) => rows.push({
            key: `${frame}:${i}`, cls: b.class, score: b.score, camera: camName, age: now - at,
        }));
    }
    rows.sort((a, b) => b.score - a.score);
    const hist = sortedHistogram(history.histogram);
    const maxCount = hist[0]?.[1] ?? 1;

    return (
        <div data-testid="detection-panel" style={{display: "flex", flexDirection: "column", gap: 10}}>
            <div style={{display: "flex", alignItems: "center", gap: 8, flexWrap: "wrap"}}>
                <strong>{t("perception.detections")}</strong>
                {obstacleClose
                    ? <Tag color="red" data-testid="obstacle-close-badge">{t("perception.obstacleClose")}</Tag>
                    : <Tag color="green" data-testid="obstacle-clear-badge">{t("perception.obstacleClear")}</Tag>}
                {history.messages > 0 && (
                    <span style={{fontSize: 12, opacity: 0.7}} data-testid="det-messages">
                        {t("perception.messagesSeen", {count: history.messages})}
                    </span>
                )}
            </div>
            {history.messages === 0 ? (
                <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description={t("perception.noDetections")}/>
            ) : (
                <div style={{display: "grid", gap: 16, gridTemplateColumns: "repeat(auto-fit, minmax(280px, 1fr))"}}>
                    <div>
                        <div style={{fontSize: 12, opacity: 0.7, marginBottom: 4}}>
                            {t("perception.latestFrame")}{" "}
                            {frames.map(([frame, {summary}]) => (
                                <Tag key={frame} data-testid={`det-frame-${frame}`}>
                                    {(cameraForFrame(cameras, frame)?.label ?? frame)}: {summary.count}
                                </Tag>
                            ))}
                        </div>
                        {rows.length === 0 ? (
                            <div data-testid="det-none" style={{opacity: 0.75}}>{t("perception.nothingDetected")}</div>
                        ) : (
                            <Table<Row>
                                data-testid="det-table"
                                size="small"
                                pagination={false}
                                dataSource={rows.slice(0, 20)}
                                columns={[
                                    {title: t("perception.class"), dataIndex: "cls"},
                                    {title: t("perception.score"), dataIndex: "score", render: (s: number) => `${Math.round(s * 100)}%`},
                                    {title: t("perception.camera"), dataIndex: "camera"},
                                ]}
                            />
                        )}
                    </div>
                    <div>
                        <div style={{fontSize: 12, opacity: 0.7, marginBottom: 4}}>{t("perception.histogram")}</div>
                        {hist.length === 0 ? <span>-</span> : (
                            <div data-testid="det-histogram" style={{display: "flex", flexDirection: "column", gap: 4}}>
                                {hist.slice(0, 12).map(([cls, n]) => (
                                    <div key={cls} style={{display: "grid", gridTemplateColumns: "110px 1fr 48px", gap: 8, alignItems: "center", fontSize: 13}}>
                                        <span style={{overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap"}}>{cls}</span>
                                        <div style={{background: "var(--ant-color-fill-secondary, #f0f0f0)", borderRadius: 3, height: 10}}>
                                            <div style={{width: `${(n / maxCount) * 100}%`, height: "100%", borderRadius: 3, background: "#1677ff"}}/>
                                        </div>
                                        <span style={{textAlign: "right"}}>{n}</span>
                                    </div>
                                ))}
                            </div>
                        )}
                    </div>
                </div>
            )}
        </div>
    );
}
