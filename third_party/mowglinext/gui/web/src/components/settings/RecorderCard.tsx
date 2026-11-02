import React, { useCallback, useEffect, useState } from "react";
import { Alert, Button, Card, Space, Switch, Tag, Typography } from "antd";
import { DatabaseOutlined, ReloadOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useApi } from "../../hooks/useApi.ts";

const { Paragraph, Text } = Typography;

type RecorderStatus = {
    enabled?: boolean;
    recording?: boolean;
    paused_low_disk?: boolean;
    session?: string | null;
    ring_bytes?: number;
    free_bytes?: number;
    restarts?: number;
    pending_pins?: number;
    mow?: string | null;
    mow_pins?: boolean;
    topics?: number;
};

/** The backend answers `available: false` when no bag_recorder node is running. */
function isNotRunning(data: unknown): boolean {
    return (data as { available?: boolean } | undefined)?.available === false;
}

function parseStatus(data: unknown): RecorderStatus | null {
    const msg = (data as { message?: string } | undefined)?.message;
    if (!msg) return null;
    try {
        return JSON.parse(msg) as RecorderStatus;
    } catch {
        return null;
    }
}

const gb = (bytes: number) => (bytes / 1e9).toFixed(1);

/**
 * Always-on rosbag "black box" recorder (bag_recorder node). The switch enables or
 * disables it, including per-mow recordings; the robot remembers the setting.
 * The state is read once on mount and on the refresh button, never polled; a
 * robot without the node gets one line instead of an error and a dead switch.
 */
export const RecorderCard: React.FC = () => {
    const { t } = useTranslation();
    const guiApi = useApi();
    const [status, setStatus] = useState<RecorderStatus | null>(null);
    const [busy, setBusy] = useState(false);
    const [error, setError] = useState<string | null>(null);
    const [notRunning, setNotRunning] = useState(false);

    const call = useCallback(async (command: string, body: Record<string, unknown>) => {
        setBusy(true);
        setError(null);
        try {
            const res = await guiApi.mowglinext.callCreate(command, body);
            if (res.error) throw new Error((res.error as { error?: string }).error ?? "error");
            setNotRunning(isNotRunning(res.data));
            setStatus(parseStatus(res.data));
        } catch (e) {
            setError(e instanceof Error ? e.message : String(e));
        } finally {
            setBusy(false);
        }
    }, [guiApi]);

    const refresh = useCallback(() => call("bag_recorder_state", {}), [call]);
    useEffect(() => { void refresh(); }, [refresh]);

    const stateTag = status?.paused_low_disk
        ? <Tag color="orange">{t("settingsRecorder.pausedLowDisk")}</Tag>
        : status?.recording
            ? <Tag color="red">{t("settingsRecorder.recording")}</Tag>
            : <Tag>{t("settingsRecorder.off")}</Tag>;

    return (
        <Card size="small" title={<Space><DatabaseOutlined />{t("settingsRecorder.title")}</Space>}
              extra={<Button size="small" icon={<ReloadOutlined />} onClick={() => void refresh()} loading={busy} />}
              style={{ marginBottom: 16 }}>
            <Paragraph type="secondary" style={{ marginBottom: 12 }}>
                {t("settingsRecorder.description")}
            </Paragraph>
            {notRunning ? (
                <Text type="secondary">{t("settingsRecorder.notRunning")}</Text>
            ) : (
            <Space direction="vertical" style={{ width: "100%" }}>
                <Switch
                    checked={status?.enabled ?? false}
                    disabled={busy || !status}
                    onChange={(enabled) => void call("bag_recorder", { enabled })}
                />
                {status && (
                    <Space wrap>
                        <Text>{t("settingsRecorder.state")}:</Text>
                        {stateTag}
                        {status.mow && <Tag color="blue">{t("settingsRecorder.mowRecording")}</Tag>}
                        {status.ring_bytes != null && (
                            <Text type="secondary">{t("settingsRecorder.ring", { gb: gb(status.ring_bytes) })}</Text>
                        )}
                        {status.free_bytes != null && (
                            <Text type="secondary">{t("settingsRecorder.free", { gb: gb(status.free_bytes) })}</Text>
                        )}
                    </Space>
                )}
                {error && <Alert type="error" showIcon message={t("settingsRecorder.unreachable")}
                                 description={error} />}
            </Space>
            )}
        </Card>
    );
};
