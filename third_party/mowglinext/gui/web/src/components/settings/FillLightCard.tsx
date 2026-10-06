import React, { useCallback, useEffect, useState } from "react";
import { Alert, Button, Card, Segmented, Space, Tag, Typography } from "antd";
import { BulbOutlined, ReloadOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useApi } from "../../hooks/useApi.ts";

const { Paragraph, Text } = Typography;

type FillLightMode = "off" | "on" | "auto";

type FillLightStatus = {
    mode?: FillLightMode;
    on?: boolean;
    requested?: boolean;
    available?: boolean;
    error?: string;
    dark?: boolean;
    sun_elevation_deg?: number | null;
    mission_state?: string;
    pwm_chip?: string;
};

function parseStatus(data: unknown): FillLightStatus | null {
    const msg = (data as { message?: string } | undefined)?.message;
    if (!msg) return null;
    try {
        return JSON.parse(msg) as FillLightStatus;
    } catch {
        return null;
    }
}

/**
 * Tron fill light (night-mowing lamp) on the host PWM, served by fill_light_node.
 * Off / On / Auto (on while mowing or in transit when the sun is down at the datum).
 */
export const FillLightCard: React.FC = () => {
    const { t } = useTranslation();
    const guiApi = useApi();
    const [status, setStatus] = useState<FillLightStatus | null>(null);
    const [busy, setBusy] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const call = useCallback(async (command: string, body: Record<string, unknown>) => {
        setBusy(true);
        setError(null);
        try {
            const res = await guiApi.mowglinext.callCreate(command, body);
            if (res.error) throw new Error((res.error as { error?: string }).error ?? "error");
            setStatus(parseStatus(res.data));
        } catch (e) {
            setError(e instanceof Error ? e.message : String(e));
        } finally {
            setBusy(false);
        }
    }, [guiApi]);

    const refresh = useCallback(() => call("fill_light_state", {}), [call]);
    useEffect(() => { void refresh(); }, [refresh]);

    const setMode = (mode: FillLightMode) => call("fill_light", { mode });

    return (
        <Card size="small" title={<Space><BulbOutlined />{t("settingsFillLight.title")}</Space>}
              extra={<Button size="small" icon={<ReloadOutlined />} onClick={refresh} loading={busy} />}
              style={{ marginBottom: 16 }}>
            <Paragraph type="secondary" style={{ marginBottom: 12 }}>
                {t("settingsFillLight.description")}
            </Paragraph>
            <Space direction="vertical" style={{ width: "100%" }}>
                <Segmented<FillLightMode>
                    value={status?.mode ?? "off"}
                    disabled={busy}
                    onChange={(v) => void setMode(v)}
                    options={[
                        { label: t("settingsFillLight.modeOff"), value: "off" },
                        { label: t("settingsFillLight.modeOn"), value: "on" },
                        { label: t("settingsFillLight.modeAuto"), value: "auto" },
                    ]}
                />
                {status && (
                    <Space wrap>
                        <Text>{t("settingsFillLight.state")}:</Text>
                        <Tag color={status.on ? "gold" : "default"}>
                            {status.on ? t("settingsFillLight.lit") : t("settingsFillLight.dark")}
                        </Tag>
                        {status.sun_elevation_deg != null && (
                            <Text type="secondary">
                                {t("settingsFillLight.sun", { deg: status.sun_elevation_deg.toFixed(1) })}
                            </Text>
                        )}
                    </Space>
                )}
                {status && status.available === false && (
                    <Alert type="warning" showIcon message={t("settingsFillLight.notInstalled")}
                           description={status.pwm_chip} />
                )}
                {status?.error && <Alert type="error" showIcon message={status.error} />}
                {error && <Alert type="error" showIcon message={t("settingsFillLight.unreachable")}
                                 description={error} />}
            </Space>
        </Card>
    );
};
