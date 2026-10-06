import React from "react";
import { Card, Descriptions, Space, Tag, Typography } from "antd";
import { useTranslation } from "react-i18next";
import { GnssStatus, GnssStatusConstants } from "../../types/ros.ts";
import {
    correctionAgeLabel,
    correctionSummaryLabel,
    correctionTransportLabel,
    deriveCorrectionSummary,
    deriveGpsStatus,
    gnssReceiverLabel,
    hasGnssCapability,
} from "../../utils/gpsStatus.ts";
import {
    correctionToneTagColor,
    liveStatusTagColor,
    rtkModeTagColor,
} from "./gnssPresentation.ts";
import { normalizeGnssString } from "./gnssFormatting.ts";

const { Text } = Typography;

type Props = {
    gnssStatus: GnssStatus | undefined;
    selectedReceiverFamily: unknown;
};

export const GnssLiveStatusSummaryCard: React.FC<Props> = ({
    gnssStatus,
    selectedReceiverFamily,
}) => {
    const { t } = useTranslation();
    const liveStatus = deriveGpsStatus(gnssStatus);
    const unknownLabel = t("settingsGnssLiveStatus.unknown");
    const detectedReceiver = gnssReceiverLabel(gnssStatus);
    const receiverFamily = normalizeGnssString(selectedReceiverFamily) || "auto";
    const receiverLabel = detectedReceiver !== "GNSS" ? detectedReceiver : receiverFamily;
    const backendLabel = normalizeGnssString(gnssStatus?.backend) || unknownLabel;
    const corrections = deriveCorrectionSummary(gnssStatus);
    const correctionsTitle = corrections.origin === "stream"
        ? t("settingsGnssLiveStatus.correctionStream")
        : t("corrections.label");
    const correctionsDetail = corrections.origin === "typed" && corrections.source !== "none"
        ? [correctionTransportLabel(corrections), `${t("corrections.age")} ${correctionAgeLabel(corrections)}`]
            .filter(Boolean)
            .join(" · ")
        : undefined;
    const rtkModeLabel = (() => {
        if (!hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_RTK_MODE)) {
            return unknownLabel;
        }
        switch (gnssStatus?.rtk_mode) {
            case GnssStatusConstants.RTK_MODE_NONE:
                return t("settingsGnssLiveStatus.rtkModeNone");
            case GnssStatusConstants.RTK_MODE_FLOAT:
                return t("gpsStatus.rtkFloat");
            case GnssStatusConstants.RTK_MODE_FIXED:
                return t("gpsStatus.rtkFixed");
            case GnssStatusConstants.RTK_MODE_UNKNOWN:
            default:
                return unknownLabel;
        }
    })();

    return (
        <Card size="small" title={t("settingsGnssLiveStatus.cardTitle")} style={{ marginBottom: 16 }}>
            <Space direction="vertical" size={12} style={{ width: "100%" }}>
                <Space wrap size={[12, 8]}>
                    <Space size={4}>
                        <Text type="secondary">{t("settingsGnssLiveStatus.liveStatus")}</Text>
                        <Tag color={liveStatusTagColor(liveStatus.fixType)}>{liveStatus.label}</Tag>
                    </Space>
                    <Space size={4}>
                        <Text type="secondary">{t("settingsGnssLiveStatus.rtkMode")}</Text>
                        <Tag color={rtkModeTagColor(gnssStatus?.rtk_mode)}>{rtkModeLabel}</Tag>
                    </Space>
                    <Space size={4}>
                        <Text type="secondary">{correctionsTitle}</Text>
                        <Tag color={correctionToneTagColor(corrections.tone)} data-testid="gnss-corrections-tag">
                            {correctionSummaryLabel(corrections)}
                        </Tag>
                    </Space>
                </Space>
                {correctionsDetail && (
                    <Text type="secondary" style={{ fontSize: 12 }} data-testid="gnss-corrections-detail">
                        {correctionsDetail}
                    </Text>
                )}

                <Descriptions size="small" column={{ xs: 1, sm: 2 }}>
                    <Descriptions.Item label={t("settingsGnssLiveStatus.detectedReceiver")}>
                        {receiverLabel}
                    </Descriptions.Item>
                    <Descriptions.Item label={t("settingsGnssLiveStatus.backend")}>
                        {backendLabel}
                    </Descriptions.Item>
                </Descriptions>
            </Space>
        </Card>
    );
};
