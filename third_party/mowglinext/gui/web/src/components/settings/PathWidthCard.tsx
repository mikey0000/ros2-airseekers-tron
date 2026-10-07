import React from "react";
import {Card, Col, Form, InputNumber, Row, Select, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {useGate} from "../../hooks/useProfileGates.ts";
import {
    deriveDefaultSwathWidth,
    PATH_WIDTH_FALLBACKS,
    SWATH_OVERLAP_RANGE,
    SWATH_WIDTH_RANGE,
} from "../../utils/pathWidth.ts";
import {SettingFieldLabel} from "./SettingFieldLabel.tsx";

const {Text, Paragraph} = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
    isOverridden?: (key: string) => boolean;
    hasDefault?: (key: string) => boolean;
    onReset?: (key: string) => void;
    defaults?: Record<string, any>;
};

/**
 * Settings -> Mowing (Tron, gate "feature:area_settings"): blade disc + swath
 * overlap -> the STANDARD path width every area uses unless it sets its own
 * (mower_map map_server_node, pushed live). Defaults come from the schema
 * (blade_disc_mm, swath_overlap_m, default_swath_width_m).
 */
export const PathWidthCard: React.FC<Props> = ({values, onChange, isOverridden, hasDefault, onReset, defaults}) => {
    const {t} = useTranslation();
    const enabled = useGate("feature:area_settings");
    if (!enabled) return null;
    const label = (key: string, text: string) => (
        <SettingFieldLabel settingKey={key} label={text}
                           overridden={isOverridden?.(key) ?? false}
                           canReset={hasDefault?.(key) ?? false} onReset={onReset}/>
    );
    const get = (k: keyof typeof PATH_WIDTH_FALLBACKS) => values[k] ?? defaults?.[k] ?? PATH_WIDTH_FALLBACKS[k];
    const disc = get("blade_disc_mm");
    const overlap = get("swath_overlap_m");
    const explicit = get("default_swath_width_m");
    const width = deriveDefaultSwathWidth(disc, overlap, explicit);
    const isExplicit = typeof explicit === "number" && explicit > 0;

    return (
        <Card size="small" title={t("settingsMowing.pathWidthTitle")} style={{marginBottom: 16}} data-testid="path-width-card">
            <Paragraph type="secondary" style={{fontSize: 12}}>{t("settingsMowing.pathWidthHint")}</Paragraph>
            <Form layout="vertical" size="small">
                <Row gutter={[16, 0]}>
                    <Col xs={24} sm={8}>
                        <Form.Item label={label("blade_disc_mm", t("settingsMowing.bladeDisc"))} tooltip={t("settingsMowing.bladeDiscTooltip")}>
                            <Select value={disc} onChange={(v) => onChange("blade_disc_mm", v)} style={{width: "100%"}}
                                    data-testid="blade-disc" virtual={false}
                                    options={[
                                        {value: 220, label: t("settingsMowing.bladeDisc220")},
                                        {value: 330, label: t("settingsMowing.bladeDisc330")},
                                    ]}/>
                        </Form.Item>
                    </Col>
                    <Col xs={12} sm={8}>
                        <Form.Item label={label("swath_overlap_m", t("settingsMowing.swathOverlapM"))} tooltip={t("settingsMowing.swathOverlapMTooltip")}>
                            <InputNumber value={overlap} onChange={(v) => onChange("swath_overlap_m", v ?? 0)}
                                         min={SWATH_OVERLAP_RANGE.min} max={SWATH_OVERLAP_RANGE.max}
                                         step={SWATH_OVERLAP_RANGE.step} precision={2}
                                         disabled={isExplicit} style={{width: "100%"}} addonAfter="m"/>
                        </Form.Item>
                    </Col>
                    <Col xs={12} sm={8}>
                        <Form.Item label={label("default_swath_width_m", t("settingsMowing.defaultSwathWidthOverride"))} tooltip={t("settingsMowing.defaultSwathWidthOverrideTooltip")}>
                            <InputNumber value={explicit} onChange={(v) => onChange("default_swath_width_m", v ?? 0)}
                                         min={0} max={SWATH_WIDTH_RANGE.max} step={0.01} precision={2}
                                         placeholder={t("settingsMowing.defaultSwathWidthAuto")}
                                         style={{width: "100%"}} addonAfter="m"/>
                        </Form.Item>
                    </Col>
                </Row>
            </Form>
            <Text data-testid="default-path-width">
                {t("settingsMowing.defaultPathWidth", {value: width.toFixed(2)})}{" "}
                <Text type="secondary">
                    {isExplicit ? t("settingsMowing.defaultPathWidthExplicit")
                        : t("settingsMowing.defaultPathWidthDerived", {disc, overlap: Number(overlap).toFixed(2)})}
                </Text>
            </Text>
        </Card>
    );
};
