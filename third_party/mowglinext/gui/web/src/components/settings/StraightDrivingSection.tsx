import React from "react";
import {Alert, Card, Col, Form, InputNumber, Row, Switch, Typography} from "antd";
import {useTranslation} from "react-i18next";

const {Paragraph} = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

// Straight-line driving (profile gate "settings:straight_driving", Tron only):
// mower_control cmd_vel_slew shapes the commanded angular.z before the MCU.
// The six keys are bound live (pkg/api/param_bindings.go) and read by the node
// at startup; schema defaults are neutral (everything off).
const num = (v: unknown, fallback: number) => (typeof v === "number" && Number.isFinite(v) ? v : fallback);

export const StraightDrivingSection: React.FC<Props> = ({values, onChange}) => {
    const {t} = useTranslation();
    const hold = values.heading_hold === true;

    return (
        <div>
            <Alert type="info" showIcon style={{marginBottom: 16}} message={t("settingsStraightDriving.intro")}/>

            <Card size="small" title={t("settingsStraightDriving.trimCard")} style={{marginBottom: 16}}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item label={t("settingsStraightDriving.trimLabel")}
                                       tooltip={t("settingsStraightDriving.trimTooltip")}>
                                <InputNumber
                                    value={num(values.angular_trim_radps, 0)}
                                    onChange={(v) => onChange("angular_trim_radps", v ?? 0)}
                                    min={-0.2} max={0.2} step={0.005} precision={3}
                                    style={{width: "100%"}} addonAfter="rad/s"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item label={t("settingsStraightDriving.deadbandLabel")}
                                       tooltip={t("settingsStraightDriving.deadbandTooltip")}>
                                <InputNumber
                                    value={num(values.angular_deadband_radps, 0)}
                                    onChange={(v) => onChange("angular_deadband_radps", v ?? 0)}
                                    min={0} max={0.3} step={0.01} precision={3}
                                    style={{width: "100%"}} addonAfter="rad/s"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            <Card size="small" title={t("settingsStraightDriving.holdCard")} style={{marginBottom: 16}}>
                <Paragraph type="secondary" style={{marginTop: 0}}>
                    {t("settingsStraightDriving.holdDescription")}
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Form.Item label={t("settingsStraightDriving.holdEnable")}>
                        <Switch checked={hold} onChange={(v) => onChange("heading_hold", v)}/>
                    </Form.Item>
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item label={t("settingsStraightDriving.kpLabel")}
                                       tooltip={t("settingsStraightDriving.kpTooltip")}>
                                <InputNumber
                                    value={num(values.heading_hold_kp, 1.0)}
                                    onChange={(v) => onChange("heading_hold_kp", v ?? 1.0)}
                                    min={0} max={5} step={0.1} precision={2}
                                    disabled={!hold} style={{width: "100%"}}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item label={t("settingsStraightDriving.kdLabel")}
                                       tooltip={t("settingsStraightDriving.kdTooltip")}>
                                <InputNumber
                                    value={num(values.heading_hold_kd, 0.1)}
                                    onChange={(v) => onChange("heading_hold_kd", v ?? 0.1)}
                                    min={0} max={1} step={0.05} precision={2}
                                    disabled={!hold} style={{width: "100%"}}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item label={t("settingsStraightDriving.maxLabel")}
                                       tooltip={t("settingsStraightDriving.maxTooltip")}>
                                <InputNumber
                                    value={num(values.heading_hold_max_radps, 0.15)}
                                    onChange={(v) => onChange("heading_hold_max_radps", v ?? 0.15)}
                                    min={0} max={0.5} step={0.01} precision={2}
                                    disabled={!hold} style={{width: "100%"}} addonAfter="rad/s"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
