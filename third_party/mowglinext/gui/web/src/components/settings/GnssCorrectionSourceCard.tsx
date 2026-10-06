import {useState} from "react";
import {Alert, Button, Card, Descriptions, Form, Input, InputNumber, Modal, Select, Space, Tag, Typography} from "antd";
import {ApiOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {ContentType} from "../../api/Api.ts";
import {useApi} from "../../hooks/useApi.ts";
import type {CorrectionSummary} from "../../utils/gpsStatus.ts";
import {
    correctionAgeLabel,
    correctionStateLabel,
    correctionTransportLabel,
} from "../../utils/gpsStatus.ts";
import {correctionToneTagColor} from "../gnss/gnssPresentation.ts";

const {Text, Paragraph} = Typography;

/** Region codes accepted by the LoRa pairing service; "" lets the driver derive it from the serial. */
const LORA_AREA_CODES = ["00", "01", "02", "03", "F"] as const;

export interface LoRaPairForm {
    sn: string;
    addr: number;
    channel: number;
    area: string;
}

interface LoRaPairResult {
    success: boolean;
    message?: string;
}

type PairOutcome = {type: "success" | "warning" | "error"; message: string} | null;

interface Props {
    summary: CorrectionSummary;
}

/**
 * Status and pairing for a LoRa (radio) RTK base station. Rendered by the
 * NTRIP/corrections settings section when corrections come over a radio link.
 */
export const GnssCorrectionSourceCard: React.FC<Props> = ({summary}) => {
    const {t} = useTranslation();
    const guiApi = useApi();
    const [form] = Form.useForm<LoRaPairForm>();
    const [open, setOpen] = useState(false);
    const [submitting, setSubmitting] = useState(false);
    const [outcome, setOutcome] = useState<PairOutcome>(null);

    const transport = correctionTransportLabel(summary);

    const submit = async () => {
        let values: LoRaPairForm;
        try {
            values = await form.validateFields();
        } catch {
            return; // antd shows the field errors inline
        }
        setSubmitting(true);
        setOutcome(null);
        try {
            const res = await guiApi.request<LoRaPairResult>({
                path: "/corrections/lora/pair",
                method: "POST",
                type: ContentType.Json,
                body: {...values, sn: values.sn.trim(), area: values.area ?? ""},
                format: "json",
            });
            setOutcome(res.data?.success
                ? {type: "success", message: t("corrections.lora.pairSuccess")}
                : {type: "warning", message: t("corrections.lora.pairRejected")});
            setOpen(false);
        } catch (err: unknown) {
            const e = err as {error?: {error?: string}; message?: string};
            setOutcome({
                type: "error",
                message: t("corrections.lora.pairFailed", {error: e?.error?.error ?? e?.message ?? String(err)}),
            });
            setOpen(false);
        } finally {
            setSubmitting(false);
        }
    };

    return (
        <Card size="small" style={{marginBottom: 16}} data-testid="lora-correction-card">
            <div style={{display: "flex", justifyContent: "space-between", alignItems: "center", gap: 8, flexWrap: "wrap"}}>
                <Text strong style={{fontSize: 14}}>
                    <ApiOutlined style={{marginRight: 6}}/>{t("corrections.lora.title")}
                </Text>
                <Button size="small" onClick={() => setOpen(true)}>{t("corrections.lora.pairButton")}</Button>
            </div>
            <Paragraph type="secondary" style={{marginTop: 8, marginBottom: 8, fontSize: 12}}>
                {t("corrections.lora.intro")}
            </Paragraph>
            <Descriptions size="small" column={{xs: 1, sm: 3}}>
                <Descriptions.Item label={t("corrections.flowLabel")}>
                    <Tag color={correctionToneTagColor(summary.tone)}>{correctionStateLabel(summary)}</Tag>
                </Descriptions.Item>
                <Descriptions.Item label={t("corrections.age")}>
                    {correctionAgeLabel(summary)}
                </Descriptions.Item>
                {transport && (
                    <Descriptions.Item label={t("corrections.transportLabel")}>
                        {transport}
                    </Descriptions.Item>
                )}
            </Descriptions>
            {outcome && (
                <Alert
                    type={outcome.type}
                    showIcon
                    closable
                    onClose={() => setOutcome(null)}
                    message={outcome.message}
                    style={{marginTop: 8}}
                />
            )}

            <Modal
                open={open}
                title={t("corrections.lora.pairTitle")}
                okText={t("corrections.lora.pairSubmit")}
                onOk={() => void submit()}
                confirmLoading={submitting}
                onCancel={() => setOpen(false)}
                destroyOnHidden
            >
                <Paragraph type="secondary" style={{fontSize: 12}}>{t("corrections.lora.pairIntro")}</Paragraph>
                <Form<LoRaPairForm> form={form} layout="vertical" initialValues={{area: "", channel: 1, addr: 0}}>
                    <Form.Item name="sn" label={t("corrections.lora.sn")}
                               rules={[{required: true, whitespace: true, max: 64}]}>
                        <Input autoComplete="off"/>
                    </Form.Item>
                    <Space wrap>
                        <Form.Item name="addr" label={t("corrections.lora.addr")}
                                   rules={[{required: true, type: "integer", min: 0, max: 65535}]}>
                            <InputNumber min={0} max={65535} precision={0}/>
                        </Form.Item>
                        <Form.Item name="channel" label={t("corrections.lora.channel")}
                                   rules={[{required: true, type: "integer", min: 1, max: 75}]}>
                            <InputNumber min={1} max={75} precision={0}/>
                        </Form.Item>
                        <Form.Item name="area" label={t("corrections.lora.area")}>
                            <Select
                                style={{minWidth: 220}}
                                options={[
                                    {value: "", label: t("corrections.lora.areaAuto")},
                                    ...LORA_AREA_CODES.map((code) => ({value: code, label: code})),
                                ]}
                            />
                        </Form.Item>
                    </Space>
                </Form>
            </Modal>
        </Card>
    );
};
