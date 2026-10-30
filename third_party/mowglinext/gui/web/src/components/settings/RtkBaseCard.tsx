import {useEffect, useRef, useState} from "react";
import {Alert, App, Button, Card, Col, Descriptions, Form, Input, InputNumber, Row, Select, Space, Tag, Typography} from "antd";
import {ApiOutlined, ReloadOutlined, PoweroffOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useApi} from "../../hooks/useApi.ts";
import {pairRtkBase, readRtkBasePairing, resetRtkBase, useRtkBaseStatus} from "../../hooks/useRtkBase.ts";
import {
    RTK_BASE_MAX_CHANNEL,
    RTK_BASE_MIN_CHANNEL,
    RTK_BASE_REGIONS,
    RTK_BASE_UNIVERSAL_CODE,
    formatRtkBaseAddr,
    parseRtkBaseAddr,
    rtkBaseAreaForBand,
    rtkBaseBandLabelKey,
    rtkBaseNtripOwns,
    type RtkBasePairing,
} from "../../utils/rtkBase.ts";

const {Text, Paragraph} = Typography;

interface PairForm {
    addr: string;
    channel: number;
    area: string;
}

type Outcome = {type: "success" | "warning" | "error"; message: string} | null;

interface Props {
    /** The Settings value of Enable NTRIP (ntrip_enabled). */
    ntripEnabled: boolean;
}

/**
 * RTK base station (vendor LoRa base): shows the pairing stored on the mower's
 * RTK board and whether the base's corrections arrive, and pairs the board with
 * a base (address, channel, region) through /api/rtk-base/*. Same bytes as
 * scripts/rtk_pair.py; nothing here commands motion.
 */
export const RtkBaseCard: React.FC<Props> = ({ntripEnabled}) => {
    const {t} = useTranslation();
    const guiApi = useApi();
    const {modal} = App.useApp();
    const status = useRtkBaseStatus();
    const [form] = Form.useForm<PairForm>();
    const [busyAction, setBusyAction] = useState<"pair" | "read" | "reset" | null>(null);
    const [outcome, setOutcome] = useState<Outcome>(null);
    // Pairing returned by "Read from board", shown until the live status reports one.
    const [boardReadPairing, setBoardReadPairing] = useState<RtkBasePairing | null>(null);

    const storedPairing = status?.pairing ?? boardReadPairing;
    const ntripOwns = rtkBaseNtripOwns(status, ntripEnabled);

    // Pre-fill the form with the stored pairing once, unless the user already filled it.
    const formPrefilled = useRef(false);
    useEffect(() => {
        if (!storedPairing || formPrefilled.current || form.isFieldsTouched()) return;
        formPrefilled.current = true;
        form.setFieldsValue({
            addr: String(storedPairing.addr),
            channel: storedPairing.channel,
            area: rtkBaseAreaForBand(storedPairing.band) ?? (form.getFieldValue("area") as string),
        });
    }, [form, storedPairing]);

    const bandLabel = (pairing: RtkBasePairing) => {
        const key = rtkBaseBandLabelKey(pairing.band);
        return key ? t(key) : t("rtkBase.bandUnknown");
    };

    const fillUniversalCode = () => {
        formPrefilled.current = true;
        form.setFieldsValue({
            addr: String(RTK_BASE_UNIVERSAL_CODE.addr),
            channel: RTK_BASE_UNIVERSAL_CODE.channel,
        });
    };

    const pair = async () => {
        let values: PairForm;
        try {
            values = await form.validateFields();
        } catch {
            return; // antd shows the field errors inline
        }
        const addr = parseRtkBaseAddr(values.addr)!;
        const region = RTK_BASE_REGIONS.find((r) => r.area === values.area);
        modal.confirm({
            title: t("rtkBase.confirmTitle"),
            content: t("rtkBase.confirmBody", {
                addr: formatRtkBaseAddr(addr),
                channel: values.channel,
                region: region ? t(region.labelKey) : values.area,
            }),
            okText: t("rtkBase.pair"),
            onOk: async () => {
                setBusyAction("pair");
                setOutcome(null);
                try {
                    const res = await pairRtkBase(guiApi, addr, values.channel, values.area);
                    setOutcome(res.success
                        ? {type: "success", message: t("rtkBase.pairSuccess")}
                        : {type: "warning", message: t("rtkBase.pairRejected", {message: res.message ?? ""})});
                } catch (e) {
                    setOutcome({type: "error", message: t("rtkBase.actionFailed", {error: (e as Error).message})});
                } finally {
                    setBusyAction(null);
                }
            },
        });
    };

    const readFromBoard = async () => {
        setBusyAction("read");
        setOutcome(null);
        try {
            const res = await readRtkBasePairing(guiApi);
            if (res.ok && res.pairing) {
                setBoardReadPairing(res.pairing);
                setOutcome({type: "success", message: t("rtkBase.readSuccess", {
                    addr: formatRtkBaseAddr(res.pairing.addr), channel: res.pairing.channel,
                    band: bandLabel(res.pairing),
                })});
            } else {
                setOutcome({type: "warning", message: t("rtkBase.readNoAnswer", {message: res.message ?? ""})});
            }
        } catch (e) {
            setOutcome({type: "error", message: t("rtkBase.actionFailed", {error: (e as Error).message})});
        } finally {
            setBusyAction(null);
        }
    };

    const rebootBase = () => {
        modal.confirm({
            title: t("rtkBase.rebootConfirmTitle"),
            content: t("rtkBase.rebootConfirmBody"),
            okText: t("rtkBase.reboot"),
            onOk: async () => {
                setBusyAction("reset");
                setOutcome(null);
                try {
                    const res = await resetRtkBase(guiApi);
                    setOutcome(res.success
                        ? {type: "success", message: t("rtkBase.rebootSent")}
                        : {type: "warning", message: t("rtkBase.rebootRejected", {message: res.message ?? ""})});
                } catch (e) {
                    setOutcome({type: "error", message: t("rtkBase.actionFailed", {error: (e as Error).message})});
                } finally {
                    setBusyAction(null);
                }
            },
        });
    };

    const refStation = status?.ref_station;
    const refTag = refStation === "FINE"
        ? <Tag color="success">{t("rtkBase.refFine")}</Tag>
        : refStation === "FAIL"
            ? <Tag color="error">{t("rtkBase.refFail")}</Tag>
            : <Tag>{t("rtkBase.unknown")}</Tag>;

    return (
        <Card size="small" style={{marginBottom: 16}} data-testid="rtk-base-card">
            <Text strong style={{fontSize: 14}}>
                <ApiOutlined style={{marginRight: 6}}/>{t("rtkBase.title")}
            </Text>
            <Paragraph type="secondary" style={{marginTop: 4, marginBottom: 8, fontSize: 12}}>
                {t("rtkBase.intro")}
            </Paragraph>

            <Descriptions size="small" column={{xs: 1, sm: 2}} style={{marginBottom: 8}}>
                <Descriptions.Item label={t("rtkBase.storedPairing")}>
                    {storedPairing ? (
                        <span data-testid="rtk-base-stored">
                            {t("rtkBase.pairingSummary", {
                                addr: formatRtkBaseAddr(storedPairing.addr),
                                channel: storedPairing.channel,
                                band: bandLabel(storedPairing),
                            })}
                        </span>
                    ) : t("rtkBase.notReadYet")}
                </Descriptions.Item>
                <Descriptions.Item label={t("rtkBase.corrections")}>{refTag}</Descriptions.Item>
                <Descriptions.Item label={t("rtkBase.stationId")}>
                    {status?.station_id || "—"}
                </Descriptions.Item>
                <Descriptions.Item label={t("rtkBase.correctionAge")}>
                    {status?.diff_age_s != null ? `${status.diff_age_s.toFixed(1)} s` : "—"}
                </Descriptions.Item>
                <Descriptions.Item label={t("rtkBase.fixType")}>
                    {status?.fix_type ?? "—"}
                </Descriptions.Item>
            </Descriptions>

            {ntripOwns && (
                <Alert type="warning" showIcon style={{marginBottom: 12}} message={t("rtkBase.ntripOwns")}/>
            )}

            <Form<PairForm> form={form} layout="vertical" size="small"
                            initialValues={{area: RTK_BASE_REGIONS[0].area}}>
                <Row gutter={[12, 0]}>
                    <Col xs={24} sm={8}>
                        <Form.Item name="addr" label={t("rtkBase.address")} extra={t("rtkBase.addressHelp")}
                                   rules={[{
                                       validator: (_, v: string | undefined) => parseRtkBaseAddr(v) !== undefined
                                           ? Promise.resolve()
                                           : Promise.reject(new Error(t("rtkBase.addressInvalid"))),
                                   }]}>
                            <Input autoComplete="off" inputMode="text" placeholder="38183 / 0x9527"/>
                        </Form.Item>
                    </Col>
                    <Col xs={24} sm={6}>
                        <Form.Item name="channel" label={t("rtkBase.channel")}
                                   rules={[{
                                       required: true, type: "integer",
                                       min: RTK_BASE_MIN_CHANNEL, max: RTK_BASE_MAX_CHANNEL,
                                       message: t("rtkBase.channelInvalid"),
                                   }]}>
                            <InputNumber min={RTK_BASE_MIN_CHANNEL} max={RTK_BASE_MAX_CHANNEL} precision={0}
                                         style={{width: "100%"}}/>
                        </Form.Item>
                    </Col>
                    <Col xs={24} sm={10}>
                        <Form.Item name="area" label={t("rtkBase.region.label")}>
                            <Select options={RTK_BASE_REGIONS.map((r) => ({value: r.area, label: t(r.labelKey)}))}/>
                        </Form.Item>
                    </Col>
                </Row>
            </Form>

            <Space wrap size={[8, 8]}>
                <Button type="primary" onClick={() => void pair()} loading={busyAction === "pair"}
                        disabled={busyAction !== null && busyAction !== "pair"}>
                    {t("rtkBase.pair")}
                </Button>
                <Button onClick={fillUniversalCode}>{t("rtkBase.useUniversal")}</Button>
                <Button icon={<ReloadOutlined/>} onClick={() => void readFromBoard()}
                        loading={busyAction === "read"}>
                    {t("rtkBase.readFromBoard")}
                </Button>
                <Button danger icon={<PoweroffOutlined/>} onClick={rebootBase}
                        loading={busyAction === "reset"}>
                    {t("rtkBase.reboot")}
                </Button>
            </Space>

            {outcome && (
                <Alert type={outcome.type} showIcon closable onClose={() => setOutcome(null)}
                       message={outcome.message} style={{marginTop: 12}}/>
            )}

            <Paragraph type="secondary" style={{marginTop: 12, marginBottom: 0, fontSize: 12}}>
                {t("rtkBase.help")}
            </Paragraph>
        </Card>
    );
};
