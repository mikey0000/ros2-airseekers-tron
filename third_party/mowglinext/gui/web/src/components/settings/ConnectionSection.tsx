import {useEffect, useState} from "react";
import {Alert, App, Button, Card, Descriptions, Input, Popconfirm, Segmented, Space, Tag, Typography} from "antd";
import {DeleteOutlined, ReloadOutlined, SaveOutlined} from "@ant-design/icons";
import {Bluetooth, Wifi} from "lucide-react";
import {useTranslation} from "react-i18next";
import {
    type ConnectionMode,
    type ConnectionState,
    getConnectionManager,
    useConnectionState,
} from "../../transport/connectionManager.ts";

const {Text, Paragraph} = Typography;

const LINK_COLOR: Record<ConnectionState["ble"]["link"], string> = {
    idle: "default",
    connecting: "processing",
    connected: "success",
    reconnecting: "warning",
    unavailable: "default",
};

const SESSION_COLOR: Record<ConnectionState["ble"]["session"], string> = {
    idle: "default",
    handshaking: "processing",
    ready: "success",
    unauthenticated: "error",
    backend_unavailable: "warning",
    dead: "warning",
};

function formatBytes(n: number): string {
    if (n < 1024) return `${n} B`;
    if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} KB`;
    return `${(n / 1024 / 1024).toFixed(2)} MB`;
}

/** Bluetooth byte counters, polled once a second while the section is open. */
function useBleCounters(): {bytesIn: number; bytesOut: number} {
    const [counters, setCounters] = useState(() => getConnectionManager().getBleCounters());
    useEffect(() => {
        const timer = setInterval(() => setCounters(getConnectionManager().getBleCounters()), 1000);
        return () => clearInterval(timer);
    }, []);
    return counters;
}

/** Settings → Connection: Wi-Fi / Bluetooth switch, pairing, link details and the BLE token. */
export function ConnectionSection() {
    const {t} = useTranslation();
    const {message} = App.useApp();
    const state = useConnectionState();
    const manager = getConnectionManager();
    const [busy, setBusy] = useState(false);

    const run = async (action: () => Promise<void>) => {
        setBusy(true);
        try {
            await action();
        } catch (error) {
            // The user closing the device chooser is not an error worth reporting.
            if (!(error instanceof DOMException && error.name === "NotFoundError")) {
                void message.error(t("connection.pairFailed", {error: error instanceof Error ? error.message : String(error)}));
            }
        } finally {
            setBusy(false);
        }
    };

    const networkKey = state.networkAlive === null ? "unknown" : state.networkAlive ? "up" : "down";

    return (
        <div data-testid="connection-section">
            <Card size="small" title={t("connection.linkTitle")} style={{marginBottom: 16}}>
                <Segmented<ConnectionMode>
                    block
                    value={state.mode}
                    disabled={busy}
                    onChange={(mode) => void run(() => manager.switchTo(mode))}
                    options={[
                        {value: "wifi", label: <Space size={6}><Wifi size={14} aria-hidden/>{t("connection.wifi")}</Space>},
                        {value: "ble", label: <Space size={6}><Bluetooth size={14} aria-hidden/>{t("connection.bluetooth")}</Space>},
                    ]}
                />
                <Space wrap style={{marginTop: 12}}>
                    <Text type="secondary" style={{fontSize: 12}}>{t("connection.wifiStatus")}</Text>
                    <Tag color={state.networkAlive ? "success" : state.networkAlive === false ? "error" : "default"} data-testid="connection-network">
                        {t(`connection.network.${networkKey}`)}
                    </Tag>
                </Space>
                <Paragraph type="secondary" style={{fontSize: 12, margin: "8px 0 0"}}>{t("connection.modeHint")}</Paragraph>
            </Card>

            <BluetoothCard state={state} busy={busy} run={run}/>
            <TokenCard state={state}/>
        </div>
    );
}

interface BluetoothCardProps {
    state: ConnectionState;
    busy: boolean;
    run: (action: () => Promise<void>) => Promise<void>;
}

function BluetoothCard({state, busy, run}: BluetoothCardProps) {
    const {t} = useTranslation();
    const manager = getConnectionManager();
    const counters = useBleCounters();
    const {ble} = state;

    if (state.bleSupport !== "ok") {
        return (
            <Card size="small" title={t("connection.bluetoothTitle")} style={{marginBottom: 16}}>
                <Alert
                    type="info"
                    showIcon
                    data-testid="connection-unsupported"
                    message={t("connection.unsupportedTitle")}
                    description={t(state.bleSupport === "insecure" ? "connection.insecure" : "connection.unsupported")}
                />
            </Card>
        );
    }

    const deviceName = ble.deviceName ?? (state.deviceId ? state.deviceId : null);
    const notInRange = state.mode === "ble" && state.deviceId !== null && ble.link === "idle";

    return (
        <Card size="small" title={t("connection.bluetoothTitle")} style={{marginBottom: 16}}>
            {notInRange && <Alert type="warning" showIcon style={{marginBottom: 12}} message={t("connection.notInRange")}/>}
            <Descriptions size="small" column={1} style={{marginBottom: 12}}>
                <Descriptions.Item label={t("connection.device")}>
                    {deviceName ?? <Text type="secondary">{t("connection.noDevice")}</Text>}
                </Descriptions.Item>
                <Descriptions.Item label={t("connection.linkState")}>
                    <Tag color={LINK_COLOR[ble.link]} data-testid="connection-link-state">{t(`connection.link.${ble.link}`)}</Tag>
                </Descriptions.Item>
                <Descriptions.Item label={t("connection.sessionState")}>
                    <Tag color={SESSION_COLOR[ble.session]}>{t(`connection.session.${ble.session}`)}</Tag>
                    {ble.host && <Text type="secondary" style={{fontSize: 12}}>{ble.host}</Text>}
                </Descriptions.Item>
                <Descriptions.Item label={t("connection.rtt")}>{ble.rttMs !== null ? `${ble.rttMs} ms` : "–"}</Descriptions.Item>
                <Descriptions.Item label={t("connection.received")}>{formatBytes(counters.bytesIn)}</Descriptions.Item>
                <Descriptions.Item label={t("connection.sent")}>{formatBytes(counters.bytesOut)}</Descriptions.Item>
            </Descriptions>
            <Space wrap>
                <Button type="primary" icon={<Bluetooth size={14} aria-hidden/>} loading={busy} onClick={() => void run(() => manager.pairNew())}>
                    {t("connection.connectNew")}
                </Button>
                {notInRange && (
                    <Button icon={<ReloadOutlined/>} disabled={busy} onClick={() => void run(() => manager.switchTo("ble"))}>
                        {t("connection.reconnect")}
                    </Button>
                )}
                {state.deviceId !== null && (
                    <Popconfirm
                        title={t("connection.forgetConfirm")}
                        okText={t("connection.forget")}
                        okButtonProps={{danger: true}}
                        onConfirm={() => void run(() => manager.forgetDevice())}
                    >
                        <Button danger icon={<DeleteOutlined/>} disabled={busy}>{t("connection.forget")}</Button>
                    </Popconfirm>
                )}
            </Space>
        </Card>
    );
}

function TokenCard({state}: {state: ConnectionState}) {
    const {t} = useTranslation();
    const {message} = App.useApp();
    const [draft, setDraft] = useState(state.token ?? "");

    // Follow a token fetched in the background unless the user is editing a different value.
    const [lastStored, setLastStored] = useState(state.token);
    if (state.token !== lastStored) {
        setLastStored(state.token);
        setDraft(state.token ?? "");
    }

    const dirty = draft.trim() !== (state.token ?? "");
    const save = () => {
        getConnectionManager().setToken(draft);
        void message.success(t("connection.tokenSaved"));
    };

    return (
        <Card size="small" title={t("connection.tokenTitle")} style={{marginBottom: 16}}>
            {state.ble.session === "unauthenticated" && (
                <Alert type="error" showIcon style={{marginBottom: 8}} message={t("connection.tokenRejected")}/>
            )}
            {!state.token && <Alert type="warning" showIcon style={{marginBottom: 8}} message={t("connection.tokenMissing")}/>}
            <Paragraph type="secondary" style={{fontSize: 12, margin: "0 0 8px"}}>{t("connection.tokenHint")}</Paragraph>
            <Space.Compact style={{width: "100%"}}>
                <Input
                    data-testid="connection-token"
                    value={draft}
                    placeholder={t("connection.tokenPlaceholder")}
                    onChange={(e) => setDraft(e.target.value)}
                    onPressEnter={() => dirty && save()}
                    autoComplete="off"
                    spellCheck={false}
                    style={{fontFamily: "monospace"}}
                />
                <Button icon={<SaveOutlined/>} disabled={!dirty} onClick={save}>{t("connection.tokenSave")}</Button>
            </Space.Compact>
            {state.token && (
                <Text copyable={{text: state.token}} type="secondary" style={{fontSize: 12, display: "block", marginTop: 8, wordBreak: "break-all"}}>
                    {state.token}
                </Text>
            )}
        </Card>
    );
}
