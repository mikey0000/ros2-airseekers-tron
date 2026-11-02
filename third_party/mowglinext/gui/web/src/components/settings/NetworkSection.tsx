// Settings > Network: the mower's own Wi-Fi, driven through the host's
// NetworkManager (pkg/providers/network_wifi.go). Live link numbers come from
// /wifi/status (mower_control/wifi_monitor, topic key "wifiStatus"); profiles,
// scans and the band / power-save knobs from /api/network/wifi/*.
import {useCallback, useEffect, useRef, useState} from "react";
import {
    Alert, App, Button, Card, Checkbox, Descriptions, Empty, Input, List, Modal, Popconfirm, Radio, Space,
    Spin, Switch, Tag, Typography,
} from "antd";
import {DeleteOutlined, LockOutlined, ReloadOutlined, UnlockOutlined} from "@ant-design/icons";
import {Wifi, WifiHigh, WifiLow, WifiZero} from "lucide-react";
import {useTranslation} from "react-i18next";
import {useSearchParams} from "react-router-dom";
import {useTopic} from "../../hooks/useTopic.ts";
import {useProfileGates} from "../../hooks/useProfileGates.ts";
import {freqToChannel, selectWifiStatus, type WifiStatusView} from "../WifiChip.tsx";
import {
    bandOf,
    connectWifi,
    fetchWifiStatus,
    forgetWifi,
    groupNetworks,
    isJoinable,
    needsPassword,
    scanWifi,
    setWifiBand,
    setWifiPowersave,
    signalBars,
    WifiApiError,
    type WifiBand,
    type WifiHostStatus,
    type WifiNetworkGroup,
    type WifiSavedNetwork,
    type WifiSecurity,
} from "./networkWifi.ts";

const {Text, Paragraph, Link} = Typography;

const STATUS_POLL_MS = 5000;
/** After a join / band change NetworkManager needs a moment before the status means anything. */
const SETTLE_MS = 3000;

const BAR_ICONS = [WifiZero, WifiLow, WifiLow, WifiHigh, Wifi] as const;

function SignalIcon({strength}: {strength: number}) {
    const {t} = useTranslation();
    const bars = signalBars(strength);
    const Icon = BAR_ICONS[bars];
    return (
        <span role="img" aria-label={t("networkWifi.signal", {bars, strength})} data-bars={bars} style={{display: "inline-flex"}}>
            <Icon size={18} aria-hidden/>
        </span>
    );
}

function SecurityTag({security}: {security: WifiSecurity}) {
    const {t} = useTranslation();
    const open = security === "open" || security === "owe";
    return (
        <Tag icon={open ? <UnlockOutlined/> : <LockOutlined/>} color={open ? "default" : "blue"} data-testid="wifi-security">
            {t(`networkWifi.security.${security}`)}
        </Tag>
    );
}

/** GET /status every few seconds while the section is open. */
function useHostStatus() {
    const [status, setStatus] = useState<WifiHostStatus | null>(null);
    const [error, setError] = useState<string | null>(null);
    const [tick, setTick] = useState(0);
    const refresh = useCallback(() => setTick((n) => n + 1), []);
    useEffect(() => {
        const controller = new AbortController();
        const load = () => {
            fetchWifiStatus(controller.signal)
                .then((s) => {
                    setStatus(s);
                    setError(null);
                })
                .catch((e: unknown) => {
                    if (controller.signal.aborted) return;
                    setError(e instanceof Error ? e.message : String(e));
                });
        };
        load();
        const timer = setInterval(load, STATUS_POLL_MS);
        return () => {
            controller.abort();
            clearInterval(timer);
        };
    }, [tick]);
    return {status, error, refresh};
}

export function NetworkSection() {
    const {t} = useTranslation();
    const {isVisible, loading} = useProfileGates();
    if (!loading && !isVisible("feature:wifi_setup")) {
        return <Alert type="info" showIcon data-testid="wifi-not-on-robot" message={t("networkWifi.notOnRobot")}/>;
    }
    return <WifiSetup/>;
}

function WifiSetup() {
    const {t} = useTranslation();
    const {message} = App.useApp();
    const [, setSearchParams] = useSearchParams();
    const {status, error, refresh} = useHostStatus();
    const {data: link} = useTopic<WifiStatusView | null>("wifiStatus", null, {select: selectWifiStatus, throttleMs: 1000});
    const [busy, setBusy] = useState(false);
    const settleTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
    useEffect(() => () => {
        if (settleTimer.current) clearTimeout(settleTimer.current);
    }, []);

    const openBluetoothGuide = () => setSearchParams((current) => {
        const next = new URLSearchParams(current);
        next.set("section", "connection");
        return next;
    }, {replace: true});

    const run = async (action: () => Promise<unknown>, success: string): Promise<boolean> => {
        setBusy(true);
        try {
            await action();
            void message.success(success);
            refresh();
            if (settleTimer.current) clearTimeout(settleTimer.current);
            settleTimer.current = setTimeout(refresh, SETTLE_MS);
            return true;
        } catch (e) {
            const text = e instanceof WifiApiError && e.status === 401
                ? t("networkWifi.tokenRejected")
                : e instanceof Error ? e.message : String(e);
            void message.error(t("networkWifi.actionFailed", {error: text}));
            return false;
        } finally {
            setBusy(false);
        }
    };

    if (!status && !error) return <Spin style={{display: "block", margin: "48px auto"}}/>;

    const banner = (
        <Alert
            type="warning"
            showIcon
            style={{marginBottom: 16}}
            data-testid="wifi-drop-warning"
            message={t("networkWifi.dropWarning")}
            description={<Link onClick={openBluetoothGuide} data-testid="wifi-ble-guide-link">{t("networkWifi.bleGuideLink")}</Link>}
        />
    );

    if (!status || !status.available) {
        return (
            <div data-testid="network-section">
                <LinkCard status={status} link={link}/>
                <Alert
                    type="info"
                    showIcon
                    data-testid="wifi-unavailable"
                    message={t("networkWifi.unavailableTitle")}
                    description={<>
                        <Paragraph style={{margin: 0}}>{t("networkWifi.unavailable")}</Paragraph>
                        {(status?.reason ?? error) && <Text type="secondary" style={{fontSize: 12}}>{status?.reason ?? error}</Text>}
                    </>}
                />
            </div>
        );
    }

    const active = status.connection;
    return (
        <div data-testid="network-section">
            {banner}
            <LinkCard status={status} link={link}/>
            {active && <LinkSettingsCard active={active} busy={busy} run={run}/>}
            <NetworksCard busy={busy} run={run} saved={status.saved}/>
            <SavedCard saved={status.saved} busy={busy} run={run}/>
        </div>
    );
}

type Run = (action: () => Promise<unknown>, success: string) => Promise<boolean>;

function LinkCard({status, link}: {status: WifiHostStatus | null; link: WifiStatusView | null}) {
    const {t} = useTranslation();
    const connected = link?.connected ?? status?.connected ?? false;
    const ssid = link?.ssid || status?.ssid || "";
    const bssid = link?.bssid || status?.bssid || "";
    const freq = link?.freqMhz || status?.freq_mhz || 0;
    const band = bandOf(freq);
    let signal = "–";
    if (link?.signalDbm != null) signal = `${Math.round(link.signalDbm)} dBm`;
    else if (status?.strength) signal = `${status.strength} %`;
    return (
        <Card size="small" title={t("networkWifi.linkTitle")} style={{marginBottom: 16}} data-testid="wifi-link-card">
            <Descriptions size="small" column={1}>
                <Descriptions.Item label={t("networkWifi.state")}>
                    <Tag color={connected ? "success" : "default"} data-testid="wifi-link-state">
                        {connected ? t("networkWifi.connected") : t("networkWifi.disconnected")}
                    </Tag>
                    {status?.interface && <Text type="secondary" style={{fontSize: 12}}>{status.interface}</Text>}
                </Descriptions.Item>
                <Descriptions.Item label={t("networkWifi.network")}>
                    <span data-testid="wifi-link-ssid">{ssid || "–"}</span>
                </Descriptions.Item>
                <Descriptions.Item label={t("networkWifi.signalLabel")}>{signal}</Descriptions.Item>
                <Descriptions.Item label={t("networkWifi.channel")}>
                    {freq ? t("networkWifi.channelValue", {channel: freqToChannel(freq), band: band ?? "?", freq: Math.round(freq)}) : "–"}
                </Descriptions.Item>
                <Descriptions.Item label={t("networkWifi.accessPoint")}>
                    <Text style={{fontFamily: "monospace", fontSize: 12}}>{bssid || "–"}</Text>
                </Descriptions.Item>
                {status?.ip4 && status.ip4.length > 0 && (
                    <Descriptions.Item label={t("networkWifi.address")}>{status.ip4.join(", ")}</Descriptions.Item>
                )}
                {link && (
                    <Descriptions.Item label={t("networkWifi.bitrate")}>
                        {t("networkWifi.bitrateValue", {tx: link.txMbps.toFixed(0), rx: link.rxMbps.toFixed(0)})}
                    </Descriptions.Item>
                )}
                {link && <Descriptions.Item label={t("networkWifi.roams")}>{link.roamCount}</Descriptions.Item>}
            </Descriptions>
        </Card>
    );
}

function LinkSettingsCard({active, busy, run}: {active: WifiSavedNetwork; busy: boolean; run: Run}) {
    const {t} = useTranslation();
    const {modal} = App.useApp();
    const powersaveOn = active.powersave !== "disabled";

    const changeBand = (band: WifiBand) => {
        modal.confirm({
            title: t("networkWifi.reconnectTitle"),
            content: t("networkWifi.reconnectBody", {ssid: active.ssid}),
            okText: t("networkWifi.apply"),
            cancelText: t("networkWifi.cancel"),
            onOk: () => run(() => setWifiBand(active.uuid, band), t("networkWifi.bandSaved")),
        });
    };
    const changePowersave = (enabled: boolean) => {
        modal.confirm({
            title: t("networkWifi.reconnectTitle"),
            content: t("networkWifi.reconnectBody", {ssid: active.ssid}),
            okText: t("networkWifi.apply"),
            cancelText: t("networkWifi.cancel"),
            onOk: () => run(() => setWifiPowersave(active.uuid, enabled), t("networkWifi.powersaveSaved")),
        });
    };

    return (
        <Card size="small" title={t("networkWifi.settingsTitle", {ssid: active.ssid})} style={{marginBottom: 16}}>
            <Text strong style={{display: "block", marginBottom: 6}}>{t("networkWifi.bandTitle")}</Text>
            <Radio.Group
                data-testid="wifi-band"
                value={active.band}
                disabled={busy}
                optionType="button"
                onChange={(e) => changeBand(e.target.value as WifiBand)}
                options={[
                    {value: "", label: t("networkWifi.bandAuto")},
                    {value: "bg", label: t("networkWifi.band24")},
                    {value: "a", label: t("networkWifi.band5")},
                ]}
            />
            <Paragraph type="secondary" style={{fontSize: 12, margin: "6px 0 16px"}}>{t("networkWifi.bandHint")}</Paragraph>
            <Space align="center">
                <Switch
                    data-testid="wifi-powersave"
                    checked={powersaveOn}
                    disabled={busy}
                    onChange={changePowersave}
                />
                <Text>{t("networkWifi.powersave")}</Text>
                {(active.powersave === "default" || active.powersave === "ignore") && (
                    <Text type="secondary" style={{fontSize: 12}}>{t("networkWifi.powersaveSystem")}</Text>
                )}
            </Space>
            <Paragraph type="secondary" style={{fontSize: 12, margin: "6px 0 0"}}>{t("networkWifi.powersaveHint")}</Paragraph>
        </Card>
    );
}

function NetworksCard({busy, run, saved}: {busy: boolean; run: Run; saved: WifiSavedNetwork[]}) {
    const {t} = useTranslation();
    const [groups, setGroups] = useState<WifiNetworkGroup[] | null>(null);
    const [scanning, setScanning] = useState(false);
    const [scanError, setScanError] = useState<string | null>(null);
    const [joining, setJoining] = useState<WifiNetworkGroup | null>(null);

    const load = useCallback(async (rescan: boolean, signal?: AbortSignal) => {
        setScanning(true);
        try {
            const res = await scanWifi(rescan, signal);
            setGroups(groupNetworks(res.access_points));
            setScanError(null);
        } catch (e) {
            if (!signal?.aborted) setScanError(e instanceof Error ? e.message : String(e));
        } finally {
            if (!signal?.aborted) setScanning(false);
        }
    }, []);

    useEffect(() => {
        const controller = new AbortController();
        void load(false, controller.signal);
        return () => controller.abort();
    }, [load]);

    return (
        <Card
            size="small"
            title={t("networkWifi.availableTitle")}
            style={{marginBottom: 16}}
            extra={
                <Button size="small" icon={<ReloadOutlined/>} loading={scanning} onClick={() => void load(true)} data-testid="wifi-scan">
                    {t("networkWifi.scan")}
                </Button>
            }
        >
            {scanError && <Alert type="error" showIcon style={{marginBottom: 8}} message={scanError}/>}
            {groups === null ? <Spin/> : groups.length === 0 ? (
                <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description={t("networkWifi.noNetworks")}/>
            ) : (
                <List
                    size="small"
                    dataSource={groups}
                    data-testid="wifi-networks"
                    renderItem={(g) => (
                        <List.Item
                            data-testid="wifi-network"
                            actions={[
                                g.active ? <Tag key="c" color="success">{t("networkWifi.connected")}</Tag> : (
                                    <Button
                                        key="j"
                                        size="small"
                                        disabled={busy || !isJoinable(g.security)}
                                        title={isJoinable(g.security) ? undefined : t("networkWifi.unsupportedSecurity")}
                                        onClick={() => setJoining(g)}
                                    >
                                        {t("networkWifi.join")}
                                    </Button>
                                ),
                            ]}
                        >
                            <Space size={8} wrap>
                                <SignalIcon strength={g.strength}/>
                                <Text strong>{g.ssid}</Text>
                                <SecurityTag security={g.security}/>
                                {g.bands.map((b) => <Tag key={b}>{t("networkWifi.bandTag", {band: b})}</Tag>)}
                                {g.saved && !g.active && <Tag color="purple">{t("networkWifi.saved")}</Tag>}
                            </Space>
                        </List.Item>
                    )}
                />
            )}
            <JoinDialog
                network={joining}
                saved={joining ? saved.find((s) => s.ssid === joining.ssid) : undefined}
                onClose={() => setJoining(null)}
                run={run}
            />
        </Card>
    );
}

interface JoinDialogProps {
    network: WifiNetworkGroup | null;
    saved: WifiSavedNetwork | undefined;
    onClose: () => void;
    run: Run;
}

function JoinDialog({network, saved, onClose, run}: JoinDialogProps) {
    const {t} = useTranslation();
    const [password, setPassword] = useState("");
    const [remember, setRemember] = useState(true);
    const [submitting, setSubmitting] = useState(false);

    // Fresh form for each network the dialog opens on.
    const [shownFor, setShownFor] = useState<string | null>(null);
    if ((network?.ssid ?? null) !== shownFor) {
        setShownFor(network?.ssid ?? null);
        setPassword("");
        setRemember(true);
    }

    if (!network) return null;
    const wantsPassword = needsPassword(network.security);
    const passwordOk = !wantsPassword || (saved !== undefined && password === "") || (password.length >= 8 && password.length <= 64);

    const submit = async () => {
        setSubmitting(true);
        const ok = await run(
            () => connectWifi(saved && password === ""
                ? {uuid: saved.uuid}
                : {ssid: network.ssid, password: wantsPassword ? password : undefined, remember}),
            t("networkWifi.joining", {ssid: network.ssid}),
        );
        setSubmitting(false);
        if (ok) onClose();
    };

    return (
        <Modal
            open
            title={t("networkWifi.joinTitle", {ssid: network.ssid})}
            okText={t("networkWifi.join")}
            cancelText={t("networkWifi.cancel")}
            okButtonProps={{disabled: !passwordOk, loading: submitting}}
            onOk={() => void submit()}
            onCancel={onClose}
            destroyOnHidden
        >
            <Alert type="warning" showIcon style={{marginBottom: 12}} message={t("networkWifi.dropWarning")}/>
            {wantsPassword && (
                <>
                    <Input.Password
                        data-testid="wifi-password"
                        value={password}
                        placeholder={t(saved ? "networkWifi.passwordSavedPlaceholder" : "networkWifi.passwordPlaceholder")}
                        onChange={(e) => setPassword(e.target.value)}
                        onPressEnter={() => passwordOk && void submit()}
                        autoComplete="off"
                        maxLength={64}
                        visibilityToggle
                    />
                    {password !== "" && password.length < 8 && (
                        <Text type="danger" style={{fontSize: 12}}>{t("networkWifi.passwordShort")}</Text>
                    )}
                </>
            )}
            {!saved && (
                <Checkbox
                    data-testid="wifi-remember"
                    checked={remember}
                    onChange={(e) => setRemember(e.target.checked)}
                    style={{marginTop: 12, display: "flex"}}
                >
                    {t("networkWifi.remember")}
                </Checkbox>
            )}
            {!remember && !saved && (
                <Paragraph type="secondary" style={{fontSize: 12, margin: "4px 0 0"}}>{t("networkWifi.rememberHint")}</Paragraph>
            )}
        </Modal>
    );
}

function SavedCard({saved, busy, run}: {saved: WifiSavedNetwork[]; busy: boolean; run: Run}) {
    const {t} = useTranslation();
    return (
        <Card size="small" title={t("networkWifi.savedTitle")} style={{marginBottom: 16}}>
            {saved.length === 0 ? (
                <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description={t("networkWifi.noSaved")}/>
            ) : (
                <List
                    size="small"
                    dataSource={saved}
                    data-testid="wifi-saved"
                    renderItem={(s) => (
                        <List.Item
                            data-testid="wifi-saved-item"
                            actions={[
                                !s.active && (
                                    <Button key="c" size="small" disabled={busy}
                                        onClick={() => void run(() => connectWifi({uuid: s.uuid}), t("networkWifi.joining", {ssid: s.ssid}))}>
                                        {t("networkWifi.connect")}
                                    </Button>
                                ),
                                <Popconfirm
                                    key="f"
                                    title={t(s.active ? "networkWifi.forgetActiveConfirm" : "networkWifi.forgetConfirm", {ssid: s.ssid})}
                                    okText={t("networkWifi.forget")}
                                    cancelText={t("networkWifi.cancel")}
                                    okButtonProps={{danger: true}}
                                    onConfirm={() => void run(() => forgetWifi(s.uuid), t("networkWifi.forgotten", {ssid: s.ssid}))}
                                >
                                    <Button size="small" danger icon={<DeleteOutlined/>} disabled={busy} aria-label={t("networkWifi.forget")}/>
                                </Popconfirm>,
                            ].filter(Boolean)}
                        >
                            <Space size={8} wrap>
                                <Text strong>{s.ssid || s.id}</Text>
                                {s.active && <Tag color="success">{t("networkWifi.connected")}</Tag>}
                                {s.band && <Tag>{t(s.band === "bg" ? "networkWifi.band24" : "networkWifi.band5")}</Tag>}
                                {s.powersave === "disabled" && <Tag>{t("networkWifi.powersaveOff")}</Tag>}
                                {!s.autoconnect && <Tag>{t("networkWifi.manual")}</Tag>}
                            </Space>
                        </List.Item>
                    )}
                />
            )}
        </Card>
    );
}
