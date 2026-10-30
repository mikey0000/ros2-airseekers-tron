import {useEffect} from "react";
import {App} from "antd";
import {Bluetooth, BluetoothConnected, BluetoothOff, BluetoothSearching, Wifi, WifiOff} from "lucide-react";
import {useTranslation} from "react-i18next";
import {useNavigate} from "react-router-dom";
import {getConnectionManager, useConnectionState, type ConnectionState} from "../transport/connectionManager.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";

type Tone = "ok" | "busy" | "bad";

interface ChipView {
    tone: Tone;
    labelKey: string;
    icon: typeof Wifi;
    /** On Bluetooth with the network reachable again. */
    wifiBack: boolean;
}

function chipView(state: ConnectionState): ChipView {
    if (state.mode === "wifi") {
        if (state.networkAlive === false) return {tone: "bad", labelKey: "connection.chip.wifiDown", icon: WifiOff, wifiBack: false};
        if (state.networkAlive === null) return {tone: "busy", labelKey: "connection.chip.connecting", icon: Wifi, wifiBack: false};
        return {tone: "ok", labelKey: "connection.chip.connected", icon: Wifi, wifiBack: false};
    }
    const wifiBack = state.networkAlive === true;
    const {link, session} = state.ble;
    if (session === "unauthenticated") return {tone: "bad", labelKey: "connection.chip.tokenRejected", icon: BluetoothOff, wifiBack};
    if (link === "connected" && session === "ready") return {tone: "ok", labelKey: "connection.chip.connected", icon: BluetoothConnected, wifiBack};
    if (link === "reconnecting" || session === "dead") return {tone: "busy", labelKey: "connection.chip.reconnecting", icon: BluetoothSearching, wifiBack};
    if (link === "connecting" || link === "connected") return {tone: "busy", labelKey: "connection.chip.connecting", icon: BluetoothSearching, wifiBack};
    return {tone: "bad", labelKey: "connection.chip.notConnected", icon: link === "unavailable" ? BluetoothOff : Bluetooth, wifiBack};
}

/** Header chip: which link the app uses and its state. Tap opens Settings → Connection. */
export function ConnectionChip() {
    const {t} = useTranslation();
    const navigate = useNavigate();
    const {colors} = useThemeMode();
    const {message} = App.useApp();
    const state = useConnectionState();
    const view = chipView(state);

    useEffect(() => {
        if (state.notice !== "autoSwitchedToBle") return;
        void message.info(t("connection.autoSwitched"), 6);
        getConnectionManager().dismissNotice();
    }, [state.notice, message, t]);

    const color = {ok: colors.accent, busy: colors.warning, bad: colors.danger}[view.tone];
    const Icon = view.icon;
    const link = t(state.mode === "wifi" ? "connection.wifi" : "connection.bluetooth");
    const label = t(view.labelKey);

    return (
        <button
            type="button"
            data-testid="connection-chip"
            aria-label={t("connection.chipLabel", {link, state: label})}
            title={view.wifiBack ? t("connection.chip.wifiBack") : undefined}
            onClick={() => void navigate({pathname: "/settings", search: "?section=connection"})}
            style={{
                display: "inline-flex", alignItems: "center", gap: 6,
                height: 32, padding: "0 10px", borderRadius: 999,
                border: `1px solid ${colors.borderSubtle}`,
                background: "rgba(255, 255, 255, 0.04)",
                color: colors.text, fontSize: 12, fontWeight: 600,
                cursor: "pointer", whiteSpace: "nowrap",
            }}
        >
            <Icon size={15} color={color} aria-hidden/>
            <span style={{color}}>{label}</span>
            {view.wifiBack && <Wifi size={13} color={colors.accent} aria-hidden/>}
        </button>
    );
}
