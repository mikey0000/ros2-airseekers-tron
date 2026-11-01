// Header chip: the mower's own Wi-Fi link (mower_control/wifi_monitor, topic key
// `wifiStatus` = /wifi/status, 1 Hz). Hidden until the first message, so robots
// without the monitor show nothing.
import {Wifi, WifiHigh, WifiLow, WifiOff, WifiZero} from "lucide-react";
import {useTranslation} from "react-i18next";
import {useTopic} from "../hooks/useTopic.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";

export interface WifiStatusView {
    connected: boolean;
    ssid: string;
    bssid: string;
    freqMhz: number;
    signalDbm: number | null;
    txMbps: number;
    rxMbps: number;
    txRetries: number;
    txFailed: number;
    roamCount: number;
}

export type WifiLevel = "good" | "ok" | "weak" | "bad";

const num = (v: unknown): number => (typeof v === "number" && Number.isFinite(v) ? v : 0);
const str = (v: unknown): string => (typeof v === "string" ? v : "");

/** Raw multiplex payload (mower_interfaces/WifiStatus as JSON) -> view, or undefined. */
export function selectWifiStatus(raw: unknown): WifiStatusView | undefined {
    if (!raw || typeof raw !== "object") return undefined;
    const r = raw as Record<string, unknown>;
    if (typeof r.connected !== "boolean") return undefined;
    const sig = r.signal_dbm;
    return {
        connected: r.connected,
        ssid: str(r.ssid),
        bssid: str(r.bssid),
        freqMhz: num(r.freq_mhz),
        signalDbm: r.connected && typeof sig === "number" && Number.isFinite(sig) ? sig : null,
        txMbps: num(r.tx_bitrate_mbps),
        rxMbps: num(r.rx_bitrate_mbps),
        txRetries: num(r.tx_retries),
        txFailed: num(r.tx_failed),
        roamCount: num(r.roam_count),
    };
}

/** >-60 good, -60..-70 ok, -70..-78 weak, below (or no link) bad. */
export function wifiLevel(view: WifiStatusView): WifiLevel {
    const s = view.signalDbm;
    if (!view.connected || s === null) return view.connected ? "ok" : "bad";
    if (s > -60) return "good";
    if (s >= -70) return "ok";
    if (s >= -78) return "weak";
    return "bad";
}

/** 802.11 channel for a centre frequency in MHz (0 when unknown). */
export function freqToChannel(freqMhz: number): number {
    const f = Math.round(freqMhz);
    if (f === 2484) return 14;
    if (f >= 2412 && f <= 2472) return (f - 2407) / 5;
    if (f >= 5000 && f <= 5895) return Math.floor((f - 5000) / 5);
    if (f >= 5955 && f <= 7115) return Math.floor((f - 5950) / 5);
    return 0;
}

const ICONS = {good: Wifi, ok: WifiHigh, weak: WifiLow, bad: WifiZero} as const;

export function WifiChip() {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const {data} = useTopic<WifiStatusView | null>("wifiStatus", null, {
        select: selectWifiStatus,
        throttleMs: 1000,
    });
    if (!data) return null;

    const level = wifiLevel(data);
    const color = {good: colors.accent, ok: colors.info, weak: colors.warning, bad: colors.danger}[level];
    const Icon = data.connected ? ICONS[level] : WifiOff;
    const text = data.signalDbm !== null ? `${Math.round(data.signalDbm)} dBm` : t("wifiChip.noSignal");
    const stateKey = data.connected ? level : "disconnected";
    const tooltip = data.connected ? [
        t("wifiChip.ssid", {value: data.ssid || "-"}),
        t("wifiChip.bssid", {value: data.bssid || "-"}),
        t("wifiChip.channel", {channel: freqToChannel(data.freqMhz), freq: Math.round(data.freqMhz)}),
        t("wifiChip.bitrate", {tx: data.txMbps.toFixed(0), rx: data.rxMbps.toFixed(0)}),
        t("wifiChip.retries", {retries: data.txRetries, failed: data.txFailed}),
        t("wifiChip.roams", {count: data.roamCount}),
    ].join("\n") : t("wifiChip.roams", {count: data.roamCount});

    return (
        <span
            data-testid="wifi-chip"
            data-level={data.connected ? level : "disconnected"}
            role="status"
            aria-label={t("wifiChip.label", {state: t(`wifiChip.state.${stateKey}`)})}
            title={tooltip}
            style={{
                display: "inline-flex", alignItems: "center", gap: 6,
                height: 32, padding: "0 10px", borderRadius: 999,
                border: `1px solid ${colors.borderSubtle}`,
                background: "rgba(255, 255, 255, 0.04)",
                color: colors.text, fontSize: 12, fontWeight: 600, whiteSpace: "nowrap",
            }}
        >
            <Icon size={15} color={color} aria-hidden/>
            {text}
        </span>
    );
}
