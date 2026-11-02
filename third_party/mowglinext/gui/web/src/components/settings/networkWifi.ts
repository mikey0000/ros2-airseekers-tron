// Client for /api/network/wifi/* (pkg/api/network_wifi.go): the host's
// NetworkManager as seen from the GUI. status and scan are open; connect,
// forget, band and powersave need the X-Mower-Token header, which is the BLE
// tunnel token this browser already stores (Settings > Connection).
import {apiFetch} from "../../transport/index.ts";
import {getConnectionManager} from "../../transport/connectionManager.ts";

export type WifiSecurity = "open" | "wep" | "wpa" | "wpa2" | "wpa2-wpa3" | "wpa3" | "owe" | "enterprise";
export type WifiBand = "" | "bg" | "a";
export type WifiPowersave = "default" | "ignore" | "disabled" | "enabled";

export interface WifiSavedNetwork {
    uuid: string;
    id: string;
    ssid: string;
    security: WifiSecurity;
    band: WifiBand;
    powersave: WifiPowersave;
    autoconnect: boolean;
    hidden: boolean;
    active: boolean;
}

export interface WifiHostStatus {
    available: boolean;
    reason?: string;
    interface?: string;
    state?: string;
    connected: boolean;
    ssid?: string;
    bssid?: string;
    freq_mhz?: number;
    strength?: number;
    bitrate_kbps?: number;
    ip4?: string[];
    connection?: WifiSavedNetwork;
    saved: WifiSavedNetwork[];
}

export interface WifiAccessPoint {
    ssid: string;
    bssid: string;
    freq_mhz: number;
    strength: number;
    security: WifiSecurity;
    active: boolean;
    saved: boolean;
}

export interface WifiScanResult {
    scanned: boolean;
    access_points: WifiAccessPoint[];
}

export interface WifiConnectRequest {
    ssid?: string;
    uuid?: string;
    password?: string;
    hidden?: boolean;
    remember?: boolean;
}

export interface WifiConnectResult {
    uuid: string;
    created: boolean;
}

/** One SSID from the scan list: its strongest BSS plus the bands it is seen on. */
export interface WifiNetworkGroup {
    ssid: string;
    strength: number;
    security: WifiSecurity;
    active: boolean;
    saved: boolean;
    bands: Array<"2.4" | "5" | "6">;
    accessPoints: number;
}

export const TOKEN_HEADER = "X-Mower-Token";
const BASE = "/api/network/wifi";
const TOKEN_PATH = "/api/ble/token";

/** A non-2xx answer; `status` 401 means the token was missing or wrong. */
export class WifiApiError extends Error {
    constructor(readonly status: number, message: string) {
        super(message);
        this.name = "WifiApiError";
    }
}

async function readJson<T>(response: Response): Promise<T> {
    if (!response.ok) {
        let message = `HTTP ${response.status}`;
        try {
            const body = (await response.json()) as {error?: unknown};
            if (typeof body.error === "string" && body.error) message = body.error;
        } catch {
            // Not JSON: keep the status line.
        }
        throw new WifiApiError(response.status, message);
    }
    return (await response.json()) as T;
}

export async function fetchWifiStatus(signal?: AbortSignal): Promise<WifiHostStatus> {
    return readJson(await apiFetch(`${BASE}/status`, {signal, cache: "no-store"}));
}

/** The access points NetworkManager knows; `rescan` asks for a fresh scan first (a few seconds). */
export async function scanWifi(rescan: boolean, signal?: AbortSignal): Promise<WifiScanResult> {
    const init: RequestInit = rescan ? {method: "POST", signal} : {signal, cache: "no-store"};
    return readJson(await apiFetch(`${BASE}/scan`, init));
}

/** The stored BLE token, else (on Wi-Fi only) the backend's, which is then stored. */
export async function resolveMowerToken(refresh = false): Promise<string> {
    const manager = getConnectionManager();
    const state = manager.getState();
    if (state.token && !refresh) return state.token;
    if (state.mode !== "wifi") {
        throw new WifiApiError(401, "no Bluetooth token stored in this browser (Settings > Connection)");
    }
    const body = await readJson<{token?: unknown}>(await apiFetch(TOKEN_PATH, {cache: "no-store"}));
    if (typeof body.token !== "string" || !body.token) throw new WifiApiError(500, "the mower returned no token");
    if (body.token !== state.token) manager.setToken(body.token);
    return body.token;
}

/** POST with the token; a 401 on Wi-Fi refetches the token once (it may have been rotated). */
async function mutate<T>(path: string, payload: unknown): Promise<T> {
    const send = async (token: string) => apiFetch(`${BASE}/${path}`, {
        method: "POST",
        headers: {"Content-Type": "application/json", [TOKEN_HEADER]: token},
        body: JSON.stringify(payload),
    });
    let response = await send(await resolveMowerToken());
    if (response.status === 401 && getConnectionManager().getState().mode === "wifi") {
        response = await send(await resolveMowerToken(true));
    }
    return readJson(response);
}

export function connectWifi(req: WifiConnectRequest): Promise<WifiConnectResult> {
    return mutate("connect", req);
}

export function forgetWifi(uuid: string): Promise<{ok?: string}> {
    return mutate("forget", {uuid});
}

export function setWifiBand(uuid: string, band: WifiBand): Promise<WifiSavedNetwork> {
    return mutate("band", {uuid, band});
}

export function setWifiPowersave(uuid: string, enabled: boolean): Promise<WifiSavedNetwork> {
    return mutate("powersave", {uuid, enabled});
}

export function bandOf(freqMhz: number): "2.4" | "5" | "6" | null {
    if (freqMhz >= 2400 && freqMhz < 2500) return "2.4";
    if (freqMhz >= 5150 && freqMhz < 5925) return "5";
    if (freqMhz >= 5925 && freqMhz < 7200) return "6";
    return null;
}

/** Scan list grouped by SSID (hidden SSIDs dropped), strongest first. */
export function groupNetworks(aps: readonly WifiAccessPoint[]): WifiNetworkGroup[] {
    const groups = new Map<string, WifiNetworkGroup>();
    for (const ap of aps) {
        if (!ap.ssid) continue;
        const band = bandOf(ap.freq_mhz);
        const group = groups.get(ap.ssid);
        if (!group) {
            groups.set(ap.ssid, {
                ssid: ap.ssid, strength: ap.strength, security: ap.security, active: ap.active,
                saved: ap.saved, bands: band ? [band] : [], accessPoints: 1,
            });
            continue;
        }
        group.accessPoints += 1;
        group.active ||= ap.active;
        group.saved ||= ap.saved;
        if (ap.strength > group.strength) {
            group.strength = ap.strength;
            group.security = ap.security;
        }
        if (band && !group.bands.includes(band)) group.bands.push(band);
    }
    for (const g of groups.values()) g.bands.sort();
    return [...groups.values()].sort((a, b) => Number(b.active) - Number(a.active) || b.strength - a.strength);
}

/** 0..4 bars from NetworkManager's strength percentage. */
export function signalBars(strength: number): 0 | 1 | 2 | 3 | 4 {
    if (strength >= 75) return 4;
    if (strength >= 55) return 3;
    if (strength >= 35) return 2;
    if (strength >= 15) return 1;
    return 0;
}

/** Security this page can join: personal (PSK / SAE) and open networks. */
export function isJoinable(security: WifiSecurity): boolean {
    return security !== "enterprise" && security !== "wep";
}

export function needsPassword(security: WifiSecurity): boolean {
    return security !== "open" && security !== "owe";
}
