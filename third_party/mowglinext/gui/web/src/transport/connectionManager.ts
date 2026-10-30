/**
 * Wi-Fi / Bluetooth switching policy (docs/DESIGN_ble_transport_2026-10.md section 2,
 * "Switching"). The link is a manual choice. The one automatic rule: when the app starts
 * without a reachable network, or the network has been dead for 3 s, and a previously
 * permitted mower is in range over Bluetooth, switch to Bluetooth and say so once. Never
 * switch back by itself.
 */
import {useSyncExternalStore} from "react";
import type {TransportKind} from "./types.ts";
import type {BleController, BleStatus} from "./ble/bleTransport.ts";
import type {BleSupport} from "./ble/gattLink.ts";
import {webBluetoothSupport} from "./ble/gattLink.ts";
import {getBleTransport} from "./ble/instance.ts";
import {apiFetch} from "./index.ts";
import {setActiveTransport} from "./registry.ts";
import {networkHttpBase} from "./network.ts";

export type ConnectionMode = "wifi" | "ble";
export type ConnectionNotice = "autoSwitchedToBle";

/** Per-viewer localStorage keys. */
export const STORAGE_KEYS = {
    mode: "connection.mode",
    token: "ble.token",
    deviceId: "ble.deviceId",
} as const;

export const CONNECTION_TIMING = {
    /** Liveness probe period while on Wi-Fi. */
    probeOnWifiMs: 3000,
    /** While on Bluetooth: only to tell the user Wi-Fi is back. */
    probeOnBleMs: 10000,
    probeTimeoutMs: 2000,
    /** Network dead this long → the automatic rule may switch to Bluetooth. */
    networkDeadMs: 3000,
    /** Between two automatic attempts during one outage (each is a scan). */
    autoRetryMs: 15000,
    tokenRetryMs: 60000,
};

export interface ConnectionState {
    mode: ConnectionMode;
    ble: BleStatus;
    bleSupport: BleSupport;
    /** null until the first probe answered or failed. */
    networkAlive: boolean | null;
    lastNetworkAliveAt: number | null;
    token: string | null;
    deviceId: string | null;
    notice: ConnectionNotice | null;
}

export interface ConnectionDeps {
    ble: BleController;
    bleSupport: () => BleSupport;
    /** Seam fetch (active transport): token fetch and the probe while on Wi-Fi. */
    apiFetch: (path: string, init?: RequestInit) => Promise<Response>;
    /** Plain network fetch, bypassing the active transport: the probe while on Bluetooth. */
    networkFetch: (path: string, init?: RequestInit) => Promise<Response>;
    setActiveTransport: (kind: TransportKind) => void;
    storage: Storage | null;
    isSecureContext: () => boolean;
}

const PROBE_PATH = "/web-build.json";
const TOKEN_PATH = "/api/ble/token";

export class ConnectionManager {
    private readonly listeners = new Set<(state: ConnectionState) => void>();
    private state: ConnectionState;
    private started = false;
    private probeTimer: ReturnType<typeof setTimeout> | null = null;
    private tokenFetched = false;
    private lastTokenAttempt: number | null = null;
    private autoAttemptAt: number | null = null;
    private autoInFlight = false;
    private bleConnectInFlight = false;
    private unsubscribeBle: (() => void) | null = null;

    constructor(private readonly deps: ConnectionDeps) {
        const storedMode = readStorage(deps.storage, STORAGE_KEYS.mode);
        this.state = {
            mode: storedMode === "ble" ? "ble" : "wifi",
            ble: deps.ble.getStatus(),
            bleSupport: deps.bleSupport(),
            networkAlive: null,
            lastNetworkAliveAt: null,
            token: readStorage(deps.storage, STORAGE_KEYS.token),
            deviceId: readStorage(deps.storage, STORAGE_KEYS.deviceId),
            notice: null,
        };
        if (this.state.token) deps.ble.setToken(this.state.token);
    }

    getState = (): ConnectionState => this.state;

    subscribe = (listener: (state: ConnectionState) => void): (() => void) => {
        this.listeners.add(listener);
        return () => this.unsubscribe(listener);
    };

    unsubscribe(listener: (state: ConnectionState) => void): void {
        this.listeners.delete(listener);
    }

    unsubscribeAll(): void {
        this.listeners.clear();
    }

    /** Idempotent. Applies the stored mode and starts the liveness probe. */
    start(): void {
        if (this.started) return;
        this.started = true;
        this.unsubscribeBle = this.deps.ble.subscribe((ble) => this.update({ble}));
        this.deps.setActiveTransport(this.state.mode === "ble" ? "ble" : "network");
        if (this.state.mode === "ble") void this.connectBle();
        void this.probe();
    }

    stop(): void {
        this.started = false;
        if (this.probeTimer !== null) clearTimeout(this.probeTimer);
        this.probeTimer = null;
        this.unsubscribeBle?.();
        this.unsubscribeBle = null;
    }

    /** Manual switch (the user tapped Wi-Fi or Bluetooth). */
    async switchTo(mode: ConnectionMode): Promise<void> {
        writeStorage(this.deps.storage, STORAGE_KEYS.mode, mode);
        if (mode === this.state.mode) {
            if (mode === "ble") await this.connectBle();
            return;
        }
        this.update({mode, notice: null});
        if (mode === "wifi") {
            this.deps.setActiveTransport("network");
            this.deps.ble.disconnect();
        } else {
            this.deps.setActiveTransport("ble");
            await this.connectBle();
        }
        this.reschedule();
    }

    /** Device chooser (call from a click handler), then use Bluetooth. */
    async pairNew(): Promise<void> {
        const device = await this.deps.ble.pairNew();
        this.rememberDevice(device.id);
        writeStorage(this.deps.storage, STORAGE_KEYS.mode, "ble");
        if (this.state.mode !== "ble") {
            this.update({mode: "ble", notice: null});
            this.deps.setActiveTransport("ble");
            this.reschedule();
        }
    }

    async forgetDevice(): Promise<void> {
        await this.deps.ble.forget();
        removeStorage(this.deps.storage, STORAGE_KEYS.deviceId);
        this.update({deviceId: null});
        if (this.state.mode === "ble") await this.switchTo("wifi");
    }

    /** Manual paste, or the value fetched from the backend while on Wi-Fi. */
    setToken(token: string): void {
        const trimmed = token.trim();
        if (trimmed) writeStorage(this.deps.storage, STORAGE_KEYS.token, trimmed);
        else removeStorage(this.deps.storage, STORAGE_KEYS.token);
        this.update({token: trimmed || null});
        this.deps.ble.setToken(trimmed);
    }

    /** Bluetooth bytes in/out since the page loaded (not part of the state: they change constantly). */
    getBleCounters(): {bytesIn: number; bytesOut: number} {
        return this.deps.ble.getCounters();
    }

    dismissNotice(): void {
        this.update({notice: null});
    }

    private rememberDevice(id: string): void {
        if (!id || id === this.state.deviceId) return;
        writeStorage(this.deps.storage, STORAGE_KEYS.deviceId, id);
        this.update({deviceId: id});
    }

    private async connectBle(): Promise<boolean> {
        if (this.bleConnectInFlight || this.state.bleSupport !== "ok") return false;
        if (this.state.ble.link !== "idle") return this.state.ble.link === "connected";
        this.bleConnectInFlight = true;
        try {
            const device = await this.deps.ble.connectKnown(this.state.deviceId);
            if (device) this.rememberDevice(device.id);
            return device !== null;
        } catch {
            return false;
        } finally {
            this.bleConnectInFlight = false;
        }
    }

    private async probe(): Promise<void> {
        this.probeTimer = null;
        const mode = this.state.mode;
        const alive = await this.networkAlive(mode === "wifi" ? this.deps.apiFetch : this.deps.networkFetch);
        if (!this.started) return;
        const now = Date.now();
        if (alive) {
            this.update({networkAlive: true, lastNetworkAliveAt: now});
            this.autoAttemptAt = null;
            void this.fetchTokenOnce();
        } else {
            this.update({networkAlive: false});
            const since = this.state.lastNetworkAliveAt;
            if (this.state.mode === "wifi" && (since === null || now - since >= CONNECTION_TIMING.networkDeadMs)) {
                void this.autoSwitch();
            }
        }
        // On Bluetooth with the link dropped for good (not in range at the time): try again.
        if (this.state.mode === "ble" && this.state.ble.link === "idle") void this.connectBle();
        this.reschedule();
    }

    private reschedule(): void {
        if (!this.started) return;
        if (this.probeTimer !== null) clearTimeout(this.probeTimer);
        const period = this.state.mode === "wifi" ? CONNECTION_TIMING.probeOnWifiMs : CONNECTION_TIMING.probeOnBleMs;
        this.probeTimer = setTimeout(() => void this.probe(), period);
    }

    private async networkAlive(fetcher: ConnectionDeps["apiFetch"]): Promise<boolean> {
        const controller = new AbortController();
        const timer = setTimeout(() => controller.abort(), CONNECTION_TIMING.probeTimeoutMs);
        try {
            const response = await fetcher(PROBE_PATH, {cache: "no-store", signal: controller.signal});
            return response.ok;
        } catch {
            return false;
        } finally {
            clearTimeout(timer);
        }
    }

    /** The one automatic rule. */
    private async autoSwitch(): Promise<void> {
        if (this.autoInFlight || this.state.bleSupport !== "ok") return;
        const now = Date.now();
        if (this.autoAttemptAt !== null && now - this.autoAttemptAt < CONNECTION_TIMING.autoRetryMs) return;
        this.autoInFlight = true;
        this.autoAttemptAt = now;
        try {
            const connected = await this.connectBle();
            if (!connected || this.state.mode !== "wifi") return;
            // Not persisted: the stored mode is the user's own choice.
            this.update({mode: "ble", notice: "autoSwitchedToBle"});
            this.deps.setActiveTransport("ble");
            this.reschedule();
        } finally {
            this.autoInFlight = false;
        }
    }

    /** A phone gets the token by having reached the Wi-Fi GUI once over a secure origin. */
    private async fetchTokenOnce(): Promise<void> {
        if (this.tokenFetched || this.state.mode !== "wifi" || !this.deps.isSecureContext()) return;
        const now = Date.now();
        if (this.lastTokenAttempt !== null && now - this.lastTokenAttempt < CONNECTION_TIMING.tokenRetryMs) return;
        this.lastTokenAttempt = now;
        try {
            const response = await this.deps.apiFetch(TOKEN_PATH, {cache: "no-store"});
            if (!response.ok) return;
            const body = (await response.json()) as {token?: unknown};
            if (typeof body.token !== "string" || !body.token) return;
            this.tokenFetched = true;
            if (body.token !== this.state.token) this.setToken(body.token);
        } catch {
            // Older backend without the endpoint, or offline: retry after tokenRetryMs.
        }
    }

    private update(patch: Partial<ConnectionState>): void {
        const next = {...this.state, ...patch};
        if ((Object.keys(patch) as (keyof ConnectionState)[]).every((k) => next[k] === this.state[k])) return;
        this.state = next;
        this.listeners.forEach((l) => l(next));
    }
}

function readStorage(storage: Storage | null, key: string): string | null {
    try {
        return storage?.getItem(key) ?? null;
    } catch {
        return null;
    }
}

function writeStorage(storage: Storage | null, key: string, value: string): void {
    try {
        storage?.setItem(key, value);
    } catch {
        // Private mode or blocked storage: the choice lasts for this page only.
    }
}

function removeStorage(storage: Storage | null, key: string): void {
    try {
        storage?.removeItem(key);
    } catch {
        // As above.
    }
}

function defaultStorage(): Storage | null {
    try {
        return typeof window !== "undefined" ? window.localStorage : null;
    } catch {
        return null;
    }
}

let manager: ConnectionManager | null = null;

/** The app's connection manager, created on first use (started by transport/ble/register.ts). */
export function getConnectionManager(): ConnectionManager {
    manager ??= new ConnectionManager({
        ble: getBleTransport(),
        bleSupport: webBluetoothSupport,
        apiFetch,
        networkFetch: (path, init) => fetch(`${networkHttpBase()}${path}`, init),
        setActiveTransport,
        storage: defaultStorage(),
        isSecureContext: () => typeof window !== "undefined" && window.isSecureContext,
    });
    return manager;
}

export function useConnectionState(): ConnectionState {
    const m = getConnectionManager();
    return useSyncExternalStore(m.subscribe, m.getState);
}
