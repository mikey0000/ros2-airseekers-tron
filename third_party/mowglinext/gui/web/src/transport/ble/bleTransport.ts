import type {SocketLike, SocketOptions, Transport} from "../types.ts";
import {GattLink, type LinkCounters, type LinkState} from "./gattLink.ts";
import {Tunnel, type TunnelState} from "./tunnel.ts";

export interface BleStatus {
    link: LinkState;
    session: TunnelState;
    deviceId: string | null;
    deviceName: string | null;
    /** Mower hostname from WELCOME. */
    host: string | null;
    rttMs: number | null;
}

export interface BleDevice {
    id: string;
    name: string | null;
}

/** What the connection manager and the Settings UI drive; BleTransport implements it. */
export interface BleController {
    getStatus(): BleStatus;
    getCounters(): LinkCounters;
    subscribe(listener: (status: BleStatus) => void): () => void;
    unsubscribe(listener: (status: BleStatus) => void): void;
    /** Token for the next HELLO; restarts a running session that was refused. */
    setToken(token: string): void;
    /** Device chooser; must be called from a user gesture. */
    pairNew(): Promise<BleDevice>;
    /** Previously permitted device in range: connect. Resolves null when there is none. */
    connectKnown(deviceId: string | null): Promise<BleDevice | null>;
    disconnect(): void;
    forget(): Promise<void>;
}

/** Retry HELLO this long after the bridge said the backend was unavailable. */
const BACKEND_RETRY_MS = 5000;

/** The `ble` transport: Web Bluetooth link + tunnel (docs/ble_tunnel_protocol.md). */
export class BleTransport implements Transport, BleController {
    readonly kind = "ble" as const;

    private readonly tunnel: Tunnel;
    private readonly listeners = new Set<(status: BleStatus) => void>();
    private token = "";
    private status: BleStatus;
    private backendRetry: ReturnType<typeof setTimeout> | null = null;

    constructor(private readonly link: GattLink, client: string) {
        this.tunnel = new Tunnel(link, {client});
        this.status = this.compose();
        link.onUp = () => this.tunnel.start(this.token);
        link.subscribe(() => this.update());
        this.tunnel.subscribe((tunnelStatus) => {
            if (tunnelStatus.state === "dead") this.link.restart();
            if (tunnelStatus.state === "unauthenticated") this.link.disconnect();
            if (tunnelStatus.state === "backend_unavailable") this.retryBackendLater();
            this.update();
        });
    }

    fetch(path: string, init?: RequestInit): Promise<Response> {
        return this.tunnel.openHttp(path, init);
    }

    openSocket(path: string, options?: SocketOptions): SocketLike {
        return this.tunnel.openSocket(path, options);
    }

    getStatus(): BleStatus {
        return this.status;
    }

    getCounters(): LinkCounters {
        return this.link.getCounters();
    }

    subscribe(listener: (status: BleStatus) => void): () => void {
        this.listeners.add(listener);
        return () => this.unsubscribe(listener);
    }

    unsubscribe(listener: (status: BleStatus) => void): void {
        this.listeners.delete(listener);
    }

    unsubscribeAll(): void {
        this.listeners.clear();
    }

    setToken(token: string): void {
        if (token === this.token) return;
        this.token = token;
        if (this.link.getStatus().state === "connected") this.tunnel.start(token);
    }

    async pairNew(): Promise<BleDevice> {
        const status = await this.link.pairNew();
        return {id: status.deviceId ?? "", name: status.deviceName};
    }

    async connectKnown(deviceId: string | null): Promise<BleDevice | null> {
        if (!(await this.link.connectKnown(deviceId))) return null;
        const status = this.link.getStatus();
        return {id: status.deviceId ?? "", name: status.deviceName};
    }

    disconnect(): void {
        this.clearBackendRetry();
        this.tunnel.stop();
        this.link.disconnect();
    }

    async forget(): Promise<void> {
        this.clearBackendRetry();
        this.tunnel.stop();
        await this.link.forget();
    }

    private retryBackendLater(): void {
        this.clearBackendRetry();
        this.backendRetry = setTimeout(() => {
            this.backendRetry = null;
            if (this.link.getStatus().state === "connected") this.tunnel.start(this.token);
        }, BACKEND_RETRY_MS);
    }

    private clearBackendRetry(): void {
        if (this.backendRetry !== null) clearTimeout(this.backendRetry);
        this.backendRetry = null;
    }

    private compose(): BleStatus {
        const link = this.link.getStatus();
        const tunnel = this.tunnel.getStatus();
        return {
            link: link.state,
            session: tunnel.state,
            deviceId: link.deviceId,
            deviceName: link.deviceName,
            host: tunnel.host,
            rttMs: tunnel.rttMs,
        };
    }

    private update(): void {
        const next = this.compose();
        const prev = this.status;
        if ((Object.keys(next) as (keyof BleStatus)[]).every((k) => next[k] === prev[k])) return;
        this.status = next;
        this.listeners.forEach((l) => l(next));
    }
}
