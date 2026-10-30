/**
 * Web Bluetooth link to the mower's BLE module (mower_docs/04-ble-protocol.md,
 * docs/ble_tunnel_protocol.md section 1): phone writes go to c1db0004 with response, at most
 * 497 bytes and one write in flight; mower bytes arrive as notifications on c1db0002.
 */
import type {ByteLink} from "./tunnel.ts";

export const SERVICE_UUID = "c1db0001-431d-4cbe-96e2-9f9286c41572";
export const NOTIFY_UUID = "c1db0002-431d-4cbe-96e2-9f9286c41572";
export const WRITE_UUID = "c1db0004-431d-4cbe-96e2-9f9286c41572";
export const NAME_PREFIX = "AIRSEEKERS_";
/** ATT MTU 500 - 3. */
export const MAX_WRITE = 497;
/** Used for the rest of the connection if a 497-byte write is rejected. */
export const FALLBACK_WRITE = 244;

export type LinkState = "idle" | "connecting" | "connected" | "reconnecting" | "unavailable";
export type BleSupport = "ok" | "insecure" | "unsupported";

export interface LinkStatus {
    state: LinkState;
    deviceId: string | null;
    deviceName: string | null;
}

export interface LinkCounters {
    bytesIn: number;
    bytesOut: number;
}

export interface GattLinkOptions {
    /** One GATT connect + service discovery attempt. */
    connectTimeoutMs?: number;
    /** How long connectKnown() listens for the device's advertisements. */
    scanTimeoutMs?: number;
    firstRetryMs?: number;
    /** The module can take 20-30 s to advertise again after a disconnect. */
    maxRetryMs?: number;
}

const DEFAULTS: Required<GattLinkOptions> = {
    connectTimeoutMs: 15000,
    scanTimeoutMs: 8000,
    firstRetryMs: 1000,
    maxRetryMs: 30000,
};

export function webBluetoothSupport(): BleSupport {
    if (typeof window === "undefined" || !window.isSecureContext) return "insecure";
    return typeof navigator !== "undefined" && navigator.bluetooth ? "ok" : "unsupported";
}

export function isWebBluetoothAvailable(): boolean {
    return webBluetoothSupport() === "ok";
}

export class GattLink implements ByteLink {
    onBytes: ((bytes: Uint8Array) => void) | null = null;
    onDown: (() => void) | null = null;
    onDrain: (() => void) | null = null;
    /** GATT connected and notifications started. */
    onUp: (() => void) | null = null;

    private readonly config: Required<GattLinkOptions>;
    private readonly listeners = new Set<(status: LinkStatus) => void>();
    private readonly queue: Uint8Array[] = [];
    private readonly counters: LinkCounters = {bytesIn: 0, bytesOut: 0};
    private readonly onNotification = (event: Event) => this.receive(event);
    private readonly onDisconnected = () => this.handleDisconnected();

    private status: LinkStatus;
    private device: BluetoothDevice | null = null;
    private writer: BluetoothRemoteGATTCharacteristic | null = null;
    private notifier: BluetoothRemoteGATTCharacteristic | null = null;
    private queuedBytes = 0;
    private writing = false;
    private maxWrite = MAX_WRITE;
    /** The caller wants the link up: an unexpected disconnect starts the reconnect loop. */
    private wanted = false;
    private retries = 0;
    private retryTimer: ReturnType<typeof setTimeout> | null = null;
    /** Bumped by every connect and disconnect so a stale async connect gives up. */
    private generation = 0;

    constructor(
        private readonly bluetooth: Bluetooth | undefined,
        options: GattLinkOptions = {},
    ) {
        this.config = {...DEFAULTS, ...options};
        this.status = {state: bluetooth ? "idle" : "unavailable", deviceId: null, deviceName: null};
    }

    get bufferedAmount(): number {
        return this.queuedBytes;
    }

    getStatus(): LinkStatus {
        return this.status;
    }

    getCounters(): LinkCounters {
        return {...this.counters};
    }

    subscribe(listener: (status: LinkStatus) => void): () => void {
        this.listeners.add(listener);
        return () => this.unsubscribe(listener);
    }

    unsubscribe(listener: (status: LinkStatus) => void): void {
        this.listeners.delete(listener);
    }

    unsubscribeAll(): void {
        this.listeners.clear();
    }

    /** Chooser for a new mower. Must run inside a user gesture (a click handler). */
    async pairNew(): Promise<LinkStatus> {
        const bluetooth = this.requireBluetooth();
        const device = await bluetooth.requestDevice({
            filters: [{namePrefix: NAME_PREFIX}],
            optionalServices: [SERVICE_UUID],
        });
        this.useDevice(device);
        if (!(await this.connectOnce())) throw new Error("Could not connect to the mower over Bluetooth");
        return this.status;
    }

    /**
     * Connect to a device this origin was already granted (no gesture needed). Prefers
     * `deviceId`, else the first AIRSEEKERS_ device. Resolves false when there is none or it is
     * not in range.
     */
    async connectKnown(deviceId: string | null = null): Promise<boolean> {
        const bluetooth = this.bluetooth;
        if (!bluetooth?.getDevices) return false;
        if (this.status.state === "connected") return true;
        const devices = await bluetooth.getDevices();
        const device = devices.find((d) => deviceId !== null && d.id === deviceId)
            ?? devices.find((d) => d.name?.startsWith(NAME_PREFIX));
        if (!device) return false;
        if (!(await this.inRange(device))) return false;
        this.useDevice(device);
        return this.connectOnce();
    }

    /** Close the link and stop reconnecting. */
    disconnect(): void {
        this.wanted = false;
        this.generation++;
        this.clearRetry();
        const wasUp = this.writer !== null;
        this.teardown();
        this.device?.gatt?.disconnect();
        this.setState(this.bluetooth ? "idle" : "unavailable");
        if (wasUp) this.onDown?.();
    }

    /** Revoke the permission for the current device and disconnect. */
    async forget(): Promise<void> {
        const device = this.device;
        this.disconnect();
        this.device?.removeEventListener("gattserverdisconnected", this.onDisconnected);
        this.device = null;
        this.status = {...this.status, deviceId: null, deviceName: null};
        this.emit();
        await device?.forget?.();
    }

    /** Drop and re-establish the GATT connection (the tunnel saw a dead link). */
    restart(): void {
        if (!this.device) return;
        this.wanted = true;
        if (this.device.gatt?.connected) this.device.gatt.disconnect();
        // Chrome fires gattserverdisconnected for a local disconnect too; cover browsers that don't.
        if (this.writer !== null || this.status.state === "connected") this.handleDisconnected();
    }

    write(bytes: Uint8Array): void {
        if (!this.writer) return;
        this.queue.push(bytes);
        this.queuedBytes += bytes.length;
        void this.pump();
    }

    private requireBluetooth(): Bluetooth {
        if (!this.bluetooth) throw new Error("Web Bluetooth is not available");
        return this.bluetooth;
    }

    private useDevice(device: BluetoothDevice): void {
        if (this.device === device) return;
        this.device?.removeEventListener("gattserverdisconnected", this.onDisconnected);
        this.device = device;
        device.addEventListener("gattserverdisconnected", this.onDisconnected);
        this.status = {...this.status, deviceId: device.id, deviceName: device.name ?? null};
        this.emit();
    }

    private async inRange(device: BluetoothDevice): Promise<boolean> {
        // Without watchAdvertisements a connect attempt is the only probe.
        if (!device.watchAdvertisements) return true;
        const controller = new AbortController();
        try {
            return await new Promise<boolean>((resolve) => {
                const timer = setTimeout(() => resolve(false), this.config.scanTimeoutMs);
                device.addEventListener("advertisementreceived", () => {
                    clearTimeout(timer);
                    resolve(true);
                }, {once: true});
                device.watchAdvertisements!({signal: controller.signal}).catch(() => {
                    clearTimeout(timer);
                    resolve(true);
                });
            });
        } finally {
            controller.abort();
        }
    }

    /** One connect + discovery attempt. On success the link is up and reconnects on its own. */
    private async connectOnce(): Promise<boolean> {
        const device = this.device;
        const gatt = device?.gatt;
        if (!gatt) return false;
        const generation = ++this.generation;
        this.clearRetry();
        this.setState(this.wanted ? "reconnecting" : "connecting");
        try {
            const server = await withTimeout(gatt.connect(), this.config.connectTimeoutMs);
            const service = await server.getPrimaryService(SERVICE_UUID);
            const notifier = await service.getCharacteristic(NOTIFY_UUID);
            const writer = await service.getCharacteristic(WRITE_UUID);
            notifier.addEventListener("characteristicvaluechanged", this.onNotification);
            await notifier.startNotifications();
            if (generation !== this.generation) {
                notifier.removeEventListener("characteristicvaluechanged", this.onNotification);
                return false;
            }
            this.notifier = notifier;
            this.writer = writer;
            this.maxWrite = MAX_WRITE;
            this.wanted = true;
            this.retries = 0;
            this.clearRetry();
            this.setState("connected");
            this.onUp?.();
            return true;
        } catch {
            if (generation !== this.generation) return false;
            gatt.disconnect();
            if (this.wanted) this.scheduleRetry();
            else this.setState(this.bluetooth ? "idle" : "unavailable");
            return false;
        }
    }

    private handleDisconnected(): void {
        const wasUp = this.writer !== null;
        this.teardown();
        if (wasUp) this.onDown?.();
        if (this.wanted) {
            if (this.retryTimer === null) this.scheduleRetry();
        } else {
            this.setState(this.bluetooth ? "idle" : "unavailable");
        }
    }

    private scheduleRetry(): void {
        this.clearRetry();
        this.setState("reconnecting");
        const delay = Math.min(this.config.firstRetryMs * 2 ** this.retries, this.config.maxRetryMs);
        this.retries++;
        this.retryTimer = setTimeout(() => {
            this.retryTimer = null;
            if (this.status.state !== "connected") void this.connectOnce();
        }, delay);
    }

    private clearRetry(): void {
        if (this.retryTimer !== null) clearTimeout(this.retryTimer);
        this.retryTimer = null;
    }

    private teardown(): void {
        this.notifier?.removeEventListener("characteristicvaluechanged", this.onNotification);
        this.notifier = null;
        this.writer = null;
        this.queue.length = 0;
        this.queuedBytes = 0;
    }

    private receive(event: Event): void {
        const value = (event.target as BluetoothRemoteGATTCharacteristic | null)?.value;
        if (!value) return;
        // Copy: the browser may reuse the DataView's buffer for the next notification.
        const bytes = new Uint8Array(value.buffer.slice(value.byteOffset, value.byteOffset + value.byteLength));
        this.counters.bytesIn += bytes.length;
        this.onBytes?.(bytes);
    }

    private async pump(): Promise<void> {
        const writer = this.writer;
        if (this.writing || !writer || this.queue.length === 0) return;
        const chunk = this.takeChunk();
        this.writing = true;
        try {
            await writer.writeValueWithResponse(chunk);
            this.counters.bytesOut += chunk.length;
        } catch {
            if (this.writer !== writer) return;
            if (this.maxWrite > FALLBACK_WRITE && chunk.length > FALLBACK_WRITE) {
                this.maxWrite = FALLBACK_WRITE;
                this.queue.unshift(chunk);
                this.queuedBytes += chunk.length;
            } else {
                this.restart();
                return;
            }
        } finally {
            this.writing = false;
        }
        if (this.writer !== writer) return;
        if (this.queuedBytes < MAX_WRITE) this.onDrain?.();
        void this.pump();
    }

    /** As many whole frames as fit one write; a frame larger than the write size is split. */
    private takeChunk(): Uint8Array<ArrayBuffer> {
        const parts: Uint8Array[] = [];
        let size = 0;
        while (this.queue.length > 0) {
            const next = this.queue[0];
            if (size + next.length <= this.maxWrite) {
                parts.push(next);
                size += next.length;
                this.queue.shift();
            } else {
                if (size === 0) {
                    parts.push(next.subarray(0, this.maxWrite));
                    this.queue[0] = next.subarray(this.maxWrite);
                    size = this.maxWrite;
                }
                break;
            }
        }
        this.queuedBytes -= size;
        const chunk = new Uint8Array(size);
        let offset = 0;
        for (const p of parts) {
            chunk.set(p, offset);
            offset += p.length;
        }
        return chunk;
    }

    private setState(state: LinkState): void {
        if (this.status.state === state) return;
        this.status = {...this.status, state};
        this.emit();
    }

    private emit(): void {
        const status = this.status;
        this.listeners.forEach((l) => l(status));
    }
}

function withTimeout<T>(promise: Promise<T>, ms: number): Promise<T> {
    return new Promise<T>((resolve, reject) => {
        const timer = setTimeout(() => reject(new Error("timed out")), ms);
        promise.then((v) => {
            clearTimeout(timer);
            resolve(v);
        }, (e: unknown) => {
            clearTimeout(timer);
            reject(e instanceof Error ? e : new Error(String(e)));
        });
    });
}
