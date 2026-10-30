import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import type {BleController, BleDevice, BleStatus} from "./ble/bleTransport.ts";
import type {TransportKind} from "./types.ts";
import {ConnectionManager, type ConnectionDeps, STORAGE_KEYS} from "./connectionManager.ts";

class FakeBle implements BleController {
    status: BleStatus = {link: "idle", session: "idle", deviceId: null, deviceName: null, host: null, rttMs: null};
    /** Device in range for connectKnown; null = nothing permitted / in range. */
    known: BleDevice | null = null;
    token = "";
    readonly connectKnown = vi.fn((deviceId: string | null) => {
        void deviceId;
        if (!this.known) return Promise.resolve(null);
        this.set({link: "connected", deviceId: this.known.id, deviceName: this.known.name});
        return Promise.resolve(this.known);
    });
    readonly disconnect = vi.fn(() => this.set({link: "idle"}));
    readonly pairNew = vi.fn(() => {
        const device = {id: "new-device", name: "AIRSEEKERS_ABCDEF"};
        this.set({link: "connected", deviceId: device.id});
        return Promise.resolve(device);
    });
    readonly forget = vi.fn(() => {
        this.set({link: "idle", deviceId: null});
        return Promise.resolve();
    });
    private readonly listeners = new Set<(s: BleStatus) => void>();

    getStatus(): BleStatus {
        return this.status;
    }

    getCounters() {
        return {bytesIn: 0, bytesOut: 0};
    }

    subscribe(listener: (s: BleStatus) => void): () => void {
        this.listeners.add(listener);
        return () => this.unsubscribe(listener);
    }

    unsubscribe(listener: (s: BleStatus) => void): void {
        this.listeners.delete(listener);
    }

    setToken(token: string): void {
        this.token = token;
    }

    set(patch: Partial<BleStatus>): void {
        this.status = {...this.status, ...patch};
        this.listeners.forEach((l) => l(this.status));
    }
}

class MemoryStorage {
    readonly data = new Map<string, string>();
    getItem(key: string) { return this.data.get(key) ?? null; }
    setItem(key: string, value: string) { this.data.set(key, value); }
    removeItem(key: string) { this.data.delete(key); }
}

let ble: FakeBle;
let storage: MemoryStorage;
let networkUp: boolean;
let active: TransportKind[];
let apiFetch: ReturnType<typeof vi.fn<ConnectionDeps["apiFetch"]>>;
let networkFetch: ReturnType<typeof vi.fn<ConnectionDeps["networkFetch"]>>;
let manager: ConnectionManager;

function respond(path: string): Promise<Response> {
    if (!networkUp) return Promise.reject(new TypeError("Failed to fetch"));
    if (path === "/api/ble/token") return Promise.resolve(Response.json({token: "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567"}));
    return Promise.resolve(Response.json({id: "build"}));
}

function create(secure = true): ConnectionManager {
    manager = new ConnectionManager({
        ble,
        bleSupport: () => "ok",
        apiFetch,
        networkFetch,
        setActiveTransport: (kind) => active.push(kind),
        storage: storage as unknown as Storage,
        isSecureContext: () => secure,
    });
    return manager;
}

beforeEach(() => {
    vi.useFakeTimers();
    ble = new FakeBle();
    storage = new MemoryStorage();
    networkUp = true;
    active = [];
    apiFetch = vi.fn((path: string) => respond(path));
    networkFetch = vi.fn((path: string) => respond(path));
});

afterEach(() => {
    manager?.stop();
    vi.useRealTimers();
});

describe("ConnectionManager", () => {
    it("starts on Wi-Fi and probes every 3 s through the seam", async () => {
        create().start();
        await vi.advanceTimersByTimeAsync(0);
        expect(active).toEqual(["network"]);
        expect(manager.getState()).toMatchObject({mode: "wifi", networkAlive: true});
        const probes = () => apiFetch.mock.calls.filter(([p]) => p === "/web-build.json").length;
        expect(probes()).toBe(1);
        expect(apiFetch).toHaveBeenCalledWith("/web-build.json", expect.objectContaining({cache: "no-store"}));
        await vi.advanceTimersByTimeAsync(9000);
        expect(probes()).toBe(4);
    });

    it("fetches the BLE token once while on Wi-Fi and stores it", async () => {
        create().start();
        await vi.advanceTimersByTimeAsync(10000);
        const tokenCalls = apiFetch.mock.calls.filter(([p]) => p === "/api/ble/token");
        expect(tokenCalls).toHaveLength(1);
        expect(storage.data.get(STORAGE_KEYS.token)).toBe("ABCDEFGHIJKLMNOPQRSTUVWXYZ234567");
        expect(manager.getState().token).toBe("ABCDEFGHIJKLMNOPQRSTUVWXYZ234567");
        expect(ble.token).toBe("ABCDEFGHIJKLMNOPQRSTUVWXYZ234567");
    });

    it("does not fetch the token outside a secure context", async () => {
        create(false).start();
        await vi.advanceTimersByTimeAsync(10000);
        expect(apiFetch.mock.calls.filter(([p]) => p === "/api/ble/token")).toHaveLength(0);
    });

    it("loads the stored token and device into the BLE transport", () => {
        storage.setItem(STORAGE_KEYS.token, "STORED");
        storage.setItem(STORAGE_KEYS.deviceId, "dev-1");
        create();
        expect(ble.token).toBe("STORED");
        expect(manager.getState()).toMatchObject({token: "STORED", deviceId: "dev-1"});
    });

    it("switches to Bluetooth by itself when offline at start and a known device is in range", async () => {
        networkUp = false;
        ble.known = {id: "dev-1", name: "AIRSEEKERS_123456"};
        create().start();
        await vi.advanceTimersByTimeAsync(0);
        expect(manager.getState()).toMatchObject({mode: "ble", notice: "autoSwitchedToBle", deviceId: "dev-1"});
        expect(active).toEqual(["network", "ble"]);
        expect(storage.data.get(STORAGE_KEYS.deviceId)).toBe("dev-1");
        // The automatic switch is not stored as the user's choice.
        expect(storage.data.has(STORAGE_KEYS.mode)).toBe(false);
        manager.dismissNotice();
        expect(manager.getState().notice).toBeNull();
    });

    it("switches after the network has been dead for 3 s", async () => {
        ble.known = {id: "dev-1", name: "AIRSEEKERS_123456"};
        create().start();
        await vi.advanceTimersByTimeAsync(0);
        expect(manager.getState().mode).toBe("wifi");
        networkUp = false;
        await vi.advanceTimersByTimeAsync(3000); // probe at 3 s fails, last alive was 0 s
        expect(manager.getState().mode).toBe("ble");
        expect(ble.connectKnown).toHaveBeenCalledTimes(1);
    });

    it("stays on Wi-Fi when no permitted device is in range", async () => {
        networkUp = false;
        create().start();
        await vi.advanceTimersByTimeAsync(0);
        expect(ble.connectKnown).toHaveBeenCalledTimes(1);
        expect(manager.getState()).toMatchObject({mode: "wifi", networkAlive: false, notice: null});
        // Retries at most every 15 s during one outage.
        await vi.advanceTimersByTimeAsync(12000);
        expect(ble.connectKnown).toHaveBeenCalledTimes(1);
        await vi.advanceTimersByTimeAsync(3000);
        expect(ble.connectKnown).toHaveBeenCalledTimes(2);
        expect(active).toEqual(["network"]);
    });

    it("never switches back to Wi-Fi by itself", async () => {
        networkUp = false;
        ble.known = {id: "dev-1", name: "AIRSEEKERS_123456"};
        create().start();
        await vi.advanceTimersByTimeAsync(0);
        expect(manager.getState().mode).toBe("ble");
        networkUp = true;
        await vi.advanceTimersByTimeAsync(30000);
        expect(manager.getState()).toMatchObject({mode: "ble", networkAlive: true});
        expect(active).toEqual(["network", "ble"]);
        // On Bluetooth the probe bypasses the seam and runs every 10 s.
        expect(networkFetch).toHaveBeenCalledTimes(3);
    });

    it("switches manually both ways and remembers the choice", async () => {
        ble.known = {id: "dev-1", name: "AIRSEEKERS_123456"};
        create().start();
        await vi.advanceTimersByTimeAsync(0);
        await manager.switchTo("ble");
        expect(manager.getState().mode).toBe("ble");
        expect(ble.status.link).toBe("connected");
        expect(storage.data.get(STORAGE_KEYS.mode)).toBe("ble");
        await manager.switchTo("wifi");
        expect(manager.getState().mode).toBe("wifi");
        expect(ble.disconnect).toHaveBeenCalled();
        expect(storage.data.get(STORAGE_KEYS.mode)).toBe("wifi");
        expect(active).toEqual(["network", "ble", "network"]);
    });

    it("starts on Bluetooth when that was the stored choice", async () => {
        storage.setItem(STORAGE_KEYS.mode, "ble");
        storage.setItem(STORAGE_KEYS.deviceId, "dev-1");
        ble.known = {id: "dev-1", name: "AIRSEEKERS_123456"};
        create().start();
        await vi.advanceTimersByTimeAsync(0);
        expect(active).toEqual(["ble"]);
        expect(ble.connectKnown).toHaveBeenCalledWith("dev-1");
    });

    it("pairs a new device, remembers it and uses Bluetooth; forgetting returns to Wi-Fi", async () => {
        create().start();
        await vi.advanceTimersByTimeAsync(0);
        await manager.pairNew();
        expect(manager.getState()).toMatchObject({mode: "ble", deviceId: "new-device"});
        expect(storage.data.get(STORAGE_KEYS.deviceId)).toBe("new-device");
        await manager.forgetDevice();
        expect(manager.getState()).toMatchObject({mode: "wifi", deviceId: null});
        expect(storage.data.has(STORAGE_KEYS.deviceId)).toBe(false);
    });

    it("survives storage that throws", () => {
        const broken = {
            getItem: () => { throw new Error("blocked"); },
            setItem: () => { throw new Error("blocked"); },
            removeItem: () => { throw new Error("blocked"); },
        };
        storage = broken as unknown as MemoryStorage;
        create();
        expect(() => manager.setToken("X")).not.toThrow();
        expect(manager.getState().token).toBe("X");
    });
});
