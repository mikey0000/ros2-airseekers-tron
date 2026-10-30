import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {FALLBACK_WRITE, GattLink, MAX_WRITE, NAME_PREFIX, NOTIFY_UUID, SERVICE_UUID, WRITE_UUID} from "./gattLink.ts";

/** A scripted Web Bluetooth device: writes resolve when the test says so. */
class FakeCharacteristic extends EventTarget {
    value?: DataView;
    readonly writes: Uint8Array[] = [];
    pending: {resolve: () => void; reject: (e: unknown) => void}[] = [];
    rejectAbove = Infinity;

    constructor(readonly uuid: string) {
        super();
    }

    startNotifications() {
        return Promise.resolve(this as unknown as BluetoothRemoteGATTCharacteristic);
    }

    stopNotifications() {
        return this.startNotifications();
    }

    writeValueWithResponse(value: BufferSource): Promise<void> {
        const bytes = new Uint8Array(value as ArrayBuffer);
        if (bytes.length > this.rejectAbove) return Promise.reject(new DOMException("too long", "NotSupportedError"));
        this.writes.push(bytes);
        return new Promise((resolve, reject) => this.pending.push({resolve, reject}));
    }

    notify(bytes: Uint8Array): void {
        this.value = new DataView(bytes.buffer.slice(0) as ArrayBuffer);
        this.dispatchEvent(new Event("characteristicvaluechanged"));
    }
}

class FakeDevice extends EventTarget {
    readonly notifier = new FakeCharacteristic(NOTIFY_UUID);
    readonly writer = new FakeCharacteristic(WRITE_UUID);
    connected = false;
    connectFails = 0;
    connects = 0;
    readonly gatt = {
        connected: false,
        connect: () => {
            this.connects++;
            if (this.connectFails > 0) {
                this.connectFails--;
                return Promise.reject(new Error("NetworkError"));
            }
            this.gatt.connected = true;
            return Promise.resolve(this.gatt);
        },
        disconnect: () => {
            if (!this.gatt.connected) return;
            this.gatt.connected = false;
            this.dispatchEvent(new Event("gattserverdisconnected"));
        },
        getPrimaryService: (uuid: string) => {
            expect(uuid).toBe(SERVICE_UUID);
            return Promise.resolve({
                getCharacteristic: (c: string) => Promise.resolve(c === NOTIFY_UUID ? this.notifier : this.writer),
            });
        },
    };

    constructor(readonly id: string, readonly name: string) {
        super();
    }

    /** The mower walked away. */
    drop(): void {
        this.gatt.connected = false;
        this.dispatchEvent(new Event("gattserverdisconnected"));
    }
}

let device: FakeDevice;
let requestDevice: ReturnType<typeof vi.fn>;
let link: GattLink;

beforeEach(() => {
    vi.useFakeTimers();
    device = new FakeDevice("dev-1", `${NAME_PREFIX}ABC123`);
    requestDevice = vi.fn(() => Promise.resolve(device));
    const bluetooth = {
        requestDevice,
        getDevices: () => Promise.resolve([device]),
        getAvailability: () => Promise.resolve(true),
    } as unknown as Bluetooth;
    link = new GattLink(bluetooth);
});

afterEach(() => {
    link.disconnect();
    vi.useRealTimers();
});

async function settle(): Promise<void> {
    await vi.advanceTimersByTimeAsync(0);
}

describe("GattLink", () => {
    it("pairs with the name-prefix filter and the vendor service", async () => {
        const up = vi.fn();
        link.onUp = up;
        await link.pairNew();
        expect(requestDevice).toHaveBeenCalledWith({filters: [{namePrefix: "AIRSEEKERS_"}], optionalServices: [SERVICE_UUID]});
        expect(link.getStatus()).toEqual({state: "connected", deviceId: "dev-1", deviceName: "AIRSEEKERS_ABC123"});
        expect(up).toHaveBeenCalledTimes(1);
    });

    it("connects to a previously permitted device without a gesture", async () => {
        expect(await link.connectKnown("dev-1")).toBe(true);
        expect(requestDevice).not.toHaveBeenCalled();
        expect(link.getStatus().state).toBe("connected");
    });

    it("delivers notifications and counts bytes", async () => {
        await link.connectKnown(null);
        const received: number[][] = [];
        link.onBytes = (b) => received.push([...b]);
        device.notifier.notify(Uint8Array.of(1, 2, 3));
        expect(received).toEqual([[1, 2, 3]]);
        expect(link.getCounters().bytesIn).toBe(3);
    });

    it("keeps one write in flight and packs whole frames up to 497 bytes", async () => {
        await link.connectKnown(null);
        link.write(new Uint8Array(100));
        link.write(new Uint8Array(300));
        link.write(new Uint8Array(150));
        link.write(new Uint8Array(150));
        expect(device.writer.writes.map((w) => w.length)).toEqual([100]);
        device.writer.pending.shift()!.resolve();
        await settle();
        // 300 + 150 fit 497; the next 150 would not.
        expect(device.writer.writes.map((w) => w.length)).toEqual([100, 450]);
        device.writer.pending.shift()!.resolve();
        await settle();
        expect(device.writer.writes.map((w) => w.length)).toEqual([100, 450, 150]);
        device.writer.pending.shift()!.resolve();
        await settle();
        expect(link.getCounters().bytesOut).toBe(700);
        expect(link.bufferedAmount).toBe(0);
    });

    it("falls back to 244-byte writes when a 497-byte write is rejected", async () => {
        await link.connectKnown(null);
        device.writer.rejectAbove = FALLBACK_WRITE;
        link.write(new Uint8Array(490));
        await settle();
        expect(device.writer.writes.map((w) => w.length)).toEqual([244]);
        device.writer.pending.shift()!.resolve();
        await settle();
        device.writer.pending.shift()!.resolve();
        await settle();
        expect(device.writer.writes.map((w) => w.length)).toEqual([244, 244, 2]);
        expect(MAX_WRITE).toBe(497);
    });

    it("reconnects patiently with backoff up to 30 s after an unexpected drop", async () => {
        await link.connectKnown(null);
        const down = vi.fn();
        link.onDown = down;
        device.connectFails = 10;
        device.drop();
        expect(down).toHaveBeenCalledTimes(1);
        expect(link.getStatus().state).toBe("reconnecting");
        const connectsAt = (ms: number) => vi.advanceTimersByTimeAsync(ms).then(() => device.connects);
        const base = device.connects;
        expect(await connectsAt(1000)).toBe(base + 1);   // 1 s
        expect(await connectsAt(2000)).toBe(base + 2);   // 2 s
        expect(await connectsAt(4000)).toBe(base + 3);   // 4 s
        expect(await connectsAt(8000)).toBe(base + 4);   // 8 s
        expect(await connectsAt(16000)).toBe(base + 5);  // 16 s
        expect(await connectsAt(29999)).toBe(base + 5);  // capped at 30 s
        expect(await connectsAt(1)).toBe(base + 6);
        device.connectFails = 0;
        await vi.advanceTimersByTimeAsync(30000);
        expect(link.getStatus().state).toBe("connected");
    });

    it("stops reconnecting when told to disconnect", async () => {
        await link.connectKnown(null);
        device.connectFails = 100;
        device.drop();
        link.disconnect();
        const connects = device.connects;
        await vi.advanceTimersByTimeAsync(120000);
        expect(device.connects).toBe(connects);
        expect(link.getStatus().state).toBe("idle");
    });
});
