import {beforeEach, describe, expect, it, vi} from "vitest";
import type {SocketLike, Transport, TransportKind} from "./types.ts";
import {TRANSPORT_SWITCHED_CLOSE_CODE} from "./types.ts";

class FakeSocket implements SocketLike {
    readyState = 0;
    binaryType: BinaryType = "blob";
    onopen: ((ev: Event) => void) | null = null;
    onmessage: ((ev: MessageEvent) => void) | null = null;
    onclose: ((ev: CloseEvent) => void) | null = null;
    onerror: ((ev: Event) => void) | null = null;
    closedWith: [number | undefined, string | undefined] | null = null;
    sent: unknown[] = [];
    constructor(readonly url: string) {}
    send(data: unknown) { this.sent.push(data); }
    close(code?: number, reason?: string) { this.closedWith = [code, reason]; this.readyState = 3; }
    // test helper: the transport reports the socket closed
    fireClose(code = 1000) { this.readyState = 3; this.onclose?.(new CloseEvent("close", {code})); }
}

class FakeTransport implements Transport {
    sockets: FakeSocket[] = [];
    constructor(readonly kind: TransportKind) {}
    fetch = vi.fn((path: string) => Promise.resolve(new Response(`${this.kind}:${path}`)));
    openSocket(path: string) {
        const s = new FakeSocket(`${this.kind}:${path}`);
        this.sockets.push(s);
        return s;
    }
}

type Seam = typeof import("./index.ts");
let seam: Seam;
let net: FakeTransport;
let ble: FakeTransport;

beforeEach(async () => {
    vi.resetModules();
    seam = await import("./index.ts");
    net = new FakeTransport("network");
    ble = new FakeTransport("ble");
    seam.registerTransport("network", () => net);
});

describe("transport registry", () => {
    it("starts on network and routes fetch and sockets to the active transport", async () => {
        expect(seam.getActiveKind()).toBe("network");
        expect(seam.getActiveTransport()).toBe(net);
        expect(await (await seam.apiFetch("/api/x")).text()).toBe("network:/api/x");
        seam.registerTransport("ble", () => ble);
        seam.setActiveTransport("ble");
        expect(await (await seam.apiFetch("/api/x", {method: "POST"})).text()).toBe("ble:/api/x");
        expect(ble.fetch).toHaveBeenCalledWith("/api/x", {method: "POST"});
        expect(seam.openApiSocket("/api/s").url).toBe("ble:/api/s");
    });

    it("calls each factory lazily and once", () => {
        const factory = vi.fn(() => ble);
        seam.registerTransport("ble", factory);
        expect(factory).not.toHaveBeenCalled();
        seam.setActiveTransport("ble");
        seam.getActiveTransport();
        seam.getActiveTransport();
        expect(factory).toHaveBeenCalledTimes(1);
    });

    it("throws for an unregistered kind and stays on the current one", () => {
        expect(() => seam.setActiveTransport("ble")).toThrow(/not registered/);
        expect(seam.getActiveKind()).toBe("network");
    });

    it("switching closes every live seam socket with 4000, then notifies listeners", () => {
        seam.registerTransport("ble", () => ble);
        const a = seam.openApiSocket("/api/a");
        const b = seam.openApiSocket("/api/b", {prio: true});
        const order: string[] = [];
        const closes: CloseEvent[] = [];
        a.onclose = (ev) => { order.push("close a"); closes.push(ev); };
        b.onclose = (ev) => { order.push("close b"); closes.push(ev); };
        seam.onTransportChange((kind) => order.push(`change ${kind}`));

        seam.setActiveTransport("ble");

        expect(order).toEqual(["close a", "close b", "change ble"]);
        expect(closes.map((e) => [e.code, e.reason])).toEqual([
            [TRANSPORT_SWITCHED_CLOSE_CODE, "transport switched"],
            [TRANSPORT_SWITCHED_CLOSE_CODE, "transport switched"],
        ]);
        expect(net.sockets.map((s) => s.closedWith)).toEqual([
            [4000, "transport switched"], [4000, "transport switched"],
        ]);
    });

    it("a socket reopened from onclose lands on the new transport and is not closed again", () => {
        seam.registerTransport("ble", () => ble);
        const a = seam.openApiSocket("/api/a");
        let reopened: SocketLike | null = null;
        a.onclose = () => { reopened = seam.openApiSocket("/api/a"); };
        seam.setActiveTransport("ble");
        expect(reopened!.url).toBe("ble:/api/a");
        expect(ble.sockets[0].closedWith).toBeNull();
    });

    it("does not touch sockets that already closed", () => {
        seam.registerTransport("ble", () => ble);
        const byOwner = seam.openApiSocket("/api/owner");
        const byPeer = seam.openApiSocket("/api/peer");
        const peerClose = vi.fn();
        byPeer.onclose = peerClose;
        byOwner.close();
        net.sockets[1].fireClose(1006);
        expect(peerClose).toHaveBeenCalledTimes(1);
        seam.setActiveTransport("ble");
        expect(net.sockets[0].closedWith).toEqual([undefined, undefined]);
        expect(peerClose).toHaveBeenCalledTimes(1);
    });

    it("is a no-op when the kind is already active", () => {
        const s = seam.openApiSocket("/api/a");
        const cb = vi.fn();
        seam.onTransportChange(cb);
        seam.setActiveTransport("network");
        expect(cb).not.toHaveBeenCalled();
        expect(s.readyState).toBe(0);
    });

    it("unsubscribed listeners are not called", () => {
        seam.registerTransport("ble", () => ble);
        const cb = vi.fn();
        const off = seam.onTransportChange(cb);
        off();
        seam.setActiveTransport("ble");
        expect(cb).not.toHaveBeenCalled();
    });

    it("forwards events, binaryType and send to the transport socket", () => {
        const s = seam.openApiSocket("/api/a");
        const inner = net.sockets[0];
        s.binaryType = "arraybuffer";
        expect(inner.binaryType).toBe("arraybuffer");
        const onmessage = vi.fn();
        s.onmessage = onmessage;
        inner.onmessage?.(new MessageEvent("message", {data: "x"}));
        expect(onmessage).toHaveBeenCalledTimes(1);
        s.send("hi");
        expect(inner.sent).toEqual(["hi"]);
    });
});
