import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {pack} from "msgpackr";
import {
    HEARTBEAT_TOPIC, LIVENESS_TIMEOUT_MS, PUBLISH_HEARTBEAT, PublishSocket, reconnectDelayMs,
} from "./reconnect.ts";
import {MultiplexedSocket} from "./multiplexedSocket.ts";

class FakeWS {
    static all: FakeWS[] = [];
    readyState = 0;
    binaryType = "";
    sent: string[] = [];
    closed = false;
    onopen: ((e?: unknown) => void) | null = null;
    onclose: ((e?: unknown) => void) | null = null;
    onerror: ((e?: unknown) => void) | null = null;
    onmessage: ((e: {data: unknown}) => void) | null = null;
    constructor(public url: string) { FakeWS.all.push(this); }
    send(d: string) { this.sent.push(d); }
    close() { this.closed = true; this.readyState = 3; }
    // test helpers
    open() { this.readyState = 1; this.onopen?.(); }
    drop() { this.readyState = 3; this.onclose?.(); }
    msg(data: unknown) { this.onmessage?.({data}); }
}
const last = () => FakeWS.all[FakeWS.all.length - 1];

beforeEach(() => {
    FakeWS.all = [];
    vi.useFakeTimers();
    vi.stubGlobal("WebSocket", FakeWS);
});
afterEach(() => {
    vi.useRealTimers();
    vi.unstubAllGlobals();
});

describe("reconnectDelayMs", () => {
    it("is fast and capped at 1 s", () => {
        expect([0, 1, 2, 3, 10].map(reconnectDelayMs)).toEqual([250, 500, 1000, 1000, 1000]);
    });
});

describe("PublishSocket (joystick)", () => {
    const made: PublishSocket[] = [];
    afterEach(() => { made.splice(0).forEach((s) => s.stop()); });
    const make = () => {
        const s = new PublishSocket("ws://x/api/mowglinext/publish/joy",
            (u) => new FakeWS(u) as unknown as WebSocket, () => Date.now());
        made.push(s);
        return s;
    };

    it("drops (never queues) messages while disconnected", () => {
        const s = make();
        s.start();
        expect(s.send({v: 1})).toBe(false);
        last().open();
        expect(s.send({v: 2})).toBe(true);
        last().drop();
        expect(s.send({v: 3})).toBe(false);
        vi.advanceTimersByTime(250);
        last().open();
        // only the message sent while open was ever written; nothing replayed
        expect(FakeWS.all.flatMap((w) => w.sent)).toEqual([JSON.stringify({v: 2})]);
    });

    it("reconnects within ~1 s and reports reconnecting -> open", () => {
        const s = make();
        const seen: string[] = [];
        s.onStatusChange((st) => seen.push(st));
        s.start();
        last().open();
        last().drop();
        expect(s.getStatus()).toBe("reconnecting");
        vi.advanceTimersByTime(249);
        expect(FakeWS.all).toHaveLength(1);
        vi.advanceTimersByTime(1);
        expect(FakeWS.all).toHaveLength(2);
        last().open();
        expect(s.getStatus()).toBe("open");
        expect(seen).toEqual(["connecting", "open", "reconnecting", "open"]);
    });

    it("keeps retrying with the delay capped at 1 s", () => {
        const s = make();
        s.start();
        last().drop(); // never opened
        vi.advanceTimersByTime(250);
        last().drop();
        vi.advanceTimersByTime(500);
        last().drop();
        vi.advanceTimersByTime(1000);
        last().drop();
        vi.advanceTimersByTime(1000);
        expect(FakeWS.all).toHaveLength(5);
    });

    it("treats a silent (half-open) socket as dead and reconnects", () => {
        const s = make();
        s.start();
        const first = last();
        first.open();
        for (let i = 0; i < 5; i++) {
            vi.advanceTimersByTime(1000);
            first.msg(PUBLISH_HEARTBEAT); // heartbeats keep it alive
        }
        expect(s.getStatus()).toBe("open");
        vi.advanceTimersByTime(LIVENESS_TIMEOUT_MS + 600); // heartbeats stop
        expect(first.closed).toBe(true);
        expect(s.getStatus()).toBe("reconnecting");
        vi.advanceTimersByTime(250);
        expect(FakeWS.all).toHaveLength(2);
    });

    it("retries immediately when the network comes back", () => {
        const s = make();
        s.start();
        last().open();
        last().drop();
        window.dispatchEvent(new Event("online"));
        expect(FakeWS.all).toHaveLength(2);
        s.stop();
    });

    it("stop() closes and does not reconnect", () => {
        const s = make();
        s.start();
        last().open();
        s.stop();
        vi.advanceTimersByTime(5000);
        expect(FakeWS.all).toHaveLength(1);
        expect(s.getStatus()).toBe("idle");
    });
});

describe("MultiplexedSocket reconnect", () => {
    const frame = (topic: string, data: unknown) => {
        const b = pack({topic, data});
        const ab = new ArrayBuffer(b.length);
        new Uint8Array(ab).set(b);
        return ab;
    };

    it("re-subscribes every topic after a drop and re-delivers state", () => {
        const m = new MultiplexedSocket("ws://x/mux");
        const got: unknown[] = [];
        m.subscribe("highLevelStatus", (d) => got.push(d));
        m.subscribe("recordingTrajectory", () => {});
        last().open();
        expect(last().sent.map((s) => JSON.parse(s).topic)).toEqual(["highLevelStatus", "recordingTrajectory"]);
        last().msg(frame("highLevelStatus", {state_name: "RECORDING"}));
        last().drop();
        expect(m.getStatus()).toBe("closed");
        expect(m.wasEverOpen()).toBe(true);
        vi.advanceTimersByTime(250);
        last().open();
        expect(last().sent.map((s) => JSON.parse(s).topic)).toEqual(["highLevelStatus", "recordingTrajectory"]);
        last().msg(frame("highLevelStatus", {state_name: "RECORDING"}));
        expect(got).toEqual([{state_name: "RECORDING"}, {state_name: "RECORDING"}]);
    });

    it("heartbeat frames keep it alive; silence triggers a reconnect", () => {
        const m = new MultiplexedSocket("ws://x/mux");
        m.subscribe("emergency", () => {});
        const first = last();
        first.open();
        for (let i = 0; i < 5; i++) {
            vi.advanceTimersByTime(1000);
            first.msg(frame(HEARTBEAT_TOPIC, null));
        }
        expect(m.getStatus()).toBe("open");
        vi.advanceTimersByTime(LIVENESS_TIMEOUT_MS + 600);
        expect(m.getStatus()).not.toBe("open");
        vi.advanceTimersByTime(250);
        expect(FakeWS.all).toHaveLength(2);
    });

    it("backoff never exceeds 1 s", () => {
        const m = new MultiplexedSocket("ws://x/mux");
        m.subscribe("status", () => {});
        for (let i = 0; i < 6; i++) {
            last().drop();
            vi.advanceTimersByTime(1000);
        }
        expect(FakeWS.all).toHaveLength(7);
    });
});
