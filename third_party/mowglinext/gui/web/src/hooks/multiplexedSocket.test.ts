import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {pack} from "msgpackr";
import {MultiplexedSocket} from "./multiplexedSocket.ts";

class FakeWS {
    static all: FakeWS[] = [];
    readyState = 0;
    binaryType = "";
    sent: string[] = [];
    onopen: (() => void) | null = null;
    onclose: (() => void) | null = null;
    onerror: (() => void) | null = null;
    onmessage: ((e: {data: unknown}) => void) | null = null;
    constructor(public url: string) { FakeWS.all.push(this); }
    send(d: string) { this.sent.push(d); }
    close() { this.readyState = 3; }
    open() { this.readyState = 1; this.onopen?.(); }
    msg(topic: string, data: unknown) {
        const b = pack({topic, data});
        const ab = new ArrayBuffer(b.length);
        new Uint8Array(ab).set(b);
        this.onmessage?.({data: ab});
    }
}
const last = () => FakeWS.all[FakeWS.all.length - 1];
const flush = () => new Promise<void>((r) => queueMicrotask(r));

beforeEach(() => {
    FakeWS.all = [];
    vi.stubGlobal("WebSocket", FakeWS);
});
afterEach(() => vi.unstubAllGlobals());

describe("MultiplexedSocket late subscribers", () => {
    it("hands a latched state payload to a listener that subscribes after it arrived", async () => {
        const m = new MultiplexedSocket("/mux");
        m.subscribe("coverageResumeAvailable", () => {});
        last().open();
        last().msg("coverageResumeAvailable", {data: true});

        const late = vi.fn();
        m.subscribe("coverageResumeAvailable", late);
        expect(late).not.toHaveBeenCalled(); // never synchronously inside subscribe()
        await flush();
        expect(late).toHaveBeenCalledWith({data: true}, true);
        // The server keeps one upstream subscription per topic: no second subscribe op.
        expect(last().sent.filter((s) => JSON.parse(s).topic === "coverageResumeAvailable")).toHaveLength(1);
    });

    it("does not replay event topics or a topic everyone left", async () => {
        const m = new MultiplexedSocket("/mux");
        m.subscribe("alerts", () => {});
        const off = m.subscribe("power", () => {});
        m.subscribe("status", () => {});
        last().open();
        last().msg("alerts", {data: "{}"});
        last().msg("power", {v_battery: 26});
        off();

        const alerts = vi.fn();
        const power = vi.fn();
        m.subscribe("alerts", alerts);
        m.subscribe("power", power);
        await flush();
        expect(alerts).not.toHaveBeenCalled();
        expect(power).not.toHaveBeenCalled();
    });

    it("skips the replay when the listener unsubscribed first", async () => {
        const m = new MultiplexedSocket("/mux");
        m.subscribe("map", () => {});
        last().open();
        last().msg("map", {areas: []});
        const late = vi.fn();
        m.subscribe("map", late)();
        await flush();
        expect(late).not.toHaveBeenCalled();
    });
});
