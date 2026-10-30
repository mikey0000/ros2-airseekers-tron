import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {encodeFrame, type Frame, FrameKind, FrameParser, fromUtf8, jsonPayload, parseJsonPayload, utf8} from "./frame.ts";
import {type ByteLink, Tunnel} from "./tunnel.ts";

/** Scripted link: captures what the tunnel writes, feeds mower frames with a running seq. */
class FakeLink implements ByteLink {
    onBytes: ((bytes: Uint8Array) => void) | null = null;
    onDown: (() => void) | null = null;
    onDrain: (() => void) | null = null;
    readonly sent: Frame[] = [];
    private readonly parser = new FrameParser((f) => this.sent.push(f));
    private seq = 0;

    write(bytes: Uint8Array): void {
        this.parser.push(bytes);
    }

    /** Send a mower frame; returns its wire length. */
    feed(stream: number, kind: number, payload: Uint8Array = new Uint8Array(0), seq = this.seq): number {
        this.seq = (seq + 1) & 0xff;
        const wire = encodeFrame({stream, kind, seq, payload});
        this.onBytes?.(wire);
        return wire.length;
    }

    feedJson(stream: number, kind: number, value: unknown): number {
        return this.feed(stream, kind, jsonPayload(value));
    }

    resetSeq(): void {
        this.seq = 0;
    }

    kinds(kind: number): Frame[] {
        return this.sent.filter((f) => f.kind === kind);
    }

    take(): Frame[] {
        return this.sent.splice(0);
    }
}

let link: FakeLink;
let tunnel: Tunnel;

/** Start a session and answer WELCOME; returns the WELCOME wire length. */
function welcome(): number {
    tunnel.start("TOKEN");
    return link.feedJson(0, FrameKind.WELCOME, {v: 1, host: "mower", max_payload: 480, session: "0badcafe"});
}

const ignore = () => undefined;

/** Let promise continuations (whenReady, gzip) run. */
async function flush(): Promise<void> {
    for (let i = 0; i < 20; i++) await Promise.resolve();
}

beforeEach(() => {
    vi.useFakeTimers();
    link = new FakeLink();
    tunnel = new Tunnel(link, {client: "test"});
});

afterEach(() => {
    tunnel.stop();
    vi.useRealTimers();
});

describe("session", () => {
    it("sends HELLO first and becomes ready on WELCOME", () => {
        tunnel.start("TOKEN");
        const [hello] = link.take();
        expect(hello.kind).toBe(FrameKind.HELLO);
        expect(hello.seq).toBe(0);
        expect(parseJsonPayload(hello.payload)).toEqual({v: 1, token: "TOKEN", client: "test", window: 2048, max_rate: 5000});
        expect(tunnel.getStatus().state).toBe("handshaking");
        link.feedJson(0, FrameKind.WELCOME, {v: 1, host: "mower", max_payload: 480, session: "0badcafe"});
        expect(tunnel.getStatus()).toMatchObject({state: "ready", host: "mower", session: "0badcafe"});
    });

    it("goes unauthenticated on a rejected token and fails pending requests", async () => {
        tunnel.start("WRONG");
        const pending = tunnel.openHttp("/api/settings");
        await flush();
        link.feedJson(0, FrameKind.WELCOME, {v: 1, error: "unauthenticated"});
        await expect(pending).rejects.toThrow(TypeError);
        expect(tunnel.getStatus().state).toBe("unauthenticated");
        await expect(tunnel.openHttp("/api/settings")).rejects.toThrow(TypeError);
    });

    it("reports backend_unavailable", () => {
        tunnel.start("TOKEN");
        link.feedJson(0, FrameKind.WELCOME, {v: 1, error: "backend_unavailable"});
        expect(tunnel.getStatus().state).toBe("backend_unavailable");
    });

    it("pings every second and measures the round trip", () => {
        welcome();
        link.take();
        vi.advanceTimersByTime(1000);
        const pings = link.kinds(FrameKind.PING);
        expect(pings).toHaveLength(1);
        expect(pings[0].payload).toHaveLength(8);
        vi.advanceTimersByTime(80);
        link.feed(0, FrameKind.PONG, pings[0].payload);
        expect(tunnel.getStatus().rttMs).toBe(80);
        vi.advanceTimersByTime(920);
        expect(link.kinds(FrameKind.PING)).toHaveLength(2);
    });

    it("declares the link dead after 3 s without a frame", () => {
        welcome();
        const states: string[] = [];
        tunnel.subscribe((s) => states.push(s.state));
        vi.advanceTimersByTime(2000);
        link.feed(0, FrameKind.PONG, new Uint8Array(8));
        vi.advanceTimersByTime(2900);
        expect(tunnel.getStatus().state).toBe("ready");
        vi.advanceTimersByTime(200);
        expect(tunnel.getStatus().state).toBe("dead");
        expect(states).toEqual(["dead"]);
        const pings = link.kinds(FrameKind.PING).length;
        vi.advanceTimersByTime(5000);
        expect(link.kinds(FrameKind.PING)).toHaveLength(pings);
    });

    it("times out a handshake that never gets WELCOME", () => {
        tunnel.start("TOKEN");
        vi.advanceTimersByTime(3000);
        expect(tunnel.getStatus().state).toBe("dead");
    });

    it("link down fails every stream", async () => {
        welcome();
        const request = tunnel.openHttp("/api/settings");
        await flush();
        link.onDown?.();
        await expect(request).rejects.toThrow(TypeError);
        expect(tunnel.getStatus().state).toBe("idle");
    });
});

describe("credits", () => {
    it("credits cumulative wire bytes once window/4 arrived", () => {
        let total = welcome();
        link.take();
        // 512 = 2048 / 4. Frames of 108 wire bytes (100-byte payload) on an unknown stream.
        while (total + 108 < 512) {
            total += link.feed(9, FrameKind.DATA, new Uint8Array(100));
            expect(link.kinds(FrameKind.CREDIT)).toHaveLength(0);
        }
        total += link.feed(9, FrameKind.DATA, new Uint8Array(100));
        const credits = link.kinds(FrameKind.CREDIT);
        expect(credits).toHaveLength(1);
        expect(new DataView(credits[0].payload.buffer).getUint32(0, true)).toBe(total);
    });

    it("credits after 250 ms when fewer bytes arrived", () => {
        tunnel.start("TOKEN");
        link.take();
        const wire = link.feedJson(0, FrameKind.WELCOME, {v: 1, host: "m", max_payload: 480, session: "1"});
        vi.advanceTimersByTime(249);
        expect(link.kinds(FrameKind.CREDIT)).toHaveLength(0);
        vi.advanceTimersByTime(1);
        const credits = link.kinds(FrameKind.CREDIT);
        expect(credits).toHaveLength(1);
        expect(new DataView(credits[0].payload.buffer).getUint32(0, true)).toBe(wire);
        vi.advanceTimersByTime(500);
        expect(link.kinds(FrameKind.CREDIT)).toHaveLength(1);
    });
});

describe("sequence", () => {
    it("numbers outgoing frames per session", () => {
        welcome();
        tunnel.openHttp("/api/a").catch(ignore);
        return flush().then(() => {
            expect(link.sent.map((f) => f.seq)).toEqual([0, 1, 2]);
        });
    });

    it("sends RESET on a gap, fails streams and resyncs on the mower's seq 0", async () => {
        welcome();
        const socket = tunnel.openSocket("/api/mowglinext/multiplex");
        const closes: number[] = [];
        socket.onclose = (ev) => closes.push(ev.code);
        await flush();
        link.take();
        link.feed(1, FrameKind.HEAD, jsonPayload({status: 101}));
        expect(socket.readyState).toBe(1);
        link.feed(0, FrameKind.PONG, new Uint8Array(8), 9); // expected 2, got 9
        const [reset] = link.take();
        expect(reset.kind).toBe(FrameKind.RESET);
        expect(reset.seq).toBe(0);
        expect(closes).toEqual([1006]);
        // Late frames of the old numbering are dropped, not answered with another RESET.
        link.feed(1, FrameKind.DATA, new Uint8Array(4), 10);
        expect(link.sent).toHaveLength(0);
        link.feed(0, FrameKind.PONG, new Uint8Array(8), 0);
        vi.advanceTimersByTime(1000);
        const after = link.take();
        expect(after.map((f) => f.kind)).toContain(FrameKind.PING);
        expect(after.map((f) => f.kind)).not.toContain(FrameKind.RESET);
        expect(after[0].seq).toBe(0);
        expect(tunnel.getStatus().state).toBe("ready");
    });

    it("handles a RESET from the mower", async () => {
        welcome();
        const request = tunnel.openHttp("/api/a");
        await flush();
        link.feed(0, FrameKind.RESET);
        await expect(request).rejects.toThrow(TypeError);
        link.take();
        link.resetSeq();
        link.feed(0, FrameKind.PONG, new Uint8Array(8));
        vi.advanceTimersByTime(1000);
        expect(link.sent[0].seq).toBe(0);
        expect(link.kinds(FrameKind.PING)).toHaveLength(1);
        expect(link.kinds(FrameKind.RESET)).toHaveLength(0);
    });
});

describe("HTTP", () => {
    it("round-trips a GET with a JSON response", async () => {
        welcome();
        link.take();
        const pending = tunnel.openHttp("/api/settings", {headers: {Accept: "application/json", "X-Other": "dropped"}});
        await flush();
        const [open, end] = link.take();
        expect(open).toMatchObject({stream: 1, kind: FrameKind.OPEN});
        expect(parseJsonPayload(open.payload)).toEqual({t: "http", method: "GET", path: "/api/settings", headers: {accept: "application/json"}});
        expect(end).toMatchObject({stream: 1, kind: FrameKind.END});
        expect(end.payload).toHaveLength(0);
        link.feedJson(1, FrameKind.HEAD, {status: 200, headers: {"content-type": "application/json"}});
        link.feed(1, FrameKind.DATA, utf8('{"a":'));
        link.feed(1, FrameKind.END, utf8("1}"));
        link.feedJson(1, FrameKind.CLOSE, {code: 1000, reason: ""});
        const response = await pending;
        expect(response).toBeInstanceOf(Response);
        expect(response.status).toBe(200);
        expect(response.headers.get("content-type")).toBe("application/json");
        expect(await response.json()).toEqual({a: 1});
    });

    it("sends a body as DATA* END and inflates a gzip response", async () => {
        welcome();
        link.take();
        const body = "x".repeat(1000);
        const pending = tunnel.openHttp("/api/settings", {method: "post", body});
        await flush();
        const frames = link.take();
        expect(frames.map((f) => f.kind)).toEqual([FrameKind.OPEN, FrameKind.DATA, FrameKind.DATA, FrameKind.END]);
        expect(parseJsonPayload(frames[0].payload)).toMatchObject({method: "POST", headers: {"content-type": "text/plain;charset=UTF-8"}});
        expect(frames.slice(1).map((f) => f.payload.length)).toEqual([480, 480, 40]);

        const json = JSON.stringify({items: Array.from({length: 200}, (_, i) => i)});
        const gz = new Uint8Array(await new Response(new Response(utf8(json)).body!.pipeThrough(new CompressionStream("gzip"))).arrayBuffer());
        link.feedJson(1, FrameKind.HEAD, {status: 200, headers: {"content-type": "application/json", "content-encoding": "gzip"}});
        for (let i = 0; i < gz.length; i += 480) {
            link.feed(1, i + 480 >= gz.length ? FrameKind.END : FrameKind.DATA, gz.subarray(i, i + 480));
        }
        const response = await pending;
        expect(response.headers.get("content-encoding")).toBeNull();
        expect(await response.text()).toBe(json);
    });

    it("resolves a HEAD + CLOSE answer with an empty body", async () => {
        welcome();
        const pending = tunnel.openHttp("/etc/passwd");
        await flush();
        link.feedJson(1, FrameKind.HEAD, {status: 403});
        link.feed(1, FrameKind.CLOSE);
        const response = await pending;
        expect(response.status).toBe(403);
        expect(await response.text()).toBe("");
    });

    it("resolves an empty backend body (HEAD with headers, empty END)", async () => {
        welcome();
        const pending = tunnel.openHttp("/api/empty", {method: "DELETE"});
        await flush();
        link.feedJson(1, FrameKind.HEAD, {status: 204, headers: {}});
        link.feed(1, FrameKind.END);
        link.feed(1, FrameKind.CLOSE);
        expect((await pending).status).toBe(204);
    });

    it("rejects when the stream closes before END", async () => {
        welcome();
        const pending = tunnel.openHttp("/api/settings");
        await flush();
        link.feedJson(1, FrameKind.HEAD, {status: 200, headers: {"content-type": "application/json"}});
        link.feedJson(1, FrameKind.CLOSE, {code: 4499, reason: "backend failed"});
        await expect(pending).rejects.toThrow(/4499: backend failed/);
    });

    it("rejects FormData bodies and unsupported methods", async () => {
        welcome();
        await expect(tunnel.openHttp("/api/x", {method: "POST", body: new FormData()})).rejects.toThrow(TypeError);
        await expect(tunnel.openHttp("/api/x", {method: "OPTIONS"})).rejects.toThrow(TypeError);
    });

    it("aborts with CLOSE and frees the stream id", async () => {
        welcome();
        const controller = new AbortController();
        const pending = tunnel.openHttp("/api/slow", {signal: controller.signal});
        await flush();
        link.take();
        controller.abort();
        await expect(pending).rejects.toMatchObject({name: "AbortError"});
        expect(link.take()).toMatchObject([{stream: 1, kind: FrameKind.CLOSE}]);
        tunnel.openHttp("/api/next").catch(ignore);
        await flush();
        expect(link.take()[0].stream).toBe(3);
    });
});

describe("WebSocket", () => {
    it("opens, exchanges text and binary messages, and closes", async () => {
        welcome();
        link.take();
        const socket = tunnel.openSocket("/api/mowglinext/publish/joy", {prio: true});
        socket.binaryType = "arraybuffer";
        const messages: unknown[] = [];
        const events: string[] = [];
        socket.onopen = () => events.push("open");
        socket.onmessage = (ev) => messages.push(ev.data);
        socket.onclose = (ev) => events.push(`close ${ev.code}`);
        expect(socket.readyState).toBe(0);
        await flush();
        const [open] = link.take();
        expect(parseJsonPayload(open.payload)).toEqual({t: "ws", path: "/api/mowglinext/publish/joy", prio: true, lossy: false});

        link.feedJson(1, FrameKind.HEAD, {status: 101});
        expect(socket.readyState).toBe(1);
        expect(events).toEqual(["open"]);

        link.feed(1, FrameKind.TEXT_END, utf8("hb"));
        link.feed(1, FrameKind.DATA, Uint8Array.of(1, 2));
        link.feed(1, FrameKind.END, Uint8Array.of(3));
        expect(messages[0]).toBe("hb");
        expect(new Uint8Array(messages[1] as ArrayBuffer)).toEqual(Uint8Array.of(1, 2, 3));

        socket.send("hello");
        socket.send(new Uint8Array(500));
        const sent = link.take();
        expect(sent.map((f) => [f.kind, f.payload.length])).toEqual([
            [FrameKind.TEXT_END, 5], [FrameKind.DATA, 480], [FrameKind.END, 20],
        ]);
        expect(fromUtf8(sent[0].payload)).toBe("hello");

        socket.close(1000, "bye");
        expect(link.take()).toMatchObject([{stream: 1, kind: FrameKind.CLOSE}]);
        await flush();
        expect(events).toEqual(["open", "close 1000"]);
        expect(socket.readyState).toBe(3);
    });

    it("delivers binary as Blob by default", async () => {
        welcome();
        const socket = tunnel.openSocket("/api/x");
        let data: unknown;
        socket.onmessage = (ev) => { data = ev.data; };
        await flush();
        link.feedJson(1, FrameKind.HEAD, {status: 101});
        link.feed(1, FrameKind.END, Uint8Array.of(7));
        expect(data).toBeInstanceOf(Blob);
    });

    it("propagates a tunnel-side CLOSE code", async () => {
        welcome();
        const socket = tunnel.openSocket("/api/x");
        let closed: CloseEvent | null = null;
        socket.onclose = (ev) => { closed = ev; };
        await flush();
        link.feedJson(1, FrameKind.HEAD, {status: 101});
        link.feedJson(1, FrameKind.CLOSE, {code: 4404, reason: "no such stream"});
        expect(closed).toMatchObject({code: 4404, reason: "no such stream"});
        expect(socket.readyState).toBe(3);
    });

    it("reports error + 1006 when the link goes down", async () => {
        welcome();
        const socket = tunnel.openSocket("/api/x");
        const events: string[] = [];
        socket.onerror = () => events.push("error");
        socket.onclose = (ev) => events.push(`close ${ev.code}`);
        await flush();
        link.feedJson(1, FrameKind.HEAD, {status: 101});
        link.onDown?.();
        expect(events).toEqual(["error", "close 1006"]);
    });

    it("fails a socket opened while the tunnel is down", async () => {
        const socket = tunnel.openSocket("/api/x");
        const events: string[] = [];
        socket.onclose = (ev) => events.push(`close ${ev.code}`);
        await flush();
        expect(events).toEqual(["close 1006"]);
    });

    it("refused upgrade closes abnormally", async () => {
        welcome();
        const socket = tunnel.openSocket("/api/x");
        let code = 0;
        socket.onclose = (ev) => { code = ev.code; };
        await flush();
        link.feedJson(1, FrameKind.HEAD, {status: 403});
        expect(code).toBe(1006);
    });

    it("throws when sending while connecting", () => {
        welcome();
        const socket = tunnel.openSocket("/api/x");
        expect(() => socket.send("x")).toThrow(DOMException);
    });
});
