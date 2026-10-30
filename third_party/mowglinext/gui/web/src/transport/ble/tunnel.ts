/**
 * Session and stream multiplexer of the BLE tunnel (docs/ble_tunnel_protocol.md sections 3-5),
 * over any byte link. The Web Bluetooth link is gattLink.ts; tests use a scripted fake.
 */
import type {SocketLike, SocketOptions} from "../types.ts";
import {encodeFrame, type Frame, FrameKind, FrameParser, jsonPayload, parseJsonPayload} from "./frame.ts";
import {HttpExchange, prepareHttpRequest} from "./httpStream.ts";
import {linkError, type StreamHandler, type StreamPort} from "./streamPort.ts";
import {TunnelSocket} from "./tunnelSocket.ts";

/** A bidirectional byte stream (both directions may split or merge frames arbitrarily). */
export interface ByteLink {
    write(bytes: Uint8Array): void;
    onBytes: ((bytes: Uint8Array) => void) | null;
    onDown: (() => void) | null;
    /** Bytes accepted by write() and not yet sent. Absent: the tunnel writes frames at once. */
    readonly bufferedAmount?: number;
    /** Called by the link when its buffer has room again. */
    onDrain?: (() => void) | null;
}

export type TunnelState =
    | "idle"
    | "handshaking"
    | "ready"
    | "unauthenticated"
    | "backend_unavailable"
    | "dead";

export interface TunnelStatus {
    state: TunnelState;
    /** Last PING round trip. */
    rttMs: number | null;
    host: string | null;
    session: string | null;
}

export interface TunnelOptions {
    /** HELLO `client`, e.g. "mowglinext-web/<build id>". */
    client: string;
    /** Wire bytes the mower may send beyond the last CREDIT (protocol maximum 8192). */
    window?: number;
    /** Mower output cap in wire bytes per second (protocol maximum 8000). */
    maxRate?: number;
    pingIntervalMs?: number;
    deadAfterMs?: number;
    creditIntervalMs?: number;
    /** After sending RESET, how long to wait for the mower's seq 0 before sending another. */
    resyncTimeoutMs?: number;
}

const DEFAULTS = {
    window: 2048,
    maxRate: 5000,
    pingIntervalMs: 1000,
    deadAfterMs: 3000,
    creditIntervalMs: 250,
    resyncTimeoutMs: 2000,
};

/** Keep at most about one 497-byte GATT write queued in the link so later frames can still jump the queue. */
const LINK_HIGH_WATER = 497;
const MAX_STREAM_ID = 65535;

interface Outgoing {
    stream: number;
    kind: number;
    payload: Uint8Array;
}

interface Waiter {
    resolve: () => void;
    reject: (error: unknown) => void;
}

export class Tunnel {
    // Assigned in the constructor: this project's vitest/vite transform runs field initializers
    // before parameter properties are assigned, so an initializer cannot read a constructor arg.
    private readonly config: typeof DEFAULTS & {client: string};
    private readonly parser = new FrameParser((frame, length) => this.onFrame(frame, length));
    private readonly streams = new Map<number, StreamHandler>();
    private readonly queues = {control: [] as Outgoing[], prio: [] as Outgoing[], normal: [] as Outgoing[]};
    private readonly listeners = new Set<(status: TunnelStatus) => void>();
    private readonly waiters: Waiter[] = [];
    private readonly pingsInFlight = new Map<number, number>();
    private readonly port: StreamPort = {
        send: (stream, kind, payload, prio) => this.enqueue(prio ? "prio" : "normal", {stream, kind, payload}),
        release: (stream) => {
            this.streams.delete(stream);
        },
    };

    private status: TunnelStatus = {state: "idle", rttMs: null, host: null, session: null};
    private token = "";
    private nextStreamId = 1;
    private sendSeq = 0;
    /** Next seq expected from the mower. */
    private expectedSeq = 0;
    /** We sent RESET and drop frames until the mower's numbering restarts at 0. */
    private resyncSince: number | null = null;
    /** Cumulative wire bytes received this session (u32), and the value last sent in CREDIT. */
    private received = 0;
    private credited = 0;
    private lastCreditAt = 0;
    private lastFrameAt = 0;
    private pingCounter = 0;
    private pingTimer: ReturnType<typeof setInterval> | null = null;
    private tickTimer: ReturnType<typeof setInterval> | null = null;

    constructor(private readonly link: ByteLink, options: TunnelOptions) {
        this.config = {...DEFAULTS, ...definedOnly(options), client: options.client};
        link.onBytes = (bytes) => this.parser.push(bytes);
        link.onDown = () => {
            // Keep a rejected token visible; the session is over either way.
            if (this.status.state !== "unauthenticated" && this.status.state !== "backend_unavailable") this.end("idle", "link down");
        };
        link.onDrain = () => this.pump();
    }

    getStatus(): TunnelStatus {
        return this.status;
    }

    subscribe(listener: (status: TunnelStatus) => void): () => void {
        this.listeners.add(listener);
        return () => this.unsubscribe(listener);
    }

    unsubscribe(listener: (status: TunnelStatus) => void): void {
        this.listeners.delete(listener);
    }

    unsubscribeAll(): void {
        this.listeners.clear();
    }

    /** The link is up: start a session with HELLO. Any previous session is discarded. */
    start(token: string): void {
        this.end("idle", "session restarted");
        this.token = token;
        this.parser.reset();
        this.sendSeq = 0;
        this.expectedSeq = 0;
        this.resyncSince = null;
        this.received = 0;
        this.credited = 0;
        this.lastCreditAt = Date.now();
        this.lastFrameAt = Date.now();
        this.setStatus({state: "handshaking", rttMs: null, host: null, session: null});
        this.enqueue("control", {stream: 0, kind: FrameKind.HELLO, payload: jsonPayload({
            v: 1,
            token: this.token,
            client: this.config.client,
            window: this.config.window,
            max_rate: this.config.maxRate,
        })});
        this.tickTimer = setInterval(() => this.tick(), Math.min(this.config.creditIntervalMs, 250));
    }

    /** End the session (fail every stream) without touching the link. */
    stop(): void {
        this.end("idle", "stopped");
    }

    async openHttp(path: string, init: RequestInit = {}): Promise<Response> {
        const signal = init.signal ?? null;
        if (signal?.aborted) throw signal.reason;
        const request = await prepareHttpRequest(path, init);
        if (jsonPayload(request.open).length > 480) throw new TypeError("Request head too large for the tunnel");
        await this.whenReady();
        if (signal?.aborted) throw signal.reason;
        return new Promise<Response>((resolve, reject) => {
            const id = this.allocateStream();
            const exchange = new HttpExchange(this.port, id, request, signal, resolve, reject);
            this.streams.set(id, exchange);
            exchange.start();
        });
    }

    openSocket(path: string, options: SocketOptions = {}): SocketLike {
        const socket = new TunnelSocket(this.port, path, options);
        this.whenReady().then(() => {
            if (socket.readyState !== 0) return;
            const id = this.allocateStream();
            this.streams.set(id, socket);
            if (!socket.attach(id)) this.streams.delete(id);
        }, () => socket.fail());
        return socket;
    }

    private whenReady(): Promise<void> {
        if (this.status.state === "ready") return Promise.resolve();
        if (this.status.state !== "handshaking") return Promise.reject(linkError(this.status.state));
        return new Promise((resolve, reject) => this.waiters.push({resolve, reject}));
    }

    private allocateStream(): number {
        for (let tries = 0; tries <= MAX_STREAM_ID / 2; tries++) {
            const id = this.nextStreamId;
            this.nextStreamId = id >= MAX_STREAM_ID ? 1 : id + 2;
            if (!this.streams.has(id)) return id;
        }
        throw linkError("no free stream id");
    }

    private onFrame(frame: Frame, wireLength: number): void {
        this.lastFrameAt = Date.now();
        if (this.status.state !== "handshaking" && this.status.state !== "ready") return;
        if (this.resyncSince !== null) {
            if (frame.seq !== 0 || frame.kind === FrameKind.RESET) return;
            this.resyncSince = null;
            this.expectedSeq = 0;
        }
        if (frame.kind === FrameKind.RESET) {
            this.onRemoteReset();
            return;
        }
        if (frame.seq !== this.expectedSeq) {
            this.sendReset();
            return;
        }
        this.expectedSeq = (frame.seq + 1) & 0xff;
        this.received = (this.received + wireLength) >>> 0;
        if (((this.received - this.credited) >>> 0) >= this.config.window / 4) this.sendCredit();

        switch (frame.kind) {
            case FrameKind.WELCOME:
                this.onWelcome(frame.payload);
                return;
            case FrameKind.PONG:
                this.onPong(frame.payload);
                return;
            case FrameKind.HEAD:
            case FrameKind.DATA:
            case FrameKind.END:
            case FrameKind.TEXT_END:
            case FrameKind.CLOSE:
                if (frame.stream !== 0) this.streams.get(frame.stream)?.onFrame(frame);
                return;
            default:
                // Unknown kinds and phone-only kinds are ignored.
                return;
        }
    }

    private onWelcome(payload: Uint8Array): void {
        if (this.status.state !== "handshaking") return;
        const welcome = parseJsonPayload(payload) as {error?: unknown; host?: unknown; session?: unknown} | null;
        if (!welcome || welcome.error !== undefined) {
            this.end(welcome?.error === "backend_unavailable" ? "backend_unavailable" : "unauthenticated",
                typeof welcome?.error === "string" ? welcome.error : "bad welcome");
            return;
        }
        this.setStatus({
            ...this.status,
            state: "ready",
            host: typeof welcome.host === "string" ? welcome.host : null,
            session: typeof welcome.session === "string" ? welcome.session : null,
        });
        this.waiters.splice(0).forEach((w) => w.resolve());
        this.pingTimer = setInterval(() => this.sendPing(), this.config.pingIntervalMs);
    }

    private sendPing(): void {
        const id = (this.pingCounter = (this.pingCounter + 1) >>> 0);
        const payload = new Uint8Array(8);
        new DataView(payload.buffer).setUint32(0, id, true);
        this.pingsInFlight.set(id, Date.now());
        for (const old of this.pingsInFlight.keys()) {
            if (this.pingsInFlight.size <= 8) break;
            this.pingsInFlight.delete(old);
        }
        this.enqueue("control", {stream: 0, kind: FrameKind.PING, payload});
    }

    private onPong(payload: Uint8Array): void {
        if (payload.length !== 8) return;
        const id = new DataView(payload.buffer, payload.byteOffset, 8).getUint32(0, true);
        const sentAt = this.pingsInFlight.get(id);
        if (sentAt === undefined) return;
        this.pingsInFlight.delete(id);
        this.setStatus({...this.status, rttMs: Date.now() - sentAt});
    }

    private tick(): void {
        const now = Date.now();
        if (now - this.lastFrameAt >= this.config.deadAfterMs) {
            this.end("dead", "no frame for 3 s");
            return;
        }
        if (this.resyncSince !== null && now - this.resyncSince >= this.config.resyncTimeoutMs) {
            this.sendReset();
            return;
        }
        if (this.received !== this.credited && now - this.lastCreditAt >= this.config.creditIntervalMs) this.sendCredit();
    }

    private sendCredit(): void {
        const payload = new Uint8Array(4);
        new DataView(payload.buffer).setUint32(0, this.received, true);
        this.credited = this.received;
        this.lastCreditAt = Date.now();
        this.enqueue("control", {stream: 0, kind: FrameKind.CREDIT, payload});
    }

    /** A seq gap: every stream's message boundaries are unreliable now. */
    private sendReset(): void {
        this.failStreams("sequence gap, tunnel reset");
        this.clearQueues();
        this.enqueue("control", {stream: 0, kind: FrameKind.RESET, payload: new Uint8Array(0)});
        this.resyncSince = Date.now();
        this.received = 0;
        this.credited = 0;
    }

    private onRemoteReset(): void {
        this.failStreams("tunnel reset by the mower");
        this.clearQueues();
        this.sendSeq = 0;
        // The mower's next frame after its RESET has seq 0 (the RESET's own seq is not checked).
        this.expectedSeq = 0;
        this.received = 0;
        this.credited = 0;
    }

    private enqueue(queue: keyof Tunnel["queues"], item: Outgoing): void {
        this.queues[queue].push(item);
        this.pump();
    }

    private pump(): void {
        while ((this.link.bufferedAmount ?? 0) < LINK_HIGH_WATER) {
            const item = this.queues.control.shift() ?? this.queues.prio.shift() ?? this.queues.normal.shift();
            if (!item) return;
            // RESET is sent with seq 0 and the next frame has seq 0 again (protocol section 4).
            const seq = item.kind === FrameKind.RESET ? 0 : this.sendSeq;
            this.sendSeq = item.kind === FrameKind.RESET ? 0 : (seq + 1) & 0xff;
            this.link.write(encodeFrame({...item, seq}));
        }
    }

    private clearQueues(): void {
        this.queues.control.length = 0;
        this.queues.prio.length = 0;
        this.queues.normal.length = 0;
    }

    private failStreams(reason: string): void {
        const streams = [...this.streams.values()];
        this.streams.clear();
        streams.forEach((s) => s.fail(reason));
    }

    private end(state: TunnelState, reason: string): void {
        if (this.pingTimer !== null) clearInterval(this.pingTimer);
        if (this.tickTimer !== null) clearInterval(this.tickTimer);
        this.pingTimer = null;
        this.tickTimer = null;
        this.pingsInFlight.clear();
        this.clearQueues();
        this.waiters.splice(0).forEach((w) => w.reject(linkError(reason)));
        this.failStreams(reason);
        this.setStatus({...this.status, state, rttMs: null});
    }

    private setStatus(status: TunnelStatus): void {
        const changed = Object.keys(status).some((k) => status[k as keyof TunnelStatus] !== this.status[k as keyof TunnelStatus]);
        this.status = status;
        if (changed) this.listeners.forEach((l) => l(status));
    }
}

function definedOnly<T extends object>(value: T): Partial<T> {
    return Object.fromEntries(Object.entries(value).filter(([, v]) => v !== undefined)) as Partial<T>;
}
