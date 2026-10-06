// Single-connection WebSocket multiplexer for /api/mowglinext/multiplex.
//
// Replaces the one-WebSocket-per-topic pattern that opened ~25 connections
// per browser tab. All callers share one socket; each subscribe() registers
// a listener and tracks ref-count per topic so the server keeps exactly one
// upstream subscription per topic per tab.
//
// Wire format (matches MultiplexRoute in gui/pkg/api/mowglinext.go):
//   client → server: {"op": "subscribe"|"unsubscribe", "topic": "<key>"}  (JSON text)
//   server → client: MessagePack BINARY frame {"topic": "<key>", "data": <decoded msg object>}
//
// The server msgpack-encodes the already-decoded ROS message (snake_case keys
// preserved), so listeners receive the message OBJECT directly — no per-hook
// JSON.parse, no base64. Keeps the heavy number-array parse off the browser
// main thread (one msgpack decode vs JSON.parse(envelope)+atob+JSON.parse).

import {unpack} from "msgpackr";
import {LIVENESS_CHECK_MS, LIVENESS_TIMEOUT_MS, onNetworkRegained, reconnectDelayMs} from "./reconnect.ts";

type Listener = (data: unknown, first: boolean) => void;

/**
 * Public connection status. "closed" covers both never-connected/idle and a
 * dropped connection awaiting reconnect — consumers only need to know whether
 * live data can currently arrive.
 */
export type MultiplexStatus = "connecting" | "open" | "closed";

type StatusListener = (status: MultiplexStatus) => void;

/** Minimum interval between "malformed frame" console warnings. */
const DECODE_WARN_INTERVAL_MS = 10_000;

interface ServerFrame {
    topic: string;
    data: unknown;
}

interface ClientOp {
    op: "subscribe" | "unsubscribe";
    topic: string;
}

export class MultiplexedSocket {
    private url: string;
    private ws: WebSocket | null = null;
    private state: "idle" | "connecting" | "open" = "idle";
    private listeners = new Map<string, Set<Listener>>();
    // Functions that have not yet received their first payload — they get
    // first=true on the next delivery, then are removed.
    private pendingFirst = new WeakSet<Listener>();
    private reconnectAttempt = 0;
    private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    private statusListeners = new Set<StatusListener>();
    private lastDecodeWarnAt = 0;
    private livenessTimer: ReturnType<typeof setInterval> | null = null;
    private lastRxAt = 0;
    private everOpened = false;

    constructor(url: string, private readonly now: () => number = () => Date.now()) {
        this.url = url;
        // Wi-Fi back / phone unlocked: retry now instead of waiting for backoff.
        onNetworkRegained(() => this.retryNow());
    }

    /** True once the socket has been open at least once (drives "reconnecting"). */
    wasEverOpen(): boolean {
        return this.everOpened;
    }

    /** Skip the pending backoff wait and reconnect immediately. */
    retryNow(): void {
        if (this.state !== "idle" || this.listeners.size === 0) return;
        if (this.reconnectTimer != null) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }
        this.connect();
    }

    /** Current status for the shared connection ("closed" when idle). */
    getStatus(): MultiplexStatus {
        switch (this.state) {
            case "open":
                return "open";
            case "connecting":
                return "connecting";
            default:
                return "closed";
        }
    }

    /**
     * Register for status transitions (connecting → open → closed → ...).
     * Returns an unregister function. The callback is NOT invoked with the
     * current status on registration — read {@link getStatus} for that.
     */
    onStatusChange(cb: StatusListener): () => void {
        this.statusListeners.add(cb);
        return () => {
            this.statusListeners.delete(cb);
        };
    }

    private notifyStatus(): void {
        const status = this.getStatus();
        for (const cb of Array.from(this.statusListeners)) {
            try {
                cb(status);
            } catch (err) {
                console.error("MultiplexedSocket: status listener threw", err);
            }
        }
    }

    subscribe(topic: string, listener: Listener): () => void {
        let set = this.listeners.get(topic);
        const isFirstSubscriberForTopic = !set || set.size === 0;
        if (!set) {
            set = new Set();
            this.listeners.set(topic, set);
        }
        set.add(listener);
        this.pendingFirst.add(listener);

        if (this.state === "idle") {
            this.connect();
        } else if (this.state === "open" && isFirstSubscriberForTopic) {
            this.send({op: "subscribe", topic});
        }

        return () => this.unsubscribe(topic, listener);
    }

    private unsubscribe(topic: string, listener: Listener): void {
        const set = this.listeners.get(topic);
        if (!set) return;
        set.delete(listener);
        this.pendingFirst.delete(listener);
        if (set.size === 0) {
            this.listeners.delete(topic);
            if (this.state === "open") {
                this.send({op: "unsubscribe", topic});
            }
        }
        if (this.listeners.size === 0) {
            // Cancel any pending reconnect — nothing to subscribe for.
            if (this.reconnectTimer != null) {
                clearTimeout(this.reconnectTimer);
                this.reconnectTimer = null;
            }
            // Close the socket whether open OR still connecting — an in-flight
            // handshake with no listeners left would otherwise become an
            // orphan connection. onclose will not reconnect because the
            // listeners map is now empty.
            if (this.ws) {
                try { this.ws.close(); } catch { /* ignore */ }
            }
        }
    }

    private connect(): void {
        if (this.state !== "idle") return;
        if (this.listeners.size === 0) return;
        this.state = "connecting";
        this.notifyStatus();

        const ws = new WebSocket(this.url);
        // Server frames are MessagePack binary; receive them as ArrayBuffer.
        ws.binaryType = "arraybuffer";
        this.ws = ws;

        ws.onopen = () => {
            // Every subscriber may have gone away during the handshake —
            // don't keep an orphan connection alive.
            if (this.listeners.size === 0) {
                try { ws.close(); } catch { /* ignore */ }
                return;
            }
            this.state = "open";
            this.reconnectAttempt = 0;
            this.everOpened = true;
            this.lastRxAt = this.now();
            this.startLiveness(ws);
            this.notifyStatus();
            // Re-subscribe to every topic that still has listeners.
            for (const topic of this.listeners.keys()) {
                this.send({op: "subscribe", topic});
            }
        };

        ws.onmessage = (e: MessageEvent) => {
            // MessagePack binary frame → {topic, data: <decoded object>}.
            this.lastRxAt = this.now();
            let frame: ServerFrame;
            try {
                if (!(e.data instanceof ArrayBuffer)) return;
                frame = unpack(new Uint8Array(e.data)) as ServerFrame;
            } catch (err) {
                this.warnDecodeFailure(e.data, err);
                return;
            }
            const set = this.listeners.get(frame.topic);
            if (!set || set.size === 0) return;
            // Snapshot listeners so a callback that unsubscribes mid-iteration
            // does not affect the current dispatch.
            const snapshot = Array.from(set);
            for (const cb of snapshot) {
                const isFirst = this.pendingFirst.has(cb);
                if (isFirst) this.pendingFirst.delete(cb);
                try {
                    cb(frame.data, isFirst);
                } catch (err) {
                    console.error("MultiplexedSocket: listener threw", err);
                }
            }
        };

        ws.onerror = () => this.handleDrop(ws);
        ws.onclose = () => this.handleDrop(ws);
    }

    /**
     * Forget `ws` and reconnect if anything is still subscribed. Also used by
     * the liveness watchdog for a half-open socket, whose close handshake
     * would otherwise take tens of seconds to fire onclose.
     */
    private handleDrop(ws: WebSocket): void {
        if (this.ws !== ws) return;
        this.ws = null;
        ws.onopen = ws.onclose = ws.onerror = ws.onmessage = null;
        try { ws.close(); } catch { /* ignore */ }
        this.stopLiveness();
        this.state = "idle";
        this.notifyStatus();
        // Reconnect only if there's still something to listen for.
        if (this.listeners.size > 0) {
            this.scheduleReconnect();
        }
    }

    private startLiveness(ws: WebSocket): void {
        this.stopLiveness();
        this.livenessTimer = setInterval(() => {
            if (this.ws === ws && this.now() - this.lastRxAt > LIVENESS_TIMEOUT_MS) {
                this.handleDrop(ws);
            }
        }, LIVENESS_CHECK_MS);
    }

    private stopLiveness(): void {
        if (this.livenessTimer != null) clearInterval(this.livenessTimer);
        this.livenessTimer = null;
    }

    /**
     * Rate-limited (max one per {@link DECODE_WARN_INTERVAL_MS}) warning for
     * frames that fail msgpack decoding. The topic is part of the frame that
     * failed to decode, so only the raw size + subscribed topics are known.
     */
    private warnDecodeFailure(data: ArrayBuffer, err: unknown): void {
        const now = Date.now();
        if (now - this.lastDecodeWarnAt < DECODE_WARN_INTERVAL_MS) return;
        this.lastDecodeWarnAt = now;
        console.warn(
            "MultiplexedSocket: dropping undecodable frame",
            {
                byteLength: data.byteLength,
                subscribedTopics: Array.from(this.listeners.keys()),
            },
            err,
        );
    }

    private scheduleReconnect(): void {
        if (this.reconnectTimer != null) return;
        const delay = reconnectDelayMs(this.reconnectAttempt);
        this.reconnectAttempt += 1;
        this.reconnectTimer = setTimeout(() => {
            this.reconnectTimer = null;
            this.connect();
        }, delay);
    }

    private send(op: ClientOp): void {
        if (!this.ws || this.state !== "open") return;
        try {
            this.ws.send(JSON.stringify(op));
        } catch (err) {
            console.warn("MultiplexedSocket: send failed", err);
        }
    }
}

import {wsBase} from "../utils/apiHost";

let singleton: MultiplexedSocket | null = null;

function multiplexUrl(): string {
    return `${wsBase()}/api/mowglinext/multiplex`;
}

export function getMultiplexedSocket(): MultiplexedSocket {
    if (singleton == null) {
        singleton = new MultiplexedSocket(multiplexUrl());
    }
    return singleton;
}

// Match /api/mowglinext/subscribe/<topic> exactly; everything else (e.g.
// /publish/joy) keeps its dedicated socket.
const SUBSCRIBE_PREFIX = "/api/mowglinext/subscribe/";

export function isMultiplexableSubscribeUri(uri: string): boolean {
    return uri.startsWith(SUBSCRIBE_PREFIX) && uri.length > SUBSCRIBE_PREFIX.length;
}

export function topicFromSubscribeUri(uri: string): string {
    return uri.slice(SUBSCRIBE_PREFIX.length);
}
