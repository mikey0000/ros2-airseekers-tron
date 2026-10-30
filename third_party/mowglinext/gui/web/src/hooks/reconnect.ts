import {openApiSocket, TRANSPORT_SWITCHED_CLOSE_CODE, type SocketLike} from "../transport/index.ts";

// Reconnect policy shared by the GUI's WebSockets (status multiplex + joystick).
//
// The phone roams at the edge of the house Wi-Fi: the link drops for a moment
// ("connection reset by peer", close 1005) or goes half-open (the browser keeps
// reporting OPEN while nothing arrives). The server sends a heartbeat frame
// every second on every socket (gui/pkg/api/ws_heartbeat.go), so:
//   - a socket that received nothing for LIVENESS_TIMEOUT_MS is treated as dead
//     and closed, which triggers the reconnect;
//   - reconnects are fast and capped at 1 s (LAN, not a public server);
//   - the browser's `online` event and the page becoming visible again retry
//     immediately instead of waiting for the backoff timer;
//   - a close with TRANSPORT_SWITCHED_CLOSE_CODE (Wi-Fi <-> Bluetooth switch) also
//     reconnects immediately, on the newly active transport.

/** Multiplex pseudo-topic of the server heartbeat (never subscribed). */
export const HEARTBEAT_TOPIC = "__hb";
/** Text heartbeat frame on publish (joystick) sockets. */
export const PUBLISH_HEARTBEAT = "hb";

/** Server heartbeat is 1 s; three missed beats = dead link. */
export const LIVENESS_TIMEOUT_MS = 3000;
export const LIVENESS_CHECK_MS = 500;

const RECONNECT_DELAYS_MS = [250, 500, 1000];

/** Delay before reconnect attempt `attempt` (0-based): 250, 500, then 1000 ms. */
export function reconnectDelayMs(attempt: number): number {
    return RECONNECT_DELAYS_MS[Math.min(Math.max(attempt, 0), RECONNECT_DELAYS_MS.length - 1)];
}

/**
 * Call `cb` when the network comes back or the page becomes visible again.
 * Returns an unregister function. No-op outside a browser.
 */
export function onNetworkRegained(cb: () => void): () => void {
    if (typeof window === "undefined") return () => {};
    const onVisible = () => {
        if (typeof document === "undefined" || document.visibilityState === "visible") cb();
    };
    window.addEventListener("online", cb);
    if (typeof document !== "undefined") document.addEventListener("visibilitychange", onVisible);
    return () => {
        window.removeEventListener("online", cb);
        if (typeof document !== "undefined") document.removeEventListener("visibilitychange", onVisible);
    };
}

export type PublishSocketStatus = "idle" | "connecting" | "open" | "reconnecting";

/** Opens the socket for a backend path; the default goes through the transport seam. */
type WSFactory = (path: string) => SocketLike;

/** True for the close the transport seam sends when the active transport changes. */
export function isTransportSwitchClose(ev: CloseEvent | undefined): boolean {
    return ev?.code === TRANSPORT_SWITCHED_CLOSE_CODE;
}

/**
 * PublishSocket — the joystick's dedicated WebSocket (/api/mowglinext/publish/joy).
 *
 * Safety contract: send() NEVER queues. While the socket is not open the
 * message is dropped (returns false), so a reconnect cannot replay a burst of
 * stale twists; the robot side stops on its own cmd_vel watchdog. The joystick
 * keeps re-sending the current twist every 100 ms while held, so motion resumes
 * by itself once the socket is back, without a page reload.
 */
export class PublishSocket {
    private ws: SocketLike | null = null;
    private status: PublishSocketStatus = "idle";
    private wanted = false;
    private attempt = 0;
    private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    private livenessTimer: ReturnType<typeof setInterval> | null = null;
    private lastRxAt = 0;
    private everOpened = false;
    private listeners = new Set<(s: PublishSocketStatus) => void>();
    private unregisterNetwork: (() => void) | null = null;

    constructor(
        /** Backend path, e.g. "/api/mowglinext/publish/joy". */
        readonly path: string,
        private readonly wsFactory: WSFactory = (p) => openApiSocket(p, {prio: true}),
        private readonly now: () => number = () => Date.now(),
        /** Receives non-heartbeat text frames (none today). */
        private readonly onMessage?: (data: unknown) => void,
    ) {}

    getStatus(): PublishSocketStatus {
        return this.status;
    }

    onStatusChange(cb: (s: PublishSocketStatus) => void): () => void {
        this.listeners.add(cb);
        return () => this.listeners.delete(cb);
    }

    start(): void {
        if (this.wanted) return;
        this.wanted = true;
        this.attempt = 0;
        this.everOpened = false;
        this.unregisterNetwork = onNetworkRegained(() => this.retryNow());
        this.connect();
    }

    stop(): void {
        this.wanted = false;
        this.unregisterNetwork?.();
        this.unregisterNetwork = null;
        this.clearTimers();
        const ws = this.ws;
        this.ws = null;
        if (ws) {
            ws.onopen = ws.onclose = ws.onerror = ws.onmessage = null;
            try { ws.close(); } catch { /* ignore */ }
        }
        this.setStatus("idle");
    }

    /** Send JSON now if the socket is open; otherwise drop it. */
    send(msg: unknown): boolean {
        const ws = this.ws;
        if (!ws || this.status !== "open" || ws.readyState !== 1 /* OPEN */) return false;
        try {
            ws.send(JSON.stringify(msg));
            return true;
        } catch {
            return false;
        }
    }

    /** Skip the backoff wait (network back / page visible again). */
    retryNow(): void {
        if (!this.wanted || this.status === "open" || this.status === "connecting") return;
        if (this.reconnectTimer != null) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }
        this.connect();
    }

    private setStatus(s: PublishSocketStatus): void {
        if (s === this.status) return;
        this.status = s;
        for (const cb of Array.from(this.listeners)) {
            try { cb(s); } catch (err) { console.error("PublishSocket: status listener threw", err); }
        }
    }

    private clearTimers(): void {
        if (this.reconnectTimer != null) clearTimeout(this.reconnectTimer);
        if (this.livenessTimer != null) clearInterval(this.livenessTimer);
        this.reconnectTimer = null;
        this.livenessTimer = null;
    }

    private connect(): void {
        if (!this.wanted) return;
        this.setStatus(this.everOpened ? "reconnecting" : "connecting");
        let ws: SocketLike;
        try {
            ws = this.wsFactory(this.path);
        } catch {
            this.scheduleReconnect();
            return;
        }
        this.ws = ws;
        ws.onopen = () => {
            if (this.ws !== ws) return;
            this.attempt = 0;
            this.everOpened = true;
            this.lastRxAt = this.now();
            this.setStatus("open");
            if (this.livenessTimer != null) clearInterval(this.livenessTimer);
            this.livenessTimer = setInterval(() => {
                if (this.ws === ws && this.now() - this.lastRxAt > LIVENESS_TIMEOUT_MS) {
                    this.drop(ws);
                }
            }, LIVENESS_CHECK_MS);
        };
        ws.onmessage = (e: MessageEvent) => {
            if (this.ws !== ws) return;
            this.lastRxAt = this.now();
            if (e.data !== PUBLISH_HEARTBEAT) this.onMessage?.(e.data);
        };
        ws.onerror = () => {
            if (this.ws === ws) this.drop(ws);
        };
        ws.onclose = (ev: CloseEvent | undefined) => {
            if (this.ws === ws) this.drop(ws, isTransportSwitchClose(ev));
        };
    }

    /**
     * Forget a dead/closed socket and schedule the next attempt (`immediate`: now, for a
     * transport switch).
     */
    private drop(ws: SocketLike, immediate = false): void {
        if (this.ws !== ws) return;
        this.ws = null;
        ws.onopen = ws.onclose = ws.onerror = ws.onmessage = null;
        try { ws.close(); } catch { /* ignore */ }
        if (this.livenessTimer != null) clearInterval(this.livenessTimer);
        this.livenessTimer = null;
        if (!this.wanted) {
            this.setStatus("idle");
            return;
        }
        this.setStatus(this.everOpened ? "reconnecting" : "connecting");
        if (immediate) {
            this.attempt = 0;
            if (this.reconnectTimer != null) clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
            this.connect();
        } else {
            this.scheduleReconnect();
        }
    }

    private scheduleReconnect(): void {
        if (!this.wanted || this.reconnectTimer != null) return;
        const delay = reconnectDelayMs(this.attempt);
        this.attempt += 1;
        this.reconnectTimer = setTimeout(() => {
            this.reconnectTimer = null;
            this.connect();
        }, delay);
    }
}
