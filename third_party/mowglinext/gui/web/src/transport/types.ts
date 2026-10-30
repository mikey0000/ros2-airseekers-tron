/**
 * Transport seam: everything the GUI sends to the backend goes through one of these.
 *
 * `network` is today's path (same-origin fetch + WebSocket). `ble` tunnels the same
 * HTTP and WebSocket traffic over the mower's Bluetooth LE module
 * (ros2_stack/docs/ble_tunnel_protocol.md). Callers never pick a transport; they call
 * `apiFetch` / `openApiSocket` from `./index.ts` and the active transport is used.
 */
export type TransportKind = "network" | "ble";

/**
 * The subset of the browser `WebSocket` the GUI's socket classes use
 * (hooks/multiplexedSocket.ts, hooks/reconnect.ts PublishSocket, hooks/useContainerRestart.ts).
 * readyState uses the WebSocket numbering: 0 CONNECTING, 1 OPEN, 2 CLOSING, 3 CLOSED.
 */
export interface SocketLike {
    readonly url: string;
    readonly readyState: number;
    binaryType: BinaryType;
    onopen: ((ev: Event) => void) | null;
    onmessage: ((ev: MessageEvent) => void) | null;
    onclose: ((ev: CloseEvent) => void) | null;
    onerror: ((ev: Event) => void) | null;
    send(data: string | ArrayBufferLike | ArrayBufferView | Blob): void;
    close(code?: number, reason?: string): void;
}

/** Per-socket hints the BLE tunnel uses; the network transport ignores them. */
export interface SocketOptions {
    /** Frames of this socket are sent before any other stream's (joystick). */
    prio?: boolean;
    /** The mower may drop the oldest queued messages when the link is saturated (status multiplex). */
    lossy?: boolean;
}

export interface Transport {
    readonly kind: TransportKind;
    /**
     * `path` is an absolute path on the backend ("/api/..." or "/web-build.json"), never a
     * full URL. The returned Response behaves like a fetch Response (status, headers, body
     * readers). Rejects like fetch does when the transport is down.
     */
    fetch(path: string, init?: RequestInit): Promise<Response>;
    /** `path` as above; the socket is in CONNECTING state when returned. */
    openSocket(path: string, options?: SocketOptions): SocketLike;
}

/** Close code the seam uses when it closes sockets because the active transport changed. */
export const TRANSPORT_SWITCHED_CLOSE_CODE = 4000;
