import {getActiveTransport, openTrackedSocket} from "./registry.ts";
import type {SocketLike, SocketOptions} from "./types.ts";

export {
    getActiveKind,
    getActiveTransport,
    onTransportChange,
    registerTransport,
    setActiveTransport,
} from "./registry.ts";
export {networkHttpBase} from "./network.ts";
export {TRANSPORT_SWITCHED_CLOSE_CODE} from "./types.ts";
export type {SocketLike, SocketOptions, Transport, TransportKind} from "./types.ts";

/** `fetch` against the backend over the active transport. `path` is "/api/..." or "/web-build.json". */
export function apiFetch(path: string, init?: RequestInit): Promise<Response> {
    return getActiveTransport().fetch(path, init);
}

/**
 * Open a backend WebSocket over the active transport. A transport switch closes it with
 * `TRANSPORT_SWITCHED_CLOSE_CODE`; owners reconnect right away when they see that code.
 */
export function openApiSocket(path: string, options?: SocketOptions): SocketLike {
    return openTrackedSocket(path, options);
}
