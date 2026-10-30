import {httpBase, wsBase} from "../utils/apiHost.ts";
import type {SocketLike, Transport} from "./types.ts";

/**
 * Today's path to the backend: same-origin (or `VITE_API_HOST`) fetch and WebSocket.
 * This file is the only place outside `utils/apiHost.ts` that builds backend URLs.
 */
export class NetworkTransport implements Transport {
    readonly kind = "network" as const;

    fetch(path: string, init?: RequestInit): Promise<Response> {
        return globalThis.fetch(`${httpBase()}${path}`, init);
    }

    /** Socket options are BLE scheduling hints; a TCP socket ignores them. */
    openSocket(path: string): SocketLike {
        return new WebSocket(`${wsBase()}${path}`);
    }
}

/**
 * HTTP base for URLs the browser loads by itself rather than through the seam
 * (map raster tiles and images handed to mapbox-gl, the imagery upload XHR that
 * needs progress events). These only work over the network transport.
 */
export function networkHttpBase(): string {
    return httpBase();
}
