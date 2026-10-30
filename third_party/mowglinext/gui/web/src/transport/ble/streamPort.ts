import type {Frame} from "./frame.ts";

/** What a stream (HTTP exchange or tunnelled socket) needs from the tunnel. */
export interface StreamPort {
    /** Queue one frame for this stream; `prio` frames go before other streams' frames. */
    send(stream: number, kind: number, payload: Uint8Array, prio: boolean): void;
    /** The stream id is free again (CLOSE sent or seen, or the stream ended). */
    release(stream: number): void;
}

/** A stream as the tunnel sees it. */
export interface StreamHandler {
    onFrame(frame: Frame): void;
    /** The session ended under the stream (link down, RESET, dead link): fail without sending. */
    fail(reason: string): void;
}

/** Reject like fetch does when the network is down. */
export function linkError(reason: string): TypeError {
    return new TypeError(`Failed to fetch (Bluetooth: ${reason})`);
}

export function concatBytes(chunks: readonly Uint8Array[]): Uint8Array<ArrayBuffer> {
    const total = chunks.reduce((n, c) => n + c.length, 0);
    const out = new Uint8Array(total);
    let offset = 0;
    for (const c of chunks) {
        out.set(c, offset);
        offset += c.length;
    }
    return out;
}
