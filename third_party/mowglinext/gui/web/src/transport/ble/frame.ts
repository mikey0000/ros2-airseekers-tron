/**
 * BLE tunnel framing, docs/ble_tunnel_protocol.md section 2.
 *
 *   0  2  sync    0xAA 0x55
 *   2  2  len     payload length, 0..480 (u16 LE)
 *   4  2  stream  u16 LE
 *   6  1  kind
 *   7  1  seq
 *   8  len payload
 *   8+len 2 crc16 CRC-16/CCITT-FALSE over bytes 2 .. 8+len, LE
 */

export const SYNC_0 = 0xaa;
export const SYNC_1 = 0x55;
export const HEADER_LENGTH = 8;
export const CRC_LENGTH = 2;
export const MAX_PAYLOAD = 480;
export const MAX_FRAME = HEADER_LENGTH + MAX_PAYLOAD + CRC_LENGTH;

export const FrameKind = {
    OPEN: 0,
    HEAD: 1,
    DATA: 2,
    END: 3,
    TEXT_END: 4,
    CLOSE: 5,
    CREDIT: 6,
    RESET: 7,
    PING: 8,
    PONG: 9,
    HELLO: 10,
    WELCOME: 11,
} as const;

export type FrameKind = (typeof FrameKind)[keyof typeof FrameKind];

export interface Frame {
    stream: number;
    /** A known {@link FrameKind} or an unknown number (receivers ignore unknown kinds). */
    kind: number;
    seq: number;
    payload: Uint8Array;
}

/** CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor. */
export function crc16(bytes: Uint8Array, start = 0, end = bytes.length): number {
    let crc = 0xffff;
    for (let i = start; i < end; i++) {
        crc ^= bytes[i] << 8;
        for (let bit = 0; bit < 8; bit++) {
            crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
        }
    }
    return crc;
}

export function encodeFrame(frame: Frame): Uint8Array {
    const {stream, kind, seq, payload} = frame;
    if (payload.length > MAX_PAYLOAD) throw new RangeError(`payload ${payload.length} > ${MAX_PAYLOAD}`);
    const out = new Uint8Array(HEADER_LENGTH + payload.length + CRC_LENGTH);
    const view = new DataView(out.buffer);
    out[0] = SYNC_0;
    out[1] = SYNC_1;
    view.setUint16(2, payload.length, true);
    view.setUint16(4, stream & 0xffff, true);
    out[6] = kind & 0xff;
    out[7] = seq & 0xff;
    out.set(payload, HEADER_LENGTH);
    view.setUint16(HEADER_LENGTH + payload.length, crc16(out, 2, HEADER_LENGTH + payload.length), true);
    return out;
}

/**
 * Decodes exactly one frame occupying the whole buffer; returns null when the buffer is not a
 * valid frame (wrong sync, length, or CRC).
 */
export function decodeFrame(bytes: Uint8Array): Frame | null {
    let result: Frame | null = null;
    const parser = new FrameParser((frame, wireLength) => {
        if (wireLength === bytes.length && result === null) result = frame;
    });
    parser.push(bytes);
    return result;
}

/**
 * Streaming decoder: feed arbitrary byte chunks, get whole frames. Junk before a sync word, a
 * false sync (impossible length) and frames with a bad CRC are skipped; the parser then resumes
 * at the next sync word after the rejected one.
 */
export class FrameParser {
    private buffer = new Uint8Array(0);
    /** Bytes skipped while resynchronising since construction or the last {@link reset}. */
    private skipped = 0;

    constructor(private readonly onFrame: (frame: Frame, wireLength: number) => void) {}

    get skippedBytes(): number {
        return this.skipped;
    }

    reset(): void {
        this.buffer = new Uint8Array(0);
        this.skipped = 0;
    }

    push(chunk: Uint8Array): void {
        if (chunk.length === 0) return;
        const joined = new Uint8Array(this.buffer.length + chunk.length);
        joined.set(this.buffer, 0);
        joined.set(chunk, this.buffer.length);
        let offset = 0;
        for (;;) {
            const sync = findSync(joined, offset);
            if (sync < 0) {
                // Keep a trailing 0xAA: it may be the first half of a sync word.
                const keep = joined.length > offset && joined[joined.length - 1] === SYNC_0 ? 1 : 0;
                this.skipped += joined.length - offset - keep;
                offset = joined.length - keep;
                break;
            }
            this.skipped += sync - offset;
            offset = sync;
            if (joined.length - offset < HEADER_LENGTH) break;
            const length = joined[offset + 2] | (joined[offset + 3] << 8);
            if (length > MAX_PAYLOAD) {
                offset += 1;
                this.skipped += 1;
                continue;
            }
            const total = HEADER_LENGTH + length + CRC_LENGTH;
            if (joined.length - offset < total) break;
            const crcAt = offset + HEADER_LENGTH + length;
            const wireCrc = joined[crcAt] | (joined[crcAt + 1] << 8);
            if (crc16(joined, offset + 2, crcAt) !== wireCrc) {
                offset += 1;
                this.skipped += 1;
                continue;
            }
            const frame: Frame = {
                stream: joined[offset + 4] | (joined[offset + 5] << 8),
                kind: joined[offset + 6],
                seq: joined[offset + 7],
                payload: joined.slice(offset + HEADER_LENGTH, crcAt),
            };
            offset += total;
            this.onFrame(frame, total);
        }
        this.buffer = joined.slice(offset);
    }
}

function findSync(bytes: Uint8Array, from: number): number {
    for (let i = from; i + 1 < bytes.length; i++) {
        if (bytes[i] === SYNC_0 && bytes[i + 1] === SYNC_1) return i;
    }
    return -1;
}

const encoder = new TextEncoder();
const decoder = new TextDecoder();

export function utf8(text: string): Uint8Array<ArrayBuffer> {
    return encoder.encode(text);
}

export function fromUtf8(bytes: Uint8Array): string {
    return decoder.decode(bytes);
}

export function jsonPayload(value: unknown): Uint8Array {
    return utf8(JSON.stringify(value));
}

export function parseJsonPayload(bytes: Uint8Array): unknown {
    if (bytes.length === 0) return null;
    try {
        return JSON.parse(fromUtf8(bytes)) as unknown;
    } catch {
        return null;
    }
}
