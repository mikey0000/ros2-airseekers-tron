import {beforeAll, describe, expect, it} from "vitest";
import {crc16, decodeFrame, encodeFrame, type Frame, FrameParser, MAX_PAYLOAD, utf8} from "./frame.ts";

interface VectorFrame {
    name: string;
    stream: number;
    kind: number;
    seq: number;
    payload_utf8?: string;
    payload_hex?: string;
    wire_hex: string;
}

interface Vectors {
    crc_check: {input_utf8: string; crc: number};
    frames: VectorFrame[];
    streams: {name: string; wire_hex: string; frames: string[]}[];
}

// Shared with the Python bridge's tests (scripts/ble/test_ble_mux.py); read from disk so both
// suites always run against the same file. No @types/node in this project, hence the cast; under
// vitest import.meta.url is served as "/@fs/<absolute path>".
const VECTORS_URL = new URL("../../../../../../../scripts/ble/testdata/mux_vectors.json", import.meta.url);
let vectors: Vectors;

beforeAll(async () => {
    const fs = (await import(/* @vite-ignore */ "node:fs" as string)) as {
        readFileSync(path: string, encoding: string): string;
    };
    vectors = JSON.parse(fs.readFileSync(decodeURIComponent(VECTORS_URL.pathname).replace(/^\/@fs\//, "/"), "utf8")) as Vectors;
});

function hex(text: string): Uint8Array {
    const out = new Uint8Array(text.length / 2);
    for (let i = 0; i < out.length; i++) out[i] = parseInt(text.slice(i * 2, i * 2 + 2), 16);
    return out;
}

function toHex(bytes: Uint8Array): string {
    return Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join("");
}

function payloadOf(v: VectorFrame): Uint8Array {
    return v.payload_utf8 !== undefined ? utf8(v.payload_utf8) : hex(v.payload_hex ?? "");
}

function collect(chunks: Uint8Array[]): {frames: Frame[]; lengths: number[]} {
    const frames: Frame[] = [];
    const lengths: number[] = [];
    const parser = new FrameParser((frame, length) => {
        frames.push(frame);
        lengths.push(length);
    });
    chunks.forEach((c) => parser.push(c));
    return {frames, lengths};
}

describe("crc16", () => {
    it("matches the CRC-16/CCITT-FALSE check value", () => {
        expect(crc16(utf8(vectors.crc_check.input_utf8))).toBe(vectors.crc_check.crc);
        expect(vectors.crc_check.crc).toBe(0x29b1);
    });
});

describe("frame vectors", () => {
    it("has vectors to test", () => {
        expect(vectors.frames.length).toBeGreaterThan(10);
    });

    it("encodes every vector byte for byte", () => {
        for (const v of vectors.frames) {
            const wire = encodeFrame({stream: v.stream, kind: v.kind, seq: v.seq, payload: payloadOf(v)});
            expect(toHex(wire), v.name).toBe(v.wire_hex);
        }
    });

    it("decodes every vector", () => {
        for (const v of vectors.frames) {
            const frame = decodeFrame(hex(v.wire_hex));
            expect(frame, v.name).not.toBeNull();
            expect(frame!.stream, v.name).toBe(v.stream);
            expect(frame!.kind, v.name).toBe(v.kind);
            expect(frame!.seq, v.name).toBe(v.seq);
            expect(toHex(frame!.payload), v.name).toBe(toHex(payloadOf(v)));
        }
    });

    it("decodes every vector fed one byte at a time", () => {
        for (const v of vectors.frames) {
            const wire = hex(v.wire_hex);
            const {frames, lengths} = collect(Array.from(wire, (b) => Uint8Array.of(b)));
            expect(frames, v.name).toHaveLength(1);
            expect(lengths[0], v.name).toBe(wire.length);
        }
    });

    it("rejects a payload above the maximum", () => {
        expect(() => encodeFrame({stream: 1, kind: 2, seq: 0, payload: new Uint8Array(MAX_PAYLOAD + 1)})).toThrow(RangeError);
    });
});

describe("stream vectors", () => {
    it("parses every stream vector into the named frames", () => {
        const byName = new Map(vectors.frames.map((f) => [f.name, f]));
        for (const s of vectors.streams) {
            const wire = hex(s.wire_hex);
            for (const split of [wire.length, 1, 3, 7]) {
                const chunks: Uint8Array[] = [];
                for (let i = 0; i < wire.length; i += split) chunks.push(wire.slice(i, i + split));
                const {frames} = collect(chunks);
                expect(frames.map((f) => toHex(encodeFrame(f))), `${s.name}/${split}`)
                    .toEqual(s.frames.map((n) => byName.get(n)!.wire_hex));
            }
        }
    });

    it("keeps a trailing half sync word across chunks", () => {
        const wire = hex(vectors.frames.find((f) => f.name === "reset")!.wire_hex);
        const {frames} = collect([Uint8Array.of(0x01, 0x02, wire[0]), wire.slice(1)]);
        expect(frames).toHaveLength(1);
    });
});
