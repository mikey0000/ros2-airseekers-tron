import type {SocketLike, SocketOptions} from "../types.ts";
import {type Frame, FrameKind, fromUtf8, jsonPayload, MAX_PAYLOAD, parseJsonPayload, utf8} from "./frame.ts";
import {concatBytes, type StreamHandler, type StreamPort} from "./streamPort.ts";

const CONNECTING = 0;
const OPEN = 1;
const CLOSING = 2;
const CLOSED = 3;

/** Close code for an abnormal end (link lost, refused upgrade), as browsers report it. */
export const ABNORMAL_CLOSURE = 1006;
const NO_STATUS = 1005;

/**
 * A WebSocket-shaped object on a tunnel stream (ble_tunnel_protocol.md section 3.2). It is
 * returned in CONNECTING state; {@link attach} sends the OPEN once the session is ready.
 */
export class TunnelSocket implements SocketLike, StreamHandler {
    binaryType: BinaryType = "blob";
    onopen: ((ev: Event) => void) | null = null;
    onmessage: ((ev: MessageEvent) => void) | null = null;
    onclose: ((ev: CloseEvent) => void) | null = null;
    onerror: ((ev: Event) => void) | null = null;

    private state = CONNECTING;
    private stream: number | null = null;
    private readonly chunks: Uint8Array[] = [];
    /** Serialises sends while a Blob is being read, so messages keep their order. */
    private pendingSend: Promise<void> | null = null;

    constructor(
        private readonly port: StreamPort,
        readonly url: string,
        private readonly options: SocketOptions,
    ) {}

    get readyState(): number {
        return this.state;
    }

    /** The session is ready: send OPEN on `stream`. Returns false when the socket was closed meanwhile. */
    attach(stream: number): boolean {
        if (this.state !== CONNECTING) return false;
        this.stream = stream;
        this.port.send(stream, FrameKind.OPEN, jsonPayload({
            t: "ws",
            path: this.url,
            prio: this.options.prio ?? false,
            lossy: this.options.lossy ?? false,
        }), this.prio);
        return true;
    }

    onFrame(frame: Frame): void {
        switch (frame.kind) {
            case FrameKind.HEAD: {
                const head = parseJsonPayload(frame.payload) as {status?: unknown} | null;
                if (this.state !== CONNECTING) return;
                if (head?.status === 101) {
                    this.state = OPEN;
                    this.onopen?.(new Event("open"));
                } else {
                    this.release();
                    this.abnormalClose();
                }
                return;
            }
            case FrameKind.DATA:
                this.chunks.push(frame.payload);
                return;
            case FrameKind.END:
            case FrameKind.TEXT_END:
                this.chunks.push(frame.payload);
                this.deliver(concatBytes(this.chunks.splice(0)), frame.kind === FrameKind.TEXT_END);
                return;
            case FrameKind.CLOSE: {
                if (this.state === CLOSED) return;
                const json = parseJsonPayload(frame.payload) as {code?: unknown; reason?: unknown} | null;
                this.release();
                this.state = CLOSED;
                this.onclose?.(new CloseEvent("close", {
                    code: typeof json?.code === "number" ? json.code : NO_STATUS,
                    reason: typeof json?.reason === "string" ? json.reason : "",
                    wasClean: true,
                }));
                return;
            }
        }
    }

    fail(): void {
        this.stream = null;
        this.abnormalClose();
    }

    send(data: string | ArrayBufferLike | ArrayBufferView | Blob): void {
        if (this.state === CONNECTING) throw new DOMException("Still in CONNECTING state.", "InvalidStateError");
        if (this.state !== OPEN) return;
        if (typeof Blob !== "undefined" && data instanceof Blob) {
            this.chain(() => data.arrayBuffer().then((buffer) => this.sendMessage(new Uint8Array(buffer), FrameKind.END)));
            return;
        }
        const [bytes, kind] = typeof data === "string"
            ? [utf8(data), FrameKind.TEXT_END]
            : ArrayBuffer.isView(data)
                ? [new Uint8Array(data.buffer, data.byteOffset, data.byteLength), FrameKind.END]
                : [new Uint8Array(data as ArrayBuffer), FrameKind.END];
        if (this.pendingSend) this.chain(() => this.sendMessage(bytes, kind));
        else this.sendMessage(bytes, kind);
    }

    close(code = 1000, reason = ""): void {
        if (this.state === CLOSING || this.state === CLOSED) return;
        const stream = this.stream;
        this.state = CLOSING;
        if (stream !== null) {
            this.port.send(stream, FrameKind.CLOSE, jsonPayload({code, reason}), this.prio);
            this.release();
        }
        // Like a browser socket, onclose arrives asynchronously after close().
        queueMicrotask(() => {
            if (this.state === CLOSED) return;
            this.state = CLOSED;
            this.onclose?.(new CloseEvent("close", {code, reason, wasClean: stream !== null}));
        });
    }

    private get prio(): boolean {
        return this.options.prio ?? false;
    }

    private chain(task: () => void | Promise<void>): void {
        const next = (this.pendingSend ?? Promise.resolve()).then(task).catch(() => undefined);
        this.pendingSend = next;
        void next.then(() => {
            if (this.pendingSend === next) this.pendingSend = null;
        });
    }

    private sendMessage(bytes: Uint8Array, endKind: number): void {
        const stream = this.stream;
        if (stream === null || this.state !== OPEN) return;
        let offset = 0;
        while (bytes.length - offset > MAX_PAYLOAD) {
            this.port.send(stream, FrameKind.DATA, bytes.subarray(offset, offset + MAX_PAYLOAD), this.prio);
            offset += MAX_PAYLOAD;
        }
        this.port.send(stream, endKind, bytes.subarray(offset), this.prio);
    }

    private deliver(bytes: Uint8Array<ArrayBuffer>, text: boolean): void {
        if (this.state !== OPEN) return;
        const data = text
            ? fromUtf8(bytes)
            : this.binaryType === "arraybuffer" ? bytes.buffer : new Blob([bytes]);
        this.onmessage?.(new MessageEvent("message", {data}));
    }

    private release(): void {
        if (this.stream === null) return;
        this.port.release(this.stream);
        this.stream = null;
    }

    private abnormalClose(): void {
        if (this.state === CLOSED) return;
        this.state = CLOSED;
        this.onerror?.(new Event("error"));
        this.onclose?.(new CloseEvent("close", {code: ABNORMAL_CLOSURE, reason: "", wasClean: false}));
    }
}
