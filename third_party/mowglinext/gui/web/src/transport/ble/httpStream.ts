import {type Frame, FrameKind, jsonPayload, MAX_PAYLOAD, parseJsonPayload, utf8} from "./frame.ts";
import {concatBytes, linkError, type StreamHandler, type StreamPort} from "./streamPort.ts";

/** ble_tunnel_protocol.md section 3.1. */
export const HTTP_METHODS = ["GET", "POST", "PUT", "PATCH", "DELETE"] as const;
/** Request headers the bridge forwards; everything else is dropped on the phone already. */
export const FORWARDED_REQUEST_HEADERS = ["content-type", "accept", "if-none-match", "range"] as const;

const NULL_BODY_STATUSES = new Set([101, 204, 205, 304]);

export interface HttpOpenRequest {
    t: "http";
    method: string;
    path: string;
    headers: Record<string, string>;
}

export interface PreparedRequest {
    open: HttpOpenRequest;
    body: Uint8Array;
}

/** Validate and normalise a fetch-style request into the OPEN JSON and the body bytes. */
export async function prepareHttpRequest(path: string, init: RequestInit): Promise<PreparedRequest> {
    const method = (init.method ?? "GET").toUpperCase();
    if (!(HTTP_METHODS as readonly string[]).includes(method)) {
        throw new TypeError(`Method ${method} is not supported over Bluetooth`);
    }
    const source = new Headers(init.headers);
    const headers: Record<string, string> = {};
    for (const name of FORWARDED_REQUEST_HEADERS) {
        const value = source.get(name);
        if (value !== null) headers[name] = value;
    }
    const {bytes, contentType} = await bodyBytes(init.body);
    if (contentType && headers["content-type"] === undefined) headers["content-type"] = contentType;
    return {open: {t: "http", method, path, headers}, body: bytes};
}

async function bodyBytes(body: BodyInit | null | undefined): Promise<{bytes: Uint8Array; contentType?: string}> {
    if (body === null || body === undefined) return {bytes: new Uint8Array(0)};
    if (typeof body === "string") return {bytes: utf8(body), contentType: "text/plain;charset=UTF-8"};
    if (body instanceof ArrayBuffer) return {bytes: new Uint8Array(body)};
    if (ArrayBuffer.isView(body)) return {bytes: new Uint8Array(body.buffer, body.byteOffset, body.byteLength)};
    if (typeof Blob !== "undefined" && body instanceof Blob) {
        return {bytes: new Uint8Array(await body.arrayBuffer()), contentType: body.type || undefined};
    }
    // FormData, URLSearchParams and ReadableStream: no caller sends these to the backend today.
    throw new TypeError("This request body type is not supported over Bluetooth");
}

interface ResponseHead {
    status: number;
    headers: Record<string, string>;
    /** No "headers" key: the bridge answered itself (403, 502) and sends no END. */
    bridgeMade: boolean;
}

/** One HTTP request/response on a tunnel stream. */
export class HttpExchange implements StreamHandler {
    private head: ResponseHead | null = null;
    private readonly chunks: Uint8Array[] = [];
    private settled = false;
    private readonly onAbort = () => this.abort();

    constructor(
        private readonly port: StreamPort,
        private readonly id: number,
        private readonly request: PreparedRequest,
        private readonly signal: AbortSignal | null,
        private readonly resolve: (response: Response) => void,
        private readonly reject: (error: unknown) => void,
    ) {}

    /** Sends OPEN, the body as DATA* END (an empty END when there is none). */
    start(): void {
        this.signal?.addEventListener("abort", this.onAbort);
        this.port.send(this.id, FrameKind.OPEN, jsonPayload(this.request.open), false);
        const body = this.request.body;
        let offset = 0;
        while (body.length - offset > MAX_PAYLOAD) {
            this.port.send(this.id, FrameKind.DATA, body.subarray(offset, offset + MAX_PAYLOAD), false);
            offset += MAX_PAYLOAD;
        }
        this.port.send(this.id, FrameKind.END, body.subarray(offset), false);
    }

    onFrame(frame: Frame): void {
        if (this.settled) return;
        switch (frame.kind) {
            case FrameKind.HEAD:
                this.head = parseHead(frame.payload);
                break;
            case FrameKind.DATA:
                this.chunks.push(frame.payload);
                break;
            case FrameKind.END:
            case FrameKind.TEXT_END:
                this.chunks.push(frame.payload);
                this.complete();
                break;
            case FrameKind.CLOSE:
                // A bridge-made answer (403, 502) is HEAD without "headers", no body, then CLOSE.
                // Any other CLOSE before END means the body is incomplete (e.g. 4499 backend failed).
                if (this.head?.bridgeMade && this.chunks.length === 0 && frame.payload.length === 0) this.complete();
                else this.finish(linkError(closeReason(frame.payload)));
                break;
        }
    }

    fail(reason: string): void {
        this.finish(linkError(reason));
    }

    private abort(): void {
        if (this.settled) return;
        this.port.send(this.id, FrameKind.CLOSE, new Uint8Array(0), false);
        this.finish(this.signal?.reason ?? new DOMException("The operation was aborted.", "AbortError"));
    }

    private complete(): void {
        const head = this.head;
        if (!head) {
            this.finish(linkError("response without a head"));
            return;
        }
        this.settle();
        buildResponse(head, concatBytes(this.chunks)).then(this.resolve, this.reject);
    }

    private finish(error: unknown): void {
        if (this.settled) return;
        this.settle();
        this.reject(error);
    }

    private settle(): void {
        this.settled = true;
        this.signal?.removeEventListener("abort", this.onAbort);
        this.port.release(this.id);
    }
}

function parseHead(payload: Uint8Array): ResponseHead | null {
    const json = parseJsonPayload(payload) as {status?: unknown; headers?: unknown} | null;
    if (!json || typeof json.status !== "number") return null;
    const headers: Record<string, string> = {};
    if (json.headers && typeof json.headers === "object") {
        for (const [k, v] of Object.entries(json.headers as Record<string, unknown>)) {
            if (typeof v === "string") headers[k.toLowerCase()] = v;
        }
    }
    return {status: json.status, headers, bridgeMade: json.headers === undefined};
}

export function closeReason(payload: Uint8Array): string {
    const json = parseJsonPayload(payload) as {code?: unknown; reason?: unknown} | null;
    if (!json) return "stream closed";
    const reason = typeof json.reason === "string" && json.reason ? `: ${json.reason}` : "";
    return `stream closed (${typeof json.code === "number" ? json.code : "no code"}${reason})`;
}

async function buildResponse(head: ResponseHead, raw: Uint8Array<ArrayBuffer>): Promise<Response> {
    if (head.status < 200 || head.status > 599) throw linkError(`unexpected status ${head.status}`);
    const headers = new Headers(head.headers);
    let body = raw;
    if (headers.get("content-encoding")?.toLowerCase() === "gzip") {
        body = await gunzip(raw);
        headers.delete("content-encoding");
    }
    return new Response(NULL_BODY_STATUSES.has(head.status) ? null : body, {status: head.status, headers});
}

async function gunzip(bytes: Uint8Array<ArrayBuffer>): Promise<Uint8Array<ArrayBuffer>> {
    const stream = new Response(bytes).body!.pipeThrough(new DecompressionStream("gzip"));
    return new Uint8Array(await new Response(stream).arrayBuffer());
}
