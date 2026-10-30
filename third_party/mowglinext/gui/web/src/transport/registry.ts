import {NetworkTransport} from "./network.ts";
import {
    TRANSPORT_SWITCHED_CLOSE_CODE,
    type SocketLike,
    type SocketOptions,
    type Transport,
    type TransportKind,
} from "./types.ts";

/**
 * Which transport the GUI talks to the backend through, and the sockets opened on it.
 *
 * `network` is registered here and active until {@link setActiveTransport} switches.
 * Other kinds (`ble`) register themselves from their own module. Factories are called
 * lazily, at most once per kind (until the kind is registered again).
 */

const factories = new Map<TransportKind, () => Transport>();
const instances = new Map<TransportKind, Transport>();
const changeListeners = new Set<(kind: TransportKind) => void>();
/** Sockets opened through the seam that have not closed yet. */
const liveSockets = new Set<TrackedSocket>();
let activeKind: TransportKind = "network";

/** Register (or replace) the factory for `kind`. Replacing drops the cached instance. */
export function registerTransport(kind: TransportKind, factory: () => Transport): void {
    factories.set(kind, factory);
    instances.delete(kind);
}

function transportFor(kind: TransportKind): Transport {
    let transport = instances.get(kind);
    if (!transport) {
        const factory = factories.get(kind);
        if (!factory) throw new Error(`transport "${kind}" is not registered`);
        transport = factory();
        instances.set(kind, transport);
    }
    return transport;
}

/** The transport new requests and sockets go through (network until switched). */
export function getActiveTransport(): Transport {
    return transportFor(activeKind);
}

export function getActiveKind(): TransportKind {
    return activeKind;
}

/**
 * Make `kind` the active transport. Throws if `kind` was never registered; no-op if it
 * is already active. Otherwise every socket the seam opened and that is still live is
 * closed with {@link TRANSPORT_SWITCHED_CLOSE_CODE} (its owner sees an `onclose` with
 * that code and reconnects, which now goes through `kind`), then listeners are told.
 * In-flight fetches are left to finish or fail on the old transport.
 */
export function setActiveTransport(kind: TransportKind): void {
    if (!factories.has(kind)) throw new Error(`transport "${kind}" is not registered`);
    if (kind === activeKind) return;
    activeKind = kind;
    for (const socket of Array.from(liveSockets)) socket.closeForSwitch();
    for (const cb of Array.from(changeListeners)) {
        try {
            cb(kind);
        } catch (err) {
            console.error("transport: change listener threw", err);
        }
    }
}

/** Called with the new kind after every switch. Returns an unsubscribe function. */
export function onTransportChange(cb: (kind: TransportKind) => void): () => void {
    changeListeners.add(cb);
    return () => {
        changeListeners.delete(cb);
    };
}

/**
 * Open a socket on the active transport and track it until it closes, so a switch can
 * close it. Used by `openApiSocket` in `./index.ts`; callers use that instead.
 */
export function openTrackedSocket(path: string, options?: SocketOptions): SocketLike {
    const socket = new TrackedSocket(getActiveTransport().openSocket(path, options));
    liveSockets.add(socket);
    return socket;
}

/**
 * Wraps a transport's socket so the seam sees its close even though the owner assigns
 * the `on*` handlers after the socket is returned.
 */
class TrackedSocket implements SocketLike {
    onopen: ((ev: Event) => void) | null = null;
    onmessage: ((ev: MessageEvent) => void) | null = null;
    onclose: ((ev: CloseEvent) => void) | null = null;
    onerror: ((ev: Event) => void) | null = null;

    constructor(private readonly inner: SocketLike) {
        inner.onopen = (ev) => this.onopen?.(ev);
        inner.onmessage = (ev) => this.onmessage?.(ev);
        inner.onerror = (ev) => this.onerror?.(ev);
        inner.onclose = (ev) => {
            liveSockets.delete(this);
            this.onclose?.(ev);
        };
    }

    get url(): string {
        return this.inner.url;
    }

    get readyState(): number {
        return this.inner.readyState;
    }

    get binaryType(): BinaryType {
        return this.inner.binaryType;
    }

    set binaryType(value: BinaryType) {
        this.inner.binaryType = value;
    }

    send(data: string | ArrayBufferLike | ArrayBufferView | Blob): void {
        this.inner.send(data);
    }

    close(code?: number, reason?: string): void {
        liveSockets.delete(this);
        this.inner.close(code, reason);
    }

    /**
     * Close for a transport switch. The owner gets its `onclose` right away with
     * {@link TRANSPORT_SWITCHED_CLOSE_CODE}: a socket still connecting would otherwise
     * report 1006, and the old transport may never deliver a close event at all.
     */
    closeForSwitch(): void {
        liveSockets.delete(this);
        const inner = this.inner;
        inner.onopen = inner.onmessage = inner.onerror = inner.onclose = null;
        try {
            inner.close(TRANSPORT_SWITCHED_CLOSE_CODE, "transport switched");
        } catch {
            /* already closed */
        }
        const onclose = this.onclose;
        this.onopen = this.onmessage = this.onerror = this.onclose = null;
        onclose?.(new CloseEvent("close", {
            code: TRANSPORT_SWITCHED_CLOSE_CODE,
            reason: "transport switched",
            wasClean: true,
        }));
    }
}

registerTransport("network", () => new NetworkTransport());
