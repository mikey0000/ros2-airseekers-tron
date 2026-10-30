/**
 * Single source of truth for where the backend API lives.
 *
 * In production the GUI is served by the Go backend itself, so
 * window.location.host is always correct. In dev the same relative host goes
 * through the vite proxy (vite.config.ts), which rewrites the Origin header —
 * REQUIRED because the backend rejects WebSocket upgrades whose Origin host
 * differs from the request Host (403). Point the proxy at a robot with
 * MOWGLI_API_TARGET=http://<robot>:4006 when running vite.
 *
 * VITE_API_HOST remains as an explicit escape hatch that bypasses the proxy,
 * but note that WebSocket upgrades sent straight to a backend on another
 * host:port fail its origin check.
 *
 * Only the network transport (src/transport/network.ts) uses these; everything else
 * goes through apiFetch / openApiSocket so the active transport (Wi-Fi or Bluetooth)
 * carries it. src/transport/noDirectBackendAccess.test.ts enforces that.
 */
export function apiHost(): string {
    const override = import.meta.env.VITE_API_HOST as string | undefined;
    return override ?? window.location.host;
}

/** Base for http(s) fetches. Empty string = same-origin relative URLs. */
export function httpBase(): string {
    const override = import.meta.env.VITE_API_HOST as string | undefined;
    return override ? `http://${override}` : '';
}

/** Base for ws(s) URLs, e.g. `${wsBase()}/api/mowglinext/multiplex`. */
export function wsBase(): string {
    const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
    return `${protocol}://${apiHost()}`;
}
