# BLE transport, phases 1 and 2: job split (2026-10-30)

Design: `../DESIGN_ble_transport_2026-10.md`. Wire contract: `../ble_tunnel_protocol.md`.
Web seam contract: `third_party/mowglinext/gui/web/src/transport/types.ts`. Golden frames:
`scripts/ble/testdata/mux_vectors.json`. Four file-disjoint jobs run in parallel from one
snapshot; the main session merges them back with `git apply --3way`.

| job | branch | owns | deliverable |
|---|---|---|---|
| A secure origin | `ws/ble-https` | `gui/pkg/providers/tls*.go`, `gui/pkg/api/secure_origin.go`, `gui/pkg/api/ble_token.go`, `gui/pkg/api/api.go` (listener + route registration), `gui/web/src/components/settings/SecureOriginCard*.tsx`, `RemoteAccessSection.tsx` (mount the card), `docs/REMOTE_ACCESS.md`, i18n keys `secureOrigin.*`, `bleToken.*` | mower CA + HTTPS listener on 4443, CA download + install guide card, BLE token endpoints |
| B1 web seam + offline shell | `ws/ble-seam` | `gui/web/src/transport/index.ts`, `network.ts`, `registry.ts`, every fetch / WebSocket call site, `hooks/useApi.ts`, `hooks/reconnect.ts`, `hooks/multiplexedSocket.ts`, `hooks/useWS.ts`, `hooks/useContainerRestart.ts`, `utils/apiHost.ts`, `vite.config.ts`, `package.json`/`yarn.lock` (vite-plugin-pwa), `index.html` | all GUI traffic goes through `apiFetch` / `openApiSocket`; transports are registered and switchable; service worker precaches the shell |
| B2 BLE transport + connection UI | `ws/ble-web` | `gui/web/src/transport/ble/**`, `transport/connectionManager.ts`, `components/ConnectionChip.tsx`, `components/settings/ConnectionSection*.tsx`, `pages/SettingsPage.tsx` (one case), `hooks/useSettingsManager.ts` (one section entry), `components/AppShell.tsx` (chip only), `main.tsx` (one import), i18n keys `connection.*` | Web Bluetooth link, tunnel client (framing, streams, credits, fetch + socket adapters), manual Wi-Fi/Bluetooth switch with the one default rule, token storage |
| C mower bridge | `ws/ble-bridge` | `scripts/ble/ble_bridge.py`, `scripts/ble/ble_mux.py`, `scripts/ble/ble_ws.py`, `scripts/ble/test_ble_mux.py`, `scripts/ble/test_ble_bridge.py`, `scripts/ble/README.md`, `launch/mower.launch.py` (one `ExecuteProcess` block), `docs/ble_tunnel_protocol.md` (clarifications only) | `/dev/serial_ble` bridge daemon that terminates the tunnel and forwards to `http://127.0.0.1:4006`, with tests against a fake backend |

Cross-job contracts, fixed before the fan-out:

- Token endpoints (A provides, C consumes, B2 reads while on Wi-Fi): `GET /api/ble/token` →
  `{"token": string}`; `POST /api/ble/token/rotate` → `{"token": string}`. Token: 20 random
  bytes, base32 without padding, upper case (32 characters). Persisted in the GUI DB under
  `ble.token`, minted on first read.
- B1 registers transports by kind; B2 registers `ble` from its own module and only adds one
  import line to `main.tsx`. B2 codes against `transport/types.ts` plus this registry API,
  which B1 must implement exactly:

```ts
// web/src/transport/registry.ts (B1)
export function registerTransport(kind: TransportKind, factory: () => Transport): void;
export function getActiveTransport(): Transport;              // network until switched
export function getActiveKind(): TransportKind;
export function setActiveTransport(kind: TransportKind): void; // closes sockets opened by the seam with code 4000, callers reconnect
export function onTransportChange(cb: (kind: TransportKind) => void): () => void;
// web/src/transport/index.ts (B1)
export function apiFetch(path: string, init?: RequestInit): Promise<Response>;
export function openApiSocket(path: string, options?: SocketOptions): SocketLike;
```

- The bridge marks tunnelled requests with `X-Mower-Transport: ble` so phase 3 can rate-limit
  in the backend without touching the tunnel.
- Nothing in these jobs deploys to the mower or restarts anything on it; the owner does that.
