# Design: the web app over Bluetooth LE

Status: proposed 2026-10-30. Owner requirement: the GUI must be able to connect over BLE and
do everything it does over Wi-Fi. BLE is the fallback when Wi-Fi is down or poor, and the
preferred link for driving the mower by hand when Wi-Fi is unreliable.

## 1. What exists

- The Tron has a vendor BLE module wired as a transparent UART bridge to `/dev/serial_ble`
  (`/dev/ttyS3`, 115200 8N1). GATT (read with BlueZ, `mower_docs/04-ble-protocol.md`):
  vendor service `c1db0001-431d-4cbe-96e2-9f9286c41572`, serial pipe characteristic
  `c1db0002-…` (read / write / notify), plus a read-only `c1db0003-…` and a write-only
  `c1db0004-…` of unknown purpose. No pairing or bonding. BlueZ negotiated a 500-byte MTU.
- The web app talks only to the Go backend on port 4006: REST under `/api/…`, one
  multiplexed topic WebSocket (`/api/mowglinext/multiplex`, MessagePack frames), the joystick
  publish socket (`/api/mowglinext/publish/joy`), the heartbeat socket, and the video relay
  on 8080. It never talks to foxglove_bridge directly. There is no service worker; the PWA
  manifest only makes it installable.
- `src/base_ble` is a port of the vendor phone-app frame protocol. It is not launched.
  Upstream MowgliNext has no Bluetooth work at all (checked `dev` and `main`, 2026-10-29).

## 2. Design

**Tunnel, don't re-implement.** One generic transport carries the existing HTTP and
WebSocket API over the BLE serial pipe. Every current and future GUI feature then works over
BLE without per-feature code, subject to bandwidth.

```
phone (Chrome)                          mower
 web app ── Transport ──┐               ┌── ble_bridge ──► http://127.0.0.1:4006 (Go backend)
   fetch / WebSocket    ├─ network ─────┤
   (unchanged callers)  └─ BLE (GATT) ──┘   /dev/serial_ble, own framing
```

- **Web side.** A `Transport` seam under `httpBase()` and the three WebSocket constructors
  (`hooks/multiplexedSocket.ts`, joystick, heartbeat). The network transport is today's code.
  The BLE transport uses Web Bluetooth on the serial-pipe characteristic: writes chunked at
  MTU−3, notifications reassembled. Frames multiplex streams: stream 0 control (hello,
  auth, MTU, ping), one stream per in-flight HTTP request (method, path, headers subset,
  gzip body), one stream per WebSocket. Callers keep using `fetch`-shaped and
  `WebSocket`-shaped objects.
- **Mower side.** A small bridge daemon (Python, stdlib + the same framing module) owns
  `/dev/serial_ble`, terminates the framing, and forwards to the local backend. Runs in the
  `mower_jazzy` container or as a sidecar. It replaces the vendor BLE protocol on that port
  (the vendor app is not supported anyway; `src/base_ble` is kept only as documentation of
  the vendor frames).
- **Framing.** Length-prefixed frames with a CRC-16, stream id, flags (open/data/close),
  sequence number and a credit-based flow control per stream, because the UART between the
  module and the RK3588 has no hardware flow control and the module's buffer size is unknown.
  Keep-alive every second; a link is dead after three missed.
- **Measured link (phase 0, 2026-10-30, `analysis/2026-10-30_ble_link_measurements.md`).**
  Phone writes go to `c1db0004`, mower bytes arrive as notifications on `c1db0002`. Mower ->
  phone is clean at 5.7 KB/s and stalls above ~6-7 KB/s (module buffer overflow, recovers on
  its own); phone -> mower is 1.1-1.8 KB/s, limited by ~165 ms per write-with-response, so use
  497-byte writes with one in flight. Small-frame round trip median 78 ms, p95 ~100 ms, no
  loss. Reconnect can take 20-30 s. Flow control must be credit based on the downlink so the
  module never overflows.
- **Bandwidth policy (BLE only).** The backend learns the transport from the hello and:
  rate-limits multiplex topics (pose 5 Hz, status 1 Hz, nothing above 10 Hz), gzips REST
  bodies, serves map geometry in a compact form, and refuses video with a clear "not
  available over Bluetooth" that the GUI renders in place of the stream. The joystick socket
  gets priority (small frames, sent first).
- **Switching (owner decision 2026-10-30).** The link is a manual switch in the GUI: a
  "Connection" control (status chip in the header, full control in Settings) with Wi-Fi /
  Bluetooth. No automatic flapping between the two while both are up. The one default rule:
  when the app starts, or the network link has been dead for ~3 s (today's
  `hooks/reconnect.ts` liveness), and a previously permitted BLE device is in range, the app
  switches to Bluetooth by itself and says so; it switches back only when the user taps
  Wi-Fi. The first Bluetooth connection always needs a tap (`requestDevice` requires a user
  gesture); later connections reuse the permission via `navigator.bluetooth.getDevices()`.
  The Drive page shows which link the joystick is on.
- **Security.** The vendor module has no pairing: anyone in radio range can connect. The
  bridge must require a token in the hello. The token is minted by the backend and shown
  once in Settings (QR / copy) while on Wi-Fi, and stored by the app. Without a valid token
  the bridge answers only "unauthenticated" and drops the link. (The LAN HTTP API has no
  auth today either; this does not make BLE weaker than Wi-Fi, but BLE reaches past the
  Wi-Fi password, so the token is mandatory.)

## 3. HTTPS: how the mower gets a secure origin

Web Bluetooth and service workers only work on a secure origin. `http://<mower>:4006` is not
one. Three ways, all can coexist; the GUI only needs to be opened from one of them.

1. **Mower-issued certificate (default for everyone).** At first boot the backend creates
   its own CA and a server certificate for `https://<hostname>.local` (mDNS) and the
   current LAN IP, and serves HTTPS on 4443 next to HTTP on 4006. Settings → Remote access
   gains "Install the mower's certificate" with the CA file and the two steps for Android
   (Settings → Security → Install a certificate → CA certificate). One-time per phone. After
   that Chrome treats `https://<hostname>.local:4443` as secure. The CA is per mower, never
   leaves it, and is regenerated on "factory reset".
2. **Tailscale HTTPS (already in upstream; not used on this robot today, kept as an option).**
   `ServeHTTPS` publishes the GUI as
   `https://<node>.<tailnet>.ts.net` with a real Let's Encrypt certificate. Users who run
   remote access get a secure origin for free. Offline it still works: the service worker
   serves the cached shell for that origin without contacting the server, and BLE takes
   over as the transport.
3. **Developer flag.** `chrome://flags/#unsafely-treat-insecure-origin-as-secure` with
   `http://<mower>:4006`. Fine for development, not for users.

iOS: Safari has no Web Bluetooth and third-party iOS browsers must use WebKit. iPhone users
get BLE only through a native wrapper app later (same framing, same bridge); this design does
not block that.

## 4. Offline shell

`vite-plugin-pwa` precaches the app shell (HTML, JS, CSS, fonts, icons) at the secure origin.
API responses are never cached. The app opens with no Wi-Fi, shows "no network", and offers
Bluetooth. The first load of a new version needs the server once.

## 5. Unknowns to measure before building (phase 0)

| question | why it matters | how |
|---|---|---|
| sustained throughput phone→mower and mower→phone through the module | sets the rate policy; UART caps at ~11 KB/s | echo daemon on the mower, BlueZ client on the laptop, 60 s each way |
| byte loss under load (no flow control) | decides how aggressive credits must be | CRC-checked counters in the echo test |
| largest write the module accepts, and whether 500 MTU holds on Android | chunk size | vary write sizes; repeat with a phone |
| round-trip latency at 10 Hz small frames | joystick feel | ping stream |
| whether `c1db0004` is a config channel (baud, name) | a faster UART would double the budget | read `c1db0003`, try documented module AT forms only if the module can be identified |
| does the module keep advertising after a disconnect, reconnect time | auto-reconnect UX | repeat connect/disconnect |

Phase 0 tooling lives in `scripts/ble/` (echo daemon for the mower, laptop test client,
report template). The owner runs the daemon on the mower.

## 6. Phases

| phase | deliverable | depends on |
|---|---|---|
| 0 | measurements above, written to `docs/analysis/2026-10-30_ble_link_measurements.md` | **done 2026-10-30** |
| 1 | secure origin (mower CA + HTTPS listener on 4443, Settings → Remote access install card, `GET /api/ble/token`) and service worker shell cache | **built 2026-10-30, not yet deployed** |
| 2 | framing module (Python + TS, golden vectors in `scripts/ble/testdata/`), mower `ble_bridge` (`scripts/ble/ble_bridge.py`, launch arg `ble_bridge`), web BLE transport under the `Transport` seam (`web/src/transport/`), token auth, Settings → Connection section + header chip; REST + the three sockets tunnelled. Wire contract: `ble_tunnel_protocol.md` | **built 2026-10-30, not yet deployed or field-tested** |
| 3 | bandwidth policy in the backend, Drive-page preference, video placeholder, auto-reconnect; restrict `GET /api/ble/token` to same-origin requests (the API answers CORS `*` today, so any page open in the browser could read the token) | 2 |
| 4 | field test on the lawn at the Wi-Fi edge; docs for users | 3 |

Not in scope (agreed with the owner): video over BLE, iOS, keeping the vendor phone app
working. Automatic Wi-Fi/BLE hand-over beyond the single "offline but BLE in range" rule is
deferred until the link is proven in the field.
