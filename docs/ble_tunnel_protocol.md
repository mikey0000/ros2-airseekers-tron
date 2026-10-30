# BLE tunnel protocol (v1)

How the web app reaches the Go backend over the Tron's Bluetooth LE module when there is
no usable Wi-Fi. Design and measurements: `DESIGN_ble_transport_2026-10.md`,
`analysis/2026-10-30_ble_link_measurements.md`. Two implementations must agree on this
document byte for byte:

- mower: `scripts/ble/ble_bridge.py` (Python, stdlib only), owns `/dev/serial_ble`;
- phone: `third_party/mowglinext/gui/web/src/transport/ble/` (TypeScript, Web Bluetooth).

Golden frames for both test suites: `scripts/ble/testdata/mux_vectors.json`.

## 1. Physical link

The vendor BLE module is a transparent UART bridge, 115200 8N1, to `/dev/serial_ble`.
GATT service `c1db0001-431d-4cbe-96e2-9f9286c41572`:

| characteristic | direction | use |
|---|---|---|
| `c1db0004-431d-4cbe-96e2-9f9286c41572` | phone → mower | write with response, at most 497 bytes per write, one write in flight |
| `c1db0002-431d-4cbe-96e2-9f9286c41572` | mower → phone | notifications, up to 497 bytes each |

Both directions are **byte streams**. A write or a notification may hold several frames, a
partial frame, or both. Receivers resynchronise on the sync word. The module has no flow
control and overflows (stalls) above roughly 6 KB/s mower → phone, so the mower side is
credit limited and rate capped (section 5). Advertised name: `AIRSEEKERS_` + 6 characters.

## 2. Frame

All integers little-endian.

```
offset  size  field
0       2     sync       0xAA 0x55
2       2     len        payload length, 0..480
4       2     stream     0 = control, odd = opened by the phone, even = reserved
6       1     kind       table below
7       1     seq        per sender, +1 per frame, wraps 255 -> 0
8       len   payload
8+len   2     crc16      CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection,
                         no final xor) over bytes 2 .. 8+len (len, stream, kind, seq, payload)
```

`MAX_PAYLOAD = 480`, so a frame is at most 490 bytes and always fits one 497-byte write. A
frame with a bad CRC is dropped and the receiver resynchronises on the next sync word.

`seq` is per sender and per link session. A receiver that sees a gap sends `RESET` (section 4)
because a lost frame breaks every stream's message boundaries. `HELLO` starts both sequences:
the phone sends it with seq 0 (the mower takes whatever seq the `HELLO` carries as the start of
the phone's sequence) and `WELCOME` is the mower's seq 0. `HELLO` and `RESET` frames are never
treated as a gap.

| kind | name | stream | payload | direction |
|---|---|---|---|---|
| 0 | `OPEN` | n | JSON open request (section 3) | phone → mower |
| 1 | `HEAD` | n | JSON response head (section 3) | mower → phone |
| 2 | `DATA` | n | bytes, message continues | both |
| 3 | `END` | n | last bytes of a **binary** message, may be empty | both |
| 4 | `TEXT_END` | n | last bytes of a **text** (UTF-8) message, may be empty | both |
| 5 | `CLOSE` | n | optional JSON `{"code": int, "reason": string}` | both |
| 6 | `CREDIT` | 0 | u32 cumulative wire bytes received (section 5) | phone → mower |
| 7 | `RESET` | 0 | none | both |
| 8 | `PING` | 0 | 8 opaque bytes | phone → mower |
| 9 | `PONG` | 0 | the 8 bytes from `PING` | mower → phone |
| 10 | `HELLO` | 0 | JSON (section 4) | phone → mower |
| 11 | `WELCOME` | 0 | JSON (section 4) | mower → phone |

A **message** on a stream is the concatenation of zero or more `DATA` payloads followed by one
`END` or `TEXT_END` payload. `HEAD`, `OPEN` and `CLOSE` are single frames (their JSON is
small; a `HEAD` whose JSON would exceed 480 bytes drops headers until it fits).

## 3. Streams

Stream ids are allocated by the phone: 1, 3, 5, … up to 65535, then wrap, skipping ids still
open. Stream 0 is the control stream. The mower never opens streams in v1.

### 3.1 HTTP request

```
phone:  OPEN {"t":"http","method":"GET","path":"/api/settings","headers":{...}}
phone:  DATA* END            request body; a request without a body sends an empty END at once
mower:  HEAD {"status":200,"headers":{...}}
mower:  DATA* END            response body, possibly gzipped (below)
mower:  CLOSE
```

Either side may send `CLOSE` early to abort; the stream id is free once both sides have seen
a `CLOSE` or the stream ended normally. One `CLOSE`, from either side, ends the stream on both
sides; it is never answered with another `CLOSE`. The phone may open many HTTP streams at once; the
bridge serves them concurrently against `http://127.0.0.1:4006`.

Open request fields: `method` (GET, POST, PUT, PATCH, DELETE; anything else, or a missing
`path`, answers `CLOSE {"code":4400}`), `path` (must start with `/api/` or equal
`/web-build.json`, checked on the part before any `?`; printable ASCII only, no `..` segment;
anything else answers `HEAD {"status":403}` + `CLOSE` without touching the backend), `headers` (only these are forwarded, lower-case keys: `content-type`,
`accept`, `if-none-match`, `range`). The bridge adds `Host: 127.0.0.1:4006`,
`X-Mower-Transport: ble` and `Accept-Encoding: identity`.

Response head: `status` and these response headers when present, lower-case keys:
`content-type`, `etag`, `cache-control`, `content-disposition`, `last-modified`, plus
`content-encoding: gzip` when the bridge compressed the body. The bridge gzips a body when
its content type starts with `text/`, `application/json`, `application/javascript`,
`application/yaml`, `application/x-yaml` or `image/svg+xml` and the body is at least 256
bytes (a body of unknown length is compressed once its first 256 bytes have arrived). The
phone inflates it (`DecompressionStream("gzip")`) and removes the header before building the
`Response`.

A response from the backend always has `headers` (possibly `{}`) and always ends with `END`
(empty for an empty body), then an empty `CLOSE`. A response made by the bridge itself
(`403`, `502`) is `HEAD {"status":N}` with no `headers` and no body, then `CLOSE`. A `CLOSE`
before `END` means the body is incomplete and the phone fails the request; the bridge sends
`CLOSE {"code":4499,"reason":"backend failed"}` when the backend connection breaks after
`HEAD`. The request body is buffered by the bridge and sent to the backend with
`Content-Length` once `END` arrives.

### 3.2 WebSocket

```
phone:  OPEN {"t":"ws","path":"/api/mowglinext/multiplex","prio":false,"lossy":true}
mower:  HEAD {"status":101}                      (or another status, then CLOSE)
both:   DATA* END | DATA* TEXT_END               one WebSocket message each
either: CLOSE {"code":1000,"reason":""}
```

`path` must start with `/api/` (same character rules as HTTP, else `HEAD {"status":403}` +
`CLOSE`). The bridge opens a WebSocket client connection to the backend with no `Origin`
header (the backend accepts that) and `X-Mower-Transport: ble`. Messages the phone sends
before `HEAD 101` arrives are queued and delivered once the backend socket is open. A backend
that does not upgrade gives `HEAD` with its status (`502` when it cannot be reached), then
`CLOSE`.

Close codes: the phone's `CLOSE` code goes to the backend when a client may send it (1000-1003,
1007-1014, 3000-4999), else 1000; no payload means 1000. The backend's close code and reason
come back in the bridge's `CLOSE` (1005 when the backend sent none, 1006 when its connection
dropped without a close frame). Backend sockets closed because the session ended (RESET,
timeout, new `HELLO`) get 1001. Binary
WebSocket frames map to `END`, text frames to `TEXT_END`. The backend's 1 Hz heartbeat
messages travel as ordinary messages, so the GUI's liveness logic in `hooks/reconnect.ts`
works unchanged.

- `prio: true` (joystick): the bridge sends this stream's frames before any other stream's,
  after control frames.
- `lossy: true` (status multiplex): when more than 4 complete messages are queued for this
  stream and cannot be sent because of credit or rate limits, the bridge drops the oldest
  queued messages until 4 remain. Message boundaries are never broken: a message whose first
  frame has been sent is always completed and is not counted. Non-lossy streams queue without
  limit until the stream is closed.

## 4. Control stream and session

The first frame after a GATT connection must be `HELLO`:

```json
{"v":1,"token":"<ble token>","client":"mowglinext-web/<build id>","window":2048,"max_rate":5000}
```

`window` is how many wire bytes the mower may send beyond the last `CREDIT` (default 2048 if
absent, maximum 8192). `max_rate` is a cap in wire bytes per second (default 5000, maximum
8000). A value that is missing, not an integer or below 1 takes the default; a value above
the maximum is clamped. The mower answers on stream 0:

```json
{"v":1,"host":"<mower hostname>","max_payload":480,"session":"<8 hex chars>"}
```

or, if the token is wrong or missing or the backend cannot be asked,
`{"v":1,"error":"unauthenticated"}` (or `"error":"backend_unavailable"`; `"bad_hello"` when the
payload is not a JSON object with `"v":1`), after which the mower ignores every frame until
the next `HELLO`. The token is minted by the Go backend (`GET /api/ble/token`, rotate with
`POST /api/ble/token/rotate`); the bridge fetches it from `http://127.0.0.1:4006/api/ble/token`
and caches it for 30 s. A token that does not match a cached copy is checked once more against
a fresh fetch, so a rotation takes effect at once. Comparison is constant time.

Every `HELLO` starts a new session: the old one is discarded first (as on `RESET`). Frames
that arrive while the mower is checking the token are held and processed after the `WELCOME`,
or dropped if the `HELLO` is refused.

`RESET` (either side, stream 0): both sides discard every stream (backend WebSockets are
closed, in-flight HTTP requests abandoned), set their send `seq` to 0 and expect the peer's
next `seq` to be 0. The session and token stay valid. The phone sends `RESET` when it detects
a `seq` gap; the mower does the same. Precisely:

- The `seq` of a `RESET` frame is not checked; the mower sends it with seq 0, and its next
  frame has seq 0 again.
- The receiver of a `RESET` does not answer it with a `RESET`.
- After *sending* a `RESET`, a side drops incoming frames whose `seq` is not 0 without
  treating them as a gap (the peer sent them before it saw the `RESET`), until the first
  frame with seq 0 arrives. These stale frames are not counted for `CREDIT`.
- `CREDIT` counting restarts after the `RESET` frame: the `RESET` itself is not counted.

Keep-alive: the phone sends `PING` every 1000 ms while connected (more often is not needed;
other traffic does not replace it). The mower answers `PONG` at once. The phone treats the
link as dead after 3000 ms without any frame and reconnects the GATT link. The mower discards
the session after 5000 ms without any frame (closing its backend connections) and waits for a
new `HELLO`. After a GATT disconnect the bridge does the same.

## 5. Flow control and rate (mower → phone only)

The phone counts every wire byte it receives (sync word to CRC, every frame, including
control frames) and sends `CREDIT` with the cumulative count (u32, wraps) whenever it has
received at least `window / 4` new bytes since the last `CREDIT`, or 250 ms have passed with
new bytes received. The mower stops sending stream frames (not control frames) while
`bytes_sent - last_credit >= window` (mod 2^32; a `CREDIT` ahead of `bytes_sent` counts as
nothing in flight). On `HELLO` and `RESET` both counters start at 0: after `HELLO` the
`WELCOME` is the first counted frame.

In addition the bridge paces its output with a token bucket of `max_rate` bytes per second
(burst `window`, at least one 490-byte frame; full at session start). Control frames (stream 0,
and the `CLOSE` error replies of section 6) skip both the credit and the pacing gate but count
toward both. Phone → mower needs no credit: GATT write-with-response and one write in
flight already pace it at roughly 1 to 2 KB/s.

Send order on the mower: control frames, then `prio` streams, then other streams round robin.

## 6. Limits and errors

- Unknown `kind`: frame ignored. Unknown stream id (never opened in this session) with any
  kind but `OPEN` and `CLOSE`: `CLOSE {"code":4404,"reason":"no such stream"}`. A `CLOSE` on an
  unknown stream, and any frame on a stream that already ended in this session, is dropped
  silently.
- `OPEN` on an id already open: `CLOSE {"code":4409,"reason":"stream in use"}`; the mower ends
  its existing stream on that id as well, so the id is free on both sides afterwards.
- `OPEN` on stream 0 or an even id: `CLOSE {"code":4400,"reason":"bad stream id"}`.
- Backend connection failure for an HTTP stream: `HEAD {"status":502}` + `CLOSE`; after
  `HEAD` was sent: `CLOSE {"code":4499,"reason":"backend failed"}`.
- JSON that does not parse in `OPEN`/`HELLO`: `CLOSE {"code":4400}` / `WELCOME` with
  `"error":"bad_hello"`.
- Close codes 4400..4499 are tunnel errors; 1000..1015 and 3000..3999 are passed through
  from the backend WebSocket.
