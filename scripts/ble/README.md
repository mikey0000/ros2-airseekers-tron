# BLE link measurements (phase 0)

Tools for the measurements in `docs/DESIGN_ble_transport_2026-10.md`, section 5. They answer
how fast, how lossy and how laggy the vendor BLE module's serial pipe is, before any bridge
is built. Results go into `docs/analysis/2026-10-30_ble_link_measurements.md`.

| file | runs on | what |
|---|---|---|
| `ble_echo_daemon.py` | mower (host or `mower_jazzy` container) | owns `/dev/serial_ble`, answers the test frames. Python stdlib only, no ROS, commands nothing |
| `ble_link_test.py` | Linux laptop with BlueZ | connects to `AIRSEEKERS_*`, runs the tests, prints a markdown report |
| `ble_bluez.py` | laptop | BlueZ D-Bus client used by `ble_link_test.py` |
| `ble_frame.py` | both | framing + CRC-16, shared |
| `test_ble_frame.py` | anywhere | unit tests: `python3 -m pytest scripts/ble` or `python3 scripts/ble/ble_frame.py selftest` |
| `ble_bridge.py` | mower (`mower_jazzy`, launched by `mower.launch.py`) | the BLE tunnel daemon, see "ble_bridge" below |
| `ble_mux.py`, `ble_ws.py`, `ble_backend.py`, `ble_streams.py` | mower | the bridge's tunnel framing and link session, WebSocket client, HTTP client, stream handlers |
| `test_ble_mux.py`, `test_ble_ws.py`, `test_ble_bridge.py`, `ble_test_fakes.py` | anywhere | bridge tests (no hardware: a PTY pair and a fake backend) |

Phase-0 frame (`ble_frame.py`, echo daemon and link test only): `AA 55 | len u16 | seq u32 |
type u8 | payload | crc16-ccitt u16`, little-endian, CRC over everything between the sync word
and the CRC. Types are listed in `ble_frame.py`. The bridge uses a different frame, defined in
`docs/ble_tunnel_protocol.md` and implemented in `ble_mux.py`.

## 1. Mower side

The daemon must be the only process with `/dev/ttyS3` open. Nothing in the current stack
opens it (`base_ble` is not launched), but the daemon checks and refuses to start if
something does.

Copy the scripts to the mower if they are not there yet (from the repo root on the laptop):

```sh
scp -r ros2_stack/scripts/ble airseekers@<mower-ip>:/userdata/ros2_stack/scripts/
```

Then pick one of the two ways to run it.

**A. Inside the running container (stack stays up):**

```sh
ssh airseekers@<mower-ip>
sudo ls -l /proc/[0-9]*/fd 2>/dev/null | grep -E 'ttyS3|serial_ble'   # must print nothing
docker exec -it mower_jazzy python3 /work/scripts/ble/ble_echo_daemon.py --port /dev/ttyS3 --stats
```

**B. On the host with the stack stopped:**

```sh
ssh airseekers@<mower-ip>
docker stop mower_jazzy
cd /userdata/ros2_stack
python3 scripts/ble/ble_echo_daemon.py --stats
```

Leave it running in that terminal. Every 5 s it prints its counters. If the module sends
anything that is not a test frame (status text on connect or disconnect, for example), the
daemon prints a hexdump of it, which tells us something about the module. Stop it with Ctrl-C.
After option B, start the stack again with `docker start mower_jazzy`.

## 2. Laptop side

Requirements: BlueZ (`bluetoothctl`), Python 3 with `dbus-python` and PyGObject. Check:

```sh
python3 -c "import dbus, gi; print('ok')"
# if that fails (Debian/Ubuntu): sudo apt install python3-dbus python3-gi
# a venv needs --system-site-packages to see them
```

Disconnect any phone from the mower first; the module takes one connection. If
`bluetoothctl` has the mower connected with `notify on`, turn that off (the tool still works,
but falls back to slower notification delivery).

From the repo root, with the mower within a few metres:

```sh
python3 ros2_stack/scripts/ble/ble_link_test.py --out ~/ble_run1.md
```

It finds the first device whose name starts with `AIRSEEKERS_` (use `--address <mower-ble-address>`
if there are several), connects, and runs in order: GATT info and a read of `c1db0003`, the
write size sweep, 30 s of 10 Hz ping, 60 s phone-to-mower, 60 s mower-to-phone, 5
disconnect/reconnect cycles. About 4 minutes in total. Frames go out on `c1db0004` and come back as notifications on `c1db0002`.

Follow-up runs, only if run 1 shows loss:

```sh
# phone->mower lost frames: find the highest lossless offered rate
python3 ros2_stack/scripts/ble/ble_link_test.py --tests sink --sink-rate 8000 --out ~/ble_sink8k.md
python3 ros2_stack/scripts/ble/ble_link_test.py --tests sink --sink-rate 10000 --out ~/ble_sink10k.md
# mower->phone lost frames or CRC errors: same with the mower's send rate
python3 ros2_stack/scripts/ble/ble_link_test.py --tests source --source-rate 6000 --out ~/ble_src6k.md
python3 ros2_stack/scripts/ble/ble_link_test.py --tests source --source-rate 9000 --out ~/ble_src9k.md
# a write size above failed: rerun throughput with the largest size that passed
python3 ros2_stack/scripts/ble/ble_link_test.py --tests sink,source --chunk 180 --out ~/ble_180.md
```

Other options: `--tests info,sweep,...`, `--sizes`, `--chunk`, `--*-seconds`, `--ping-hz`,
`--no-wwr`, `--reconnects`. See `--help`.

## 3. What to paste back

1. The `--out` file(s). Device names are masked and `--address` is replaced by `<address>`.
2. The daemon's last two stats lines and any `unframed bytes` hexdumps it printed.
3. `bluetoothctl --version` and the laptop model / Bluetooth chip, if known.

## 4. Bench test without a mower

The daemon and the client also run against a local PTY pair, which checks the framing and the
test logic (the PTY has no 115200 limit, so its rates mean nothing):

```sh
socat pty,raw,echo=0,link=/tmp/ble_m pty,raw,echo=0,link=/tmp/ble_l &
python3 scripts/ble/ble_echo_daemon.py --port /tmp/ble_m --force --stats &
python3 scripts/ble/ble_link_test.py --serial /tmp/ble_l --sink-seconds 3 --source-seconds 3 \
    --ping-seconds 2 --source-rate 11000
```

(`--force` because socat itself holds the PTY open.)

## Notes

- The UART is 115200 8N1: at most 11 520 B/s on the wire, about 11 000 B/s of payload with
  244-byte frames. Anything above that in the phone-to-mower direction has to be buffered by
  the module or is lost, because there is no hardware flow control.
- Write-without-response is used for the sink test only if the characteristic advertises it
  (BlueZ listed only `write` on 2026-10-30, so expect "with response").
- Notifications are read through `AcquireNotify` (a socket per notification) when BlueZ
  allows it, else through `StartNotify` and D-Bus signals. The report says which.
- The Android repeat (does the 500-byte MTU hold on a phone) needs a Web Bluetooth page and is
  not covered here.

## ble_bridge

`ble_bridge.py` is the mower end of the BLE transport (`docs/DESIGN_ble_transport_2026-10.md`).
It owns `/dev/serial_ble` (falls back to `/dev/ttyS3`), speaks the tunnel protocol of
`docs/ble_tunnel_protocol.md` with the web app, checks the HELLO token against
`GET http://127.0.0.1:4006/api/ble/token`, and forwards the tunnelled HTTP requests and
WebSockets to the Go backend on `http://127.0.0.1:4006` with `X-Mower-Transport: ble`. Python
stdlib only, no ROS.

`launch/mower.launch.py` starts it in `mower_jazzy` (`ble_bridge:=true` by default, respawn
after 5 s). To run it by hand, start the stack with `ble_bridge:=false` (or stop the launched
one) and then:

```sh
docker exec -it mower_jazzy python3 /work/scripts/ble/ble_bridge.py --log-level DEBUG
```

| option | default | |
|---|---|---|
| `--port` | `/dev/serial_ble`, else `/dev/ttyS3` | serial device |
| `--backend` | `http://127.0.0.1:4006` | Go backend |
| `--log-level` | `INFO` | `INFO`: session start/end, one line per stream open and close, refusals. `DEBUG`: also every frame in and out. `WARNING`: only problems (refused HELLO, seq gaps, token fetch failures) |
| `--token-ttl` | `30` | seconds the backend token is cached |
| `--session-timeout` | `5` | seconds without a frame before the session is dropped |
| `--force` | off | start although another process has the port open |

**The echo daemon and the bridge never run together.** Both refuse to start while another
process has the port open (a `/proc` scan, `--force` skips it) and take `TIOCEXCL`; the bridge
also holds an exclusive `flock` on the port so a second bridge cannot start. For phase-0
measurements, start the stack with `ble_bridge:=false` first; while the echo daemon runs, a
launched bridge would exit and be respawned every 5 s.

Exit codes: 0 after SIGTERM / SIGINT, 1 on a serial error (launch respawns it), 2 when the port
is busy or locked.

Tests: `python3 -m pytest scripts/ble -q` (about 3 s; a PTY pair stands in for the UART and an
asyncio fake backend serves the token, JSON, gzip, chunked and WebSocket endpoints).
