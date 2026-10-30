# BLE link measurements (phase 0)

Date: <run date>. Status: template, not yet measured.
Design: `docs/DESIGN_ble_transport_2026-10.md`, section 5. Tools and run instructions:
`scripts/ble/README.md`.

Setup: mower daemon run as <container / host>; laptop <model, Bluetooth chip>, BlueZ <version>;
distance <m>, line of sight <yes/no>.

## Thresholds

| question | good | marginal | bad |
|---|---|---|---|
| joystick round trip (d) | p95 < 150 ms and < 1 % lost | p95 150-300 ms | p95 > 300 ms or > 5 % lost |
| tunnel throughput, each direction (b, c) | ≥ 8 KB/s payload | 4-8 KB/s: REST + throttled topics only | < 4 KB/s: joystick + status only, no tunnel |
| loss at full rate (b, c) | 0 frames | lossless only below a rate cap: set credits to ≤ 80 % of that cap | lossy at every rate: bridge needs ACK + retransmit |
| largest write (a) | 497 B (MTU 500 − 3) | 244 B | < 180 B |
| reconnect (f) | Connect() < 3 s every time | 3-10 s | > 10 s or a failed cycle: module may not re-advertise |

Ceiling for reference: 115200 8N1 is 11 520 B/s on the UART, about 11 000 B/s of payload with
244-byte frames. A rate near that in (c) means the UART, not BLE, is the limit.

## Results

### (e) GATT info

| item | value |
|---|---|
| ATT MTU | |
| notifications via | |
| flags c1db0002 | |
| flags c1db0003 | |
| flags c1db0004 | |

c1db0003 contents:

```
<hexdump>
```

### (a) Write size sweep

| write B | accepted | echo ok | lost | CRC err | garbage B | median RTT ms |
|---|---|---|---|---|---|---|
| 20 | | | | | | |
| 100 | | | | | | |
| 180 | | | | | | |
| 244 | | | | | | |
| 497 | | | | | | |

### (d) Round trip, PING 10 Hz for 30 s

| sent | answered | lost | min ms | median ms | p95 ms | max ms |
|---|---|---|---|---|---|---|
| | | | | | | |

### (b) Phone to mower (SINK, 60 s)

| rate cap | write mode | frame B | offered B/s | received payload B/s | frames lost | mower CRC err | seq gaps |
|---|---|---|---|---|---|---|---|
| none | | | | | | | |

### (c) Mower to phone (SOURCE, 60 s)

| rate cap | frame B | received payload B/s | frames lost | CRC err | seq gaps | garbage B |
|---|---|---|---|---|---|---|
| full UART | | | | | | |

### (f) Disconnect / reconnect

| # | disconnect s | Connect() s | services resolved s | first PONG s | error |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |
| 4 | | | | | |
| 5 | | | | | |

### Module chatter

Unframed bytes the daemon saw on the UART (connect / disconnect messages, if any):

```
<hexdump or "none">
```

## Interpretation

- Throughput budget per direction: <B/s> → rate policy in design section 2.
- Credit window: <bytes> (from the lossless rate cap and the module's apparent buffer).
- Write chunk size for the bridge: <B>.
- Joystick over BLE: <yes / no / with caveats>.
- c1db0003 / c1db0004: <module identified or not; config channel worth probing or not>.
- Open: repeat (a) and (d) from an Android phone (Web Bluetooth) to confirm the MTU.

## Conclusions (2026-10-30, after runs 1-2 and the follow-up probes)

Setup: echo daemon in `mower_jazzy` on `/dev/ttyS3` at 115200; laptop with BlueZ 5.85, Intel AX211,
about 3 m from the mower; writes on `c1db0004`, notifications on `c1db0002`.

| question | answer |
|---|---|
| direction split | phone -> mower on `c1db0004` (write), mower -> phone as notifications on `c1db0002`; writes to `c1db0002` never reach the UART |
| notification size | one notification per frame up to 497 B (no 20-byte chunking); ATT MTU 500 |
| mower -> phone sustained | **5.7 KB/s payload clean at a 6 KB/s cap, 0 loss, 0 CRC errors.** At 8 KB/s the stream ran 49 frames then stopped; at 9.5 and 11 KB/s almost nothing arrived. The module's UART->BLE buffer overflows above ~6-7 KB/s and the stream stalls instead of degrading. The link recovered by itself (pings normal straight after) |
| phone -> mower sustained | bounded by write-with-response latency, not by the UART: ~165 ms + ~0.23 ms/B per write, so **1.1 KB/s with 244-B writes, 1.8 KB/s with 497-B writes**, 0 write errors. `write-without-response` is not advertised, so one write in flight is the rule |
| round trip, small frames | 10 Hz PING: **median 78 ms, p95 90-110 ms, max 204 ms, 0 lost** (three runs) |
| largest write | 497 B accepted and echoed intact |
| reconnect | after a disconnect the module sometimes needs 20-25 s to be seen again; BlueZ `Connect()` returned NoReply or `le-connection-abort-by-local` in 2 of 5 cycles per run; successful connects take 0.3-0.6 s plus ~1 s for services |
| `c1db0003` | read returns the last bytes the module notified (a mirror of the UART->BLE buffer), not module info |
| open | the daemon's receive counters were not readable during SINK (STATS replies went missing while the phone was writing continuously), so phone -> mower loss after acceptance is unverified; the daemon's own stats lines cover it |

What this means for `DESIGN_ble_transport_2026-10.md`: the heavy direction (status, maps, settings
reads) is the fast one. Budget 5 KB/s down with strict credit-based flow control that never lets the
module's buffer fill, and ~1.5 KB/s up using the largest writes possible (497 B at MTU 500) with one
write in flight. Joystick frames (tens of bytes at 10 Hz) fit with room to spare and see ~80 ms of
one-way-plus-return latency. A 60 KB map arrives in about 12 s uncompressed, 3-4 s gzipped.
Reconnection needs a scan-until-seen loop with retries and patience of up to 30 s.

## Run 1 (2026-10-30 17:00, laptop BlueZ, writes on c1db0004): raw output

Sink and source numbers in this run are invalid (client write queue bug, fixed afterwards); sweep, ping and reconnect are valid.

## BLE link measurements, 2026-10-30 17:00

Command: `ble_link_test.py --out /tmp/claude-1000/-home-michael-git-airseekers-decompile/2a91bfb2-4086-4bb9-8475-b2832d27ca9f/scratchpad/ble_run3.md`

### (e) GATT info

| item | value |
|---|---|
| device | AIRSEEKERS_xxxxxx |
| ATT MTU | 500 |
| notifications via | AcquireNotify |
| flags 00002a00 | read |
| flags 00002a01 | read |
| flags 00002a05 | indicate |
| flags 00002aa6 | read |
| flags c1db0002 | read,write,notify |
| flags c1db0003 | read |
| flags c1db0004 | write |

c1db0003 read, 51 bytes:

```
0000  aa 55 28 00 00 00 00 00 86 0b 00 00 00 01 00 00  .U(.............
0010  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
0020  00 00 00 00 00 00 00 00 00 00 00 00 00 f8 38 03  ..............8.
0030  00 4e 6b                                         .Nk
```

### (a) Write size sweep (ECHO, 10 frames per size)

| write B | accepted | echo ok | lost | CRC err | garbage B | median RTT ms |
|---|---|---|---|---|---|---|
| 20 | yes | 10/10 | 0 | 0 | 0 | 90.2 |
| 100 | yes | 10/10 | 0 | 0 | 0 | 209.9 |
| 180 | yes | 10/10 | 0 | 0 | 0 | 330.6 |
| 244 | yes | 10/10 | 0 | 0 | 0 | 436.9 |
| 497 | yes | 10/10 | 0 | 0 | 0 | 842.4 |

### (d) Round trip, PING at 10 Hz for 30 s

| sent | answered | lost | min ms | median ms | p95 ms | max ms |
|---|---|---|---|---|---|---|
| 300 | 300 | 0 | 68.3 | 79.1 | 89.7 | 124.2 |

### (b) phone->mower throughput (SINK)

| metric | value |
|---|---|
| write mode | with response |
| frame size B (payload) | 244 (233) |
| duration s | 60.0 |
| frames sent | 391215 |
| offered wire B/s | 1590937 |
| payload B/s sent | 1519215 |
| frames received by mower | 0 |
| payload B/s received (mower clock) | 540 |
| frames lost | 391215 (100.00 %) |
| mower CRC errors | - |
| mower garbage B | - |
| mower seq gaps | - |
| write errors | 390951 |
| first write error | org.bluez.Error.InProgress: org.bluez.Error.InProgress: In Progress |

### (c) mower->phone SOURCE

no answer to RESET from the daemon

### (f) Disconnect / reconnect x5 (pause 1.0 s)

| # | disconnect s | Connect() s | services resolved s | first PONG s | error |
|---|---|---|---|---|---|
| 1 | 2.61 | 25.28 | 25.42 | 25.54 |  |
| 2 | - | - | - | - | org.bluez.Error.Failed: le-connection-abort-by-local |
| 3 | 0.01 | 5.02 | 5.17 | 5.29 |  |
| 4 | 1.79 | 4.32 | 4.46 | 4.61 |  |
| 5 | 2.40 | 4.59 | 4.75 | 4.90 |  |

## Run 2 (2026-10-30 17:06, fixed client: one write in flight, notification histogram): raw output

## BLE link measurements, 2026-10-30 17:06

Command: `ble_link_test.py --out /tmp/claude-1000/-home-michael-git-airseekers-decompile/2a91bfb2-4086-4bb9-8475-b2832d27ca9f/scratchpad/ble_run4.md`

### (e) GATT info

| item | value |
|---|---|
| device | AIRSEEKERS_xxxxxx |
| ATT MTU | 500 |
| notifications via | AcquireNotify |
| conn interval (adapter limits) | n/a (needs root) |
| flags 00002a00 | read |
| flags 00002a01 | read |
| flags 00002a05 | indicate |
| flags 00002aa6 | read |
| flags c1db0002 | read,write,notify |
| flags c1db0003 | read |
| flags c1db0004 | write |

c1db0003 read, 51 bytes:

```
0000  aa 55 28 00 5f 00 00 00 86 eb fc 00 00 0e 01 00  .U(._...........
0010  00 00 00 00 00 00 00 00 00 29 f7 05 00 09 01 00  .........)......
0020  00 31 f1 00 00 5f 00 00 00 6d 12 00 00 a9 33 05  .1..._...m....3.
0030  00 e4 b9                                         ...
```

### (a) Write size sweep (ECHO, 10 frames per size)

| write B | accepted | echo ok | lost | CRC err | garbage B | median RTT ms | notifs per echo | notifs by size <=20 / 21-100 / 101-244 / 245+ |
|---|---|---|---|---|---|---|---|---|
| 20 | yes | 10/10 | 0 | 0 | 0 | 89.5 | 1.0 | 10 / 0 / 0 / 0 |
| 100 | yes | 10/10 | 0 | 0 | 0 | 224.0 | 1.0 | 0 / 10 / 0 / 0 |
| 180 | yes | 10/10 | 0 | 0 | 0 | 330.3 | 1.0 | 0 / 0 / 10 / 0 |
| 244 | yes | 10/10 | 0 | 0 | 0 | 436.1 | 1.3 | 0 / 0 / 13 / 0 |
| 497 | yes | 10/10 | 0 | 0 | 0 | 840.0 | 1.0 | 0 / 0 / 0 / 10 |

Each echo comes back as a frame the same size as the write. If the module notifies in 20-byte pieces, notifs per echo is about write size / 20.

### (d) Round trip, PING at 10 Hz for 30 s

| sent | answered | lost | min ms | median ms | p95 ms | max ms |
|---|---|---|---|---|---|---|
| 300 | 300 | 0 | 66.9 | 77.6 | 108.5 | 203.8 |

### (b) phone->mower throughput (SINK)

| metric | value |
|---|---|
| write mode | with response, 1 in flight |
| rate cap wire B/s | 8000 |
| frame size B (payload) | 244 (233) |
| duration s | 60.5 |
| writes issued | 270 |
| writes accepted | 270 |
| accepted wire B/s | 1089 |
| accepted payload B/s | 1040 |
| frames received by mower | no STATS reply |
| payload B/s received (mower clock) | 553 |
| frames lost after acceptance | - |
| mower CRC errors | - |
| mower garbage B | - |
| mower seq gaps | - |
| write errors | 0 |

### (c) mower->phone SOURCE

no answer to RESET from the daemon

### (f) Disconnect / reconnect x5 (pause 1.0 s)

| # | disconnect s | scan until seen s | Connect() s | Connect() calls | services resolved s | first PONG s | error |
|---|---|---|---|---|---|---|---|
| 1 | 2.51 | 23.86 | 0.24 | 1 | 0.39 | 0.48 |  |
| 2 | 2.59 | 1.04 | - | - | - | - | org.freedesktop.DBus.Error.NoReply: Did not receive a reply. Possible causes include: the remote application did not sen |
| 3 | 0.01 | 1.95 | 0.60 | 1 | 1.33 | 1.42 |  |
| 4 | 2.40 | 0.51 | 0.23 | 1 | 0.38 | 0.47 |  |
| 5 | 1.96 | 0.22 | - | - | - | - | org.freedesktop.DBus.Error.NoReply: Did not receive a reply. Possible causes include: the remote application did not sen |

Connect(), services and first PONG are timed from the start of the successful Connect(); scanning is stopped before it.
