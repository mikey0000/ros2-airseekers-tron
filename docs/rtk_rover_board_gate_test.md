# rtk_rover board: RTCM gate and LoRa fallback test

**Date:** 2026-10-14. Times below are the `mower_jazzy` container clock (20:3x-20:4x);
node log epochs are given where they matter.
**Robot:** Airseekers Tron at 192.168.1.105, stack in `mower_jazzy` (up 14 h), driver
node `/um960_gps_driver` (`um960_node.py`).
**Robot state:** `/mission/activity` = `docked_idle` before and after the test;
`/behavior_tree_node/high_level_status` = `CHARGING`, battery 98 %, no emergency;
`/odom` twist zero; no `/cmd_vel` traffic. No motion, cutter, mission, docking or MCU
command was sent. Only runtime parameter changes on `/um960_gps_driver` were made. Nothing
was restarted, and no file on the robot was written.

## TL;DR

| Question | Verdict |
|---|---|
| 1. Does the board gate host-written RTCM? | **The keep-alive is not needed while RTCM flows.** With the `$GNRTK,ON*69` keep-alive stopped for ~292 s and RTCM still being written, the board never reported OFF, `$GNSTA` stayed `FINE`, receiver diff_age stayed 0.6-2.6 s, and the fix stayed RTK_FIXED throughout. The board's ~6 s watchdog is fed by host RTCM, not only by the `$GNRTK,ON` literal. **Not measured:** whether RTCM reaches the UM960 while the board is actually in LoRa/OFF mode. In this test the board never went OFF while RTCM was flowing, and the driver cannot write RTCM to an OFF board without first sending `$GNRTK,ON` (see Limitations). |
| 2. Does the LoRa base path work on its own? | **The fallback works; no base station delivered corrections.** After all host writes stopped, the board reported `$GNRTK,OFF` 5.9 s later and `$GNSTA` changed to `FAIL` at the same moment. In ~6.5 min of LoRa mode no correction arrived: diff_age climbed to 120 s, then the fix dropped from RTK_FIXED to single (`GPS`). No vendor base station is reachable or paired with this rover. |

Restoration: all parameters were set back to their original values and verified identical.
The handshake, keep-alive and RTCM resumed, and the fix was RTK_FIXED again 19 s after the
restore. `/userdata/ros2/ntrip.yaml` was not created, and `/userdata/mower/ntrip.yaml` is
unchanged (mtime 2026-03-30).

## What the driver allows at runtime (from `um960_node.py`)

- Runtime (in `CORRECTION_PARAMS`, applied 0.3 s after the last SetParameters, restarting the
  NTRIP client and keep-alive thread): `ntrip_keepalive_s`, `forward_rtcm_without_board_ack`,
  `rtcm_source`, `correction_source`, `ntrip_settings_file`, `ntrip_config_file`, ...
- **Start-only:** `publish_nmea`, because the `/nmea` publisher is created in `__init__`.
  Raw `$GNSTA`/`$GNREF`/`$GNRTK` sentences were therefore not visible. Instead, their parsed
  values were read from `/fix_status` (`ref=` = `$GNSTA` FINE/FAIL, `station=` = `$GNREF`
  id), `/gps/corrections` and `/ntrip/status` (`rover_nrtk` = latest `$GNRTK`). Board state
  changes come from the node log ("rover board reports network RTK ...").
- **Persisting params:** a runtime change to `rtcm_source`, `correction_source`,
  `ntrip_enabled` or any caster field is written to `ntrip_settings_file`. On this robot that
  file (`/userdata/ros2/ntrip.yaml`) does not exist. The caster comes from parameters
  (`config_from: parameters`), so touching any of these would have **created** the file.
  Changing `rtcm_source` to `lora`/`none` would also have stored `enabled: false` there, and
  setting it back to `""`/`auto` does not undo that. These params were therefore not used.
- **Re-handshake on ON->OFF edge:** `_consume_rover_line()` sends `RTKRESET` +
  `$GNRTK,ON*69` whenever the board's report goes ON -> anything else while the source is
  NTRIP. No parameter disables this.

## Original parameter values (all restored)

`rtcm_source: ""`, `correction_source: ntrip` (→ effective mode `ntrip`),
`ntrip_enabled: false`, `ntrip_keepalive_s: 4.0`, `forward_rtcm_without_board_ack: false`,
`ntrip_fallback_grace_s: 5.0`, `ntrip_settings_file: /userdata/ros2/ntrip.yaml` (absent),
`ntrip_config_file: /userdata/mower/ntrip.yaml` (`ntrip_enable: false`),
`ntrip_netmode: ""`, `rtk_reset_on_start: false`, `publish_nmea: false`,
`corrections_stale_s: 10.0`, `lora_pairing_enabled: false`, `ntrip_validate_rtcm: true`.

## Parameters changed

| Phase | Parameter | Before | During | Restored |
|---|---|---|---|---|
| 1 gate (20:35:13, one atomic SetParameters) | `ntrip_keepalive_s` | 4.0 | 100000.0 | 4.0 |
| 1 gate | `forward_rtcm_without_board_ack` | false | true | false |
| 2 LoRa (20:40:03) | `ntrip_settings_file` | /userdata/ros2/ntrip.yaml | /proc/meminfo | /userdata/ros2/ntrip.yaml |

All three were restored in one atomic SetParameters at 20:46:05. `rtcm_source` was not
touched: with `correction_source: ntrip` the mode is already `ntrip`, so the node never hands
back to LoRa by itself.

How phase 2 stops host writes without persisting anything: `/proc/meminfo` parses as a YAML
mapping without `host`. The driver therefore sees "rtcm_source=ntrip but host/mountpoint
empty" and falls to mode `none`. That means no NTRIP client, no RTCM, no keep-alive and no
write to the port. `ntrip_settings_file` is not a persisted param, and `/proc` cannot be
written even if a persist had been triggered. The file's mtime is stable, so the
settings-file watcher did not loop. This was checked in the container before use.

## Phase 0: baseline (20:32:32-20:34:34, 123 samples at 1 Hz)

- Fix: RTK_FIXED in 123/123 samples; diff_age 1.4-2.4 s; `station=0000`; `ref=FINE`.
- `/gps/corrections`: source `ntrip`, `rover_nrtk=ON`, `rtcm_gate=open`, `flow=active`,
  ~1.37 kB/s RTCM written. `serial_bytes_written - bytes_written` grew by 420 B in 121 s:
  30 keep-alives × 14 B (`$GNRTK,ON*69\r\n`), one every 4 s.
- `rtcm_held_bytes` = 4134. This was held at node start (13:28), before the board first
  reported ON.
- Receiver RTK timeout: no status reports it. As phase 2 shows, the effective hold is
  **120 s**, not the vendor-configured `CONFIG RTK TIMEOUT 20`.
- Board report history in old logs: no ON->OFF edge has ever been logged with the 4 s
  keep-alive running. After every port open, the board reports OFF a few seconds after
  `$GNRTK,ON*69` and then ON about 1.4 s later.

## Phase 1: gate test (keep-alive stopped, RTCM forced), 20:35:16 → 20:40:08

Timeline:

```
20:35:13  SetParameters ntrip_keepalive_s=100000, forward_rtcm_without_board_ack=true
1791923716.023  (20:35:16) node: "network RTK on (correction source changed): sent RTKRESET + $GNRTK,ON*69"
1791923716.063  node: "rover board reports network RTK ON"     <- reply 40 ms after the ON
1791923716.501  NTRIP reconnected, streaming
20:35:16-20:40:07  no further $GNRTK,ON*69 written (serial_bytes_written - rtcm bytes
                   constant at 71046249 for the whole phase), RTCM written continuously
```

Samples, excerpt:

```
20:35:15 q=RTK_FIXED diff_age=0.5s ref=FINE | nrtk=ON gate=open   ... ser_wr=71046211
20:35:16 q=RTK_FIXED diff_age=0.6s ref=FINE | nrtk=ON gate=forced flow=waiting ser_wr=71046249  <- RTKRESET+ON written
20:35:19 q=RTK_FIXED diff_age=0.6s ref=FINE | nrtk=ON gate=forced flow=active rx=2937   ser_wr=71049186
20:37:16 q=RTK_FIXED diff_age=0.6s ref=FINE | nrtk=ON gate=forced flow=active ...
20:39:14 q=RTK_FIXED diff_age=0.6s ref=FINE | nrtk=ON gate=forced flow=active rx=322468 ser_wr=71368717
20:40:07 q=RTK_FIXED diff_age=0.7s ref=FINE | nrtk=ON gate=forced flow=active rx=395806 ser_wr=71442055
```

- About 292 s without the keep-alive (≈48× the believed 6 s board timeout), 395,806 B of
  RTCM written.
- Fix RTK_FIXED in every 1 Hz sample (260/260 in the phase-1 log plus 18 in phase 2 before
  the switch). diff_age stayed 0.5-2.6 s (2.6 s once, during an NTRIP chunk gap). `ref=FINE`
  throughout.
- The board **never reported OFF**. No log line "rover board reports network RTK OFF", and
  `/ntrip/status rover_nrtk` stayed ON. The driver's ON->OFF re-handshake therefore never
  fired and no further RTKRESET was sent. Phase 2 shows that the board does report OFF on its
  own within 6 s, so the absence of a report here means the board stayed in network mode.
- The RTKRESET sent at 20:35:16, while the board was already ON and RTCM resumed within
  0.5 s, caused **no visible degradation**. No sample left RTK_FIXED, there was no STALE,
  and diff_age did not jump.

Interpretation: diff_age is the receiver's age of the RTCM it is using. It stayed below 3 s
and started climbing 1 s/s the moment host RTCM stopped (phase 2). The UM960 was therefore
using the host RTCM the whole time, and the board passed it while hearing no `$GNRTK,ON*69`.
The board's ~6 s fallback timer is reset by host RTCM, not only by the keep-alive.

## Phase 2: LoRa test (no host writes at all), 20:40:08 → 20:46:08

```
1791924008.083  (20:40:08) node: rtcm_source=ntrip but host/mountpoint empty (... /proc/meminfo ...) -> "corrections: none"
1791924013.978  (20:40:14) node: "rover board reports network RTK OFF"     <- 5.9 s after the last host write
```

| Time | Fix | diff_age | `$GNSTA` (ref) | `$GNREF` (station) | board `$GNRTK` |
|---|---|---|---|---|---|
| 20:40:07 | RTK_FIXED | 0.7 s | FINE | 0000 | ON |
| 20:40:08 | RTK_FIXED | 1.7 s (climbing 1 s/s) | FINE | 0000 | ON |
| 20:40:14 | RTK_FIXED | 6.4 s | **FAIL** | 0000 | **OFF** (5.9 s) |
| 20:42:07 | RTK_FIXED | 119.7 s | FAIL | 0000 | OFF |
| 20:42:08 | **GPS** (single), cov_xx 0.15 m² | 120.0 s (saturates) | FAIL | 0000 | OFF |
| 20:42:08-20:46:07 | GPS, 34 sats, hdop 0.44 | 120.0 s | FAIL | 0000 | OFF |

- The board falls back to LoRa on its own **5.9 s** after the host stops writing. This
  matches the vendor-derived ~6 s, but the trigger is "no host bytes", not "no keep-alive".
  `$GNSTA` reference status flips FINE→FAIL in the same second. It looks like the board's
  own correction-input status.
- `$GNREF` station id stayed `0000`. That is the same id seen under NTRIP, so it is
  stale/unspecific, not a LoRa base.
- **No corrections from a LoRa base during ~6 min in LoRa mode (20:40:14-20:46:08):**
  diff_age never reset, `$GNSTA` never returned to FINE, and no new station id appeared.
  No vendor base station is reachable/paired (`lora_pairing_enabled: false`; set_lora has
  never written a pairing). Whether the LoRa path would work with a paired, powered base
  could not be tested.
- Receiver RTK hold: RTK_FIXED was kept on stale corrections until diff_age = 120 s, then it
  went to single. The effective RTK timeout is **120 s**, not 20 s. Either the vendor
  `CONFIG RTK TIMEOUT 20` is not in effect on this receiver or it means something else.
  This is why the classic "fix degrades around the 20 s RTK timeout" check would not have
  worked here, and diff_age is the better signal.

## Phase 3: restore, 20:46:05 → RTK_FIXED at 20:46:27

```
20:46:05  SetParameters ntrip_settings_file=/userdata/ros2/ntrip.yaml, ntrip_keepalive_s=4.0, forward=false
1791924368.079 (20:46:08.08) node: "network RTK on (correction source changed): sent RTKRESET + $GNRTK,ON*69"
1791924368.488 NTRIP streaming
1791924369.184 node: "holding RTCM (4414 bytes so far): rover board reports network RTK unknown, not ON"
1791924370.075 node: "no fresh UM960 fix for 2.0s, stopping /fix"          <- receiver output paused
1791924374.395 (+6.3 s) node: "rover board reports network RTK OFF"
1791924375.791 (+7.7 s) node: "rover board reports network RTK ON"        <- gate opens
20:46:14-20:46:21  fix INVALID (GGA quality 0), diff_age 120
20:46:22  RTK_FLOAT diff_age 0.6 s
20:46:27  RTK_FIXED                                                         <- 19 s after the restore
20:46:27-20:49:14  RTK_FIXED, ref=FINE, nrtk=ON, gate=open, flow=active, keep-alive 14 B / 4 s
```

- Re-acquisition from LoRa/single to RTK_FIXED took **19 s**: 7.7 s for the board to switch
  to network mode, then 11.6 s to float→fixed.
- Switching the board from LoRa to network is not instant. The board did not answer the ON
  at once, as it did when already ON (40 ms). It reported OFF at +6.3 s and ON at +7.7 s.
  Meanwhile the UM960 stopped emitting a solution for about 10 s. Two causes are possible:
  the RTKRESET (this time it reached a receiver with no RTCM flowing) or the board's mode
  switch. They cannot be separated with runtime parameters. 9,689 B of RTCM were held by the
  driver gate during the switch (held counter 4134 → 13823).
- Final state verified: all 14 recorded parameters identical to the originals (`diff`
  clean). `/gps/corrections` shows `rover_nrtk=ON`, `rtcm_gate=open`, `flow=active`,
  `ntrip_state=streaming`. The keep-alive is back at 14 B every 4 s. Fix RTK_FIXED,
  diff_age 1.6 s. Mission still `docked_idle`.

## Limitations

1. **"RTCM into an OFF board" was not isolated.** The intended outcome-A/B test needs the
   board in LoRa mode while the host writes RTCM. With runtime parameters this cannot be
   reached:
   - while RTCM flows, the board never drops to OFF (phase 1);
   - every path that starts host RTCM (`_start_corrections` in ntrip mode) first sends
     `RTKRESET` + `$GNRTK,ON*69`, and the ON->OFF edge handler in `_consume_rover_line`
     re-sends them unconditionally.

   A node restart does not help either: it needs a **code change** (e.g. a
   `ntrip_board_handshake: false` parameter that suppresses the start/edge `$GNRTK,ON*69`
   and the keep-alive), then a restart, then writing RTCM to a board that is in LoRa mode
   (e.g. right after a phase-2 style pause). The owner has to do that. With such a
   parameter set to `false` plus `forward_rtcm_without_board_ack: true`, the expected
   observations are: does diff_age reset / `$GNSTA` go FINE / does the board ever report ON
   by itself.
2. `publish_nmea` is start-only, so raw `$GNSTA`/`$GNREF`/`$GNRTK` sentences were not
   captured. Only the driver's parsed fields and its logged board-state changes were.
3. No LoRa base station appears to be reachable or paired, so the LoRa path itself (base →
   board → UM960) is untested.
4. 1 Hz sampling: sub-second float episodes would not show. None of the RTK_FIXED runs
   shows a diff_age jump or covariance change that would suggest one.

## What this means for Universal GNSS

- **Pre-RTCM `$GNRTK,ON*69` + 4 s keep-alive hook:** a periodic keep-alive is **not
  needed while RTCM is streaming**. The board stayed in network mode for ~5 min on RTCM
  alone. Both of these are still open:
  - is an initial `$GNRTK,ON*69` needed to take the board out of LoRa mode (its state at
    power-up, and after any >6 s RTCM gap such as a caster outage or reconnect)?
  - does the board pass RTCM while in LoRa mode?

  Until limitation 1 is tested, the safe upstream design is a small hook:
  - write `$GNRTK,ON*69\r\n` once before the first RTCM after connect, and again whenever
    RTCM resumes after a gap of more than ~5 s;
  - cheapest robust variant: write it every ~4 s while the NTRIP client is connected. It
    costs 14 B and matches the vendor.

  If the follow-up test shows RTCM alone flips the board to network mode, the hook can be
  dropped and the switch is nearly free.
- **RTKRESET on ON->OFF edges:** not observed to matter. No ON->OFF edge happened while RTCM
  flowed, so the vendor reset-on-edge path never triggered. An RTKRESET sent while the
  board was ON and RTCM was flowing had no visible effect. The receiver also re-converged
  without trouble after a 6 min correction gap (float at +14 s, fixed at +19 s). Universal
  GNSS can omit the RTKRESET. The only edge that matters in practice is "RTCM resumes after
  >6 s", and that is handled by the ON-before-RTCM above.
- **LoRa fallback:** nothing beyond stopping all host writes. The board reported OFF 5.9 s
  after the last host byte (RTCM or keep-alive) and needs no OFF command. Stopping only the
  keep-alive while RTCM continues does **not** hand back to LoRa. A Universal GNSS
  configuration that wants the LoRa base must stop writing RTCM to the port (disable the
  NTRIP input), not merely skip the handshake. Note also that the receiver holds RTK_FIXED
  on stale corrections for 120 s, so a consumer should gate on diff_age, not only on the
  fix type.
