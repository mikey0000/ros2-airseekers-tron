# Pairing the RTK base station (LoRa), no vendor app needed

The Tron can take RTK corrections from an internet caster (NTRIP, see `um960.md`) or over
LoRa radio from the Airseekers base station that came with the mower. For the LoRa path the
mower has to be "paired" with the base: the mower's RTK board is told three numbers (the
base's LoRa **address**, **channel** and the radio **band** for your region) and stores them
permanently. The base is never configured by the mower.

With the stock firmware this is done by the vendor app through the Airseekers cloud. This
stack does it locally, either from the web app (below) or with `scripts/rtk_pair.py`
(already on the mower under `/userdata/ros2_stack/scripts/`). Both only talk to the RTK
board; they cannot move the mower or start the blade.

## What you need

- The base station, powered on, outdoors with a clear view of the sky, within a few metres
  of the mower for the first test. After power-up the base needs about **2 minutes** to
  work out its own position (survey-in) before it starts transmitting corrections.
- The base's serial number, printed on its label under the QR code (16 digits starting
  with `3001`).
- For the script only: a terminal on the mower, `ssh airseekers@<mower-ip>` (same login as in `deployment.md`).
- Your region, for the radio band:

  | where you are | `--area` | band |
  |---|---|---|
  | Europe, UK and everywhere not listed below | `02` | 863 MHz |
  | North America | `01` | 902 MHz |
  | Australia, New Zealand | `03` | 927 MHz |

  **The band must match your location, not where the mower was paired.** The base picks
  its band from its own GNSS position every time it gets a fix; the mower's board only
  changes band when you tell it. A mower paired at the factory in China is on the Europe
  band and will never hear a base running in New Zealand or the USA until it is re-paired
  with the right `--area`.

## Which case are you in?

**A. The base came in the box with the mower.** It was paired at the factory, so the
address and channel are already stored in the mower. You only need to check them and set
the band for your region. Go to steps 1, 3 and 4 below, using the numbers that step 1 prints.

**B. A new, second-hand or replacement base.** You need its address and channel. In order
of preference:

1. If it was ever paired with any mower in the vendor app, that app's RTK info page shows
   the address and channel. Copy them once; nothing else is needed from the cloud.
2. Try the vendor "universal code" (the app's own no-cloud fallback): address 38183,
   channel 40 (`rtk_pair.py universal`, or the `pair` command with your `--area`). This is
   what the app uses when it cannot reach the cloud; it has not been verified on a base in
   this project.
3. Read the base itself over its serial port (needs a USB-UART adapter and opening the
   case; see "Reading the base directly" at the end).

**C. It used to work and stopped.** Go to step 3 (watch). If nothing arrives after
3 minutes, power-cycle the base, wait 2 minutes and watch again, then re-pair (step 2).

## From the web app

The quickest way, with the stack running. Open `http://<mower-ip>:4006`, then
**Settings → GPS & Positioning → RTK base station**.

1. The card shows the **stored pairing** (address in decimal and hex, channel, band) and
   the live status: **Base corrections** "receiving the base" (`$GNSTA,FINE`) or "no base
   signal" (`FAIL`), station id, correction age and fix type. If the stored pairing says
   "not read yet", press **Read from board**.
2. If a yellow box says **NTRIP currently owns the receiver**, go to Settings → NTRIP
   Corrections, switch **Enable NTRIP** off and save. The base is ignored while NTRIP
   feeds the receiver, and the mower refuses to pair then.
3. Fill in **Base address** (decimal or `0x` hex), **Channel** (1-75) and **Region**
   (Europe / UK 863 MHz, North America 902 MHz, Australia / New Zealand 927 MHz). For a
   base that came in the box the form is pre-filled with the stored values: only check the
   region (case A below). For a base with unknown pairing press **Use universal code**
   (address 38183, channel 40; case B).
4. Press **Pair** and confirm. The mower writes the band and the pairing, reads them back
   and reports "Pairing stored" only when the board holds the new values.
5. Power the base within a few metres of the mower with a clear view of the sky and wait
   about 2 minutes for its survey-in. **Base corrections** turns to "receiving the base"
   and the fix type to `RTK_FLOAT` and then `RTK_FIXED`.

**Reboot base** sends `$BRESET` through the mower's RTK board, which relays a reset to the
base (untested on hardware; power-cycling the base does the same). If the web app cannot
reach the board (the card says the service is not available), use the script below.

## Step by step (script over ssh)

Stop the stack first so nothing else uses the RTK port while you work:

```sh
ssh airseekers@<mower-ip>
docker stop mower_jazzy
cd /userdata/ros2_stack
```

**1. See what is stored now**

```sh
python3 scripts/rtk_pair.py query
#   << $GNCON,9527,28*48        <- address, channel (hex)
#   << $GLIST,0*59              <- band currently set (0 = Europe, 1 = N. America, 2 = AU/NZ)
#   stored pairing: addr=38183 (0x9527) channel=40 (0x28)
```

If the band line does not match your region, or the address/channel are not your base's,
go to step 2. If everything matches, go to step 3.

**2. Write the pairing** (base powered, close by, sky view)

```sh
# your own base: address and channel from step 1 or from the app's RTK page
python3 scripts/rtk_pair.py pair --sn <base serial> --addr <addr> --channel <channel> --area 03

# a base with unknown pairing: try the universal code, with your region's band
python3 scripts/rtk_pair.py pair --sn 3001024503000000 --addr 38183 --channel 40 --area 03
```

`--addr` takes decimal or `0x` hex, `--channel` is decimal (1-75). The script writes the
band, then the address/channel, reads the stored values back, and then watches for up to
3 minutes for corrections to arrive. To change only the band, run `pair` with the stored
address and channel and the new `--area`.

**3. Watch for corrections** (no writes; use this any time)

```sh
python3 scripts/rtk_pair.py watch --seconds 240
```

Good output, once the base has finished its survey-in:

```
  << $GNSTA,FINE,..           <- the board is receiving the base
  fix  fix=RTK float ...      <- then
  fix  fix=RTK fixed sats=37 diff_age=1.0s station=...
result: LoRa corrections LIVE ($GNSTA FINE)
```

Bad output: `$GNSTA,FAIL`, `$GNRTK,OFF` repeating, `fix=single`, `diff_age=-`. See
"If it does not work" below.

**4. Start the stack and tell it to use the base**

```sh
docker start mower_jazzy
```

In the web app (`http://<mower-ip>:4006`) open **Settings → GPS / Positioning** and switch
**Enable NTRIP** off. On this robot NTRIP off means "use the LoRa base"
(`launch/robot_settings.py` maps it to the GPS driver's `correction_source`). With NTRIP on, the
stack feeds the board from the internet caster and the LoRa base is ignored while that
works; the board falls back to the base by itself within a few seconds if the caster stops.

## If it does not work

| symptom | likely cause | what to do |
|---|---|---|
| `$GNSTA,FAIL` for more than 3 min after powering the base | wrong band for your region | re-pair with the right `--area` (step 2) |
| same, band is right | wrong address/channel, base out of range, or base still surveying in | check the base's status light, move within a few metres, wait 2 min, re-pair |
| `query` prints `no $GNCON answer` | something else has the port | `docker stop mower_jazzy`, check `docker ps`, retry |
| `cannot open /dev/serial_rtk` | permissions | prefix with `sudo` |
| `fix=RTK fixed` but `diff_age` keeps growing | corrections stopped (base off, or stale NTRIP from before the stack was stopped) | power-cycle the base and watch again |
| works near the base, drops in the far corners of the lawn | LoRa range / antenna placement | mount the base higher with line of sight to the lawn |

The base never needs to be reconfigured: it transmits on its own address and channel from
the factory and changes only its band, automatically, from its own position.

## Worked example: a base that came in the box

What the development robot looked like (identifiers replaced; the pattern is what matters):

| item | value |
|---|---|
| base serial (label) | `3001XXXXXXXXXXXX`, 16 digits |
| address / channel stored by the factory | non-default (not the universal code), so the factory did pair this mower with *a* base |
| band stored by the factory | `$GLIST,0` (Europe): the app bound it at the factory in China |
| board default before the factory write | `$GNCON,9527,18` (address 0x9527, channel 24) |

The owner never paired it with the vendor app. In New Zealand the base switches itself to
the 927 MHz band, so the robot is deaf until re-paired with the stored address and channel
and `--area 03`:

```sh
python3 scripts/rtk_pair.py pair --sn <label serial> --addr <stored addr> --channel <stored channel> --area 03
```

**Caveat:** the serial the factory bound (found in the vendor log, `/userdata/log/base.log*`,
"set lora sn:") did not match the code on the box, which had 17 digits. Either the box
carries a different code or the shipped base is not the unit the factory paired. Check the
sticker on the base itself. If the command above gives no corrections after the survey-in,
treat it as case B (universal code, or read the base directly). Not tested yet: the robot
runs on NTRIP for now.

---

## Reference: what the vendor app does (recovered)

| Step | Where | What |
|---|---|---|
| 1 | app | scans the base's QR code (its serial number, `rtk_sn`) |
| 2 | app → cloud | `POST /api/web/device/rtk/address-info` with `sn`, `latitude`, `longitude`; the answer carries `address`, `channel`, `area` |
| 3 | app → mower BLE | command `0x87` / protobuf `mower.rtk.BindRTK{sn, addr, channel, area}` (`mower_docs/04-ble-protocol.md`, `reference/proto/rtk.proto`) |
| 4 | `base_ble_node` | `/mower_gps_node/ntrip false`, then `/mower_gps_node/set_lora` |
| 5 | `mower_gps_driver` | writes `$GLIST,<band>` then, 0.5 s later, `$GNCON,<addr hex4>,<channel hex2>` to `/dev/serial_rtk` |
| 6 | `base_ble_node` | waits for `/mower_gps_node/info` to show differential data and checks the LoRa SNR up to 5 times; replies SUCCESS / WAIT_COMMUNICATION / COMMUNICATION_ERR / WEAK_COMMUNICATION |

`scripts/rtk_pair.py` reproduces step 5 byte for byte (Python stdlib only; runs on the mower
host or inside `mower_jazzy`). The cloud is only a lookup table from base serial to the three
numbers; everything else is local.

Area → band (vendor `setBand`): `00`, `02` → `$GLIST,0` (Europe 863 MHz), `01` → `$GLIST,1`
(Americas 902 MHz), `03` → `$GLIST,2` (Australia / NZ 927 MHz), `F` → `$GLIST,F` (all). With
no `--area` the script uses `sn[4:6]` of the base serial, which is the region the base was
*sold* for, not necessarily where it is; prefer an explicit `--area`.

The app's cloud-less fallback ("universal code", `confirmCustomBind`) binds address 38183
(0x9527), channel 40, area 02 with serial `3001024503000000`. The rover board's own
out-of-box read-back on the development robot was address 0x9527, channel 24, so 0x9527 is
plausibly the factory default of the radio modules; the universal code is untested here.

## Reference: base station firmware (`mcu_decompile/rtk_station`)

Full analysis in `analysis/2026-10-30_rtk_station_identity.md`. The short version:

- Address and channel are **not** derived from the serial number. They live in the base's
  external LoRa radio module and are written only by a `$GNCON` on the base's own wired
  port (at the factory). There is no formula to compute them from the label.
- Band: chosen by the base from its own GNSS fix (North America → 1, Australia and the New
  Zealand box lat -47..-34 / lon 165..179 → 2, elsewhere → 0), rewritten on every fix, so
  forcing the base to another band with `$GLIST` does not hold. The rover board has no
  such logic: its band changes only on a `$GLIST` from the host.
- After power-up the base runs a 120 s survey-in, then transmits RTCM 1006, 1033 and MSM
  for GPS, GLONASS, Galileo and BeiDou about once a second.

## Reference: NTRIP and Universal GNSS

- The pairing is independent of the NTRIP path. Network RTK (`$GNRTK,ON` + host RTCM)
  temporarily overrides LoRa; when the host stops writing, the board returns to the stored
  LoRa pairing within ~6 s (`rtk_rover_board_gate_test.md`).
- With the Universal GNSS sidecar (`gnss_stack: universal`) there is no `set_lora` service;
  pair with this script. With `um960_gps_driver` the web app card, the BLE `0x87` path
  and the `/mower_gps_node/set_lora` service write the same bytes as the script and read
  the pairing back (`lora_pairing_enabled: true` is the default; `um960.md`).
- From inside the container instead of the host: `docker exec -it mower_jazzy python3
  /work/scripts/rtk_pair.py query` works as well, with the GPS node stopped.

## Reading the base directly (advanced)

The base's command port is USART1 on PB6/PB7, 115200 8N1, 3.3 V (PB8 looks like an RS-485
direction pin, so the board may carry an RS-485 transceiver; check before wiring). There is
no USB. At boot it prints `$GNVER`, `$GNSNR,<id>,<serial>`, `$GNCON,<addr>,<chan>` and
`$GLIST,<band>`; on request it answers `$GNCON,0000,FF*4B` (read back address/channel),
`$GNSNR*46` (serial) and `$GNVER*48`. It also accepts `$GNCON,<addr>,<chan>`, `$GLIST,<n>`
and `$BRESET`, so a base can be re-addressed, but that is untested and never needed: the
mower only has to match the base.
