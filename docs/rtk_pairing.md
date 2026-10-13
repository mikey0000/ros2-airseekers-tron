# Pairing the LoRa RTK base station without the Airseekers cloud

The Tron receives RTK corrections either from NTRIP (host-injected, see `um960.md`) or
over LoRa from the vendor base station. The LoRa link is handled entirely by the
`rtk_rover` board behind `/dev/serial_rtk`: the host never talks to the radio, it only
tells the board which base to listen to. That "pairing" is a one-off write the board
stores in flash, so it survives reboots and stack changes, and it is the same for our
`um960_gps_driver` and for a Universal GNSS sidecar (neither needs to know about it).

`scripts/rtk_pair.py` reproduces the vendor write byte for byte (Python stdlib only; runs
on the mower host as root or inside `mower_jazzy`). Nothing in it commands motion.

## What the vendor app does (recovered)

| Step | Where | What |
|---|---|---|
| 1 | app | scans the base's QR code (its serial number, `rtk_sn`) |
| 2 | app → cloud | `POST /api/web/device/rtk/address-info` with `sn`, `latitude`, `longitude`; the answer carries `address`, `channel`, `area` |
| 3 | app → mower BLE | command `0x87` / protobuf `mower.rtk.BindRTK{sn, addr, channel, area}` (`mower_docs/04-ble-protocol.md`, `reference/proto/rtk.proto`) |
| 4 | `base_ble_node` | `/mower_gps_node/ntrip false`, then `/mower_gps_node/set_lora` |
| 5 | `mower_gps_driver` | writes `$GLIST,<band>` then, 0.5 s later, `$GNCON,<addr hex4>,<channel hex2>` to `/dev/serial_rtk` |
| 6 | `base_ble_node` | waits for `/mower_gps_node/info` to show differential data from a non-zero station and checks the LoRa SNR up to 5 times; replies SUCCESS / WAIT_COMMUNICATION / COMMUNICATION_ERR / WEAK_COMMUNICATION |

The base station itself is never configured by the mower. Its firmware
(`mcu_decompile/rtk_station`) derives its own LoRa identity from its serial number and
runs a 120 s survey-in (`PQTMCFGSVIN,W,1,120,...`) after power-up before it transmits
RTCM. The cloud is therefore only a lookup table from base serial to the three numbers
the board needs. Everything else is local.

The app also has a cloud-less fallback ("universal code", `confirmCustomBind`): it binds
to **addr 38183 (0x9527), channel 40, area 02** with sn `3001024503000000`. That is the
pairing a base answers on out of the box.

Band from the area / serial (vendor `setBand`): `00`, `02` → `$GLIST,0` (Europe 863 MHz),
`01` → `$GLIST,1` (Americas 902 MHz), `03` → `$GLIST,2` (Australia / NZ 927 MHz),
`F` → `$GLIST,F` (all). With no area the region is `sn[4:6]` of the **base** serial, so
the label on the base already tells you the band.

## Finding the address and channel without the cloud

In order of preference:

1. **It is probably still stored.** If the base was paired once in the vendor app, the
   board remembers it. `rtk_pair.py query` prints it (the vendor driver asks the same
   `$GNCON,0000,FF*4B` on every connect).
2. **The app's RTK info page** shows the bound address / channel of a base the account
   already paired (that page reads the cloud, but you only need to copy the numbers once).
3. **The universal code** (`rtk_pair.py universal`): factory default of a new base.
4. The base serial gives the band only, not the address. There is no mower-side way to
   discover a base's address: the board does not report what it hears from other bases.

## Procedure

Stop the stack first so nothing else reads the port or sends NTRIP / `$GNRTK,ON` while
the board is being configured (the board ignores host corrections in LoRa mode anyway,
but a second reader splits the replies):

```sh
ssh root@192.168.1.105
docker stop mower_jazzy
cd /userdata/ros2_stack

# 1. what is stored now?
python3 scripts/rtk_pair.py query
#   stored pairing: addr=38183 (0x9527) channel=40 (0x28)

# 2. (only if it is wrong / a new base) write the pairing. Base within ~2 m, sky view.
python3 scripts/rtk_pair.py pair --sn 3001034903001234 --addr 38183 --channel 40
#   or, new base straight out of the box:
python3 scripts/rtk_pair.py universal
#   --area 03 forces the Australia/NZ band if the base SN does not encode the region.
#   The script echoes the board's reply, re-reads the stored pairing and then watches
#   for 180 s (base survey-in ~120 s) until $GNSTA FINE / station id != 0000.

# 3. just watch corrections (no writes), e.g. after powering the base
python3 scripts/rtk_pair.py watch --seconds 240

docker start mower_jazzy
```

Expected watch output once the base transmits: `$GNSTA,FINE`, GGA `fix=RTK float` then
`RTK fixed`, `diff_age` ≈ 1 s, `station` non-zero. `$GNSTA,FAIL` with `station=0000`
after the survey-in means the base is on another address / channel / band, or out of
range (the vendor app insists on ≤ 2 m for the binding itself).

From inside the container instead of the host: `docker exec -it mower_jazzy python3
/work/scripts/rtk_pair.py query` works as well, but stop our GPS node first (or run
the whole stack stopped as above).

## Interaction with NTRIP and Universal GNSS

* The pairing is independent of the NTRIP path. Network RTK (`$GNRTK,ON` + host RTCM)
  temporarily overrides LoRa; when the host stops writing, the board returns to the
  stored LoRa pairing within ~6 s (`rtk_rover_board_gate_test.md`).
* With the Universal GNSS sidecar (`gnss_stack: universal`) there is no `set_lora`
  service; pair with this script. With `um960_gps_driver` the BLE `0x87` path and the
  `/mower_gps_node/set_lora` service still exist but are a logged stub unless
  `lora_pairing_enabled: true` (`um960.md`); the script is the verified path.
* The `rtk_station` firmware accepts the same `$GNCON` / `$GLIST` / `$BRESET` sentences on
  its own UART, so a base could in principle be re-addressed too. Not needed and not
  tested; the mower only ever has to match the base.
