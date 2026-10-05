# um960_gps_driver

ROS 2 driver for the Unicore UM960 RTK receiver on `/dev/serial_rtk`. Full documentation:
[`docs/um960.md`](../../docs/um960.md).

## Choosing the correction source

`correction_source` selects where RTCM corrections come from:

* `lora` — vendor RTK base station over LoRa; the rover board feeds the UM960 and the
  host only reads. `/mower_gps_node/set_lora` is a logged stub unless
  `lora_pairing_enabled: true` (then it writes the vendor `$GLIST`/`$GNCON` commands).
* `ntrip` — host NTRIP v1/v2 client: RTCM from your caster is written into the
  receiver port, the GGA is uploaded every `ntrip_gga_interval_s`. The rtk_rover board
  is switched with `RTKRESET` + `$GNRTK,ON*69` (again on an ON→OFF report) and kept
  there with `$GNRTK,ON*69` every `ntrip_keepalive_s` (4 s); RTCM is only forwarded
  while the board reports `$GNRTK,ON` (`forward_rtcm_without_board_ack: true` for
  bench tests). `ntrip_netmode`: `4g` → usb0, `wifi` → wlan0, `auto`/`""` unbound.
  There is no OFF command: switching back to `lora` simply stops writing and the
  board returns to LoRa after ~6 s.
* `none` — no corrections.
* `""` (default) — `ntrip` when the vendor file `ntrip_config_file`
  (default `/userdata/mower/ntrip.yaml`, keys `ntrip_enable`, `ntrip_ip`, `ntrip_port`,
  `ntrip_user`, `ntrip_passwd`, `ntrip_mountpoint`) has `ntrip_enable: true`, else `lora`.
  The `ntrip_*` fields of that file fill in whatever the `ntrip_*` parameters leave empty
  (when `ntrip_host` is empty). In a container `/userdata/mower` must be mounted.

Credentials: put them in `config/um960_secrets.yaml` (git-ignored, start from
`config/um960_secrets.example.yaml`) or in the vendor file, never in `um960.yaml`.
Pass the secrets file as a second `--params-file`.

Verify: `ros2 topic echo /fix_status` shows `corr_src=ntrip corr=streaming
corr_flow=active corr_age=…` and, once the receiver uses the corrections,
`solution=NARROW_INT` (RTK fixed). `ros2 topic echo /gps/corrections` has the details.
Bench test: `ros2 run um960_gps_driver fake_ntrip_caster` (see the docs).
