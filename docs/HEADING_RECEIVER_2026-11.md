# Dual-antenna heading receiver: spec (2026-11-03)

Goal (REVIEW_2026-11-02 step 4): an absolute heading at every START, with no COG calibration
drive, no blind drive and no dock seed. The magnetometer experiment (`heading_mag_seed`,
`scripts/mag_cal.py`) runs first; this spec is the hardware path if its verdict is not `seed`.

## Candidates (UM982: dual-antenna, L1/L2/L5, internal heading engine)

| Option | What you get | Rough price | Notes |
|---|---|---|---|
| ArduSimple simpleRTK3B Compass | UM982 board, USB-C (CDC) + UART on headers, 2x SMA | ~EUR 300 | Recommended: documented, USB powered, UART pins free, heading and position from one board |
| Holybro H-RTK Unicore UM982 | cased UM982 + 2 helical antennas, JST-GH UART/CAN (DroneCAN) | ~USD 400 with antennas | Built for Pixhawk; UART at 3.3 V, needs a USB-UART adapter or a free host UART |
| Bare UM982 module | castellated module | ~USD 120 | Needs our own carrier (LDO, LNA bias, RF layout): not worth it for one robot |

## What the repo already supports
- `um960_gps_driver/parsers.py` parses `$GPHPR` / `$GNHPR` (both field layouts), keeps the
  quality flag (`HPR_STATUS_FIXED = 4`) and an age stamp; `um960_node` publishes `/heading`
  and gates `/heading_valid` on `publish_rtk_heading` (default false), freshness
  (`heading_timeout_s`) and QF 4. There is no `UNIHEADING` parser (baseline length and
  heading std dev are not read): add it before relying on the std dev in the EKF covariance.
- `correction_source: none` lets a second node instance run a heading-only receiver.
- `heading_logic` source `rtk`: seed from `rtk_seed_samples` consistent samples, blend
  (`rtk_alpha`), outlier gate, `rtk_dual_antenna: true` allows samples at rest,
  `rtk_baseline_yaw_deg` (antenna baseline vs forward axis), UTM grid convergence from `/fix`.
  This is the fix for REVIEW_2026-10-13 B8 (sticky, unvalidated "dual-antenna seeding"):
  status checked, age-stamped, baseline yaw, blended not replaced, off by default.

## Antennas and baseline
- Two identical L1/L2/L5 antennas, same model and same cable length and type (the receiver
  differences the carrier phase; unequal cables add a fixed delay that the RTK solution has to
  absorb). Helical (no ground plane needed, smaller, ~USD 40-80 each) is the better fit for a
  plastic shell; a patch needs a >= 10 cm metal ground plane under each.
- The Tron body is about 0.7 m long. A fore-aft baseline of 0.5-0.6 m fits on the top cover.
- Accuracy: the UM982 heading spec is about 0.1-0.15 deg (1 sigma) at 1 m. Carrier-phase
  noise scales with 1 / baseline, so expect 0.2-0.3 deg at 0.5 m. Caveat: at short baselines
  multipath off the body and the lawn, phase-centre variation and a low sky view under trees
  dominate. Budget 0.5 deg and confirm with the bench test below, not the datasheet.

## Mounting
- Both antennas at the same height, baseline along the forward axis (then
  `rtk_baseline_yaw_deg` = 0 or 180), measured to 1 deg; the pitch output then reads the slope.
- Away from the blade motor (centre of the deck) and its cable, from the stereo module
  (front: its LIS2MDL and the module's own RF), and from the existing UM960 antenna (>= 15 cm).
- Clear sky above the top cover; no metal or carbon over the elements; strain-relieved cables.

## Power and serial to the RK3588
Today the UM960 sits behind the vendor `rtk_rover` board on `/dev/serial_rtk` (ttyS4): LoRa
corrections from the base station, host RTCM only after `$GNRTK,ON*69`
(`docs/rtk_rover_board_gate_test.md`). Free host serial: none documented;
ttyS1/3/4/9 and ttyUSB0-2 (LTE) are taken.
- **Option A (recommended first): keep the UM960 for position, add the UM982 heading-only.**
  simpleRTK3B Compass on USB (5 V and data on one cable, shows up as ttyACM), a second
  `um960_node` with `correction_source: none`, `publish_rtk_heading: true`, its own topics.
  Moving-base heading needs no RTCM. Nothing on the LoRa path changes; fully reversible.
- **Option B: UM982 for position and heading.** Either it replaces the UM960 behind the
  `rtk_rover` board (unknown: whether the board forwards to anything but the UM960 at its
  configured baud and log set), or it is fed RTCM by the host NTRIP client, which needs a
  caster for the site (the LoRa base does not reach the host) and a working LTE / Wi-Fi link.
- **Open wiring question:** is there a free USB host port (or a UART pad pair) on the Tron's
  RK3588 board reachable from under the top cover, and can it supply ~0.5 W at 5 V? Without
  one, Option A needs a USB hub on the existing LTE modem's bus or a new UART device-tree node.

## Integration plan
1. Bench (antennas on a board on the lawn, robot driven by hand): log `/heading` (UM982) next
   to the aligner's COG heading; `scripts/mag_cal.py`-style stats: bias, std, p95.
2. Mount; measure `rtk_baseline_yaw_deg`; set `rtk_dual_antenna: true` in the aligner.
3. Fuse the GNSS heading as absolute yaw in both EKFs (`ekf_node` and `ekf_map`): a small
   converter publishes it as yaw-only `sensor_msgs/Imu` (covariance from UNIHEADING std dev,
   huge when QF != 4) as a second IMU input with only yaw enabled. The aligned WIT/stereo yaw
   stays the high-rate input; the GNSS heading removes its drift.
4. `heading_logic` becomes a divergence watchdog (GNSS heading vs gyro + COG, alarm and
   hold on a sustained disagreement) instead of the source of truth.
5. Behind a flag, then deleted after two clean field weeks: the blind calibration drive, the
   `pose_prior` re-seed, the stationary hold (`hold_move`, `held`), the calibration drive
   itself, the magnetometer seed, and the persisted offset continuity check.

## Shopping list (rough, check current prices)
| Item | Qty | ~Price |
|---|---|---|
| ArduSimple simpleRTK3B Compass (UM982) | 1 | EUR 300 |
| Multi-band helical antenna (L1/L2/L5), same model | 2 | EUR 50-80 each |
| SMA cable, same length (<= 0.5 m) | 2 | EUR 10-15 each |
| USB-C cable, strain relief, 3D-printed antenna mounts | 1 set | EUR 20 |

## Acceptance (REVIEW_2026-11-02 step 4)
- 20 STARTs: 10 off-dock, 5 after a power cycle on the lawn, 5 after 30 min idle, with zero
  HEADING_NOT_ALIGNED and no calibration drive.
- GNSS-minus-COG heading p95 under 1 deg over a full mow (blade on), QF 4 for >= 99 % of
  the mow time, no heading jump > 2 deg without a matching gyro rotation.
