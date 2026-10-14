# Heading during the 2026-10-14 mow (bag `/userdata/ros2/mows/20261014-021244`)

Owner report: "the heading is not working quite right again". Bag seconds below are relative to
the session start (02:12:46 UTC). Evidence: `/heading_aligner/status` JSON (1 Hz), `/fix`,
`/odometry/filtered_map`, container log.

## What happened

1. **The stereo yaw integration drifts while moving; the WIT does not.** The status field
   `virtual_minus_wit_deg` (stereo-integrated virtual yaw minus the JY61P yaw) went from
   −50.1° at t = 3 to +2.4° at t = 1102: +14° over the first 12 minutes of mowing (≈ 1.2°/min)
   and +38° over the 6-minute stop-start before the transit abort (≈ 6°/min). It did not move
   at all during the 20 minutes at rest (rest hold). The COG corrections over the same first
   10 minutes were all negative (offset +0.96° → −7.77°, innovations −2…−7.6°, 10 % blended),
   i.e. the GPS course agreed with the WIT and disagreed with the stereo yaw by the same
   growing amount. The last accepted COG window was at t = 607; every later window was
   rejected ("IMU yaw changed > 15°" in the weaving / pivoting, or "slow").
2. **By the end of mow 1 the heading was ≈ 60° wrong.** Chord of the last arc (t 1090–1100):
   GPS track −13°, EKF yaw ≈ 45° over the same arc. The mission ended itself at t = 1120:
   boundary violation 0.34 m outside, blade off, then the recovery planner failed four times
   ("Start occupied" / no plan within 0.3 m) → MOWING_INCOMPLETE (20 of 27 sub-paths).
3. **START accepted the stale heading.** 20 minutes later the second START saw
   "heading aligned (cog)" with quality *poor* and a 1200 s old last fix and planned at once
   (`_heading_ok` checks only `aligned` + source). The RETURNING_HOME transit drove on a
   ≈ 65° wrong heading until the aligner re-seeded from three consistent COG outliers
   (−64.4°, −65.4°, −64°) at t = 2423: offset −7.77° → −74.44°. After that the COG innovations
   were +4.0°, +3.4° and the dock approach worked.
4. The teleop in between is innocent: the WIT and the virtual yaw both counted 92° for it
   (commanded 163°; the rest was pivot slip).

## Why the stereo path drifts (best reading of the code)

`YawSource.on_stereo` skips the increment of any sample whose stamp gap to the previous
sample exceeds `max_dt` (50 ms); the WIT only fills in when the *receive* time of the last
stereo sample is older than `max_dt` at the WIT callback (`on_wit`, `gap`). A burst that
arrives late but arrives (IIO watermark bursts, a late 50 Hz poll of the depth-100 best-effort
queue under load) therefore loses its rotation outright. Lost rotation has the sign of the
rotation at the time: mow 1 was edge-first clockwise laps (cumulative −560° in 2.5 min) and
every pivot request in the stop-start was clockwise, so the loss shows as the observed
positive drift, and it scales with how much the robot was turning (6°/min while pivoting
vs 1.2°/min in the laps). A wrong in-motion bias (−0.025°/s rest bias ≈ 1.5°/min) fits the
lap phase but not the 6°/min phase. The gyro scale is fine: gyro turn / GPS turn between
straight legs = 0.99–1.05.

## Proposed fixes (not implemented)

- `heading_logic.YawSource`: never discard rotation. Integrate a gap longer than `max_dt`
  with the mean of the bracketing rates (or the WIT delta over that gap), and count
  `stereo_gaps` / `stereo_gap_s` in the status so the next bag shows it. Log a WARN when
  `virtual_minus_wit_deg` moves more than ~5° in a minute while the stereo source is active.
- `mission_fsm._heading_ok`: a `cog` heading whose last COG fix is older than N s (300?) or
  whose quality is `poor` counts like a `file` prior: run the heading calibration drive first
  (it already exists for the file source, 1 m straight, blade off).
- Field check after the first fix: `virtual_minus_wit_deg` must stay within a few degrees over
  a mow; if the WIT keeps agreeing with the GPS course, `imu_source: wit` is the fallback.
