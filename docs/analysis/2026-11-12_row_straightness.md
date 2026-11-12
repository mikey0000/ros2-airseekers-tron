# Row straightness after V2: row entry, antenna tilt, roll feed-forward, 2026-11-12

Branch `ws/rows1112` (off master e62237e). Follows the 2026-11-12 field review of the V2 mow
(area 4, 63 rows; `2026-11-06_mppi_damping.md` has the V2 change and its field result).

## 1. Field findings

- The planned rows are straight, and the EKF map pose follows RTK to 0.2 cm. What is left is
  the drive and what the controller sees.
- In-row deviation, settled part: 1.65 cm rms. The 0.08-0.4 Hz weave band is 1.21 cm of it.
- **Row entry.** The first 1.5 m are 3.3 cm rms. At the row start the mower is already at
  0.20 m/s with 8.3 deg rms heading error. The bias depends on the direction: +5.3 deg mean on
  the -75 deg rows, -2.1 deg on the 105 deg rows. It settles after about 3 m. Fixing it should
  take the whole-row rms from 1.99 to about 1.7 cm.
- **Antenna tilt sway.** The RTK antenna is about 0.324 m above the ground (URDF gps_link z
  0.214 above the axle + 0.11 m wheel radius). Nothing compensated body roll or pitch: the EKF
  is 2D and navsat_transform removes only the horizontal 0.098 m lever. The unexplained lateral
  input of the row loop correlates with 0.324 sin(roll) at r -0.73, coefficient -1.34, zero
  lag. So the controller steered against the antenna's sway. The coefficient is 1.3 x nominal:
  either the effective height is larger or the 2 Hz `/tilt/status` roll under-reads. Expected
  from compensation: weave about -19 %, slow bow about -20 %, and the ~5 cm offset at 9 deg
  roll goes away.
- **Roll feed-forward.** `k_roll` in PathAlignAheadCritic read the raw `/imu/data` roll every
  cycle, so the roll wobble also moved the controller's target. About -5 %.

## 2. Changes

| Change | Where | Parameters (default) |
|---|---|---|
| Fix moved from the antenna to the ground point | `mower_localization/gps_gate.py`, `gps_tilt.py` | `tilt_compensation` (true), `tilt_gain` (1.0), `tilt_antenna_height` (0 = TF), `tilt_axle_height` (0.11), `tilt_fix_latency_s` (0.1), `tilt_max_age_s` (0.1), `tilt_max_yaw_sigma_deg` (20), `tilt_imu_topic` (/imu/data_aligned). Launch: `gps_tilt_compensation`, `gps_tilt_gain` |
| Row entry speed hold | `mowgli_nav2_plugins` RowEntryCritic (`row_entry.hpp`), FollowCoveragePathMPPI | `entry_speed` 0.10 m/s, `heading_tol_deg` 2.6, `ramp_accel` 0.10 m/s^2, `release_speed` 0.30, `max_entry_m` 3.0, `row_min_m` 1.0, `row_straight_tol_deg` 6, `cost_weight` 30 |
| Roll feed-forward low-pass | PathAlignAheadCritic (`path_trim.hpp` RollFilter) | `roll_tau_s` 3.0 (0 = raw) |
| `/imu/data` in the bag ring | `mower_control/bag_recorder.py` DEFAULT_TOPICS | 100 Hz |
| Sim antenna height | `mower_sim` sim_node / loc_drift | `gps_antenna_height` 0.324, `gps_tilt_compensation`, `gps_tilt_gain` |

### 2.1 Tilt compensation

Body x forward, y left, z up (REP-103: roll > 0 = right side down, pitch > 0 = nose down).
A point h above the ground contact moves horizontally by

    forward = h cos(roll) sin(pitch),   left = -h sin(roll)

in the body-yaw frame, rotated by the ENU yaw into the map. gps_gate subtracts that from the
fix (lat / lon, local WGS84 radii) before navsat_transform sees it.

**Sign check against the field.** On a roll > 0 the antenna moves to the right, so the
reported position moves right and the controller steers left. The field's lateral input N
(left of travel +) regressed on 0.324 sin(roll) with coefficient -1.34. The model's lateral
antenna displacement is -h sin(roll): the same sign. `test_gps_tilt.py` checks this for both
row directions of area 4.

Inputs and guards:
- Roll, pitch and ENU yaw come from `/imu/data_aligned` (heading_aligner's republish of the
  WIT message: gravity roll / pitch, yaw rotated into ENU).
- The sample used is the one nearest the fix epoch, `stamp - tilt_fix_latency_s`. The UM960
  driver stamps on receipt, about 0.1 s after the epoch.
- A sample further than `tilt_max_age_s` from the epoch is not used. Nor is one whose yaw sigma
  (`orientation_covariance[8]`, heading_aligner's quality) is above 20 deg. In both cases the
  fix passes unmoved, counted and warned.
- Height: base_link -> fix frame z from `/tf_static` (read without tf2_ros, `static_tf.py`
  now also resolves translations) plus the wheel radius. 0.324 until `/tf_static` arrives.
  `tilt_antenna_height` > 0 overrides it. `tilt_gain` scales it for the field A/B: the
  regression suggests about 1.3.

### 2.2 Row entry

Cause in the sim (per-row table, section 3): the coverage pivot ends **short** of the row
heading. On the `-d` runs the heading at the first forward sample is 4-9 deg off, and on the
side the pivot came from. The Spin is sent short by `pivot_stop_lead_rad` (0.10 rad = 5.7 deg)
and the stop coast is smaller than that, so the remainder is left to FollowPath, which
finishes it while already driving at 0.2 m/s. Two smaller terms add to it:
- The join heading aims at a point 0.4 m ahead on the row. A few centimetres of lateral offset
  after the pivot are therefore already several degrees off the row.
- A turn about a centre ahead of base_link moves base_link sideways by turn direction.

Short pivots on alternating turn directions give the field's direction-dependent sign.

A stop lead that fits the field coast (or one learned per direction) would fix the pivot
itself. The coast is a turf property that the sim does not model reliably, and `/imu/data`
was not in the bags until now. So the fix here takes any residual, whatever its cause:
RowEntryCritic. When a new path starts with a straight row (at least 1.0 m within 6 deg; MPPI only sees the path up to prune_distance 1.2 m ahead) and
the heading is more than 2.6 deg off it, forward speed is held at 0.10 m/s until the heading is
within 2.6 deg. Then the cap rises at 0.1 m/s^2 and lets go at 0.30. An aligned start, a bend
or a short turn leg is never capped, and the hold ends after 3 m regardless. It is an MPPI
cost on the sampled forward speed above the cap, so wz is untouched. At 0.10 m/s the inner
wheel stays above the 0.06 m/s floor up to |w| 0.2, so MPPI can still turn. The rest of the
row runs at the cut speed.

### 2.3 Roll feed-forward

`path_trim::RollFilter`: a first-order low-pass with tau 3 s, fed in the IMU callback. It
restarts at the new sample after a gap longer than `roll_timeout_s`. At 0.5 Hz a 3 s filter
passes 11 % of the wobble and all of the steady slope (`test_path_trim.cpp`).

### 2.4 Bag ring

`/imu/data` (100 Hz) is in DEFAULT_TOPICS again. Size, measured on a synthetic 60 s split
(324 B CDR per message, JY61P quantisation, sqlite3 rows, zstd -1 as `bag_compress`): 2.36 MB
raw and 0.43 MB compressed per minute. That is **~141 MB/h before compression and ~26 MB/h in
the ring**. The 11-12 mow pin was ~57 MB/h, so this adds about 45 %. CPU: ~8 % of a core
while the full ring runs (2026-10-09 measurement for a 100 Hz topic). The low-power ring does
not record it.

## 3. Sim gate

All runs are `-d -l -s 2 -x 3`, output under `../simout_rows1112/`, with the controller loop at a
median of 20 Hz. Metrics come from two scratch scripts. `entry.py` covers rows = plan straights
of at least 2.3 m, using truth: |d| and heading rms over the first 1.5 m, the heading at the
first forward sample, v at s = 0.3 m, and settle = the first s after which |d| <= 1.5 cm and
|psi| <= 2 deg hold for 1 m. `slope_off.py` gives the mean truth offset toward +x (uphill) on
samples with |roll| > 5 deg, and the map estimate minus truth there. The weave figures come
from `simout_look1105/weave.py`.

**Baseline.** The master run from the main checkout (`master-flat`, e62237e) went
STUCK_NEEDS_HELP at 470 s: a transit route corner pivot was refused 4 times at (-0.98, 4.85).
That left only 3 rows, too few to use. The first branch run (`branch-flat`) had RowEntryCritic
loaded but never holding: MPPI gives critics the path only up to `prune_distance` 1.2 m, so
`row_min_m` 1.5 never matched. On the flat world roll is 0, so the tilt compensation and the
roll low-pass do nothing there. That run is therefore master's row behaviour on this branch,
and it is used as the flat baseline. A second branch run showed holds ending early. The goal
MPPI passes is in odom and creeps with map -> odom, so the critic now treats only a goal jump
(> 0.25 m per cycle, as LegGoalChecker) as a new path.

### 3.1 real_run5 (flat)

| | master-eq (`branch-flat`) | **branch (`branch3-flat`)** |
|---|---|---|
| verdict | FAIL (a 0.112 / 46, i) | FAIL (i only; a 0.090 / 0) |
| b / c / d / e | 0 / ok / 0 / 0 of 21 (0.125) | 0 / ok / 0 / 0 of 21 (0.118) |
| i side / nose / sweep | 0.112 / 0.083 / 0.104 | 0.131 / 0.071 / 0.088 |
| entry 0-1.5 m: \|d\| rms / psi rms | 2.33 cm / 3.51 deg | **1.97 cm / 2.97 deg** |
| heading at the first forward sample (mean / rms) | +0.8 / 7.7 deg | +1.5 / 6.5 deg |
| v at s = 0.3 m | 0.205 | 0.194 |
| settle p50 / p90 (settled rows) | 1.53 / 2.12 m (4 of 12) | 2.66 / 3.12 m (4 of 12) |
| whole row rms / settled rms | 1.89 / 1.54 cm | 1.83 / 1.73 cm |
| weave band rms / XTE p90 / max | 0.97 / 2.62 / 5.27 cm | **0.85** / 2.84 / 5.20 cm |

The holds are short. After a pivot the robot starts from rest, MPPI turns onto the row within
about 1.5-2.4 s sim (log: hold -> aligned), and the ramp releases 2 s later. So v at 0.3 m
hardly changes: the robot is already past most of the ramp. The entry lateral drops 15 % and
the entry heading 15 %. Only 4 of 12 rows meet the strict settle test either way, so settle
p50 here is noise. i side (0.131) is the run's worst sample, at a transit, as on master's
fails. 12 rows per run is a small sample.

### 3.2 real_run5_slope (10 deg west edge)

| | V2 master (`simout_v2rb/v2-slope-s2`) | branch, tilt on | branch, tilt on, rerun | branch, **tilt off** |
|---|---|---|---|---|
| verdict | FAIL | FAIL | FAIL | FAIL |
| a max / samples | 0.162 / 89 | 0.409 / 1302 (RETURNING_HOME) | **0.135 / 65** | 0.134 / 54 |
| b / c / d | 0 / ok / 0 | 1 / STUCK / 1 | 0 / ok / 0 | 0 / ok / 0 |
| e | 5 of 21 (0.363) | 5 of 19 (0.390) | 6 of 21 (0.437) | 7 of 21 (0.377) |
| h p99 line / arc | 0.234 / 0.329 | 0.223 / 0.259 | 0.240 / 0.264 | 0.219 / 0.192 |
| entry \|d\| rms / psi rms | 3.42 cm / 6.20 deg | 2.90 / 5.20 | 3.51 / 6.12 | 4.24 / 6.62 |
| weave band rms | 1.27 cm | 1.44 | 0.93 | 1.05 |
| map estimate - truth, slope samples (x) | -0.19 cm | -0.04 | -0.30 | **-5.40** |
| truth offset toward uphill, slope samples | -2.31 cm | -2.78 | -3.57 | -1.50 |

- **Tilt on.** With compensation on, the sim's map estimate on the slope equals the truth, as
  before this change (the old sim had no antenna height). So "tilt on" is the master
  localisation, and these runs check that nothing else changed. The first run went STUCK in a
  post-mow transit at (-5.02, -0.43). The stuck escape's reverse made no progress on the slope
  (terrain reverse slip), the forward arc crossed the line by 0.10 m, and the retrace stalled.
  That transit runs FollowPath, which has neither the row hold nor the roll feed-forward, and
  its localisation is identical to master's. The rerun with the same flags was clean, with a
  and samples better than V2's. Read the first run as slope-world spread.
- **Tilt off.** This is what the robot did before this change, now modelled: the map estimate
  sits **5.4 cm downhill** of the truth at 10 deg (0.324 sin 10 deg = 5.6 cm). With
  compensation on it is 0.0-0.3 cm. The truth offset on the slope rows moves by only 1-2 cm,
  because the rows cross the 2.8 m band at 50 deg and spend ~25 s on it in total. The
  20 s trim and the controller do not settle in that time. On a long slope row the whole
  5.6 cm reaches the ground track: the field's ~5 cm at 9 deg.
- **No sim terrain wobble.** The sim's roll is the steady slope value, so the weave gain the
  field expects from compensating roll wobble (about -19 %) cannot show here. Flat world:
  compensation is exactly neutral (roll 0). Slope world: neutral on the control loop, and it
  removes the steady offset.

## 4. What needs the field

1. **Tilt compensation A/B** (launch `gps_tilt_compensation:=false` vs default): weave band rms
   and slow bow on area 4, and the offset on a slope row. With `/imu/data` now in the bag,
   repeat the field regression of the lateral input on h sin(roll) at 100 Hz. If the
   coefficient is still about -0.3 (residual) instead of ~0, try `gps_tilt_gain:=1.3`.
2. **Row entry**: entry 1.5 m rms (field 3.3 cm) and the start heading per direction. The
   pivot's short stop (`pivot_stop_lead_rad` 0.10 vs the turf coast) can be measured from the
   new `/imu/data` yaw rate per turn direction. A per-direction lead is the follow-up if the
   hold is not enough. `entry_speed` / `ramp_accel` are live MPPI parameters. If the
   field shows the same "fast align, then fast ramp", try `ramp_accel` 0.05.
3. **Roll low-pass**: part of the same A/B (no separate switch needed: `roll_tau_s` 0 = old).
4. **Bag size**: check the pin size of the next mow (expect ~57 -> ~83 MB/h) and recorder CPU.

Reverts without a rebuild: `gps_tilt_compensation:=false` (launch), RowEntryCritic
`enabled: false` or dropped from the critics list, `roll_tau_s: 0.0`.
