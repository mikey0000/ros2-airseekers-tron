# Wheel scale, heading bias and motor current (A2 + A4), 2026-10-22

Offline analysis of recorded field bags; no mower access was needed. Review items A2 and A4
of `2026-10-21_independent_review.md`. Branch `ws/wheel-cal`.

## Data

| set | what | used for |
|---|---|---|
| `field_mow_2026-10-21c` | evening mow, 73 min, 80 splits (split 81 is corrupt, 41 recovered from WAL) | everything |
| `field_stuck_2026-10-21b` | 25 min stuck session, short legs | current baseline, detector false positives |
| `field_pivot_test2` | 4 pivots at 0.3 rad/s with raw `/imu/data` | pivot rate |
| run 7 (`20261016-054318`, partial, 17 min) | earlier mow, before the lever-arm fix | cross-check only (n = 4 straights) |

Bag rates: `/fix` 10 Hz (RTK fixed for the whole 21c mow), `/cmd_vel` 20 Hz,
`/mcu/measured_speed` 5 Hz, `/mower_sensor_info` 10 Hz. `MotorInfo` itself changes only
once per second: the MOTORS frame rides with battery/version (median 0.95 s between
changes). `/imu/data` is not recorded during a mow, so the EKF yaw rate stands in for it.
The extraction is reproducible with `scripts/wheel_cal.py` (A2) and the scratch scripts
described under each section.

## A2: wheel scale, track and the right-of-travel bias

### Method

1. **Straights.**
   - Take the forward RTK track (v_cmd > 0.08 m/s), split it with Ramer-Douglas-Peucker
     (5 cm tolerance), and keep pieces of 3 m or more whose quadratic-fit residual is
     under 3 cm. Drop the first 0.5 s and the last 0.2 s of each piece.
   - Result: 31-32 segments, 158-162 m, median 4.4 m at 0.215 m/s, all with the blade on
     but one.
   - "|w_cmd| ≈ 0 for 3 m" never happens: MPPI weaves at a median |w| of 0.077 rad/s. The
     segments are therefore straight in the ground track, and the command is averaged.
2. **Per segment:**
   - RTK curvature, from a quadratic fit of the lateral deviation against distance;
   - mean `/cmd_vel` w;
   - mean MCU measured v and w;
   - MotorInfo rpm (left and right);
   - ground speed from RTK;
   - the fused yaw from `/odometry/filtered_map` minus the course of the EKF base track
     (the "heading lead");
   - all of these are given with a 95 % CI over segments.
3. **Opposite-direction pairs.**
   - Pair adjacent antiparallel lanes driven back to back (under 300 s apart, under 0.3 m
     apart; n = 12).
   - The mean of the two lanes is the vehicle-frame part. The half-difference is the
     map-fixed part (cross slope, terrain).
4. **Lane-gap bias.** With a steady offset d to the right of travel, two adjacent opposite
   lanes are s + 2d apart when each lies on the other's left, and s − 2d apart when each lies
   on the other's right. Fitting s and d over the pairs measures the steady tracking offset
   in the RTK frame without needing the plan, which is not in the bags.

### Results (21c mow)

| quantity | value (95 % CI) | reading |
|---|---|---|
| ground / commanded speed | 1.0076 ± 0.0048 | speed loop fine (0.8 %) |
| ground / MCU measured speed | **1.049 ± 0.005** | `/odom` vx reads 4.9 % low |
| RTK curvature on straights | −0.0007 ± 0.0020 1/m | closed loop holds a straight course |
| commanded w on straights, paired | **+0.0055 ± 0.0037 rad/s** (+0.026 ± 0.017 1/m) | a small CW yaw disturbance that MPPI cancels |
| commanded w, map-fixed half-difference | ±0.0058 rad/s | as large as the vehicle part: terrain |
| MotorInfo rpm right/left, paired | 1.002 ± 0.012 | no measurable wheel-scale asymmetry (integer rpm at 1 Hz: ±1.2 % resolution) |
| **fused yaw − course** | **+2.37 ± 0.51 deg** (paired +2.37 ± 0.79) | EKF heading leads the true course, constant in the vehicle frame |
| lane-gap offset | **+1.64 ± 0.62 cm right of travel**, swath 0.187 ± 0.012 m | the "right bias" (run 7: 4 cm) |
| pivots, measured/commanded, CCW / CW | 0.748 ± 0.035 / 0.806 ± 0.030 | see Pivots |

### Cause: the heading aligner lags the gyro drift

- `heading_aligner` blends COG measurements into a fixed IMU→ENU offset (alpha 0.1, one
  update about every 5.5 s).
- During the 21c mow the offset walked from 115 deg to −81 deg. The drift rate was
  −4.4 to −4.8 deg/min for the first 40 min, then −0.7 to −1.9 deg/min.
- A low-pass that follows a ramp lags it by rate × T × (1 − α)/α. That is about 2.4 deg at
  4.5 deg/min.
- The aligner's own log shows the same thing: the mean COG innovation over the 815 updates
  was **−2.41 deg** (sd 2.53).
- Per segment, the heading lead regresses on the local offset drift rate:
  `lead = 0.13 (±0.49) − 0.74 (±0.14) deg per (deg/min)`, with r = −0.88. The implied lag is
  44 s, and the intercept is zero, so the drift lag explains all of the lead.
- How this gives a right-of-travel offset:
  - The controller holds the fused yaw on the path. The true heading is therefore 2.4 deg
    clockwise of the path, and the robot drifts right.
  - RTK pulls the position back, and the proportional-only MPPI settles with a steady
    offset to the right.
  - The sign is independent of travel direction.
- Pair-level fit: `d = 0.64 (±1.09) cm + 0.43 (±0.40) cm/deg × lead`.
  - At 21c's 2.4 deg this gives 1.7 cm.
  - Run 7's straights had a lead of about 9 deg (n = 4, pre-lever-arm). The fit then
    predicts about 4.5 cm, which matches the 4 cm run 7 bias.

### Antenna lateral offset against wheel bias: the discriminator

- **A lateral antenna error (`gps_link` y) cannot produce this signal.** The EKF places
  base_link at antenna − R·lever, and the controller drives that estimate onto the path. The
  antenna track (RTK) and the EKF track then sit exactly on the planned lanes. The physical
  base_link is shifted by the error, but nothing that is measured in the RTK or EKF frame
  sees it: lane gaps, the EKF-vs-plan offset and the commanded w all stay at their nominal
  values.
  - The bias that was measured (run 7 EKF-vs-plan, and the 21c RTK lane gaps) is
    therefore a tracking error, not an antenna error.
  - An antenna y error is only visible against something physical, such as the cut stripe
    against the antenna track, or a pivot-circle centre fit. The 10-20 error budget fitted
    it as −0.02 ± 0.05 m, which is undetermined. **No `gps_link` change is justified.**
- **A wheel-speed or curvature asymmetry** shows up as a steady mean w_cmd at zero RTK
  curvature. That mean is the same in the vehicle frame on both directions of a pair, while
  slope effects flip.
  - It is present but small: +0.0055 ± 0.0037 rad/s.
  - Under the controller's proportional gain it accounts for the residual d0 = 0.6 ± 1.1 cm
    at most.
  - MotorInfo rpm shows no wheel asymmetry within ±1.2 %.
- **A heading lead** shows up as fused yaw − course, constant in the vehicle frame, with
  w_cmd unaffected. This is the dominant term.

### Pivots

`field_pivot_test2` (raw IMU) gives measured/commanded = 0.81 ± 0.06 (n = 4). The 21c mow
(EKF yaw rate, n = 125, blade on) gives `yaw = 0.784·w_cmd − 0.0096 (±0.006) rad/s`.

- **The pivot is not centred.** The wheel driving backward turns at 3.0 rpm, against 5.1 rpm
  for the wheel driving forward (MotorInfo, both directions).
  - Mean wheel speed 4.05 rpm × 2 / 0.39 m gives 0.24 rad/s. That is the measured
    0.235 rad/s, so the under-rotation is the MCU under-driving the reversing wheel, not
    ground slip.
  - The 1 rpm forward bias, 0.012 m/s, is the 5 cm "commanded creep" per pivot from the
    10-20 error budget.
  - The MCU's own measured w is 0.90 × the command.
- **Effective track.** The data gives no evidence that the 0.40 m split against the 0.39 m
  track matters on straights.
  - For pivots, the yaw from the wheel differential matches the 0.39 m physical track once
    the reversing-wheel deficit is counted.
  - A track change in the MCU split would rescale all turning (+2.6 %). The closed-loop
    yaw controllers already absorb that.
- **The same −0.01 rad/s additive CW yaw shows up in blade-on pivots.** Taken with the
  +0.0055 rad/s straight-line trim, a constant CW yaw torque while cutting (blade reaction or
  side load) is the likely source. The blade-off data (n = 14 pivots, 1 straight) is too thin
  to confirm it; the field procedure below separates it.

### Correction implemented

**`heading_aligner`: drift-rate tracking (alpha-beta), commit `a5c6a14`.** The parameters
live in `AlignParams` and are also ROS parameters:

| param | default | revert | meaning |
|---|---|---|---|
| `rate_beta` | 0.005 | **0.0** (old behaviour) | rate gain per COG update: `rate += β·innov/dt` |
| `max_offset_rate` | 0.15 deg/s | n/a | clamp (21c peak 0.08 deg/s) |
| `rate_horizon` | 120 s | n/a | the rate is integrated at most this long after the last accepted COG |

- The offset is integrated at the tracked rate on every IMU sample.
- Every seed and reset zeroes the rate: COG seed or replace, dock, RTK, file, IMU reset.
- `status` reports `offset_rate_deg_min`.
- β ≈ α²/(2−α) gives a critically damped alpha-beta filter. It is bounded by the clamp and
  the horizon, so a parked robot cannot spin its heading.

**Replay on the 815 real 21c COG measurements:**
- mean innovation −2.42 → **+0.01 deg**;
- sd 2.47 → 2.01 deg;
- worst 60-update rolling mean 4.0 → 0.64 deg.

Unit tests (`test_heading_logic.py`, 5 new; package suite 245/245):
- a 3 deg/min ramp lags 1.05 deg with β = 0 and 0.05 deg with the new default;
- the clamp holds;
- the horizon holds;
- reset on seed and on IMU reset;
- step response settles;
- β = 0 reproduces the old blend.

**Expected effect:**
- the heading lead goes from about 2.4 deg to below 0.3 deg;
- the right-of-travel offset goes from 1.6 cm (21c) to about 0.6 ± 1.1 cm (the residual
  d0);
- alternate lane gaps go from 15.2/21.6 cm to about 18.6 ± 2 cm.

It is not a controller change. The sim cannot show it: the sim IMU has no drift ramp. The
check is the next field mow: run `scripts/wheel_cal.py` on it and expect a heading lead
under 0.5 deg and a lane-gap offset under 1 cm.

**Not implemented, and why:**
- **Per-wheel scale.** The asymmetry is not measurable (rpm ratio 1.002 ± 0.012), and the
  MCU takes only v/w, so a host-side "wheel scale" would be a curvature trim anyway.
- **Curvature/yaw trim (+0.0055 rad/s).** It is significant only at about 3σ, it is blade-on
  only, and it is mixed with terrain. Decide after the blade-off/on field calibration. If it
  is confirmed, add an additive `yaw_trim_rad_s` in `cmd_vel_slew` while |v| > 0 and the blade
  is on (revert 0), and do not use a wheel-split constant.
- **Slow integral cross-track trim.** Not needed once the heading lead is gone. If a
  residual over 1 cm remains, the place would be the MPPI path-follow reference (a ≤ 2 cm
  lateral shift, time constant ≥ 60 s, frozen in pivots). Mission/coverage are owned by
  `ws/edge-layout2`.
- **Odometry scale.** `/odom` vx reads 4.9 % low (ground/MCU 1.049 ± 0.005). It only matters
  during RTK loss, at 5 cm per metre of dead reckoning. Recommended follow-up: an
  `odom_linear_scale` of 1.049 in `mcu_node` for the measured speed (not `linear_scale`,
  which maps commands, where the ratio is 1.008).
- **`gps_link` y.** Not observable from these data (see the discriminator).

### Field calibration procedure (owner)

1. **Set up.**
   - RTK fixed, flat open strip of 6 m or more, firm grass.
   - The stack runs as for a mow: `bag_recorder` records the topics below. Otherwise
     record them with `ros2 bag record`.
   - Topics: `/fix /cmd_vel /mcu/measured_speed /mcu/commanded_speed /mower_sensor_info
     /odometry/filtered /odometry/filtered_map /imu/data /heading_aligner/status /battery`.
2. **Blade OFF, manual mode (GUI joystick).**
   - Drive 4 straight lines of 5-6 m at a steady ~0.2 m/s with no steering input.
   - Drive back each time on the strip right next to the outgoing line, about 20 cm
     over.
   - Then do 2 pivots each way at the 0.3 rad/s pivot speed, each held at least 3 s.
3. **Repeat step 2 with the blade ON**, same strip and same speed.
4. **Run** `scripts/wheel_cal.py <bag dir>` on each half. If one bag holds both halves,
   use `--min-len 3`; it reports per run. The script prints the speed and odom scale, the
   paired yaw-rate bias (vehicle frame) with its CI, the rpm ratio, the heading lead, the
   lane-gap offset, the pivot ratios, and the suggested constants.
5. **Read the result.**
   - **Blade-off yaw bias significant:** a drive asymmetry. Set the trim, or ask whether
     the MCU accepts a split constant.
   - **Blade-on only:** a cutter torque. Use a blade-gated trim.
   - **Heading lead over 1 deg:** check `rate_beta` and the aligner log
     (`offset_rate_deg_min`).

## A4: motor current

### Semantics (from bags + driver + spec)

- **Wire order** is cutter, left, right, height. `mcu_node` publishes `MowerSensorInfo` in
  10 mA units:
  - cutter raw is already 10 mA;
  - drive raw is read as mA, so msg = raw / 10.
- **Rate:** 1 Hz, not 33 Hz (it rides in the 1 Hz MOTORS frame). This bounds any
  current-based reaction time.
- **"left"/"right" labels are swapped against the physical wheels.**
  - In CCW pivots (IMU-confirmed w > 0), the motor labelled *left* turns forward (rpm −5.2,
    where negative means forward) and *right* turns backward (+3.1). CW pivots are the
    mirror.
  - Currents agree: the forward torque sign is negative on "left" and positive on "right"
    (mirrored mounting), and the pivot pattern matches only with the labels swapped.
  - Nothing consumes the per-wheel identity today. Fix it in `mcu_node` (`MOTOR_NAMES`)
    once it is confirmed on hardware: lift one wheel, drive, and see which label moves.
- **Drive current** is signed torque current. Typical values (A, as `mcu_node` SI;
  absolute scale unverified):

  | state | per wheel |
  |---|---|
  | standstill hold | 4.1 / 0.7 |
  | steady forward | 4.0 / 4.2 (sum median 8.4, p99 16.5, p99.9 20) |
  | reverse | about 6 |
  | pivots | 7-9 (sum 15-16 average, peaks 23) |
  | wheel spin (06:37) | 10-21 (sum 23-31) |

  Noise in steady forward driving is sd 3 A per wheel at 1 Hz.
- **Battery regression:**
  - draw = 0.80 A + 0.018 A per drive-A (msg/100) + 3.24 A × cutting, residual 1.55 A;
  - battery draw is 0.6 A still, 1.2 A driving, 4.3 A cutting (24 V).
- **The cutter "current" is not a load signal.**
  - It is symmetric about zero (p1/p50/p99 = −214/−0.9/+214 "A") with lag-1 autocorrelation
    0.76: an aliased AC phase sample.
  - Its 1 s envelope correlates with battery draw at r = 0.15.
  - Cutter rpm is regulated and quantised (2954/3006/3060, 52 rpm steps) and does not
    sag, r = 0.06.
  - **The only blade-load proxy is battery current** (cutting adds 2.5-7.8 A, p5-p95),
    at 1.7 Hz and mixed with the drive.

### Events

| event | current | ground speed | lead |
|---|---|---|---|
| 07:28:33.8 bumper (the 07:29 abort) | sum 9 → 11 → 17 A in the last 2 s; battery 2.3 → 4.6 A | RTK 0.23 → 0.15 m/s against 0.18 commanded | the 17 A sample comes 0.2 s before the bumper and is inside the forward p99 |
| 07:39:51.1 bumper (the 07:40 abort) | sum 6 → 14 A at 51.0 | 0.17 against 0.21 | 0.1 s, in the noise |
| 06:36:57-06:37:00 wheel spin | sum 23-31 A for 3 s; one wheel at 15-18 rpm, the other at 2-4 rpm | RTK 0.01 m/s against 0.12 commanded | fires at 06:36:58.8, **2.0 s before** the stuck guard (06:37:00.78) |
| 21b stuck events (6) | sum 7-16 A | v_cmd 0.00-0.02 (MPPI dithering) | not current events: nothing to detect |

### Viability

- **Bumper pre-emption: not viable.**
  - The load rises at most 0.3-1 s before contact.
  - The 1 Hz frames catch it at most once, inside the normal forward spread.
  - A threshold low enough to catch 07:28 (17 A) fires about 50 times per hour of forward
    driving.
- **Sustained stall / dig: viable as a confirmation, on thin evidence.**
  - Rule: sum > 20 A, mean |v_cmd| ≥ 0.08 over 1 s, RTK speed < 0.3·|v_cmd|.
  - In the 73 min mow it gives exactly 1 hit, the 06:37 spin, 2 s before the stuck guard.
  - In the 21b session it gives 1 hit: a 25 A reverse at 01:25:38 at 0.11 m/s against
    0.15, which is plausibly real load.
  - Requiring 2 consecutive frames removes that hit but also delays the true one past the
    guard.
- **Blade-load-adaptive speed: not supported by current data.**
  - The cutter current field is unusable and rpm does not sag.
  - Battery current carries the blade load, but there is no grass-height ground truth to
    calibrate it.
  - Spin-up windows show 3.5-4.1 A in the first 20 s against 3.3-5.2 A in the next 40 s,
    with no consistent long-grass signature.
  - Needed first: a mow with known long and short patches, and battery current logged at
    its full rate.

### Detector (pure logic only), commit `096ed0a`

`mower_control/motor_load.py` `DriveLoadDetector`:
- `on_motor()` takes each new MotorInfo frame, `on_motion()` takes v_cmd and ground speed,
  and `update()` returns `{kind: spin|stall, load_a, cmd_mps, ground_mps}` with a 10 s
  holdoff;
- it makes no decision when the frame is older than 2.5 s;
- it reports kind `spin` when the mean |rpm| is above 2.
- 4 unit tests (mower_control 686/686).

It is not wired in. Proposed integration once `ws/edge-layout2` lands: the slip_detector
node feeds it, and an event becomes a `/stuck_event` reason `drive_load` that starts the
existing escape 2 s earlier. Pivots stay with the stuck guard.

## Follow-ups

1. **Field check** of the aligner change on the next mow with `scripts/wheel_cal.py`:
   expect a heading lead under 0.5 deg and a lane-gap offset under 1 cm.
2. **Blade off/on calibration drive** (procedure above), to settle the +0.0055 rad/s yaw
   bias and the cutter-torque hypothesis.
3. **`mcu_node`:**
   - `odom_linear_scale`: **done 2026-10-29** (review B9, branch `ws/loc1029`). The
     parameter scales the MCU-measured linear speed used for `/odom` (twist and dead
     reckoning) only, not the commands; `/mcu/measured_speed` stays raw. `bringup.launch.py`
     sets 1.06 (field 1.049 on 10-22, 1.066 on 10-28; it varies with turf, re-check with
     `scripts/wheel_cal.py`, whose new "ground / fused odom speed" line should read
     1.00 ± 0.01 afterwards). Stock has no equivalent:
     `MowerOdom::speedDataCallbackSDK` (mower_base_node, 0x003e5c1c) dead-reckons the raw
     SpeedData m/s with no scale or wheel geometry, so stock odometry reads short too;
   - confirm the left/right label swap wheels-up and fix `MOTOR_NAMES`;
   - raise the pivot reversing-wheel deficit (3.0 against 5.1 rpm) with the MCU team or
     the cmd_vel_slew shaping.
4. **Wire `DriveLoadDetector`** after `ws/edge-layout2`, as advisory first (log only) for
   2 mows.
