# Error budget: leg-end and pivot accuracy (2026-10-20)

Step 2 of the coverage plan review. The question is whether manoeuvre-dense plans (many
pivots, short legs) are viable, and how far each edge mode has to inset the base_link path.
The go/no-go rule: **if the p99 leg-end or pivot error is over 0.10 m, manoeuvre-dense plans
are not viable.**

Tool: `scripts/sim_gate/accuracy.py`, which takes `field <series.json> <mission.log>` or
`sim <run>/data.json`. Each logged in-place pivot (pivot vertex, route corner, leg start,
final approach) is paired with its `/spin` goal and the next `/follow_path` goal. Pose `a`
is taken when the spin is sent and pose `b` when the next leg is sent.

- **Leg-end along / lateral:** `a` minus the vertex, in the heading frame at `a`.
  Positive along means an overshoot.
- **Pivot drift:** `|b - a|`. The excursion is the maximum displacement during the pivot.
- **Heading error:** the yaw turned minus the commanded angle.
- **Leg-start offset:** `b` minus the vertex, in the frame of the target heading.

Legs are classed by length: a swath is at least 1.5 m, a connector is shorter than
1.5 m, and a route corner counts as transit. All numbers are absolute values in m (heading
in deg). With n = 9-27 per class, **p99 is the max**.

## Data

- **Field:** run 7 (`20261016-054318`, master f257a15, the first complete mow), 46 pivots.
  The reference is the EKF (`/odometry/filtered_map`), so these are control errors as the
  stack saw them. There is no ground truth in the field.
- **Sim:** new world `src/mower_sim/worlds/accuracy.yaml`, an irregular 3.4 x 2.7 m hexagon
  run with the committed parameters and master's execution path.
  - 26-27 pivots per run, swath legs 1.5-3 m, connectors 0.2-1.2 m, about 6 min of mowing.
  - Two runs on domain 181: ideal, and `-d` (slip, wheel bias, pivot efficiency 0.85, loc
    drift, seed 1).
  - Reference: `/sim/ground_truth`.
  - The ideal run ended NAV_TO_DOCK_FAILED. The dock sits inside this small lawn; this does
    not affect the metrics.

## Results (p99 = max; median / p99)

| Metric | Class | Field run 7 (EKF) | Sim ideal (truth) | Sim `-d` (truth) |
|---|---|---|---|---|
| Leg-end along | swath | 0.008 / 0.023 | 0.011 / 0.017 | 0.051 / **0.120** |
| | connector | 0.037 / *0.350* | 0.008 / 0.018 | 0.008 / 0.055 |
| | transit | 0.046 / 0.096 | 0.008 / 0.029 | 0.021 / 0.058 |
| Leg-end lateral | swath | 0.052 / **0.106** | 0.018 / 0.064 | 0.025 / 0.064 |
| | connector | 0.029 / 0.058 | 0.025 / 0.064 | 0.011 / 0.059 |
| | transit | 0.022 / **0.330** | 0.000 / 0.033 | 0.024 / 0.058 |
| Pivot drift | all | 0.087 / **0.385** (p90 0.221) | 0.053 / 0.080 | 0.046 / 0.078 (EKF 0.113) |
| Pivot excursion | all | 0.094 / **0.402** | 0.053 / 0.080 | 0.048 / 0.080 (EKF 0.117) |
| Pivot heading error (deg) | all | 3.4 / 7.2 | 1.5 / 2.9 | 1.3 / 2.1 |
| Leg-start along | all | 0.103 / **0.456** (p90 0.250) | 0.024 / 0.091 | 0.049 / **0.121** |
| Leg-start lateral | all | 0.031 / **0.475** (p90 0.113) | 0.044 / 0.085 | 0.057 / **0.130** |
| Mowing cross-track (gate h, line) | | p99 0.089 (long swaths) | p99 **0.178** | p99 **0.207** |

The connector along 0.35 m and leg-start lateral 0.475 m come from run 7's double vertex
(-2.10, 1.63) / (-2.18, 1.68), which is 0.09 m apart. That geometry is manoeuvre-dense by
itself.

Findings:

1. **Pivots are not pure rotations, even in the ideal sim.** base_link moves 5 cm (median)
   in every pivot with a perfect drive. This is commanded motion: the spin, then the shaper
   or wheel floor. It is not slip.
   - In the field, the drift fits `2 L sin(dθ/2)` with L ≈ 0.06 m in most pivots, and two
     pivots drift 0.38 m.
   - The field drift is therefore either the same commanded creep or a GNSS lever-arm or
     heading error in the EKF. The latter matches run 7's open "4 cm right bias".
2. **Every pivot turns too far:** 46/46 in the field, 53/53 in the sim. The overshoot is
   1-3 deg in the sim and 2-7 deg in the field.
3. **Short legs do not settle.** The accuracy world has the same controller as run 7 but
   legs under 3 m. Its mowing cross-track p99 is 0.18-0.21 m, against 0.089 m on run 7's
   3.5-7 m swaths, and corner deviation fails gate e (max 0.23 m). The leg-start offset
   (field p90 0.25 m along / 0.11 m lateral, plus the heading error) is still being removed
   when a short leg ends.

## Verdict: NO-GO for manoeuvre-dense plans

The p99 exceeds 0.10 m in the field (leg-end lateral 0.106 swath / 0.33 transit, pivot drift
0.385) and in the sim with disturbance (leg-end along 0.120, leg-start 0.13, cross-track
0.21). Only the ideal sim stays under 0.10 m.

Plans should keep pivots few and legs long until the following are fixed and re-measured
with this tool:

- the pivot drift (pure-pivot regime, the lever arm);
- the spin overshoot.

## Recommended insets (base_link path to the boundary)

Footprint, from base_link: nose 0.52 m, rear 0.23 m, half-width 0.27 m. The pivot sweep
radius is 0.586 m (nose corner) and 0.355 m (rear corner).

Error terms, worst of field and sim `-d` p99:

- lateral 0.11 m;
- along 0.12 m;
- pivot excursion 0.22 m (field p90; the p99 is 0.40 m and is treated as a defect to fix,
  not budgeted).

| Edge mode | Driving along the edge (side) | Leg end facing the edge (nose) | Pivot at a vertex (sweep) |
|---|---|---|---|
| **Hard**: whole body inside + 0.10 m | 0.27 + 0.11 + 0.10 = **0.48 m** | 0.52 + 0.12 + 0.10 = **0.74 m** | 0.586 + 0.11 + 0.22 + 0.10 = **1.02 m** (1.20 m at the p99 drift) |
| **Soft**: sides may overhang 0.25 m, nose + rear inside | 0.27 - 0.25 + 0.11 = **0.13 m** | 0.52 + 0.12 = **0.64 m** | 0.586 + 0.11 + 0.22 = **0.92 m** (nose corners) |

So in both modes a pivot costs about 0.5 m more inset than a straight pass along the edge.
Edge passes should therefore turn on arcs or pivot only at the inner end, never on the
boundary.

**Minimum viable leg length: 1.5 m.**

- On legs of 1.5 m or more (run 7 swaths) the leg end holds 0.106 m lateral and 0.023 m
  along.
- Below that, the start offset (up to 0.25-0.33 m) and the 2-7 deg heading error are not
  removed before the leg ends. Cross-track p99 roughly doubles.
- Connectors under 1.5 m are acceptable only as non-cutting moves with the margins above.

## Pivot fix (b4784b0, same day)

**The metric was wrong.** The leg end was taken when the spin goal was sent, while the
robot was still coasting. The pivot end was taken when the next leg was sent, while the
robot was still turning. `accuracy.py` now measures both at rest (ebe1e7d). Re-measured,
the ideal sim's "5 cm pivot drift" turns out to be the leg coasting 5 cm past the vertex.
The rotation itself moves base_link by 4 mm. The overshoot at rest is 7 deg, not the 1.5
deg reported above.

Root causes:

1. **Legs end at 0.15-0.22 m/s and coast past the vertex.** The along-path goal checker
   fires at the vertex plane. The robot then coasts v·0.2 s + v²/1.6 (cmd_vel_slew decel
   0.8 m/s², 0.15 s wheel lag, pipeline delay). Fix: `coverage_corner_goal_checker` ends
   the leg that stopping distance short of the plane (`stop_latency_s` 0.2, `stop_decel`
   0.8).
2. **The Spin velocity limits were above the MCU clamp.** The settings were max 1.0, min
   0.4 and acc 2.0 rad/s, while the MCU clamps at 0.3. As a result:
   - Spin never decelerated and stopped from 0.3 rad/s.
   - The slew first spent 0.1 s winding a phantom 0.4 rad/s down to 0.3.

   Fix: 0.3 / 0.25 (the pivot floor) / 1.0 (the slew decel), cycle 20 Hz. The mission
   also sends the Spin 0.04 rad short (`pivot_stop_lead_rad`) to cover the stop coast.

Results on the accuracy world. Values are median / p90 against sim truth, at rest. In the
"before" columns the drift measures 4 mm in the ideal run and 0.02 m under `-d` after the
re-measurement.

| | Ideal before | Ideal after | `-d` before | `-d` after |
|---|---|---|---|---|
| Leg-end along (m) | 0.052 / 0.078 | **0.012 / 0.029** | 0.077 / 0.116 | **0.030 / 0.091** |
| Leg-end lateral (m) | 0.019 / 0.063 | 0.012 / 0.048 | 0.025 / 0.059 | 0.022 / 0.053 |
| Pivot drift, rotation only (m) | 0.004 / 0.004 | 0.004 / 0.005 | 0.011 / 0.020 | 0.011 / 0.021 |
| Heading overshoot at rest (deg) | 7.0 / 8.4 | **1.3 / 3.4** | 5.3 / 7.0 | **0.6 / 1.8** |
| Leg-start lateral (m) | 0.044 / 0.074 | **0.013 / 0.037** | 0.057 / 0.121 | **0.015 / 0.070** |
| One-wheel ticks | 0 | 0 | 0 | 0 |
| Gate e / h (line p99) | 0.180 / 0.178 | 0.192 / 0.188 | 0.232 / 0.207 | 0.219 / 0.206 |

**Field run 7, re-measured at rest:**

- coast 0.044 m;
- rotation drift 0.110 / 0.220 m;
- overshoot 4.7 / 7.8 deg.

The sim cannot reproduce the rotation drift. A drift of about 2·L·sin(θ/2) with
L ≈ 6 cm points to a GNSS lever-arm or antenna-offset error, or to an off-centre pivot on
turf.

**Field check:** do 10 pivots of ±90 deg and ±180 deg on flat turf, with a marked
base_link (rear-axle centre) point. Compare its tape-measured displacement and the final
heading with the EKF's. The result separates a real displacement from a lever-arm error,
and calibrates `pivot_stop_lead_rad` and `stop_latency_s`.

## Lead 0.06 (2bf0f97) and regression check

On the accuracy world, values are median / p90 against sim truth, at rest. All four runs
had 0 one-wheel ticks.

| | Ideal 0.04 | Ideal 0.06 | `-d` 0.04 | `-d` 0.06 |
|---|---|---|---|---|
| Heading overshoot (deg, abs) | 1.3 / 3.4 | **0.7 / 1.5** (signed -0.2) | 0.6 / 1.8 | **0.5 / 1.1** (signed -0.5) |
| Leg-end along (m) | 0.012 / 0.029 | 0.017 / 0.029 | 0.030 / 0.091 | 0.032 / 0.093 (p99 0.147) |
| Leg-end lateral (m) | 0.012 / 0.048 | 0.012 / 0.065 | 0.022 / 0.053 | 0.045 / 0.086 (p99 0.101) |
| Leg-start lateral (m) | 0.013 / 0.037 | 0.016 / 0.041 | 0.015 / 0.070 | 0.033 / 0.077 |
| Rotation drift (m) | 0.004 / 0.005 | 0.004 / 0.005 | 0.011 / 0.021 | 0.011 / 0.023 |
| Docked | no (dock inside the lawn) | yes | yes | yes |

**real_run5, ideal (2bf0f97, domain 181):**

- Mow complete. Docking failed with DOCK_MAXOUT after 3 vision retries. This is the known
  sim marker-yaw failure (simulation.md); the docking server's own controller does not use
  behavior_server.
- 0 hard-boundary trips. Max outside in every phase is 0.000 m.
- 0 one-wheel ticks (134 floor-stalled).
- Corners: 0 of 29 deviate more than 0.15 m (max 0.056 m).
- MOWING cross-track p99: line 0.037, arc 0.028, pivot vertex 0.032, pivoting 0.056 m.
- The 7 DriveOnHeading final approaches completed without an abort. BackUp is not used.

## Field pivot test (owner, by hand)

The test has two purposes:

- Find out whether run 7's ~6 cm pivot-centre offset is a real displacement on turf or an
  antenna lever-arm error in the EKF.
- Calibrate `pivot_stop_lead_rad`. The mission applies the lead; a hand-sent Spin does not,
  so the overshoot measured here is the lead needed.

The owner triggers every motion. Claude sends none.

1. **Ground.** Pick flat, mown turf with an RTK fix. Stick tape on the ground under the
   rear-axle centre (base_link): halfway between the two wheel hubs, plumbed down.
   - Mark a second point 0.5 m ahead on the centreline to read the heading.
   - Clear 0.7 m around the robot: the nose swings about 0.6 m.
2. **Mode.** In the GUI, start manual mowing (joystick mode, blade off). That puts the
   mission in MANUAL_MOWING with motion enabled and Nav2 active.
   - Leave the joystick untouched: the teleop lane (priority 20) would override the Spin.
   - The GUI STOP and the hardware e-stop stay live throughout.
3. **Record.** Run this on the mower, in its own shell:

   ```
   docker exec -it mower_jazzy bash -c 'source /opt/ros/jazzy/setup.bash && source /work/install/setup.bash && ros2 daemon stop; cd /userdata/ros2/logs && ros2 bag record -s sqlite3 -o pivot_test /fix /fix_gated /odometry/gps /odometry/filtered /odometry/filtered_map /imu/data /tf /tf_static /cmd_vel /cmd_vel_nav /cmd_vel_raw /mcu/measured_speed /mcu/commanded_speed /spin/_action/status /behavior_tree_node/high_level_status'
   ```

   The container mounts `/userdata/ros2_stack` at `/work` (checked 2026-10-20). Do not filter
   `/tf`: it carries map→odom→base_link. There is no base_link→gps TF: the stack treats
   the antenna as sitting on base_link, so any real offset shows up as a pivot circle.
4. **Pivots.** Run 10 pivots, all from the same mark, in this order: +90, -90, +90, -90,
   +180, -180, +180, -180, +90, -90 (degrees). For each one, in a second shell:

   ```
   docker exec -it mower_jazzy bash -c 'source /opt/ros/jazzy/setup.bash && source /work/install/setup.bash && ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 1.5708, time_allowance: {sec: 30}}"'
   ```

   Use target_yaw ±1.5708 or ±3.1416. This is exactly the production pivot path:
   behavior_server Spin → nav lane → cmd_vel_slew → MCU.
5. **Measure after each pivot.**
   - Tape the offset of the new rear-axle point from the floor mark, along and across
     the start heading.
   - Measure the heading change against the 0.5 m nose mark: lateral offset / 0.5 m,
     converted to degrees.
   - Note the time.
   - Move the robot back onto the mark with the joystick before the next pivot.
6. **Stop.** Ctrl-C the recorder, end manual mowing in the GUI, and copy the bag off
   read-only.

How to read the results:

- **Lever-arm error.** The tape shows the axle mark staying within about 1 cm while the EKF
  base_link draws a circle (`/odometry/filtered_map`, ≈ 2·L·sin(θ/2)). The radius of the
  raw `/fix` circle about the true pivot point gives the real antenna offset; add a
  base_link→gps TF (and the EKF/navsat lever arm) to match.
- **Real displacement.** Tape and EKF agree on the displacement. The cause is turf
  mechanics, which a TF fix cannot correct: budget for it in the pivot inset.
- **Stop lead.** Take the tape heading and the final IMU/EKF yaw minus the target, as the
  median over the 10 pivots. Set `pivot_stop_lead_rad` to that value in rad (the sim implies
  0.06 rad). The coast time from `/cmd_vel` = 0 to
  `/mcu/measured_speed` = 0 calibrates `stop_latency_s`.

## Field pivot test result (bag `pivot_test`, 2026-10-20)

The bag ran 611 s in MANUAL_MOWING with RTK fixed: `/fix` status 2, static noise 1.7 mm.
It contains **4 Spin pivots, not 10** (+90, +90, +180, -180). They were segmented from
`/cmd_vel_nav`. The Spin profile already shows the new limits (0.30 → 0.27 → 0.25 → 0
rad/s), and no mission lead was applied. Turf rotation is stick-slip: IMU |w| ranges
0.1-0.5 rad/s under a 0.3 command, so a 180-degree turn takes 12-16 s.

Overshoot is measured against the target. "Coast" and "stop" are measured from the
Spin's first zero command, to `/imu` |w| < 0.02 rad/s. Displacement is the EKF base_link
displacement, start to end. The tape showed about 0 for every pivot.

| Pivot | IMU-integrated | EKF / odom yaw | Overshoot (IMU / odom) | Coast | Stop | EKF disp. (`filtered_map` / `odometry/gps`) |
|---|---|---|---|---|---|---|
| +90 | 91.2 | 92.4 | +1.2 / +2.4 | 0.9 | 0.11 s | 0.161 / 0.153 m |
| +90 | 93.3 | 92.0 | +3.3 / +2.0 | 1.7 | 0.35 s | 0.194 / 0.192 m |
| +180 | 182.8 | 183.0 | +2.8 / +3.0 | 1.1 | 0.26 s | 0.079 / 0.079 m |
| -180 | 185.2 | 185.7 | +5.2 / +5.7 | 4.1 | 0.24 s | 0.179 / 0.179 m |

Angles are in degrees.

1. **Overshoot.** The median is 3.0 deg on the IMU (2.7 deg on odom), at most 5.7 deg.
   - It does not scale cleanly with turn size: +1.2 and +3.3 at 90 deg, +2.8 and +5.2 at
     180 deg. A fixed lead is right.
   - About 2 deg of it is coast after the Spin's stop command, which ends 0.1-0.35 s later
     (median 0.25 s). The rest is the Spin's own stop decision on stick-slip rotation.
   - The owner's "about 5 deg" read matches the worst pivot.
2. **Displacement is the antenna, not the axle.**
   - The EKF reports 0.08-0.19 m per pivot while the tape shows about 0, and
     `/odometry/gps` reports the same displacement.
   - `/fix` traces a circle of radius 0.068-0.102 m (Kasa fit). For 90 deg pivots the
     chord, 2R·sin(θ/2), matches the EKF displacement.
   - (Corrected below.) The UM960 stamps `gps_link`: bringup.launch.py overrides the
     driver default `gps`. The URDF had base_link→gps_link at (0, 0, 0.3), so
     navsat_transform treated the antenna as sitting on base_link.
   - A joint least-squares fit per pivot (pivot centre plus a vehicle-frame lever, EKF
     yaw) gives x = +0.085, +0.119, +0.017, +0.081 m and y = -0.058, -0.028, -0.057,
     +0.030 m.
   - Sweeping the fix latency from -0.6 s to +1.5 s does not reconcile the lateral sign,
     and the fit residual is 2.7 cm rms against 1.7 mm static noise. The centre wanders
     during stick-slip.
   - **Forward offset: about +0.07 m** (range 0.02-0.12). **Lateral offset: undetermined**
     (mean -0.02 m, spread ±0.05).
   - This matches run 7's ~6 cm pivot-centre offset (2·L·sin(θ/2) with L ≈ 0.06-0.10).
3. **Run 7's 4 cm right-of-travel bias is not explained.** That bias was measured in the
   EKF frame. A constant antenna offset is self-consistent on a straight, and a forward
   offset cannot bias lateral tracking at all. It stays open (wheel bias or the
   proportional-only MPPI correction).

Recommendations (no code changed yet):

- **`pivot_stop_lead_rad`:** keep 0.06 rad (3.4 deg), which already covers the 3.0 deg
  field median. The field n = 4; re-check with 10 or more pivots after the lever-arm
  change.
- **`stop_latency_s` / `stop_decel`:** keep 0.2 s / 0.8 m/s².
  - The angular stop measured 0.25 s median, in line with 0.2 s plus one Spin cycle.
  - The linear stop was not exercised: pure pivots only. Calibrate it from the next mow's
    leg ends with `accuracy.py field`.
- **Lever arm:** add a static TF base_link→`gps` (the frame the UM960 stamps) of
  x +0.07, y 0.00, z 0.30.
  - Do it in mower_bringup's localization launch (static_transform_publisher). The
    on-robot URDF publishing `gps_link` lives outside this repo.
  - navsat_transform then subtracts the rotated lever when it builds `/odometry/gps`.
    The EKF configs and the controllers are unchanged.
  - Measure y on the robot with a ruler (antenna centre against the rear-axle centre)
    rather than trusting this fit.
- **Downstream after the TF:** the EKF base_link moves about 7 cm back along the heading.
  - Re-measure the docked dock pose (it shifts about 7 cm along the dock yaw).
  - Boundaries recorded by driving were recorded at the antenna. They shift by at most
    7 cm, in a heading-dependent way.
  - mower_sim already stamps `gps_link`; mirror the TF in the sim so gates stay
    comparable.
  - Pivot drift and run 7's pivot-centre offset should drop to the turf-scrub level.

## Lever arm applied (ws/lever-arm, 01f2e97)

The owner measured the antenna centre at x +0.10 m from the rear axle (base_link), y 0.

**Frames.**

- The UM960 `/fix` is stamped `gps_link`: `bringup.launch.py` overrides the driver
  default `gps`, and the field bag confirms it.
- robot_state_publisher publishes base_link→gps_link from `config/urdf/mower.urdf.xacro`
  (`gps_xyz`).
- That transform was (0, 0, 0.3). It is now **(0.10, 0, 0.3)**. This is the only stack
  change; no frame was renamed.
- navsat_transform looks up base_link→gps_link. It rotates the arm by its world yaw (the
  IMU, `use_odometry_yaw: false`) and removes it when it builds `/odometry/gps`, so both
  EKFs receive base_link.
- `mower_sim` now publishes `/fix` at the antenna, base_link + R(yaw)·(0.10, 0), through
  the `gps_antenna_x` / `gps_antenna_y` parameters. A test keeps them equal to the URDF.
- The gates run `localization:=sim`, which does not use `/fix`, so they show only that
  nothing else moved.

**Other consumers of the GNSS position:**

| Consumer | How it uses the fix | Impact |
|---|---|---|
| heading_aligner | Course over ground from `/odometry/gps` on straight drives | The offset is constant on a straight, so no change |
| gps_gate | Quality only | None |
| GUI `/gps/fix` and datum check | Status and position display | Display only. The datum (map origin) keeps its meaning |
| mower_alerts | Covariance | None |
| map_server | Uses the EKF pose (`/odometry/filtered_map`), not the fix | Affected through the EKF; see below |
| docking | Uses the EKF pose | Affected through the EKF; see below |
| `rtk_pair.py` | Pairing only | None |

**Saved dock pose: shift it, or re-record it on the charger.** The pose was recorded as
the EKF pose while docked, which was the antenna position. With the lever arm, the same
docked robot reads 0.10 m further back along the dock yaw:

- new = old − 0.10·(cos yaw, sin yaw);
- for the run 5 dock (0.236, -0.637, yaw 1.773 rad) that is **(+0.020, −0.098) →
  (0.256, −0.735)**.

Without the shift, the docking approach target sits 10 cm too deep. The simplest fix is to
press "set docking point" again with the robot on the charger after the deploy (requires
`dock_pose_measured`).

**Recorded paths and boundaries: no migration needed.** They were recorded at the antenna,
10 cm ahead of the axle along the direction of travel. A point shifted along its own
direction of travel stays on the same line, so straight edges are unchanged. A bend's
vertex moves at most 0.10 m along the edge, and the polygon's inset is unchanged to
within a few millimetres.

A one-off migration is possible but not recommended. It would be an optional script that
moves each recorded vertex 0.10 m back along its recorded heading: the direction from the
previous vertex, in recording order, which only `areas.dat`'s vertex order preserves.

**Deploy:** the URDF is read by xacro at launch (`config/urdf`, symlinked).

1. Sync.
2. Restart the stack. The mower_sim change only matters on the dev host.
3. Re-set the docking point on the charger.
4. Re-run the field pivot test: the EKF displacement per pivot should drop from 0.08-0.19
   m to the turf-scrub level, a few centimetres.
