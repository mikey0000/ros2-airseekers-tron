# Ring drift: why the 2026-10-06 mows A and B stopped on the boundary

Date: 2026-10-06. Area 0 ("Area 1", 8.2 m², 16-vertex polygon in `/userdata/ros2/maps/areas.dat`).
Plan: zigzag, `operation_width` 0.18, `headland_rings` 2, edge first. RPP coverage controller
`FollowCoveragePath`, `desired_linear_vel` 0.3.

## Data sources and a gap

- **The passive recorder did not cover either run.** `/tmp/ftc_rec_b.csv` stops at
  1791265387.0, about 19 minutes before run A. It covers 194 s with the robot standing still:
  3895 `/cmd_vel` rows, all zero. Its duration limit ran out. So there is **no pose trace,
  no /odom twist and no carrot** for A or B. Cross-track error cannot be measured directly.
- The evidence below comes from the per-node ROS logs in the container
  (`/root/.ros/log/*_179126576*.log`). `docker logs` aborts on a NUL byte
  (`invalid character '\x00'`), so use the node log files.
  - `cmd_vel_slew`: 1 Hz "applied" line. This is the `/cmd_vel` that goes to the MCU driver, before the MCU clamp.
  - `slip_detector`: |cmd − measured| events. The measured value is **`/odom`, the MCU's
    wheel-speed estimate** (log line "measured twist source: /odom"). `/wheel_vel` does not exist.
  - `behavior_tree_node`, `map_server_node`, `controller_server`, `heading_aligner`.
- Plan geometry: I re-planned the same polygon offline with the live `/coverage/plan` service
  (a stateless planner; same 55 poses, 2 rings, 10 swaths as the mission log).

**Before the next test, re-run the recorder with no time limit**, or record a bag of `/cmd_vel`, `/odom`,
`/odometry/filtered_map` and `/mower_mcu_driver/sent_speed`. The "after" numbers in this document
are predictions, not measurements.

## Timeline

| t (s, …266xxx) | Event |
|---|---|
| A 500–551 | MOWING on ring 1. `/cmd_vel` angular.z swings between −0.60 and +0.52 every 1–2 s at 0.3 m/s (a weave) |
| A 552–567 | linear 0, angular +0.13…+0.30 for 16 s: rotating in place while barely turning (the "rocking" that was fixed live with max_angular_accel 3.2) |
| A 570.55 | RPP "collision ahead" ×4, then boundary violation at (−1.77, −1.69). That point is 0.35 m outside edge 10→11, the SE edge just after the 109° SW corner |
| A 573.8 | Recovery transit 1/2 (blade off). Commands: linear 0, angular +0.20…+0.29 for 10 s |
| A 583.8 | Lethal stop (more than 0.5 m outside) during the transit. Not logged by map_server, which only logs while cutting |
| B 766.9 | Re-plan. Resume cursor pose 156 (ring 2, NW edge) |
| B 768–802 | follow_path #1. 11 slip events. RPP "collision ahead" ×7, then "Controller patience exceeded", abort |
| B 802.8 | Retry from pose 227 (ring 2, SE edge, approaching the 109° SW corner) |
| B 804–814 | Pivot attempts at ±0.30 rad/s with linear 0. Measured yaw rate about 0.02–0.08 rad/s |
| B 815–838 | linear 0.15–0.30, angular +0.23…+0.60 almost continuously for 23 s: a sustained left turn that does not converge |
| B 838.21 | heading_aligner: "IMU yaw changed 20.9 deg" inside a COG window. Heading quality "poor" since 807 |
| B 838.50 | Soft violation (debounced) at (0.02, 0.26) |
| B 838.64 | Lethal violation at (0.06, 0.23), 0.51 m outside edge 13→14 (SE side), **0.14 s after the soft pause** |

## (a) What limits the achieved yaw rate

### Commanded yaw rate (`cmd_vel_slew` applied, 1 Hz samples)

| Run | Samples while moving | \|ω\| > 0.30 (above the MCU clamp) | \|ω\| = 0.60 (recurring ceiling upstream of the slew; source not identified, smoother max is 1.0) | Sign flips of ω at v ≥ 0.25 |
|---|---|---|---|---|
| A (500–570) | 69 | 26 (38 %) | 8 | 11 |
| B (768–838) | 69 | 37 (54 %) | 10 | 5 |

### Slip events turned into the measured yaw rate

`d_angular = |ω_cmd − ω_odom|`, threshold 0.30. Taking ω_cmd from the nearest 1 Hz slew sample:

| Command | Events (A+B) | Implied measured \|ω_odom\| |
|---|---|---|
| Arc, v 0.05–0.30, \|ω\| ≥ 0.45 | 34 | 0.09–0.30, median 0.23 |
| Arc, v 0.30, \|ω\| 0.32–0.44 | 8 | 0.01–0.10, median 0.06 |
| Pivot, v 0, \|ω\| 0.27–0.40 | 8 | **0.02–0.08**, median 0.05 |

In every one of the 50 events the measured rate was at most 0.30. It never exceeds the clamp.

### Verdict

1. **MCU clamp: the primary limit.** `mcu_node._map_command` clamps linear and angular
   **independently** to ±0.3. RPP asked for 0.4–0.6 rad/s at 0.3 m/s (R = 0.5–0.75 m). The wire
   carried 0.3/0.3 (R = 1.0 m). The robot then turned on at least twice the radius the
   controller planned. Clamping each axis separately does not preserve curvature, and every
   curvature error points outward on a turn.
2. **Drive under load: a second, real limit.** Below the clamp, `/odom` (wheel speeds, so this is
   *not* ground slip) shows that the drive does not reach 0.3 rad/s. Arcs commanded ≥ 0.45 achieve 0.09–0.30 rad/s (median 0.23), mid arcs only about 0.06,
   and pivots with the blade on in turf achieve almost nothing (0.02–0.08 rad/s at 0.3 commanded).
   This matches REVIEW_2026-10-06 ("measured about 0.2 rad/s in turf"). Wheel encoders cannot see
   grass slip, so the actual turn rate over ground may be lower still.
3. **Smoother and slew: not limiting.** The `cmd_vel_slew` target and applied values match in 54/69 (A) and 51/69 (B)
   samples. Where they differ it is ±0.1–0.15 for one tick (angular accel 1.0 rad/s²).
   `velocity_smoother` (θ max 1.0, accel 3.0) passes everything the clamp later cuts.

## (b) Does RPP's lookahead or regulation push it outside at the corners?

Yes, through **regulation that triggers too late**, plus a weave caused by the short carrot:

- `regulated_linear_scaling_min_radius: 0.5` only slows the robot when R < 0.5 m (ω > 0.6 at 0.3 m/s).
  For 0.5 m < R < 1.0 m, RPP keeps 0.3 m/s and asks for 0.3–0.6 rad/s. That is the 0.3/0.4–0.6 band
  that dominates both logs, and the clamp then widens the actual radius.
- Ring corners on this polygon: ring 1 has 109° over 0.23 m at the SW corner (vertices 9–10), 92° at the N tip,
  and a **146°/175° spike** at the NE (vertices 15/0, a 0.6 m sliver that ring 1 follows). Ring 2 still
  has the 109° SW corner. The swath ends are 127–159° hairpins over 0.18–0.45 m. At R ≥ 1 m, no corner
  on this lawn can be tracked. Pivoting (`rotate_to_heading`) is the alternative, but pivots stall in turf (see (a)).
- Lookahead 0.45 m (0.3 m/s × 1.5 s), minimum 0.3 m. With actuator lag and the yaw cap, the loop is
  under-damped: ω flips sign every 1–2 s at full speed (16 flips at v ≥ 0.25 in the two runs). Each half-cycle
  builds lateral error before the clamped turn removes it.
- Heading: COG alignment stayed "fair", then "poor" (offset 103.44° never refined, because the robot
  was always turning). A yaw bias e gives a steady pure-pursuit offset of about L·tan e
  (≈ 0.16 m at 20°, L = 0.45). This is a secondary factor.
- Both violations sit at or just after the SE/SW side (A: 0.35 m off edge 10→11; B: 0.51 m off
  edge 13→14), downstream of the 109° SW corner and the swath hairpins on the SE side.

## (c) Is the outer ring too close to the boundary?

Yes. **`mower_coverage_node` passed `border_inset = 0.0`**, so ring 1's centreline lies **exactly on the
recorded polygon** (minimum distance 0.00 m in the re-plan). Every outward tracking error is then
outside the area:

| Margin (base_link vs recorded polygon) | Value | Confirmation | Effective reserve from the ring-1 centreline |
|---|---|---|---|
| soft (pause) | 0.30 m | 3 samples @ 5 Hz = 0.6 s ≈ 0.18 m at 0.3 m/s | 0.30 m |
| lethal (latched e-stop) | 0.50 m | 1 sample | 0.50 m |

With the debounce, a robot leaving at 0.3 m/s is already about 0.48 m out when the soft pause
confirms. In run B the pause came 0.14 s before the lethal latch, so the pause had no effect.
The observed outward excursions are at least 0.35 m (A) and at least 0.51 m (B) from a line that the ring
centre sits on. The ring-2 centre is only 0.18 m inside.

Sizing the inset: outer ring centre ≥ tracking error + half swath. After the controller change
below, the expected cross-track error at ≤ 0.2 rad/s turns is about 0.10–0.15 m (an estimate; the
FTC simulation with the same yaw limit gives < 0.07 m). Half swath is 0.09 m, so **0.20 m**.
Re-planned with 0.20: minimum distance to the boundary is 0.20 m, the NE spike is gone (no turns > 120° on the ring),
the plan has 2 rings + 8 swaths, 40.5 m (was 55.9 m), and F2C coverage fraction is 0.84. Cost: an uncut
band of about 0.20 − 0.10 (blade radius) = 0.10 m along the edge, to be trimmed by a later edge pass
or by lowering the inset once tracking has been measured.

## Vendor ROS 1 behaviour (from the docs)

- `wheel_control_semantics.md`: the vendor sends raw `float32 lin, ang` with **no clamp**. The ±0.3
  is the clamp of the vendor `PidControllerROS` position primitive (rotate / moveStraight), not of its
  path follower. Our port applies ±0.3 to *every* command, per axis.
- `audit_2026-10-05/vendor_mission_layer.md`: vendor `PlannerPath.type` has separate **edge "origin"
  and "inset 1/2"** paths, so the vendor drove inset edge laps as well as the on-the-line one.
  The inset distance is not documented in the decompile notes (not found in the
  `ros2_port_handoff` planner sources either). Open question.

## Changes made (local + rsynced to `/userdata/ros2_stack`; no restart)

| File | Parameter | Old → new | Why |
|---|---|---|---|
| `src/mower_navigation/config/nav2_params.yaml` `FollowCoveragePath` | `regulated_linear_scaling_min_radius` | 0.5 → **1.5** | RPP slows itself so that v/R ≤ 0.3/1.5 = 0.2 rad/s, which is what the drive delivers. Curvature is kept by the controller, not lost at the clamp |
| 〃 | `regulated_linear_scaling_min_speed` | 0.10 → **0.08** | keeps ω = v/R ≤ 0.2–0.3 down to R ≈ 0.27–0.4 m before rotate-to-heading takes over |
| 〃 | `lookahead_dist` / `min_` / `max_` | 0.4/0.3/0.6 → **0.5/0.4/0.7** | damps the weave (lower path-following gain against a lagging, capped actuator) |
| 〃 | `max_angular_accel` | 3.2 (live fix, kept) | |
| `src/mower_coverage/src/mower_coverage_node.cpp` | new param `border_inset_m` (read per request; default 0.0) | hard-coded 0.0 → param | the planner already supported `border_inset`; the node never exposed it |
| `src/mower_coverage_bridge/launch/coverage_bridge.launch.py` | launch arg `border_inset_m` | → **0.20** | ring-1 centre 0.20 m inside the line (see (c)) |
| `src/mower_map/config/map_server.yaml` | `boundary_check_rate_hz` / `boundary_debounce_samples` | 5 / 3 → **10 / 2** | soft confirmation 0.6 s → 0.2 s (0.06 m). The soft pause can now happen before the lethal latch |
| 〃 | soft / lethal margins | 0.3 / 0.5 (kept) | with the 0.20 inset, the reserve from the ring becomes 0.5 m (pause) and 0.7 m (lethal) |
| `src/mower_navigation/test/test_params_yaml.py` | new `test_coverage_rpp_respects_drive_yaw_limit` | | guards desired_linear_vel / min_radius ≤ 0.21 rad/s |

`mission.yaml` was not changed. Its boundary block (recover after 3 s, 2 recoveries) is reasonable once the
soft pause fires before the lethal one. The margins live in `map_server.yaml`, not `mission.yaml`.

Tests: `./scripts/dev_build.sh test --packages-select mower_coverage mower_coverage_bridge
mower_navigation mower_map` gives 903 tests, 0 errors, 0 failures.

**Activation on the mower:** the YAML and launch files are symlinked from `install/` into `src/`, so they take effect
on the next stack restart. `border_inset_m` needs a mower-side rebuild of
`mower_coverage` (C++) before that restart. Until then the new launch argument is ignored, which is safe, and the ring
stays on the line. `nav2_params` depends on `desired_linear_vel` staying 0.3. If the GUI cut speed is raised,
raise `regulated_linear_scaling_min_radius` to v/0.2.

## Recommended follow-ups (code, not done here)

1. **Clamp that preserves curvature** in `mcu_node._map_command`, or better in `cmd_vel_slew`: if |ω| > ω_max,
   scale v by ω_max/|ω| as well. This removes the outward bias for every controller and every lane.
2. **Pivots in turf barely turn** (0.02–0.08 rad/s at 0.3 commanded). Characterise the in-place turn with blade on and off.
   Consider `use_rotate_to_heading` with a small forward speed (an arc), or a pivot deadband boost.
3. Try `FollowCoveragePathFTC` (decoupled lateral PID, `max_cmd_vel_ang` 0.3, < 7 cm cross-track in simulation)
   on this lawn as an A/B against the retuned RPP.
4. `map_server_node` logs a lethal violation only while cutting. Log it in transit too (run A's lethal stop had no position logged).
5. Recorder: drop the duration limit and add `/mower_mcu_driver/sent_speed` and the base_link pose, so that next time
   cross-track error and the post-clamp command can be measured.
