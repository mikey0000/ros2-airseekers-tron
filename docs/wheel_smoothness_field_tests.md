# Wheel smoothness: what changed, what is still open, and how to field-test it

Review of 2026-10-14 (velocity chain nav2 → twist_mux → cmd_vel_slew → mcu_node, compared
with the vendor controller). Fixes 1, 2, 3, 6, 7 are merged on master (6f6db39), unit-tested
only. Nothing has been driven yet. Every item below needs one field run and a bag review
before it counts as done. Background: `wheel_control_semantics.md`.

## Merged, awaiting field test

| # | Change | Where | Live-settable? |
|---|---|---|---|
| 1 | Turn shaper keeps its pivot/arc regime: switches only `turn_shaper_hysteresis_mps` (0.02 m/s) past the boundary or after `turn_shaper_latch_ticks` (6) ticks | `cmd_vel_slew.py` `TurnShaper` | yes (`ros2 param set /cmd_vel_slew ...`) |
| 2 | MPPI `ax_max 0.5`, `ax_min -0.8`, `az_max 1.0` (FollowPath and FollowCoveragePathMPPI) so rollouts match the slew | `nav2_params.yaml` | restart |
| 3 | `max_linear_decel` 0.8 (vendor value), `max_angular_decel` = angular accel; emergency / bumper lane zero is applied at once (`hard_stop_lanes`) | `cmd_vel_slew.py` | restart |
| 6 | Both axes ramp together whenever the turn shaper is active (`slew_both_wheels_always` true): an arc from rest keeps its curvature and never puts the inner wheel in (−0.06, 0) | `cmd_vel_slew.py` `slew_coupled` | restart |
| 7 | nav2 velocity_smoother widened (accel 1.0, decel −1.5) so cmd_vel_slew is the single limiter | `nav2_params.yaml` | restart |

Known trade-offs: with 6, the inner wheel may roll forward below 0.06 m/s for the first and
last ticks of an arc (same direction as the outer wheel, so not a one-wheel pivot); set
`slew_both_wheels_always: false` to get the old per-axis slew back. Coupled stops from an
arc are paced by whichever axis hits its decel limit first.

## Not changed yet (decide after the baseline runs)

| # | Issue | Candidate fix | Kind |
|---|---|---|---|
| 4 | Docking reverse steers on one wheel: 0.035–0.08 m/s with up to 0.3 rad/s puts the inner wheel below the 0.06 floor; docking is exempt from the shaper. Vendor: linear 0.05, heading kp 0.2, angular cap 0.2 | (a) cap `|w| <= (|v| - 0.06) / 0.24` in `reverse_control`, or (b) `min_turn_speed` 0.035 → ~0.09 while steering, `kp_heading` 1.5 → 0.5, `max_angular` 0.3 → 0.2 (`docking.yaml`) | (a) code, (b) params |
| 5 | Angular clamp 0.3 rad/s is ours, not the vendor's (vendor rotated at 0.4–0.8). A 0.3 pivot puts each wheel at 0.07 m/s, on the deadband floor; pivots on turf achieved 0.02–0.08 rad/s | `mcu_node angular_max` 0.4, with `velocity_smoother max_velocity[2]`, MPPI `wz_max`, RPP rotate speed and the shaper's pivot floor moved together | params, 4 files |
| m1 | mcu_node clamps linear and angular independently, distorting curvature above the clamp | clamp proportionally | code |
| m2 | `/odom` falls back to the commanded speed when the MCU measurement is stale | publish nothing / inflate covariance | code |
| m3 | heartbeat period documented inconsistently (100 ms vs 10 ms) | doc check against the frame log | doc |

## Field tests

Deploy and restart are the owner's steps (`deployment.md`). The rolling black box
(`bag_recorder`) already records every lane (`/cmd_vel_nav`, `/cmd_vel_docking`,
`/cmd_vel_bumper`, `/cmd_vel_emergency`), `/cmd_vel`, `/mcu/commanded_speed`,
`/mcu/measured_speed`, `/odom`, `/imu/data_aligned` and `/cmd_vel_slew/gate_status`; each
mow is pinned under `/userdata/ros2/mows/<ts>/` and each emergency under
`/userdata/ros2/incidents/<ts>_<reason>/`. Copy the directory off the mower after each run.

After the restart, confirm the new build is live before driving:

```sh
ros2 param get /cmd_vel_slew max_linear_decel          # 0.8
ros2 param get /cmd_vel_slew slew_both_wheels_always   # True
ros2 param get /controller_server FollowPath.ax_min    # -0.8
```

### Test A: mow the closest area (fixes 1, 2, 6, 7; baseline for 5)

One short mission from the dock: transit out, a few lanes, transit back. Watch for:
hunting at turn entry (the old pivot/arc flip), the first metre of each arc, the stop at
each goal, and how long a pivot takes to actually turn the robot.

Bag review (`/userdata/ros2/mows/<ts>/`):
- regime flips: count sign changes of `/cmd_vel.linear.x` while `/cmd_vel_nav.angular.z` is
  near ±0.3; expect none sustained
- one-wheel ticks: `inner = |v| − |w|·0.24` on `/cmd_vel`; count ticks with inner in
  (−0.06, 0): expect 0 (ramps may sit in (0, 0.06))
- lag: `/cmd_vel_nav` → `/cmd_vel` → `/mcu/commanded_speed` delay, expect ≤ ~60 ms (was ~100)
- pivot authority: commanded `/cmd_vel.angular.z` vs IMU yaw rate during pivots; the ratio is
  the evidence for or against raising the clamp (item 5)
- goal stops: distance travelled from the first zero on `/cmd_vel_nav` to measured zero

### Test B: emergency and bumper stops (fix 3)

During Test A, at a straight at full speed: step into the path at a safe distance (vision
emergency lane), later nudge the bumper while it moves slowly. Expect the robot to stop
without the old 0.6 s / 9 cm roll-out.

Bag review (`/userdata/ros2/incidents/<ts>_*/`): time from the first zero on
`/cmd_vel_emergency` (or `/cmd_vel_bumper`) to zero on `/cmd_vel`, expect one tick (≤ 50 ms),
and to zero on `/mcu/commanded_speed`, expect the next frame; `/mcu/measured_speed` reaching
zero gives the MCU's own braking time.

### Test C: undock then re-dock (item 4)

Run it **first on the current code** to get a baseline, since the docking controller is
unchanged: undock, drive off a metre or two, dock. Watch the reverse: does one wheel stall
while the other pushes, does the heading oscillate, where does it end up laterally.

Bag review: on `/cmd_vel_docking` during the reverse, `inner = |v| − |w|·0.24`; count ticks
in (−0.06, 0]; marker lateral error at contact (docking status / `final_max_lateral` 0.15).
If stalled-wheel ticks dominate, apply (b) from the table (params only, restart) and repeat;
if steering authority is then too weak to centre, (a) is the code change.

### Test D: angular clamp (item 5)

Only after Test A shows pivots under-rotating. Wheels-up first: set `angular_max` 0.4 and
confirm the wheels spin smoothly at a 0.4 rad/s pivot (0.096 m/s per wheel) without
cogging. Then one short mow of the closest area with the four parameters moved together
(table above). Abort criteria: visible turf scrub or wheel slip in a pivot. Review as Test A,
comparing pivot authority and completion time against the baseline.

## Review loop

For each run: bag directory, what was observed, the metrics above, decision (keep / revert
/ tune). Any revert is a parameter flip plus restart; the code paths all keep the old
behaviour behind their parameters (`turn_shaper_hysteresis_mps: 0.0`,
`slew_both_wheels_always: false`, `hard_stop_lanes: []`, `max_linear_decel: 0.5`).
