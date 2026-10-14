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
| 8 | Pure rotations join the pivot regime: v = 0 with 0 < \|w\| < 0.25 snaps to the 0.25 rad/s pivot and is latched like any other pivot (v = 0 no longer resets the latch and is sign-compatible with either v); \|w\| < `turn_shaper_pivot_deadband_rps` (0.03) → (0, 0). Revert: `turn_shaper_pure_pivot: false` | `cmd_vel_slew.py` `shape_for_both_wheels`, `TurnShaper` | yes (`turn_shaper_pure_pivot`, `turn_shaper_pivot_deadband_rps`); the code itself needs a restart |
| 9 | Dock phases (RETURNING_HOME, LOW_BATTERY_DOCKING, RAIN_DETECTED_DOCKING, COVERAGE_FAILED_DOCKING) shape the nav-driven transit; the shaper is gated on the twist_mux lane: only `/cmd_vel_nav` is shaped, never docking/bumper/emergency/teleop (teleop only with `shape_manual`). Revert: `turn_shaper_dock_transit: false` | `cmd_vel_slew.py` `TurnShaper.is_active`, `NAV_SHAPE_PHASES` | yes (`turn_shaper_dock_transit`); the code itself needs a restart |

Known trade-offs: with 6, the inner wheel may roll forward below 0.06 m/s for the first and
last ticks of an arc (same direction as the outer wheel, so not a one-wheel pivot); set
`slew_both_wheels_always: false` to get the old per-axis slew back. Coupled stops from an
arc are paced by whichever axis hits its decel limit first.

What to look for after 8 and 9 (`wheel_bag_metrics.py`): regime flips at turn entries
(pass ↔ pivot should be gone), ticks with v = 0 and 0 < \|w\| < 0.24 on `/cmd_vel` (expect
0), one-wheel ticks on the nav lane during the return transit (expect 0), the docking-lane
reverse on `/cmd_vel` identical to `/cmd_vel_docking`, and goal stops (the controller's
v = 0, \|w\| 0.03–0.1 tail now turns at 0.25 rad/s and should end the goal sooner).
Trade-off of 8: a held arc may continue for up to `turn_shaper_latch_ticks` (6) ticks when
the controller drops v to 0 at \|w\| < 0.083 rad/s; above that the hysteresis switches to
the pivot at once.

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

`scripts/wheel_bag_metrics.py <mow or incident dir>` computes every metric above from the
pinned bags (regime flips, one-wheel ticks, lag, stop latency and roll-out, pivot authority,
peak accel/decel; `--csv` dumps the 20 Hz series for plotting). A pin is a directory of bare
`*.db3.zstd` splits without `metadata.yaml` (hard links out of the ring, renamed), so the
script reads the sqlite files directly rather than through the rosbag2 reader. Deserialising
needs rclpy, so it runs in the dev image; decompress on the host first unless the image has
`zstd` (added to `docker/Dockerfile.dev-amd64`, present after the next image rebuild):

```sh
scp -r root@192.168.1.105:/userdata/ros2/mows/<ts> /tmp/mow-<ts>
zstd -dkq /tmp/mow-<ts>/*.zstd
docker run --rm -v "$PWD":/work -v /tmp/mow-<ts>:/bag mower:jazzy-dev-amd64 bash -c \
  'source /opt/ros/jazzy/setup.bash && python3 /work/scripts/wheel_bag_metrics.py /bag --csv /bag/series.csv'
```


For each run: bag directory, what was observed, the metrics above, decision (keep / revert
/ tune). Any revert is a parameter flip plus restart; the code paths all keep the old
behaviour behind their parameters (`turn_shaper_hysteresis_mps: 0.0`,
`slew_both_wheels_always: false`, `hard_stop_lanes: []`, `max_linear_decel: 0.5`,
`turn_shaper_pure_pivot: false`, `turn_shaper_dock_transit: false`).

## Run 1: 2026-10-14 mow `/userdata/ros2/mows/20261014-021244` (fixes 1, 2, 3, 6, 7 live)

Timeline (bag seconds): mow 8–1120 with a transit abort at 868 and an emergency stop at 1095;
idle until 2341; 6 s of teleop; second mission from 2357, return + docking 2371–2570 (one
RETRY at 2547, docked at 2570). 43 splits, 2570 s.

| Metric | Result | Verdict |
|---|---|---|
| emergency stop (fix 3) | lane zero → `/cmd_vel` zero 49 ms → MCU 54 ms → measured < 0.01 m/s 453 ms, rolled 0.02 m | fix 3 works (was ~0.6 s / 9 cm) |
| regime flips (fix 1) | 548 changes, 425 within 1 s; 108 pass→pivot, 132 pass→arc, 90 pivot→pass, 150 arc→pass; only 68 arc↔pivot | latch works for arc↔pivot; the hunting is pass↔pivot/arc, see below |
| one-wheel ticks on `/cmd_vel` | 4564 ticks with v = 0 and 0 < \|w\| < 0.24 (both wheels under the 0.06 m/s deadband); 151 counter-rotation ticks on the nav lane during the return transit (t ≈ 2424, v 0.016, w 0.09); 123 on the docking lane during the reverse (v −0.035, w ±0.3) | two gaps, below |
| pivot authority | 0.25–0.30 rad/s commanded, MCU wheel yaw 0.23–0.28, EKF/IMU yaw 0.24 (t 63, 5.7 s) then 0.07–0.17 (ratio 0.24–0.59) | wheels turn, body does not: slip on turf. Raising the clamp (item 5) would not help |
| arcs | 135 episodes, mean radius 0.64 m; 2533 ramp ticks in (0, 0.06) | as designed by fix 6 |
| peak accel | `/cmd_vel` −5.4 m/s² (hard stops), MCU measured +0.80 / −1.22 m/s² | fine |
| lag | cross-correlation says 150 ms but it is dominated by the ramps; sample by sample `/cmd_vel` follows the lane within one 20 Hz tick | metric to be replaced by an edge-based one |
| goal stops | not measurable: after every goal the nav lane keeps sending v = 0 with w 0.03–0.1 for seconds | depends on the slow-pivot fix |

What the shaper actually does at a turn entry (t 51–64): MPPI dithers between v = 0.01 with
w ≈ −0.1 (inner < 0, snapped to a 0.25 rad/s pivot) and v = 0.000 with the same w (a pure
rotation, which `shape_for_both_wheels` passes unchanged and which resets the latch in
`TurnShaper._latched`). `/cmd_vel` therefore alternates 0.25 / 0.10 rad/s every ~0.3 s. The
pure-rotation pass-through dates from the 2026-10-07 heading-hunting observation, before the
latch existed. Minutes 12–13 (t 720–840, the stop-start before the transit abort) are 1300
ticks of v = 0, \|w\| < 0.24: the controller asked for rotations the wheels cannot deliver.

Decisions:
- keep fixes 1, 2, 3, 6, 7 as deployed;
- **next code change (fix 8)**: pure rotations join the pivot regime: v = 0 with 0 < \|w\| <
  floor snaps to the 0.25 rad/s pivot and is latched like any other pivot (the latch, not a
  pass-through, is now the guard against 0.25 pulses near a goal); tiny \|w\| below a
  deadband (≈ 0.03 rad/s) becomes zero;
- **fix 9**: add `RETURNING_HOME` (nav-driven return transit) to `SHAPE_PHASES`; only the
  marker-guided docking/undocking phases stay exempt;
- item 4 confirmed on the docking lane (123 ticks of one wheel forward, one back at
  −0.035 m/s); apply (b) params first;
- item 5 stays parked: the slip ratio says more angular rate scrubs more, the creeping pivot
  assist is the lever.
