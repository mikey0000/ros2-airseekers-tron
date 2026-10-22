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
| 10 | Turn-leg completion cap: a turn leg whose end is reached in xy (pose index at its last pose, or within `turn_leg_skip_m`) is done after `turn_leg_done_after_s` (20 s) or `turn_leg_done_after_rot_rad` (6.28) of accumulated map yaw change, without the yaw test. Leg loop guard: a follow_path whose pose index has not advanced for `leg_loop_abort_s` (90 s) is cancelled and handled as a follow abort (retries). Revert: `turn_leg_done_after_s: 0.0` + `turn_leg_done_after_rot_rad: 0.0` (cap), `leg_loop_abort_s: 0.0` (loop guard) | `mission_fsm.py` `_leg_watch` | restart (mission.yaml; doubles: write `0.0`, not `0`) |
| 11 | Route cusp (reversal > `route_cusp_angle_rad` 2.0, incl. sharp leg junctions): a route follow abort within `route_cusp_radius_m` (0.5) pivots with Nav2 Spin to the route direction after the cusp and re-sends the rest of the route, at most `route_cusp_retries` (2) per cusp, then a transit abort (no costmap planner at a cusp). The `/navigate_to_pose` fallback (off-graph start, abort away from a cusp) is cancelled as a transit abort once the robot is more than `transit_fallback_max_offpath_m` (0.35) from the route / outside the mowing areas. Revert: `route_cusp_retries: 0` (old fallback), `transit_fallback_max_offpath_m: 0.0` (unbounded) | `mission_fsm.py` `_route_cusp_turn`, `_fallback_corridor_tick`; `mission_node.py` Spin client | restart |

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

## Run 2: 2026-10-14 mow `/userdata/ros2/mows/20261014-041213` (fixes 1-3, 6-9 + heading gap/stale fixes live)

37 minutes, 37 splits. Mission: START 04:12:13 UTC, transit, mowing, stuck guard twice, idle
(lifted) at 1804 s, re-start 1961 s, return + dock 1987–2209 s. Owner: "it didn't follow the
path back to the dock at the start either; driving outside areas / paths is bad".

| Metric | Run 1 | Run 2 | Verdict |
|---|---|---|---|
| sustained slow pivots on `/cmd_vel` (v = 0, 0 < \|w\| < 0.24 for > 0.3 s) | 80 runs, 200 s, longest 24 s | 7 runs, 67 s, longest 0.5 s | fix 8 works; the remaining ticks are the 0.25 s ramps through the deadband at every pivot start/stop |
| emergency stop lane zero → `/cmd_vel` / MCU / measured | 49 / 54 / 453 ms | 24-28 / 141-148 / 361 ms, 1 cm | fix 3 holds (two events) |
| bumper stop | – | 51 / 79 / 501 ms, 5 cm | fine |
| pivot assist | – | 127 creeps, 50 held (lethal / vision) | works with the snapped 0.25 pivots |
| time in pure pivots (v < 0.01, \|w\| ≥ 0.2) | 152 s | 593 s (86 episodes at 0.25, 66 at 0.30; longest 27 s, 18 s, 18 s ...) | see the spinning below |
| pivot authority (EKF yaw rate / commanded) | 0.24-0.93 | 0.24-1.2 | unchanged: turf slip |
| output regime changes (classified from `/cmd_vel`, not the status string) | 1657 | 2744 | mostly arc↔straight at \|w\| ≈ 0.02 and the ramp through the deadband; cosmetic. The status-string count in the script is misleading because a legal ≥ 0.25 pivot reads "pass" |
| stereo gaps (new counters) | – | 28 gaps, 5.3 s bridged, longest 0.62 s, 0 dropped | bridging works, but see below |
| stereo − WIT yaw | +52° over the mow | −5° → +12.7° (0-890 s) → −22.7° (2205 s) | both gyros drift, rotation-correlated |
| aligner offset | −74.4 → −7.8 (re-seed) | −73.1 → −95.3, innovations −5…−15°, never positive | the heading error kept growing all mow |

### What went wrong, in order

1. **Transit from the dock did not follow the drawn route (t 4-108 s).** The route is dock
   → 1.1 m west on the dock path → junction → 1.5 m ENE on the Area 1 path (a hairpin). The
   robot pivoted 20 s at the dock (slip), drove the dock-path leg, reached the junction at
   t = 36 facing west with 2 cm cross-track, and MPPI (`FollowPath`) then output 0-0.015 m/s
   and no rotation for 17 s: the progress checker (0.25 m / 0.5 rad in 20 s) aborted it
   ("Failed to make progress"). Nearest lethal cells (nav keepout corridor, half-width 0.35 m)
   were 0.75-1.0 m away on every side but ahead; the local costmap published nothing between
   t = 33 and t = 54. The mission then fell back to the costmap planner (`navigate_to_pose`),
   whose path the robot tracked up to 0.56 m off while pivoting, and which aborted 0.5 m short of
   the goal (transit abort #172); the retry went straight to MOWING. Same fallback again at
   t = 1556 s and "no route (start is off the route graph)" at t = 1985 s.
2. **Seven minutes spinning on a 0.8 m turn leg (t 1140-1576 s).** Sub-path 7 of 27 is a
   Reeds-Shepp turn piece ((−1.09, 3.68) → (−1.36, 4.31), 0.8 m; its neighbours are 0.2-0.5 m).
   `pose_index` reached the leg's last pose at t = 1152 and stayed there; the
   `coverage_leg_goal_checker` (xy 0.08 m, yaw 0.5 rad) and the mission's `turn_leg_skip`
   (0.20 m / 0.5 rad) were never both satisfied with the heading ≈ 10° off and the pivots
   slipping, so MPPI looped: 0.9 m out, U-turn, 0.9 m back, every 37 s, ≈ 4000° of rotation,
   until the stuck guard (4 s pivot, 4.8° of yaw) fired at t = 1576. The mission has no time
   or rotation cap on a leg once the progress checker is satisfied by the looping.
3. **Heading drift is rotation-correlated.** Per minute, Δ(stereo − WIT) ≈ −0.65 % of the net
   rotation (minutes 19-25: +570°/min → −3.7°/min; minutes 3-5: −220…−410°/min → +3°/min),
   and the offset moved −0.4 % of the net rotation vs the GPS course in the laps. The 4000° of
   spinning turned a ~1 % gyro scale mismatch into 20-40° of heading error for both sources.
   The gap bridging (fix A) did its job (28 gaps, none dropped) but the stereo source is not
   the clean one and neither is the WIT. The stale-COG START gate (fix B) was not exercised
   (the saved offset passed the continuity check, source `file_verified` → `cog` within 90 s).

### Decisions / next fixes

- keep fixes 1-3, 6-9 and the heading fixes A, B;
- **fix 10 (mission)**: a turn leg whose end pose has been reached in xy (pose_index at the
  last pose, or within `turn_leg_skip_m`) is done after a cap: ≥ 1 full rotation of
  `/odometry/filtered_map` yaw or ≥ 20 s on the leg, "next leg from here", without the yaw
  test; and the per-leg looping must count as no progress (the leg's own timer, not Nav2's);
- **fix 11 (mission / nav)**: no free-space fallback on a route hairpin. When the route
  follower reports no progress at a cusp, stop, pivot to the next leg's direction with the
  existing turn-in-place path (the docking about-turn already does this), re-send the
  remainder of the route; the costmap-planner fallback stays only for a start off the route
  graph and must be bounded to the corridor (reject a plan whose cross-track to the drawn path
  exceeds the corridor half-width). Also check why the local costmap stopped publishing for
  21 s at the junction;
- **heading**: estimate a per-source yaw-rate scale from the COG turns (gyro Δyaw / COG Δcog
  between consecutive straight windows, both sources) and a bias from the innovation trend;
  until then every long pivot session costs ~1 % of its rotation in heading. The spinning fix
  (10) removes most of the exposure.

## Run 3: 2026-10-14 05:31 UTC, `/userdata/ros2/mows/20261014-053115` (fixes 10/11 live)

29 min. Owner: "in sharp corners it could reverse and turn rather than overshoot the corner,
the turn itself looks good. Nearing the last half it started driving outside the area; going
outside the area is an immediate fail. Docking failed to follow the start of the path again."
**Verdict: FAIL** (left the area; owner rule).

Outside Area 1 and outside every drawn path polygon (EKF map pose vs `areas.dat`):

| When | State | Max outside |
|---|---|---|
| t 37–49 | TRANSIT dock → area (route corner) | 0.85 m |
| t 182–195 | MOWING outer ring | 0.16 m |
| t 504–525 | TRANSIT in-area leg | 0.43 m |
| t 750–764 | TRANSIT in-area leg | 0.33 m |
| t 1252–1316 | TRANSIT 0.6 m leg → bank, roll 24°, STUCK_NEEDS_HELP, owner drove it out | 0.45 m |
| t 1561–1568 | TRANSIT in-area leg | 0.57 m |
| t 1733–1739 | RETURNING_HOME | 0.59 m |

Mowing stays inside (≤ 0.16 m, the outer ring); **every real excursion is a transit**.

Causes, from the bag:
1. **The turn shaper widens the controller's corners.** MPPI asks for slow tight corners
   (v 0.02–0.05 m/s, w 0.1–0.2 rad/s: r 0.2–0.3 m). The inner wheel is in (0, 0.06), so the
   shaper's `_tight_arc` keeps |w| and raises v until the inner wheel is 0.06 m/s: r 0.5–1.0 m
   (`shape_status` "arc r=0.56 … 0.96 m"). Every corner is driven 2–4× wider than asked:
   this is the overshoot the owner saw. Fix 9 (dock-transit shaping) extended it to the return.
2. **Nothing stops a transit outside the area.** `soft_boundary_margin_m` 0.3 acts only in
   MOWING (`mission_fsm` boundary pause); `lethal_boundary_margin_m` 0.5 never fired
   (0 episodes). The fix 11 corridor bound applies only to the costmap fallback, not to the
   MPPI route follow; inside the mowing area it counts as 0.
3. **MPPI transit cannot reverse** (`vx_min 0.0`), so a corner it overshoots can only be
   recovered by a wider loop.
4. **Start**: route abort ("Failed to make progress") at (−3.50, 1.43), 0.75 m past the dock
   junction after the widened corner, beyond `route_cusp_radius_m` 0.5: no about-turn, costmap
   fallback again. **Return**: dock route follow aborted after 150 s at (−0.69, 0.04) →
   "the docking server navigates" → its own `navigate_to_pose` to the approach pose
   (free space, not the drawn path); fix 11 does not cover the docking server's leg.
5. **Fix 10 bug**: the 49 m outer ring ends at the cusp of a reverse leg, so it counted as a
   turn leg; `progress_local` jumped to the last pose while the robot was 4.58 m from the end
   (the nearest-index search matched the ring's own end), the 20 s cap fired and 4.6 m of
   ring were skipped (then a 4.6 m transit across the area).

Also measured:
- Heading was **good**: EKF yaw vs GPS chord median within ±3° all run (offset −96 → −104°
  over 29 min). The WIT drifted −5°/min; the stereo source was right. The divergence WARN
  fired every minute because of the WIT; it should name the drifting source.
- Turn-leg cap fired 3× correctly on short legs; one normal skip. Leg loop guard: no firing.
- Pivots 7662 ticks ≈ 410 s of 1737 s (24 %, criterion < 25 %); sustained slow pivots 5,
  longest 0.6 s.
- Emergency stops 2–53 ms to `/cmd_vel` zero.

Decisions: keep fix 10 (with the bug fixed), keep the cusp about-turn, **revert fix 9 by
param (`turn_shaper_dock_transit: false`) until the shaper choice below lands**, and make
the boundary a hard rule (roadmap task 0.4).

## Run 4 change-set (roadmap task 0.4: the boundary is a hard rule), branch `ws/hard-boundary`

Unit-tested only (dev container); nothing driven yet. Each item reverts by one parameter.

| | Change | Params (default) | Revert |
|---|---|---|---|
| a | Hard boundary: robot centre > `hard_boundary_m` outside areas ∪ drawn paths ∪ dock corridor (`/map_server_node/outside_m`, new) in TRANSIT / MOWING / dock phases → cancel, zero, blade off, `boundary` incident, reverse ≤ 0.10 m/s along the last 1.5 m of EKF poses on `/cmd_vel_emergency` until inside + 0.10 m (both wheels backwards, or a ≥ 0.25 rad/s pivot first), then a transit abort (MOWING: sub-path resumed; dock: re-run). 2nd excursion per mission, no progress, still outside, or no track behind it → STUCK_NEEDS_HELP | `hard_boundary_m` 0.10, `hard_boundary_max_aborts` 2, `hard_boundary_retrace_speed_mps` 0.10, `hard_boundary_trail_m` 1.5, `hard_boundary_retrace_extra_m` 0.10, `hard_boundary_retrace_max_m` 1.0 (mission.yaml) | `hard_boundary: false` |
| b | Turn shaper: a widening-zone command whose r_req = \|v\|/\|w\| < 0.5 × the widened radius snaps to a pure pivot (v 0, \|w\| ≥ 0.25) instead of the 2–4× wider arc; latch holds across ratio ± 0.1 | `turn_shaper_pivot_instead_ratio` 0.5, `turn_shaper_pivot_instead_hysteresis` 0.1 | `turn_shaper_pivot_instead: false` |
| c | MPPI transit `FollowPath.vx_min` −0.10 (PreferForwardCritic kept) | nav2_params.yaml | `vx_min: 0.0` |
| d | Route split at every corner ≥ 79° (2026-10-15 fix, was > 90°), and before a route leg (transit or dock route) ≥ 79° off the robot's yaw: stop, Nav2 Spin to the leg heading, then follow; the leg into a split corner ends on it (`coverage_leg_goal_checker`); `route_cusp_radius_m` 0.5 → 1.0 | `route_corner_turn_rad` 1.38, `route_corner_goal_checker_id` coverage_leg_goal_checker | `route_corner_turn_rad: 0.0` (+ `route_cusp_radius_m: 0.5`); run 4 threshold: `1.5708`; checker: `''` |
| e | Docking server only within 0.35 m of the approach pose; farther, the dock route is re-planned and driven again (2×), then NAV_TO_DOCK_FAILED in place; obstacle aborts wait first | `dock_server_nav_max_m` 0.35, `dock_route_retries` 2, `dock_approach_distance_m` 0.8 | `dock_server_nav_max_m: 0.0` |
| f | Turn-leg cap: end reached only within `turn_leg_skip_m` of the leg end (never `progress_local`), capped only if the leg's own length ≤ `turn_leg_max_m` | — | `turn_leg_end_by_distance: false` |

What to look for in the bag / log:
- `/map_server_node/outside_m` (now recorded): **pass = no sample > 0.10 m in a full mow**.
  Every `HARD BOUNDARY` log line and `boundary` incident is a fail of the run (each one
  pins the bag); check the retrace on `/cmd_vel_emergency`: linear ≤ 0 and ≥ −0.10, no tick
  with the inner wheel in (−0.06, 0.06) (`wheel_bag_metrics.py` one-wheel count on the
  emergency lane), and `boundary: back inside after reversing … m` in the log.
- Corners: `/cmd_vel_slew/shape_status` should show `pivot … instead of an arc` where run 3
  showed `arc r=0.56 … 0.96 m`; no arc r > 0.5 m on transit corners; route corners within
  0.15 m of the route (EKF pose vs the plan). Log: `route corner at (x, y): stop and pivot`.
- MPPI reverse: `/cmd_vel_nav` linear < 0 should be rare and short (backing out of an
  overshoot, ≥ −0.10).
- Docking: **no `navigate_to_pose` goal from the docking server farther than 0.35 m from the
  approach pose**; `dock: … driving the dock route again` instead; the approach / about-turn
  / marker reverse unchanged.
- Rings: no `turn leg … leg done` line for a sub-path longer than 1.2 m.
- Watch: Spin refusing at a corner (the leg is then sent anyway: `route corner pivot …:
  following the leg anyway`); a hard-boundary trigger at the dock if the dock corridor is
  missing (dock pose not set); with (b) in place fix 9 (`turn_shaper_dock_transit`, reverted
  after run 3) can be re-tested.

### Sim fixes before the field test (2026-10-15, branch `ws/hb-fixes`)

A sim run on a copy of the real map (`docs/simulation.md`, `areas_file`) found two bugs in
the change-set above. Both are fixed, unit-tested and re-run in the sim (domain 173,
`localization:=sim`, `nav2_accdefault.yaml` because of the MPPI accel livelock).

- **(a) the retrace declared "back inside" while still outside.** When the reversed
  distance reached the recorded track length it resumed if `outside_m <= hard_boundary_m`
  (sim: "back inside after reversing 0.40 m" at 0.066 m outside, then a resume from there).
  Now success needs a fresh `~/outside_m` ≤ 0 and `hard_boundary_retrace_extra_m` reversed
  past the re-entry point, or inside when the track / `hard_boundary_retrace_max_m` ends
  first. Still outside at the track end, or no fresh `outside_m`, stops in
  STUCK_NEEDS_HELP (`retrace ended still 0.033 m outside after reversing 0.25 m ...`); never
  a resume, never a reverse beyond the recorded track. No new parameter (revert of the
  whole retrace: `hard_boundary: false`). Log on success:
  `boundary: back inside after reversing 0.34 m (0.10 m past the re-entry, excursion n/N)`.
  Sim: 3 natural trips on the outer ring all ended 0.10–0.11 m past the ground-truth
  re-entry and resumed; with a forced 0.2 m trail, one trip ended inside at the track end
  (resumed) and one 0.031 m outside (STUCK_NEEDS_HELP, no resume). Emergency lane linear
  −0.10..0, no one-wheel tick, blade off throughout.
- **(d) drawn 90° corners were not split.** The split compared single 3-pose chords with
  `> pi/2` (the dock path's drawn 90° corner measures 89.99°), and after a cut skipped 6
  poses, so a second corner within ~0.5 m was missed. The leg before a split corner ended
  ~0.2 m short (`general_goal_checker` xy 0.25 m), and the pivot happened there.
  Now `geometry.route_corners` measures each bend between the straight runs either side
  (Douglas–Peucker 0.02 m: pose spacing and drawing wobble do not matter, the cut is the
  vertex pose); every bend over the threshold is its own split however close the next is;
  same-way smaller bends within 0.3 m add up to one corner; inclusive with 1°. Threshold
  1.38 rad (79°): an arc of the 0.5 m minimum turn radius cuts a corner of 79.5° by the
  0.15 m pass limit. The leg into a split corner arrives with its run's yaw and uses
  `route_corner_goal_checker_id` (`coverage_leg_goal_checker`: 0.08 m, or after the
  closest approach within 0.20 m). `route_corner_turn_rad: 0.0` still selects the old split.

Sim, real map, dock → Area 1 and RETURNING_HOME (max deviation of the ground truth from the
planned route within 0.4 m of each drawn vertex while the route was driven; before = run r7
of the first sim pass, after = two runs):

| Route | Vertex | Bend | Before | After | Stop / pivot after |
|---|---|---|---|---|---|
| dock → area | (−0.14, −0.17) | −90° | 0.19 | 0.06 / 0.07 | stopped 0.07–0.08 m from the vertex, pivot −90° (before: no split) |
| dock → area | (−0.33, 0.12) | +42° | 0.20 | 0.07 / 0.07 | not split (arc) |
| dock → area | (−2.26, 0.37) | −39° | 0.15 | 0.24 / 0.24 | not split: MPPI cuts it |
| dock → area | (−3.25, 1.12) | −111° | 0.20 | 0.11 / 0.11 | stopped 0.08 m from the vertex (before 0.24), pivot −130° |
| dock → area | (−2.15, 1.66) | −57° | 0.22 | 0.20 / 0.20 | not split: MPPI cuts it toward the leg end |
| area → dock | (−3.25, 1.12) | +86..+120° | 0.20 | 0.10 / 0.13 | stopped 0.03–0.08 m from the vertex (before 0.25), pivot |
| area → dock | (−2.26, 0.37) | +39° | 0.29 | 0.27 / 0.34 | not split: MPPI cuts it |
| area → dock | (−0.33, 0.12) | −42° | 0.16 | 0.13 / 0.10 | not split |
| area → dock | (−0.13, −0.18) | +90° | 0.16 | 0.13 / 0.11 | split, but the robot reached it 66° off the next leg (it had cut the −42° bend), under 79°: no pivot, MPPI arc |

Every split corner is now within 0.15 m. The remaining fails are the 39° and 57° bends,
which MPPI cuts on the long dock-path leg (0.20–0.34 m); that is controller tracking, not
the split (a pivot at every 40° bend would cost far more pivot time). Docking: both runs
ended `DOCK_MAXOUT` after three RETRYs at ~0.3–0.4 m along / +0.06–0.10 m lateral / +26–30°
heading. The first sim pass (r7) shows the same RETRY pattern and docked on its third
attempt, so this is the sim docking approach, not the route change; worth a look before
the field test.

What to look for in the run 4 bag in addition: `route corner at (x, y): stop and pivot` at
every drawn ~90° corner, the robot stopped within ~0.08 m of the corner before it (EKF
pose vs the route vertex); `boundary: back inside … (0.10 m past the re-entry …)` or
`retrace ended still … outside` (then STUCK_NEEDS_HELP), never a resume while
`/map_server_node/outside_m` > 0.


### 2026-10-15 field: undock donut and two corner stucks (branch `ws/followups`)

**Undock donut (first run with the run 4 change-set).** After undocking the start route was
split at its first corner: `route corner at (0.14, -0.28): stop and pivot 111 deg (1/2)`.
That first leg was the 0.17 m connector plan_route draws from the robot to the dock path
(heading −146°), which turns 90° onto the path at (0.0, −0.38): `route_corners` did split
there (0.17 m ≥ its 0.15 m arm), so the robot pivoted to the stub, overshot to −178°, and
MPPI got a 0.17 m path ending in a 90° hook, with the leg goal checker wanting 0.08 m and
the stub's yaw. For 82 s MPPI output |v| ≤ 0.03 m/s with |w| 0.05–0.3, the turn shaper made
each tick a 0.25 rad/s pivot, and the robot circled at ~(0.3, −0.6) until STOP. The
PoseProgressChecker counts rotation, so Nav2 never aborted (only the 300 s transit timeout
would have). 1e19c15 (MPPI default accel limits) fixed the main sim cause.

**Run mow-040433, STUCK1 / STUCK2 (after 1e19c15).** Both at the Dock path / Area 1 junction
area, both a route leg into a split corner that never ended. STUCK1 (TRANSIT): the robot's
0.36 m connector to the dock path (47°), then 90° onto it; the robot got within 0.07 m of
the join at 86° (39° off the connector's yaw, LegGoalChecker yaw 0.5 rad), so the leg never
ended and MPPI turned / dithered in place until the stuck guard. STUCK2 (RETURNING_HOME):
the dock route was split at (−3.25, 1.12) (`/plan` index 13, +106°), the robot stopped
0.12 m short of it at −147° and MPPI turned in place for ~12 s (v −0.06..+0.02, w
0.05–0.18). In both cases no `route corner` pivot was logged because the pivot comes only
after the leg into the corner succeeds. Shaper check: MPPI's w kept one sign throughout
(no overshoot-and-correct cycle), the robot turned at ~0.1 rad/s against a commanded 0.25
(slope / turf), and the reverses are MPPI's own v < 0 (vx_min −0.10): in the field the
shaper's 0.25 rad/s snap amplified MPPI 2.5× but did not drive the loop (the unended corner
leg did). The sim (no slip) does show the amplified loop: run r4's costmap fallback asked
~0.1 rad/s, got ±0.25 pivots, overshot, MPPI reversed w, and the robot rocked ±15° on the
spot for 20 s (ended by the new progress watch). So a flag, **off by default**:
`turn_shaper_transit_pivot_hold_rps` (cmd_vel_slew; e.g. 0.2): in TRANSIT / nav-lane dock
transit a command the shaper would snap to a pivot while |w| asked < this is held at (0, 0);
pivots stay ≥ 0.25 rad/s, the mission's corner rule / progress watch then decide. Sim r5
with 0.2: transit unchanged (3 holds logged, corner Spins unaffected).

| | Change | Params (default) | Revert |
|---|---|---|---|
| h | Route stubs: a corner (≥ 79°) within 0.3 m of a route end cuts the stub off (the route starts / ends at the corner, so the pre-leg pivot turns to the heading after it); the first leg joins at the robot's projection onto the first 0.5 m, never at a start behind it | `route_stub_trim_m` 0.3, `route_start_join_m` 0.5 | `0.0` each |
| i | Route legs (transit + dock route) name a translation-only progress checker (`transit_progress_checker`, SimpleProgressChecker 0.25 m / 20 s); coverage follows and the Nav2 BTs keep `progress_checker` (rotation counts). Jazzy needs an id per FollowPath goal once two are loaded | `route_progress_checker_id`, `follow_progress_checker_id` | `route_progress_checker_id: progress_checker` |
| j | Mission route-leg watch: the robot's projection onto the leg must advance 0.15 m in 20 s, else cancel and handle as a FollowPath abort; a corner leg that aborts within 0.25 m of its corner (no obstacle) counts as reached; a `/navigate_to_pose` transit (costmap fallback) must get 0.15 m closer to its target in 20 s, else transit abort | `route_progress_window_s` 20, `route_progress_min_m` 0.15, `route_leg_end_accept_m` 0.25 | `route_progress_window_s: 0.0`, `route_leg_end_accept_m: 0.0` |
| k | Corner legs end by the mission, yaw ignored: within 0.08 m, or within 0.15 m once the distance stopped shrinking for 2 s; then the corner pivot | `route_corner_xy_m` 0.08, `route_corner_reach_m` 0.15, `route_corner_stall_s` 2 | `route_corner_stall_s: 0.0` |
| l | At a split corner (legs after the first) the pre-leg Spin triggers at 0.6 rad (34°) instead of 79° (MPPI turn-ins are the dither regime) | `route_leg_pivot_min_rad` 0.6 | `0.0` |
| m | (off) Transit pivot hold in the shaper: a would-be 0.25 pivot for a request < X rad/s is (0, 0) in TRANSIT / dock transit | `turn_shaper_transit_pivot_hold_rps` 0.0 | already off |

Unit tests (`test_route_followups.py`) replay the real `/plan`s: the donut route
(`undock_spin_2026-10-15_plan.json`: no pivot, first leg starts on the corner heading 124°,
was a 111° pivot onto the stub), all 8 run 4 plans (`run4_2026-10-15_plans.json`: no leg
begins with a ≥ 79° turn within 0.3 m, every non-last leg ends on a corner), STUCK2 (leg
ends after 2 s at 0.12 m, Spin +106°) and STUCK1 rebuilt (leg ends at 0.07 m, Spin ~51°).

Sim (real map, domain 174, committed nav2 params, `localization:=sim`): r1 docked start:
undock → route (3 legs) → Spin 112° at the approach pose → first FollowPath 7 s after the
route; every split corner stopped 0.07–0.08 m from its vertex, deviation at the pivot ≤ 0.04 m,
no spinning; −39° / −57° bends still cut 0.21–0.29 m (MPPI tracking, unchanged). r2 / r5
(start at the run 3 post-undock pose): first FollowPath 1.7 s after the route, corner legs
ended by the mission rule (`corner … reached (0.08 m)`). r3 / r4 forced spin (an extra
50 Hz `/cmd_vel` of 0.3 rad/s): the route leg aborted by `transit_progress_checker` 20 s
after the robot stopped advancing, then the costmap fallback (rotation-counting checker)
spun / rocked until the new mission watch on it aborted it 20 s later and the transit retry
drove the route to the area.

What to look for: `route: … stub before the corner … dropped`, `route: first leg joins at`,
`route leg i/n: corner (x, y) reached (d m): leg done`, `route corner at …: stop and pivot`
right after it; any `route progress watch` line is a leg that turned in place for 20 s (a
fail to investigate, but it no longer spins for minutes).


## Run 5: 2026-10-15 05:17, `mow-051745` (follow-ups live), and the path conditioner (branch `ws/coverage-corners`)

**What happened.** Undock and the start transit worked. On sub-path 0, the outer ring, the
ring start already arced 0.062 m outside Area 1 at (-2.09, 1.10) (FollowCoveragePath rotated
part way from the transit's ~30 deg to -85 deg, then an arc to -160 deg). 20 s later RPP drove
the ~100 deg V corner at (-4.74, -1.17) as an arc: 0.26 m cross-track, the robot centre 0.11 m
outside. The hard boundary stopped, blade off, retraced 0.40 m and resumed the same leg the
same way: the second trip ended in STUCK_NEEDS_HELP 47 s into mowing. MOWING cross-track
against the planned lines: median 0.026 / p90 0.074 / p99 0.204 / max 0.262 m, every large
error at a sharp corner.

**Cause.** Coverage rings are the area polygon offset inward (0.05 m) and keep every drawn
vertex; drawn paths are polylines with sharp vertices. A vertex cannot be driven without
stopping; RPP arcs wide (design `DESIGN_path_conditioning_2026-10.md` section 1).

**Sim gate (new, `scripts/sim_gate.sh`, `simulation.md` "Sim gate").** A full mow on a copy of
run 5's map. On the deployed code (9c2fdd1) it reproduces the failure mode with and without
the ground disturbance: two hard-boundary trips on the outer ring (at its start, the spot the
field run missed by 4 cm), the second after resuming the same arc, STUCK_NEEDS_HELP ~30 s
into mowing.

**First fix, dropped.** A coverage-only split at corners >= 79 deg plus a boundary-retry
special case (resume at the corner with a pivot). One sim run: no trip, but replaced by the
owner's direction (design 2026-10): handle corners once, in one path conditioner, not with
another runtime rule. Not committed.

**Change (design steps 2-3, `path_conditioning`).** `mower_mission/path_conditioning.py`
conditions every coverage sub-path except the turn legs, and every transit / dock route,
against the allowed region (areas U path channels U dock corridor): per corner the largest
fillet r in [0.5, 1.0] m that fits, else a pivot at the vertex; outermost-ring convex corners
>= 79 deg at the edge pivot (edge cut); resampled at 0.05 m with segment types; validated.
Pivots run as a leg end on the vertex (the existing corner-reached rule) + Nav2 Spin, blade
on. A coverage leg whose join heading is > 0.6 rad off is turned to first (the ring start).
On run 5's ring: the V (-4.74, -1.17) and (-0.50, 5.32) pivot by the edge policy, 18 corners
are fillets (r 0.5-1.0), the rest pivots where no fillet fits. The dock path's 39 deg bend is a
r = 1.0 fillet; its drawn 90 deg corner by the dock pivots.

| | Change | Params (default) | Revert |
|---|---|---|---|
| n | Path conditioner on coverage sub-paths (not turn legs) and transit / dock routes; pivots as leg end + Spin; `~/mow_plan` types, `~/conditioned_route` | `path_conditioning` true, `path_conditioning_r_min_m` 0.5, `_r_max_m` 1.0, `_margin_m` 0.05, `_half_width_m` 0.24, `_slack_m` 0.05, `_pivot_min_rad` 0.35, `_edge_pivot_rad` 1.38, `_edge_clearance_m` 0.20 | `path_conditioning: false` |
| o | Coverage leg join pivot (> this off the robot's yaw, not on turn legs / swath-end turns) | `path_conditioning_start_pivot_rad` 0.6 | `0.0` |

**Sim gate results** (full table in `simulation.md`): with conditioning, ideal and disturbed,
0 samples outside, 0 hard-boundary trips, every sharp corner <= 0.114 m (V corner 0.038 m),
no one-wheel ticks (except the docking server's marker reverse, 114 ticks, in the one run that
docked), pivot share 23-26 %. **The gate does NOT pass**: in 2 of 3 runs the 0.54 m swath-end
turn leg (sub-path 3, sent as planned) circles under the shaper's widened r = 0.64 m arc,
"Failed to make progress" x4, NOT mowed, MOWING_INCOMPLETE; cross-track p99 0.22-0.38 m comes
from turn legs and leg joins (bodies: median 0.004-0.013 m). Runtime rules that still fired:
route corner pivots 7-8, route corner-reached 1-2, stub trim / start join 1-2, fix 10 turn-leg
cap 2-4, shaper widened arcs 155-460 and pivot-instead 15-36, pivot-assist creep 6-11; no
costmap fallback, no progress watch, no loop guard, no stuck guard. Not for the field yet:
next is the turn legs (condition / track them with the same rules, or the shaper as a clamp,
design steps 4-5).

What the sim cannot show: slope (the field's Spin timeouts on the slope, pivot vs K-turn),
real turf slip and grip in pivots beyond the disturbance model, RTK float / heading error.

## Runs 6 and 7: 2026-10-16, path conditioning + MPPI everywhere, then the goal-drift fix

**Run 6** (`20261016-030725` .. `-031918`, master f84420b): the robot drove straight through
pivot vertices (passing 0.03 m from them) and tripped the hard boundary 4 times. Cause:
`LegGoalChecker` reset its leg whenever the goal moved > 1 mm; controller_server re-transforms the
map goal into odom each cycle and map->odom moved > 1 mm on 97 % of 10 Hz steps (median
5.4-5.7 mm), so the travel gate never opened. The sim had a static map->odom. Also a -111 deg
route pivot on the dock-path slope timed out ("aborted") and the leg was followed anyway.
Fixed on `ws/goal-drift` (f257a15): drift-tolerant goal checker, pivot retries, sim map->odom
drift, resume scenario in the gate.

**Run 7** (`20261016-054318`, master f257a15): first complete mow with no intervention.
- Area 0: 9/9 sub-paths, 0 not mowed; 26/26 pivot vertices pivoted (median 0.05 m from the
  vertex when the pivot started; worst 0.34 m at the 0.09 m-apart double vertex (-2.10, 1.63) /
  (-2.18, 1.68)); docked first try 18 s after the dock goal.
- Boundary: 0 m outside while mowing and in transit; max 0.076 m (10 samples > 0.05) on the
  return route at (-1.7, 0.1); 0 hard-boundary trips.
- Cross-track while mowing (vs the conditioned plan, EKF): median 0.042, p90 0.072, p99 0.089,
  max 0.098 m (run 5 RPP: 0.026 / 0.074 / 0.204 / 0.262). Arcs p99 0.091.
- **Open: one-sided offset.** On straights 88 % of samples are right of the direction of travel,
  median -0.040 m. It follows the robot, not the map, so it is a vehicle-frame bias (wheel speed
  bias with MPPI's proportional-only correction, or a lateral GNSS lever arm), not noise; with
  zig-zag lanes it narrows / widens alternate lane gaps by ~8 cm.
- MPPI commands in the wheel-floor band: mowing 2.0 %, transit 2.9 %, return 1.5 % (field before
  the critic: 24-84 %). controller_server: 28 "missed its desired rate" (min 11.9 Hz) in 18 min,
  0 "Optimizer fail".
- Keep: everything. Next: find the 4 cm right bias (one change, sim first).

## Field mow 2026-10-22 morning: session `20261021-202110` (20:21-21:37 UTC, Area 6)

Data: `ros2_stack-wt/accuracy-simout/field_mow_2026-10-22/` (bag, incidents, node logs,
`wheel_cal*.txt`, `accuracy.txt`, `wheel_metrics.txt`). Splits 1, 9, 29 and 37 are corrupt on the
mower (2 truncated, 2 not zstd); 73/77 read. Split 9 holds the 20:30:21 trip (log only).

**Code that actually ran: ec8be43, not 6117732.** `mower_jazzy` and every node process started
2026-10-21 08:23 UTC (ec8be43 was synced at 08:20 UTC). The 6117732 sync (18:34 UTC) and the 7fb699e
sync (21:09 UTC, mid-mow, `dirty:5`) only replaced files under `/userdata/ros2_stack`; nothing was
restarted, so the alpha-beta heading aligner (a5c6a14), docking2 and the drive-load detector were
NOT live. The aligner log has no drift-rate field (old code). DEPLOYED_REV now names 7fb699e while
the processes run ec8be43: any node respawn would load a mix. A deploy must restart the stack.

### Outcome

| | |
|---|---|
| session | 75.8 min: MOWING 56.8, TRANSIT 6.9 (incl. 7 detours), STUCK_NEEDS_HELP 10.2, manual 0.6, other 1.3 |
| plan | Area 6, cross pattern, 5 sub-paths 514.7 m; sub-paths 0-2 done (88.3 + 420.4 + 2.4 m) |
| not done | sub-paths 3-4 (~3.6 m) and 7 skipped detour stretches (~2 m each, "re-tried at the area end", never reached) |
| stops | 8 NEEDS_HELP (4 boundary retrace, 4 coverage Spin), 9 dynamic-obstacle stops (2 person, 7 "cat"), ~2.3 min waiting |
| end | owner STOP button 21:36:54 in TRANSIT to sub-path 4/5 (route-corner pivot at (-30.96, -12.65)); no dock attempt, robot then driven manually / boundary recording |

### Boundary trips (all MOWING, sub-path 0 = the edge ring, base_link-only 0.10 m check)

| UTC | base_link pose, yaw | out (base_link / worst nose corner) | what happened | recovery |
|---|---|---|---|---|
| 20:25:39 | (-27.80, -15.79), -157° | 0.11 / 0.71 | S corner; stop_zone halt 20:25:31-34, then MPPI arc (w +0.23) out of the ring at -0.07 m | retrace 0.30 m OK; then follow_path abort (costmap) -> detour, 2 m skipped |
| 20:26:24 | (-31.32, -13.02), 107° | 0.11 / 0.32 | W corner; rode the line (0 to +0.04 m) for ~2 m after the detour rejoin, then the corner arc (w -0.29) | retrace capped at 1.00 m, still 0.070 out -> NEEDS_HELP |
| 20:28:33 | (-31.15, -13.37), 97° | 0.10 / 0.17 | operator START while 0.07 out: resume leg-start pivot (-30°) pushes base_link over 0.10 | retrace 1.0 m, 0.091 out -> NEEDS_HELP |
| 20:29:30 | (-31.15, -13.37), 118° | 0.10 / 0.39 | same, during the resume pivot | 0.075 out -> NEEDS_HELP |
| 20:30:21 | (-31.10, -13.45) (log) | 0.10 / - | same, during the resume pivot | 0.093 out -> NEEDS_HELP; operator drove it in manually |
| 20:34:39 | (-26.96, -2.10), 59° | 0.11 / 0.65 | NE corner after 2 costmap detours (trunk/wood at 0.5 m); corner arc then a pivot moved base_link +0.04 -> +0.10 | retrace 0.32 m OK |

Known class: the old (non-stock) layout keeps the edge ring only ~0.10-0.15 m inside, the ring
weaves (w saturating at ±0.30), and corner arcs/pivots carry base_link over the 0.10 m line. No row
end tripped (sub-path 1, 420 m: 0 trips). New: (a) the 1.0 m retrace cap fails when the robot rode
the line for longer than that; (b) START after a boundary NEEDS_HELP resumes in place while still
outside and the leg-start pivot re-trips at once: 3 of the 4 boundary NEEDS_HELP were this loop.

### Heading lead (A2): control run, the fix was not live

| | 21c evening | this run (3 m straights, n=19) | this run (2 m, n=33) |
|---|---|---|---|
| fused yaw - course | +2.37 ± 0.51° | +1.28 ± 0.66° | +1.07 ± 0.55° |
| paired | +2.37 ± 0.79° | +2.10 ± 1.73° (n=3) | +1.32 ± 0.89° (n=10) |
| lane-gap offset (right of travel) | 1.64 ± 0.62 cm | n/a (3 pairs) | 1.26 ± 0.74 cm (n=10) |
| aligner COG innovation mean | -2.41° (sd 2.53) | -1.11° (sd 2.57, n=1227) | |
| aligner offset drift | -4.4..-4.8°/min first 40 min | -0.93°/min mean (-105° -> -175°), peak ~-4°/min 20:47-20:58 | |
| ground / MCU speed | 1.049 | 1.059 | |
| pivot measured/commanded CCW / CW | 0.748 / 0.806 | 0.736 / 0.713 | |

The smaller lead matches the slower gyro drift this morning (A2 fit: lead ≈ 0.74° per °/min of offset drift),
not a fix. Target (lead < 0.5°, offset < 1 cm) not met and not testable until the stack restarts.

### Obstacle stops and turns

| per hour (1.26 h) | 21c | this run |
|---|---|---|
| collision_monitor stop_zone | 0 | 11 episodes (22 lines), 8.7/h |
| Spin "Collision Ahead" | - | 13 aborts in 5 pivots (3.2 NEEDS_HELP-producing/h); the 2 retries fire in the same second |
| false stuck (slip_detector) | - | 0 (132 slip blips, all cleared in 0.1 s) |
| NEEDS_HELP | 2 | 8 (6.3/h) |
| MPPI "Optimizer fail" | - | 370 (clusters at 20:25, 20:33-34) |

Spin aborts (21:12, 21:18, 21:29, 21:31, 21:32) are row-end pivots at the W/S edges with 17-34 lethal
local-costmap cells within 0.9 m, mostly just outside the area line, inside the 0.64 m sweep.
Turns (`accuracy.py field`, 103 pivots, EKF): pivot drift median 0.054 m / p90 0.144; heading error
median -0.7° (29/103 turned too far; run 7: 46/46 too far); leg-end lateral median 0.040 / p90
0.120 m; outliers > 0.4 m are pivots interrupted by obstacle waits. One-wheel ticks: 321 inner in
(-0.06, 0) and 999 in (0, 0.06) of 68 480 moving ticks.

### Safety / new

- Blade stayed ON at every person (0.5-0.6 m) and cat (0.4-0.9 m) stop (`blade_policy continuous`).
  Still open from 21c; living-obstacle stops must force the blade off.
- "cat" detections that never moved for 30 s recur at the N row ends around (-27..-28, -4..-5), the
  21c obstacle spot next to the "trunk": possibly a misclassified static object; unverified.
- supervisor blackbox writes fail (`incompatible constructor arguments`, 0 messages) at every event.
- Recorder dropped at the start (20:21:10, back in 7 s) and at 21:37:08.

Next: restart the stack on the intended rev and re-run (the A2 validation is still owed); fix the
resume-while-outside loop (re-enter before any pivot) and the retrace cap; force blade off for
living obstacles.

## Field mow 2026-10-22 midday (1b862ad): session `20261021-230412` (23:04-01:08 UTC, Area 7)

Data: `ros2_stack-wt/accuracy-simout/field_mow_2026-10-22b/` (bag, node logs, incident metadata,
`wheel_cal.txt`, `accuracy.txt`, `wheel_metrics.txt`, `series.json`, `extra.json`). Splits 37, 45,
86 and 94 are corrupt on the mower (60 s gaps at 23:41, 23:49, 00:30, 00:38); 121/125 read.

**Code that ran: 1b862ad.** `mower_jazzy` started 23:03:04 UTC; `stereo_cam` opens the front camera
as YUYV and `/heading_aligner/status` carries `offset_rate_deg_min`. Still the OLD coverage layout
(`ws/edge-layout2` not merged). The four `mission_boundary` incidents of the day (21:47:52 0.10 m
MOWING, 22:17:04 0.13 m TRANSIT, 22:42:53 2.94 m and 22:43:29 10.27 m MOWING; sessions 214524,
221614, 223745) all belong to the earlier, pre-restart runs and are not analysed here. The recorder
incidents at 23:04:17 and 01:08:42 are the usual session-start false alarms.

### Outcome

| | |
|---|---|
| area | Area 7 (index 5), cross pattern, 2 runs |
| run 1/2 | 23:04:44-00:22:48: 5/5 sub-paths, 766.0 m, 100 % |
| run 2/2 | 00:22:57-01:08:28: 13 sub-paths, 446.2 of 765.5 m (58.3 %), stopped in sub-path 10/13 |
| total | 1212 of 1531 m (79 %); MOWING 120.1 min, TRANSIT 3.8 min (7 transits), PLANNING 0.3 min |
| stops | 0 NEEDS_HELP, 0 skips, 1 dynamic stop (person, left camera, 2 s) |
| end | 01:08:28 battery 14 % < 15 %: stop in place (`battery_low_action=stop`), resume cursor saved; RETURNING_HOME only at 01:10:29 from an external `high_level_control(2)` (dock run: separate analysis) |

### Heading lead (A2): the fix works

| | 21c evening | 22 Oct morning (old code) | this run (1b862ad, 3 m straights) |
|---|---|---|---|
| fused yaw - course | +2.37 ± 0.51° | +1.28 ± 0.66° (n=19) | **-0.07 ± 0.21° (n=65)** |
| paired lanes | +2.37 ± 0.79° | +1.32 ± 0.89° (2 m, n=10) | **+0.03 ± 0.11° (n=29)** |
| lane-gap offset (right of travel) | 1.64 ± 0.62 cm | 1.26 ± 0.74 cm | **0.25 ± 0.19 cm** (swath 0.179 m) |
| aligner COG innovation mean | -2.41° (sd 2.53) | -1.11° (sd 2.57) | **+0.01° (sd 2.04, n=1459)** |
| aligner offset drift | -4.4..-4.8°/min | -0.93°/min mean | -38° -> -155° -> -41°; rate state -4.2..+4.7°/min, \|rate\| > 2°/min half the time |
| ground / MCU speed | 1.049 | 1.059 | 1.063 |
| pivot measured/commanded CCW / CW | 0.748 / 0.806 | 0.736 / 0.713 | 0.804 / 0.814 |

Both targets met (lead < 0.5°, offset < 1 cm) while the gyro offset drifted harder than on any
earlier run, so the drift-rate state (`rate_beta` 0.005) is doing the work. The rate shows in
`/heading_aligner/status` (`offset_rate_deg_min`), not in the node's text log lines.
wheel_cal also flags ground/commanded 1.021 (`linear_scale` 0.980 suggested) and a significant
yaw trim +0.0062 ± 0.0018 rad/s (right/left 1.011): candidates, not applied.

### Edge, boundary, obstacles, turns

| per run (2.07 h) | morning run (1.26 h) | this run |
|---|---|---|
| boundary trips / START-while-outside / retrace-cap fails | 6 / 3 / 1 | 0 / 0 / 0 (max `outside_m` 0.044 m, never lethal) |
| Spin "Collision Ahead" aborts | 13 | 0 (217/217 spins succeeded) |
| collision_monitor stop_zone | 11 episodes | 0 (11 slowdown episodes) |
| false stuck (slip_detector) | 0 | 0 (188 blips, all cleared) |
| dynamic stops | 2 person, 7 "cat" | 1 person (2 s), 0 cat |
| MPPI "Optimizer fail" | 370 | 1 |

Front camera (YUYV): no "cat" at all. New class: "hedgehog", 3 static policies plus 10 "obstacle
close" warnings at 0.41-0.77 m, mostly at the NE leg start (-12.7, -3.5); static, so no stops. Raw
detections (1 Hz sample): stone 488, wood 475, hedgehog 64, sports ball 35, shovel 25, person 19,
dog 2. No segmentation topic is bagged, so the grass-segmentation change cannot be judged.

Turns (`accuracy.py field`, 215 pivots): pivot drift median 0.037 / p90 0.104 m (morning 0.054 /
0.144); heading error median -0.4° (83/215 too far); leg-end lateral swath 0.024 / p90 0.084,
connector 0.098 / 0.142 m. (Transit rows include the dock route after 01:10 and are not mow data.)
One-wheel ticks: 0 inner in (-0.06, 0) (morning 321) and 1017 in (0, 0.06) of 141 786 moving ticks.

### Health and battery

- No OOM (`dmesg` clean since the 22:14 reboot), no node restarts (supervisor: only the recorder's
  session-start respawn), RTK fixed 100 %, localization `ok` and tilt `ok` throughout. CPU/memory
  are not bagged; Nav2 controller missed 20 Hz 344 times (2.8/min, 18 below 10 Hz; morning 3.3/min).
  At 01:30 UTC the host had 291 MB available and load 7-10.
- Battery 22.9 V / 59 % -> 19.4 V / 14 % over 124 min (0.36 %/min, ~3 A). The 15 % threshold stops
  in place; it does not return home by itself.

Next: decide whether low battery should return home automatically; re-check the hedgehog spot; the
yaw trim and `linear_scale` candidates; edge-layout2 field run.

## Field mow 2026-10-23 morning (00c041d)

Sessions `20261022-201535` and `20261022-202312` (20:15-20:39 UTC), area index 5 ("Area 7"),
edge-layout2 + mem-diet + obstacle hotfix. Owner report: "vision can see trunk, which it should
just go around and not get stuck". Data: `ros2_stack-wt/accuracy-simout/field_trunk_2026-10-23/`
(bags, incidents, node logs with ntrip lines removed, `maps/`, `dump.json`).

**Trunk position.** 888 of the front-camera (`vio_camera`) ranged trunk/wood detections with a
score of 0.5 or more fall in one 0.1 m cell, map (-6.2, -0.6). The best estimate of the trunk is
(-6.15, -0.57), **0.36 m inside the west area line**, beside the lap-0 corner (-6.49, -0.69). The
lap-0 path runs on the line, so the body (half-width 0.235 m) passes 0.36 m from the box centre and
clips the real trunk. The trunk reached the local costmap only as the det_range centroid disc of
about 50-70 cells at (-6.05, -0.55), when the robot was 1-3 m away. When the robot was within
about 0.9 m, the trunk was beside or behind the camera and had no costmap cells (20:23:36 at
0.39 m, 20:26:58 at 0.31 m, 20:29:26). Stereo never marked it.

| UTC | event | robot | to trunk | to line (+ in) |
|---|---|---|---|---|
| 20:17:26 | lap 0: 3 bumper hits at the NW corner, edge-blocked skip 2 m | (-5.63, 1.97) | 2.6 | +0.14 |
| 20:19:11-37 | SW K-turn pivots drift 0.15 m (×4), costmap transit leaves the corridor | (-6.15, -2.75) | 2.2 | -0.35 |
| 20:20:27 | boundary out, front_left 0.74 m (pitch +10°, roll -8°: on a bank) | (-5.96, -2.71) | 2.2 | -0.17 |
| 20:21:07 | STUCK (wheels 0.22 m vs EKF 0.02 m, reversing); escapes 0.01 / 0.11 / 0.04 m; STUCK_NEEDS_HELP | (-6.06, -2.84) | 2.3 | -0.33 |
| 20:21:44 | still stuck under the joystick (wheels 0.24 m, EKF 0.02 m); 20:22:54 lift = emergency (owner picked it up) | | | |
| 20:23:26 | trunk static, `act`, on path (1.12 m / -23°) | (-6.27, -1.94) | 1.4 | -0.02 |
| 20:23:37-41 | bumper hit on the trunk; MPPI "Optimizer fail" ×53, patience; edge-blocked skip | (-6.57, -0.58) | 0.42 | 0.00 |
| 20:23:42-24:14 | route pivot "Collision Ahead" ×2, then pivot drift = transit_abort | (-6.70, -0.67) | 0.56 | -0.16 |
| 20:24:18 | person wait 15 s (blade already off for the transit) | | | |
| 20:25:06 | 3 bumper hits at the NW corner again = transit_abort | (-5.99, 2.00) | 2.6 | -0.15 |
| 20:26:46-50 | lap 0 from the south: cm stop, MPPI fail, edge-blocked skip | (-6.50, -1.20) | 0.72 | -0.05 |
| 20:28:18-29 | pivot drift ×2 at (-2.95, 0.83) = transit_abort, then skip | | 3.5 | +0.5 |
| 20:30:41-52 | cov pivot drift, then route-corner Spin "Collision Ahead" ×3: STUCK_NEEDS_HELP | (-6.40, 0.09) | 0.71 | +0.34 |
| 20:37:15 | owner STOP + resume; heading calibration; mowing sub-path 4/6 again | | | |

**Root causes.**
1. Coverage has no "go around". `_follow_done` routes edge-lap aborts (the stretch path and
   `_edge_lap`) straight to `_edge_skip` (2.0 m). `_try_detour` is reached only on rows, and it
   is a blade-off Nav2 jump 1 m ahead. MPPI cannot deviate either: its horizon is 0.6 m
   (40 × 0.05 s × 0.3 m/s), with PathAlign 30, Constraint 4 and WheelFloor 30 against
   CostCritic 3.81. The tree is 0.36 m inside the line, and the outside of the line is off
   limits, so lap 0 cannot pass.
2. The obstacle has no width: det_range puts one centroid point per box. A footprint check
   (`consider_footprint`) against that one cell 0.5 m from the line finds no collision, so the
   robot scrapes the trunk. The bumper and its marks then cause the abort.
3. A Spin aborted by "Collision Ahead" in a route corner (`_pivot_unfinished`) ends in
   STUCK_NEEDS_HELP after 2 re-sends. A coverage pivot has a blocked-pivot skip, but a transit
   route corner has no back-off or skip.
4. Pivot drift: 16 SPIN DRIFT trips (0.15 m) in about 43 spins, against a median of 0.037 m
   yesterday. Each one re-plans a transit, which put the robot into the corner pivot next to
   the trunk. Wet turf or slope (roll 4-5°) is suspected.
5. The 20:21 stuck was real: the robot was 0.33 m outside the SW line, on a bank, and the joystick
   could not reverse it either. The K-turn at (-5.81, -2.62) is planned with the body 0.54 m out.

`blade_policy` is `continuous` (logged; Area 7 has no override). The person wait happened in
TRANSIT, where the blade was already off; `_soft_blade_off` keeps it on in MOWING.

**Fixes (not on edge-layout3):** detection width in the costmap (det_range: bbox width to a
segment or disc at the range, or keep marks for static classes); an edge-lap static-obstacle
detour (pull lap 0 inward around a known static mark, or a persistent no-go disc from
obstacle_tracker in the planner); a route-corner Spin "Collision Ahead" fallback (back up 0.3 m and
re-plan, or skip) instead of STUCK_NEEDS_HELP; check why stereo does not mark the trunk;
investigate the pivot drift on this lawn; review the SW K-turn overhang (0.54 m) next to the bank.

## Field mow 2026-10-23 midday (00c041d)

The run after the trunk incident, from the owner's resume at 20:37:15 UTC to the dock at 21:08:07.
Session `20261022-202312` (splits 15-41; splits 31 and 39 are corrupt and fall in the manual
phase), plus `20261022-210452` and `20261022-210518` (the return to dock). Data is in
`ros2_stack-wt/accuracy-simout/field_mow_2026-10-23b/`: bags, incidents, node logs (ntrip lines
removed), `dock.json`/`mow.json` dumps, `wheel_cal_*.txt` and `raw/health_2110.txt`. The stack
started at 19:38:49 and was not restarted.

### Outcome

| | |
|---|---|
| area | Area 7 (index 5) only, resumed at sub-path 4/6 (3/6 done, 43.2 %, 71.2 of 164.8 m) |
| mowed this run | sub-path index 3 to 56.4 % (+21.8 m), then abandoned at 20:45:15 ("NOT mowed: transit made no progress 4 times"); sub-paths 4 (7.0 m) and 5 (1.1 m) done; retries of the skipped stretches all failed again |
| end | 20:50:05 STUCK_NEEDS_HELP (boundary twice within 1 m at the SW bank); owner MANUAL 20:50:14-21:04:44; owner RETURN 21:05:17 |
| `mow_progress` | 15.4 % at 20:50. `mowed_m` fell from 93.0 to 16.9 m when sub-path 3 was dropped, so the counter resets on a NOT-mowed sub-path (bug) |
| time (20:37:15-20:50:05) | MOWING 5.3 min, TRANSIT 7.3 min, of which 111 s were retry waits (12 retries), PLANNING/RTK 0.2 min |
| NEEDS_HELP | 1 (20:50:05: boundary trip 2 within 1.0 m at (-6.07, -2.66)) |
| skips | 5 edge-blocked (3 pivot drift x2, 1 collision_monitor stop 3 s, 1 bumper x3), 3 BUMPER SKIP; 3 retried skipped stretches "still blocked"; mission skip total 1.4 m plus the rest of sub-path index 3 (poses 1865-3164) |
| STUCK | 4, all escaped (3 reverse 0.30 m, 1 forward arc after the reverse made 0.01 m) |
| boundary trips | 2 in the mow plus 1 in the dock run (below); 0 START-while-outside (1 START, from inside) |
| K-turns | 1 planned, at (-5.81, -2.62) with the body 0.54 m out; its 2 about-turn pivots (-247 and -259 deg) both pivot-drifted, so it was skipped |
| pivot drift | 19 SPIN DRIFT in 39 spins (11 coverage, 6 route corner, 2 final approach); in the morning it was 16 in 43 |

The robot spent the run at the same three spots as in the morning:
- the trunk corner: STUCK, bumper and pivot-drift events at (-6.60, -0.28), (-6.67, -0.42),
  (-6.58, -0.67), (-6.47, -0.38) and (-6.63, -0.42), all within 0.5 m of the trunk at
  (-6.15, -0.57), between 20:39 and 20:45;
- the NW corner: 3 bumper hits at (-5.44, 2.08) at 20:48;
- the SW bank: boundary trips at 20:49.

**Boundary recovery (the stock-style reverse 0.3 m and resume) works.**
- Trip 1, 20:49:27: front_left 0.82 m out (soft limit 0.74); it moved 0.30 m back and resumed.
- Trip 2, 20:49:59: 0.74 m out; it moved 0.16 m back and was inside again. Because this was the
  second trip within 1.0 m, the mission went to NEEDS_HELP, as designed.

Both trips were on the SW bank next to the K-turn, where the robot was stuck in the morning at
20:21. Under the joystick at 20:50:21 it was stuck again (wheels 0.36 m, EKF 0.04 m).

### Return home and docking

The owner triggered it (`high_level_control(2)` at 21:05:17; battery 75 %). The dock route was
7.0 m (Area 7 2.8 m + Dock path 4.2 m). It had 3 route-corner pivot drifts and then a
boundary trip.

| UTC | state | what happened |
|---|---|---|
| 21:06:24 | ALIGNING | 108 deg off; pivoted to -42 deg, then "marker in gate while aligning" ended it early |
| 21:06:29 | LATERAL_ALIGN | RTK lateral -0.21 m (> 0.15): forward arc 0.43 m, 0.65 -> 1.05 m out (cap 0.8 + 0.25); lateral -0.21 -> -0.23, heading -40 -> +20 deg. The marker was seen in 7/7 frames at 0.66-0.89 m out and lost at 1.0 m |
| 21:06:33 | SEARCHING | 15 s, 96 frames, 0 % detected; exposure auto -> 100/90/80/30 |
| 21:06:48 | NAV_TO_APPROACH (retry 1/8, offset -0.10 m) | Nav2 to the approach pose, facing the dock: 1.72 m forward and 0.46 m back, then a boundary trip at (-0.61, -0.21) with base_link 0.12 m out (21:07:09); a 0.28 m retrace worked |
| 21:07:13 | BT dock route again | 1.2 m, 3 legs, 1 pivot drift |
| 21:07:53 | ALIGNING -> SEARCHING -> DOCKING | lateral -0.07 m; marker accepted after 4 detections in 0.5 s at 0.75 m out (RTK 0.64 m; agrees within 0.01 m and 12.8 deg) |
| 21:07:58-08:05 | DOCKING | 0.66 m reverse; KF rejected 9 headings, 0 positions, 0 clamped; KF lateral 0.03-0.07 m; no realign (the pose check at 0.65 m passed) |
| 21:08:05 | FINAL_DOCKING | marker lost 0.08 m from the contacts, straight reverse |
| 21:08:07 | CHARGING | debounced contact; `charging on -> ok`; current -0.6 -> +4.2 A; BT "docked at 74 %"; offset 0.216 m measured (0.218 configured) |

The stuck guard stood down at 21:08:05.8 ("the docking server owns the push onto the contacts")
and was armed again at 21:08:07.4. Time to dock was 2 min 50 s; the first dock goal plus its
recovery cost 1 min 43 s.

**Forward moves in the dock sequence** (owner: "it doesn't need to move forward so much when
getting ready to align")

| move | state / param | driven | needed? |
|---|---|---|---|
| route to the approach pose | BT dock route, `approach_distance` 0.8 | arrived 0.68 m out | yes. Attempt 2 docked from 0.68 m |
| ALIGNING | `align_creep` 0, `align_max_travel` 0.4 | 0.04-0.05 m (pivot slip) | yes, but attempt 1 left it 42 deg off, because the marker gate is forced in ALIGNING |
| **LATERAL_ALIGN forward arc** | `lateral_align_tolerance` 0.15, cap `approach_distance` + `forward_cap_margin` 0.25 = 1.05 m | **0.43 m** | **no.** It started with heading -40 deg (the code allows up to 45 deg), so the arc only turned the robot and lateral did not improve (-0.21 -> -0.23). It drove the marker out of view. The reverse controller handles 0.2 m from a marker lock |
| pose-check realign | `realign_forward_distance` 0.65 (clamped to the cap) | not triggered | - |
| **retry re-approach** | `retry_lateral_offsets` 0.1, Nav2 with `force=True` | **1.72 m forward + 0.46 m back**, then a boundary trip and a 1.2 m re-route | **no.** It was 0.31 m from the offset approach pose (inside `approach_skip_radius_m` 0.6), but the forced Nav2 goal facing the dock made it loop |

Recommendations (no code changed):
1. Start LATERAL_ALIGN only when |heading| <= 15 deg (now a hard-coded 45 deg) and no marker is
   in view. Raise `lateral_align_tolerance` from 0.15 to 0.25 m.
2. Lower `forward_cap_margin` from 0.25 to 0.10 (cap 0.90 m). The marker was lost at about 1.0 m.
3. A retry after a failed search within `approach_skip_radius_m` should align in place and
   continue, not run a forced Nav2 goal.
4. Lower `realign_forward_distance` from 0.65 to 0.40.
5. Keep `approach_distance` 0.8 and the both-wheels pivot floor.

### Heading (wheel_cal, 20:38 onward)

Only 6 straights of 1.5 m or more (12.3 m) were usable: the mow was all short edge legs, and no
straight reached 3 m. There were no paired lanes, so there is no lane-gap offset.

| | this run | 22 Oct midday |
|---|---|---|
| fused yaw - course | -0.46 ± 1.50° (n=6) | -0.07 ± 0.21° |
| aligner COG innovation | -0.01° (sd 2.17, n=50) | +0.01° (sd 2.04) |
| offset drift rate | -1.47..+0.98°/min, never above 2 | -4.2..+4.7°/min |
| ground / commanded, ground / MCU | 1.024 / 1.070 | 1.021 / 1.063 |
| pivot measured/commanded CCW / CW | **0.51 / 0.63** | 0.80 / 0.81 |

The heading is fine. Pivots deliver 20-40 % less yaw rate than yesterday: wet turf or slope slip.
This is the pivot-drift cause.

### Health, cameras, controller

- **Memory:**
  - at 21:06 and 21:10 (docking and docked), 1470-1481 MB available, against the 297 MB
    baseline;
  - swap 22 MB used;
  - zram 21.8 MB of data in 4.5 MB (ratio 4.8);
  - no OOM;
  - no memory series is bagged.
- **Load:** 8.3 / 11.3 / 12.0 at 21:06; 6.6 / 9.8 / 11.3 at 21:10.
- **node_host processes:** cameras 252 MB RSS / 55 % CPU, perception 210 / 18, housekeeping 137 /
  19, gui 121 / 15, control 115 / 28, localization 105 / 18. Other large processes:
  docking_server 255 MB, map_server 122 MB / 56 % CPU, foxglove 210 MB.
- **Crashes and restarts:**
  - supervisor: 0 restarts apart from the rosbag2_recorder respawns at each session start
    (21:04:55, 21:05:21, 21:08:18);
  - the 21:04:58 `crash_node_down_rosbag2_recorder` incident is that respawn.
- **Cameras:** no image topics are bagged, and the composed camera host logs no rates. The
  config is OA `fps` <= 10 (`cameras.yaml`), so the 15 Hz baseline cannot be compared.
  - Detector throughput (`detections_ranged`) in MOWING: left/right OA and vio 4.84-4.88 Hz
    against the 5 Hz cap (97 %), and rear 1.0 Hz. In MANUAL it was 4.3 Hz, and in the dock run
    4.8 Hz (rear 2.0 Hz).
  - Stereo: pairs at 10.0 Hz (source 23.8 Hz); `stereo_depth` processed about 3.8 frames/s,
    27 ms per frame.
  - Docking processed 6-7 rear frames/s (cap 10).
  - So there is no sign of camera starvation from M2.
- **Controller:** 55 misses of the 20 Hz target from 20:37 to 21:08 (min 11.2 Hz, median 18.8,
  none below 10). That is 3.5 per active minute, against 3.3 in the morning and 2.8 yesterday.
  19 of them came during the 21:06-21:07 dock navigation.

**Next:**
- a trunk/edge detour, as in the morning section;
- the SW bank: move the K-turn or the boundary inward;
- the pivot slip on wet turf: wait for dry turf, or accept the drift and widen the threshold
  for the edge;
- the `mow_progress` reset;
- the dock forward moves above.
