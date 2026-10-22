# Stock Airseekers firmware: edges, boundary-out, obstacles and docking (2026-10-21)

These are reverse-engineered findings from stock OTA v1.3.27-rc.1 (`ota_v1.3.27-rc.1-tron+20260713/`). They were gathered to compare our port against stock now that the owner has measured the chassis (see [Measured geometry](#measured-geometry)).

- **C** marks findings confirmed from source, a config file or a decompile.
- **I** marks findings that are inferred.

The Ghidra decompiles are kept outside the repo:

- `native_decompile/mower_controller_2026-10-21/`: `check.c` is `libmower_controller_check.so`, `mc.c` is `libmower_controller.so`.
- `native_decompile/mower_charge_2026-10-21/`: `mower_charge` and `libmower_charge_core.so`.

## Measured geometry

The owner measured the chassis on 2026-10-20. All values are relative to base_link, which is the rear drive-axle centre.

| Item | Value |
|---|---|
| Body | front 0.60, rear 0.10, half-width 0.235 (0.47 m overall with trim), length 0.70 |
| Track | 0.39 m (physical). The MCU splits wheel speeds with a 0.40 m track. |
| Wheels | 0.22 m diameter, 6 cm tread |
| GNSS antenna | 0.098 m forward, 0.214 m up |
| Deck centre | 0.23 m forward |
| Blades | small 220 mm (fitted); big 270–300 mm |
| Pivot sweep radius | 0.645 m at the front corners, 0.255 m at the rear corners |

Owner rule for soft areas, which is all of them for now: a drawn or driven area line is where the rear axle should go.

## Boundary-out check and recovery

Sources: `libmower_controller_check.so` (`CheckRegion`) and `libmower_controller.so` (`NavEdge`).

All values are hard-coded in `CheckRegion::Reset` @0x1c1f54; there are no parameter strings (C).

| Value | Default |
|---|---|
| mower_length | 0.70 |
| mower_half_width | 0.225 |
| front_severe_distance | 0.50 |
| back_severe_distance | 0.20 |
| slice_ratio_threshold | 0.70 |
| safe_distance | 1.0 |
| Slice sample step | max(0.02, map resolution); 0.05 when no resolution is set |
| Recovery move | 0.3 m |
| Recovery speed | −0.2 m/s on a FRONT trip, +0.2 m/s on a BACK trip |
| Move timeout | 5 s |

### Detection

`GetBoundaryOutDistanceByDirection` @0x1c37b0 does the check (C):

- It samples one line across the body at x = 0.20 in a [0, 0.70] body frame, over the full width.
- A sample counts as outside if its raw-map cell is anything other than free or channel.
- The check trips when the outside ratio is at least 0.70. The result is binary: it trips or it doesn't.
- Only severity 3 is used, and the direction is effectively always FRONT.
- A latch increments an event id once per episode. There is no debounce.
- The check runs only in NavEdge, i.e. on edge laps; coverage rows are not checked.

### Recovery

`onBoundaryOut` @0x92e818 runs the recovery (C):

1. Reverse 0.3 m in a straight line at 0.2 m/s, with a 5 s timeout.
2. Rotate in place to the edge path's heading at the closest point.
3. If no heading is found, resume without rotating.
4. Re-initialise tracking on the same edge path.

There is no retry cap and no task failure: if the robot is still out, the check re-triggers (I).

### Our port, before round 2

- `_hard_boundary_tick` (`mission_fsm.py`; parameters in `mission.yaml:360-391`) trips when base_link is more than 0.10 m outside (`/map_server_node/outside_m`).
- Recovery retraces the recorded track in reverse at 0.10 m/s for up to 1.0 m.
- A second trip, or a failed retrace, goes to STUCK.

### Plan for round 2 (decided 2026-10-21)

- Check body points instead of base_link only: the 4 corners plus the mid-front and mid-rear points.
- Limits per edge mode:
  - **Hard:** any point more than 0.10 m out trips.
  - **Soft:** limits that normal soft-layout driving never reaches, about 0.25–0.30 at the sides and about 0.30 at the nose.
- Recovery copies stock: blade off, reverse 0.3 m (at our 0.10 m/s cap), rotate to the path heading, resume. Track retrace stays as the fallback.
- Keep our retry cap and the STUCK escalation, which stock lacks.

### Our port, round 2 (implemented 2026-10-21, branch `ws/edge-layout2`)

Code: `mower_mission/body_boundary.py`, `MissionFSM._hard_boundary_tick` and helpers; the README "Boundary out" section lists the parameters.

- **Points:** the 4 footprint corners, the mid-front (nose) and the mid-rear (tail), against the mission's own area snapshot (mowing areas, drawn paths and the dock corridor). A GUI map reload does not reach this snapshot.
- **Where the body check applies:** whenever base_link is in, or less than 1 m outside, a mowing area. On a drawn path only, in the dock corridor and in dock phases, the base_link check stays (more than 0.10 out).
- **Limits:**
  - Hard: any point more than 0.10 out.
  - Soft: rear corners 0.35, front corners 0.50, nose 0.35, tail 0.30.
  - The front corners get 0.50 because on lap 0 along a tight convex bulge they sit outside by geometry alone, sqrt((R + 0.235)² + 0.6²) − R. On owner_far that is up to 0.43 nominal, and no K-turn can avoid it. Stock's check line sits near the axle and does not see it. Lateral drift is still caught by the rear corners, which are not affected by curvature.
- **Why these soft numbers:**
  - Normal soft driving: lap 0 puts the sides 0.235 out; a row end 0.45 in puts the nose 0.15 and the front corners at most 0.195 out, pivot sweep included.
  - Sim tracking error is p99 0.07 while pivoting and 0.13 over all samples.
  - Stock with our half-width trips with the outer side about 0.33 out (0.235 + 0.4 × 0.235 at its 70 % line ratio).
  - Tighter soft limits would trip on normal lap-0 drift. That was the round-1 problem, where base_link at 0.10 tripped on lap 0.
- **Recovery:**
  - A nose or tail trip moves 0.3 m straight at no more than 0.10 m/s (forward when the worst point is behind base_link), with a 5 s timeout.
  - A side-only trip (lateral drift, where a straight move would not help) reverses about 0.8 m along the planned leg toward the interior.
  - The mission then resumes from the robot's projection, and the leg-start pivot turns to the path heading, as stock does.
  - If the robot is still beyond a limit, the driven-track retrace runs (fallback).
- **Budget:** 6 per mission (was 2), plus 2 within 1 m, then STUCK_NEEDS_HELP. Stock has no cap.
- **Added 2026-10-22, after the sim gates and the morning field mow:**
  - **Plan-relative soft limit:** within 1 m of a conditioned corner whose planned swept body is out by b, any point may reach b + 0.20.
  - **No recovery ends outside:** up to 3 bounded inward moves are made first.
  - **START or resume while outside:** the robot moves inward first, with no pivot, and the move is not counted.
  - **Coverage pivots** turn toward the interior, choosing the direction by swept body.
  - **Transits to an edge-lap start** end 0.30 m inside, beside the start.
  - **Pivot retries:** a refused pivot is re-sent after 2 s.
- **Jumps:** a reading that jumps more than 1 m in one tick is held off for up to 2 s and is not counted.
- **Atomic map replace:** map_server `begin/commit/abort_map_edit`, which the GUI PUT /map uses, means no half-built map is ever published.

## Edges, turns and coverage

Sources: `libmower_planning_core.so`, `mower_logic/config/coverage.yaml`, `task.proto`.

The planner works on a 0.05 m/px raster with erosions of 0.15, 0.30 and 0.45 (C). From that:

- Edge laps run at 0.00, 0.15 and 0.30 from the line. The first lap runs base_link on the line.
- Rows fill the region eroded by 0.45, so rows end 0.45 inside the line.
- Row spacing defaults to 0.15; the app offers 0.13–0.18 for the small blade and 0.16–0.23 for the big blade.

Turn handling:

- TurningMode 0–6 covers in-place turns, arcs, fish-tail turns, slope turns and an adaptive mode (C). In-place turns are forced at the start and end of each path (C).
- When a turn or path end is blocked, stock skips it and moves to the next path (C): "skip in-place rotation", "Cover end pose is likely occupied … switch to next path".

Body fit: stock does NOT keep the body inside the line. Body overhang stays within half the mower's width, and stock relies on the boundary having been recorded by driving it (I).

There is no user-facing setting to shrink areas (I).

### Our port, before round 2

- The outer lap was 0.24 m inside the line and rows ended 0.14 m inside.
- At row ends the nose was about 0.46 past the line, and pivots swept about 0.50 past.
- `ws/edge-layout` (round 1) implements the stock-style layout.

### Edge-ring turns (round 2, 2026-10-21)

What stock does (I: the decompile has no code bodies for these, only headers and comments):

- `task.proto` TurningMode (field `truning_mode`) has Default, In_Place, Circular_Arc, Inner_Fish_Tail, Outer_Fish_Tail, Slope and Self_Adaptive (C).
- `mower_controller/src/nav_cover/nav_cover.h:89-125` `NavCoverUturn` has `uturnInplace`, `fishExtern`, `fishInner` and `uturnArc`. The fish-tails are row-to-row U-turns, and there is no corner or edge-lap K-turn logic (C/I).
- `makepath/makefish_extern.h` (outer): arc forward to perpendicular, straight reverse 0.6, arc into the next row (arc r 0.3, spacing 0.15) (I, from comments).
- `makepath/makefish.h` ("inner", inferred): backward arc away from row B, forward 0.75, backward arc onto B. Its own comment says "outer fish-tail", so the names may be swapped (I).
- On edges, stock forces in-place turns at path starts and ends, and skips blocked turns (C, see above).

Where our large ring turns came from, and what changed:

- **Rounding:** the round-1 r 0.7 lap rounding never took effect, because F2C mitres the buffered laps. Lap 0 now keeps sharp vertices on purpose, and the conditioner handles corners with the body model.
- **Lap direction:** lap 0 ran counter-clockwise and laps 1–2 clockwise, which forced about-turns at hand-offs. All laps are now clockwise.
- **Lap starts:** lap 0 started on a convex vertex at (-1.09, 2.14), giving a -119° hand-off pivot that swept the body 0.5 m out. Lap starts now sit at least 0.6 m from vertices.
- **"Hairpins":** the -174°/+170° pivots at (-4.4, 2.9) and the reversing tail at (-0.91, 4.84) were stacks of sliver fill passes, not merged hairpins. Soft fill passes now need a length of at least 0.30 m and at least 35 % new area. Real hairpins condition to 90 / straight / 90.
- **Hand-off pivots:** `_join_heading` now stops its look-ahead at a pivot vertex, giving -21° then -112° instead of a single -138°.
- **K-turns (ours, not stock):** a convex corner whose pivot or fillet would put the body past the planning limit (soft 0.26, hard 0.0) gets a 3-point turn:
  1. forward to P1 = V − a·u_in;
  2. about-turn through the interior;
  3. reverse with the blade off to V;
  4. pivot onto the next run.

  Only the rear corners (0.255) sweep out. Unit-tested on a ring with a sharp corner on the line.
- **Curved approaches:** P1 is measured as arc length back along the incoming path, and the reverse leg retraces it. Pivots on the line along it, such as jogs, become in-place pivots inside the reverse leg. Corners under 60° also try a K-turn when they would exceed 0.33. Where nothing fits, the option with the least overhang is used and logged at warn.
- **What remains:** run5 has no corner over 0.35 nominal. owner_far has 7 lap-0 corners on drawn convex bulges tighter than about r 1.3, at 0.36–0.43 nominal, and these are front-corner overhang. Driving the line there puts the front corners out whichever way the robot turns, so these are inside the 0.50 front-corner trip limit. Stepping lap 0 inward at tight bends would be a planner change, for the owner to decide.

### Decision 2026-10-22: lap 0 stays on the driven line (owner option (c))

- **The bulges are driven arcs.** Area 6's "bulges" are arcs the owner drove: v10–15 R≈0.72, v21–25 R 0.71, v31–33 R 0.44, v40–41 R 0.44.
- **Antenna-trace argument.** Following the recorded line with base_link reproduces the owner's own drive to within 1 cm; the antenna-trace difference is ≤ 1 cm. Stepping lap 0 inward would cut short of what the owner drove.
- **Arc envelope.** On such an arc the front corners sit sqrt((R + 0.235)² + 0.6²) − R out, and the nose sqrt(R² + 0.6²) − R:

  | Arc R | Front corners out (m) | Nose out (m) |
  |---|---|---|
  | 0.44 | 0.46 | 0.30 |
  | 0.72 | 0.41 | |

  This is geometry, not error.
- **Conditioner.** A convex fillet on a lap that runs on the line is held to max(limit, its own envelope + 0.03). A K-turn only replaces a pivot that no fillet with r ≥ r_min can replace. Area outlines are cleaned in memory before planning (jog drop, then Douglas-Peucker 0.05), which removes the 9 cm cusp at (-27.19, -3.48)→(-27.28, -3.49) and the v42/v0/v1 start wobble. The areas file and the boundary-check outline stay as recorded.
- **Lap-0 K-turns on replay:** Area 6 drops from 5 to 0; run5 Area 1 has 1 (+100° V).
- **Soft trip limits:**
  - front corners 0.55: the R 0.44 envelope 0.46 plus tracking p99 0.07;
  - nose 0.40;
  - rear corners stay at 0.35 and still catch lateral drift.
- **Blade reference.**
  - The blade centre stays at the deck centre (blade_link 0.23 forward). We do not adopt stock's 0.398.
  - The cut width follows the fitted 220 mm disc everywhere:
    - map_server paints mow progress with blade_radius = blade_disc_mm / 2000 = 0.11 (it was 0.15, a 0.30 disc);
    - coverage cut_width_m is 0.22 (was 0.20).
  - `blade_disc_mm` is the one robot-wide setting, and the UI cutter_size setting (B1) will drive it.

## Obstacles

Source: `mower_task/mower_logic/config/costmap.yaml` and controller strings.

- The costmap camera layers are configured as STVL with decay (5 s for the camera, 10–50 s for the bumper), but they are commented out of the shipped plugin list. The stock costmap therefore never holds camera marks (C).
- The stereo minimum obstacle height is 0.15 (C).
- Obstacles come from labelled YOLO detections plus the bumper (`NavEdge::CheckVisionObstacle`; "skips unsupported vision label") (C).
- The avoidance sequence is: back up, plan around, rotate, rejoin (C).
- Rotation collision checks run against the static map, not live sensors (`hasRotationArcCollision`) (C).
- `ObsStrategy` option 2, `OS_Bumper_AI_Except_Edge`, turns AI avoidance off on edge laps (C).

### Our port

- On 2026-10-21 the field false stops came from stereo picking up grass when the ground fit failed.
- The collision_monitor stop zone stayed armed during slow arcs.
- Stale side marks blocked Spin.
- Fixes are on `ws/obstacle-hotfix`.

## Docking

Sources: `mower_charge` and `libmower_charge_core.so`. The node launched in production is `src/mower/launch/mower_controller.launch:6` (C).

### Approach and alignment

- A*/TEB to `undock_pose` (0.8, 0) on the fused GNSS pose.
- Adjust, using the RTK pose in the dock frame:
  1. Rotate to within 8.5° of the dock yaw.
  2. If more than 0.15 m off the dock axis, drive a line back onto it.
  3. Search.
- Out of range during docking (RTK x ±2 m, y ±1.6 m) leads to RETRY.

### Detection

- Rear camera, `DICT_4X4_50`, marker size 0.04, EMA with alpha 0.2. The detection timer runs every 0.06 s.
- Search: after 3 s, it checks the detection ratio over 20 frames. If the ratio is 0.2 or less, it changes the exposure (auto, then manual 100…5) and searches again, up to 30 times.
- A day/night HSV-brightness check picks the exposure preset.
- The 0.5 m / 25° gate (`CheckInitJump`) has no call site; it is dead code (C).

### Reverse-in control (`LineTrackingDocking`)

- It tracks a 1.2 m line in the marker frame with lookahead 0.5.
- PD gains: distance (1.15, 0.5), angle (0.32, 0.45).
- Speed: 0.1 nominal, maximum 0.2.
- A Kalman filter predicts from cmd_vel and corrects with the marker (Q 0.01, R 0.1). Innovations are clamped at 0.5 m / 20°.
- Pose check at 0.65 m: if lateral error is over 0.05 m or angle error over 8°, it drives forward 0.65 m and restarts.
- Near field starts below 0.35 m.

### Marker lost

- It keeps reversing on the Kalman-filter prediction, slowing down as it goes, capped at 0.08 m/s and 0.15 rad/s.
- In the near field it reverses blind at 0.06 m/s for 2.5 s.
- "Lost" is declared at more than 0.7 m with 31 frames missing, or after 5 s with no image.
- It cycles the exposure every 1 s and goes to RETRY after 20 s.
- The overall timeout is 60 s.

### Retry, contact and undock

- Retry: drive a line on RTK to the dock x with a lateral offset that alternates and grows, ±0.1 per step up to ±0.3. Up to 8 retries.
- Contact: final reverse at −0.05 m/s for up to 10 s, waiting for the charging-done signal, debounced over 3–4 samples.
- Undock: a PID-controlled 0.8 m drive.

### Exposure details (decompiled, `main/mower_charge.c` OnSetParam / OnDocking)

- The manual sequence is a `std::map<int, double> exposures_`: 1→100, 2→90, 3→80, 4→30,
  5→20, 6→10, 7→5, 8→2, 9→1 (`exposure_absolute`, 100 µs units). After index 9 it goes
  back to auto ("From manual to AUTO ... Manual Set End") and starts over.
- The first step from auto reads the current auto value and sets manual to that value
  minus 50 ("From auto to manual ... SetManualExpose is: %d", `uVar2 - 0x32`). We start
  at 100 instead (reading the auto value back is unreliable on UVC).
- The camera commands are shell calls: `v4l2-ctl --device=/dev/rear_camera
  --set-ctrl=exposure_auto=1 --set-ctrl=exposure_absolute=N` and `exposure_auto=3`
  (aperture priority = UVC auto). SetAutoExposure also runs when the camera is opened.
- Why lower exposure helps at dusk: auto exposure opens up for the dark lawn, so the
  backlit marker (about 30 px at 1 m) saturates into a white blob without corners.

### Our port (2026-10-21, branch `ws/docking`)

Before: no exposure control, a 0.5 m / 25° search gate, no lateral alignment, three
retries from the same pose, and a stop when the marker is lost.

Implemented, with the stock structure and our speeds (README "Stock structure port"):

1. Exposure stepping. `mower_docking/exposure.py` steps through stock's sequence:
   detection ratio ≤ 0.2 over 3 s in SEARCHING, every 1 s while lost in DOCKING. A dark
   first frame (mean grey < 50) starts on manual 30. Auto is restored after docking and
   after undocking. The commands go over `/rear_camera/exposure_cmd` to `mower_cameras`
   (`camera_node` owns `/dev/rear_camera` on the mower), which uses V4L2 ioctls
   (`v4l2_ctrl.py`) instead of shelling out; missing controls only report `ok: false`.
2. Search gate off (`use_marker_gate: false`). ALIGNING keeps it for its early exit.
   SEARCH windows are logged with raw frame and detection counts, and the accepted marker
   pose is compared with the RTK pose (log only).
3. Retries: 8, re-approaching on lines offset 0, ±0.1, ±0.2, ±0.3 m from the axis,
   starting on the side the marker was last seen from. Mission `dock_timeout_s` 180 → 480.
4. RTK lateral pre-alignment (`LATERAL_ALIGN`): more than 0.15 m off this attempt's line
   → a forward arc at 0.12 m/s with |w| ≤ 0.3 (both wheels ≥ 0.06 m/s) back onto it, then
   reverse on RTK to the approach distance. Needs a measured dock pose and RTK fixed.
5. Kalman filter in the docked frame (odom prediction, Q 0.01, R 0.1, innovations clamped
   at 0.5 m / 20°). Marker lost outside the final zone → keep reversing on the prediction
   for 2 s at ≤ 0.08 m/s, then hold, RETRY after 8 s (was: stop at once, RETRY after 3 s).
6. Pose check once per approach at a marker distance of 0.65 m: > 0.05 m or > 8° →
   forward 0.65 m and restart (was 0.15 m / 20° at the final zone, forward 0.8 m).
7. Diagnostics bagged by `bag_recorder`: `/mower_docking/marker_pose`,
   `/mower_docking/diag` (1 Hz), `/mower_docking/debug_image/compressed` (1 Hz grey JPEG,
   dock goals only) and `/rear_camera/exposure_state`.

Not ported: stock's low speeds (0.02 m/s, 0.15 rad/s) — they break the 0.06 m/s wheel
floor; stock's 1.2 m tracking line with lookahead (our reverse controller is already
tuned in the sim and the field); the auto-value-minus-50 first manual step.

`marker_size` stays 0.04. The physical square is about 0.055 m (not measured exactly),
so marker distances read about 0.73 × true. Stock uses 0.04 on the same dock, so its
marker-frame thresholds (0.65 m, 0.05 m / 8°) transfer unchanged. Changing it means
re-calibrating `docked_marker_offset` and rescaling the marker-distance parameters (list
in the mower_docking README, "marker_size").

Sim results (2026-10-21, `ws/docking`; mower_sim, marker rendered at 0.055 m):

- real_run5 ideal full gate: PASS (undock, mow, return, docked first attempt, one_wheel 0).
  The marker was lost 0.31 m out; the filter prediction carried the reverse through it.
- Dock-only from 1.2 m out, 0.30 m off the axis: LATERAL_ALIGN brought it to 0.00 m on
  RTK, then docked first attempt (32 s).
- Dock-only from 1.0 m, -0.12 m, -8.6°: docked (44 s), but the pose check fired twice on
  heading (12.7°, 9.8°) and the 2-realign budget absorbed it. The marker-normal yaw is
  the noisiest quantity at this marker size (the accepting detection differed from RTK by
  up to 12°), so the 8° heading limit may cost realigns in the field too.
- Not testable in the sim: exposure (the renderer ignores it), the retry offsets (no
  failures to retry), dusk.

### Field docking 2026-10-21 night (ec8be43, synced 08:20 UTC)

Data: bag `20261021-082701`, docking log `python3_342`, incidents 08:28:57 and 08:30:42
(copied to `ros2_stack-wt/accuracy-simout/field_dock_2026-10-21/`). The 08:17:49
`crash_docking_failed` came from the old code (DOCK_NOT_FOUND after 3 re-approaches,
no exposure control).

Three dock goals with the new code (08:27:08, 08:29:58, 08:30:58). None docked: two
ended in the hard boundary, and the owner stopped the third at 08:32:19 and docked by
joystick.

- **Exposure works.** All 20 `exposure_state` replies were `ok: true` (auto→100→…→20,
  30→5, 30→2→1→auto). On auto the backlit marker glows and saturates (mean grey 68,
  not decodable). On manual ≤ 30 the frame is black (mean 12.0–13.7) with a sharp
  marker. There were 327 detections at exposures 20, 5 and 2. The SEARCH-window log line
  reports "0 with a marker" even when it accepts one: a counting bug in the log only.
- **The marker heading is bimodal (IPPE flips).** Compared with RTK over 329
  detections: 71 % sit at −7.5° ± 3.9° (a constant bias: camera yaw or dock yaw), and
  29 % are flips 22° (mean) away, with the lateral off by 0.35 m. Lateral is computed
  through the marker yaw, so each flip also moves the lateral. The marker is 30–50 px
  (0.04 m configured, about 0.055 m real). The KF (gain about 0.5 at Q 0.01 per 20 Hz
  tick and markers at about 4 Hz) passes half of each clamped 20° / 0.5 m innovation.
- **Reverse control swings.** The commanded w changes sign 0.9–1.8 times per second in
  DOCKING. With k_lateral 7, h_d saturates at 28.6° from 0.07 m of lateral, so the RTK
  heading ran 13→33° and 0→40° while the lateral was only 0.1–0.2 m. The robot reaches
  the 0.65 m pose check mid-swing, and every check failed (−167° from a bogus first
  solve, 30.7°, 20.8°, 29.6°, 14.7°, 45°). That spent both realigns on all three goals.
  Commanded against measured speed: v 0.90, w 0.94 (median). Tracking is fine; this is
  not wheel-floor arcing.
- **LATERAL_ALIGN loop.** Its forward phase starts after the 0.65 m realign, about
  1.1–1.2 m out. It hit the front bumper 13 times at a fixed 1.35–1.46 m from the dock
  (something physical there). Each time the bumper back-off reversed at −28° heading,
  then ALIGNING and LATERAL_ALIGN ran again. The lateral walked 0.28 → 0.63 m.
- **Boundary trips.** 08:28:57 (0.14 m): the owner's joystick (`cmd_vel_teleop`
  −0.11 m/s, −0.6 rad/s overrides docking in twist_mux) reversed the robot out of the
  Dock path polygon. The LATERAL_ALIGN/bumper loop had already left it 0.63 m off the
  axis, outside the 0.35 m dock corridor. 08:30:42 (0.11 m): code only. The realign 2/2
  forward drive (0.58 → 1.21 m) and then the LATERAL_ALIGN forward arc (→ 1.40 m,
  lateral −0.18) went past the corridor cap (approach 0.8 + 0.35 m) and out of the Dock
  path polygon. **This is a regression:** neither manoeuvre checks the boundary.

Fixes proposed that night (implemented on `ws/docking2`, next section): take the heading from RTK/odom and use the marker only for
position, or solve with `solvePnPGeneric` and keep the IPPE solution nearest the
prediction. Reject a first detection that differs from RTK by more than 30° / 0.3 m
before it is accepted or pose-checked, and require ≥ 2 consistent detections. Feed the
KF the median, gate innovations with a χ² test instead of clamping them, and scale Q by
dt. Loosen the pose check to about 12° (bias-corrected) and check it on the median of the
last 3–5 detections. Use k_lateral about 2.5 with h_d ≤ 15°, and add an angular rate
limit. Cap every forward manoeuvre (realign, LATERAL_ALIGN, retry) at remaining ≤
approach + 0.25 m, or clip it to the outside_m/corridor polygon. No LATERAL_ALIGN after a
realign. After a bumper hit in LATERAL_ALIGN, end the pre-alignment instead of looping.
Pause docking, or end the goal, while teleop is active.

### Fixes (2026-10-21, branch `ws/docking2`)

**Where the −7.5° bias comes from: the rear camera yaw, not the dock yaw.**

- The stored dock yaw (100.7°) is an old measurement; `set_docking_point` preserves it.
  The docked EKF yaw (113.8°) is no check of it: it reads the same to 5 digits whether
  the robot is parked crooked or straight, because a standing single-antenna robot does
  not observe its yaw. The evening forward undock (bag 20261021-062832, 06:28:34) gives
  EKF yaw 99.9–101.4° and a track chord of about 104°: within ~3° of the stored yaw.
- A least-squares fit over the 232 consistent detections (marker bearing and marker
  normal in base_link against the RTK track; marker map pose and a yaw offset free) is
  best with a camera (or EKF) yaw error of −5.5°. Rotating the detections by −6.5° about
  the camera removes the heading residual (+0.8° at −7.5°, −1.2° at −5.5°); the marker
  normal then lies within 1.5° of the stored dock yaw. The marker centre sits 8–11 cm
  right of the stored dock axis (marker mounting, or the dock position).
- Fix: `rear_camera_yaw_correction_deg: -6.5`. A dock-yaw re-measurement from the
  forward undock chord is added anyway (mower_docking README, "Dock yaw re-measurement").

**Implemented** (mower_docking README, "Night field fixes"):

- marker position with the filter's odometry heading;
- a χ²-gated filter with per-second Q, and a restart on a consistent cluster of
  rejections;
- 3 consistent detections, cross-checked against RTK (30° / 0.3 m), before DOCKING;
- k_lateral 2.5, h_d ≤ 15°, a 0.6 rad/s² angular rate limit;
- pose check 0.08 m / 12° on the median of 5 detections;
- forward moves capped at approach + 0.25 m on RTK;
- no LATERAL_ALIGN after a realign, and a bumper hit ends LATERAL_ALIGN;
- the joystick pauses the goal;
- the SEARCH log count is fixed.

Not changed: the 1.35–1.46 m front-bumper trigger (the owner will be asked what is there);
twist_mux priorities and the mission.

**Field replay** (`scripts/replay_field_dock.py` on ex082701, open loop on the recorded
track, 6 SEARCHING→DOCKING windows):

| | new code | old settings |
|---|---|---|
| filter heading vs RTK, median | −0.8° | −8.2° |
| filter heading vs RTK, p5..p95 | −3.9..+4.5° | −28.2..0.0° |
| sign changes of w, per second | 0.04 | 1.22 (recorded: 34 in 27 s) |

The −167° first solve does not start DOCKING.

**Synthetic tests**: 29 % flips of ±22°, a 7.5° mis-mounted camera, 4 Hz, 90 closed-loop
runs. All docked. The filter heading stayed within 5° of the truth in 83 runs, p90 4.8°.
The exceptions are initial transients from the 3-detection start. In one run three
same-side flips started DOCKING 26° off; the restart caught it within 2 s. Sign changes of
w were at most 0.34/s. With the old settings the same stream gives over 0.5/s or a swing of
more than 20°.
**Sim (mower_sim, marker rendered with the −6.5° camera error; out:
`accuracy-simout/docking2/`).** The sim cannot inject flips; its renderer only gives
the solver's own noise.

- **Dock-only, start 1.2 m out, 0.30 m off the axis.** Docked in 88 s, after 2 retries.
  LATERAL_ALIGN now stops at the corridor end, 1.20 m out (the old code drove on).
  At 1.2 m / 0.3 m off, the solves agreed with each other but differed from RTK by
  23° / 0.62 m, so the RTK check rejected them; the robot then docked from the offset-retry
  approaches. Cost: time. Benefit: no DOCKING start on a biased solve.
- **Dock-only, start 1.0 m out, −0.15 m, −11.5°.** Docked in 13 s, first attempt, no
  realign.
- **real_run5 ideal full gate: PASS.** The robot docked on the first attempt: accepted
  after 6 detections, 2.2° from RTK, no pose-check realign. The undock chord measured the
  dock yaw at 101.6° (world: 101.6°).

### Low-battery return 2026-10-22 (master 1b862ad, branch `ws/dock-return`)

Owner: "when the battery is low it should automatically go back and charge; it did not, it
just stopped". Data: rosout of `behavior_tree_node` / `mower_docking` / `slip_detector`, bags
`20261021-230412` (mow) and `20261022-011031` (return), copied to
`ros2_stack-wt/accuracy-simout/field_dock_return_2026-10-22/` (`timeline.txt`,
`scripts/extract.py`, `scripts/compact.py`). Split `011031_0` (01:10–01:11) and `_8` are
truncated on the device (recorder restart).

Timeline (UTC):

| time | event |
|---|---|
| 01:08:28 | MOWING area 5 at (-21.2, -6.5), battery 14 % < `battery_low_percent` 15 (device GUI yaml): `battery_low_action=stop` (default since c29dde1) -> IDLE in place. **This is "it just stopped".** |
| 01:10:29 | `high_level_control(2)` (GUI HOME) -> RETURNING_HOME, dock route 35.0 m (3 legs) |
| 01:13:15 | docking server at the approach pose: ALIGNING (93 deg) -> SEARCHING -> DOCKING (marker 0.89 m, RTK agrees within 0.03 m / 5 deg) |
| 01:13:30.6 | marker lost 0.04 m out -> FINAL_DOCKING, reverse 0.05 m/s |
| 01:13:34.17 | slip_detector `/stuck`: "commanded v=-0.05 for 4.0 s, EKF moved 0.041 m, wheels 0.17 m" |
| 01:13:34 | same tick: `is_docking_done` debounced, mower_docking CHARGING -> SUCCEEDED DOCKED; mission logs "docked at 13 %" **and** "STUCK ... escape 1/3", cancels the dock goal |
| 01:13:35–38 | escape step 1 reverse 0.1 m/s seats the robot: `is_charging`, v_charge 19.7 V, 3.3 A |
| 01:13:38–42 | escape step 2 forward arc drives it off the contacts (charging stops 01:13:40.5) |
| 01:14:08–01:16:12 | re-dock: pose-check realign, RTK-disagreement rejections, retry 5/8 with -0.30 m offset, LATERAL_ALIGN, DOCKING, FINAL 01:16:08; `/stuck` after exactly 4.0 s of FINAL -> escape |
| 01:16:41–54 | DOCKING, FINAL 01:16:50, `/stuck` at 4.0 s -> "stuck again at the same spot" -> STUCK_NEEDS_HELP |
| 01:24:47 | owner HOME at (0.38, -1.32), i.e. in the dock pocket: dock route starts with a -84 deg pivot in place against the charger -> `/stuck` (w=-0.30, EKF 0.003 m) -> escape -> re-dock |
| 01:25:33–37 | FINAL, `/stuck` at 4.0 s -> STUCK_NEEDS_HELP; 01:27:42 docked by hand (12 %) |

Findings:

1. **Automatic return was off.** `battery_low_action` defaulted to `stop`; the device yaml
   (`config/gui/mowgli_robot.yaml`) sets only `battery_low_percent: 15`,
   `battery_full_percent: 80`, `battery_max_charge_percent: 80`.
2. **The slip_detector stuck guard owns no exemption for docking.** `STUCK_PHASES` includes
   RETURNING_HOME / *_DOCKING for the whole phase, including the docking action's final push.
   FINAL_DOCKING pushes against the charger at 0.05 m/s until the contacts close: the map EKF
   stops at ~0.02 m past the stored dock pose (0.417, -1.307, 100.7 deg) in every attempt
   while the wheels (and `/odometry/filtered`) keep going 0.13–0.17 m. That is `stuck`
   (EKF < 0.05 m and path < 0.10 m in 4 s) every time. All four final pushes were cut at
   exactly 4.0 s, before mower_docking's own sequence (final budget -> settle 3 s -> creep
   0.15 m -> RETRY, `final_timeout_s` 10) could run.
3. **Contacts work, they need the push.** Attempt 1 closed the contacts 3.4 s into the push
   (no debounce or timeout problem: debounced contact and the stuck latch fell in the same
   0.1 s). The escape's 0.1 m/s reverse then charged at 3.3 A. Attempts 2–4 ended 3–6 cm
   left of the dock axis by RTK (marker KF said +2.5..+5 cm, so the lateral evidence is not
   conclusive) and were cut before the settle/creep.
4. **The mission ignored the contacts.** `_stuck_tick` runs before the dock-contact handling
   and does not look at `is_docking_done` / `is_charging`; the escape kept going after its
   reverse had seated the robot.
5. HOME pressed with the robot already in the dock pocket plans a dock route that begins
   with an in-place pivot against the charger (not fixed here, see "next").

Fixes (`ws/dock-return`):

- `mower_control/slip_detector`: subscribes `/mower_docking/state`; while it reports
  `stuck_dock_exempt_states` (FINAL_DOCKING, CHARGING) and is fresher than
  `stuck_dock_state_timeout_s` (1 s) the StuckDetector runs suppressed (latch cleared, window
  reset). DOCKING / RETRY / LATERAL_ALIGN / no goal stay guarded; a dead docking node
  re-arms the guard after 1 s. FINAL_DOCKING is bounded by docking's own budget and timeout.
- `mower_mission`: a `/stuck` latch in a dock phase while `is_docking_done` or `is_charging`
  is ignored (latch reset); a running stuck escape in a dock phase ends as docked when the
  contacts report (no forward arc off the charger).
- `battery_low_action` default `dock` (Params, `mission.yaml`, GUI select default).

Battery threshold: the device runs `battery_low_percent` 15 (stock 20, our yaml 20). The mow
drained ~0.36 %/min; the 35 m return took 2.8 min and the dock goal may take up to
`dock_timeout_s` 480 s, so a far-area return with a full retry sequence costs about
(3 + 8) x 0.36 = 4 %. 15 % works on paper, but leaves no reserve for a blocked wait, a
NAV re-plan or a second dock goal, and the pack sits at 19.5 V from 14 % down. Recommended:
20 % (GUI Battery -> `battery_low_percent`, a live param).

Next: HOME from inside the dock pocket should reverse/creep (or drive straight out) instead
of pivoting; watch FINAL_DOCKING with the exemption for lateral offsets that never close the
contacts (then tighten `final_max_lateral` 0.08).

## Field symptoms these findings explain (2026-10-20/21)

- Dusk docking failed with "no usable marker", although the dock has a backlit marker. Without exposure control the camera cannot read it at dusk; stock varies the exposure.
- The mower stopped and got stuck at edges.
  - Turns sat too close to the edge, at the 0.14 m row ends.
  - Stereo marked grass as obstacles once the ground fit failed.
  - The stop zone stayed armed during turns.
  - The stuck guard counted collision-monitor stops as stuck.
- Turns before 2026-10-20 were jerky and overshot. The wrong 0.48 track put pivots below the wheel floor; this is fixed on master.
