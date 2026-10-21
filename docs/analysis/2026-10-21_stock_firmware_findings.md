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

### Our port

- No exposure control.
- A 0.5 m / 25° search gate is enforced.
- No lateral alignment before searching.
- Retries use the same pose.
- The robot stops when the marker is lost.

Planned fixes, in order:

1. Exposure control.
2. Drop the gate.
3. Retries with lateral offsets.
4. RTK lateral alignment before searching.
5. Kalman filter, and keep reversing when the marker is lost.
6. Pose check at 0.65 m.

All of these must keep our wheel floor (0.06 m/s) and the both-wheels rule.

The marker's black square should be measured; the config says 0.04 m while the physical marker is about 5.5 cm.

## Field symptoms these findings explain (2026-10-20/21)

- Dusk docking failed with "no usable marker", although the dock has a backlit marker. Without exposure control the camera cannot read it at dusk; stock varies the exposure.
- The mower stopped and got stuck at edges.
  - Turns sat too close to the edge, at the 0.14 m row ends.
  - Stereo marked grass as obstacles once the ground fit failed.
  - The stop zone stayed armed during turns.
  - The stuck guard counted collision-monitor stops as stuck.
- Turns before 2026-10-20 were jerky and overshot. The wrong 0.48 track put pivots below the wheel floor; this is fixed on master.
