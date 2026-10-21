# Grass false obstacle stops: cause and hotfix (2026-10-21)

In field session 2026-10-21 (01:24 UTC, Area 6, master ed78907) the mower repeatedly stopped or reported STUCK near edges. The owner reported that nothing was actually in the way: the mower stopped in front of grass while turning, and there was no stone in the path. The owner was right; the first diagnosis ("a stone and a tree just outside the line") was wrong. The data is in `ros2_stack-wt/accuracy-simout/field_stuck_2026-10-21b/`.

## Cause

The following are high-confidence findings unless marked otherwise.

- **Stereo ground fit failed over thick grass.** All 125 collision_monitor stop_zone events came from the stereo source; the bumper was silent.
  - About 85% of them happened while the stereo ground fit was failing (fit_ok 0). Over thick grass the visible "ground" is the grass top, which sits more than 0.08 m (`ground_max_dh_m`) from the prior plane.
  - When the fit failed, stereo_depth fell back to the fixed prior plane (h 0.24) with a 0.12 margin and marked the grass as obstacles: 600–1400 points per frame, against 100–250 on lawn.
  - It also stopped clearing.
  - The fit failed about 50% of the session, more on slopes (pitch up to 13°, beyond `ground_max_tilt_deg` 8).
- **Stop zone armed during turns.** `collision_zone_gate` armed the stop zone whenever v ≥ 0.08 and ignored w.
  - Slow arcs and the first command after a Spin (the idle re-arm after 1 s) therefore swept the stop box over grass.
  - `approach_footprint` was never gated. It crawled the robot to v≈0.02, which `cmd_vel_slew` turned into ±0.3 pivots flipping every 0.2 s.
- **Spin "Collision Ahead" came from stale lethal cells beside the nose.**
  - The cells sat 0.57–0.69 m away at bearings −18° to −71°, outside the area line.
  - ObstacleLayer clears only by raytrace, so cells the camera can no longer see never clear.
  - Spin used `simulate_ahead_time` 2.0.
- **The detector produced false "stone" detections,** including one 0.33–0.52 m ahead, which is under the deck.
- **The mission stuck guard counted collision-monitor-zeroed commands as effort,** so it reported false STUCK. This is fixed separately on `ws/edge-layout`.
- **Not a 3f08b88 regression.** The camera TF and height did not change, and the same failures appear in the 17–18 Oct logs.

## Hotfix (`ws/obstacle-hotfix`, on master ed78907)

- **0c5c17c**, cherry-picked from edge-layout 7a44fe3:
  - new package `mower_costmap_layers::DecayingObstacleLayer` (5 s decay) for the stereo and det marks;
  - stereo_depth reuses the last good ground plane for up to 3 s, after which only points more than 0.25 m above the prior mark;
  - `ground_max_tilt_deg` 12;
  - obstacle points outside areas and paths are dropped (via `/keepout_mask`).
- **c2ce56d:**
  - stereo min height 0.15 for collision_monitor and both costmaps, and `ground_margin_m` 0.15 (stock values);
  - `approach_footprint` uses the bumper only;
  - `collision_zone_gate` arms the zones only when v ≥ 0.08 and |w|/v ≤ 1.0, and keeps them off during Spin/BackUp/DriveOnHeading and for 2 s after;
  - Spin `simulate_ahead_time` 0.5;
  - det_range marks only named classes with score ≥ 0.5.
- **Map tagging is kept.** The obstacle tracker reads the same cleaned cloud, so it gets the same filtering.

## Trade-offs to remember

- Obstacles 0.08–0.15 m tall are no longer seen by stereo; the bumper still catches them. The 2026-10-06 box test relied on 0.08.
- Stereo no longer feeds the approach slowdown. Forward stereo protection is the stop and slowdown zones only, and those are off in tight arcs (|w|/v > 1, r < 1 m) and pivots. Pivots and tight arcs rely on the bumper.
- **The sim cannot test grass.** It generates ground-truth clouds and does not run stereo_depth, so the field run is the real test.

## Validation

| Gate | Result |
|---|---|
| Unit tests | 212 pass |
| real_run5 ideal | PASS; line / arc 0.043 / 0.032 (master 0.034 / 0.035) |
| real_run5 `-d -s 1` | Complete, docked, 0 trips, but one TRANSIT corner at 0.205 m (master: 0 over). Likely disturbance variance, because no obstacle logic can act in an empty world. A second seed was run to check. |

A cross-track jump to 0.604 seen on the edge-layout base was variance in edge-layout's pivot sequencing at a sub-path hand-off (the edge-layout resume run shows 0.608 too), not this fix.

## What to compare against stock

Stock never held camera marks in its costmap: the layers are commented out. It uses labelled YOLO detections plus the bumper, and turns AI avoidance off on edge laps. See `2026-10-21_stock_firmware_findings.md`. Moving fully to that strategy (back up → plan around → rejoin for labelled obstacles) is the follow-up.

## Field run 2026-10-21 evening

This run was session `20261021-062832` (06:28–07:50+ UTC, Area 6), deployed 7532831, which is ed78907 plus this hotfix. The data is in `ros2_stack-wt/accuracy-simout/field_mow_2026-10-21c/`. Bag splits 41 and 49 are corrupt on the mower (bad zstd), so 07:09:44 comes from the logs only. The docker json logs are corrupt as well; the node logs that are readable are `/userdata/ros2/log/<node>_<pid>_<ms>.log`.

**The grass fix worked.** The table compares this run with the morning run (about 22 min driving).

| Metric | Morning | Evening (72.7 min driving) |
|---|---|---|
| stop_zone events | 125 (about 340/h) | 0 |
| Spin "Collision Ahead" | 18 | 0 |
| slip_detector STUCK | 6 | 1 (a real wheel-spin while reversing in a detour, 06:37) |
| STUCK_NEEDS_HELP | 11 | 2 (both from the boundary budget, not from obstacles) |

In the evening run, one detour (06:36) came from a stereo block at the area line. Only the slowdown zone fired, 13 times.

**The remaining stops were not grass:**

- **06:31:27, the 23.37 m "outside" event, was a GUI map save during the mission.**
  - `PUT /api/mowglinext/map` at 06:31:27 does "All map layers and areas cleared", then re-adds the areas one by one.
  - The boundary check ran when only Area 1 and the lawn section existed. The robot was in fact 0.36 m inside.
  - The only edit was one Area 6 vertex dragged along the edge (area +0.01 m²).
  - It still cost a retrace, used boundary budget 1/2, and discarded the resume cursor at the 07:06 restart, so the area was re-mowed from 0 %.
  - The keepout mask was also published half-built (nav free cells dropped to 237).
- **07:05:32 (TRANSIT, 0.10 m): a 64° route-corner Spin slipped.**
  - The Spin took 12.7 s, with the wheels at w≈0.28 and the EKF at about 0.085 rad/s. RTK was fixed while the robot slid 0.40 m west to the line.
  - The robot then followed the leg out.
  - The same west-edge spot (-33.5, -6.4) caused the morning stuck and abort events there.
  - This trip was 2/2, so the run went to STUCK_NEEDS_HELP.
- **07:09:44 (MOWING, 0.10 m): edge-ring tracking error at a corner, base_link-only check.** This is the known issue; the bag is missing.
- **07:29:02 and 07:40:23 follow_abort, then 07:29:11 (0.13 m, 2/2 → NEEDS_HELP) and 07:40:42 (0.10 m) boundary trips: a real physical obstacle just outside the line at (-27.8, -4.1).**
  - On rows ending perpendicular to the edge (heading about 125°), base_link reached the line (−0.04 to −0.08 m) and the nose, about 0.35 m beyond, hit the obstacle.
  - The bumper fired 6 and 9 times in turn, each followed by a 2 s back-off and a retry, until the controller reported "Failed to make progress".
  - The retry then tripped the 0.10 m check.
  - The costmap was empty: the camera marks outside the area are dropped by design.

**Safety notes:**

- The hotfix caused no regression.
- During the 4 "cat ahead" stops (score up to 0.97, the cat filling the frame), `_soft_blade_off` kept the blade running (blade_policy continuous, since 397fdf8). The log line still says "blade off". Living-obstacle stops should force the blade off.
- The startup EMERGENCY (06:27:58) was the mission node's `/hardware_bridge/emergency silent > 2 s` watchdog firing before the first MCU message. The robot was docked and the emergency cleared in 0.8 s.

**Fixes, mapped to step 3 round 2:**

- Use the stock edge layout so that row ends do not point the nose at the line.
- Make the boundary-out check body-aware, with the reverse-0.3 m recovery. Also skip a stretch after N bumper hits at the same spot.
- Gate Spin progress, and skip or replan a corner when the robot drifts more than 0.15 m during a pivot.

**Fixes outside step 3 round 2:**

- Make map replacement atomic in map_server: build, then swap. Alternatively, the mission ignores boundary checks for 2 s after `areas cleared`.
- Do not count a trip toward the 2/2 budget when outside_m jumps by more than 1 m in one tick.
- Arm the emergency watchdog only after the first message.
- Turn the blade off for dynamic (animal or person) stops.
