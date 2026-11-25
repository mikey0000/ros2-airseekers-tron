# Sub-path start approach: cross-track at index 0 (2026-11-25)

Branch `ws/approach1125` from master 2ca8d8a. Unit-tested and run in the sim; not field tested.

## 1. The failure

`real_run5_cross60` (`2026-11-12_cross_single_plan.md`, Sim results) failed criterion (h): the
MOWING line cross-track p99 was 0.318 m. The tail was at index 0 of short sub-paths and showed up
in both run 1's sub-paths and run 2's. A master run today (2ca8d8a, `-l -x 3 -s 1`) gives the same
result: h line p99 0.334 m, max 0.683 m.

## 2. What (h) and (i) measure, and whether that is fair

- (h): the ground-truth distance to the sent sub-path, in a window of ±1.5 m around the
  mission's published `current_path_index`, at 25 Hz in MOWING. Pivots in place are counted
  separately, and lines, arcs and corners must each have p99 <= 0.10 m.
- (i): the body overhang beyond the soft limit, minus the overhang of the planned pose being
  tracked, which must be <= 0.10 m in MOWING. TRANSIT samples are judged on the absolute
  overhang, which must be <= 0.40 m.

The evaluator is fair for both. I split the (h) samples over 0.10 m by sub-path and index
(a copy of the (h) loop with a dump added):

| Where | Samples > 0.10 m | Cause |
|---|---|---|
| Sub-paths 7, 16 and 18, index 0 | 423 (231 over 0.30 m) | The robot was 0.58 m from the sub-path start, under `transit_gap_m` 0.6, so there was no transit. The leg-start pivot was made where the robot stood, aimed 1.5 m down the leg, and the tracker cut in. Cross-track was 0.65 m at the start and fell to 0.15 m over the first 1.2 m. The start was not cut. |
| Sub-path 10, index 130-143 while the robot was at 68-95 | 250 (0.15-0.30 m) | The sub-path crosses itself at (-4.21, 0.37). `nearest_index` searches 3 m ahead and took the later pass, so the published index jumped 67 to 130. (h) then judged the robot against the wrong stretch. The robot tracked within 0.03 m. This is a mission bug, not an evaluator bug: a hold there would have resumed at index 130 and left 3 m unmowed. |

On run5, (i) failed on master in two ways that differ by seed and branch:

- `simout_strip1113/run5`, sweep 0.134: a lap-0 pivot vertex at (-0.81, 5.37). The pivot
  heading was computed when the leg's along-path goal checker fired, with the robot still
  rolling 0.07-0.09 m short of the vertex. Aimed from there at the next vertex 0.33 m on, it
  was -54 deg instead of -71 deg. That is under the 20 deg "follow anyway" limit, so the
  robot followed and was 0.10 m beside the short leg at the next vertex. Localisation error
  was 0.01-0.03 m.
- `simout_rtkdrop1113/run5` (identical to master in `src`), side 0.105: lap 0 on the convex
  arc near (-0.53, 4.55), with MPPI running 0.08 m outside the arc. This is the known lap-0
  convex-bend issue in `STATUS.md` and is controller-side. This change does not address it.

Cross-track on the transit's final approach and on MPPI's start-of-path behaviour were ruled
out as sources of the (h) tail. MPPI only does what the leg-start pivot sets up for it.

## 3. The fix (`mower_mission`)

1. **Leg-start approach** (`TransitMixin._leg_start_approach`, query `_leg_start_unreached`):
   - When it applies: a sub-path's own start (`start_local` 0) within `transit_gap_m` but
     more than the new parameter `leg_start_reach_m` (0.12 m) from the robot.
   - What it does: the existing final approach hop (`_route_final_approach`: Spin to face the
     start, DriveOnHeading onto it, or reverse along the first leg on the line), then the
     leg-start pivot on the start.
   - Excluded:
     - the robot is behind the start on the line of its first leg (the tracker drives
       straight on);
     - reverse legs, K-turn legs, and the leg after a turn leg (the turn-leg caps end it
       short on purpose);
     - edge-lap starts within `edge_approach_near_m` of the boundary (approached from inside
       by `_transit_approach` / `_join_leg`);
     - resumes inside a sub-path.
   - Logged as `robot X m from the sub-path start: final approach onto it`.
   - `leg_start_reach_m` 0 reverts it.
2. **The final approach Spin turns the interior way** (`_pivot_way`), as coverage pivots do.
   The hop now starts at row ends near the edge.
3. **Coverage pivots are aimed from where the robot stops** (`PivotsMixin._pivot_pose`). The
   pose is moved along its yaw to abreast of the leg start, at most `leg_start_reach_m`, and
   only forward. It is used for the join heading and `_pivot_way`.
4. **Progress keeps the earlier pass** (`geometry.nearest_index(..., tie=)`, used by
   `_track_progress` with `PASS_TIE_M` 0.15). The index is the first local minimum of the
   distance within 0.15 m of the nearest. Where the path crosses or runs back beside itself,
   progress stays on the current pass.

## 4. Tests

- `test_geometry_resume.py::test_nearest_index_tie_keeps_the_earlier_pass_at_a_crossing`
- `test_path_conditioning.py`:
  - `test_leg_start_beyond_reach_is_a_final_approach_hop_not_a_pivot_where_the_robot_is`
    (0.4 m gap: TRANSIT, Spin +90, DriveOnHeading 0.4, then the leg-start pivot on the
    start);
  - `test_leg_start_within_reach_pivots_in_place` (0.1 m);
  - `test_leg_start_reach_zero_is_the_old_pivot_in_place` (revert);
  - `test_pivot_pose_rolls_on_to_abreast_of_the_start_within_reach`;
  - `test_leg_start_pivot_is_aimed_from_where_the_rolling_robot_stops` (0.08 m short: the
    pivot is within 1 deg of the leg, against 10 deg before).
- The harness in `test_mission_fsm.py` now accepts a DriveOnHeading with the blade on during
  an in-area transit (`blade_transit`). The post-transit final approach already did this in
  the sim. The hop is the first unit test to drive it.

The four behaviour tests and the tie test fail on master's code. On the host, the mission
suite fails only the three rclpy `test_external_rain` node tests, which fail on master too.
`scripts/ci_test.sh`: colcon, scripts and replay all PASS (3972 test cases, 0 failed).

## 5. Sim (`-l -x 3 -s 1 -t 4000`, outputs in `simout_approach1125/`, bags deleted)

| Criterion | real_run5 master (`simout_rtkdrop1113/run5`) | real_run5 branch | cross60 master (`master-fixed60`) | cross60 branch |
|---|---|---|---|---|
| a | PASS 0.096 | PASS 0.087 | PASS 0.078 | PASS 0.077 |
| b, c, d | PASS | PASS | PASS | PASS |
| e | PASS 0.118 | PASS 0.138 | PASS 0.123 | PASS 0.121 |
| f | PASS | PASS | PASS | PASS |
| h line / arc p99 | PASS 0.067 / 0.065 | PASS 0.086 / 0.052 | **FAIL 0.334** / 0.052 | **PASS 0.091** / 0.051 |
| h line max | 0.118 | 0.138 | 0.683 | 0.149 |
| i side / nose / sweep; transit | FAIL 0.105 / 0.081 / 0.077; 0.374 | **PASS 0.076 / 0.070 / 0.099; 0.365** | FAIL 0.085 / 0.062 / 0.127; 0.354 | FAIL 0.105 / 0.050 / 0.123; 0.370 |
| Verdict | FAIL (i) | **PASS** | FAIL (h, i) | FAIL (i) |
| Lawn cut (ground truth) | 97.7 % | 97.6 % | (not compared) | 98.4 % |

On cross60 the hop fired 3 times, at 0.54-0.56 m, the gaps that caused the old tail. It did
not fire on run5.

What is left of (i) on cross60:

- **Sweep 0.123, sub-path 4 index 0.** A transit's final approach (an old path) was decided
  with the robot still rolling. It was aimed 43 deg from a pose 0.08 m back, ended 0.11 m
  beside the start, and the 150 deg leg-start pivot swept there. This is the same
  stale-pose mechanism as fix 3, in `_route_final_approach`'s Spin. Master's worst sweep
  (0.127, sub-path 16 index 0) was the leg-start case that this branch now hops.
- **Side 0.105, the lap-0 convex arc at (-0.53, 4.5).** This is the controller-side bend
  that master's run5 failed on.

Next: aim the final approach Spin from `_pivot_pose` too, and rerun both worlds.

STATUS: index-0 approach fixed (cross60 (h) 0.334 to 0.091, run5 PASS). The cross60 (i)
remains: a transit final-approach Spin aimed from a rolling pose, and the lap-0 convex arc.
