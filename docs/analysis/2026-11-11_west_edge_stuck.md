# West edge: turns out of the area and phantom edge blocks (2026-11-11, ws/edge1111)

Field mow 2026-11-11, area 3 ("further backyard lawn"), master 28d3450. Two
STUCK_NEEDS_HELP stops at the west edge between (-29.4, -14.9) and (-31.7, -11.7), and
repeated edge skips and retries at that corner. The edge there has a 8-10 deg side slope,
downhill to the west, which is out of the area. Sources: the mission log and the bag
(`/odometry/filtered_map`, `/cmd_vel_nav_raw`, `/cmd_vel`, `/tilt/status`,
`/collision_monitor_state`, `/local_costmap/costmap`, `/ai/det/detections_ranged`, the
published mow plan), measured against the area outline. Body points come from
`body_boundary` (front 0.60 m, rear 0.10 m, half width 0.235 m from base_link). "Out" is the
distance outside the line.

## Fault 1a: the turn at pivot vertex 786 went out, then the robot drove along the line

Sub-path 2 (lap at the 0.15 m inset) had poses 758 -> 786 skipped (fault 2). The transit
went to the approach point 0.30 m inside pose 786 at (-31.13, -11.86). Pose 786
(-31.42, -11.96) is a pivot vertex. The leg from it starts with the 0.15 m sideways step
east from the 0.15 m to the 0.30 m inset (poses 786 -> 789). After that the lap runs
north-north-west again.

| t (s, from 1794359753.8) | what happened (bag) |
|---|---|
| 0.0 - 4.1 | route-corner pivot -39 deg at (-30.63, -13.16). Spin at -0.7 rad/s commanded, but yaw only went 150 -> 120 deg. base_link slid 0.31 m north-west (downhill) -> SPIN DRIFT |
| 11.2 - 14.9 | "transit ended 0.08 m from the sub-path start: final approach (turn 69 deg, then straight)". The 8 cm miss was to the approach point. The turn took the nose from 84 to 154 deg, toward the outside, and the hop moved 0.1 m west |
| 15.0 - 20.2 | vertex pivot +104 deg (CCW, 156 -> -100 deg) through west. front_right reached 0.41 m out. base_link slid 0.32 m west (downhill, roll 7.7-8.1 deg) -> SPIN DRIFT, re-pivot +61 deg to -57 deg |
| 23.6 - 32.6 | follow_path: the robot drove **south** along the line (x -31.40, base_link 0.02-0.15 m inside), heading -57 -> -104 deg, front_right 0.35-0.51 m out. At 28.9 s the bumper hit at (-31.40, -12.39): the bumper was about 0.2 m outside the line. BOUNDARY OUT front_right 0.46 m at 32.2 s |
| 36.6 | recovery reverse arc: inside after one move. STUCK_NEEDS_HELP "left the boundary 2 times within 1.0 m": the earlier excursion was 374 s before, 0.9 m away (fault 1c) |

**Why the robot turned to the south.** `_transit_approach` places the approach point on
the inward normal of the start pose's own heading (109 deg, `geo.heading_of`), so the
point is east of pose 786. `_join_leg` (fsm/phases/mowing.py) then offset each leg pose by
the normal of the chord between its neighbours. On poses 786-789 that chord is the
eastward step (17-37 deg), so "inward" (the right side) pointed **south**, along the edge.
The joined leg started about 0.27 m south of the lap, at (-31.25, -12.22). The pivot gate
aimed at it with a +104 deg turn through west. The nose swept the outside and the pivot
slid downhill. MPPI then tracked the false leg south along the line, with the west front
corner over the edge on the side slope.

**Fix 1 (mowing.py `_join_leg`).** The join is now one vector: the inward normal of the
start heading, the same vector the approach point and every `m.join` producer use, faded
by the cosine. It starts exactly at the approach point and can never point along the
edge. With the field geometry the leg-start pivot from the approach point now faces
east and north (into the lap), the short way round, with the nose over the lawn.

**Fix 2 (transit.py `_route_final_approach`).** When the transit ended within
`edge_approach_hop_min_m` (0.15 m) of an edge approach point, there is no final-approach
hop. The point is only a waypoint inside, and the join starts there. The 69 deg outward
turn for 8 cm goes away.

## Fault 1b: the retry joined the lap diagonally at a bend and overshot outward

At the end of the area, the retry of the skipped stretch 758 -> 786 started 0.600 m from
pose 758 (-30.94, -13.24), with `transit_gap_m` at 0.6, so there was no transit. "pivot
-161 deg the long way round (199 deg) ... turn toward the interior": this pivot was
correct, and the robot faced 123 deg. MPPI then joined the lap diagonally. The robot
reached the lap's 34 deg right bend (poses 757-760, 135 -> 101 deg) at heading 140 deg.
An RTK float hold stopped it there for 5.4 s. On resume, `/cmd_vel` asked for -0.18 to
-0.27 rad/s, but the yaw only went 140 -> 130 deg over 0.27 m (roll -8 deg, downhill to
the outside). base_link ended 0.07 m outside the lap line, and front_left was 0.36 m out
against the 0.35 side-slope limit. A one-move recovery succeeded, then the spot rule
stopped the mow (the earlier excursion was 46 min before, 0.15 m away).

The same join pattern caused the other west-edge trips that day. One was sub-path 0's
join at pose 765, which ends at that bend: the robot held heading 148-153 deg against the
lap's 127, and front_left reached 0.47 m out. The other was a transit whose -77 deg final
approach spin slid 0.27 m west (front_left 0.40 m out).

**Fix 3 (mowing.py `_dispatch` / `_edge_start_needs_approach`).** An edge start (within
`edge_approach_near_m` of the line) more than `edge_start_gap_m` (0.40 m) away gets the
transit to its approach point and the tangent join. Before, it got a leg-start pivot and
a tracker leg across up to 0.6 m. There is no change when a join onto that start is
already set (the boundary rejoin, the end of an edge detour), and none at the approach
point itself (0.30 m).

## Fault 2: phantom blocks at the corner

* **Detector marks on the line.** There were 183 ranged `potted plant` detections in the
  west-edge box. Their signed distance to the line was median 0.38 m **outside**
  (range -0.72 .. +0.13). The tracker turned them into a zone `potted plant
  (-30.59, -13.83) r 0.56` (0.06 m inside) and marks (-30.58, -13.78) / (-30.59, -13.88)
  r 0.56 (0.10 / 0.02 m inside) and (-31.37, -13.34) r 0.80 (0.26 m outside). Their discs
  blocked every pre-pivot sweep at that corner: "(e) ... no clear alternative: skip the
  leg", the route-corner retries 1/3-3/3 with 3 / 10 / 30 s waits, and "edge blocked:
  skip 1.4 m (... 758 -> 786 ...)". That skip is what fault 1a's join started from.
* **The pending 1.0 m keepout.** `(-31.28, -12.48) r 1.00` (0.11 m inside) is the
  obstacle tracker's repeated-stuck-spot proposal: the mean of the two STUCK_NEEDS_HELP
  positions, (-31.37, -12.02) and (-31.19, -12.94). Both stops were boundary spot-rule
  stops, not obstacles; the one bumper contact there (fault 1a) was about 0.2 m outside
  the line. After START the keepout blocked all 8 route-corner sweeps of the retries.
  Stretch 2288-2329 was excluded and the mow ended MOWING_INCOMPLETE (99.3 %).
* **"follow_path aborted, collision ahead"** happened three times at (-29.26, -15.03),
  (-29.45, -14.62) and (-29.39, -14.45). Each was a collision-monitor `stop_zone` (stereo
  points 0.15-1.2 m high, 0.60-0.83 m ahead). The local costmap at the first abort had
  lethal clusters 0-0.9 m **inside** the line north-west of the robot. At the third there
  was a small cluster at the line. `stereo_depth` already drops points more than 0.05 m
  outside the area, so these lie inside the recorded line. They may be foliage hanging
  over it, or something else. They are not changed here (see owner decisions).
* Every skipped stretch was re-tried at the end of the area and again after each START.
  That is why the mower kept going back to that corner.

**Rejected first fix.** Commit 40a3a49 ignored static objects centred within 0.25 m of
the line. The sim (`owner_area3_edge1111` v1, plants r 0.2 centred 0.38 m out, near face
0.18 m out) proved it wrong: the lap-0 body, which overhangs 0.235 m, hit plant_south
twice and the mow ended STUCK_NEEDS_HELP. The base_link excursion of that run (0.238 m,
from t 432 s) started 3 s after the first contact, during the stuck escape; it was not a
tracking or slope fault. Perimeter objects are real and stay obstacles.

**Fix 4: perimeter objects at their true size, laps bend inward round them**
(fsm/phases/nogo.py `_nogo_true_perimeter`, `_nogo_edge_discs`).
* A static mark centred within `nogo_perimeter_band_m` (0.6) of the region's edge, on
  either side, is a perimeter object. The pivot sweep check and the edge detour use it at
  r - `nogo_perimeter_mark_shrink_m` (0.05): the detected half width plus 0.05 m, instead of
  the tracker's half width plus 0.10 m. Its position is not changed. Interior marks,
  zones and keepouts are unchanged.
* Edge laps now bend inward round perimeter marks (`nogo_edge_perimeter_marks`), blade on,
  by body reach + `nogo_edge_mark_clearance_m` (0.20) - face distance. The bend is clamped
  at `nogo_edge_mark_max_offset_m` (0.40 m), not dropped when the need is larger. It ramps
  in and out at 3 m per m of offset and holds the offset `nogo_edge_mark_along_pad_m`
  (0.5 m) longer on either side, because marks ranged from 2-3 m scatter along the lap. Each
  of these was added after a failed sim run (below). Before, the edge detour took zones
  only. Interior marks still stay out of it (their range scatter made trunk_x1's detour
  2 m wide).

**Fix 5: no repeated retries back to the same object** (fsm/phases/pivots.py
`_skip_piece`, fsm/holds.py `_stretch_blocked`).
* `static_skip_excludes` (true): a piece skipped for a static obstacle (a pivot sweep
  blocked by a static disc, a route-corner sweep-blocked skip, a bumper skip) is excluded
  at once (blocked_static, reason logged). It is not queued for the end-of-area retry.
* `skipped_retry_once` (true): any other skipped stretch (for example a collision-monitor
  stop) is retried once at the end of the area. If it is still blocked it is excluded
  with the reason ("still blocked on its one retry"). It is never queued again after a
  START.

**Fix 6: stuck-spot keepouts only from obstacles** (mower_map/obstacle_tracker_stuck.py
`stuck_event_spot`, obstacle_tracker_node.py). A STUCK_NEEDS_HELP within
`stuck_skip_window_s` (30 s) of a `boundary` incident is not filed as a stuck spot, because
there was no obstacle. A stop within `stuck_contact_window_s` (10 s) and 1 m of a bumper
contact is filed at the contact point (the bumper cloud, 0.4 m ahead of base_link), not
at base_link. Keepouts are never filtered by position.

## Review follow-ups (2026-11-11)

* **Slope switch grace** (fsm/guards/boundary.py `_hb_body_eval`). When roll crosses
  `edge_slope_roll_deg`, the front limit used to fall from 0.55 to 0.35 in the same tick,
  and a corner already 0.36-0.47 m out tripped at once (field STUCK #2; the switch was
  logged 4-10 ms before the trip). For `edge_slope_grace_s` (10 s) after the switch, the
  front corners keep min(0.55, their outside distance at the switch +
  `edge_slope_grace_grow_m` 0.05). Only a growing excursion trips.
* **Spot rule window** (fsm/guards/spin_drift.py). `boundary_spot_window_s` (600 s): only
  excursions this recent count toward "left the boundary 2 times within 1.0 m". In the
  field, the first excursions were 6 and 46 min earlier. **Owner decision**
  (2026-11-11): 600 s confirmed. <= 0 restores the whole-mission rule.
* **Join clamp** (mowing.py `_join_leg`). Where the lap turns 90 deg or more inside
  `edge_join_m`, the fixed join vector would lose its inward component (about 3 cm
  outward at 120 deg). Each offset is now clamped to be non-negative along the lap's local
  inward normal (chord over +-0.10 m). This also covers the boundary rejoin and the
  edge-detour rejoin, which use the same join.
* **Follow-up, not done.** The route-corner and final-approach spins do not use the
  interior-way choice (`_pivot_way`) or the slope pre-drive (`_slope_pivot`). The -77 deg
  final-approach spin at 1794360166 slid 0.27 m out. The new edge-start transit runs from
  ~16 `_dispatch` callers: watch the sim for extra transits.

## Tests

* `test_body_boundary.py`: the field step geometry mirrored onto a square, with
  `test_join_from_a_vertex_whose_first_step_is_sideways_starts_at_the_approach_point`,
  `test_edge_start_beyond_the_gap_gets_the_approach_transit_not_a_diagonal_leg`,
  `..._gap_off_or_close_keeps_the_pivot_and_follow`,
  `..._gap_leaves_a_set_join_alone`,
  `test_no_final_approach_hop_onto_a_near_edge_approach_point`,
  `test_final_approach_hop_still_runs_beyond_the_hop_minimum`. The first, second and fifth
  fail on master. Also `test_join_offset_never_points_outward_across_a_bend[90, 120]` and
  `test_slope_switch_itself_does_not_trip_only_a_growing_excursion`. The old slope test
  runs with `edge_slope_grace_s` 0.
* `test_hard_boundary.py`: `test_spot_rule_only_counts_excursions_within_the_window`.
* `test_nogo.py`: `test_perimeter_marks_keep_their_place_at_true_size_interior_marks_unchanged`,
  `test_edge_lap_bends_inward_round_a_perimeter_mark_the_body_would_touch`,
  `test_edge_lap_ignores_a_perimeter_mark_the_body_cannot_touch`.
* `test_mission_fsm.py`:
  `test_skipped_stretch_still_blocked_on_its_one_retry_is_excluded_not_requeued` and
  `test_sweep_blocked_skip_by_a_static_disc_is_excluded_at_once`. The 2026-11-01
  retry-pass tests run with both new flags off.
* `mower_map/test/test_obstacle_tracker_stuck.py`:
  `test_stuck_spot_after_a_boundary_excursion_is_not_filed_contact_goes_to_the_contact`.
* Sim worlds, both with the area 3 polygon and a 9 deg slope downhill west along the
  west edge:
  * `owner_area3_edge1111.yaml`: plants r 0.2 centred 0.6 m out (near face 0.4 m). The
    field evidence puts the real face beyond ~0.38 m: the lap-0 retry at 1794363861-877
    passed with the front corner 0.30-0.38 m out and no contact.
  * `owner_area3_edge1111_inside.yaml`: the same plants centred 0.38 m out (near face
    0.18 m), i.e. placed inside the lap-0 overhang after the line was recorded. It passes
    when the lap bends inward with no contact.

## Owner decisions

1. **`boundary_spot_window_s` 600 s**: confirmed by the owner 2026-11-11.
2. **Collision-monitor stops at (-29.3..-29.5, -14.5..-15.0).** The points lie inside the
   recorded line. Owner (2026-11-11): foliage probably overhangs there, and object
   avoidance should go around it without re-planning. Open: a collision-monitor stop
   should start a local detour on the inward side and rejoin the lap, not skip or
   retry the stretch. The sim test needs overhanging foliage passed without a stop.
3. **Lap-0 joins on a side slope.** The cosine join (0.30 m over 1.5 m) plans the front
   corner up to 0.33 m out, against the 0.35 side-slope limit. A longer join (about 3.3 m
   for an 8 deg merge) would fix it, but costs edge cut at every lap start. Not changed.
4. **Pivot creep was 0.06-0.10 m/s today** on that slope (the 0.31 m slide during a
   30 deg pivot, with the wheels spinning at -0.7 rad/s commanded). The creep model
   assumes 0.045 m/s at 10 deg. Wet turf is likely.
5. **Excluding stretches at once.** With `static_skip_excludes` and `skipped_retry_once`
   on, an object that was only there for a while (a toy, a hose) leaves its stretch
   unmown until the next mow, instead of being retried after each START. Either flag can
   be set to false.

## Sim verdicts (seed 2, -d -l, -x 2)

| run | world | result |
|---|---|---|
| edge1111_area3 (v1, commit 40a3a49) | plants 0.38 m out | FAIL: the perimeter filter ignored the marks; the lap-0 body hit plant_south twice -> STUCK |
| edge1111_area3_v2 | plants 0.6 m out (realistic) | FAIL (gate): no contact, no STUCK, no retry loop, MOWING_COMPLETE (lawn cut 98.7 %), docking still running at the 3600 s limit. One BOUNDARY OUT at (-31.79, -11.38): front_left 0.46 m at the lap-0 -> lap-1 inset-step vertex on the slope, where yaw drifted 102 -> 117 deg (outward) as the chunk ended at the vertex. The plants are 2.5 m away; this is slope tracking, not obstacle handling |
| edge1111_inside_v2 | plants 0.38 m out (inside the overhang) | FAIL: 0.28 m detour with 0.10 m clearance; an 18 deg yaw error on the slope met the pot -> STUCK |
| edge1111_inside_v3 | same | FAIL: no contact, but the detour came only beside the pot (need > cap) and aborted; route-corner pivot near the pot refused -> STUCK |
| edge1111_inside_v4 | same | FAIL: the detour came early (clamped), but the pot lay ~0.5 m later along the lap than its marks: met on the return ramp -> STUCK |
| edge1111_inside_v5 | same | FAIL: laps 0 and 1 now clear the pot by detour, but the lap-2 detour (0.40 m) aborted beside it. In the transit after the skip, the route-corner pivot near the pot ran "as planned" after its sweep retries and touched it (2 bumps) -> STUCK |
| real_run5_slope | | not run (the gate above did not pass) |

Open after these runs:
1. The route-corner gate pivots "as planned" when its sweep retries run out, even with a
   static disc in the sweep. It should skip or re-route instead.
2. Aborted edge-detour follows beside the object (the collision monitor stops MPPI on the
   bend).
3. The slope excursion at the lap-0 -> lap-1 inset-step vertex in the main world.

## Second round (2026-11-12): step round it, rejoin, carry on

Owner rule (2026-11-11, binding): foliage over the perimeter is expected, and an object at the
edge is avoided locally. The mower steps round it on the inward side, rejoins the lap past it
and carries on with the same plan. A skip, a re-plan, a retry loop or a STUCK is a failure; a
skip is allowed only when there is no room inside the line. Master 8e30cd5 is merged in.

### What changed

Mission (`fsm/phases/nogo.py`, `nogo.py`, `fsm/phases/pivots.py`, `fsm/holds.py`):
* **Detour aborted beside the object** (open item 1). `_nogo_detour_again`: the
  robot reverses along the detour poses it drove, then the lap before them. It bends again
  from the lap pose nearest the robot, using the local costmap's cells and the generic cap
  (`nogo_edge_max_offset_m` 0.60, or the costmap cap 1.50) instead of the 0.40 mark cap. This
  is done at most `nogo_edge_detour_again` (2) times per detour; after that the stretch is
  skipped as before. A person or animal (the dynamic policy) still skips.
* **Collision-monitor stop held on an edge lap** (open item 3). `_edge_block_tick` tries the
  costmap edge detour (back-up first when the cells are within 0.8 m) before the skip.
  `nogo_edge_costmap` is now **true**.
* **Route-corner sweep still blocked by a static object** (open item 2). After the
  maneuvers and the one re-route, the gate no longer pivots "as planned".
  `_nogo_route_reverse` reverses straight by up to `nogo_route_reverse_m` (0.60), with the
  body kept inside and the line behind it clear of the discs, then re-routes from there,
  once per corner. If there is no room, or the robot already reversed there, the transit is
  aborted without the pivot (its retries, then the sweep-blocked skip).
* **The mission's back-ups only reverse** (`_reverse_cmd`). In foliage v1, the again back-up's
  first target lay ahead of the robot, and `_track_cmd` drove forward into the next pot.
* **The interior side by depth.** `edge_detour(depth=...)` takes the side whose point 0.3 m
  off is deeper inside. On a lap 0.45 m in, both 0.3 m points are inside, so the old
  `inside` test took the left side, which was outward on a clockwise lap: "body 0.59 m
  beyond its limit", and the stretch was skipped (foliage v2).
* **The bend holds while a disc is beside the body.** The need is computed over the whole
  segment for every disc beside the body within the lookahead. Before, it stopped at the
  2.5 m lookahead (later at lookahead + 1.3 m), so the return ramp started 0.4-0.8 m before
  the pot and turned the nose 28-30 deg outward into it (inside v6 / v7: 13 and 17 bumper
  contacts). This probably also explains v4's "pot met on the return ramp".
* **The detour starts at the robot's inward offset** (`start_offset_m`), not on the lap. In
  foliage v3, a detour started while the robot merged out from 0.10 m inside, and the nose
  swung outward on the slope.
* **No detour from a far progress point** (`nogo_edge_detour_start_max_m` 0.8). If neither
  the lap point nor the detour's first pose is next to the robot, the detour is not started.
* **A detour that "succeeds" with more than `follow_end_tolerance_m` left** sends the rest
  (`_nogo_detour_resend`, up to `follow_premature_retries`). In foliage v1 / v5, a detour
  after a collision-monitor back-up "reached the goal" 10 ms after it was sent.
* **Perimeter-mark detours end `boundary_recover_margin_m` (0.10) inside the lap** and
  merge with the tangent join, as the costmap detours do. The end comes after the last pose
  that needs the bend, not after the peak (realistic v3, inside v9).
* The silent refusals are now logged: no costmap cells on the lap ahead (with their age), and
  the stretch already detoured.

Sim (`mower_sim`):
* **`solid: false`, `z_min`** obstacles. Stereo sees them (the local costmap and the
  collision monitor), but the wheels and the bumper pass through them. New world
  `owner_area3_edge1111_foliage.yaml`: the realistic world plus a clump of leaves, r 0.35,
  centred 0.12 m outside the line at (-29.60, -14.85). It hangs 0.15-0.55 m above the
  ground, where the field's collision monitor stopped three times.
* **`stereo_drop_outside_areas`** (default on, tolerance 0.05 m), as stereo_depth's
  `drop_marks_outside_areas` on the robot. Before, the sim's collision monitor and costmap saw
  pots beyond the line, which the robot does not see. That caused the "aborted beside the
  pot" stops of v3 / v5 (stop zone over the outside of a bend). Pots beyond the line are now
  seen by the detector marks only, as in the field. This changes every sim world with
  obstacles outside the areas.

### Sim verdicts (seed 2, `-d -l -s 2 -x 2`, `-t 4500` from inside v6)

| run | world | commit | result |
|---|---|---|---|
| edge1111_foliage_v1 | foliage | 29b4e62 | FAIL: the foliage stops were detoured, but the again back-up drove forward into plant_mid, which caused a STUCK escape and a BOUNDARY OUT rear 0.37 m; the run timed out at 3600 s (97.3 %) |
| edge1111_foliage_v2 | foliage | 1038283 | FAIL: lap 0 stepped round the foliage (again-detour, 0.71 m). On lap 2 the detour bent outward ("0.59 m beyond its limit") and the stretch was skipped. One BOUNDARY OUT rear_left 0.35 m on a pot detour; run timed out at 3600 s (92.3 %) |
| edge1111_foliage_v3 | foliage | interior side by depth | MOWING_COMPLETE, docked, no STUCK, no hard-boundary trip. Laps 0 and 2 stepped round the foliage, lap 1 needed no detour. One nose contact with plant_mid on a pot detour; an interior swath end at the foliage was skipped and excluded (2.0 m); 98.4 % cut |
| edge1111_inside_v6 | inside | start offset | MOWING_COMPLETE, docked, no STUCK, but 13 contacts with plant_south (return ramp before the pot) and 1 coverage-pivot skip, excluded (2.0 m) |
| edge1111_inside_v7 | inside | bend held to lookahead + 1.3 m | MOWING_COMPLETE, docked, 17 contacts (the ramp still began 0.4 m before the pot); again-detours cleared it each time; 0 skips |
| **edge1111_inside_v8** | inside | 4575702 | **MOWING_COMPLETE, docked, 0 contacts (min clearance 0.188 m), no STUCK, no skip, no exclusion**, 98.3 % cut. One BOUNDARY OUT front_left 0.35 m on the lap-0 detour (on the slope), recovered in one move. base_link 0.167 m out at the lap-0 -> lap-1 step vertex (item 4) |
| edge1111_foliage_v4 | foliage | 4575702 | FAIL: on laps 1 and 2 the foliage collision aborts skipped with no logged reason (2 stretches excluded, 4.1 m), and 1 contact with plant_mid. On the way home a route-corner Spin at the foliage was refused 3 times -> STUCK_NEEDS_HELP |
| edge1111_foliage_v5 | foliage | refusal logging | FAIL: every foliage meeting was stepped round (laps 0, 2, and a held collision-monitor stop), but one detour "reached the goal" at once and the lap then hit plant_south 3 times (bumper skip, excluded). STUCK_NEEDS_HELP on the way home: route-corner Spin refused 3 times at (-29.40, -13.83) |
| **edge1111_foliage_v6** | foliage | detour resend | **MOWING_COMPLETE, docked, 0 contacts (min clearance 0.074 m), no STUCK, no skip, no exclusion**, 98.8 % cut. Every foliage meeting was stepped round: on lap 0 by the again-detour (0.71 m), on lap 2 by two held collision-monitor stops (back-up, then a detour of 0.46 / 0.31 m). Two BOUNDARY OUT trips, each recovered in one move and the lap resumed: front_left 0.55 m at (-28.80, -15.29) on lap 0 on the slope, and nose 0.41 m at (-30.01, -14.17) on lap 2 right after a detour rejoin |
| edge1111_area3_v3 | realistic | detour resend | FAIL: the longer mark detour (0.28 m) returned onto lap 0 on the slope and put front_left 0.35 m out, twice at the same spot -> spot-rule STUCK_NEEDS_HELP at 545 s. 0 contacts |
| **edge1111_area3_v4** | realistic | mark detours end 0.10 m inside | **MOWING_COMPLETE, docked, 0 contacts, 0 hard-boundary trips, no STUCK, no skip**, 98.7 % cut. Gate FAIL only on a (0.187 m at the step vertex, item 4), e, h, i, j |
| edge1111_inside_v9 | inside | as area3_v4 | FAIL (owner rule): a mark detour ended in a dip between two scattered marks, beside the pot; the lap-2 join drove into it (3 bumper hits, bumper skip, 2.0 m excluded). MOWING_COMPLETE, docked |
| edge1111_inside_v10 | inside | the end waits for the last disc | FAIL: a detour "succeeded" 3 times with 1.1 m left (re-sent each time, the robot did not move); the rejoin then put the nose 0.41 m out next to the pot, and 4 boundary recovery moves (wheel slip on the slope, 3 contacts) failed -> STUCK_NEEDS_HELP at 955 s. Bag kept |
| real_run5_slope | | | not run (the edge worlds do not pass) |

No single commit has passed all three worlds yet. Foliage v6 and inside v8 met the owner
criterion before the last two detour-end changes, and realistic v4 met it after the first
of them. Inside v10, on the final code, failed.

### Against the owner rule

(v6 and v8 ran before the last two detour-end changes; inside v10 on the final code failed.)
* **Foliage (v6):** passes the owner criterion. The mower went round the overhang on every
  lap with no stop-skip and no STUCK, and mowed the stretch (blade on during the detours).
* **Pot inside the overhang (v8):** passes the owner criterion. No contact, no skip, no
  exclusion, no STUCK.
* Neither passes the sim gate. BOUNDARY OUT on the slope (recovered), item 4's step-vertex
  excursion, and the gate's i / h / e criteria all still fail.

### Still open

1. **BOUNDARY OUT on the slope around detours** (v6: 2, v8: 1, all recovered in one
   move). A detour ramp or rejoin on the 9 deg side slope swings a front corner out. The
   slope-aware pivot / tracking follow-up above is not done.
2. **Item 4, the lap-0 -> lap-1 step vertex at (-31.83, -11.35)**: 0.10-0.22 m base_link
   excursion in every run of this round, with the same yaw drift as v2. Not addressed. The
   master baseline on this world has not been run (one sim at a time; the edge worlds took
   priority).
3. **Route-corner Spin refused next to costmap cells** (foliage v4 / v5 on the way home:
   "route corner pivot aborted 3 times" -> STUCK_NEEDS_HELP). The sweep gate only knows
   zones and marks, not costmap cells, so Nav2's Spin refuses and the existing back-off
   path runs out. Not seen in v6; the new route-corner reverse covers static marks only.
4. **Foliage world, run to run.** v3 / v6 pass the owner rule, v4 / v5 did not (fixed since:
   the silent refusal is now logged, the premature detour success is re-sent). One seed
   only; more seeds are needed before the field.
5. **An interior swath end under the foliage** (v3, v5) is skipped by the interior detour and
   excluded (2.0 m). That ground lies under the leaves; the owner rule covers the edge
   laps. Owner call whether that is acceptable.
6. **The detour follow "succeeds" at once** (foliage v1 / v5, inside v10). Re-sending the
   rest (`_nogo_detour_resend`) does not help: in v10 it "succeeded" three more times with
   1.1 m left and the robot did not move. The cause is in the controller / goal checker
   (`coverage_goal_checker` is a stateful SimpleGoalChecker, xy 0.25): open. Next step:
   read the v10 bag (kept) for the sent path against the robot pose.
7. **Detour ends** (realistic v3 vs inside v9 / v10). Ending on the lap swings the front out
   on the slope, and ending 0.10 m inside cut the detour short between scattered marks. The
   last-disc rule is untested beyond v10.
8. The sim now drops stereo points outside the areas (as the robot does). Baselines of other
   worlds with obstacles beyond the line will change.

