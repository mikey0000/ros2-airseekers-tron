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
  field, the first excursions were 6 and 46 min earlier. **Owner decision**: the default
  of 600 s was proposed to the owner and is not yet confirmed. <= 0 restores the
  whole-mission rule.
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

1. **`boundary_spot_window_s` 600 s** (see above).
2. **Collision-monitor stops at (-29.3..-29.5, -14.5..-15.0).** The points lie inside the
   recorded line. Please check the spot for overhanging foliage. These stretches now get
   one retry and are then excluded, with the reason in the log.
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
