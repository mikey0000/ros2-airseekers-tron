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
* **The bumper keepout.** The fault 1a bump created a pending keepout `(-31.28, -12.48)
  r 1.00` (0.11 m inside). After START it blocked all 8 route-corner sweeps of the
  retries. Stretch 2288-2329 was excluded and the mow ended MOWING_INCOMPLETE (99.3 %).
* **"follow_path aborted, collision ahead"** happened three times at (-29.26, -15.03),
  (-29.45, -14.62) and (-29.39, -14.45). Each was a collision-monitor `stop_zone` (stereo
  points 0.15-1.2 m high, 0.60-0.83 m ahead). The local costmap at the first abort had
  lethal clusters 0-0.9 m **inside** the line north-west of the robot. At the third there
  was a small cluster at the line. `stereo_depth` already drops points more than 0.05 m
  outside the area, so these lie inside the recorded line. They may be foliage hanging
  over it, or something else. They are not changed here (see owner decisions).
* Every skipped stretch was re-tried at the end of the area and again after each START.
  That is why the mower kept going back to that corner.

**Fix 4 (fsm/phases/nogo.py `_nogo_inside_perimeter`, param `nogo_edge_ignore_m` 0.25).**
The blocking consumers of the zone set are the pre-pivot sweep check (coverage pivots,
K-turns, route corners) and the plan-time mark exclusions. They now ignore a static
object (mark, candidate or pending zone) whose centre is outside the
allowed region or within 0.25 m inside its boundary. The owner recorded that line by
driving it, and lap 0 runs with base_link on it and the sides 0.235 m over, so nothing
can stand there. Person / animal zones and owner-accepted keepouts always count. The
edge-lap detour still sees these objects and bends the lap inward round them when a
detour fits. The collision monitor, the bumper, and the obstacle-guard decisions are
unchanged. Each ignored object is logged once: "... beyond the recorded perimeter, does
not block". With the field discs, every block above goes. The plant zone, the three plant
marks and the bumper keepout are all within 0.25 m of the line or outside it.

## Tests

* `test_body_boundary.py`: the field step geometry mirrored onto a square, with
  `test_join_from_a_vertex_whose_first_step_is_sideways_starts_at_the_approach_point`,
  `test_edge_start_beyond_the_gap_gets_the_approach_transit_not_a_diagonal_leg`,
  `..._gap_off_or_close_keeps_the_pivot_and_follow`,
  `..._gap_leaves_a_set_join_alone`,
  `test_no_final_approach_hop_onto_a_near_edge_approach_point`,
  `test_final_approach_hop_still_runs_beyond_the_hop_minimum`. The first, second and fifth
  fail on master.
* `test_nogo.py`: the field discs mirrored onto a square, with
  `test_static_discs_on_or_beyond_the_area_line_do_not_block`,
  `test_perimeter_filter_tolerance_and_off_switch`,
  `test_plan_time_exclusion_ignores_marks_on_the_area_line`.
* Sim world `src/mower_sim/worlds/owner_area3_edge1111.yaml`: the area 3 polygon, a 9 deg
  slope downhill west along the west edge, and two detector-visible potted plants 0.38 m
  outside the line.

## Owner decisions

1. **`nogo_edge_ignore_m` 0.25.** A small object lying on the edge line itself, such as a
   stone dropped there, no longer stops a pivot or skips an edge stretch. The lap bends
   round it if the edge detour fits; otherwise the bumper / stuck rules apply. Setting 0
   ignores only objects centred outside the line. Setting < 0 restores the old behaviour.
2. **The boundary spot rule** (`hard_boundary_spot_max` 2 within `hard_boundary_spot_m`
   1.0) has no time window. Both STUCKs came from a second excursion that was recovered
   in one move: the first excursion was 374 s earlier in one case and 46 min earlier in
   the other. A window, for example 10 min, is an owner call. It is not changed here.
3. **Collision-monitor stops at (-29.3..-29.5, -14.5..-15.0).** The points lie inside the
   recorded line. Please check the spot for overhanging foliage. No software change is
   made: these points come from the physical safety layer.
4. **Lap-0 joins on a side slope.** The cosine join (0.30 m over 1.5 m) plans the front
   corner up to 0.33 m out. The side-slope front limit is 0.35, and the tracking lag on a
   slope eats that margin. A longer join (about 3.3 m for an 8 deg merge) would fix it,
   but it costs edge cut at every lap start. Not changed.
5. **Pivot creep was 0.06-0.10 m/s today** on that slope (the 0.31 m slide during a
   30 deg pivot, with the wheels spinning at -0.7 rad/s commanded). The creep model
   assumes 0.045 m/s at 10 deg. Wet turf is likely.
