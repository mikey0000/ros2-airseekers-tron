# Narrow strips: detector, shuttle planner, plan-quality gate (2026-11-13)

Branch `ws/strip1113` from master 8e769e7. Unit-tested and run in the sim; not field tested.

## 1. The field mow

Area 4 (map area 1) is a strip 6.3 m long. It is 0.72 m wide at the west end and 0.48 m at
the east end, its long axis runs at about 17.7 deg, and it covers 3.76 m2. The south long
side is the owner's drop; the mapped south edge is wheel-safe (there is room between it and
the drop). The north side is a driveway. The short ends are lawn.

What happened on 2026-11-13 (zigzag, edge laps 3, laps 2, inset 0.05, racetrack, slope_mode
updown at 17.74 deg):

- The soft edge layout puts laps at 0 / 0.15 / 0.30 m and rows 0.45 m in, so rows need a
  width over 0.90 m. Fields2Cover kept lap 0 only. After conditioning it was 6 fragments, 13 m
  in total, with 0.1-0.2 m end turnarounds needing K-turns. A 0.9 m stretch of the north
  edge was missing and the centre was not covered.
- `/coverage/plan_quality` said coverage_fraction 0.315. Nothing in the mission reads that
  topic, so the plan ran.
- On the strip: a collision-monitor skip on the north edge, 7 pivots at the east tip,
  STUCK, a tip sub-path "NOT mowed" (its transit goal was off the route graph), and a
  transit BOUNDARY OUT with the nose 0.48 m out.
- The mowed raster says 46 % was cut. MOWING_INCOMPLETE / COMPLETE said "98.8 % mowed",
  which is the path-length share.

## 2. What changed

### Detector (`mower_coverage_bridge/strip.py`)

An area is a narrow strip when all three hold:

| Test | Threshold | Why |
|---|---|---|
| largest inscribed circle radius | < 0.385 m | a pivot sweeps a circle of radius 0.645 m about base_link; 0.645 - 0.26 of soft overhang leaves 0.385 |
| length / width (minimum-area rectangle) | >= 3 | a stubby narrow area is no strip |
| length | >= 1.4 m | passes need room; a shorter narrow area meets the plan-quality gate instead |

On the real map files it fires on Area 4 only: inscribed radius 0.35 m, rectangle
6.28 x 0.85 m. It stays off on every mowing lawn of `real_run5_areas.dat`,
`real_run5_smooth_areas.dat`, `owner_far_areas.dat` and the 2026-11-13 live map (inscribed
radius 0.45 m and up). The navigation paths (Dock path, Path 2) would also count as narrow,
but they are never coverage-planned.

### Strip planner (`strip.py`, coverage bridge)

- **Passes.** The candidate axes are:
  - the minimum-area rectangle's long side;
  - the two long sides' own directions;
  - the goal's mow angle, when it is within 20 deg of the axis.

  Each axis is tried entered from either end. Base_link stations are scanned every 0.02 m
  on lines 0.01 m apart. Each line's longest run of valid poses, trimmed to where the blade
  still reaches the area, is a candidate. Only lines whose run starts within 0.5 m of the
  earliest start are kept, so every pass begins at the entry end. Passes are picked greedily
  by the new blade cut they add on a 0.02 m raster, so the spacing follows the taper.
- **Body limits.** Every planned pose keeps the six body points of the boundary guard
  within the soft trip limits less a tracking margin: nose 0.28 (trip 0.40), tail 0.15
  (0.30), front corners 0.40 (0.55), rear corners 0.20 (0.35). The tail and rear corners
  were 0.20 / 0.25 until 2026-11-25 (section 5). Base_link itself stays
  inside the outline (the guard trips it at 0.10 out: a pass start on the slanted west end
  put it 0.11 out in the second sim run). Both long sides are ordinary
  soft edges: the owner confirmed that the usual boundary behaviour applies. The ends are
  lawn, and the same limits let the nose and tail run a little past them. Holes: the
  footprint never touches them.
- **Shuttle, no pivots.** Each pass is driven forward to its far end and then in reverse
  straight back along the same line.
  - The next pass starts with a forward lane change: a cosine S from the current line onto
    the next one. It uses the shortest of 1.2 / 1.5 / 2.0 / 1.0 / 0.8 m whose poses all
    keep the limits, started up to 1 m later if needed. A pass no lane change reaches is
    left out.
  - The reverse leg retraces the pass's forward track. Where the pass's line starts before
    the end of its lane change, the leg runs on along the same line to the pass's own start.
    That stretch was not driven forward, but it is straight, on the planned line, and every
    pose of it keeps the limits.
  - After the last pass a longer lane change (1.5-3.0 m) returns to the first pass's line,
    and a reverse leg along it ends the plan exactly where it began. That is the pose the
    inbound transit reached, so the route graph is reachable from it. In the third sim run,
    the plan ended on the far long side in a corner the route graph does not reach. The
    in-area join transit then pivoted at the corner and tripped the boundary guard twice.
    Plans that end where they began are preferred over every other criterion.
- **Choice.** The plan with the most cut wins; plans within 3 % of each other are decided by
  the wider entry end (room to arrive and leave), then by the shorter drive.
- **Output.** The pieces reuse the existing reverse sub-path machinery:
  - one point chain with reverse flags is cut by `splitter.split_reverse` into forward and
    reverse sub-paths that share their cusp poses, so the mission sees no transit;
  - reverse poses face against the travel (`splitter.yaws`), and the mission drives them with
    `FollowCoveragePathReverseMPPI`;
  - each pass's straight part is a swath segment;
  - the message starts `strip plan:`;
  - `/coverage/plan_quality` carries the raster cut fraction and `strip: true`.

  If no pass of at least 0.7 m fits, the goal is refused with the reason.

Area 4 (unit test, the map polygon): 5 passes of 4.0-5.9 m at 17.7 deg, entered from the
west (wide) end, 12 sub-paths (6 forward, 6 reverse), 99.7 % of the strip under the
planned blade path, 59 m of drive, 1.8 s to plan on the dev host.

### Plan-quality gate and fragments (coverage bridge)

- **Gate.** A Fields2Cover plan with a `coverage_fraction` under `plan_coverage_min` (0.70)
  is refused, unless a strip plan cuts more. The goal is aborted with the reason, e.g.
  "plan covers only 31 % of the area (minimum 70 %): area not mowed; no strip plan fits:
  ...". The mission's existing planning-failure path takes over: AREA_UNREACHABLE, the next
  area, and MOWING_INCOMPLETE naming "area N planning failed: <reason>". A fragment plan is
  never driven.
- **Threshold.** The lawns plan 0.93-0.99: real_run5 Area 1 0.952, heart 0.968, Area 5
  0.930, Area 6 0.986, and on the live map further / closer backyard 0.992 / 0.994 and
  Clothesline 0.983. Area 4 plans 0.317.
- **Fragments.** `min_fragment_m` (0.70) leaves out a forward sub-path shorter than about one
  body length that joins neither neighbour, i.e. a scrap needing a transit both ways
  (`splitter.drop_fragments`, named in the summary). It drops nothing on any current lawn.
  Area 4's field fragments came from the mission's conditioning; the strip plan replaces that
  plan.
- **Parameters.** `strip_mode` (auto / on / off), `plan_coverage_min` and `min_fragment_m`
  are coverage_server parameters with these defaults. There is no new area setting and no
  GUI change.

### Mission (`mower_mission`)

- **Driven as planned.** A strip plan (`is_strip_plan`: the bridge's tag, also inside a
  merged multi-run message) skips path conditioning. `join_turn_chains` would otherwise join
  a short pass, its retrace and the next pass into one forward path with a connector.
  Fillets, pivots and K-turns would add the turns the planner avoids.
- **Edge-lap rules off.** `_edge_lap` is false on a strip plan. Every pass runs near the
  outline, and the edge-lap skips (`_edge_skip`) would cut a pass short.
- **Obstacles.** The owner's rule: "obstacles will have to stop the mower if it cannot go
  around within the area".
  - The normal local detour is tried first: `_try_detour`, with its inside-the-area check.
  - When no detour fits, the mower holds through the existing blocked / dynamic waits, with
    the blade per the continuous-blade policy.
  - If the obstacle is still there, the mower stops in STUCK_NEEDS_HELP (`_no_detour`)
    instead of giving the sub-path up.
  - A detour target past the end of a strip pass is not taken as "skip the rest of the
    pass".
  - The pass is never cut short, nothing is re-planned, and START resumes the pass.
  - Normal plans are unchanged: the sub-path is NOT mowed and the mow goes on.
- **Dock join in a narrow area.** When the route graph cannot be reached from where the
  mow ended, the dock join drives to the inbound route's entry into the area: its first
  pose at least `dock_join_inset_m` (1 m) in. No point in a strip is that far in, so the
  deepest inbound pose inside the area is used instead. A route that ends up to
  `dock_join_edge_tol_m` outside the outline now counts as ending in the area: the strip
  start was 0.002 m outside. This is the fallback; the plan itself ends where it began.
- **Finish message.** The text now leads with the cut share of the finished areas' mowable
  polygons from the mowed rasters, followed by the path share: "cut 46.0 % of the area
  (mowed raster), 98.8 % of the planned path mowed". The best run of an area counts.
  MOWING_COMPLETE always carries it once a raster has reported.

### Reverse controller (`mower_navigation`, `mowgli_nav2_plugins`)

The 6 m retraces are the first long reverse legs the stack drives; until now reverse legs
were K-turn and swath-end turn legs of 0.3-1.5 m. With the old reverse MPPI the sim (`-l`)
weaved ±12 deg / 0.17 m on them. `FollowCoveragePathReverseMPPI` now uses
`PathAlignAheadCritic` in place of `PathAlignCritic`, with the forward controller's damping
mirrored:

- `control_point_x` -0.30. A negative control point lies behind base_link, which is the end
  that leads in reverse. `path_align_ahead` now looks it up |x| along the path; until now
  only 0 and positive values were used, and their meaning is unchanged.
- weight 30, `latency_s` 0.2, and the slow cross-track trim (tau 20 s, at most 0.10 m).

The sim's turn-leg line cross-track p99 went from 0.186 to 0.055 m. Revert: the old
`PathAlignCritic` block (weight 20).

### Sim gate (`scripts/sim_gate/evaluate.py`)

Criterion (i) compared the body with the planned body facing along the segment. On a
segment driven in reverse the robot faces against it, so the nose was judged where the tail
belongs. The planned heading is now flipped when the robot faces against the segment.

## 3. Sim

World `src/mower_sim/worlds/owner_area4_strip.yaml`:

- `owner_far_areas.dat` with Area 4 as the lawn and the field's settings, docked start;
- both long sides are ordinary soft edges and the ends are flat sim ground, so there is no
  drop line (the owner: the mapped south edge is wheel-safe).

All runs were `-l -x 3 -s 1 -t 4000`, one at a time; output is in
`ros2_stack-wt/simout_strip1113/`. Runs 1-4 drove intermediate code. Each one found the
problem that the next change fixes:

| Run | Code | Verdict | What failed |
|---|---|---|---|
| strip-l-s1 | first version | FAIL (c, h, i) | mowed 10/10, cut 96.5 %, no STUCK; reverse legs weaved ±12 deg / 0.17 m (h turn-leg p99 0.186); (i) also judged reverse legs facing the wrong way (evaluator); dock route: the plan's end in the SW corner was off the route graph, NAV_TO_DOCK_FAILED |
| strip2-l-s1 | + reverse latency projection, evaluator fix | FAIL (c, h, i) | reverse p99 0.125 (slow one-sided drifts to 0.13 m); dock join still found no entry (the route ended 0.002 m outside the outline) |
| strip3-l-s1 | + reverse weight 30 / trim, dock-join fallback | FAIL (a, b, f, h, i) | docked, but the dock join pivoted at the SW corner: 2 BOUNDARY OUT in RETURNING_HOME; a pass start put base_link 0.11 m out on the slanted end |
| strip4-l-s1 | + plan ends where it began, base_link kept inside | FAIL (h, i) | a-g pass; reverse p99 0.149, side over plan 0.225 on one reverse leg |
| **strip5-l-s1** | + reverse control point -0.30 | **PASS** | every criterion |
| run5-l-s1 (real_run5) | final | FAIL (i only) | the known master failure (STATUS "Current state": lap 0 over its soft limit at convex bends; i side 0.103 / nose 0.059 / sweep 0.134, transit 0.354); a, b, c, d, e, f, h pass |

strip5 in detail:

- a 0.000 m; b 0; MOWING_COMPLETE, docked and charging; d 0 / 0 / 0; e 0 of 5;
  f 0; i side / nose / sweep over plan 0.046 / 0.035 / 0.085.
- Cross-track: line p99 0.055 m (forward and reverse; reported as turn-leg because the
  plan is not conditioned).
- Cut: 94.2 % of the strip by ground truth (the mission raster says 94.7 %). Plan 99.7 %.
  The field cut 46 %.
- MOWING took 352 s.
- The finish text: "cut 94.7 % of the area (mowed raster), 100.0 % of the planned path
  mowed".
- No pivot while mowing. The pivots left are the transit's at the west tip:
  - on arrival, a route-corner pivot of 93 deg and a leg-start pivot of 88 deg;
  - leaving, ±93 deg.

  Both happen at the strip's start pose, where the inbound route ends, with the body over
  plan at most 0.085 m.

real_run5 shows the strip changes leave a normal lawn alone:

- The detector logged "inscribed radius 0.59 m >= 0.385" and did not fire.
- Coverage_fraction was 0.952, so the plan passed the gate.
- Cut 97.7 % by ground truth, the same as the 2026-11-06 V2 run (97.7 %).
- The reverse controller change drives the lawn's K-turn legs: 60 samples, cross-track
  p99 0.011 m.

## 4. Open

- **Field test.** Nothing here has run on the mower. The reverse retraces are the first long
  reverse drives: watch the reverse cross-track on the field bag (sim p99 0.055 m) and the
  rear camera image on the first run. The rear camera is not used for detection: the
  safety argument is that each reverse leg retraces a line driven seconds earlier.
- **Entry pivots at the west tip.** See section 5: the in-strip pivots are gone; the one
  pivot left, onto the pass axis past the tip, has only ~0.10 m of mapped room.
- **Obstacles in a strip.** The owner's rule: try the normal detour, else hold, then stop
  in STUCK_NEEDS_HELP. Unit-tested only; no sim scenario with an obstacle in the strip.
- **Slope.** Done in section 5: the entry end follows the fall line.
- **Reverse controller on other reverse legs.** The K-turn and swath-end reverse legs now
  run with the new critic. real_run5's 60 reverse samples tracked at 0.011 m p99, but no
  slope world has been rerun.
- **`mission.yaml`** is unchanged. The bridge gained three parameters (`strip_mode`,
  `plan_coverage_min`, `min_fragment_m`). There is no new area setting and no GUI change.

## 5. Entry, exit and the fall line (2026-11-25, branch ws/stripentry1125)

Branch `ws/stripentry1125` from master 2ca8d8a. Unit-tested and run in the sim; not field
tested.

### Field

The strip runs of 2026-11-13 (95.1 % cut), 11-24 22:14 UTC and 11-25 had clean mowing legs
(max overhang 0.19 m on 11-24, no pivot inside the strip). The trouble was around them:

- **Entry (11-24).** The transit came along Path 2 from the east, hopped square off the
  centreline onto the strip start (route corner 93 deg) and then turned onto the pass (final
  approach / leg-start pivot, up to 104 deg). On the 11 deg slope each pivot at the tip slid
  0.15 m (SPIN DRIFT "body near its limit"), so it took about 9 pivots and 4 SPIN DRIFT
  aborts. There were 2 BOUNDARY OUT, 2 STUCK and a STUCK_NEEDS_HELP, base_link went up to
  0.71-0.78 m outside the polygon, and mowing started 4.5 min after the plan.
- **Exit / dock route (11-24).** The dock route hopped square onto Path 2 again: pivots of
  -171 / 105 / -175 deg at (-16.7..-17.2, 0.5..1.5), each aborted 3 times by SPIN DRIFT,
  so two STUCK_NEEDS_HELP.
- **East tip (11-25).** The reverse legs that start at the 0.48 m east tip tripped the
  guard: BOUNDARY OUT rear_right / rear_left 0.36 (limit 0.35) and tail 0.30 (limit 0.30),
  then STUCK_NEEDS_HELP "left the boundary 2 times". The planner had kept those points at
  0.25 / 0.20, so the reverse tracking error there was about 0.11 m.

### Which end to enter: data

The bag of 11-24 (the 11-13 bag is still on the mower) on the strip: pitch -10.5 deg with
the robot facing 0-30 deg (nose up). The downhill direction from pitch and roll is
198 deg (resultant 0.95), and the terrain memory gives a fall line of 15-19 deg. So the
**west tip is the downhill end**, and master already entered there. It entered there
because the west tip is the wide end, not because of the slope.

| Evidence | Forward legs uphill (enter at the downhill end) | Forward legs downhill (enter at the uphill end) |
|---|---|---|
| Wheel odometry vs RTK, 11-13 drift study | uphill 1.02-1.05 (a few % off) | downhill 0.84 on the strip (16 % off) |
| Same, 11-24 strip, 3 s chunks (RTK / wheel; the absolute scale of `/mcu/measured_speed` is off, compare the rows) | forward uphill 1.08 (n 23) | reverse downhill 1.21 (n 30); forward downhill elsewhere 1.25 (n 10) |
| Pass ends, 11-24 | forward ends (uphill) within 0.01 m of the plan | reverse ends (downhill) within 0.02 m |

Downhill driving is where the wheels lose grip (12-16 % against a few % uphill). The
forward legs carry the steering (the lane-change S-curves), and the reverse legs are
straight retraces of a line driven seconds earlier. So the forward legs should run uphill:
**enter from the downhill end**. Stopping at the far end of an uphill pass is helped by
gravity. The reverse legs end, downhill, at the entry end, which is lawn. The 11-24 shuttle
did exactly that and ended its passes within 0.02 m.

### Changes

- **Signed fall line** (`mower_map/terrain.py`). `slope_axis` also returns
  `downhill_deg`, the direction of the mean gradient. It is None when the gradients do not
  agree on one direction (a ridge or a valley: the mean is under half the dominant
  slope). `fall_line_downhill` applies the gates `choose_mow_angle` uses (at least 3 deg,
  anisotropy 0.3). `get_area_settings` adds `slope_downhill_deg` (read-only, whatever the
  `slope_mode`). The mission passes it on as the coverage server's `strip_downhill_deg`
  (-1 = none known).
- **Entry end** (`strip.py` `_choose`). Ending where it began comes first. Then the plans
  within `cut_tie` (3 %) of the most cut are kept. Among them the planner prefers forward
  legs within `fall_line_deg` (60) of uphill, then the wider entry end (unchanged as the
  tie-breaker), then the shorter drive. The plan message says "entered from the downhill
  end". On Area 4 the downhill end and the wide end are the same (west).
- **Body limits at the tips** (`strip.py`). Rear corners 0.25 -> 0.20 and tail 0.20 -> 0.15,
  so both are now 0.15 under the guard. That costs 1 % of Area 4's planned cut (99 %).
- **Arrive along the pass** (mission `_route_tangent`). A strip plan always gets the tangent
  approach, whether or not the `tangent_approach` area setting is on. The route ends on a
  straight along the first pass from `strip_approach_m` (1.0 / 0.8 / 0.6 m) behind the
  start, so the turn onto the pass happens past the tip and not inside it. The checks
  along the straight are base_link inside the allowed region (strip, paths and lawn) and
  every body point `strip_approach_margin_m` (0.10) under its soft guard limit. The
  lawn-edge check (base_link 0.285 m inside) cannot pass in a 0.5-0.7 m strip. Of the
  lengths that pass, the planner takes the one whose pivot point leaves the body the most
  room through a full turn. Room is measured there and again `spin_drift_hard_m` (0.30) further
  downhill: along `slope_downhill_deg` when known, else back along the approach (a strip
  mowed along its fall line creeps along its axis). The robot then reaches the start
  aligned, so there is no final-approach turn and no leg-start pivot.
- **Shallow merges onto paths** (`route_graph.py` `_merge_ends`). A route that starts or
  ends beside a path's centreline used to hop square onto it and turn 90 deg: two pivots a
  few cm apart. It now merges onto the line `merge_angle_deg` (15) ahead, when the merge
  stays inside the areas and paths and the line runs straight that far. Leaving the strip
  start, the dock route's first heading is 16 deg off the robot's instead of 109 deg. This
  is the shared cause of the entry hop and the exit / dock-route pivots beside the strip.

### Tests

- `mower_map/test/test_terrain.py`: the signed downhill for 198 and 30 deg with an IMU bias;
  none on a ridge; the gates.
- `test_terrain_node.py`: `slope_downhill_deg` in the area settings, also with
  `slope_mode` off.
- `test_route_graph.py`: from the strip start on the owner's map, a first heading within
  15.5 deg of Path 2 and no turn over 16 deg in the first metres (square with
  `merge_angle_deg` 0). The goal end merges the same way.
- `mower_coverage_bridge/test/test_strip.py`:
  - an even 6 x 0.6 m strip is entered from its downhill end for downhill 180 / 0 / 200 /
    -20 deg;
  - a wedge with the slope across it, or no slope, is entered at its wide end;
  - Area 4 with the field's 198 deg is entered at the west tip.
- `mower_mission/test/test_terrain_hooks.py`: `strip_downhill_deg` reaches the coverage
  server (-1 without a slope).
- `test_path_conditioning.py`:
  - a strip plan's route ends on a straight from 1 m past the tip;
  - with no room past the tip the route is unchanged;
  - with a short lawn past the tip the 0.6 m approach is taken.

### Sim

All runs `-l -x 3 -s 1 -t 4000`, under the shared lock, output in
`ros2_stack-wt/simout_stripentry1125/`. The master runs used a detached master worktree.

- New world `owner_area4_strip_slope.yaml` is `owner_area4_strip.yaml` plus an 11 deg slope
  region with downhill at 197.7 deg. It covers the strip, Path 2 beside it and the lawn past
  the west tip.
  - The default slip (calibrated on a side slope: a reverse loses 95 %, a forward drive
    25 %) stalled master's first uphill pass (STUCK_NEEDS_HELP, run master-slope-l-s1), which
    the field strip never did.
  - The world therefore sets forward / reverse slip to 0.05 / 0.15. The pivot creep stays
    at the calibrated 0.045 m/s.
- The sim's terrain memory is empty when the plan is made, so the downhill entry is not
  exercised in the sim. It is covered by the unit tests.

Entry: from 2 m before the start until 0.3 m along the first pass. Overhang: base_link
outside the strip polygon (the branch's pivot point lies on Path 2 past the tip, on
purpose). Exit: from RETURNING_HOME until 2 m from the start.

| Run | Verdict | Entry | Pivots at the entry | SPIN DRIFT entry / exit | Exit | Max base_link past the strip (entry) | Cut (gt) |
|---|---|---|---|---|---|---|---|
| master flat (rtkdrop1113 strip-l-s1) | PASS | 21 s | 93 + 91 at the tip, final 14 | 0 / 0 | 15 s, pivots 100 / -90 at the tip | 0.16 m | 95.4 % |
| **branch flat (strip-l-s1)** | **PASS** | 23 s | 178 past the tip, final -5, no leg-start pivot | 0 / 0 | 9 s, one 21 deg turn | 0.55 m (on Path 2) | 93.1 % |
| master slope (master-slope2-l-s1) | PASS | 30 s | 93 at the tip, final 96, leg start -24 | 2 / 1 | 19 s, 93 / -86 | 0.26 m | 94.4 % |
| branch slope, approach 1.0 m (slope-l-s1) | FAIL (e, i) | 42 s | -179 past the tip, final 83, leg start -66 (slope pre-drive) | 2 / 0 | 9 s, no pivot | 1.20 m | 93.9 % |
| **branch slope, final (slope2-l-s1)** | FAIL (e, i) | 36 s | -176 at 0.6 m past the tip, final 58, leg start -34 | 2 / 0 | 8 s, no pivot | 0.84 m | 94.7 % |
| master real_run5 (rtkdrop1113 run5-l-s1) | FAIL (i only) | | | | | | 97.6 % |
| **branch real_run5 (run5-l-s1)** | **PASS** | | | | | | 97.1 % |

- **Flat.** The branch does what it should. The two turns at the tip become one turn past
  it, and the robot then reaches the start aligned (final -5 deg, no leg-start pivot). The
  exit loses both of its pivots.
- **Slope, exit.** Fixed: no pivot and no SPIN DRIFT, 8 s against 19 s.
- **Slope, entry.** Not fixed. The one remaining turn onto the pass (about 180 deg, since the
  route comes from the east along Path 2) still slid and was stopped twice "near its limit".
  The robot then needed a final approach and a leg-start pivot.
  - The cause is the map. Past the west tip the allowed region is only Path 2's 0.7 m bend:
    a full turn there has at most about 0.10 m of room under the guard (front corners at
    the side limit 0.35 on a slope), wherever the pivot point is put.
  - With a 1.8 x 1.2 m lawn polygon mapped past the tip, the same turn would have 0.27-0.29 m
    of room, also after 0.3 m of creep (offline, `bbody.evaluate`). The owner says that
    ground is lawn.
  - (e) fails on that pivot's 0.32 m slide, counted as a route corner. (i) fails on a weave
    of up to 13 deg on the last reverse leg at (-12.7..-13.1, 1.8), side 0.149 over plan.
    master-slope2 had 0.057, so this is slope reverse tracking, not the approach.
- **real_run5.** PASS: the strip detector stays off, and the merges and approach change
  nothing that fails. (i) 0.098 against master's 0.105 is within run-to-run noise. Cut
  97.1 % against 97.6 %.

### Open

- **Map the lawn past the west tip.** A lawn polygon of about 1.5-2 m past the tip (owner:
  it is lawn) gives the turn onto the pass room for its slope creep, and the branch's
  approach then uses it.
- **Pivot-drift guard in confined spots.** The 0.15 m "near its limit" stop re-pivots from
  the drifted pose. On a slope the drift simply continues, so the retries run out
  (STUCK_NEEDS_HELP). The same happens at (-10.4, 3.1), (-9..-8.8, 3.9) and the dock corner
  (0.1, -0.5). The slope pre-drive (`_slope_pivot`) covers only coverage pivots on a side
  slope, not route-corner pivots facing downhill. Not changed here.
- **Reverse legs on the slope.** One 13 deg weave on the slope world's last reverse leg.
  The field's 11-25 east-tip trips were reverse legs too. The tighter rear limits leave
  0.15 m, which needs a field check.
- **Transit trips on Path 2 beside the strip (11-25).** At (-13.9..-14.3, 2.0..2.3) and
  (-15.9, 1.1): route-corner pivots on the slope with SPIN DRIFT, then BOUNDARY OUT. Where
  these are the square hops, the merge removes them; corners of Path 2 itself are unchanged.
- **Exit to the west.** The plan ends at the start facing east. A route that leaves west
  (to the lawn section) still turns about 180 deg at the tip.

