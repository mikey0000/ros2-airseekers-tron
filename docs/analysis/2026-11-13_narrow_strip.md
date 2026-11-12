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
  within the soft trip limits less a tracking margin: nose 0.28 (trip 0.40), tail 0.20
  (0.30), front corners 0.40 (0.55), rear corners 0.25 (0.35). Base_link itself stays
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
- **Entry pivots at the west tip.** The transit arrives at the plan start from the north
  and turns about 90 deg twice there (once on leaving). The sim keeps the body within 0.085
  m of the plan. On a slope a pivot creeps, so if the field shows it, use
  `tangent_approach` or an entry pose on the axis beyond the tip.
- **Obstacles in a strip.** The owner's rule: try the normal detour, else hold, then stop
  in STUCK_NEEDS_HELP. Unit-tested only; no sim scenario with an obstacle in the strip.
- **Slope.** Area 4 runs along a slope (updown). The planner picks the entry end by width,
  not by the fall line. Entering at the downhill end with the nose uphill needs the signed
  downhill direction from the terrain memory.
- **Reverse controller on other reverse legs.** The K-turn and swath-end reverse legs now
  run with the new critic. real_run5's 60 reverse samples tracked at 0.011 m p99, but no
  slope world has been rerun.
- **`mission.yaml`** is unchanged. The bridge gained three parameters (`strip_mode`,
  `plan_coverage_min`, `min_fragment_m`). There is no new area setting and no GUI change.
