# Boundary smoothing for recorded areas (2026-11-09, `ws/smooth1109`)

## Problem

Areas recorded by driving the mower around them zigzag. Owner's measurement: each vertex's
distance from the chord of its two neighbours.

| Area | p50 | p90 | max |
|---|---|---|---|
| closer backyard lawn (48 vertices) | 10 cm | 21 cm | 48 cm |
| further backyard lawn | 8 cm | 16 cm | 21 cm |
| Area 5 | 8 cm | 18 cm | 23 cm |
| Clothesline | 7 cm | 15 cm | 22 cm |
| GUI-drawn paths | 1-2 cm | | |

Lap 0 runs on the line, and laps 1-2 (0.15 / 0.30 m) are offsets of it, so every zig is
copied into every lap. The 2026-11-08 field mow had squiggly edge laps while the rows were
straight.

Today the recorder runs Douglas-Peucker at 0.05 m on the driven track
(`geometry.simplify_ring`). The mission then cleans the outline in memory before planning
(`clean_area_ring`: jogs of 90 deg or more within 0.10 m, then DP 0.05 m). Neither removes
these zigs: on the 2026-11-05 map snapshot the cleaned rings have the same wiggle as the
stored ones (closer backyard lawn p50 0.097 -> 0.097 m). DP keeps a subset of the noisy
samples where they are, and any zig taller than the tolerance stays.

## What other projects do

| Project | What happens to a recorded boundary | Source |
|---|---|---|
| MowgliNext (upstream of our recorder) | Douglas-Peucker at 0.05 m on finish. The header explains the tolerance: below the 5 cm sampling gate DP "only preserves jitter" | `third_party/mowglinext/mowgli_behavior/include/mowgli_behavior/recording_nodes.hpp` |
| OpenMower | No simplification. Only a 0.1 m minimum point spacing (`NEW_POINT_MIN_DISTANCE`), then the ring is closed | https://github.com/ClemensElflein/open_mower_ros/blob/main/src/mower_logic/src/mower_logic/behaviors/AreaRecordingBehavior.cpp (l. 403-405, 436-441) and `AreaRecordingBehavior.h:43` |
| AgOpenGPS | Resampling only (`FixFenceLine`: 1.1-3.3 m spacing by field size, points inserted / dropped). No smoothing | https://github.com/agopengps-official/AgOpenGPS/blob/master/SourceCode/GPS/Classes/CFenceLine.cs |
| Fields2Cover (in our stack) | No simplify or smooth on `Cell` / `Cells` / `LinearRing`, only buffers | https://github.com/Fields2Cover/Fields2Cover/tree/main/include/fields2cover/types |
| shapely / GEOS | `shapely.simplify(tol, preserve_topology=True)`: Douglas-Peucker that avoids invalid output | https://shapely.readthedocs.io/en/stable/reference/shapely.simplify.html |
| boost::geometry | `simplify`: "Geometries might become invalid by using simplify" | https://www.boost.org/doc/libs/release/libs/geometry/doc/html/geometry/reference/algorithms/simplify/simplify_3.html |
| JTS / GEOS | `PolygonHullSimplifier`: outer / inner hulls. "Inner hulls are contained by the input geometry", "the hull vertices are a subset of the input vertices", the result is valid. It stops at a vertex fraction or an area-delta ratio | https://locationtech.github.io/jts/javadoc/org/locationtech/jts/simplify/PolygonHullSimplifier.html, https://libgeos.org/doxygen/geos__c_8h.html (`GEOSPolygonHullSimplifyMode_r`, `GEOSHULL_PARAM_VERTEX_RATIO` / `AREA_RATIO`) |

Algorithms considered, with their references:
- Douglas-Peucker: Douglas & Peucker 1973, Cartographica 10(2) 112-122.
- Visvalingam-Whyatt: The Cartographic Journal 30(1) 46-51, 1993.
- Chaikin corner cutting: CGIP 3(4) 346-349, 1974.
- Savitzky-Golay: Analytical Chemistry 36(8) 1627-1639, 1964.
- Split-and-merge line fitting: Nguyen et al., IROS 2005.

Chaikin and Savitzky-Golay round corners and move the line both ways. RTS smoothing would
need the raw track and covariances, which neither the stored areas nor the GUI have.

## Choice: the inner polygon hull, stopped by a distance

`src/mower_map/mower_map/boundary_smoothing.py` (pure Python, no shapely):

- It is JTS's inner-hull rule. It repeatedly removes a convex vertex whose corner triangle
  holds no other vertex (an ear), smallest triangle first (the Visvalingam-Whyatt order).
- The stop is Douglas-Peucker's: a vertex goes only while every recorded vertex of the span
  the new edge replaces is within `tolerance` of that edge.
- GEOS has no distance mode and shapely 2.0.3 (the image) does not expose the hull, so
  this is a ~100-line port.

Guarantees (unit-tested: `src/mower_map/test/test_boundary_smoothing.py`):

| | Guarantee | Why |
|---|---|---|
| Deviation | Hausdorff distance between recorded and smoothed line <= tolerance | the vertex check per replaced span; distance to a segment is convex, so it bounds whole edges |
| Direction | inward or on, never outward | each removal cuts an ear off, which lies inside the polygon |
| Corners | vertices are recorded vertices, unmoved; a reflex vertex is never removed while reflex; a convex corner is at most bevelled by the tolerance, never rounded | vertex subset, the convex test |
| Validity | simple, same winding | removing an ear of a simple polygon leaves it simple; a self-crossing input is returned unchanged with a reason |
| Area | only shrinks, reported | inward only |

**Why inward-only.** The owner decides where the lines are. A smoothed line that may move
outward would put the mower beyond a line the owner drove. Inward-only costs at most the
tolerance of edge cut, and the 0.15 / 0.30 laps and the deck overhang still reach the edge.

**Not chosen.**
- Symmetric DP (shapely) at the same tolerance smooths about as well (table below), but
  moves the line outward by up to the tolerance (0.08-0.34 m² per area outside the
  recorded polygon).
- DP intersected with the recorded polygon is inward-only, but brings the inward zigs back.
- A least-squares split-and-merge prototype produced bevel artefacts. It needs more work to
  give a hard bound.

**Tolerance.** One parameter, default 0.08 m (`DEFAULT_TOLERANCE_M`, mission
`record_smooth_tolerance_m`, CLI `--tolerance`). It is the top of the requested 3-8 cm range
and under half a swath (0.18 m). Zigs taller than the tolerance stay. On these maps those
are real kinks or recording errors (e.g. the 0.48 m notch in the closer backyard lawn and
0.70 m on Clothesline), better fixed by editing the line in the GUI than by a larger
tolerance.

## Where it hooks in

- **Recording** (`mower_mission/fsm/phases/recording.py`):
  - On `RECORD_FINISH` of an AREA, the 0.05 m-DP ring is smoothed before `add_area`.
  - The ring as recorded, the tolerance, the max deviation and the area before and after go
    into the recording quality sidecar (`<recording_quality_dir>/<name>.json`, key
    `smoothing`). The GUI reads that file and ignores the new key.
  - The log has one line per smoothed recording.
  - No-go zones are not smoothed, because inward would shrink a keep-out. Paths are bands,
    not lines.
  - `record_smooth_tolerance_m: 0` turns it off.
- **Existing areas** (`mower_map/smooth_areas.py`, `ros2 run mower_map smooth_areas`):
  - Reads an `areas.dat` and writes a proposal (`areas.smoothed.dat`, a complete map with
    only the selected boundaries changed) plus a report (`areas.smoothed.json`, also printed).
  - It refuses to write over its input. The owner applies the proposal by copying it over
    `areas.dat` and calling `load_areas` (mower_map README, "Smoothing existing
    boundaries").
  - `--area` selects areas one by one. `--restore-from <record_quality dir>` writes the
    recorded rings back as a proposal.
  - A GUI "Smooth boundary" action was not built. The GUI edits the map in the browser and
    writes it whole through `replace_areas`. A preview there needs either a second (TS)
    implementation of the algorithm or a new service type with the three-consumer codegen.
    The CLI gives the same per-area confirm with one implementation.
- **A6:** one new mission parameter (`record_smooth_tolerance_m`) and no removal. The
  tolerance moves where the mower drives the edge (owner rule), so it must be tunable and
  switchable off per robot. Nothing existing does that job: `record_simplify_tolerance_m`
  is the sampling DP (both directions, all recording kinds), and `area_outline_*` is the
  in-memory planner cleanup of every area, which must not move lines the owner can see.

## Evaluation on the owner's areas (2026-11-05 snapshot, tolerance 0.08 m)

Offline only; the map file was not changed. Numbers only (no coordinates).

### Outline

- *Vertex zigzag* is the owner's metric. It rises after smoothing because the vertices that
  remain are farther apart, so the chords are longer.
- *Wiggle 0.5 m* is the same metric on the line resampled every 0.5 m, so the scale is
  fixed.
- *Excess turning* is Σ|turn| − 360°: the back-and-forth heading change a lap on this line
  drives.

| Area | vertices | vertex zigzag p50/p90/max (m) | wiggle 0.5 m p50/p90 (m) | excess turning (deg) | max dev (m) | area change |
|---|---|---|---|---|---|---|
| heart | 48 -> 26 | 0.056/0.081/0.234 -> 0.074/0.208/0.296 | 0.065/0.137 -> 0.060/0.146 | 1583 -> 416 | 0.078 | -0.31 m² (-2.50 %) |
| Area 4 | 26 -> 14 | 0.056/0.109/0.205 -> 0.071/0.386/0.531 | 0.017/0.241 -> 0.004/0.220 | 500 -> 193 | 0.078 | -0.35 m² (-9.24 %) |
| Area 5 | 23 -> 19 | 0.103/0.212/0.266 -> 0.135/0.252/0.403 | 0.043/0.176 -> 0.021/0.166 | 586 -> 268 | 0.074 | -0.09 m² (-0.78 %) |
| further backyard lawn | 43 -> 37 | 0.098/0.180/0.257 -> 0.111/0.283/0.567 | 0.019/0.126 -> 0.016/0.130 | 563 -> 528 | 0.079 | -0.23 m² (-0.25 %) |
| closer backyard lawn | 48 -> 41 | 0.097/0.161/0.478 -> 0.102/0.314/0.478 | 0.023/0.112 -> 0.018/0.117 | 663 -> 582 | 0.076 | -0.43 m² (-0.32 %) |
| Clothesline | 26 -> 21 | 0.098/0.189/0.700 -> 0.103/0.274/0.700 | 0.023/0.113 -> 0.020/0.117 | 562 -> 195 | 0.080 | -0.10 m² (-0.36 %) |

Every area was checked independently with shapely:
- 0.000 m² of the result is outside the recorded polygon;
- the result is valid with the same orientation;
- the Hausdorff distance equals the reported max deviation.

Area 4 is a 3.8 m² sliver, so the same 8 cm costs 9 % of it.

Symmetric DP at 0.08 m for comparison (excess turning):

| Area | Excess turning, DP 0.08 m (deg) |
|---|---|
| heart | 182 |
| Area 4 | 92 |
| Area 5 | 251 |
| further backyard lawn | 373 |
| closer backyard lawn | 514 |
| Clothesline | 170 |

Each area also had 0.08-0.34 m² outside the recorded line.

### Planner

The planner (`plan_dump`, the node's `planCoverage` call, soft layout, 3 laps) was run with
the mission's in-memory outline cleanup. The output went through the mission conditioner and
an idealised drive into the mission's MowedRaster (the 2026-11-05 `pipeline.py`).

- *Lap squiggle* is the lap points' distance from the 2 m chord centred on them. It is
  counted only on straight edges: segments of at least 3 m of the recorded ring at DP 0.3 m,
  ignoring 1 m at each end, and the same stretches for every variant.
- *Lap turning* is Σ|heading change| over all laps.
- *Cut %* is of the area's own polygon, which is smaller after smoothing.

| Area | lap squiggle p50/p90 (cm) | lap turning (deg) | cut % | planner coverage | drive (m) |
|---|---|---|---|---|---|
| heart | 5.5/10.5 -> 5.0/10.0 | 1747 -> 1673 | 99.42 -> 98.93 | 0.987 -> 0.980 | 90.5 -> 89.3 |
| Area 4 | 0.0/4.6 -> 0.5/4.7 | 503 -> 513 | 41.77 -> 41.97 | 0.315 -> 0.336 | 13.3 -> 13.1 |
| Area 5 | 9.6/44.4 -> 9.6/44.3 | 1455 -> 1414 | 74.63 -> 73.98 | 0.722 -> 0.722 | 75.1 -> 73.8 |
| further backyard lawn | 0.0/8.0 -> 0.2/8.1 | 2237 -> 2180 | 99.75 -> 99.84 | 0.992 -> 0.992 | 592.7 -> 581.3 |
| closer backyard lawn | 3.2/10.8 -> 2.3/9.9 | 2584 -> 2416 | 99.72 -> 99.73 | 0.995 -> 0.995 | 851.6 -> 840.3 |
| Clothesline | 4.0/32.3 -> 3.4/31.8 | 2101 -> 2074 | 91.96 -> 91.34 | 0.933 -> 0.933 | 165.9 -> 165.0 |

At 0.10 m, more is removed:

| Area | lap squiggle p50 (cm) | lap turning (deg) | drive (m) | cut % |
|---|---|---|---|---|
| closer backyard lawn | 1.3 | 2237 (-13 %) | 827.6 | 99.76 |
| Clothesline | 2.3 | 1801 (-14 %) | 163.0 | 91.07 |

Area 5 is 5.4 cm at 0.10 m.

**Reading.**
- At 0.08 m the smoothing removes the small zigs: outline excess turning falls 6-74 %, and
  lap turning falls 2-7 % (Area 4, the sliver, +2 %).
- The closer lawn's straight-edge lap squiggle p50 drops by 0.9 cm.
- Coverage is unchanged within run noise and the drive is 1-2 % shorter.
- The p90 / max squiggle is unchanged. It comes from kinks larger than the tolerance (and,
  in the small areas, curved stretches the straight-edge test still includes). Those need
  a GUI edit, a re-recording, or a larger `--tolerance` for that area, at the price of a
  larger inward move.
- The snapshot is from 2026-11-05. The map driven on 2026-11-08 may differ.

Scratch (not in the repo):
- PNGs before / after per area: `scratchpad/smooth1109/png/`.
- Tools: `scratchpad/smooth1109/tools/` (`check.py`, `lapmetric.py`, `pipeline.py`,
  `plan_dump.cpp`).

## Sim

Three gates on this branch:
- the world: `src/mower_sim/worlds/real_run5_smooth.yaml`, which is real_run5 with Area 1
  smoothed by the CLI (18 -> 14 vertices, max 0.073 m, -0.14 m²);
- flags: `-d -x 3`;
- output: `ros2_stack-wt/simout_smooth1109` (bags pruned).

| Run | a (max out, samples > 0.10) | b | c | d STUCK | e | h p99 line | i | cut % (gt) | verdict |
|---|---|---|---|---|---|---|---|---|---|
| real_run5, seed 2 (baseline, same code) | 0.067 m, 0 | 0 | pass | 0 | 1 of 21 (0.164) | 0.075 | pass | 97.1 | FAIL (e) |
| real_run5_smooth, seed 2 | 0.345 m, 276 | **1** | pass | 0 | 0 of 19 | 0.200 | fail | 95.5 | FAIL |
| real_run5_smooth, seed 1 | 0.084 m, 0 | **1** | pass | 0 | 0 of 19 | 0.086 | fail | 96.3 | FAIL |

**The smoothed world fails b (one hard-boundary trip) in both runs. That is a regression on
this world and blocks the gate.**

The trip is the same in both runs, 5 s into the first sub-path:
- With the smoothed ring the planner starts lap 0 at a different place, on the top edge at
  (-1.96, 4.88) instead of the west edge.
- The transit ends 0.13-0.21 m short of that start, inside the area.
- `_route_final_approach` (`fsm/phases/transit.py`) turns 79-112 deg and drives straight
  onto the start. The start lies on the line, so the robot arrives about 45 deg off the
  edge heading.
- The nose (0.60 m ahead of base_link) is then 0.44 m out, over the 0.43 soft limit. The
  trip fires 0.06-0.17 s after the leg-start pivot begins, so it is the approach, not the
  pivot.

The smoothing kept its bounds: the top edge moved 4 cm inward there, and the robot was
about 0.10 m beyond the recorded line as well. The mission problem it exposes is
independent of smoothing: a final approach to a lap-0 start on the line from a steep angle.
It can happen on a recorded map whenever the planner's start lands where a transit ends
inside at an angle.

Criterion a in seed 2 (0.345 m) is the recovery after that trip (276 samples, 76.7-87.7 s).
Seed 1 had a 0.084 m. Coverage is 1-1.5 points lower, inside the run-to-run spread of these
seeds but not shown to be equal. Corners (e) and STUCK (d) are fine.

**Before the owner applies a proposal:** fix or guard the final approach (e.g. approach a
lap-0 start along the lap heading, or check the arrival body overhang the way the pivot gate
does), then re-gate real_run5_smooth. That is mission transit code, outside this branch.

### Re-gate after the rebase (2026-11-25)

The b trip above is fixed on master by `2026-11-09_final_approach.md` (ws/approach1109). The
branch was rebased on master and re-gated with `-l -x 3 -s 1` (output
`ros2_stack-wt/simout_smooth1125`):

| Run | a (max out, samples > 0.10) | b | c | d STUCK | e | h p99 line | i side / nose / sweep | cut % (gt) | verdict |
|---|---|---|---|---|---|---|---|---|---|
| real_run5 | 0.058 m, 0 | 0 | pass | 0 | 0 of 19 (0.128) | 0.057 | 0.060 / 0.045 / 0.092 | 97.9 | PASS |
| real_run5_smooth | 0.071 m, 0 | **0** | pass | 0 | 1 of 20 (0.225) | 0.068 | 0.059 / 0.031 / **0.102** | 96.5 | FAIL (e, i) |

- b is gone. a is within limits and nothing goes STUCK.
- e is lap 0's -93 deg vertex at (-0.80, 5.34). The final-approach note's master runs on
  this world failed e at the same corner and i, too, and the mowing code here is master's.
  So these are mission corner handling on the smoothed geometry, not the smoothing code.
- real_run5 passes with the same numbers as master's PASS on 2026-11-25, so the branch does
  not regress the normal map.

Before the owner applies a proposal, the e corner on the smoothed Area 1 needs a fix or an
accepted waiver, and then a re-gate.


### Lap start and lap-0 fillet cap (2026-11-28)

e9682d3 moved lap 0's start so the edge-start join has 2.3 m of straight lap before the
first corner. That fixed e and i on real_run5_smooth, but lawn cut dropped (96.2 -> 93.5 %
smooth, 97.9 -> 96.9 % real_run5). The moved start gave the runs room for wide convex fillets
(`path_conditioning_convex_r_max_m` 2.0) at sharp lap-0 corners. An r 1.5 fillet at Area 1's
-118 deg tip (-0.34, 5.52) passed 1.4 m inside the tip, and an r 2.0 fillet at the -67 deg
corner (-6.04, 1.43) passed 0.39 m inside its tip.

Fix: at a lap-0 corner ON the line that turns at least `kturn_min_rad`, the widening stops at
`path_conditioning_lap0_sharp_r_max_m` (1.0 = r_max; 2.0 reverts). The r 1.0 fillet is over
the 0.35 body cap there, so the existing corner cut takes it: 0.60 m inside the -118 deg tip
and 0.30 m inside the -67 deg one. Gentle corners and inset (hard) laps keep their wide
fillets. Output `ros2_stack-wt/simout_cap1128`, `-l -x 3 -s 1`:

| Run | a | b | c | d | e | f | g | h p99 line | i side / nose / sweep | cut % (gt) | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| real_run5_smooth | 0.055 m, 0 | 0 | pass | 0 | 0 of 20 (0.095) | 0 | 16 % | 0.064 | 0.066 / 0.029 / 0.092 | 96.4 (e9682d3 93.5, before 96.2) | PASS |
| real_run5 | 0.057 m, 0 | 0 | pass | 0 | 0 of 17 (0.129) | 0 | 17 % | 0.074 | **0.103** / 0.051 / 0.100 | 97.1 (e9682d3 96.9, before 97.9) | FAIL (i) |
| real_run5, re-run | 0.097 m, 0 | 0 | pass | 0 | 0 of 17 (0.127) | 0 | 16 % | 0.081 | 0.071 / 0.033 / 0.097 | 97.2 | PASS |

- real_run5_smooth is back at its pre-e9682d3 coverage, and e and i still pass.
- The real_run5 i fail (side 0.103, limit 0.10) is the rejoin leg of the -114 deg corner
  cut at (-0.61, 3.70), where the plan was 0.294 m out. That cut is the same before and after
  this change, and the re-run passes with side 0.071, so the fail is run-to-run spread at an
  existing cut.
- real_run5 recovers only 0.2-0.3 of the 1.0 point. That corner was the only real_run5 corner
  this change touched (-64 deg at (-6.04, 1.43), now cut 0.30 m inside). The rest of the loss
  is probably the other cluster around (-3, 4.5), next to the new lap start (-3.27, 4.37).
  This cap does not change it.
