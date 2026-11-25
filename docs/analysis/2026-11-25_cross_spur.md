# The cross-plan "spur" at (-1.05, 3.95) (2026-11-25, ws/spur1125)

## Question

The cross single-plan sims (`2026-11-12_cross_single_plan.md`, Sim results) failed criterion e
on `real_run5_cross` at a 180 deg out-and-back at (-1.05, 3.95), the end of the second row
set (1 of 68 corners, 0.384 m). The same vertex failed on the old two-plan run. Where in the
coverage pipeline does the spur come from, and why does it fail?

## Reproduction (offline, no sim)

The planner and the bridge were started in the dev image (`coverage_bridge.launch.py`, the
committed defaults) and sent the two PlanCoverage goals the mission sent in the sim: Area 1 of
the run's `maps/areas.dat` cleaned as `_plan_outline` does, the coverage parameters from the
sim's `mission.log`, run 1 AUTO with rings, run 2 AUTO + perpendicular with `include_rings`
false. Both results match the sim exactly (18 and 44 sub-paths, the same splitter and planner
messages). Run 2 was then put through `condition_plan` (join_turn_chains + condition) with the
shipped mission parameters: 44 -> 4 sub-paths, the first 1432 poses long, the spur at its
pose 1415, the same as the sim's `~/mow_plan` sub-path 12.

## Where the spur comes from

Stage by stage (run 2 of `real_run5_cross`):

| Stage | Spur at (-1.05, 3.95)? |
|---|---|
| Planner (`mower_coverage` `fillGaps`) | Yes: swath 29 (-1.667, 4.471) -> (-1.048, 3.951), swath 30 (-1.063, 3.964) -> (-1.667, 4.471) |
| Bridge splitter | Joins 29 and 30 into bridge sub-path 40 (gap 0.02 m); adds nothing |
| Turn joins / conditioning | Keeps it; the tip becomes a planned 180 deg pivot (`p`) |
| Cross second-pass join (`_merge_plan_runs`) | Not involved: run 2 is conditioned on its own |

Swaths 29 and 30 are a **gap-fill pocket spur**, a deliberate planner feature (2026-11-05,
the owner's "small V"): a gap that no row and no fill pass of at least 1.0 m reaches gets an
in-and-out pass of at least 0.5 m. The planner reports it: "gap fill: 10 pass(es) (3 pocket
spur(s), 1 along a lap band)". Here the gap is the north-east corner of run 2's rows, where 10
rows were dropped as shorter than 1.2 m. The plan has three such spurs (also (-1.853, 4.392)
and the south-west lobe at (-5.029, -0.122), which run 1 drives too). real_run5 alone has two.

## Why it failed e: progress jumped onto the spur's way back

The robot drove the spur as planned. Ground truth follows every planned pivot (1368, 1390,
1399, the about-turn at 1415) within a few cm. The failure is in the mission's progress
index (`high_level_status.current_path_index`, `_track_progress`):

- `geometry.nearest_index` takes the nearest pose up to `progress_window_m` (3 m) ahead.
- In that sub-path the conditioner filleted the row 26 -> fill pass 27 hairpin (-117 deg,
  r 0.6) into an arc that passes within 6 mm of fill pass 28's connector to the spur mouth,
  2.2 m later on the path. The spur's way back lies on its way in.
- Replaying the sim's map poses through `nearest_index`: progress jumped 1343 -> 1389 (map
  pose 0.009 m from the arc, 0.006 m from the connector), then 1395 -> 1428 (the way back,
  0.060 m against 0.066 m), while the robot was still on the arc.
- Criterion e attributes a sample to a corner by its progress index (within 1.5 m of path).
  The arc samples, 0.3-0.4 m from the spur line, were judged as the spur's about-turn:
  max dev 0.384 m.

The same replay finds jumps of 0.8-2.9 m on every recent real_run5-map run: 5 on the AUTO + 90
cross run, 4 on the 20 + 60 run and 2 on plain real_run5 (each on a pocket spur, where the
way in and the way back are within float noise of each other). Progress is more than the
gate's view. `start_local` (the restart pose after a hold, retry, stuck or escape), the
resume cursor, the boundary and spin-drift guards and the follow-end check all read it. A
hold on a spur's way in would have restarted the follow on its way back and left the tip
uncut.

## Fix

`geometry.progress_index` (new, beside `nearest_index`, which is unchanged for its other
callers): the nearest pose in the window, except that a later stretch of the path wins over
the stretch the robot is on only when it is closer by more than 0.10 m
(`PROGRESS_SWITCH_M`). The stretch the robot is on runs from the current progress until the
path first gets 0.10 m farther from the robot than its closest pose so far. A robot that is
really farther along (a cut corner, a lagging update) is still picked up: the path leads to
it, or the later pose is clearly closer. `_track_progress` uses it.

Replayed over the same sims: 0 jumps (from 5 / 4 / 2), and in no sample is the new index more
than 0.1 m farther from the robot than the old one.

Not changed: the pocket spurs themselves (owner feature, driven fine), the planner and the
splitter.

## Tests

- `test_geometry_resume.py`: `test_progress_index_stays_on_a_spur_way_in_its_way_back_runs_over`
  (the planner's spur from run 2, the way back starting 0.02 m short of the tip as `fillGaps`
  plans it: `nearest_index` jumps, `progress_index` stays on the way in),
  `test_progress_index_takes_a_later_stretch_clearly_closer`.
- `test_mission_fsm.py`: `test_progress_on_a_pocket_spur_way_in_stays_off_its_way_back` (the
  spur geometry through planning, conditioning and `_track_progress`; fails on the old
  tracker).
- `scripts/ci_test.sh`: colcon PASS (3969 test cases, 0 failed), scripts PASS, replay PASS.

## Sim (seed 1, x3, field drift)

Commit 551bfed, `scripts/sim_gate.sh -w <world> -l -x 3 -s 1 -t 4000`, output in
`simout_spur1125/` (bags deleted). Baselines: the cross worlds against the Sim results of
`2026-11-12_cross_single_plan.md` (commit cfa24f2), real_run5 against `simout_rtkdrop1113/run5-l-s1`
(a7a9a6e; master 2ca8d8a only adds a docs commit and a bringup default after it).

| World | Crit. | Master | This branch |
|---|---|---|---|
| `real_run5_cross` (AUTO + 90) | e | FAIL 1 of 68 (0.384 m, the spur) | **PASS** 0 of 68 (0.139 m); spur 0.069 m |
| | i | FAIL: nose 0.137 / sweep 0.168 at (-0.93, 4.85) (the spur's corner of the area); transit 0.386 | FAIL: sweep 0.104 at lap 0 (-5.26, -0.87), the known lap-0 overhang; side 0.071, nose 0.064; transit 0.364 |
| | other | a-d, f-h, j, k PASS | same; docked, 16/16 sub-paths |
| `real_run5_cross60` (20 + 60) | e | FAIL 2 of 41 (0.155 m) at (-4.46, -0.15) / (-4.59, -0.03) | FAIL 2 of 41 (0.161 m), the same two -90 deg corners (not spur related) |
| | h | FAIL line p99 0.318 | FAIL line p99 0.309 (short sub-path approaches, as before) |
| | i | FAIL side 0.093, sweep 0.118; transit 0.366 | FAIL side 0.120 (run 1 lap 0 at (-0.54, 4.51), sub-path 0, which had no progress jumps), sweep 0.124; transit 0.372 |
| `real_run5` | all | FAIL i only (side 0.105, transit 0.374) | **PASS** (side 0.089, transit 0.371) |

The fix clears e on `real_run5_cross` and that world's spur-area part of i (nose / sweep at
(-0.93, 4.85)). i still fails there on the lap-0 corner that fails on master. No new failure:
the fixed60 failures (e, h, i) are the ones master has, at the same places, and they do not
come from the spur. real_run5's i margin is noise around the limit (0.105 -> 0.089, limit
0.10). Replaying the new auto90 run's map poses: `nearest_index` would still have jumped 5
times (spur ways back, the arc / connector), `progress_index` did not.

## Open

- The r 0.6 fillet at the row 26 -> fill pass 27 hairpin cuts 0.55 m inside the vertex. That
  leaves about 1 m of fill pass 27 and the end of row 26 uncut, which is the ground the fill
  pass was planned for. The conditioner does not limit a coverage fillet's cut. Run 1's laps
  have 0.42-0.45 m cuts at -93 / -114 deg corners too. A cut cap for coverage fillets would
  change lap corners, so it needs its own sim gate. Not done here.
- Run 2 of a cross plan plans the south-west lobe spur again, the one run 1 drives
  ((-5.029, -0.122), 0.5 m in and out). It is harmless but redundant: the cross runs share
  their fill target.
- `boundary.py` (recovery target) and the validator in `path_conditioning.py` still use
  `nearest_index` with a 3 m window. They search near a known index, not the robot's progress,
  so they were left alone.
