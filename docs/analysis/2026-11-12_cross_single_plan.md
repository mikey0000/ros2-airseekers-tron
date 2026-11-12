# Cross mow as one plan (2026-11-12, ws/crossplan1112)

## Owner report

> "When the mower is mowing the second lot of paths when doing cross, it mows the perimeter
> again. It's also planning again, which I think is the problem. Planning should encompass the
> entire mow."

## Cause

`_build_runs` gives a cross area two runs per repeat. Each run went through PLANNING
(`_plan_run` -> `_start_plan` -> `_on_plan`) and `_area_done` started the next run with a new
plan. Every plan used the area's full coverage settings (`headland_rings` = perimeter laps,
the edge layout's laps), so each later run drove the perimeter / edge laps again. With
`repeat` 2 the laps were driven four times.

The coverage server was no help: the bridge sends `cross` and `alternate` to the planner as
`zigzag`. Each PlanCoverage goal produces one row set.

## Design chosen: (a), every run planned at area start, rings on one run only

At area start, a cross area plans **all its runs** (every repeat) one after the other and
concatenates them into one mission plan:

1. Run 1 (the first run, or the last when `edge_first` is false) is planned as before, rings
   included.
2. Every other run is planned with the new `/coverage_server` parameter `include_rings: false`.
   The planner still builds the full layout. The bridge drops the ring segments before it joins
   the segments into sub-paths (`splitter.plan(drop_rings=True)`).
3. The results are concatenated in run order. Each run's result is path-conditioned on its own,
   as the node does for every plan result. `sub_runs` records the run of every drivable
   sub-path.

Why (a) and not (b) (the coverage server producing both row sets):

- **Row inset.** Turning the rings off on the planner side (`headland_rings` 0, or `edge_laps`
  0) would move the rows out to the boundary, because the planner derives the row area from the
  laps: soft rows sit 0.15 x `edge_laps` in, hard rows b + 0.11, legacy rows inside the
  headland. Dropping the rings *after* the plan keeps every row exactly where the full plan puts
  it, so no new inset parameter is needed. The rows of run 2 stay inside the same inner
  boundary as run 1's rows, and gap-fill passes are the ones computed with the rings present.
- **Small change.** Only the Python bridge changes: one parameter and five lines in the splitter.
  No interface change, no C++ rebuild, and the PlanCoverage result format stays the same. (b)
  would need a second row set in the C++ planner, two-direction ordering, segment types per row
  set, and the AUTO + perpendicular logic moved into the planner.
- **Reuse.** The AUTO resolution (`auto_line_angle` / `resolved_run`, from run 1's planned
  rows) and the perpendicular flag work unchanged, because run 1 is planned before run 2 in the
  same sequence. The preview already planned every direction this way.

Cost: all N planner calls (and their path conditioning) happen at area start, not one at the
start of each run. The total planning time is about the same as before, but it is all spent
before the first row.

### repeat > 1 (goal 2)

All repeats of a cross area go into the one plan: laps, then r0 rows, r+off rows, r0 rows,
r+off rows. There is no second perimeter. I found no strong reason to re-drive the laps per
repeat. The laps are the slowest and riskiest part of the mow (edge pivots, boundary checks),
and the owner's report asks for the planning to cover the entire mow. The rows of the later
repeats still cover the whole row area again. Zigzag and alternate repeats are unchanged: one plan per run,
laps included. The owner can ask for laps per repeat; that would mean `include_rings` on the
first run of every repeat (`rings_position` per repeat), a one-line change.

### Order of the laps

`edge_first` true: the laps come with the first run (laps, rows 1, rows 2). `edge_first` false:
they come with the last run (rows 1, rows 2, laps), so the area still ends on its edge.

## What changed

| Where | Change |
|---|---|
| `mower_coverage_bridge` | `include_rings` parameter (default true); `splitter.plan(drop_rings=...)`; README |
| `fsm/phases/areas.py` | `_plan_group` (cross = every run), `_plan_run` / `_plan_area` / `_plan_params` (per-run parameters with `include_rings`), `_collect_plan_run` (learns each run's row angle, plans the next run, returns the whole plan), `_merge_plan_runs`, `_plan_sub_runs`, `_advance_run`, `_run_after_plan`, `rings_position`; `_planned_run` is the run being planned (used by `_current_run` / `_start_plan`) |
| `fsm/coverage.py` | `_run_raster_progress` (raster progress of the run in progress only), `_earlier_run_subpaths`; `_raster_resume` uses the former |
| `fsm/phases/outcome.py` | `_area_done`: a repeat run follows only after the plan's last run (`_run_after_plan`); `_blocked_mowed_later` drops blocked stretches that the later rows of the plan sweep over |
| `fsm/phases/nogo.py` | 4 lines: the mid-mow re-plan plans every run of the plan (`_plan_area`), keeps the earlier runs done and the raster mapping on the current run, and sets `sub_runs` |
| `fsm/phases/mowing.py` | 1 line: `_next_subpath` calls `_advance_run` |
| `fsm/holds.py` | the coverage-parameters reply is also taken mid-mow during a no-go re-plan |
| `fsm/phases/preview.py` | `include_rings` per previewed cross run (laps once, as the mow) |
| `mow_progress.py`, `mission_node.py` | `~/mow_plan` carries `runs`; `plan_id` uses the plan's first run, so it stays the same across the runs of a cross plan |
| GUI `missionProgress.ts`, `MapPage.tsx` | `MissionPlan.runs`; live stretches of the cross row set are drawn in the cross colour while still to mow (the preview's cross-pass colour) |
| `mower_sim/worlds/real_run5_cross60.yaml` | fixed 20 deg + 60 deg cross world |

### What keeps working, and how

- **AUTO resolution / perpendicular.** The runs are planned in order. `_collect_plan_run` stores
  each run's row angle in `m.line_angles` before the next run's parameters are sent, so
  `_current_run` resolves `auto_offset_deg` from run 1's rows. The "AUTO rows not planned in
  this session" fallback no longer happens for cross, because run 1 is always planned first.
  AUTO + 90 still sends `perpendicular`.
- **GUI run label.** `m.run_i` is now the run whose rows are being mowed. `_advance_run` moves
  it forward when the next sub-path belongs to a later run, and re-publishes
  `active_area_settings` (`run` / `runs`). It only moves forward: an earlier run's leftover
  sub-path (a failure retried on resume) does not move the label back.
- **mow_plan run tags.** `runs` per sub-path. The GUI marks odd runs of a multi-run plan as the
  cross row set (a cross plan alternates start angle and cross angle). The laps belong to the run
  that carries them.
- **Blocked stretches** (2026-11-11 rule). Before, a run that ended with only blocked stretches
  went on to the next run, whose new plan mowed the ground again. Now the mow simply goes on into
  the next run's rows of the same plan. At the end of the area, the end-of-area pass retries the
  skipped stretches. A stretch still blocked on an earlier run's rows is dropped (log + the same
  `run_blocked_continue` incident) when the later runs' sub-paths sweep over >= 90 % of its
  poses. The check uses a scratch MowedRaster of the planned geometry, with half the swath
  width. A stretch blocked on the laps, or in the last run, still leaves the area
  MOWING_INCOMPLETE. This matters now: the laps are driven once, so no later run covers them.
- **Failures.** A sub-path NOT mowed in run 1 no longer stops the area before run 2. The plan
  goes on, and the area ends incomplete with START resuming the missing sub-path.
- **Planning failure.** If any run of a cross plan fails to plan, the whole area plan fails
  (`area N: coverage planning failed (run 2/2: ...)`, AREA_UNREACHABLE). Before, run 1 would
  already have been mowed. Mowing a partial cross plan as if it were the whole one would mark
  the area complete without its cross rows, so the whole plan fails instead.
- **Preview.** It already planned every distinct direction and concatenated them. It now sends
  `include_rings` false for the cross runs that do not carry the laps, so it shows exactly the
  laps-once plan. It still previews distinct directions, not repeats.
- **No-go re-plan mid-mow.** It plans every run of the plan again, with per-run parameters, and
  then installs the merged plan. Sub-paths of earlier runs are taken as done. The raster and
  `done_fps` map the current run.
- **Zigzag / alternate.** `_plan_group` returns the one run, `include_rings` is always true, and
  a repeat still re-plans with its laps. The tests are unchanged apart from the new parameter
  key in the expected dicts.

## Resume

The resume file format is unchanged. `area_run <idx> <k>` now means "the plan is in run k's
rows".

- **Same plan** (fingerprint matches): the cursor's completed sub-paths and resume pose are used
  as before. The mowed raster maps only the run in progress (`_run_raster_progress`). A new
  raster starts at every run boundary, because run 2's rows cross ground that run 1 cut: run 1's
  raster would mark every run 2 row as mowed. The run's cut report is logged before the raster
  is reset.
- **Old per-run cursor, run 2** (`area_run 1`, fingerprint of the old run-2-only plan): the
  geometry differs, so `_cursor_remap` maps it. The laps and run 1's rows count as done (they
  were finished before the old code moved to run 2). Run 2's rows are mapped by the saved run-2
  raster and `done_fps`. The mow continues inside run 2's rows, after the cut lead-in. A
  "plan geometry changed: resume cursor mapped ... by position" warning is logged.
- **Old per-run cursor, run 1** (no `area_run`): run 1's part of the new plan is planned with the
  same parameters as the old run-1 plan, so its sub-paths have the same fingerprints. `done_fps`
  and the raster restore the progress, and the plan goes on into run 2.
- **Map edit between sessions while in run k:** as above, earlier runs done, run k mapped by its
  raster.

## Tests

- mower_mission (`test_mission_fsm.py`): `test_cross_plans_once_laps_then_both_row_sets`,
  `test_cross_edge_last_puts_the_laps_with_the_last_row_set`,
  `test_cross_plan_failing_in_a_later_run_fails_the_area_plan`,
  `test_alternate_and_zigzag_repeats_still_plan_each_run_with_its_laps`, the rewritten
  `test_cross_*` goal / AUTO / perpendicular tests (one plan),
  `test_active_settings_report_every_run_row_angle_on_auto` (label follows the rows),
  `test_cross_stretch_blocked_on_run_1_rows_mowed_by_run_2_completes_the_area`,
  `test_cross_stretch_blocked_where_run_2_does_not_mow_ends_incomplete`,
  `test_last_run_with_a_blocked_stretch_still_ends_incomplete`, and preview `include_rings`.
- `test_mowed_raster.py`: `test_cross_later_rows_start_with_an_empty_raster_and_resume_in_them`
  (restart in run 2), `test_old_per_run_cross_cursor_resumes_in_the_second_row_set_of_the_one_plan`,
  `test_old_per_run_cross_cursor_of_run_1_keeps_its_unchanged_rows`,
  `test_no_go_replan_of_a_cross_plan_plans_every_run_and_keeps_the_earlier_ones_done`.
- `test_mow_progress.py`: `runs` tags, and the plan id stays the same across runs.
- bridge `test_splitter.py`: `drop_rings` keeps the rows unchanged (edge-first and swaths-first).
- GUI `missionProgress.test.ts`: run tags and cross stretches.

## Sim results (seed 1, x3)

`SIM_GATE_OUT` must be an absolute path (a relative one fails the docker bind mount).

| World | Behaviour | Gate |
|---|---|---|
| `real_run5_cross` (AUTO + 90) | One plan: `include_rings=True` then `False`, `mowing the rows of run 2/2` once, no edge laps after it, 16/16 sub-paths, docked, 0 STUCK | FAIL e (1 of 68, 0.384 m), i (transit 0.386 m) |
| `real_run5_cross60` (20 + 60 = 80) | Same; run 2 planned at 80 deg; 22/22 sub-paths, docked, 0 STUCK | FAIL e (2 of 41, 0.155 m), h (line p99 0.318 m), i (transit 0.366 m) |

None of the failures come from this change:

- e on `real_run5_cross`: the 180 deg out-and-back spur at (-1.05, 3.95) at the end of the
  second row set is in the planner output. The old two-plan run (`simout_cross1111/cross-l-s1`)
  fails the same vertex (0.452 m) and 8 more corners (worst 0.799 m).
- h on `real_run5_cross60`: the tail is the approach at index 0 of short sub-paths (0.58 m
  falling to 0.15 m), in run 1's sub-paths (planned as before) as well as run 2's.
- i: a transit sample, about the same as the old run (0.374 m).

Against the old two-plan run on `real_run5_cross`: corners 9 to 1 failing, side overhang
0.317 to 0.059 m, line p99 5.54 to 0.095 m, moving time 1870 to 1659 s (-11 %, laps once).
