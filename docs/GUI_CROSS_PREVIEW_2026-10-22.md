# Plan preview of the "Cross" pattern (2026-10-22)

Owner report: "when I select and plan cross, it only ever shows the first set of lines and
not the lines crossing over".

## What cross does

The per-area setting `path_mode` (GUI "Mowing pattern", `third_party/mowglinext/gui/web/src/utils/areaSettings.ts:6`)
can be `zigzag | cross | alternate | spiral | contour_only`. The coverage planner and the
coverage bridge treat `cross` and `alternate` as plain zigzag (`coverage_planning.cpp`
`parsePathMode`, `coverage_server_node.py` `_planner_path_mode`). The crosshatch is a
**mission-level** mode. `mission_fsm._build_runs` turns `cross` into two runs per repeat,
at `angle` and `angle + 90` (on an AUTO angle, the second run sets the `perpendicular` flag).
The mission mows both row sets in the same session. `alternate` is different: it turns the
angle by `alternate_angle_offset_deg` between sessions, using
`mission_alternate_counts.json`.

## Root cause

The plan preview (`POST /mowglinext/plan/preview` -> `/mission/preview_plan`,
`mission_fsm._preview_on_service` / `_preview_on_plan`) built the full run list but
planned only `cur['runs'][0]`. The summary segments and `/coverage/full_plan` therefore
held only the first row set. The GUI drew exactly what it received. The planner, the bridge
and the GUI were all correct.

## Fix (branch `ws/gui-cross`, `mower_mission` only)

The preview now plans every *distinct* direction of an area's runs (`_preview_runs`), one
plan after another, and concatenates them into the area's result: segments, transits,
counts, length and the drawn path. Cross shows both row sets. Zigzag with `repeat` > 1
still plans once. For alternate it shows the directions the next session would mow. The
per-area summary gains `preview_runs`, `run_angles_deg` and `run_perpendicular`. A failure
in the second run is reported as `run 2/2: <message>`.

Tests: `src/mower_mission/test/test_mission_fsm.py` `test_preview_cross_*` and
`test_preview_zigzag_repeat_plans_once_and_cross_second_failure_reports`.

Deploy: `mower_mission` is Python, so a sync plus a restart of the stack container is enough.
The GUI does not change, so no GUI image rebuild is needed.
