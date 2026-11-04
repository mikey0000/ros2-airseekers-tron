# 2026-11-04 area 3: edge-lap branch, pivot after an RTK hold, resume after a pending zone

Field mow on 2026-11-04 (UTC 23:17 to 00:08, bag 20261103-231722), area 3 "further backyard
lawn". The owner approved all three fixes on the condition that they pass the sim.
Branch `ws/fix1104`, from master 41e61e3.

## 1. Edge lap: an obstacle that exists only in the costmap

### Field evidence

A branch lay across edge lap 0 at about map (-28.3, -15.3) to (-29.2, -14.0). No camera
classified it. It existed only as stereo-depth lethal cells in the local costmap. Each
time the lap reached it, MPPI logged "Optimizer fail to compute path", then "Controller
patience exceeded", and follow_path aborted. The mission then skipped part of the lap:

| UTC | Sub-path | Poses | At | Skipped |
|---|---|---|---|---|
| 23:24:47 | 0 | 689 -> 730 | (-27.66, -15.71) | 2.0 m |
| 23:29:56 | 2 | 722 -> 763 | (-29.62, -14.41) | 2.0 m |
| 23:36:43 | 4 | 571 -> 612 | (-29.41, -14.50) | 2.0 m |
| 23:47:39 | 4 | 854 -> 894 | (-28.80, -14.83) | 2.0 m |
| 23:49:08 | 4 | 1082 -> 1089 | (-28.66, -14.79) | 0.3 m |

Each row is the log line `edge blocked: skip ... follow_path aborted, collision ahead`.

The B6 inward detour on the same laps did run that mow, eight times, but only for no-go
zones and static marks (`edge lap: no-go zone / static mark on the edge lap ahead: detour
inward ...`).

### Cause

The abort path already tries the edge detour before it skips (master
`fsm/phases/transit.py:1044-1052`). The detour, however, only sees discs from
`_nogo_edge_discs` (master `fsm/phases/nogo.py:467`): tracker zones and a ranged static
detection. A costmap-only obstacle produces neither, so `_nogo_edge_detour` returns
False and `_edge_skip` runs. The mission had no input from the costmap at all.

### Fix (commit 26bf0e3)

This extends the existing detour trigger. It adds no new FSM guard, so owner rule A6
needs no matching removal.

- **Costmap input.** `mission_node.py:830-880` subscribes to `/local_costmap/costmap` and
  `/local_costmap/costmap_updates`. The local costmap uses the odom frame. Lethal cells
  (OccupancyGrid value 100) are moved into the map frame with map <- odom, computed from
  `/odometry/filtered_map` and `/odometry/filtered`. This avoids a TF listener. The FSM
  receives them as `inputs.costmap_lethal` with a receipt time.
- **Costmap discs.** `fsm/phases/nogo.py:484` `_nogo_costmap_discs` builds discs on a
  collision abort only (`collided` or a collision_monitor STOP). It takes fresh lethal
  cells (at most 3 s old) inside the area plus 0.05 m, within 0.6 m of the lap ahead. It
  then adds every cell 8-connected to those, so a branch reaching further into the area
  counts in full. Cells are binned to 0.10 m discs.
- **Detour.** `_nogo_edge_detour` (`fsm/phases/nogo.py:526`) takes these discs on that
  path only (`fsm/phases/transit.py:1048` passes `costmap=True`). For costmap discs the
  inward offset may reach `nogo_edge_costmap_max_offset_m` 1.5 m (zones keep 0.6 m), with
  a 1.2 m ramp. A zone has the re-plan as its alternative; a costmap obstacle only has
  the skip.
- **Chains of discs.** `nogo.edge_detour` (`nogo.py:358`) now uses, at each pose, the
  least inward offset whose body clears every disc beside it. Before, a disc that was
  out of reach of the unshifted body was ignored even when the shifted body ran into it.
  With one disc the result is unchanged.
- **Map tag (B6: "detour, tag on the map, rejoin").** The `edge_detour` incident is
  placed on the obstacle: the centroid of the cells the detour bends round
  (`fsm/phases/nogo.py:593`, `_incident(..., xy=)`).
- **Fallback.** The existing skip is still the fallback in these cases:
  - no detour fits within the offset ("no inward detour fits");
  - the body would exceed its limit (the existing "the detour would put the body ...
    beyond its limit: no detour" check);
  - the costmap is stale;
  - the detour itself aborts (`_nogo_detour_done` -> `_edge_skip`).
- **Blade.** It stays on in the detour (owner policy, unchanged).

New parameters in `mission.yaml`: `nogo_edge_costmap`, `_max_age_s` 3.0, `_corridor_m`
0.60, `_max_offset_m` 1.50, `_ramp_m` 1.20. New mission_node topics:
`local_costmap_topic`, `local_costmap_updates_topic`, `odom_frame_pose_topic` (each '' =
off).

## 2. An RTK-float hold mid-pivot skipped the rest of the pivot on resume

### Field evidence

| UTC | Log |
|---|---|
| 23:42:29 | `area 3 sub-path 4: pivot vertex at (-27.81, -15.19): pivot 135 deg in place` |
| 23:42:29 | `localization degraded (ekf_map sigma 0.17 m > 0.15 m (rtk float)): stopped, blade off` |
| 23:42:33 | `localization ok again after 4 s: resuming` -> `sending /follow_path goal` (no spin) |
| 23:42:41 | `BOUNDARY OUT: body front_right 0.57 m outside (soft limit 0.56)` |
| 23:53:12 | `sub-path 5: pivot vertex at (-27.56, -15.27): pivot 54 deg in place`, then the same hold |
| 23:53:23 | resume -> `/follow_path`, no spin |
| 23:53:27 | `BOUNDARY OUT ... 0.58 m`; STUCK_NEEDS_HELP at 23:53:31 (two exits within 1 m) |

### Cause

- `fsm/phases/pivots.py:92` sets `m.pivot_done = key` when the pivot STARTS.
- The hold (`fsm/holds.py` `_loc_hold_tick`; `_dyn_context` treats step `corner_pivot`
  as 'follow', `fsm/holds.py:333`) cancels the Spin. Its resume `Hold.resume` ->
  `_dispatch` -> `_begin_blade` -> `_follow` -> `_cov_pivot` then sees
  `m.pivot_done == key` (`fsm/phases/pivots.py:46`). It takes the pivot as done and
  follows the leg from the half-turned heading.
- The dynamic-obstacle hold has the same path.

### Fix (commit 4cebcc9)

`Hold.stop` (`fsm/hold.py:35`) is the stop shared by the localization hold and the
dynamic-obstacle hold. It now clears `pivot_done` / `pivot_target` when the step is
`corner_pivot`. The resume runs `_cov_pivot` again from the current pose and pivots the
remaining angle: the planned way round when the conditioner planned one, else
`_pivot_way`. If the robot is already within the pivot threshold, it pivots nothing.

Variants:

- **K-turn about-turn.** It uses the same `_cov_pivot` path and step `corner_pivot`, so
  the fix covers it.
- **Slope pre-drive.** The pivot after the pre-drive is a `corner_pivot`, so it is
  covered. On resume, `slope_pivot_key` was already consumed, so a remaining pivot gets
  its own pre-drive if the side slope still needs one.
- **Interrupting the pre-drive itself.** That step is `slope_pivot`, and `_dyn_context`
  has no hold for it, which is unchanged. The path-blocked wait (`_blk`) never starts
  from a Spin.

## 3. A resume re-planned away its progress because of an unreviewed pending zone

### Field evidence

- **23:53:31** STUCK_NEEDS_HELP. The obstacle_tracker proposed `no-go zone #1097
  (pending review): stuck r=1.00 m`.
- **00:02:35-37** The GUI sent commands 7, 8, 1. The mission logged
  `resuming (area 3 only)`.
- **00:02:47** The plan came back different: 541.0 -> 526.0 m, with a new fillet at
  (-28.88, -14.89). The mission logged `plan geometry changed, stale resume cursor
  discarded; kept 1 done sub-path(s)` and restarted the mow at sub-path 1, edge laps
  included. The owner called this "a big no no".
- **00:02:55** The zone was ignored in the GUI, too late.
- **23:36** The same clear / start sequence without a pending zone resumed correctly at
  sub-path 5/10.

### Causes

- **Pending zones reached the plan.** map_server's `get_mowing_area` returns every
  obstacle of the area, pending ones included (`obstacle_info[i].pending`).
  `mission_node.py` (master :1539) dropped the flag. The FSM planned every obstacle as a
  coverage hole (`_nogo_area_holes` -> `_start_plan`).
- **Mid-mow re-plans.** `nogo_replan_states` defaulted to `[pending, accepted]`, so a
  pending zone on the remaining path also triggered a mid-mow re-plan.
- **The cursor was discarded.** When the geometry changed, `_on_plan` (master
  `fsm/phases/areas.py:347`) kept only the sub-paths whose own fingerprint was
  unchanged. A hole near the edge changes every edge lap, so everything else restarted.

### Fix (commit ffe80e9)

**(a) Pending zones never shape the plan.**

- `mission_node.py:1613` passes `obstacle_pending` per obstacle.
- `fsm/common.py:1651` `accepted_area` moves pending obstacles out of `obstacles` into
  `pending_obstacles`. It is applied in the area enumeration (`fsm/holds.py:1055`) and
  in the preview (`fsm/phases/preview.py:77`).
- So pending zones are no longer plan holes, path conditioning region holes or boundary
  keepouts. The mission logs `area N: k pending (unreviewed) keepout(s) left out of the
  plan`.
- `nogo_replan_states` now defaults to `[accepted]` (`fsm/common.py:1458`,
  `config/mission.yaml`).
- Pending zones still act at run time through `/obstacle_tracker/nogo_zones`: the
  edge-lap detour, and the pre-pivot check, which skips the leg instead of re-planning.
  A pending zone on an interior swath no longer causes a re-plan; the costmap and
  obstacle avoidance still handle a real obstacle there.

**(b) A changed plan maps the resume cursor by position.**

- **Stored geometry.** The cursor stores the mowed part of the plan as simplified
  polylines (`area_mowed` rows, `resume.py:122/165`): completed sub-paths, the current
  one up to `resume_index`, minus skipped and excluded stretches (`resume.py:221`
  `mowed_pieces`, `:195` `simplify_polyline`, Douglas-Peucker on the segment distance,
  1 cm). `_cursor_mowed_update` (`fsm/phases/areas.py:401`) refreshes it on every persist
  (`fsm/core.py:457`), with the pieces cached per plan. Upstream readers ignore the
  unknown tag.
- **Mapping.** A plan whose geometry differs from the cursor goes through
  `_cursor_remap` (`fsm/phases/areas.py:351`). It uses the mid-mow re-plan's rule,
  `nogo.mowed_progress` (tolerance swath/2 + 0.02 m):
  - a sub-path at least 90 % mowed is done;
  - the mowed lead-in of the first other one is the resume point.

  Unchanged-fingerprint sub-paths still count as done.
- **Logging.** The result is logged as `resume cursor mapped onto the new plan by
  position: k/N sub-paths mowed, resume (sub, local)`. A cursor with progress that maps
  to nothing is logged as an ERROR (`... could not be mapped onto the new plan ...:
  mowing the area from its start`), never silently.

## Tests

Unit tests (mower_mission). Each fails on master 41e61e3 and passes on the branch: the
branch test files were run against the master code, 8 failed.

| Fix | Test |
|---|---|
| 1 | `test_nogo.py::test_edge_lap_costmap_only_obstacle_detours_tags_and_rejoins`, `..._too_deep_for_a_detour_is_skipped`, `test_edge_lap_stale_costmap_is_not_used` |
| 2 | `test_hold_pivot_resume.py`: loc hold and dynamic hold mid-pivot redo the pivot; a finished pivot is not redone |
| 3a | `test_nogo.py::test_restart_never_plans_a_pending_zone_as_a_hole`, `test_pending_zone_on_the_remaining_path_never_replans`, `test_cov_gate_e_pending_zone_in_the_sweep_skips_the_leg_no_replan` |
| 3b | `test_mission_fsm.py::test_resume_mapped_by_position_when_plan_geometry_changes` (across a cursor file reload), `test_resume_that_cannot_be_mapped_logs_an_error_and_starts_over`; `test_geometry_resume.py::test_area_mowed_rows_round_trip...` |

Tests changed on purpose:

- `test_resume_discarded_when_plan_changes` is replaced by the mapping tests.
- The re-plan mechanics tests in `test_nogo.py` now use accepted zones.
- `test_restart_plans_pending_zones_as_holes...` now plans accepted zones and leaves
  pending ones out.

## Sim

### Support added (commit 138ff2e)

- **mower_sim rtk anchor `pivot`.** Event times count from the first in-place rotation
  with the cutter on, held 0.4 s.
- **`sim_gate.sh --at T:cmd:N | T:stuck:X:Y`.** HighLevelControl commands and stuck
  incidents at sim times. Two stuck incidents at least 60 s wall apart make
  obstacle_tracker propose a pending zone, as in the field. An IDLE state does not end
  the run while steps are left.
- **World `worlds/owner_area3_fix1104.yaml`.** owner_far's Area 6 is the same polygon as
  area 3. It contains:
  - (i) a stereo-only 1.2 m branch across edge lap 0 at (-28.7, -14.9);
  - (ii) RTK float 12 s at the first bladed pivot;
  - (iii), via `--at`, a pending zone at (-28.0, -9.0), STOP at +1300 s, START at
    +1330 s.

### Runs: not yet run

On 2026-11-04 the host failed the gate preconditions: `powerprofilesctl get` = balanced and
load5 4.2-5.9. `sim_gate.sh` would set the profile itself, and the agent may not change
system settings, so no gate was run. The owner's approval needs these runs.

Run them one at a time with `SIM_GATE_OUT=<...>/ros2_stack-wt/simout_fix1104`. For the
master runs, use a master worktree that has this branch's `src/mower_sim` (rtk.py,
sim_node.py, the world) and `scripts/sim_gate.sh`.

| Run | Command (from the stack root) | Expected on the branch |
|---|---|---|
| scenarios | `sim_gate.sh -n fix1104-scen -w src/mower_sim/worlds/owner_area3_fix1104.yaml -x 2 -t 3600 --at 700:stuck:-28.0:-9.0 --at 900:stuck:-28.0:-9.0 --at 1300:cmd:8 --at 1330:cmd:1` | (i) `edge lap: ... (N costmap cells): detour inward`, then `edge detour done: rejoining`, no `edge blocked: skip` near (-28.7, -14.9); (ii) after the hold's `resuming`, a `pivot ... in place` before `/follow_path`, no BOUNDARY OUT; (iii) `1 pending (unreviewed) keepout(s) left out of the plan`, `resuming (k/N sub-paths done ...)` at the STOP's sub-path, no `plan geometry changed` |
| full gate | `sim_gate.sh -n fix1104-d1 -d -s 1 -x 1` | no new failures vs master, a/b clean, docked |
| fast gate | `sim_gate.sh -n fix1104-l3 -l -x 3` (launch.log "desired rate of 60") | as above |

| Criterion / scenario | master | branch | verdict |
|---|---|---|---|
| (i) costmap branch on lap 0 | not run | not run | pending |
| (ii) RTK hold mid-pivot | not run | not run | pending |
| (iii) STOP / START with a pending zone | not run | not run | pending |
| `-d -s 1 -x 1` a-i | not run | not run | pending |
| `-l -x 3` a-i | not run | not run | pending |

Known residual: criterion i also fails on master.

CI (`scripts/ci_test.sh`, branch 138ff2e): colcon PASS (3795 cases, 0 failed), scripts PASS,
replay PASS.
