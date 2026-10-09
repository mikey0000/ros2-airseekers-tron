# Obstacle reroute: plan around obstacles instead of pausing

Date: 2026-10-09. Local edits only; nothing below has run on the mower yet.

## Problem

Stereo (`/stereo_depth/points`), YOLO (`/ai/det/obstacle_points`) and bumper (`/bumper_cloud`)
fed only the LOCAL costmap. The global costmap had the nav mask, terrain and inflation layers
and nothing else. So the mission's detour (`/navigate_to_pose`, 1 Hz replanning BT, NavFn)
planned straight through the obstacle. RPP's `use_collision_detection` then aborted with
"collision ahead" and the mission paused. When FollowPath failed, the BT's context recovery
(`ClearEntireCostmap` local) and the RoundRobin clearing erased the obstacle mark, so the robot
drove at the obstacle again.

## Change

All in `src/mower_navigation/config/nav2_params.yaml` and `behavior_trees/*.xml`:

- **Global costmap `obstacle_layer`.** Sources: `stereo` (marking + clearing, same band as the
  local costmap), `stereo_clear` (the new `/stereo_depth/clear_points` ground cloud from
  `mower_cameras`, clearing only), `det_range` and `bumper` (marking only).
  `footprint_clearing_enabled: false`, and `expected_update_rate: 0.0` on every source, because
  the Humble planner_server waits for `isCurrent()`. Plugin order: static, terrain, obstacle,
  inflation. `update_frequency` raised from 1 to 4 Hz.
- **Global inflation 0.6 m / scaling 10** (was 0.26 / 25). The inscribed radius is 0.25 m and
  the circumscribed radius is about 0.61 m. The lethal set is unchanged; the planner now keeps
  clearance from obstacles.
- **Local costmap.** Adds `stereo_clear` as a clearing-only source and grows the window from
  4 x 4 m to 6 x 6 m. There is still no inflation in the local costmap (the Humble `isCurrent`
  bug).
- **BTs.** The FollowPath context recovery is now `Wait 1 s` instead of `ClearLocalCostmap`.
  PipelineSequence keeps replanning during the wait, so FollowPath restarts on a path that goes
  around the obstacle. In the RoundRobin, `Wait 2 s` now runs first and clearing both costmaps
  runs second.
- **`stereo_costmap:=false`** (`launch/robot_settings.py`) strips `stereo`, `stereo_clear` and
  `det_range` from both costmaps.

## Live checks needed

1. Perimeter and dock reachability under the wider inflation. Re-run the planner-only plan
   dock -> (-1.80, -1.29) and a dock -> area transit.
2. planner_server CPU at 4 Hz. Each update re-inflates the full 500 x 500 rolling grid. If it
   nears one core, drop to 2 Hz.
3. False stereo marks now persist in the map frame until `stereo_clear` raytraces them. Watch
   for a lawn patch that stays lethal after the robot drives away.
4. `mower_mission` still clears both costmaps on transit retry and stuck escape
   (`mission.yaml clear_costmaps_on_retry: true`). That wipes the global marks too. Decide
   whether the mission should keep doing it.
5. The planner's context recovery (`ClearGlobalCostmap-Context` on a ComputePathToPose
   failure) also wipes the global marks. This is kept as the last resort for a false mark that
   blocks every path.
