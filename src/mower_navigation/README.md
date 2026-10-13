# mower_navigation

Nav2 (ROS 2 Jazzy, Nav2 1.1.x) config and launch for the Airseekers Tron.
This package holds only launch files and config (ament_cmake, Apache-2.0).
Localization (gps_gate, navsat_transform, ekf) stays in `launch/nav2.launch.py`.

```bash
ros2 launch mower_navigation navigation.launch.py            # params_file, use_keepout:=false, autostart:=true, transit_controller:=mppi|rpp
python3 src/mower_navigation/test/test_params_yaml.py         # static YAML check (python3 + pyyaml)
python3 src/mower_navigation/test/nav_stack_smoke.py          # in the dev image, with the stack launched (see its docstring)
```

## Nodes

| Node | Plugins |
|---|---|
| `controller_server` (+ `local_costmap`) | `FollowPath` = MPPI (transit, since 2026-10-09; `transit_controller:=rpp` puts `FollowPathRPP` back under this id), `FollowPathRPP`, `FollowCoveragePath` = RPP (default coverage), `FollowCoveragePathMPPI`, `FollowCoveragePathFTC`, `FollowCoveragePathReverse`; goal checkers `general_goal_checker`, `coverage_goal_checker`, `coverage_goal_checker_ftc`, `coverage_leg_goal_checker`; `progress_checker` = PoseProgressChecker |
| `planner_server` (+ `global_costmap`) | `GridBased` = `nav2_smac_planner/SmacPlannerHybrid` (DUBIN, r_min 0.45 m, full footprint check), `GridBasedNavFn` = NavFn (BT fallback) |
| `behavior_server` | `spin`, `backup`, `drive_on_heading`, `wait` |
| `bt_navigator` | Nav2 default trees (Humble 1.1.x copies in `behavior_trees/`, updated to Jazzy's BT.CPP v4 format) |
| `velocity_smoother` | OPEN_LOOP, max vel [0.5, 0, 0.3] |
| `collision_monitor` | stop_zone / slowdown_zone (forward only) + approach_footprint; sources stereo, bumper |
| `collision_zone_gate` | plain node: enables the stop/slowdown zones only while driving forward |
| `lifecycle_manager_navigation` | autostart; nodes in the order above (collision_monitor last) |
| `costmap_filter_info_server` + `lifecycle_manager_costmap_filters` | only with `use_keepout:=true` |

This package does not start map_server or AMCL.

## Frames and topics

`map` -> `odom` (a static identity for now) -> `base_link` (from ekf_node) -> `base_footprint` (URDF).
Every node uses `robot_base_frame: base_link`. Odometry comes from `/odometry/filtered`.

Velocity chain (unstamped `geometry_msgs/Twist`):

```
controller_server  cmd_vel               -> /cmd_vel_nav_raw
velocity_smoother  /cmd_vel_nav_raw      -> /cmd_vel_nav_smoothed
collision_monitor  /cmd_vel_nav_smoothed -> /cmd_vel_nav   (twist_mux nav lane, priority 10, timeout 0.6 s)
behavior_server    cmd_vel               -> /cmd_vel_nav
```

Docking (`/cmd_vel_docking`), teleop, bumper and emergency lanes do not pass through the
collision_monitor. Nav2's stop/slowdown polygons ignore the direction of travel, so
`collision_zone_gate` switches them off for reverse legs and pivots (the approach polygon
is direction-aware and always on). Details in the `collision_monitor` block of
`config/nav2_params.yaml`.

Costmaps:
- **Global:** frame `map`, rolling window 50x50 m at 0.1 m. Layers `static_layer` (`/nav_keepout_mask`), `terrain_layer`, `obstacle_layer` (stereo, stereo_clear, det_range, bumper), `inflation_layer` (0.65 m >= circumscribed radius, scaling 10); filter `keepout_filter`.
- **Local:** frame `odom`, rolling window 6x6 m at 0.05 m. Layers `obstacle_layer` (bumper, stereo, stereo_clear, det_range), `bounds_keeper` (source-less ObstacleLayer with footprint clearing: workaround for the InflationLayer reset bug in Humble 1.1.20, kept until re-verified on Jazzy) and `inflation_layer` (1.0 m, scaling 4; MPPI's footprint check needs it).

Footprint: `[[0.52,0.27],[0.52,-0.27],[-0.23,-0.27],[-0.23,0.27]]`. It comes from the chassis, wheel and bumper sizes in `config/urdf/mower.urdf.xacro`, measured from base_link.

## Coverage controller ids

The MowgliNext behaviour tree asks controller_server for `controller_id="FollowCoveragePath"` and `goal_checker_id="coverage_goal_checker"` on swaths. Transit uses `FollowPath` with `general_goal_checker`.
- Today both ids map to stock Jazzy plugins (RPP at 0.25 m/s and SimpleGoalChecker).
- When `mower_nav2_plugins` ports FTCController and PathProgressGoalChecker, change only the `plugin:` lines. The ids stay the same.

Because two goal checkers are loaded, Nav2 rejects a FollowPath goal that has no `goal_checker_id`. For that reason `behavior_trees/` holds the Nav2 default trees (the Humble 1.1.x copies, updated with `BTCPP_format="4"` for Jazzy) with `goal_checker_id="general_goal_checker"` added. navigation.launch.py injects them into bt_navigator.

## Keepout

The `keepout_filter` is always configured. If no `/costmap_filter_info` arrives it logs "Filter mask was not received" once and stays idle. This does **not** block lifecycle activation (verified).
- `use_keepout:=true` also starts `costmap_filter_info_server`, which announces `/keepout_mask`.
- Leave it false once the ported MowgliNext `map_server_node` publishes the filter info itself.

## Deferred

- **Real `map -> odom`:** it is a static identity today. A GPS datum or localization back-end will publish it later.
- **Keepout mask source:** `/keepout_mask` will come from the ported MowgliNext `map_server_node`. Nothing publishes it yet.
- **Docking:** `opennav_docking` exists on Jazzy, but we use `mower_docking`; Undock = `BackUp`.
- **FTCController:** ported (`src/mowgli_nav2_plugins`, `FollowCoveragePathFTC`), not the default since the 2026-10-06 ring drift.
- **Custom MowgliNext BTs:** waiting on the mowgli_behavior port.
