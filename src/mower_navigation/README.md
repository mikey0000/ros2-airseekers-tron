# mower_navigation

Nav2 (ROS 2 Humble, Nav2 1.1.x) config and launch for the Airseekers Tron.
This package holds only launch files and config (ament_cmake, Apache-2.0).
Localization (gps_gate, navsat_transform, ekf) stays in `launch/nav2.launch.py`.

```bash
ros2 launch mower_navigation navigation.launch.py            # params_file, use_keepout:=false, autostart:=true
python3 src/mower_navigation/test/test_params_yaml.py         # static YAML check (python3 + pyyaml)
```

## Nodes

| Node | Plugins |
|---|---|
| `controller_server` (+ `local_costmap`) | `FollowPath`, `FollowCoveragePath` = RegulatedPurePursuit; goal checkers `general_goal_checker` (0.15 m / 0.25 rad), `coverage_goal_checker` (0.10 m, yaw ignored); `progress_checker` = SimpleProgressChecker (0.3 m / 15 s) |
| `planner_server` (+ `global_costmap`) | `GridBased` = `nav2_navfn_planner/NavfnPlanner` |
| `behavior_server` | `spin`, `backup`, `drive_on_heading`, `wait` |
| `bt_navigator` | Humble default trees (copies in `behavior_trees/`, see below) |
| `velocity_smoother` | OPEN_LOOP, max vel [0.5, 0, 1.0] |
| `lifecycle_manager_navigation` | autostart; nodes in the order above |
| `costmap_filter_info_server` + `lifecycle_manager_costmap_filters` | only with `use_keepout:=true` |

This package does not start map_server or AMCL.

## Frames and topics

`map` -> `odom` (a static identity for now) -> `base_link` (from ekf_node) -> `base_footprint` (URDF).
Every node uses `robot_base_frame: base_link`. Odometry comes from `/odometry/filtered`.

Velocity chain (unstamped `geometry_msgs/Twist`):

```
controller_server  cmd_vel          -> /cmd_vel_nav_raw
velocity_smoother  /cmd_vel_nav_raw -> /cmd_vel_nav     (twist_mux nav lane, priority 10, timeout 0.6 s)
behavior_server    cmd_vel          -> /cmd_vel_nav
```

Costmaps:
- **Global:** frame `map`, rolling window 50x50 m at 0.1 m. Layers are `inflation_layer` and filter `keepout_filter` (info `/costmap_filter_info`, mask `/keepout_mask`). It has no static layer.
- **Local:** frame `odom`, rolling window 4x4 m at 0.05 m. Layers are `obstacle_layer` (fed from `/bumper_cloud`, marking only, range 1.0 m, persistence 2 s, footprint clearing off) and `inflation_layer` (radius 0.4, scaling 3.0).

Footprint: `[[0.52,0.27],[0.52,-0.27],[-0.23,-0.27],[-0.23,0.27]]`. It comes from the chassis, wheel and bumper sizes in `config/urdf/mower.urdf.xacro`, measured from base_link.

## Coverage controller ids

The MowgliNext behaviour tree asks controller_server for `controller_id="FollowCoveragePath"` and `goal_checker_id="coverage_goal_checker"` on swaths. Transit uses `FollowPath` with `general_goal_checker`.
- Today both ids map to stock Humble plugins (RPP at 0.25 m/s and SimpleGoalChecker).
- When `mower_nav2_plugins` ports FTCController and PathProgressGoalChecker, change only the `plugin:` lines. The ids stay the same.

Because two goal checkers are loaded, Humble rejects a FollowPath goal that has no `goal_checker_id`. For that reason `behavior_trees/` holds copies of the Humble default trees with `goal_checker_id="general_goal_checker"` added. navigation.launch.py injects them into bt_navigator.

## Keepout

The `keepout_filter` is always configured. If no `/costmap_filter_info` arrives it logs "Filter mask was not received" once and stays idle. This does **not** block lifecycle activation (verified).
- `use_keepout:=true` also starts `costmap_filter_info_server`, which announces `/keepout_mask`.
- Leave it false once the ported MowgliNext `map_server_node` publishes the filter info itself.

## Deferred

- **Real `map -> odom`:** it is a static identity today. A GPS datum or localization back-end will publish it later.
- **Keepout mask source:** `/keepout_mask` will come from the ported MowgliNext `map_server_node`. Nothing publishes it yet.
- **Docking:** Humble has no `opennav_docking`. Undock = `BackUp`. Docking goes to mower_behavior.
- **collision_monitor:** there is no scan source, so it would only pass commands through. It is omitted.
- **FTCController / PathProgressGoalChecker:** waiting on the `mower_nav2_plugins` port.
- **Custom MowgliNext BTs:** waiting on the mowgli_behavior port.
