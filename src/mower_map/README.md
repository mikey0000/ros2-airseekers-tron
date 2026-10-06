# mower_map

Zone/map server for the Airseekers Tron. `map_server_node` is a Python port of the
MowgliNext `mowgli_map` `map_server_node` service and topic contract
(`third_party/mowglinext/mowgli_map`, GPL-3.0). The MowgliNext GUI edits mowing areas,
obstacles and the dock through it. Nav2 gets its keep-outs from it. The mission layer reads
areas, progress and boundary state from it.

- `mower_map/areas.py`: ROS-free core. It holds the data model, `areas.dat` I/O, the
  rasteriser, boundary checks, the recovery point and the vendor importer. It needs only
  numpy and PyYAML, not shapely.
- `mower_map/map_server_node.py`: the rclpy node.
- `mower_map/import_vendor_geojson.py`: the vendor map importer CLI.

```bash
ros2 launch mower_map map_server.launch.py maps_dir:=/ros2_ws/maps \
    robot_yaml_path:=/ros2_ws/config/gui/mowgli_robot.yaml
```

## Services (node name `map_server_node`)

| Service | Type | Behaviour |
|---|---|---|
| `/map_server_node/add_area` | `mowgli_interfaces/AddMowingArea` | Appends an area. It is a navigation area if either `is_navigation_area` flag is set. Obstacles take name and source from the index-aligned `obstacle_info`. Polygons with fewer than 3 points are rejected. Rebuilds the mask, saves `areas.dat` and publishes `replan_needed`. |
| `/map_server_node/get_mowing_area` | `mowgli_interfaces/GetMowingArea` | Returns `index` -> `MapArea` with `obstacle_info` (id, source, pending). Returns `success=false` past the end. |
| `/map_server_node/clear_map` | `std_srvs/Trigger` | Drops all areas and resets mow progress, then saves. The dock pose is kept. |
| `/map_server_node/save_areas` | `std_srvs/Trigger` | Writes `areas.dat`. |
| `/map_server_node/load_areas` | `std_srvs/Trigger` | Re-reads `areas.dat`. |
| `/map_server_node/set_docking_point` | `mowgli_interfaces/SetDockingPoint` | Applies the gates below, then sets position (request, or the mean `/odometry/filtered_map` position when `use_gps_position`). Yaw comes from PRESERVE / REQUEST / MOTION. Writes `dock_pose.yaml` (with `dock_pose_measured: true` when the yaw is real: MOTION, REQUEST, PRESERVE of a measured yaw, or PRESERVE with `use_gps_position` while the stored yaw is still the placeholder, which captures the docked robot's fused heading), and `robot_yaml_path` if set. Publishes `docking_pose` and `docking_pose_measured`. |
| `/map_server_node/promote_obstacle` | `mowgli_interfaces/PromoteObstacle` | Lookup order: `pending_id` accepts a pending proposal. Otherwise `polygon` is used as given. Otherwise `obstacle_id` is looked up in the last `/obstacle_tracker/obstacles`. Duplicates (centroid within 10 cm) are a no-op. Navigation areas are refused. |
| `/map_server_node/discard_obstacle` | `mowgli_interfaces/ClearObstacle` | Drops a PENDING obstacle by `MapObstacleInfo.id`. |
| `/map_server_node/get_recovery_point` | `mowgli_interfaces/GetRecoveryPoint` | When the robot is outside every area, returns the nearest edge point moved `boundary_recovery_offset_m` further in, plus the distance outside. |
| `/map_server_node/reset_mow_progress` | `std_srvs/Trigger` | Not in upstream. Zeroes the progress grid, e.g. when a new mow run starts. |
| `/map_server_node/set_area_settings` | `mower_interfaces/SetAreaSettings` | Not in upstream. Per-area mowing settings, see [Per-area mowing settings](#per-area-mowing-settings). `area_index` as in `get_mowing_area`, 255 = defaults. |
| `/map_server_node/get_area_settings` | `mower_interfaces/GetAreaSettings` | Not in upstream. The effective settings (area over defaults over built-in, every key present) as JSON; 255 = the defaults. `success=false` past the end. |

`set_docking_point` gates, as in upstream. Every gate is re-read on each call, so
`ros2 param set` works:

- `require_charging` (true): a `/mower_base/status` no older than
  `dock_set_status_max_age_s` (2 s) with `is_charging` or `is_docking_done` set.
- `require_rtk_accuracy` (true): a `/gps/status` (`GnssStatus`) no older than 2 s with a
  valid `horizontal_accuracy_m` <= `dock_set_gps_accuracy_max_m` (0.04).
- `require_yaw_converged` (true): at least `yaw_convergence_min_samples` (20) odometry
  yaws in the last `yaw_convergence_window_s` (5 s), with circular std <= 0.5 deg.
- `dock_gates_override` (false): skips every gate. Use it for bench testing only.

## Topics

| Topic | Type | QoS | Content |
|---|---|---|---|
| `/keepout_mask` (mowing mask) | `nav_msgs/OccupancyGrid` | latched | Frame `map`, `resolution` (0.1). Covers every area and the dock outline, plus `mask_margin` (2 m). Outside all areas = 100. Inside any mowing or navigation area = 0. Obstacles (grown by `obstacle_margin`) and the dock outline = 100. With no areas the mask is all 0, apart from the dock outline. | Coverage / mowing semantics; Nav2 does not use it.
| `/nav_keepout_mask` (navigation mask) | `nav_msgs/OccupancyGrid` | latched | Same grid as `/keepout_mask`. Free = mowing areas dilated outward by `nav_margin_m` (0.35 m = half footprint width 0.27 + 0.08; 0.8 in `config/map_server.yaml`) ∪ navigation areas / drawn paths dilated by only `path_margin_m` (0.15 m), so a drawn path stays a narrow band the planner follows instead of merging into one blob with the lawn margin, MINUS obstacles dilated by `nav_obstacle_margin_m` (0.10 m); everything else 100. The dock outline is NOT lethal here (it is a mowing exclusion; the robot parks inside it, so a lethal dock would make the start cell unplannable). The margin lets the robot centre run on a coverage ring ~0.09 m inside the boundary and reach the dock approach pose. Plus the **dock corridor** (`dock_corridor_enabled`, default true): every cell within `nav_margin_m` of the polyline dock -> approach pose (dock + `approach_distance` 0.8 m along the dock yaw, same parameter as mower_docking) -> closest point of the nearest mowing/navigation area boundary is free, so docking plans even when the dock lies outside the drawn areas; if that last gap exceeds `dock_corridor_max_m` (5.0) a warning is logged and only the dock->approach capsule is freed. When a user path exists (a navigation area with channel metadata) and `dock_corridor_soft_with_paths` (default true), only the dock -> approach capsule is freed; the automatic approach -> area leg is written as 90 (soft) so Nav2 prefers the drawn path but can still fall back on the straight line (the GUI labels the overlay "fallback"). Obstacles still override the corridor; the mowing mask is unaffected. Used by the Nav2 global costmap static layer and keepout filter. With no areas the mask is all 0 apart from obstacles. |
| `/nav_keepout_mask` soft band + return corridor | | | **Soft band** (`nav_soft_band_m`, 1.5 m): cells outside the free set (areas + `nav_margin_m` + dock corridor) but within 1.5 m of it are 90 instead of 100. The Humble KeepoutFilter scales mask 0..100 linearly to costmap 0..254 (100 = lethal), so 90 -> ~229: plannable but expensive. Which mechanism is live (verified on the mower 2026-10-07: every soft cell read 89 on `/global_costmap/costmap`, i.e. cost 229): the **KeepoutFilter** (`/costmap_filter_info` -> `/nav_keepout_mask`) applies it; the static layer was trinary and wrote the band as free underneath. Since 2026-10-07 the global `static_layer` is non-trinary (`trinary_costmap: false`, `lethal_cost_threshold: 100`, `unknown_cost_value: -1`) so it writes the same 229 itself and the band survives even without the filter. The local costmap has no nav-mask layer (obstacles only). **Return corridor** (`return_corridor_enabled`, true): whenever the robot pose (`odom_topic`) is not on a free cell, a capsule of half-width `nav_margin_m` from the robot to the closest free cell is freed (rebuilt when the robot moves > `return_corridor_rebuild_m` 0.5 m, cleared when it is back inside; the grid grows to include the robot). Gap > `return_corridor_max_m` (15): warning, no corridor. Obstacles override both. Rebuilds for the corridor do not publish `replan_needed`. |
| `~/return_corridor` | `geometry_msgs/PolygonStamped` | latched | Outline of the return corridor (empty when none). |
| `~/dock_corridor` | `geometry_msgs/PolygonStamped` | latched | Outline of the dock corridor freed in `/nav_keepout_mask` (map frame); empty polygon when disabled or no dock. |
| `/costmap_filter_info` | `nav2_msgs/CostmapFilterInfo` | latched | type 0 (keepout), `/nav_keepout_mask`, base 0, multiplier 1 |
| `/map_server_node/mow_progress` | `nav_msgs/OccupancyGrid` | latched | Same grid as the mask. A cell is 100 where the `blade_link` disc (`blade_radius` 0.15 m) passed while `/mower_base/status.is_cutting`. Republished every 2 s when it changes. |
| `/map_server_node/docking_pose` | `geometry_msgs/PoseStamped` | latched | Dock pose, frame `map` |
| `/map_server_node/docking_pose_measured` | `std_msgs/Bool` | latched | True only when the dock pose was recorded by `set_docking_point` (`dock_pose_measured: true` in `dock_pose.yaml`). False for the (0,0,0) placeholder. mower_docking refuses blind docking and heading_aligner refuses dock heading seeds while it is false. |
| `/map_server_node/boundary_violation` | `std_msgs/Bool` | depth 1 | True when the robot is outside all areas by more than `soft_boundary_margin_m` (0) for `boundary_debounce_samples` (3) checks. Checked at `boundary_check_rate_hz` (5). |
| `/map_server_node/lethal_boundary_violation` | `std_msgs/Bool` | depth 1 | True when the robot is outside all areas by more than `lethal_boundary_margin_m` (0.5) |
| `/map_server_node/replan_needed` | `std_msgs/Bool` | depth 1 | `true` on any area, obstacle or dock change |
| `/map_server_node/area_settings` | `std_msgs/String` | latched | JSON `{"defaults": {...every key...}, "areas": {"<area name>": {...only the area's own keys...}}}`. Every current area is listed (`{}` = all defaults). Republished on every settings change, area add, load and prune. |

The node subscribes to:

- `/odometry/filtered_map` (`odom_topic`), for robot pose and yaw.
- `/mower_base/status`, for `is_cutting`, `is_charging` and `is_docking_done`.
- `/gps/status`.
- `/obstacle_tracker/obstacles`, cached for `promote_obstacle`.

The blade offset is looked up once from TF (odometry `child_frame_id` -> `blade_link`). It
falls back to `blade_offset_x/y` (0.3, 0).

## Files (in `maps_dir`, default `/ros2_ws/maps`)

**`area_settings.yaml`**: per-area mowing settings, see [Per-area mowing settings](#per-area-mowing-settings).

**`areas.dat`** is byte-compatible with upstream `save_areas_to_file` /
`load_areas_from_file`, so files can be swapped in either direction. Coordinates are
float32, printed `%g`. Pending obstacles are never written. The datum stamp is written only
when the `datum_lat`/`datum_lon` params are non-zero.

```
# Mowgli ROS2 — Persisted areas and docking point
# Auto-generated by map_server_node. Do not edit manually.

datum_lat: 48.123456789          (optional)
datum_lon: 11.000000001

area_count: 1

area_0_name: lawn
area_0_polygon: 0,0;10,0;10,10;0,10
area_0_is_navigation: 0
area_0_obstacle_count: 1
area_0_obstacle_0: 4,4;6,4;6,6;4,6
area_0_obstacle_0_name: tree      (optional)
area_0_obstacle_0_source: 1       (optional, omitted for SOURCE_USER)
```

**`dock_pose.yaml`** is not in upstream, which keeps the dock only in `mowgli_robot.yaml`:

```yaml
dock_pose_x: 0.000000
dock_pose_y: 0.000000
dock_pose_yaw: 0.000000
dock_outline: "0.2,0.275;0.2,-0.275;-0.2,-0.275;-0.2,0.275"   # dock-local frame
dock_pose_measured: true   # only written by set_docking_point; absent = placeholder
```

When `robot_yaml_path` is set, `set_docking_point` also splices `dock_pose_x/y/yaw` into
that file in place. Comments and layout are kept, the same as upstream
`robot_yaml_scalar::UpdateDockPose`. That way the GUI sees the dock. On startup the dock is
read from `dock_pose.yaml`, falling back to `robot_yaml_path`. A dock without an outline
gets the default rectangle from `dock_outline_x_min/x_max/half_width`, which is the vendor
type-5 outline.

## Per-area mowing settings

Stored in **`area_settings.yaml`** next to `areas.dat` (parameter `area_settings_file`,
default `<dir of areas_file>/area_settings.yaml`), keyed by area **name** (areas.dat has no
stable id; two areas with the same name share settings), plus `defaults`:

```yaml
version: 1
defaults: {cutter_height_mm: 55}
areas:
  Front lawn: {path_mode: spiral, perimeter_laps: 3}
```

| Key | Type | Range | Default | Meaning |
|---|---|---|---|---|
| `cutter_height_mm` | int | 30-90 (Tron) | 50 | blade height; the mission maps it to the MCU percent |
| `perimeter_laps` | int | 0-4 | 2 | headland rings (vendor perimeter laps) |
| `path_mode` | string | `zigzag` `cross` `alternate` `spiral` `contour_only` | `zigzag` | vendor TaskUnit path_mode 0 zigzag, 1 cross, 2 alternate zigzag, 3 spiral; `contour_only` = vendor cut_mode 2 |
| `mow_angle_deg` | float | -1 or 0-180 | -1 (auto) | swath heading; < 0 = auto, >= 180 folded |
| `cut_speed_mps` | float | 0.05-0.5 | 0.3 | FollowCoveragePath speed while mowing |
| `swath_overlap_m` | float | 0-0.1 | 0.02 | swath spacing = cut width - this |
| `edge_first` | bool | | true | false = swaths first, then the perimeter laps |
| `repeat` | int | 1-10 | 1 | mow the area N times per session |
| `alternate_angle_offset_deg` | float | 0-180 | 90 | `alternate`: angle step per run (and per session); `cross` is always two passes 90 deg apart |

`set_area_settings` takes a JSON object and **merges** it into the stored values of that
area (or the defaults for 255): a key set to `null` is removed (back to the default), and
`{}` clears every value of that area. Unknown keys, wrong types and out-of-range values
reject the whole request (`success=false`, `message` says why). On success `message` holds
the effective settings. The file is written atomically and the latched topic republished.

Area edits: the GUI edits areas by `clear_map` + `add_area` of every area. Settings are
kept across `clear_map`; an area re-added under a **new name with the same polygon** (all
vertices within 1 cm) takes the old name's settings (rename). Entries whose area has not
come back `area_settings_prune_delay_s` (30 s) after a `clear_map` are dropped (delete).
`load_areas` does the same immediately (rename by polygon, then prune). A rename combined
with a polygon edit loses the settings (they cannot be matched).

## Importing the vendor map

```bash
ros2 run mower_map import_vendor_geojson /path/to/map.geojson --out /ros2_ws/maps/areas.dat \
    [--robot-yaml /ros2_ws/config/gui/mowgli_robot.yaml]
# then restart map_server_node, or: ros2 service call /map_server_node/load_areas std_srvs/srv/Trigger
```

Vendor coordinates are local metres with the origin at the rear axle on the charger and X
forward. That is our map frame when the datum is the dock. Only x and y are used: z, roll,
pitch and quality are dropped.

| Vendor `type` | Becomes |
|---|---|
| 1 area (Polygon) | Mowing area. Inner rings become obstacles. |
| 4 obstacle | Obstacle of the area named by `parent_id`, else the area containing its centroid, else the area it overlaps most. Otherwise it is dropped with a warning. |
| 4 obstacle containing the charge point | Dropped by default (`--keep-dock-zone-obstacles` keeps it). This is the vendor's dock no-mow zone. Live map: x -0.275..1.725. As a lethal keepout it would cover the dock and the undock point. |
| 3 channel (LineString) | Navigation area `channel <name>`: the line buffered by `--channel-half-width` (0.35 m) with square caps. It connects the dock to the area, which would otherwise be lethal. Use `--no-channels` to skip. |
| 5 dock outline | `dock_outline` in `dock_pose.yaml` (dock-local frame) |
| 6 charge_point, 7 undock_point | Dock pose. Position = charge point. Yaw = atan2 from charge point to undock point, which is the heading when leaving. |
| 8 RTK base | Ignored |

## Paths (channels)

A path is a drive-only connection between areas, or between an area and the
dock. On the vendor map these are the type-3 "channel" LineStrings. A path is
stored as an ordinary **navigation area**: its polygon is the centreline
buffered by half the width with square caps (each end is extended by half a
width). The map server needs nothing special for it:

- `/nav_keepout_mask` (the Nav2 global costmap mask) frees navigation areas
  like mowing areas (grown by `nav_margin_m`). This lets Nav2
  `navigate_to_pose` plan transits along the band, whether between areas or
  to the dock approach pose. The mission needs no changes.
- The band is never mowed. The mission and coverage only plan non-navigation
  areas (`mission_fsm.py`, `is_navigation_area` filter), navigation areas have
  no mow settings, and `promote_obstacle` refuses them. `/keepout_mask` (the
  "may be here" mask; nothing in this stack consumes it, Nav2 reads
  `/nav_keepout_mask`) keeps the upstream semantics, where every area is 0.

To let the GUI re-open a path as a line, `areas.dat` carries two optional
keys per path. Readers that do not know them ignore them:

```text
area_3_is_navigation: 1
area_3_channel: 1.2,-0.4;6.5,-0.4;9.8,2.1      # centreline, map metres
area_3_channel_width_m: 0.7
```

| Service | Type (reused) | Payload |
|---|---|---|
| `/map_server_node/set_area_channel` | `mower_interfaces/SetAreaSettings` | `area_index` plus `settings_json` `{"points": [[x, y], ...], "width_m": w}`. `{}` clears it. The area must be a navigation area. The call takes 2 or more finite points and a width of 0.1..5 m. It autosaves `areas.dat`. |
| `/map_server_node/get_area_channel` | `mower_interfaces/GetAreaSettings` | `settings_json` is the JSON above, or `{}` for a plain polygon. `success=false` past the end. |

The GUI map PUT is `clear_map` -> `add_area` x N (+ `set_area_channel` after each path)
-> `save_areas`. Its 5 s poll adds `get_area_channel` for every navigation area. The
results go into `Map.navigation_channels`. The vendor import keeps each channel's
centreline (width = 2 x `--channel-half-width`), so imported channels can also be
edited as lines.

GUI (Map -> Edit map, left toolbar):

- **Draw path** (node icon): click the vertices, then double-click (or press Enter) to
  finish. An end within 0.5 m of an area outline snaps onto the outline. The square cap
  then overlaps the area by half a width. An end within 0.5 m of the dock (or its approach
  point) snaps to the dock approach pose, which is 0.8 m in front of the charger. An end
  inside an area stays where it is. If an end touches nothing, the panel shows "not
  connected". The path is still allowed.
- **Path to dock** (home icon): the line starts at the dock approach pose. Click towards
  the area and finish inside it, or on its edge.
- The path panel (top right) offers the name and the width (0.5..3.0 m, default 0.7 m).
  It shows how each end is connected. When an end is at the dock, "continue into the dock"
  extends the band onto the charger. The line's vertices can be dragged on the map while
  the panel is open. **Save** adds the band, and **Save the map** (toolbar) persists it.
- **Edit path** (pencil icon, enabled with one path selected) re-opens a saved path as a
  line. From there you can drag vertices, change the width or name, or **Delete path**.
- Saved paths show as a translucent band with a dashed centreline.

## GUI flow

The MowgliNext GUI (via `mower_gui_bridge` / foxglove) does the following:

- Every 5 s it polls `get_mowing_area` index 0, 1, ... until `success=false`.
- A map PUT is `clear_map` -> `add_area` x N -> `save_areas`. `add_area` already autosaves.
- The dock is set with `set_docking_point`, and the GUI subscribes to
  `/map_server_node/docking_pose`.
- The mow overlay is `/map_server_node/mow_progress`.
- Obstacle actions call `promote_obstacle` / `discard_obstacle`.

## Notes and deviations from upstream

- Two masks, two semantics. `/keepout_mask` is the MOWING mask: the dock outline is lethal
  there when `dock_keepout` is true (no cutting over the charger) and everything outside
  the areas is lethal. `/nav_keepout_mask` is the NAVIGATION mask fed to Nav2: areas grown
  by `nav_margin_m`, obstacles grown by `nav_obstacle_margin_m`, and the dock outline is
  never lethal, so Nav2 can plan from the charger, to coverage rings near the boundary and
  to the docking approach pose (0.8 m ahead of the dock).
- Not ported: the grid_map classification layers, speed filter, soft-penalty (50) cells,
  dig-event pending proposals (the pending model and its accept/discard paths are there,
  but nothing creates proposals yet), datum migration (a datum mismatch is only logged),
  and the dock approach corridor carve-out.
- `use_gps_position` averages the fused `/odometry/filtered_map` position over the yaw
  window. Upstream averages raw RTK `/gps/fix` and corrects for the lever arm.
- `clear_map` persists immediately. Upstream only persists on the next add or save.

## Tests

```bash
python3 -m pytest src/mower_map/test                         # host, no ROS needed
./scripts/dev_build.sh test --packages-select mower_map      # colcon
```
