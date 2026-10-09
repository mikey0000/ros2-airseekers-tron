> Study date: 2026-10-05. Produced by a Claude Opus subagent (read-only study of `/tmp/mowglinext-src` @ `faf658b`, now vendored under `third_party/mowglinext/`).
> Point-in-time snapshot: several items it lists as "needed on our side" were started the same day. See `docs/STATUS.md` for the current state.

# MowgliNext user layer: what we can lift into the Airseekers Tron Humble stack

Research was read-only against `/tmp/mowglinext-src` @ `faf658b`. Nothing was modified. The best sources in that repo are its own generated indexes: `docs/claude/ros-interfaces.md` (every topic and service, grep-confirmed), `docs/claude/high-level-api.md`, and `docs/claude/codemaps/{gui_backend,gui_frontend,mowgli_behavior,mowgli_map}.md`. I spot-checked their claims against source.

## 0. Corrections to what we assumed

1. **`app/` is empty.** `app/mobile/` holds only a stale Vite cache (`.vite/deps/_metadata.json`, `package.json`). `docs/FIRST_BOOT.md:122` lists "a mobile app" as *not shipped*. The mobile story is the GUI as a **PWA** (`gui/web/public/manifest.json`, an iOS add-to-home-screen banner), plus MQTT, HomeKit and push notifications. There is no native app to lift.
2. **Licence.** The repo is GPLv3 plus a commercial licence (`LICENSE`). GUI files carry no per-file headers. Most C++ files are `GPL-3.0` (headers like "Copyright 2026 Mowgli Project" / "Cedric"). The exception is `mowgli_coverage/package.xml`, which declares **BSD-3-Clause**, but its `.cpp` files carry a bare copyright line and no licence text, so treat it as ambiguous. Anything we lift is GPLv3 for us unless we buy the commercial licence. That decision blocks any product use.
3. **The GUI's foxglove client offers only subprotocol `foxglove.sdk.v1`** (`gui/pkg/foxglove/client.go:179`). That is the newer SDK-based foxglove_bridge 3.x. I could not check whether `ros-jazzy-foxglove-bridge` in our snapshot is 3.x or the old 0.8.x, which speaks `foxglove.websocket.v1`. If it is old, add the second subprotocol to that list (one line), and also check JSON client-publish and the services capability.
4. **The joystick only streams while the mission layer reports `state_name == "RECORDING"` or `"MANUAL_MOWING"`** (`gui/web/src/pages/map/hooks/useMapStreams.ts:377-393`). Joystick drive therefore needs at least a stub `high_level_control` / `high_level_status` server, not just a twist relay.
5. **The scheduler ignores its own `Area` field.** `gui/pkg/providers/scheduler.go:172-177` stores `schedule.Area` but sends only `HighLevelControl{command:1}`.

## 1. Component inventory

### 1.1 Web GUI: `gui/` (Go backend + React SPA)

- **Size and language:** Go 1.25 (gin, gorilla/websocket, bitcask, mochi-mqtt, brutella/hap, docker SDK), about 24k non-test lines including 2.6k of generated swagger. React 19 + TypeScript + Vite + antd 5 + mapbox-gl 3 + mapbox-gl-draw, about 48k lines and 448 source files. i18n is en/fr only.
- **Licence:** GPLv3/commercial (repo-level).

**What the user gets:**
- Dashboard with state, battery, progress and weather.
- Mapbox map with live pose, coverage plan, mow-progress raster and obstacles.
- Map editor: draw, split, merge and undo areas, navigation areas and obstacles, set the dock point, import OpenMower maps.
- Joystick overlay for manual mowing and area recording.
- Start / Pause(Stop) / Home / Resume-vs-start-fresh / E-stop re-arm.
- Weekly schedule.
- Session statistics and heatmap.
- Settings editor (a schema-driven `mowgli_robot.yaml` editor), a live ROS parameter editor, diagnostics, container logs, rosbag record/download, host reboot/poweroff.
- Push notifications (Telegram, Pushover, ntfy, webhook), an embedded MQTT broker, a HomeKit switch.
- Their own firmware flashing, GNSS configurator and drive-PID tuner. All three are irrelevant to us.

**Non-ROS interfaces:**
- HTTP on `:4006`: SPA, `/api/*` and `/swagger/`. There is **no auth**, and WebSocket `CheckOrigin` requires the Origin host to equal the Host header.
- Browser WebSockets: `/api/mowglinext/multiplex` (msgpack frames), `/api/mowglinext/subscribe/:key` (base64 text) and `/api/mowglinext/publish/joy`. The publish route ignores its path parameter and always publishes `/cmd_vel_teleop`.
- Upstream links: `ws://localhost:8765` (foxglove_bridge) and `ws://localhost:8766` (`cmd_vel_ws_relay`).
- Bitcask key-value store at `$DB_PATH`. Keys: `schedule:<id>`, `mowing.sessions`, `onboarding.completed`, `notifications.*`, `system.*`.
- Files: reads and writes `MOWER_YAML_CONFIG_FILE` (`mowgli_robot.yaml`). It needs `asserts/mower_config.schema.json` **relative to the working directory**.
- Docker socket. Container names `mowgli-ros2` and `mowgli-gps` are hard-coded for the rosbag, drive-tuning and GNSS tools.

**ROS contract it consumes** (`gui/pkg/providers/ros.go` `topicMap`). These are exact names, and all the `mowgli_interfaces/*` type strings are hard-coded:

| GUI key | Topic | Type |
|---|---|---|
| status | `/hardware_bridge/status` | `mowgli_interfaces/msg/Status` (uses `mow_enabled`, `is_charging`, `mower_motor_rpm`, `mower_esc_status`, `rain_detected`, `firmware_*`) |
| highLevelStatus | `/behavior_tree_node/high_level_status` | `mowgli_interfaces/msg/HighLevelStatus` (`state`, `state_name`, `current_area`, `coverage_percent`, `battery_percent`, `is_charging`, `emergency`, …) |
| emergency / power | `/hardware_bridge/emergency`, `/hardware_bridge/power` | `Emergency{active_emergency,latched_emergency,lift_warning,reason}`, `Power{v_battery,v_charge,charge_current,charger_enabled}` |
| pose, fusionRaw | `/odometry/filtered_map` | `nav_msgs/Odometry` in the **map frame = ENU metres about `datum_lat/lon`** |
| gps / gnssStatus | `/gps/fix`, `/gps/status` | `NavSatFix`, `mowgli_interfaces/msg/GnssStatus` |
| imu, ticks, wheelOdom | `/imu/data`, `/wheel_ticks`, `/wheel_odom` | `Imu`, `WheelTick`, `Odometry` |
| path / plan | `/coverage/full_plan`, `/plan` | `nav_msgs/Path` |
| mowProgress | `/map_server_node/mow_progress` | `OccupancyGrid` (transient_local) |
| map (virtual) | polls `/map_server_node/get_mowing_area` (index 0..N until `success=false`, every 5 s) plus `/map_server_node/docking_pose` (PoseStamped) | `GetMowingArea` → `MapArea{name, area, obstacles[], is_navigation_area, obstacle_info[]}` |
| others | `/scan`, `/fusion_graph/*`, `/obstacle_tracker/obstacles`, `/diagnostics`, `/behavior_tree_log`, `/robot_description`, `/behavior_tree_node/recording_trajectory`, `/behavior_tree_node/coverage_resume_available` (Bool, latched), `/imu/cog_heading`, `/imu/mag_yaw`, `/calibrate_imu_yaw_node/dock_calibration/status` | optional; a missing topic just shows no data |

**Services it calls** (`gui/pkg/api/mowglinext.go:541-724`, 10 s timeout each):
- `/behavior_tree_node/high_level_control` (`HighLevelControl{command}`: 1 start, 2 home, 3 record, 5 finish recording, 6 cancel recording, 7 manual mow, 8 stop)
- `/behavior_tree_node/start_in_area` (`StartInArea{area}`)
- `/behavior_tree_node/clear_coverage_resume` (Trigger)
- `/hardware_bridge/emergency_stop` (`EmergencyStop{emergency}`)
- `/hardware_bridge/mower_control` (`MowerControl{mow_enabled,mow_direction}`)
- `/map_server_node/{add_area,clear_map,save_areas,set_docking_point,promote_obstacle,discard_obstacle}`
- `/navsat_to_absolute_pose/set_datum`
- `/hardware_bridge/reboot_board`
- `/fusion_graph_node/{save_graph,clear_graph,clear_lidar_map}`
- `/obstacle_tracker/clear_obstacle`

Map save (`PUT /api/mowglinext/map`) is `clear_map` → `add_area`×N → `save_areas`.

**It publishes** `/cmd_vel_teleop` (`TwistStamped`), via the 8766 relay when that is connected, otherwise via foxglove `clientAdvertise` with JSON encoding. Browser caps are 0.25 m/s and 0.6 rad/s (`useManualMode.ts:15-16`), sent every 100 ms.

**Assumptions that differ from ours:**
- Every `mowgli_interfaces` type and node name is hard-coded.
- The map frame must be ENU about a fixed `datum_lat/datum_lon` read from `mowgli_robot.yaml` (`utils/map.tsx`, `METERS_PER_DEG`). Our `navsat.yaml` uses `wait_for_datum: false` (origin = first fix) and has no `map→odom`, so the GUI map would be wrong until we pin a datum.
- An onboarding redirect fires until DB key `onboarding.completed` is true.
- Firmware, GNSS and drive-tuning pages assume their STM32 and universal-gnss.
- `useMowerAction` depends on response shapes (`{message}` vs `OkResponse`).

**How to run:** `cd gui && make build` (4-stage Dockerfile: golang:1.25 → node:22 → ubuntu:22.04 with openocd + platformio → final). Use `network_mode: host`. For us, strip the openocd/platformio stage; arm64 is fine because the build uses `CGO_ENABLED=0`. Environment: `FOXGLOVE_URL`, `MOWER_YAML_CONFIG_FILE`, `DB_PATH`, `WEB_DIR`, `API_ADDR`. Dev loop: `MOWGLI_API_TARGET=http://<ip>:4006 yarn dev`.

**Effort: needs an adapter node** (plus vendoring `mowgli_interfaces`). The GUI never touches ROS except through foxglove, so if we satisfy the topic/service names and types above it works unmodified. Hide or disable the firmware, GNSS, drive-tuning, updater and remote-access pages (small frontend edits). Map and datum correctness depends on our localisation.

### 1.2 `mowgli_interfaces`

- **Path:** `ros2/src/mowgli_interfaces/` (16 msg, 15 srv, 3 action, header-only helpers `gnss_status_utils.hpp`, `wgs84_projection.hpp`, `coverage_geometry.hpp`, `robot_yaml_scalar.hpp`).
- **Licence:** GPL-3.0.
- **Dependencies:** only `builtin_interfaces`, `std_msgs`, `geometry_msgs`, `nav_msgs`.
- **Effort: drop-in.** Vendor it **verbatim under the same package name**, because the GUI and every Mowgli node reference `mowgli_interfaces/...` type strings. Keep our `mower_interfaces` alongside it for the vendor contract.

### 1.3 Teleop relay: `ros2/src/mowgli_bringup/scripts/cmd_vel_ws_relay.py`

- **Size:** about 150 lines of Python (rclpy + `websockets`).
- **What it does:** listens on WS `:8766` for JSON `{"twist":{linear,angular}}`, clamps to 2.0 m/s and 5.0 rad/s, publishes `/cmd_vel_teleop` (TwistStamped, RELIABLE depth 10), and replays the last command every 50 ms within a 125 ms lease.
- **Effort: drop-in.** Lower the clamps for the Tron, and re-tune the lease to whatever the vendor MCU's `type=255` heartbeat and command watchdog turn out to be (still uncharacterised).

### 1.4 `twist_mux` configuration: `ros2/src/mowgli_bringup/config/twist_mux.yaml`

Uses `use_stamped: true` and no `locks:` block. Lanes:

| Lane | Topic | Priority | Timeout |
|---|---|---|---|
| navigation | `/cmd_vel_monitored` | 10 | 0.6 s |
| docking | `/cmd_vel_docking` | 15 | 0.5 s |
| teleop | `/cmd_vel_teleop` | 20 | 0.5 s |
| tuning | `/cmd_vel_tuning` | 30 | 0.5 s |
| emergency | `/cmd_vel_emergency` | 100 | 0.2 s |

Output goes to `/cmd_vel`.

**Effort: drop-in**, with two changes on our side:
- Our `cmd_vel_slew` reads `/cmd_vel_raw` and writes `/cmd_vel`, so remap the mux output to `/cmd_vel_raw`.
- `src/bumper_controller/src/bumper_controller.cpp:39` publishes an **unstamped `Twist` on `/cmd_vel`**. That conflicts with the TwistStamped `/cmd_vel` our MCU driver subscribes to and bypasses the mux. Move it to a lane such as `/cmd_vel_emergency` as TwistStamped.

### 1.5 Mission layer: `ros2/src/mowgli_behavior` (`behavior_tree_node`)

- **Size and language:** C++17, BehaviorTree.CPP v4. `trees/main_tree.xml` is 1,359 lines; together with `mowgli_map` the package source is about 31k C++ lines including 19 gtest targets. 56 registered nodes.
- **Licence:** GPL-3.0.
- **What it does:** the whole mission state machine; see §2.

**ROS contract (served):**
- `~/high_level_control` → `/behavior_tree_node/high_level_control`
- `~/start_in_area`
- `~/clear_coverage_resume`
- publishes `~/high_level_status` (state transitions plus a 1 Hz republish), `~/coverage_resume_available`, `~/recording_trajectory`, `/coverage/full_plan`, `/controller_server/FollowCoveragePath/global_plan`, `/cmd_vel_emergency`, `/cmd_vel_nav` (escape manoeuvre), `/cmd_vel_teleop` (yaw-seed manoeuvre), `/fusion_graph_node/set_pose`

**Subscribes:**
- `/hardware_bridge/{status,emergency,power}`; **more than 2 s silence on `emergency` is treated as an emergency**
- `/cmd_vel`
- `/map_server_node/{replan_needed,boundary_violation,lethal_boundary_violation}`
- `/odometry/filtered_map`, `/gps/absolute_pose`, `/gps/status`
- `/collision_monitor_state`
- `/scan_collision` (liveness only)
- `/global_costmap/costmap`

**Clients:**
- `/hardware_bridge/mower_control`, `/hardware_bridge/emergency_stop`
- `/map_server_node/{get_mowing_area,add_area,get_recovery_point}`
- `/plan_coverage` (action)
- `/follow_path` with `controller_id="FollowCoveragePath"` and `goal_checker_id="coverage_goal_checker"`
- `/navigate_to_pose`, `/backup` (`nav2_msgs/BackUp`, which is the undock)
- `/dock_robot`
- costmap clear services, `keepout_filter/toggle_filter`, lifecycle `manage_nodes` / `is_active`
- `/controller_server` `set_parameters`

TF lookups are `map→base_footprint`, **hard-coded**.

**Assumptions that differ from ours:**
- Includes `nav2_msgs/action/dock_robot.hpp`, `undock_robot.hpp` and `nav2_msgs/msg/collision_monitor_state.hpp`. **None of these exist in Humble's nav2_msgs.**
- Assumes the `opennav_docking` `docking_server`, a GTSAM `fusion_graph` (`/fusion_graph_node/set_pose`), LiDAR liveness (`IsScanStale`), and the FTC controller.
- Assumes firmware HL-mode mirroring: their firmware zeroes the blade unless the HL mode is MOWING or MANUAL. Our vendor MCU will not do this.
- `base_footprint` is required.

**Effort: needs rework.** Port the tree in stages:
- Strip the LiDAR, fusion_graph-calibration and escape branches.
- Stub `CollisionMonitorState`.
- Replace `DockRobot` with our own dock routine or a Humble build of opennav_docking. I could not verify whether a Humble opennav_docking backport or binary exists; `mowglinext_integration.md` contradicts itself on this (§4.4 vs §6.1).

The tree XML, `coverage_persistence.cpp`, `recording_nodes.cpp`, `battery_filter.cpp`, `status_nodes.cpp`, `localization_health.hpp` and the `FollowStrip` blade logic are the highest-value pieces.

### 1.6 Zones and map storage: `ros2/src/mowgli_map` (`map_server_node`, `obstacle_tracker_node`)

- **Language:** C++, depends on grid_map (`ros-jazzy-grid-map-*` is apt-installable).
- **Licence:** GPL-3.0.
- **What it does:** area CRUD, `areas.dat` persistence, the Nav2 keepout mask, the mow-progress raster, dock pose, boundary violation, and pending/accepted dig keepouts.

**ROS contract:**
- **Services:** `/map_server_node/{add_area (AddMowingArea), get_mowing_area (GetMowingArea), clear_map, save_areas, load_areas, set_docking_point (SetDockingPoint), promote_obstacle, discard_obstacle, discard_dig_keepouts_near_robot, get_recovery_point}`.
- **Publishes:** `/keepout_mask` + `/costmap_filter_info` (latched), `~/mow_progress`, `~/docking_pose`, `~/boundary_violation`, `~/lethal_boundary_violation`, `~/replan_needed`.
- **Subscribes:** `/hardware_bridge/status`, `/odometry/filtered_map`, `/gps/fix`, `/gps/pose_cov`, `/global_costmap/costmap`, `/hardware_bridge/dig_event`.

**File format** (`area_manager.cpp:1510-1570`, `/ros2_ws/maps/areas.dat`), a plain-text key/value file:

```
datum_lat: <deg>
datum_lon: <deg>
area_count: N
area_<i>_name: <name>
area_<i>_polygon: <points>
area_<i>_is_navigation: 0|1
area_<i>_obstacle_count: M
area_<i>_obstacle_<j>: <points>
area_<i>_obstacle_<j>_name: <name>
area_<i>_obstacle_<j>_source: 0|1|2
```

Polygons are in metres in the map frame. The dock pose is **not** in this file; it is spliced into `mowgli_robot.yaml` (`dock_pose_x/y/yaw`). Datum migration re-projects everything if the datum changes.

**Assumptions:**
- `base_footprint` and `blade_link` TF frames are hard-coded (`map_server_node.cpp:695`, `area_manager.cpp:880` for `base_footprint→gps_link`).
- Mow-progress stamping needs `Status.mow_enabled`, `mower_esc_status != 0`, RPM ≥ 1000 and blade telemetry younger than 1 s.
- `set_docking_point` rejects unless charging, GPS σ ≤ 4 cm and yaw has converged.
- `/map` input is dead.

**Effort: needs an adapter node, light patching.** It builds on Humble with no Lyrical-only APIs that I found, though I did not compile it. Feed it from the adapter's `/hardware_bridge/status` and add `base_footprint` and `blade_link` to our URDF. This one package gives the GUI zones, keepouts and progress.

### 1.7 Coverage planner: `ros2/src/mowgli_coverage` (`coverage_server`, `/plan_coverage` action)

Fields2Cover **v3** source build: headland rings, boustrophedon swaths, then hole-free `drivable_subpaths`. Our `mower_coverage` already exists as a `PlanCoverage` **service** on apt F2C 2.1, returning one `nav_msgs/Path` (`src/mower_interfaces/srv/PlanCoverage.srv`).

**Effort: needs an adapter node.** Write a thin action server named `/plan_coverage` of type `mowgli_interfaces/action/PlanCoverage` that calls our service and splits the path into `segments`, `segment_types` and `drivable_subpaths` at transits. Porting their server instead means a source F2C v3 build on arm64, which is not worth it now.

### 1.8 Coverage path follower: `mowgli_nav2_plugins` (`FTCController` + `PathProgressGoalChecker`)

Already planned in `mowglinext_baseline.md` §7.2, so I did not re-audit it. It is what the BT's `FollowStrip` targets by plugin name; RPP could stand in at first.

### 1.9 MQTT bridge: `mowgli_monitoring/mqtt_bridge_node` (C++) and `docs/MQTT_CONTROL.md`

- **Outbound** (QoS 1, default prefix `mowgli`): `<prefix>/{status,power,emergency,high_level_status,position,gps,diagnostics,available}`.
- **Inbound:** `<prefix>/command` → `HighLevelControl`.
- **Effort: drop-in once the adapter exists.** This is the stable contract for Home Assistant or a phone app. The GUI's embedded broker (`gui/pkg/providers/mqtt.go`, prefix `/gui`) is a separate, cruder copy.

### 1.10 Smaller GUI-side features (all inside the GUI container, no new ROS needs)

| Feature | Code | Notes |
|---|---|---|
| Scheduler | `gui/pkg/providers/scheduler.go` | 1-min tick; `safeToStart` requires no emergency and state not NULL/AUTONOMOUS/RECORDING; then calls command 1. Area is ignored (see §0). |
| Session tracker | `session_tracker.go` | Driven by `highLevelStatus` + `wheelOdom`. |
| Notifications | `notify_*.go` | Keyed on `state_name` strings such as `MOWING_COMPLETE`, `DIG_OBSTRUCTION`, `*_FAILED`. |
| Weather | open-meteo at the datum | — |
| HomeKit switch | `:8000` | ON → START, OFF → HOME. |
| Rosbag tool | — | Assumes container `mowgli-ros2`. |

**Effort: drop-in** once `high_level_status` and `high_level_control` exist. To get correct labels and notifications, our mission layer must emit their `state_name` vocabulary (35 values, listed in `docs/claude/codemaps/mowgli_behavior.md` around line 179).

## 2. How MowgliNext structures the mission layer

**Single command entry point.** Everything calls `/behavior_tree_node/high_level_control`: GUI, scheduler, MQTT, HomeKit and firmware buttons (via `hardware_bridge`). Single status output: `/behavior_tree_node/high_level_status`. State codes: 0 NULL/emergency, 1 IDLE (including docked, charging, stop-hold, rain wait, complete), 2 AUTONOMOUS (including RETURNING_HOME), 3 RECORDING, 4 MANUAL_MOWING. `state_name` carries the detail.

**Tree shape.** The root is a `ReactiveSequence` of guards evaluated every tick: EmergencyGuard → SensorSafetyGuard → BoundaryGuard → LocalizationGuard → GPSModeSelector → Nav2ResumeGuard → MainLogic. Every guard handler must end in `<AlwaysFailure/>`, which a test enforces. Guards that pause rather than end a mow tick `<MarkGuardHalt>` first.

**Start → mow → done:**
1. `COMMAND_START`.
2. PreFlightCheck: battery, fix type ≥ 2 on the dock, TF available.
3. Undock = `BackUp` of `undock_distance` (default 1.0 m) at `undock_speed`. Then `WaitForGpsFix(min_fix_type=4)`, then heading calibration from the undock line fit.
4. `GetNextUnmowedArea`: skips navigation areas; retires an area after 5 dispatches with no progress.
5. `PlanCoverageArea`: fetches the area via `get_mowing_area`, sends it to `/plan_coverage` with `mow_angle_deg` (-1 = auto), publishes `/coverage/full_plan`.
6. `FollowStrip`: each `drivable_subpath` becomes **one** `FollowPath` goal on the FTC controller. Transits between sub-paths use `/navigate_to_pose` with the blade off. Obstacle detours happen inside a sub-path.
7. When all areas are done: `MOWING_COMPLETE`, then home.

**Pause, resume, stop:**
- `COMMAND_STOP` (8) runs `StopHoldSequence`: blade off, halt in place, state 1. Nav2 is not paused.
- `COMMAND_START` resumes from the persisted cursor.
- The resume cursor lives in `/ros2_ws/maps/coverage_resume.txt` (`coverage_persistence.cpp`, header `mowgli_coverage_resume v2`, atomic tmp+rename). It stores `current_command`, `single_area_target`, `current_area`, `completed_areas` and per-area pose_count / fingerprint / resume index / completed swaths.
- On boot, mowing auto-continues only if `current_command` was START and a resumable snapshot exists.
- `~/clear_coverage_resume` means "start fresh". The GUI shows Resume vs Start-fresh from `coverage_resume_available`.

**Docking and charging:**
- `COMMAND_HOME` (2) runs `HomeSequence`: `/dock_robot` (opennav_docking `SimpleChargingDock`, `staging_x_offset: -1.5`, `charging_threshold: 0.3`), using `/battery_state.current` and `Status.is_charging`.
- Low or critical battery triggers a dock, then a charge hold in `CHARGING` / `CRITICAL_BATTERY_CHARGING`, then auto-resume at `battery_full_percent`. A manual "Resume now" is allowed above `battery_manual_resume_percent`.
- Rain triggers dock and wait.
- The dock pose comes only from `set_docking_point` (which needs charging plus RTK σ ≤ 4 cm) or the dock-calibration action.

**Recording and manual mode:**
- `COMMAND_RECORD_AREA` (3) samples the pose at 10 Hz (points under 5 cm apart are dropped) while the user drives with the joystick.
- `COMMAND_RECORD_FINISH` (5) simplifies the track (Douglas-Peucker, 0.05 m) and calls `add_area`. `COMMAND_RECORD_CANCEL` (6) discards it.
- `COMMAND_MANUAL_MOW` (7) is teleop **with the blade on**; the BT enables the blade after publishing state 4. Recording mode is the blade-off joystick mode.

**Blade and safety interlocks:**
- The blade is commanded only by the BT, through `/hardware_bridge/mower_control`.
- `FollowStrip::sendCurrentSwath` forces the blade off before any transit longer than 0.6 m and at every sub-path boundary.
- EmergencyGuard turns the blade off and streams zero velocity on `/cmd_vel_emergency` (priority 100).
- The e-stop is a firmware latch reached via `/hardware_bridge/emergency_stop`, not a twist_mux lock. Re-arm happens on the dock automatically, or from the GUI.
- `collision_monitor` filters only the Nav2 lane, so teleop bypasses it.
- The dig detector in `hardware_bridge` latches `dig_escalated` and puts the BT into `DIG_OBSTRUCTION`.
- The firmware mirrors the HL state and refuses the blade while IDLE or NULL. We lose this backstop with the vendor MCU, so our adapter or driver must refuse cutter-on unless the HL state is 2 or 4.

**Zones:** stored in `areas.dat` (format above) by `map_server_node`. The GUI edits them by service calls only, never by touching the file. Keepouts reach Nav2 as `/keepout_mask` through `keepout_filter`.

## 3. Recommended integration order

### Phase A: joystick-drivable mower with the stock web UI (about 1–2 weeks, no Nav2 needed)

1. Vendor `mowgli_interfaces` verbatim.
2. Run `foxglove_bridge` in our Humble image and confirm its subprotocol and version (§0.3). Patch the GUI dialer if needed.
3. Add `twist_mux` with the Mowgli lanes, output `/cmd_vel_raw` → `cmd_vel_slew` → `/cmd_vel`. Fix `bumper_controller`'s unstamped `/cmd_vel`. Add `cmd_vel_ws_relay.py` with Tron-safe clamps.
4. Write a new **`mower_gui_bridge`** adapter node (Python, roughly 400–600 lines):
   - From `mower_mcu_driver` data, publish `/hardware_bridge/status`, `/hardware_bridge/emergency` (≥ 1 Hz always, because silence reads as an emergency) and `/hardware_bridge/power`.
   - Serve `/hardware_bridge/mower_control` → our cutter control, refusing unless HL state is 2 or 4.
   - Serve `/hardware_bridge/emergency_stop` → vendor `/clear_estop` (and set, if the MCU allows).
   - Publish `/gps/status` (GnssStatus from `um960_gps_driver`).
   - Remap `/odom` → `/wheel_odom` and `/imu` → `/imu/data`.
   - Republish the EKF pose as `/odometry/filtered_map` with `frame_id: map`.
   - Provide a **stub** `/behavior_tree_node/high_level_control` and `high_level_status` covering IDLE ↔ RECORDING (blade-off joystick), MANUAL_MOWING (blade-on joystick) and STOP, using MowgliNext's `state_name` strings.
5. **Pin the map datum**: set `navsat_transform` `wait_for_datum: true` with a fixed `datum` equal to the `datum_lat/lon` written into a minimal `mowgli_robot.yaml`, and give it a `map→odom` publisher. Add `base_footprint` (identity to `base_link`) and `blade_link` to the URDF and point the EKF at `base_footprint`.
6. Build a slimmed GUI image (drop openocd/platformio). Pre-set `onboarding.completed=true`. Hide the Firmware, GNSS, Drive-tuning, Updates and Remote-access UI.

### Phase B: zones and coverage

7. Bring up `mowgli_map/map_server_node` on Humble (grid_map from apt). This makes the GUI map editor, dock point and keepout mask live; recording then calls `add_area`.
8. Write a `/plan_coverage` action adapter over our F2C 2.1 `mower_coverage` service.
9. Execution: either port `mowgli_behavior` minus docking, LiDAR and fusion_graph (a Humble patch for the `DockRobot` and `CollisionMonitorState` includes), or a smaller BT of our own that reuses `main_tree.xml`'s mow/stop/record branches, `coverage_persistence.cpp` and the `FollowStrip` blade rules. Start with RPP and port FTC after. Add the `keepout_filter` to our Nav2 configuration.

### Phase C: docking and scheduling

10. Docking: write a dock/undock state machine using `BackUp` for undock and the MCU charge contact (mod 3) for confirmation, or a Humble opennav_docking build (availability unverified). Keep `set_docking_point`'s gates.
11. The scheduler, notifications, statistics and MQTT/Home Assistant work for free once the BT emits the real `state_name` set. Fix the scheduler's ignored `Area` field (use `start_in_area`).
12. `mqtt_bridge_node` is optional, as the stable external API for any Airseekers phone integration.

## 4. Could not verify

- The `ros-jazzy-foxglove-bridge` version and subprotocol in our snapshot.
- Whether a Humble opennav_docking build exists.
- That `mowgli_map` and `mowgli_behavior` actually compile on Humble (only includes and dependencies were inspected).
- Vendor MCU behaviour for host-initiated e-stop, cutter commands and the heartbeat watchdog. The relay's 125 ms lease assumes a 200 ms firmware watchdog that our MCU may not have.

Nothing was built or run.
