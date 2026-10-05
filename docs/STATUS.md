# Airseekers Tron ROS 2 port: status

Last updated 2026-10-05. Read this first. The point-in-time audits that fed it are in
`docs/audit_2026-10-05/` (stack audit, MowgliNext reuse study, vendor mission-layer reconstruction).

## Goal

A working ROS 2 Humble lawn mower on the Tron hardware (RK3588 brain, 6 MCUs over serial, UM960
RTK, WIT JY61P IMU, Metoak stereo, RK3588 NPU), controlled by the user through the MowgliNext web
GUI: joystick drive, zone recording and editing, start/pause/home, docking and scheduling. The
vendor ROS 1 stack is the behavioural reference; MowgliNext supplies the user layer and the mission
architecture; everything runs in the Humble container on the stock 20.04 kernel.

## What exists and works (verified 2026-10-05)

All 16 packages under `src/` build in the amd64 dev image (`docker/Dockerfile.dev-amd64`,
`scripts/dev_build.sh build|test|shell`), except `src/open_vins`, which is a symlink to the external
OpenVINS clone and is skipped. Pytest, run against stubbed ROS on the host:

| Package | Tests |
|---|---|
| `mower_mcu_driver` | 29 |
| `um960_gps_driver` | 47 |
| `mower_localization` | 25 |
| `base_ble` | 14 |
| `wit_imu_driver` | 8 (new protocol tests) |
| `stereo_vio_bridge` | 6 |
| `det_ros` | 4 |

Fixes landed the same day as the audit:

- `bumper_controller`: compile errors fixed (PID member names, missing `finishToIdle` declaration, generic-lambda service callback, default-arg constructor). It now publishes unstamped `Twist` on its own lane `/cmd_vel_bumper` instead of `/cmd_vel`.
- `mower_coverage`: Fields2Cover 2.1 API fixes (`get_Area`, non-const `SGObjective&`).
- `mower_interfaces`: `MappingControl.srv` and `SetLoRa.srv` registered in `CMakeLists.txt`.
- `gps_gate` and `um960_gps_driver` use the real Humble `NavSatStatus` values. RTK fixed and float map to `GBAS_FIX`, DGPS to `SBAS_FIX`; the gate separates fixed from float by covariance. Launch default `min_fix_status` is 0.
- `wit_imu_driver` rewritten for the real 11-byte WIT protocol (`wit_imu_driver/wit_protocol.py`, unit-tested) and publishes `/imu/data` directly. The `/imu` remap in `launch/nav2.launch.py` is gone.
- `mower_mcu_driver` gained `/cutter_control`, `/charging`, `/clear_estop`, `/cutter_off`, publishes `/mower_base/status` (MowerBaseDevStatus) and `/estop` (Bool), has a host-side lift/stop interlock plus an e-stop latch (`/estop_request`), sends cutter-off on shutdown, moved the placeholder IMU to `/mcu/imu`, and subscribes unstamped `Twist` on `/cmd_vel` (plus `/cmd_vel_stamped`). Default `linear_max` is 0.5 m/s.
- `base_ble` publishes `Twist` on `/cmd_vel_teleop`, calls `/cutter_control` and `/clear_estop` (Empty).
- `cmd_vel_slew` and `slip_detector` use unstamped `Twist`.
- `mowgli_interfaces` vendored verbatim at `src/mowgli_interfaces` (GPL-3.0). The MowgliNext GUI and reference packages (`mowgli_map`, `mowgli_behavior`, `mowgli_coverage`, `mowgli_nav2_plugins`, `mowgli_monitoring`, `mowgli_bringup`) are vendored under `third_party/mowglinext/` with `LICENSE` and `UPSTREAM_COMMIT`.
- `.gitignore` for `build/`, `install/`, `log/`, pycache.

## Architecture and contract

Humble convention throughout: unstamped `geometry_msgs/Twist` on every command topic (Humble
`twist_mux` 4.3 and Nav2 Humble have no stamped mode).

Command chain: lanes -> `twist_mux` -> `/cmd_vel_raw` -> `cmd_vel_slew` -> `/cmd_vel` -> `mcu_node`.

| Lane | Priority | Source |
|---|---|---|
| `/cmd_vel_nav` | 10 | Nav2 controller / velocity smoother |
| `/cmd_vel_docking` | 15 | docking controller (Phase C) |
| `/cmd_vel_teleop` | 20 | GUI joystick via `cmd_vel_ws_relay` (port 8766), `base_ble` |
| `/cmd_vel_bumper` | 50 | `bumper_controller` back-up and rotate |
| `/cmd_vel_emergency` | 100 | mission layer emergency zero |
| lock `/estop` | | `mcu_node` (Bool, true while interlocked or latched) |

Driver contract (`mcu_node`): publishes `/odom`, `/battery`, `/mower_sensor_info`,
`/mower_base/status`, `/estop`, `/mcu/imu`; subscribes `/cmd_vel`, `/cmd_vel_stamped`,
`/estop_request`; serves `/cutter_control` (CutterControl), `/charging` (ChargingControl),
`/clear_estop` (Empty), `/cutter_off` (Trigger). `wit_node` publishes `/imu/data`. `um960_node`
publishes `/fix`, `/fix_status`, `/vel`, `/heading`. `gps_gate` republishes `/fix_gated`.
`ekf_node` publishes `/odometry/filtered` and owns `odom -> base_link`.

Frames: `map` (static identity to `odom` for Phase A) -> `odom` (EKF) -> `base_link` (rear axle)
-> `base_footprint`, `imu_link`, `gps_link`, `blade_link`, `cutter_link`.

GUI adapter (`mower_gui_bridge`, in progress): presents our topics under the names the MowgliNext
GUI hard-codes: `/hardware_bridge/{status,emergency,power}` and services
`/hardware_bridge/{mower_control,emergency_stop,reboot_board}`, `/gps/{fix,status}`,
`/wheel_odom`, `/odometry/filtered_map` (frame `map`), and a stub mission layer
`/behavior_tree_node/{high_level_control,high_level_status,start_in_area,clear_coverage_resume,coverage_resume_available}`
covering IDLE, RECORDING (blade off), MANUAL_MOWING (blade on) and STOP with MowgliNext
`state_name` strings. Blade-on is refused unless the high-level state is 2 or 4.

Ports: GUI `4006`, `foxglove_bridge` `8765` (Humble ships 3.5.0, which speaks `foxglove.sdk.v1`
as the GUI expects), teleop relay `8766`.

## Completed later the same day (2026-10-05, parallel agents)

All of the following build in the amd64 dev image and were exercised in-container (no hardware):

- `src/mower_gui_bridge` (`gui_bridge`): the adapter above, 30 tests. Blade-on is refused outside HL states 2/4 or during an emergency; blade-off is never refused; MCU telemetry older than 1 s counts as an emergency. See its README.
- `src/mower_teleop`: `cmd_vel_ws_relay` (ws :8766, Twist out, clamps 0.5 m/s / 1.0 rad/s, 0.25 s lease) + `twist_mux.yaml` (lanes above, `/estop` lock) + `teleop.launch.py`. Verified: GUI-style JSON frame -> `/cmd_vel_teleop` -> mux -> slew -> `/cmd_vel`, and the `/estop` lock zeroes it.
- `src/mower_bringup` + `launch/mower.launch.py`: single entry point (drivers, robot_state_publisher, static `map -> odom`, control nodes, teleop, gui_bridge, foxglove_bridge 3.5.0, localization, optional navigation). `docker/docker-compose.yml` now runs it; `Dockerfile.humble` carries every dependency (websockets comes from pip: Jammy's apt 9.1 is broken on Python 3.10). URDF gained `base_footprint` and `blade_link`. `ekf.yaml` int/float covariance fix (rcl refused to load it).
- `src/mower_navigation`: Humble `nav2_params.yaml` + `navigation.launch.py`; all lifecycle nodes activate; `FollowPath`/`FollowCoveragePath` RPP controllers, `general_goal_checker`/`coverage_goal_checker`, NavFn, behaviors, velocity_smoother -> `/cmd_vel_nav`, keepout filter slot (`use_keepout`).
- Datum pinning: `datum_lat`/`datum_lon`/`datum_yaw` launch args on `mower.launch.py` -> `navsat_transform_node` (`wait_for_datum: true` when set). Must match `config/gui/mowgli_robot.yaml`.
- GUI image: `gui/Dockerfile` + `gui/build.sh` (Go cross-compile + vite on the x86 host, final arm64 stage copies files only; 159 MB), hidden Firmware/GNSS/Drive-tuning/Updates/Remote-access/rosbag pages via `web/src/tronFeatures.ts`, `ONBOARDING_COMPLETED` env, `docker/docker-compose.gui.yml`, `docs/gui.md`. Smoke-tested on amd64 and handshake confirmed against foxglove_bridge 3.5.0.
- `src/mower_proto`: regenerated with protoc 3.12.4 (Jammy apt); `generate_proto.sh` refuses newer protoc.
- `src/mower_coverage`: 3 real planner bugs fixed (swath ends overshot the boundary by op_width/2 with headlands off; wrong `atan2` for the reported angle; longest-edge angle computed on the widened cell). 15/15 gtests.
- Test suites now pass both on the host (stubs) and under real rclpy in the container: `colcon test` = 188 tests, 0 failures, 21 skipped (stub-only tests skip under rclpy). A real bug surfaced: `mcu_node._on_imu` assigned 36-element covariances to 9-element Imu fields (fixed).
- `third_party/COLCON_IGNORE`: colcon must not crawl the vendored MowgliNext packages (they target a newer Nav2 and broke the workspace build).
- `scripts/deploy_to_mower.sh`: rsync to `/userdata/ros2_stack` on the device (root fs is 100 % full, /userdata has 18 GB), image build and low-parallelism colcon build on the mower, up/down/logs/shell. Not yet run: the mower dropped off the LAN during the session (vendor ROS stack was NOT running on it; Docker 26 and an older `mower:humble` image are present).

## Roadmap

**Phase A: joystick plus GUI (this week).** The items above. Exit criterion: the stock MowgliNext
GUI on port 4006 shows pose, battery, GPS and e-stop state, and drives the mower with the joystick
in RECORDING and MANUAL_MOWING with the blade interlocked by the MCU driver.

**Phase B: zones and coverage.**
- Port `third_party/mowglinext/mowgli_map` `map_server_node` (grid_map from apt). It owns zone CRUD, `areas.dat` (plain key/value: `datum_lat/lon`, `area_<i>_polygon` in map-frame metres, obstacles), the keepout mask, mow progress and dock pose. Needs `base_footprint` and `blade_link` TF.
- `/plan_coverage` action adapter (`mowgli_interfaces/action/PlanCoverage`) over our Fields2Cover 2.1 `mower_coverage` service, splitting the path into `drivable_subpaths` at transits.
- Mission BT: port `mowgli_behavior` `main_tree.xml` minus LiDAR, fusion_graph and `DockRobot` (Humble `nav2_msgs` lacks `DockRobot` and `CollisionMonitorState`), or write our own BT reusing `coverage_persistence.cpp` and the `FollowStrip` blade rules (blade off before any transit over 0.6 m and at every sub-path boundary).
- Nav2 keepout filter fed by `/keepout_mask`.

**Phase C: docking and scheduling.**
- Docking: the vendor docks in reverse on an ArUco marker seen by the rear camera (`mower_charge`: search, dock, final straight reverse at 0.05 m/s, retry; contact confirmed by `is_docking_done` debounced over 3 samples, then `/charging true`). Either reimplement that or backport `opennav_docking`. Undock is a `BackUp` of 0.8 m to the vendor `undock_point` (0.8, 0).
- Scheduler, notifications, statistics, HomeKit and the MQTT bridge in `third_party/mowglinext/mowgli_monitoring` work unchanged once the mission layer emits the real `state_name` set.

Vendor facts that constrain Phases B and C (details in `docs/audit_2026-10-05/vendor_mission_layer.md`):

| GeoJSON `type` | Meaning |
|---|---|
| 1 | mowing area polygon |
| 3 | channel (dock to area) line |
| 4 | obstacle / no-go polygon |
| 5 | dock outline polygon (0.4 x 0.55 m) |
| 6 | `charge_point` (0, 0) |
| 7 | `undock_point` (0.8, 0) |
| 8 | RTK base station |

The vendor map frame is local metres with the origin at the rear axle while on the charger, X
forward, Y left; geographic anchoring comes from `charging_station_gps` and
`charging_station_orientation`. The vendor cutter interlock order is `/controller/ctrl pause`
first, then cutter off, before every transit, dock and pause. The vendor local HTTP API on port
13344 offers `/task/{start,pause,resume,stop,dock,unDock}`, `/map/{save,list,delete}`,
`/task/getCoveragePath`, `/system/clearAlarm`, `/motor/cutter/control` and a task-info
WebSocket on 13345; the MowgliNext GUI replaces it, but it remains the reference for an
Airseekers-app-compatible surface later.

## Hardware bring-up (mower on the ground, 2026-10-06)

Verified live on the mower with the stack running from Docker (`mower:humble`, `/userdata/ros2_stack`):

- Serial links: MCU 67-100 Hz sensor/speed frames, battery 22.0 V, firmware v0.6.36 / v0.6.34; WIT IMU 100 Hz
  on `/imu/data` (RELIABLE QoS, the EKF receives it); UM960 10 Hz, 29 satellites, rover board in LoRa mode
  ("network RTK OFF" = default); EKF 30 Hz; GUI on :4006 connected to foxglove and the relay.
- Heartbeat period 100 ms confirmed (vendor tap). The vendor forwards IMU to the MCU (~44 Hz): implemented.
- Lift flag: cutter firmware latches it after ~17 s; released on the mower by the module-10 clear frame with
  the IMU forwarded as pitch/roll ~0 (`docs/mcu_protocol_spec.md` 10c). `/clear_estop` does exactly that.
- Drive: `/cmd_vel_teleop` 0.1 m/s for 2 s moved the mower; MCU measured 0.07-0.11 m/s, both motors drew current,
  clean stop. Chain GUI/relay -> twist_mux -> cmd_vel_slew -> MCU proven on hardware.
- NPU: `best_large_0208.rknn` loads with runtime 2.3.0, inference 86 ms, 9 outputs as det_ros expects.
- Power key = `/dev/input/event5` ("key input") `KEY_P` (25); `base_keys` is being wired to it.

Still to verify: cutter command (speed/height scale), rain/bumper/stop inputs, RTK fix with the LoRa base or NTRIP,
dock contact (`is_docking_done`) and charging, side cameras through the rkisp pipeline, rear camera stream.

## Open risks

- GPLv3: the GUI and `mowgli_interfaces` are GPL-3.0 with a commercial option. Fine for personal use; a product decision later.
- The JY61P yaw is gyro-integrated, not earth-referenced. `navsat_transform` needs `yaw_offset` or a GPS heading source; `/heading` from the UM960 is only useful if it is dual-antenna (unverified).
- The MCU clear-estop frame is unknown, so a firmware e-stop latch may need a power cycle.
- MowgliNext mission packages target a newer Nav2 (`DockRobot`, `CollisionMonitorState`), so `mowgli_behavior` cannot be built on Humble as-is.
- `map -> odom` is a static identity for now. The `datum_lat`/`datum_lon`/`datum_yaw` launch args exist (`nav2.launch.py`, forwarded by `mower.launch.py`);
  the value is still to be set per site (dock position, same as `config/gui/mowgli_robot.yaml`).
  `map -> odom` remains static identity.
