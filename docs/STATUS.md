# Airseekers Tron ROS 2 port: status

Last updated 2026-10-05. Read this first. The point-in-time audits that fed it are in
`docs/audit_2026-10-05/` (stack audit, MowgliNext reuse study, vendor mission-layer reconstruction).

## Goal

A working ROS 2 Jazzy lawn mower on the Tron hardware (RK3588 brain, 6 MCUs over serial, UM960
RTK, WIT JY61P IMU, Metoak stereo, RK3588 NPU), controlled by the user through the MowgliNext web
GUI: joystick drive, zone recording and editing, start/pause/home, docking and scheduling. The
vendor ROS 1 stack is the behavioural reference; MowgliNext supplies the user layer and the mission
architecture; everything runs in the Jazzy container on the stock 20.04 kernel.

## What exists and works (verified 2026-10-05)

The 16 packages that were under `src/` on 2026-10-05 (31 now) built in the amd64 dev image (`docker/Dockerfile.dev-amd64`,
`scripts/dev_build.sh build|test|shell`). OpenVINS is not under `src/`: it is the
`third_party/open_vins` git submodule (COLCON_IGNOREd), built only by `scripts/build_openvins_mower.sh`. Pytest, run against stubbed ROS on the host:

| Package | Tests |
|---|---|
| `mower_mcu_driver` | 29 |
| `um960_gps_driver` | 47 |
| `mower_localization` | 80 |
| `base_ble` | 14 |
| `wit_imu_driver` | 8 (new protocol tests) |
| `stereo_vio_bridge` | 10 |
| `det_ros` | 4 |

Fixes landed the same day as the audit:

- `bumper_controller`: compile errors fixed (PID member names, missing `finishToIdle` declaration, generic-lambda service callback, default-arg constructor). It now publishes unstamped `Twist` on its own lane `/cmd_vel_bumper` instead of `/cmd_vel`.
- `mower_coverage`: Fields2Cover 2.1 API fixes (`get_Area`, non-const `SGObjective&`).
- `mower_interfaces`: `MappingControl.srv` and `SetLoRa.srv` registered in `CMakeLists.txt`.
- `gps_gate` and `um960_gps_driver` use the real ROS 2 `NavSatStatus` values. RTK fixed and float map to `GBAS_FIX`, DGPS to `SBAS_FIX`; the gate separates fixed from float by covariance. Launch default `min_fix_status` is 0.
- `wit_imu_driver` rewritten for the real 11-byte WIT protocol (`wit_imu_driver/wit_protocol.py`, unit-tested) and publishes `/imu/data` directly. The `/imu` remap in `launch/nav2.launch.py` is gone.
- `mower_mcu_driver` gained `/cutter_control`, `/charging`, `/clear_estop`, `/cutter_off`, publishes `/mower_base/status` (MowerBaseDevStatus) and `/estop` (Bool), has a host-side lift/stop interlock plus an e-stop latch (`/estop_request`), sends cutter-off on shutdown, moved the placeholder IMU to `/mcu/imu`, and subscribes unstamped `Twist` on `/cmd_vel` (plus `/cmd_vel_stamped`). Default `linear_max` is 0.5 m/s.
- `base_ble` publishes `Twist` on `/cmd_vel_teleop`, calls `/cutter_control` and `/clear_estop` (Empty).
- `cmd_vel_slew` and `slip_detector` use unstamped `Twist`.
- `mowgli_interfaces` vendored verbatim at `src/mowgli_interfaces` (GPL-3.0). The MowgliNext GUI and reference packages (`mowgli_map`, `mowgli_behavior`, `mowgli_coverage`, `mowgli_nav2_plugins`, `mowgli_monitoring`, `mowgli_bringup`) are vendored under `third_party/mowglinext/` with `LICENSE` and `UPSTREAM_COMMIT`.
- `.gitignore` for `build/`, `install/`, `log/`, pycache.

## Architecture and contract

Jazzy convention throughout: unstamped `geometry_msgs/Twist` on every command topic
(`enable_stamped_cmd_vel` stays false; `twist_mux` 4.5 and Nav2 1.3 support stamped twist but
the whole chain uses unstamped).

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
`ekf_node` publishes `/odometry/filtered` and owns `odom -> base_link`; `ekf_map` (dual mode,
default since 2026-10-09) publishes `/odometry/filtered_map` and owns `map -> odom`.

Frames: `map` (`ekf_map`, GPS-anchored; static identity only with `localization_mode:=single`) -> `odom` (EKF) -> `base_link` (rear axle)
-> `base_footprint`, `imu_link`, `gps_link`, `blade_link`, `cutter_link`.

GUI adapter (`mower_gui_bridge`, in progress): presents our topics under the names the MowgliNext
GUI hard-codes: `/hardware_bridge/{status,emergency,power}` and services
`/hardware_bridge/{mower_control,emergency_stop,reboot_board}`, `/gps/{fix,status}`,
`/wheel_odom`, `/odometry/filtered_map` (frame `map`), and a stub mission layer
`/behavior_tree_node/{high_level_control,high_level_status,start_in_area,clear_coverage_resume,coverage_resume_available}`
covering IDLE, RECORDING (blade off), MANUAL_MOWING (blade on) and STOP with MowgliNext
`state_name` strings. Blade-on is refused unless the high-level state is 2 or 4.

Ports: GUI `4006`, `foxglove_bridge` `8765` (Jazzy ships 3.6.0; the Go GUI speaks
`foxglove.sdk.v1`, verified live 2026-10-09 — 3.6.0 negotiates that subprotocol and
advertises `clientPublish`/`parameters`/`services`), teleop relay `8766`.

## Completed later the same day (2026-10-05, parallel agents)

All of the following build in the amd64 dev image and were exercised in-container (no hardware):

- `src/mower_gui_bridge` (`gui_bridge`): the adapter above, 30 tests. Blade-on is refused outside HL states 2/4 or during an emergency; blade-off is never refused; MCU telemetry older than 1 s counts as an emergency. See its README.
- `src/mower_teleop`: `cmd_vel_ws_relay` (ws :8766, Twist out, clamps 0.5 m/s / 1.0 rad/s, 0.25 s lease) + `twist_mux.yaml` (lanes above, `/estop` lock) + `teleop.launch.py`. Verified: GUI-style JSON frame -> `/cmd_vel_teleop` -> mux -> slew -> `/cmd_vel`, and the `/estop` lock zeroes it.
- `src/mower_bringup` + `launch/mower.launch.py`: single entry point (drivers, robot_state_publisher, static `map -> odom`, control nodes, teleop, gui_bridge, foxglove_bridge 3.6.0, localization, optional navigation). `docker/docker-compose.yml` now runs it; `Dockerfile.jazzy` carries every dependency (websockets comes from Noble's apt 12.x). URDF gained `base_footprint` and `blade_link`. `ekf.yaml` int/float covariance fix (rcl refused to load it).
- `src/mower_navigation`: Jazzy `nav2_params.yaml` + `navigation.launch.py`; all lifecycle nodes activate; `FollowPath`/`FollowCoveragePath` RPP controllers, `general_goal_checker`/`coverage_goal_checker`, NavFn, behaviors, velocity_smoother -> `/cmd_vel_nav`, keepout filter slot (`use_keepout`).
- Datum pinning: `datum_lat`/`datum_lon`/`datum_yaw` launch args on `mower.launch.py` -> `navsat_transform_node` (`wait_for_datum: true` when set). Must match `config/gui/mowgli_robot.yaml`.
- GUI image: `gui/Dockerfile` + `gui/build.sh` (Go cross-compile + vite on the x86 host, final arm64 stage copies files only; 159 MB), Tron UI trim at runtime from the robot profile (`ROBOT_PROFILE` / `mower_model`, gates in `web/src/constants/profileGates.ts`: Firmware/GNSS/Drive-tuning/Updates/Remote-access/rosbag/LiDAR hidden, Perception shown), `ONBOARDING_COMPLETED` env, `docker/docker-compose.gui.yml`, `docs/gui.md`. Smoke-tested on amd64 and handshake confirmed against foxglove_bridge 3.5.0.
- `src/mower_proto`: generated with protoc 3.12.4 (imports on Noble's 3.21.12 runtime); `generate_proto.sh` accepts 3.12.x/3.21.x.
- `src/mower_coverage`: 3 real planner bugs fixed (swath ends overshot the boundary by op_width/2 with headlands off; wrong `atan2` for the reported angle; longest-edge angle computed on the widened cell). 15/15 gtests.
- Test suites now pass both on the host (stubs) and under real rclpy in the container: `colcon test` = 188 tests, 0 failures, 21 skipped (stub-only tests skip under rclpy). A real bug surfaced: `mcu_node._on_imu` assigned 36-element covariances to 9-element Imu fields (fixed).
- `third_party/COLCON_IGNORE`: colcon must not crawl the vendored MowgliNext packages (they target a newer Nav2 and broke the workspace build).
- `scripts/deploy_to_mower.sh`: rsync to `/userdata/ros2_stack` on the device (root fs is 100 % full, /userdata has 18 GB), image build and low-parallelism colcon build on the mower, up/down/logs/shell. Not yet run: the mower dropped off the LAN during the session (vendor ROS stack was NOT running on it; Docker 26 and an older `mower:jazzy` image are present).

## Roadmap

**Phase A: joystick plus GUI (this week).** The items above. Exit criterion: the stock MowgliNext
GUI on port 4006 shows pose, battery, GPS and e-stop state, and drives the mower with the joystick
in RECORDING and MANUAL_MOWING with the blade interlocked by the MCU driver.

**Phase B: zones and coverage.**
- Port `third_party/mowglinext/mowgli_map` `map_server_node` (grid_map from apt). It owns zone CRUD, `areas.dat` (plain key/value: `datum_lat/lon`, `area_<i>_polygon` in map-frame metres, obstacles), the keepout mask, mow progress and dock pose. Needs `base_footprint` and `blade_link` TF.
- `/plan_coverage` action adapter (`mowgli_interfaces/action/PlanCoverage`) over our Fields2Cover 2.1 `mower_coverage` service, splitting the path into `drivable_subpaths` at transits.
- Mission BT: port `mowgli_behavior` `main_tree.xml` minus LiDAR, fusion_graph and `DockRobot` (Jazzy's `nav2_msgs` now has `DockRobot` and `CollisionMonitorState`, so the upstream tree is a viable option), or write our own BT reusing `coverage_persistence.cpp` and the `FollowStrip` blade rules (blade off before any transit over 0.6 m and at every sub-path boundary).
- Nav2 keepout filter fed by `/keepout_mask`.

**Phase C: docking and scheduling.**
- Docking: the vendor docks in reverse on an ArUco marker seen by the rear camera (`mower_charge`: search, dock, final straight reverse at 0.05 m/s, retry; contact confirmed by `is_docking_done` debounced over 3 samples, then `/charging true`). Either reimplement that or backport `opennav_docking`. Undock is a `BackUp` of 0.8 m to the vendor `undock_point` (0.8, 0).
- Scheduler, notifications, statistics, HomeKit and the MQTT bridge in `third_party/mowglinext/mowgli_monitoring` work unchanged once the mission layer emits the real `state_name` set.
  Status 2026-10-09: notifications, scheduler, statistics, HomeKit and MQTT all live in the GUI backend (`gui/pkg/providers`) and read `/behavior_tree_node/high_level_status`, which `mower_mission` publishes with the real names. `mowgli_monitoring` (C++ `mqtt_bridge_node`, `diagnostics_node`) is not needed: the GUI's embedded MQTT broker and `gui_bridge` diagnostics cover it, so it stays COLCON_IGNOREd. Fixed in the GUI: `LOW_BATTERY_DOCKING` raises the battery notification, the HomeKit switch follows `state == AUTONOMOUS` (the GUI looked for `DOCKING`, which our mission never emits). Theft, lift and incident alerts come from `src/mower_alerts` (`/mower_alerts/events`, see its README). The GUI pushes them as kinds `theft`, `lift` and `gpsLost`, plus `emergency`, `blocked` and `battery`, and forwards them to MQTT `<prefix>/alerts`.

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

Verified live on the mower with the stack running from Docker (`mower:jazzy`, `/userdata/ros2_stack`):

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
- Power button long press (2026-10-09): the vendor kernel module `init_test.ko` times it (KEY_L at 3.0 s, KEY_R + `/usr/bin/sys_close` on release). `base_keys` `power_long_action:=sequence` stops the mower and asks the host helper (`mower-poweroff.path/.service`, `scripts/install_power_button.sh`) to power off. The helper is NOT installed on the mower yet; it installs with `DRY_RUN=1`. See `docs/buttons.md` "Power off".

Cutter height verified 2026-10-07: CutterControl height position is an absolute deck height in mm
(30-90, vendor-clamped); 50/30/90 moved the deck. height_motor telemetry is a placeholder.

Still to verify: cutter command speed scale, rain/bumper/stop inputs, RTK fix with the LoRa base or NTRIP,
dock contact (`is_docking_done`) and charging, side cameras through the rkisp pipeline, rear camera stream.


## GUI and operations, later on 2026-10-06

- MowgliNext GUI adapted for Tron in an upstreamable way (see `docs/gui_tasklist.md`, 43 tasks, all but the
  sim-only verification done): robot profiles (`ROBOT_PROFILE=AirseekersTron` or `mower_model`), feature
  gating (no LiDAR/STM32/docker UI on Tron), Perception page (camera MJPEG via the GUI's `/api/cameras` proxy
  to web_video_server :8080, detections, obstacle badge), GNSS corrections cards (LoRa/NTRIP source, flow,
  age) + LoRa pairing, settings -> ROS parameter bindings (live via foxglove parameters, persisted in the
  device-owned `config/gui/mowgli_robot.yaml` and loaded by `launch/mower.launch.py`), `/rosout` logs,
  profile-aware onboarding, vision-dock card, datum set-from-GPS (`set_datum` writes `/userdata/ros2/datum.env`
  + the GUI yaml and calls robot_localization `/datum`), router/layout fixes. Build: `ARCH=arm64 ./gui/build.sh`,
  ship with `docker save mower-gui:arm64 | ssh ... docker load`, recreate `mower_gui`.
- `config/gui/mowgli_robot.yaml` is DEVICE-OWNED: `deploy_to_mower.sh sync` excludes it (seed:
  `config/gui/mowgli_robot.yaml.seed`).
- Fast DDS leaks shared-memory segments on every restart; a full host `/dev/shm` (bind-mounted) broke every
  node and pushed the load to 50. Compose now wipes `/dev/shm/fastrtps_*` before launch.
- Idle CPU of the Python nodes cut from ~2.6 cores to ~0.5 (`sub_pump.py`: one wait-set thread, sampled
  inputs, pre-serialized publishing); second pass on cameras/IMU/LEDs and mower_control in progress.
- MCU driver: vendor speed policy (reconstructed from the decompile, `docs/wheel_control_semantics.md`): one raw
  SpeedData per /cmd_vel, clamp 0.3 m/s / 0.3 rad/s, stop = 3 zero frames 100 ms apart then SILENCE (the vendor
  host never streamed while idle and ran no wheel PID; our earlier zero stream and brake caused creep/crawl).
  Verified 60 s at rest: nothing leaves the host, measured speed at noise.
- NTRIP: nothing configured yet (vendor file empty; the vendor source tree only holds the vendor's expired
  China Mobile CORS test accounts, not usable). Enter the provider in the GUI's NTRIP section (applied live +
  at boot) or `/userdata/mower/ntrip.yaml`.

## VIO (2026-10-06, disabled by default)

OpenVINS stereo VIO -> `vio_odom_bridge` -> `/odometry/vio` -> `vio_gate` (passes only when RTK
is not FIXED and VIO is healthy) -> EKF `odom2` (twist vx/vy, low weight) via the
`config/ekf_vio.yaml` overlay. Off unless `mower.launch.py vio:=true`; the live EKF is unchanged.
`ov_msckf` is not in the mower image yet (`scripts/build_openvins_mower.sh`, stack stopped).
Blockers before it is useful: stereo pairs arrive at only 1.7 Hz from `stereo_cam`, the JY61P
gyro is clamped to 0 at rest, and no daylight/drive bag yet. Details: `docs/vio.md`.

## Tilt guard (2026-10-09, not yet run on the mower)

`mower_control/tilt_monitor` (`tilt_logic.py`) turns the IMU roll/pitch into ok/caution/limit/critical
on `/tilt/status`. The mission slows on caution, backs out and records a `steep` terrain incident on
limit, and treats critical as an emergency; the monitor also stops on `/cmd_vel_emergency` and calls
`/cutter_off` itself. The GUI shows it as `tron: Tilt`. Details: `docs/terrain_aware_planning.md` section 7.

## Localization: dual EKF, RTK quality, drift hold (2026-10-09, dev-image tested only)

- `localization_mode:=dual` (default, `mower.launch.py` -> `nav2.launch.py`): `ekf_node` (odom:
  wheel vx/yaw rate + heading_aligner yaw, NO GPS, continuous) + `ekf_map` (map, 10 Hz: same +
  `/odometry/gps`) from `config/ekf_dual.yaml`; navsat anchors on `/odometry/filtered_map`, so
  `/odometry/gps` is map-frame. `ekf_map` owns `map -> odom`; the static identity and the
  gui_bridge relay run only with `localization_mode:=single` (old config, fallback) or
  `localization:=false`. Datum args unchanged. VIO (`vio:=true`) feeds both filters
  (`ekf_dual_vio.yaml`: odom1 in ekf_node, odom2 in ekf_map; no index gaps).
- Earth-referenced yaw: the UM960 is single-antenna, so `/heading` is only GPHPR (never sent) or
  VTG course. `/heading_valid` is gated off (`publish_rtk_heading: false`, 2026-10-13, B8); the
  aligner's `rtk` source only blends a fresh, fixed GPHPR while driving straight forward (or at
  rest with `rtk_dual_antenna`), corrected by `rtk_baseline_yaw_deg` and the UTM convergence. The yaw source is the existing `heading_aligner` (COG windows on RTK, dock seed,
  persisted file; stereo-gyro integration between fixes), fused absolute by both filters.
- `gps_gate quality_covariance` (`rtk_quality.py`): class from `/fix_status`; fixed keeps the
  receiver sigma (3 cm floor), float x4 with 0.4 m floor, DGPS/single dropped.
- `localization_monitor` -> `/localization/status` (latched JSON: ok|degraded|lost, rtk class,
  since_fixed_s, dist_since_fixed_m, est_drift_m = 3 cm + 3 %/m wheel (1.5 %/m with VIO) since
  the last confirmed FIXED, capped at 0.5 m while float is fused). Budget 0.3 m -> degraded,
  1.0 m -> lost. `mower_mission` `_loc_hold_tick`: follow/transit stop with the blade off while
  not ok, resume after 3 s ok, stop in place after 600 s (`loc_*` in mission.yaml).
- Live checks needed: both EKFs' CPU on the RK3588, map -> odom behaviour on a real RTK
  fixed/float drive, the drift rates (measure with a float/dropout drive), Nav2 in dual mode.

## Rolling bag ring (2026-10-09, not yet run on the mower)

`mower_control/bag_recorder` (launch arg `bag_recorder`, default on) runs an always-on
`ros2 bag record`:
- ~70 non-image topics, 60 s zstd sqlite3 splits in `/userdata/ros2/bags`;
- pruned to 3 GB, keeping a 3 GB free floor;
- mission incidents, emergencies and supervisor dumps are pinned (hard links) to
  `/userdata/ros2/incidents`;
- pull them with `scripts/deploy_to_mower.sh bags`.

Details are in `docs/crash_recovery.md`, section "Rolling bag ring". Still to measure live:
- data rate and CPU of the recorder;
- that `/tf_static` lands in split 0.

## Rear obstacle sensing, dock-aware (2026-10-09, not yet run on the mower)

The rear camera had been left out on purpose. A person standing behind the dock made the
robot refuse to dock. It is back, with these rules:

- **Detector.** The rear camera goes through det_ros/det_ros_cpp (`extra_topics`). It is
  gated by `/vision/rear_watch`, so the NPU runs it only while the robot reverses, while it
  docks (ALIGNING through FINAL_DOCKING), and for 5 s after either.
- **Range.** There is no rear stereo. det_range ranges rear boxes on the ground plane, from
  the bbox bottom-centre, the camera_info intrinsics and the URDF pose (`mono_*`).
  - Accuracy is about 7 % per degree of pitch error at 1 m and about 14 % at 2 m.
  - The URDF pitch of the rear camera is unmeasured.
  - Rear points are kept out of the Nav2 costmap cloud.
- **Decision** (obstacle_guard, `rear_*`):
  - The rear camera counts only while reversing, while docking in reverse, or for 15 s after
    a rear hit while the robot is not driving forward.
  - Only living classes count.
  - The detection must lie in the reverse corridor: robot half width plus 0.2 m, and 1.2 m
    back from the rear edge.
  - While docking with a fresh marker, the corridor runs to the dock and ends 0.25 m before
    it. Anyone at, beside or behind the charger is ignored.
- **Effects.**
  - `/obstacle_policy` with `camera: rear`. Distance and bearing are measured from
    base_link, so the mission's path filter projects them onto the reverse leg, and the
    existing dynamic stop, wait and resume applies to reverse coverage legs.
  - `/vision/rear_blocked`. mower_docking holds on it in SEARCHING, DOCKING and
    FINAL_DOCKING: it stands still with its timers paused and resumes 1.5 s after the
    corridor is clear. After 60 s it fails with `DOCK_BLOCKED`.
  - Undocking drives forward, so the rear camera plays no part in it.
  - Per-area `obstacle_detection: none` disables all of it.
- **Verify live:**
  - The rear camera pitch and height: put a person at 1.0 m and 2.0 m behind the robot and
    compare the range in `/ai/det/detections_ranged` with the real distance.
  - Measure `dock_marker_height_m` on the charger.
  - Check that the model detects people in the rear view.
  - With someone standing behind the dock, docking must complete. With someone in the
    corridor, docking must hold and then resume.
  - Check the NPU and CPU load while the rear camera is gated open.

## Route-graph transits (2026-10-09, sim-tested only)

Mission transits (next sub-path start, end-of-area retries, blocked / dynamic / localization
re-sends, return to the dock approach) no longer leave the drawn paths to Nav2's grid
planners: map_server_node `~/plan_route` (`mower_map/route_graph.py`) follows drawn paths along
their centreline, crosses areas straight or round their inset perimeter (0.35 m clearance,
obstacle polygons avoided) and joins them only at path ends / junctions; the mission drives
the route with FollowPath (MPPI) and falls back to `/navigate_to_pose` on no route or a route
abort. `route_transits: false` restores the old behaviour. Details: `docs/route_graph.md`.
Live checks: MPPI tracking on narrow paths, the dock approach handover, CPU of a graph build
on large recorded areas.

## Vision false stop fixes (2026-10-13, unit-tested only)

At 09:45 the mission stopped for a person several metres away. The robot was pivoting toward
a detour target. Stereo ranged the person at 0.97 m and then 0.91 m (box 63 % / 74 % of the
480 px front frame, not clipped). The mission then logged the same person as off-corridor
and resumed after 2 s.

- **det_range class-size check** (`size_check_*`). For person (1.6 m), dog (0.4 m) and
  cat (0.25 m), a stereo range nearer than 0.5 x the box-height range is replaced by the
  box-height range, with sigma 0.3 x range. Boxes that touch the top or bottom of the frame
  (within 3 px), or that are >= 85 % of its height, keep the stereo range. Every close adult
  is clipped by the 0.24 m high camera. Known gap: a fully visible person shorter than about
  0.8 m within 1 m (a sitting toddler) is ranged too far.
- **obstacle_guard `ranged_persist_frames: 2`.** A ranged dynamic box must be in range on 2
  consecutive frames of its camera. This adds about 0.2-0.4 s at ~5 Hz detection.
  `bbox_h_frac` for the front camera was checked: it uses 640x480 from
  `/vio/right/camera_info`.
- **Mission in-path test** (`obstacle_sweep_horizon_s`, `obstacle_twist_max_age_s`,
  `robot_front_m`, `robot_rear_m`).
  - MOWING sub-paths and route transits keep the corridor along the known path.
  - `/navigate_to_pose` transits (detours, gaps, retries) and dock phases sweep the
    footprint along the predicted motion. This uses the applied `/cmd_vel` (else
    `/cmd_vel_nav`) held constant for 2 s.
  - A stationary TRANSIT with no fresh twist keeps the line to the target.
  - Anything within the circumscribed footprint plus margin (0.84 m) always counts.
  - A dynamic stop keeps judging with the twist and path kind it started with.
    `/mission/obstacle_decision` gains `judged_by` and `motion`.
- **Verify live:**
  - Look for the det_range "implausible" INFO lines.
  - Walk up to the robot from the front: it must still stop at about 1 m.
  - Stand about 1 m to the side of a pivoting robot: the result must be advisory.

## Swath-end turns without pivot loops (2026-10-13, unit-tested only)

At 11:25 (54 sub-paths, snake, cross, 2 headland rings, `turn_type auto`) the owner saw what
looked like 360 deg spins before each new line. No bag; from the log: each K-turn leg started
with an RPP rotate-to-heading (v 0, w +-0.3). On turf the body turned about 2 deg/s, so the
stuck guard latched (8.4 deg in 4 s against a 10 deg floor). The escape reversed 0.3 m and
resent the goal, which started with another pivot, often the other way. Aborts ("collision
ahead" 298x, "Failed to make progress" 20x) were retried the same way: 7 stuck escapes and 42
retries in 3 h. Arcs with the inner wheel at 0.06 m/s or more do move the robot.

- **Coverage RPP** (`nav2_params.yaml`):
  - `FollowCoveragePath` `rotate_to_heading_min_angle` 0.6 -> 1.2 rad. A forward leg starting
    at a reverse leg's cusp sees its carrot about 0.41 rad round (lookahead 0.4 on the r 0.5
    turn) plus the yaw the previous leg missed. It is now driven as an arc. Rotate-to-heading
    stays on for carrots behind the robot (otherwise RPP drives forward, away from them).
  - `min_approach_linear_velocity` and `regulated_linear_scaling_min_speed` -> 0.14 m/s, on
    `FollowCoveragePath` and `FollowCoveragePathReverse` (were 0.05/0.08 and 0.05/0.05). Every
    0.4 m leg is inside the approach distance, so RPP drove it at 0.05-0.1 m/s. The turn shaper
    then widened the arc (r 0.84 m instead of 0.5, so the leg ended short of its heading) or,
    at |w| 0.3, snapped it to a pure pivot. At 0.132 m/s or more, any |w| up to 0.3 keeps the
    inner wheel at 0.06 m/s or more.
  - `coverage_leg_goal_checker` `yaw_goal_tolerance` 0.35 -> 0.5. Within 8 cm of a forward
    leg's end, RPP pivots to the goal yaw until this is met.
  - Unchanged: `max_angular_accel` 6.4 (used only by rotate-to-heading).
  - FTC (`FollowCoveragePathFTC`) and MPPI coverage are not the default. FTC PRE_ROTATEs at
    the start of every goal more than `max_goal_angle_error` (15 deg) off, so it is unsuitable
    for K-turn legs as configured.
- **Stuck guard** (`slip_detector`): new `stuck_pivot_min_heading_deg` (5). When at least half
  of the window's commands are pure pivots, a 5 deg heading change in 4 s counts as progress.
  The 11:25 case (8.4 deg) no longer latches; 1 deg or 4 deg in 4 s still does. The xy and
  path rules are unchanged; driving keeps the 10 deg floor.
- **Mission**: new `turn_leg_skip_m` 0.20 and `turn_leg_skip_yaw` 0.5, the same values as the
  leg checker's overshoot / yaw tolerances. This applies to a retry or stuck-escape resume of
  a turn leg: a reverse leg, or the forward piece ending at its cusp. If the robot is within
  0.2 m of the leg's end and within 0.5 rad of its heading, the leg is counted as done and the
  next leg is sent, instead of a 1-3 pose goal whose first move is a pivot.
- **"collision ahead" at the headland** (not changed, needs a bag). The local costmap holds
  only stereo, det_range and bumper marks; there is no area mask. With 2 rings, in a rectangle,
  a swath ends 0.27 m inside the drivable line, i.e. base_link is 0.41 m from the recorded
  boundary. The nose (+0.52 m) is therefore about 0.1 m past the boundary. Anything 8 cm or
  taller at the lawn edge (hedge, bed edging, long uncut grass) is lethal right in front of
  it. For a pure pivot, RPP's check projects the footprint (corner radius 0.61 m) through
  1.0 s of rotation, uncut by the carrot distance. Fewer pivots should mean fewer of these
  aborts.
- **Planner** (`mower_coverage_node`): new `min_drivable_turn_radius_m` (0.44). A turn radius
  above 0 but below it (the GUI accepts 0-2 m) is raised to it, with a note in the plan.
- **What `auto` chose here and why.** `auto` tries a forward Dubins loop, then a Reeds-Shepp
  reverse turn, then a pivot. Each must stay inside the drivable region (the field pulled in by
  `border_inset`, 0.14 m).
  - With 2 rings, a rectangle leaves 0.27 m past the swath ends.
  - A reverse turn between snake rows (0.36 m apart) needs 0.31 m (r 0.44) or 0.36 m (r 0.5):
    forward 0.40, reverse 0.70, forward 0.45 m legs.
  - A forward loop (omega) needs 1.06 m (r 0.44) or 1.23 m (r 0.5).
  - So this area gets K-turns where an oblique edge leaves extra room, and pivots elsewhere.
    The test field `kArea1` with 2 rings and r 0.5 gives 4 reverse / 12 pivot turns at angle 0
    and 2 / 11 at 90 deg, with 0 loops. At r 0.44 it gives 6 reverse / 10 pivot turns at angle 0.
- **Does a loop fit?** No. A loop between snake rows needs a 7-ring (r 0.44) or 8-ring (r 0.5)
  headland, i.e. 1.26-1.44 m. Racetrack order (rows 2r apart, forward U-turns) needs room of
  about r, i.e. 3-4 rings. On `kArea1` with 3-4 rings that gives 2-5 loops, the rest K-turns.
  `auto` already prefers a loop whenever one fits, so it is left alone. Options for the owner:
  - 3 rings, which makes every turn a K-turn, no pivots (`kArea1`, r 0.44);
  - `min_turn_radius_m` nearer 0.44 instead of 0.5, for more K-turns;
  - racetrack order with 4 rings, for some loops.
- **Verify live:** watch the turns at the swath ends.
  - No `rotate-to-heading` at a K-turn leg start.
  - Fewer `STUCK` latches.
  - The "turn leg end ... leg done" INFO lines on retries.
  - A rosbag of a few turns (`/cmd_vel_nav`, `/cmd_vel`, `~/shape_status`, odom) to confirm
    the 0.14 m/s arcs track on turf.
## Heading after a restart off the dock (2026-10-13, unit-tested only)

10:40: the stack restarted on the lawn and START was refused ("heading not aligned"). The
offset saved 5 s earlier was rejected as "moved 1.24 m": the um960 node had sent `RTKRESET`
and the one-shot continuity check averaged five float positions of the re-convergence.
The owner's 0.05 m/s manual drive was below the COG `min_speed` (0.06).

- **Continuity check** (`heading_logic._try_verify`): the yaw test is decided first and
  rejects at once (IMU restarted). The position test uses only 5 consecutive RTK-fixed
  positions (0.5 m). The check stays pending until decided or `verify_timeout` (30 -> 180 s).
  Status `file_check`: pending | yaw_ok | yaw_failed | unverifiable | verified.
- **um960** `rtk_reset_on_start: false`: the first handshake after node start sends only
  `$GNRTK,ON*69`. Later ON -> OFF edges and source switches still reset.
- **COG** `min_speed` 0.06 -> 0.04 m/s. The message now says "drive straight 1 m at 0.1 m/s
  or more in manual, or start from the dock". Diagnostics show the COG gate reason.
- **Calibration drive** (`heading_calibration_*`, on by default). START off the dock with
  `file` + `yaw_ok` waits for RTK fixed (+3 s), then drives 1.0 m at 0.15 m/s with the
  blade off. It uses Nav2 DriveOnHeading, or BackUp when only the line behind is clear.
  The line (+ footprint, width, +-10 deg) must lie in one mowing area, outside its keepouts.
  It stops once aligned, on an in-line obstacle, STOP or emergency. It then plans as a
  normal START. Failure: idle in place, never dock or Nav2 on the unverified heading.
- **Verify live:**
  - Restart the stack off the dock: the log should show "verified ... RTK fixed".
  - Push the robot >0.5 m with the stack down: START drives 1 m, then mows.
  - With the IMU power-cycled: START is refused (yaw_failed).
  - Behavior_server output bypasses collision_monitor (its own costmap check only).
## Live blade height during a mow (2026-10-13, unit-tested only)

- **Symptom:** the GUI blade height control showed "—" with its -/+ buttons disabled during
  a mow (and in manual mow), so the deck height could not be changed.
- **Root cause:** `/cutter/height_mm` (mcu_node, latched; GUI key `cutterHeight`) was never
  added to `FOXGLOVE_GUI_TOPICS` in `launch/mower.launch.py`, so foxglove_bridge did not
  advertise it and the control never got a value. The POST route, `/cutter/set_height` and
  the driver were fine. `/behavior_tree_node/mow_plan|mow_progress` and
  `/map_server_node/terrain_grid|terrain_summary` were missing the same way and are now
  streamed. `test_foxglove_whitelist` checks every topic in the GUI's `topicMap`.
- **Driver:** `/cutter/set_height` re-sends the current blade state (enable + speed) with the
  new height in one CutterControl frame. The blade is not stopped and the deck moves at once.
  Whether the MCU ramps the blade on that repeated ON frame is unverified on hardware.
- **Override rule:** a live height holds for the rest of the mow and is never saved to the
  area. The mission does not re-send a height mid-area. Later areas of the same mow and a
  resumed mow use the live height. Before, an interrupt (stop, rain, charge, stuck, e-stop,
  slope) rebuilt the Mission and the resumed area start re-sent the saved area height. The
  live height is now kept in the resume cursor (`live_height <mm>` row). A fresh start
  (resume cleared, or the previous mow completed) uses the area settings again. "Save to
  area" in the GUI is the only way to change the area setting.
- **Deploy:** stack sync + container restart. No GUI image rebuild is needed.
- **Verify live:** during a mow, the control shows the area height; -/+ moves the deck
  without stopping the blade. Stop, then START (resume): the deck stays at the live height.

## Wheel smoothness (2026-10-14, unit-tested only)

Velocity chain review (nav2 MPPI -> velocity_smoother -> twist_mux -> cmd_vel_slew ->
mcu_node); details in `docs/wheel_control_semantics.md` ("Both-wheel turn shaper",
"Velocity chain limits").

- **Turn-shaper chatter:** a pivot/arc regime latch (`turn_shaper_hysteresis_mps` 0.02,
  `turn_shaper_latch_ticks` 6) stops the target flipping every tick when the controller
  sits on the snap boundary (|w| 0.3, v 0.070 -> pivot, 0.075 -> arc).
- **Stops:** `max_linear_decel` 0.8 (accel stays 0.5), `max_angular_decel` = accel; a zero
  on the winning emergency/bumper lane (`hard_stop_lanes`) zeroes the output at once.
- **Arcs from rest:** `slew_both_wheels_always` (true) ramps v and w together in
  turn-shaper phases (curvature kept, inner wheel never crawling backwards).
- **Nav2:** MPPI `ax_max` 0.5 / `ax_min` -0.8 / `az_max` 1.0; velocity_smoother widened to
  accel 1.0 / decel -1.5 so cmd_vel_slew is the single limiter.
- **Fix 8/9 (after run 1, branch `fix/pure-pivot-regime`, unit-tested only):** slow pure
  rotations (v = 0, |w| < 0.25) snap to the 0.25 rad/s pivot and stay in the latched pivot
  regime (run 1: 0.25/0.10 hunting at turn entries, 4564 sub-deadband rotation ticks);
  |w| < 0.03 becomes a stop (`turn_shaper_pure_pivot`, `turn_shaper_pivot_deadband_rps`).
  The nav-driven dock transit (RETURNING_HOME and the other dock phases) is shaped; the
  shaper now only shapes the twist_mux navigation lane, never docking/bumper/emergency/
  teleop (`turn_shaper_dock_transit`). Both revert live by parameter; see
  `docs/wheel_smoothness_field_tests.md`.
- **Unchanged:** docking stays exempt from the shaper, pivot assist on, zero-speed
  keepalive off, motion gate and watchdog hard zero.
- **Verify live:** MPPI transit with tight turns: `~/shape_status` should not alternate
  pivot/arc each tick; a GUI emergency stop halts on the next tick; nav stops take ~0.4 s
  from 0.3 m/s instead of 0.6 s.

## Heading: stereo gap integration + stale-COG calibration drive (2026-10-14, unit-tested only)

The 2026-10-14 mow (`docs/analysis/2026-10-14_heading_drift_mow.md`): the stereo-integrated
virtual yaw drifted +52 deg against the JY61P while moving (the WIT agreed with the GPS
course), the mow ended ~60 deg off, and the second START planned on a 1200 s old, quality
poor `cog` heading and drove its transit 65 deg off.

- **Stereo gaps** (`heading_logic.YawSource`): a stereo stamp gap of 50 ms .. `stereo_max_gap`
  (1.0 s) is bridged instead of skipped: WIT rate over the same interval x dt (when the WIT
  covered at least half of it and moved), else the mean of the bracketing stereo rates.
  One coverage rule: while stereo is active the WIT never adds (it used to fill receive
  gaps, which a late burst then counted again); when the WIT takes over it starts from the
  last stereo-covered sample. Gaps over `stereo_max_gap` add nothing. Rest hold unchanged.
  Status: `stereo_gaps`, `stereo_gap_s`, `stereo_gap_max_s`, `stereo_gaps_wit`,
  `stereo_dropped`. WARN when `virtual_minus_wit_deg` moves > `wit_divergence_warn_deg` (5)
  within 60 s with stereo active (at most once a minute).
- **Stale COG at START** (`mission_fsm._heading_cog_stale`): off the dock, a `cog` heading
  older than `heading_max_cog_age_s` (300 s; the older of the status `last_cog_age_s` and
  `age_s`) or with quality in `heading_reject_quality` (['poor']) runs the 2026-10-13
  calibration drive first ("heading cog is 1200 s old / quality poor: calibration drive
  first"), then plans once the aligner reports a fresh, acceptable fix. No clear line or
  drive disabled: refused, idle in place. START from the dock and dock / rtk /
  file_verified headings are unchanged.
- **Verify live:** over a mow `virtual_minus_wit_deg` should stay within a few degrees and
  `stereo_gaps` / `stereo_gap_max_s` show how often bursts were late; no divergence WARN.
  A START on the lawn > 5 min after the last straight drive should drive 1 m first.
  If the WIT keeps agreeing with the GPS course and the stereo does not, `imu_source: wit`
  is the fallback.

## Heading self-recovery after a reboot off the dock (2026-11-01, unit-tested only)

Owner note 2026-11-01: after a power cycle or reboot on the lawn the GUI asked for a manual
"drive straight 1 m" before START. The aligner only needs 0.6 m of RTK-fixed straight travel
and no prior to align from COG; the mission refused to drive because the saved heading's
continuity check had failed (`yaw_failed`: the JY61P yaw restarts at power-up, so the saved
IMU-to-map offset is meaningless) and the 2026-10-13 calibration drive needs a usable prior
to check its line against the map.

- **Pose prior** (`heading_logic._try_verify`, `verify_pose_prior` on). The yaw test still
  marks the record `yaw_failed` at the first IMU sample, but the position test keeps running:
  `verify_samples` consecutive RTK-fixed positions within `verify_max_move` of the saved
  position re-seed the offset from the saved MAP heading (saved IMU yaw + saved offset,
  minus the IMU yaw now). Status `file_check: pose_prior`, source stays `file` (quality
  low, not aligned). Moved farther, or float only until `verify_timeout`: stays
  `yaw_failed`, offset untouched. The first COG window replaces a `file` offset outright
  (`_update_offset`), so a prior that is wrong because the robot was turned while off
  costs nothing beyond the drive.
- **Calibration drive by the pose prior** (`fsm/phases/hcal.py`): `pose_prior` counts like
  `yaw_ok`; the 1.0 m line (+ footprint, +-10 deg cone) ahead or behind must be inside one
  area as before.
- **Blind drive** (`heading_calibration_blind` on, `heading_calibration_blind_distance_m`
  0.8): no usable prior at all (`yaw_failed` and moved, `unverifiable`, no file, source
  none) no longer refuses at preflight. After RTK fixed + the settle wait, the disc of
  0.8 m + the body's front corner radius (hypot(0.60, 0.235) = 0.64 m, so 1.44 m) around the
  robot must lie inside one mowing area and clear of its keepouts: then it creeps 0.8 m
  forward at 0.15 m/s, blade off, obstacle policy and bumper live, and stops as soon as the
  aligner reports aligned (the usual `_hcal_tick`). Disc not clear: refused in place with
  "no usable heading prior (...) and no clear 1.4 m disc", exactly as before otherwise. No
  COG after the drive: idle, never Nav2 on the unverified heading. START from the dock is
  unchanged (undock seeds the heading).
- **Verify live:** reboot the mower on the lawn where it stood: the log shows "persisted
  offset re-seeded from the saved map heading", START drives 1 m, "heading calibrated
  (cog)", then plans. Carry it 1 m sideways before the boot: "IMU restarted and the robot
  moved", START creeps 0.8 m "forward blind", aligns, plans. Put it within 1.4 m of an
  edge: refused without moving. The mower_sim's `localization: sim` mode latches an
  aligned heading, so only the full-mow regression gate covers this in sim.

## Mission: turn-leg cap, leg loop guard, route hairpins (fix 10/11, 2026-10-14, unit-tested only)

Run 2 of 2026-10-14 (`docs/wheel_smoothness_field_tests.md`, "Run 2"): 7 minutes of MPPI
looping on a 0.8 m turn leg whose last pose was reached with the heading ~10 deg off, and a
route hairpin at the dock-path / area-path junction where FollowPath stalled and the
`/navigate_to_pose` fallback drove 0.56 m off the drawn path. Branch
`fix/mission-turn-legs-hairpin`; tests in `src/mower_mission/test/test_mission_legs.py`.

- **Fix 10, turn-leg completion cap** (`mission_fsm._leg_watch`): once a turn leg's end is
  reached in xy (tracked pose index at its last pose, or within `turn_leg_skip_m`), the leg is
  done, next leg from here, after `turn_leg_done_after_s` (20 s) or
  `turn_leg_done_after_rot_rad` (6.28) of accumulated |yaw change| of the map pose, whatever
  the yaw (logged once with the reason). Turn leg = reverse leg, forward piece ending at its
  cusp, or a sub-path <= `turn_leg_max_m`. A met end yaw is left to the leg goal checker and
  `turn_leg_skip_*` as before.
- **Fix 10, leg loop guard:** a follow_path whose tracked pose index has not advanced for
  `leg_loop_abort_s` (90 s) is cancelled and handled as a follow_path abort (follow retries,
  then the sub-path is skipped). The timer belongs to the goal; dynamic / localization /
  blocked holds and stuck / tilt escapes cancel the goal, so they never count.
- **Fix 11, route cusps** (`geometry.route_cusps`, `mission_fsm._route_cusp_turn`): a cusp is
  where the planned route reverses by more than `route_cusp_angle_rad` (2.0; chord turn over
  0.3 m either side, so a filleted hairpin counts; sharp route-leg junctions too; a 90 deg
  corner does not). A route follow abort / timeout within `route_cusp_radius_m` (0.5) of a
  cusp: pivot in place with Nav2 `Spin` (behavior_server, nav lane, new `spin_action` client)
  to the route direction after the cusp, then FollowPath on the rest of the route from the
  cusp ("route cusp at (x, y): about-turn 1/2 then resume route"). After
  `route_cusp_retries` (2) about-turns at that cusp the abort is a transit abort
  (`transit_retries`, re-plans the route), never the costmap planner.
- **Fix 11, bounded fallback** (`_fallback_corridor_tick`): `/navigate_to_pose` remains only
  for a start off the route graph and a route abort away from any cusp (and the existing
  obstacle fallback after `route_blocked_retries`). Every tick the robot's own distance from
  the drawn network is checked (cross-track to the route it replaces; 0 inside a mowing area;
  the FSM never sees Nav2's plan, so the live pose is the measure); above
  `transit_fallback_max_offpath_m` (0.35 = map_server `corridor_half_width_m`) the fallback is
  cancelled as a transit abort ("transit fallback left the corridor by 0.56 m: abort"). An
  off-graph start that begins farther off is bounded once it gets within the corridor, or
  when it moves another 0.35 m away.
- **Not changed:** nav2_params tolerances; the docking-server route (`_dock_route_result`
  still hands a failed dock route to the docking server's own Nav2 leg).
- **Verify live:** no leg runs more than ~20 s past its end; no "leg loop guard" abort on a
  normal mow; at the dock-path hairpin the robot pivots (Spin) and follows the area path,
  `/navigate_to_pose` only after "no route". Spin collision-checks the footprint against the
  local costmap: check that it does not refuse the pivot inside the 0.35 m path corridor.
  Revert flags: `docs/wheel_smoothness_field_tests.md` rows 10 and 11.

## Open risks

- Grass segmentation (2026-10-09, not yet run on the mower, `nongrass:=false` by default): seg_ros now runs on the front stereo right colour eye, `nongrass_projector` projects its mask onto the ground and map_server_node keeps a confirmed non-grass memory that feeds a SOFT global-costmap layer (`nongrass_layer`). The model was trained on the OA cameras: class quality on the front eye, NPU sharing with det_ros on core 1 and the shadow false-positive rate need a live check (`docs/grass_segmentation.md`).

- Obstacle reroute (2026-10-09, not yet run on the mower): sensor obstacles are now in the global costmap, global inflation is 0.6 m, and the BTs wait and replan instead of clearing. The dock/perimeter reachability and planner CPU need a live re-check (`docs/analysis/2026-10-09_obstacle_reroute.md`).

- GPLv3: the GUI and `mowgli_interfaces` are GPL-3.0 with a commercial option. Fine for personal use; a product decision later.
- The JY61P yaw is gyro-integrated, not earth-referenced. `navsat_transform` needs `yaw_offset` or a GPS heading source; `/heading` from the UM960 is only useful if it is dual-antenna (unverified).
- The MCU clear-estop frame is unknown, so a firmware e-stop latch may need a power cycle.
- MowgliNext mission packages target a newer Nav2 (`DockRobot`, `CollisionMonitorState`): Jazzy has those msgs, but `mowgli_behavior` still assumes the Kilted BT/API set, so it is not built as-is.
- `map -> odom` comes from `ekf_map` since 2026-10-09 (not yet run on the mower). The datum
  (`datum_lat`/`datum_lon`/`datum_yaw`) is still to be set per site (dock position, same as
  `config/gui/mowgli_robot.yaml`).
