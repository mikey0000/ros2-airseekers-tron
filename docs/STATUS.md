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
  VTG course. The yaw source is the existing `heading_aligner` (COG windows on RTK, dock seed,
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
