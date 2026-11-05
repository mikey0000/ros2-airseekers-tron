# Airseekers Tron ROS 2 port: status

Read this first. The "Current state" section below was last updated 2026-11-04 (master 41e61e3).
Everything after it is a dated log, newest work at the end. Each log section's title records how
far that feature was verified when it was written, and the title is not changed later: "not yet
run on the mower" means "not confirmed at the time". The current verification state is in the open
work audit (`docs/analysis/2026-11-04_open_work_audit.md`). The point-in-time audits that fed the
first version are in `docs/audit_2026-10-05/`.

## Current state (2026-11-04)

**Deployed:** master 41e61e3, synced 2026-11-03 18:25 NZDT and running for the 2026-11-04 mow.
**Field:** full-area mows complete, with edge laps (soft layout, option (c)), no-go zones,
mid-mow re-plan, obstacle detours on labelled detections, docking by day, and resume after STOP.

**Where the work is tracked:**

| What | Where |
|---|---|
| Open work, tidy-up, stock comparison, next two weeks | `analysis/2026-11-04_open_work_audit.md` |
| Owner decisions A1-A6 and queue B1-B13 (with status) | `analysis/2026-10-21_independent_review.md` |
| Stock firmware behaviour and lessons | `analysis/2026-10-21_stock_firmware_findings.md` |
| Sim gate, criteria, baselines | `simulation.md` |
| Field mow evidence | `analysis/` (one doc per field problem), `wheel_smoothness_field_tests.md` (lapsed after 2026-10-23, B4) |

**Field problems open on 2026-11-04** (fixes on `ws/fix1104`, sim gate running;
`analysis/2026-11-04_branch_pivot_resume.md` once merged):
1. An obstacle seen only by stereo depth on an edge lap (a branch) was skipped, not detoured.
2. An RTK-float hold during a coverage pivot resumed without finishing the pivot, so the mower
   drove out past the row end.
3. A START after STUCK_NEEDS_HELP re-planned with an unreviewed pending zone, discarded the
   resume cursor and restarted the mow.

**Other known issues (details and evidence in the audit):**
- Straight-line weave: p90 about 1.9-2.0 cm, unchanged by GNSS smoothing (A/B failed 2026-11-01,
  `analysis/2026-10-29_localisation_wander.md` section 7). Further work is controller-side.
- Lap 0 runs about 1 cm over its soft limit at some convex bends; sim criterion (i) fails on master.
- Night and dusk docking: the SEARCH filter heading is now seeded from RTK (branch `ws/dusk1104`,
  unit + sim, not field tested). Frequent marker flips still trip the near-field check, whose
  heading is the raw marker median.
- The hard-boundary check does nothing when `outside_m` is stale (code review stage 2, item 10).
- Side-slope pivots on the west edge creep 0.15-0.30 m; slope-aware planning (A5) is open.
- Coverage map (A3) slice 1 on `ws/cov1105` (2026-11-05, unit + sim): a mowed raster per area,
  `mowed_area_percent` next to the plan %, resume and re-plans mapped by the raster, missed-patch
  report (off by default). Driving the missed patches and the GUI display are slice 2.
- `mission.yaml` has 359 keys against 238 when the A6 guard freeze was agreed.
- Deploy has no rollback or health check (B8).

**Superseded below:** the 2026-10-05 "What exists", "Roadmap" and "Open risks" sections describe
the first bring-up. The roadmap phases are replaced by the queue in the independent review, and
the open risks by the audit.

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
  not ok, resume after 3 s ok (`loc_*` in mission.yaml). Graded since 2026-11-03, see below.
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

## Whole plan rejected over a 0.027 m pose (2026-11-03, ws/edge-reject)

Field, build cbdddbb: START on area 1 ("Area 4", narrow strip, soft edge layout so lap 0
runs on the line) rejected the area's only sub-path. One pose was 0.027 m closer to the edge
than allowed. The mission then went MOWING_INCOMPLETE "0.0 % mowed" and returned to the dock.
The offline replay (mower map, coverage_bridge plus `condition_plan` in the dev image) gives
the same rejection at d3230fd, 1126c60, 9fc5214 and cbdddbb. No merge since d3230fd caused
it: this geometry and these settings fail on every build. The cause is in the conditioner.
Three short drawn edges at the east tip merge into one -139 deg corner, and the intersection
of its runs lies 0.5 m past the tip, inside a neighbouring path channel. The K-turn
reversed onto that point because only the point's own clearance was checked. Its leg
crossed the gap between the two polygons. Fixes:

- The runs' intersection is used (pivot or K-turn apex) only when both legs to it stay
  inside; otherwise the turn uses the drawn vertex (`path_conditioning.py` `_leg_clearance`).
- The validator projects a pose up to `path_conditioning_edge_reject_tol_m` (0.05) too close
  to the edge inward onto its allowed clearance and logs a warning. Only a deeper pose
  rejects the sub-path.
- If no part of a plan is drivable (every sub-path rejected, or no area planned) and
  nothing has been driven, the mission fails with "plan rejected: ...". It stays idle when
  docked, otherwise it docks and stays idle there with the reason. It no longer reports
  MOWING_INCOMPLETE. Mowing still undocks before RTK and planning, so in the field the robot
  drives back onto the dock.

Regression fixture: `src/mower_mission/test/data/area_edge_reject_2026-11-03.json`.

## Row links and tangent transits (2026-11-03, ws/routing-snake, gated: not adopted)

New area settings, all off by default (docs/analysis/2026-11-03_turns.md appendix B):

- `row_link_mode` snake: a row change that would become a transit is driven as a
  pivot -> row-end straight -> pivot connector inside the sub-path, within the 0.285 / 0.26
  body limits.
- `tangent_approach`: transits end on a 1.0 m straight along the first leg, so the
  leg-start pivot is skipped.
- `pivot_rate_rad_s`: recorded only.

All three are unit-tested.

real_run5 gate (-d -s 2 -x 3): snake cuts area 0 from 11 to 7 sub-paths, but the pivot
share only goes from 28 % to 27 %. The connector pivots add 40 s of row-end pivoting. a
and i regress, side 0.116 against 0.10. tangent_approach never fired. Keep both settings
off.

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

## Map editor vertex tools (2026-11-03, branch ws/vertex-merge)

Owner request: no way to merge points of a boundary polygon. The editor now merges a point dragged
onto its neighbour, deletes a point (✕ chip, Delete key, right-click / long-press menu), inserts an
edge midpoint, and has "Merge nearby points" (0.15 m) and "Simplify" (Douglas–Peucker, 0.05 m) with
a ghost preview, a minimum-vertex guard and a self-intersection check, also on boundary lines. All
changes go through the editor history (undo) and reach map_server only on Save; a trash that would
leave fewer than 3 points is now refused instead of deleting the whole polygon. Vitest covers the
geometry and the dialog reducer; checked on the sim GUI (real_run5) at 1280x800 and 390x844. Not
yet used on the mower. See docs/gui.md "Vertex editing".

## Stale-but-stationary COG heading at START (2026-11-02, branch ws/heading-stale)

Field 2026-11-02 15:19 (d3230fd): area 5 mowed with a good cog heading, STUCK_NEEDS_HELP at
a boundary, ~6 min idle, then both STARTs logged "heading cog is 350 s old: calibration drive
first" and refused because no 1.0 m line was clear. The robot had not moved.

- **Aligner** (`heading_logic.HeadingEstimator`): every absolute fix (cog / dock / rtk /
  file_verified, `_mark_fix`) anchors the RTK-fixed position; status gains
  `moved_since_fix_m` (farthest RTK-fixed position from the anchor since, None = no fixed
  position at the fix) and `imu_continuous` (false after an IMU sample gap > `imu_gap_max`
  1 s or an IMU reset). The idle activity mode only batches the IMU / stereo polling (every
  sample still integrated) and GPS keeps flowing, so both stay tracked while idle.
- **Mission** (`_heading_cog_held`): a cog fix older than `heading_max_cog_age_s` is kept,
  with its age-decayed quality ignored, while `moved_since_fix_m < heading_stale_move_m`
  (new, 0.5) and `imu_continuous`; log "heading cog fix is N s old but the robot has not
  moved (X m) and the IMU is continuous: keeping it". Either field missing: the age check
  as before.
- **No room for the 1.0 m drive** (`hcal`): try the 0.8 m straight ahead/behind by the
  prior, then the disc. Nothing clear: START at low quality when the heading was aligned
  earlier in this process lifetime and the IMU stayed continuous (cog `imu_continuous`, or
  file `yaw_ok`); else refuse with the case (never aligned since start-up / IMU not
  continuous / no usable prior / drive disabled) appended to the HEADING_NOT_ALIGNED text.
- **Aligner quality** (field 03:03-03:10 UTC: good -> fair -> poor at an unchanged offset,
  1615 s parked): while `stationary_since_fix` (moved < `hold_move` 0.5 m RTK fixed, IMU
  continuous) the age-driven sigma growth is capped at the fair band; the mission's quality
  rejection uses the same hold.
- **Manual recovery**: HEADING_NOT_ALIGNED / GUI hint now say "in manual, drive straight
  1 m or more without turning at 0.1 m/s or more"; `min_speed` 0.04 -> 0.03 (field drive
  sat at 0.04 on the gate). The 0.08 m lateral check and the IMU-span check still guard a
  slow window (test: a weaving 0.035 m/s track is rejected); min_distance unchanged.
- **Idle yaw-source flapping** (wit<->stereo every 1-2 s while idle): a switch never steps
  the virtual yaw nor resets the COG window / continuity (on_imu sees one continuous yaw),
  so it was log noise, but stereo gaps > `stereo_max_gap` were dropped. While idle
  `YawSource.set_idle` now uses `stereo_stale_exit_idle` (5 s) for both the stereo exit and
  the bridged gap length; active restores 0.5 s / 1 s.
- Observation, not addressed: 02:31 UTC WARN "virtual yaw diverging from the WIT yaw
  (-5.2 deg in 57 s, stereo gaps 37, 12.87 s bridged)": stereo IMU gaps under load.
- Note: the anchor is the last COG fix, not where the robot stopped, so a stop several
  metres after the last fix still drives (or falls back to the low-quality start).

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

2026-11-01: `return_home_on_incomplete` now defaults to true (Params, `mission.yaml`) and
is a live GUI toggle under Settings > Battery ("Return to dock when mowing is incomplete";
bindings in `launch/robot_settings.py` and the GUI `param_bindings`). Reason: the mower twice
sat idle on the lawn for over an hour after MOWING_INCOMPLETE; it now docks and keeps the
resume cursor. Sim gate still required before deploy.

## Bag ring: corrupt zstd splits, false recorder crash dumps (2026-11-01, unit-tested only)

During a mow at load avg ~12 on 8 cores, 10 of 68 one-minute splits were unreadable: half had a
zstd frame with no end block ("premature end"), half had end blocks but no frame header
("unsupported format"). rosbag2 file compression ran with one thread per core, so a split still
compressing when the next one rolled was compressed in parallel and the frames got mixed.
`record_cmd` now passes `--compression-threads 1 --compression-queue-size 2`. Separately, every
low/full ring switch stops the `ros2 bag record` child on purpose, which the supervisor reported
as `/rosbag2_recorder` vanishing with a node_down blackbox; it is now in the supervisor's
default ignore list (bag_recorder already restarts and logs real child exits). A SIGKILL after
the 15 s stop timeout is now logged as a possibly partial last split.

## Open risks

Historical (2026-10-09). The current list is in `analysis/2026-11-04_open_work_audit.md`.
Still open from this list: grass segmentation and obstacle reroute are unconfirmed on the mower,
and GPLv3. The JY61P yaw item is handled by `heading_aligner` (COG), and the datum is set per site.

- Grass segmentation (2026-10-09, not yet run on the mower, `nongrass:=false` by default): seg_ros now runs on the front stereo right colour eye, `nongrass_projector` projects its mask onto the ground and map_server_node keeps a confirmed non-grass memory that feeds a SOFT global-costmap layer (`nongrass_layer`). The model was trained on the OA cameras: class quality on the front eye, NPU sharing with det_ros on core 1 and the shadow false-positive rate need a live check (`docs/grass_segmentation.md`).

- Obstacle reroute (2026-10-09, not yet run on the mower): sensor obstacles are now in the global costmap, global inflation is 0.6 m, and the BTs wait and replan instead of clearing. The dock/perimeter reachability and planner CPU need a live re-check (`docs/analysis/2026-10-09_obstacle_reroute.md`).

- GPLv3: the GUI and `mowgli_interfaces` are GPL-3.0 with a commercial option. Fine for personal use; a product decision later.
- The JY61P yaw is gyro-integrated, not earth-referenced. `navsat_transform` needs `yaw_offset` or a GPS heading source; `/heading` from the UM960 is only useful if it is dual-antenna (unverified).
- The MCU clear-estop frame is unknown, so a firmware e-stop latch may need a power cycle.
- MowgliNext mission packages target a newer Nav2 (`DockRobot`, `CollisionMonitorState`): Jazzy has those msgs, but `mowgli_behavior` still assumes the Kilted BT/API set, so it is not built as-is.
- `map -> odom` comes from `ekf_map` since 2026-10-09 (not yet run on the mower). The datum
  (`datum_lat`/`datum_lon`/`datum_yaw`) is still to be set per site (dock position, same as
  `config/gui/mowgli_robot.yaml`).

## Wi-Fi link monitor (2026-11-01, unit-tested only, not yet run on the mower)

`mower_control/wifi_monitor` (own process, launch arg `wifi_monitor`, `wifi_iface` default
`wlan0`) reads `/proc/net/wireless`, `iw dev <iface> link`, `/sys/class/net/<iface>/operstate`
every second and `iw dev <iface> station dump` every 10 s, all without root, and publishes
`mower_interfaces/WifiStatus` on `/wifi/status` (signal, BSSID, freq, bitrates, tx
retries/failed, roam count). Connect / roam / disconnect / reconnect go to `/wifi/event`
(JSON String, old and new BSSID / freq / signal) and the log. A missing `iw` or interface gives
`connected=false`. Both topics are in the full and low-power bag rings, so roams line up with
incidents in a pin. The GUI header shows a Wi-Fi chip (dBm, four colour levels, tooltip with
SSID / AP / channel / bitrate / roams) once `/wifi/status` arrives. Parsing:
`wifi_logic.py`, tests `test/test_wifi_monitor.py`. Observability only, no behaviour change. Needs `iw`, added to
`docker/Dockerfile.jazzy` on 2026-11-01: until the mower image is rebuilt, the container has
only `/proc/net/wireless` and wifi_monitor reports `connected=false` (chip "No link": no signal, BSSID, freq,
bitrate or roams). Parsers checked against the mower's real bcmdhd output the same day.

## Bounded obstacle skips, one retry pass, plan-time mark exclusions (2026-11-01, unit + sim gate)

Field run 2026-11-01 (area 3): a transit route corner whose pre-pivot sweep stayed blocked by
three stone marks dropped a whole edge lap (38 m) after the transit retries, and a resume
retried only the first of four stretches skipped behind one tree trunk before reporting
MOWING_INCOMPLETE. Three behaviours, each behind a mission param (`config/mission.yaml`):

- `sweep_blocked_skip` (true), `sweep_blocked_max_skips` (3), length `edge_blocked_skip_m`
  (2.0, the existing edge-skip length): when a sub-path's transit is exhausted by a blocked
  sweep ("route corner pivot sweep blocked"), only the piece from the rejoin pose up to
  `edge_blocked_skip_m` along the path (at least one pose, at most the next pivot vertex) is
  recorded as skipped (the same `area_skip` stretch a detour records, re-tried in the
  end-of-area pass) and the transit goes to the pose after it. Consecutive skips on one
  sub-path are counted until the robot starts following it again; above the cap the old
  `subpath_failed` drop applies. Log: `sweep blocked: skip X m ...`, incident
  `sweep_blocked_skip`.
- `retry_each_skipped_stretch` (true): the end-of-area pass orders the skipped stretches
  nearest-first from the robot and drives every one of them once (the "next to an
  unbypassable obstacle" shortcut is not applied; the short-stretch rule stays). A retry
  stopped by an on-path obstacle decision (act / stop with on_path true), a recent collision
  ahead or a ranged static (never a person / animal) moves the stretch to the area cursor's
  new `blocked_static` list (resume file row `area_static <area> <from> <to>`). Only static
  stretches left: the area is complete and the mission ends MOWING_COMPLETE, with
  `excluded_count N, excluded_m X (static obstacles)` in the outcome text and the log.
  Anything else still blocked stays MOWING_INCOMPLETE (return_home_on_incomplete as before).
- `exclude_marked_obstacles` (true), `exclude_mark_merge_m` (0.5): at plan time, sub-path poses
  within (mark r + robot_width_m / 2 + 0.05 m) of a static obstacle mark from
  /obstacle_tracker/nogo_zones (the marks the pre-pivot sweep consults) become blocked_static
  stretches from the start; gaps shorter than 0.5 m between them are merged so an edge lap is
  not cut into slivers. Follow chunks stop at an excluded stretch and continue (transit) after
  it. Plan log: `N stretches / X m excluded by obstacle marks`. Needs fresh nogo zones at plan
  time; with none the plan is unchanged.

All three off restores the previous behaviour. The GUI progress `skipped` ranges include the
blocked_static stretches.

## Pivot-vertex chunk ends beside their start (2026-11-01, sim-tested)

Trunk gate (real_run5_trunk, seed 2): after the no-go re-plan the edge lap runs down a slit
to the trunk hole and back up 0.15 m beside itself, with pivot vertices at both ends of the
slit. `_follow` let a pivot vertex (or an excluded-piece start) override `chunk_end`'s
clearance rule, so the chunk from pose 373 ended on the vertex at 667, at the same spot; the
along-path corner checker matched it at once and follow_path "succeeded" 3/3 times with 14 m
left (sub-path NOT mowed, MOWING_INCOMPLETE). Now such a chunk ends at the furthest pose
before the vertex that clears the chunk start (`geometry.clear_end_before`) and the next
chunk reaches the vertex from close by. Log: `chunk end N passes near its own start`.

## Dock return from off the route graph joins it inside the area first (2026-11-02, unit-tested)

Field 2026-11-02 03:47 UTC: RETURN HOME from the edge-lap start of area 5 (-5.85, 1.95), 0.2 m
off the area edge, got "start is off the route graph" from plan_route (no clearance anchor that
close to the edge), handed over to the docking server, which may only navigate within 0.35 m of
the approach pose, so after 2 re-sends it was NAV_TO_DOCK_FAILED without moving. Now a dock
route start off the graph inside a mowing area (the last area enumeration) first drives a join
transit (`/navigate_to_pose`, purpose `dock_join`, state RETURNING_HOME): to where the
mission's last inbound route entered that area (first route pose `dock_join_inset_m` 1.0 m in
from the edge), else to points `dock_join_insets_m` [1, 2, 3] m in from the nearest edge,
nearest first. Every candidate must be inside the area with a straight line to it inside the
area and within `dock_join_max_m` (40 m). The join transit is cancelled once it gets more than
`transit_fallback_max_offpath_m` outside the area; obstacle aborts get the blocked wait, other
failures get `transit_retries`, then the next candidate. On arrival the dock route is planned
again; still off the graph moves on to the next candidate. With no candidates left it fails with
"start ... is off the route graph and no join point is reachable inside area <name> (<idx>)
within N m (K tried)". Outside every area (or `dock_join_max_m: 0`) the old hand-over still
applies. Covers every RETURNING_HOME (user, battery, rain, incomplete, schedule end): they all
go through `_dock`. Log: `dock: off the route graph, joining it at (x, y) via in-area transit`.
Drawn path corridors are not join regions; the FSM only knows the area outlines. A pose within `dock_join_edge_tol_m` (0.30 m) outside an outline counts as in the nearest such area (the real incident pose was 0.013 m outside area 5; replay test (c) now passes); the join line may start outside and must enter inward.

## Boundary recovery on a side slope (2026-11-02, unit-tested; sim gate pending with the integration run)

Field 2026-11-02 (area edge, 8-12 deg roll): BOUNDARY OUT on rear_left 0.384 m while
front_left was 0.43 m out. The retrace's lining-up pivot (wz +0.3) took the worst corner to
0.69 m, the reverse retrace and three inward moves spun the wheels while the RTK pose crept
downhill, and STUCK_NEEDS_HELP named the stale trip origin. Changes (mission Params +
mission.yaml):

* `boundary_recover_footprint_plan` (true): in body mode every recovery motion is chosen by
  simulating the footprint (`body_boundary.plan_recovery`) for reverse / forward straight,
  reverse / forward arcs (R 0.5, inner wheel >= 0.06 m/s), pivots both ways, 0.30 m /
  0.35 rad each. Admissible: the worst-corner outside distance (raw, keepouts count) never
  exceeds the current one by more than `boundary_recover_max_worsen_m` (0.05; pivots: never)
  and ends lower. Lowest end wins (reverse gets 0.02 credit). A motion whose live worst
  corner grows past the start + 0.05 is cut and re-planned; at most
  `boundary_recover_max_moves` (4). No admissible motion -> STUCK_NEEDS_HELP. On the
  reconstructed field geometry the outward pivot is rejected (it would reach +0.18 m) and the
  pick is the forward arc turning the nose in (0.42 -> 0.33 m); the best reverse candidate
  there is reverse straight (0.39 m), not a reverse arc: a short reverse arc swings the
  nose out one way or the tail out the other. false = the old straight move / retrace /
  inward moves.
* `boundary_recover_slip_check` (true): wheel odometry (`/odom`, new mission_node input
  `wheel_odom_topic`) vs map/RTK displacement per recovery motion while RTK is fixed; wheel >
  3.0 x real after >= 0.3 m wheel -> "wheel slip on slope (roll X deg): N m wheel odom, M m
  real", the next-best candidate once, then STUCK_NEEDS_HELP. The slip_detector stays
  suppressed during the recovery (it would latch /stuck and start its own escape); this
  check replaces it there. The old stages (flag off / base_link mode) get the same check,
  straight to needs help. Planned motions measure progress by map displacement.
* STUCK_NEEDS_HELP from the boundary recovery reports the current pose (all params).
* `edge_slope_roll_deg` (8.0): |roll| above it (fresh /tilt/status), soft edge_mode: front
  corners get the side limit 0.35 instead of the 0.55 convex-bend allowance; logged once per
  episode. 0 = off.
* Logged outside distances are the worst-corner metric everywhere. The "retrace ended
  0.523 m out" vs recomputed 0.655 m: `_hb_inside` returned the outside distance of the
  point with the largest excess over its own limit (rear_left, limit 0.35), not of the point
  furthest out (front_left, limit 0.55). The trip line now adds `worst corner <name> <m>`.

The sim gate's `-d` disturbance (wheel bias 0.04, per-wheel slip std 0.05) is mild and the
sim publishes no roll on /tilt/status, so a 3x wheel-vs-RTK slip and the slope rule never
occur there: the slip and slope paths are unit-tested only.

2026-11-03 (ws/sim-fidelity): the sim now has slope regions with tilt, `/tilt/status`, encoder
`/odom` and calibrated slip (docs/simulation.md "Terrain"), RTK float / dropout / multipath
injection and process kills in the gate. `real_run5_slope -d -s 2 -x 3`: no STUCK, but base_link
0.28 m out (485 samples) and corner deviations to 0.72 m on the west-edge slope with no
BOUNDARY OUT; the downhill creep stays under the body limits, so the recovery and the slip
check were not reached. Plain real_run5 unchanged (PASS, a 0.084, i 0.096 / 0.087 / 0.091, 0.364).

2026-11-03 (ws/boundary-resume, field 03:46 UTC, area 5, roll 8-11 deg): a rear_left trip
during an edge-lap no-go detour was recovered (one forward arc, worst corner 0.06 m out), then
the mission went MOWING -> TRANSIT, the transit re-tripped 5 s later 0.04 m away and the spot
rule ended in STUCK_NEEDS_HELP ("left the boundary 2 times within 1.0 m"). Cause of the
transit: the trip came in step `nogo_detour`, not `follow`, so the pause kept the stale chunk
start (2 m back, (-0.12, 3.19)) as the resume point, `m.nogo_detour` stayed set, and the
dispatch had to transit there from the recovery spot. Fixes: planned recovery moves go on until
the worst corner is `boundary_recover_margin_m` (0.10) under its limit (cap and no-worsening
kept; under the limit but no admissible move left = resume with a warning). The resume rejoins
the interrupted leg (follow or edge detour; the detour is re-planned from there) ahead of the
robot, offset the margin inward over edge_join_m, without a leg-start pivot when the leg is
within 60 deg of the yaw (a larger turn only when its footprint passes the margin check).
A resume that still needs a transit first drives a `boundary_recover_resume_in_m` (0.3 m)
straight, forward or reverse by the planner's (now signed) clearance scoring, and checks the
pivot toward the transit target, one more straight if it fails. A trip within
`boundary_recover_grace_s` (10 s) / `_grace_m` (1.0 m) of a resume is not a spot strike: one
more recovery with twice the margin, then counted in the mission total; a later excursion at
the same spot still stops. Regression: replay fixture `20261103_slope_resume_retrip` (test_d)
and field-geometry tests in test_body_boundary.py. Gates (seed 2, -x 3):
real_run5_slope -d: complete and docked, STUCK 0, 0 excursions / 0 recoveries (the recovery
and resume paths are again not reached in the sim), a 0.232 m / 353 samples, e 6 of 19
(0.701 m), j FAIL (the same slope tracking failures as the ws/sim-fidelity run). Plain
real_run5: PASS, a 0.076 / 0, i 0.076 / 0.039 / 0.073, transit 0.337.

## GUI UX batch (2026-11-02, branch ws/ux-batch, vitest + sim GUI check)

Fixes from the GUI UX audit (`docs/gui.md` "Owner-facing states, alerts and layout"). The
header Wi-Fi chip now appears (the backend multiplex allowlist was missing `wifiStatus`). The
terrain card can always be closed: it sits under the status line and closes on Escape or a tap
outside. On desktop the progress card no longer covers Dock, Edit map or the E-stop. The phone
toolbar wraps instead of hiding actions off-screen. Stuck, unfinished-mow, dock-failed and
heading-hint states show plain text with one primary action, and `/mower_alerts/events` reaches
the notification bell and Home's Health check. On Tron, Settings hides unused fields, and
Settings > Connection opens with a Bluetooth setup guide. The updater 503 and bag-recorder 500
console noise is gone, and the Schedule rules read the real settings. Checked in the
real_run5 sim GUI at 390x844 and 1280x800 (stuck, unfinished and dock-failed states faked on
`high_level_status`). Not done: the new-zone dialog (F12), path labels and taps (F14), the
Logs backlog (F24); all three done on 2026-11-04 (see "GUI small batch" below).

## Review fixes (2026-11-02, branch ws/review-fixes, unit-tested; sim gate pending)

Code review of 335ef53..3cfe973. Each fix has a regression test that fails on 3cfe973.

* **Boundary recovery failure stays put.** `_boundary_recovery_failed` used to end in
  `_finish_incomplete`, which docked once `return_home_on_incomplete` became true. It now passes
  `stay_reason`, so the robot stops outside the region whatever that setting says, and
  MOWING_INCOMPLETE logs "not returning home: ...". The test Harness no longer defaults
  `return_home_on_incomplete` to false. The six tests that assert a stop in place set it
  explicitly, and the recovery-failure test runs with the shipped true.
* **The stationary heading hold sees all motion.** `moved_since_fix` now grows on RTK float
  positions too, and on wheel-odometry path (`/odom`, `HeadingEstimator.on_odom`) while no
  GNSS position is fresh. When RTK becomes fixed in WAITING_FOR_RTK after a START off the
  dock, `_heading_plan_ok()` is checked again. If a held cog heading turned out stale, the
  calibration drive runs ("heading cog is N s old now RTK is fixed: calibration drive
  first").
* **One stationary threshold.** The aligner status has a new `held` field
  (`stationary_since_fix`, `hold_move` 0.5). The mission's `_heading_cog_held` reads it, and
  the `heading_stale_move_m` param is removed (Params, mission.yaml).
* **`plan_recovery` cost.** It now scans only the region and keepout edges within 2 m of the
  pose (`_LocalRegion`: inside/outside by crossing parity from a reference point, with a full
  scan when nothing is near). The result matches the full scan. On the review's 1000-vertex
  region it takes about 20 ms on amd64 (was 0.75 s). The DriveCmd(0, 0) that ends a recovery
  motion (done / timed out / grown, and the slip trip) now goes out through the node at
  once (`MissionFSM.effect_sink` / `_fx_now`), before the next plan runs.
* **`_static_block()` needs evidence on the stretch.** Outside the on-path decision, a
  remembered ranged static counts only when it is on the path or within robot_width/2 +
  obstacle_path_margin_m of the stretch. A collision counts only when the robot's nose is
  that close to the stretch. Anything else leaves the stretch `skipped`, not `blocked_static`.
* **Sweep-skip counts** are cleared with every new plan (each repeat / cross run, re-plan,
  no-go re-plan).
* **wifi_monitor.** An `iw` that fails or times out returns None, which gives link_state
  `unknown` and no event. `LinkFilter` keeps the last known sample for `stale_samples` (3).
  A disconnect is logged when operstate is down, or when iw is not connected AND
  /proc/net/wireless has no signal. If iw keeps saying "Not connected." while /proc still
  shows a signal, the disconnect comes after `stale_samples`.
* **bag_ring.** `--compression-queue-size` is 10. Overflow DROPS a split (rosbag2), so the
  old comment ("delays the next split") was wrong.

Known limitations from the review, not fixed:
- Review 8: the hold anchor (`_mark_fix`) can be the last RTK-fixed position from before a
  GNSS outage. (Its other half is fixed: `imu_continuous` / `held` in the status now turn
  false once no IMU sample has come for `imu_gap_max`.)
- Review 9: the slip check's "real" distance is the map EKF pose, which fuses wheel
  velocity, so wheel slip between GNSS updates is under-detected (check the ekf.yaml inputs).
- Review 10: `plan_recovery` / `worst_outside` count keepout incursion but `_hb_inside` /
  `evaluate` do not. A trip next to a keepout with a corner already in it can leave no
  admissible candidate, which ends in STUCK_NEEDS_HELP.

## Boundary lines (virtual walls) and the no-go zone sheet (2026-11-02, unit-tested; sim gate pending with the integration run)

Owner request: a line the mower must never cross (a drop-off, a flower-bed edge inside an
area), next to the existing area / no-go polygons. Branch `ws/boundary-lines`.

* **Data model.** `mower_map/boundary_lines.py`: `{id, name, points [[x, y], ...] (map
  metres), width_m}`; `width_m` 0..2.0 (default 0.0 = the body must not touch the line
  itself). Wire format `{"version": 1, "lines": [...]}`. Stored in `areas.dat` after the
  areas (`boundary_lines_version: 1`, `boundary_line_count`, `boundary_line_<k>_id / _name /
  _points / _width_m`), so the lines share the file's datum stamp and are never re-projected
  separately; a file without lines stays byte-identical to the upstream format. `clear_map`
  keeps them.
* **Services / topics** (`map_server_node`): `~/get_boundary_lines` / `~/set_boundary_lines`
  (the area-settings service types, JSON in `settings_json`, the whole set);
  `ReplaceAreas.srv` gained `boundary_lines_json` ('' = keep, the set is swapped in with the
  areas in the same rebuild + save); latched `~/boundary_lines` (JSON) for the mission;
  `~/boundary_line_hit` (JSON, every boundary check: the lines the footprint touches, with
  depth). GUI backend: `Map.boundary_lines` in the polled map, `ReplaceMapReq.boundary_lines`
  (absent = keep) sent in `replace_areas`, or `set_boundary_lines` after the fallback
  clear/add sequence.
* **Hard trip (true line).** Mission guard (`fsm/guards/boundary.py`, `walls.py`): in every
  hard-boundary phase the footprint box (0.60 / 0.10 / 0.235) at the current pose is tested
  against each line's centreline +- width_m / 2 (separating-axis depth, so a line between
  two body points counts). Deeper than `boundary_line_trip_m` (0.0) -> `BOUNDARY OUT: body
  crosses boundary line <id> "<name>" by N m`, blade off, retrace of the driven track until
  the body is `hard_boundary_body_clear_m` clear of that line, then the inward move away from
  the line; counted like any excursion. At START / resume: inward move first, not counted.
  `boundary_lines_enabled` (true) switches the trip and the coverage holes off.
* **Planners (thin keepout).** Each line buffered to `width_m + boundary_line_keepout_m`
  (0.57 = 2 x (body half width 0.235 + 0.05); same default in map_server and mission) is an
  extra obstacle of every area it overlaps
  (`areas.with_boundary_lines`, copies like the dock no-go): /keepout_mask, /nav_keepout_mask,
  the route graph (transits go round, or find no route through a full wall), and the
  coverage goals (`_nogo_area_holes`, preview too), so swaths are cut at the line.
  `get_mowing_area` still serves the stored areas (the GUI never sees the keepouts).
* **GUI.** Map editor tool "Boundary line" (desktop editor toolbar, phone edit cluster):
  click vertices, double-click / Enter / Finish (>= 2 points), Undo point, Cancel / Escape;
  red dashed line (#ff1744) with its name; own sheet (name, width 0..2 m, delete). Edits made
  from the view-mode sheet open the editor with the change pending (Save writes the map).
* **No-go click fix (owner bug).** A click on a no-go zone opened the mowing-area sheet: the
  view-mode click hit-tested mowing areas first. `pages/map/utils/mapHitTest.ts` now picks the
  most specific feature (boundary line within 10 px / 16 px on phones > no-go zone > path >
  mowing area, smallest polygon when nested). A no-go zone opens its own sheet (name,
  provenance, area m², delete, edit vertices, "Mow <area>"), also without area settings; an
  amber outline marks the feature whose sheet is open.
* **Sim gate.** Criterion b counts line trips (they are BOUNDARY OUT lines) and reports
  them separately (`line_trips`).

Body clearance is tested, not assumed (gtest `BoundaryLineBody`): soft and hard edge layouts,
a wall parallel to the rows 0.5 m inside an edge, rows ending at a wall, a full cut; the
footprint box along every lap, row, turn and driven connector, pivots swept, stays >= 0.05 m
off the true line. A 0.30 band failed it: the hard layout does not grow holes (laps 0.45 m
off) and its lap-corner pivot swept the nose corner (0.645 m) across the line. The soft
layout grows holes by 0.40 and passed at 0.30 too. The masks are robot-centre limits, so the
0.57 band is also what keeps a Nav2 fallback transit's body off the line. Route graph: a gap
between a wall and the area edge narrower than the body is never used (test). Not modelled:
tracking error / leg overshoot at row ends (the trip is the backstop). A line lying
outside every area is not in the masks (no area to carry it).

**Body / blade mode (2026-11-03, branch `ws/wall-mode`; unit-tested, sim gate pending).**
Each line now has a `mode`. `"body"` is the default and the behaviour above. `"blade"`
(flower beds, paths) protects only the cutting disc and lets the body overhang. The trip
tests the disc against the band: centre 0.23 m ahead of base_link
(`DECK_CENTRE_X_M`), radius from map_server's `blade_radius()` (0.11 for the 220 mm disc,
0.165 for the 330 mm one). That radius is latched as `blade_radius_m` on
`~/boundary_lines`, so the mission uses the same disc. The trip reason is `blade crosses
boundary line ...`. The planner band is `width_m + 2 x (blade radius + 0.05)` =
0.32 (220 mm) / 0.43 (330 mm), against 0.57 for body lines. The parameter
`boundary_line_blade_keepout_m` (-1 = derived from the disc) exists in map_server and in
the mission, and a `blade_disc_mm` change rebuilds the bands. Wire / areas.dat version 2
adds `mode` (`boundary_line_<k>_mode`). A v1 set, a missing mode, and an unknown mode in
areas.dat all read as body. gtest `BoundaryLineBlade` uses the same layouts as
`BoundaryLineBody`. For both discs the disc stays at least 0.27 m (hard) / 0.37 m (soft)
off the true line, with the worst point at a lap-corner pivot. Each figure is the same
for both discs, because the band grows with the radius. The gain is smaller than the band
suggests. The paths nearest a parallel wall move only 0.125 m closer (blade 0.56 soft /
0.61 hard from the line, against 0.685 / 0.735 for body lines), so the disc edge still
cuts no closer than 0.45 / 0.50 m. The rest of the gap is the edge layouts' own
clearance from holes (soft grows holes by 0.40; hard keeps its laps 0.45 off them). That
clearance is the lever for a tighter flower-bed edge.

## Sim gate loop-rate INVALID runs: two causes (2026-11-03, branch ws/sim-rate-race)

From 2026-11-01 evening every gate read a flat 7 Hz sim at `-x 3` (10 Hz at `-x 2`) and
10 Hz at `-x 1`, for old and new code alike. There were two separate causes. DDS leakage
(the gate graph on domain 175 holds only its own nodes) and thermal clock collapse (3.0-3.9
GHz average under a sustained 8-core load) were both ruled out.

* **Lifecycle race (x2 / x3, code).** mower_sim set controller_server's
  `controller_frequency` to 20 x rate once, as soon as the server was active. At the same
  moment activity_gated_lifecycle_manager pauses nav2 on the dock and resumes it on START.
  Nav2 Jazzy removes the server's dynamic-parameter callback in `on_deactivate` and does not
  re-read the parameter in `on_activate`, so a set that landed after the pause reported success
  and changed the parameter, but the loop kept running at 20 Hz wall (= 20 / rate Hz sim). This
  matched all 15 x2 / x3 runs since 2026-11-01, including a 335ef53 run at 17:28 that predates
  the "slow era". Fix: `mower_sim/ctrl_rate.py` re-sends the rate on every transition into
  active (`/controller_server/transition_event` plus a GetState poll), retries a refused set
  and logs the value read back. `evaluate.py` marks a run INVALID with "controller_frequency
  not applied (sim rate race)" when, with `-x` != 1, launch.log has a missed-rate warning
  naming another desired rate after START.
* **Power profile (x1, host).** Every 1x run from 2026-11-02 15:30 to 2026-11-03 02:55 ran
  under the balanced or quiet platform profile (asusd journal) and read 10 Hz. The first 1x
  run after the switch to performance read median 20 / p10 19. `sim_gate.sh` now refuses to
  start unless `powerprofilesctl get` is `performance` and prints the 1-minute load average.
  The stall looks tied to sim time rather than compute (controller_server 1-30 % CPU, about
  90 ms sim cycles at 1x and 3x). It is not fully explained.

## Chunk ends on a bend with a short tail (2026-11-03, branch ws/chunk-end-bend, unit + sim gate)

Sim (real_run5, `-d -s 2 -x 3`): the area-0 edge lap passes beside its own start twice. The
chunk from the pivot vertex at pose 370 cannot end near the lap start (clearance rule), so it
ended at pose 684, at the end of the west-edge bend (plan heading 90 -> 53 deg), and left a
0.52 m last chunk. FollowCoveragePathMPPI drops its path critics within 0.3-0.4 m of a goal,
so the robot reached 684 off the plan and past its heading, and the last chunk's turn swung
the tail out (criterion i side up to 0.19 at (-5.5, 2.0)). Now, when the clearance rule (or
`clear_end_before`) ends a chunk short of the next forced end (segment end, pivot vertex,
excluded piece) with less than `follow_chunk_min_tail_m` (1.5) left, the end moves back to
the furthest clear pose whose plan heading changes by less than
`follow_chunk_end_straight_deg` (10) over the `follow_chunk_end_straight_m` (0.4) before it
(`geometry.ease_chunk_end`). A pivot-vertex end is never moved. Log: `chunk end N moved back
to M: tail too short / bend`. Two gates: the rule fired at 684 -> 662 in both; side excess
on the second pass at the bend 0.051 / 0.059 (five pre-fix runs: 0.055-0.144). Overall i side
0.108 at (-0.60, 4.80) / 0.124 at (-5.73, 1.85) (the first lap's pass of the same bend, into
the pivot vertex at 367, which this rule does not touch), so i still FAILs (limit 0.10); a
0.065 / 0.091, h line 0.089 / 0.081, loop 20 Hz.

## Wi-Fi setup in the GUI (2026-11-03, branch ws/wifi-setup; Go + vitest, read-only check against a dev box)

Settings → Network (Tron profile) drives the host NetworkManager over D-Bus: link status, scan,
join / forget, band lock (Auto / 2.4 / 5 GHz) and power save. Backend
`pkg/providers/network_wifi.go` + `pkg/api/network_wifi.go`, routes `/api/network/wifi/*`; the
mutating ones need `X-Mower-Token` = the BLE token, which the BLE bridge now forwards
(`ble_streams.py`, `httpStream.ts`, `ble_tunnel_protocol.md`). `docker-compose.gui.yml` mounts
`/run/dbus/system_bus_socket`; the GUI runs as root, so NetworkManager needs no polkit rule. On the
Ubuntu dev box Docker's `docker-default` AppArmor profile blocked every D-Bus message from the
container (the page then says Wi-Fi setup is unavailable); whether the mower confines containers the
same way is unchecked (docs/gui.md lists the two commands and the options). Not yet run on the mower: connect / forget / band / power save are tested only against a fake
NetworkManager. Details: `docs/gui.md` "Wi-Fi setup".

## Reliability hazards H1-H5 (2026-11-03, branch ws/hazards, unit + launch tests, one sim gate)

Review 2026-11-02 section 2, one commit per hazard:

- **H1** `/hardware_bridge/emergency` comes from mcu_node (`mower_mcu_driver/safety_relay.py`,
  10 Hz from the I/O loop, at once on change; sim_node the same). gui_bridge only feeds
  `/gui/emergency` (`emergency_topic` default `''`).
- **H2** cmd_vel_slew, supervisor and gui_bridge run as their own processes (were in
  node_host_control / _housekeeping / _gui). Expected ~+150 MB PSS on the mower, not measured.
- **H3** stale `/localization/status` (> `loc_status_stale_s` 5 s) = degraded hold; stale
  `/supervisor/status` (> 10 s) = node down; critical nodes + map_server, both EKFs,
  controller_server, mower_docking, gui_bridge.
- **H4** `/mcu/driver_heartbeat` (2 Hz); supervisor `heartbeats` param marks a hung driver DOWN
  after 2 s. Bench procedure `docs/MCU_HEARTBEAT_BENCH_TEST.md` (not run).
- **H5** code defaults = shipped yaml (mission, docking, map_server; `mower_common.shipped_params`
  drift tests). Field values kept: dock_timeout_s 480, dock_action_timeout_s 540,
  manual_blade_requires_enable true, docking max_retries 8 (GUI schema too), marker_lost_hold_s 8,
  search_min_detections 3, map nav_margin_m 0.05, path_margin_m 0, nav_soft_band_m 0,
  boundary_check_rate_hz 10, boundary_debounce_samples 2.

Sim gate `-d -s 2 -x 3`: b-h PASS, a 0.104 m / 26 samples (FAIL by 4 mm; master 0.081),
i 0.091 / 0.044 / 0.115, transit 0.365 (master 0.101 / 0.098 / 0.082, 0.364); loop 20 Hz.
These are inside the recent same-code spread (a 0.06-0.13, side 0.09-0.19). Kill test:
`kill -9` of coverage_action_server mid-mow. The mission stayed in MOWING (no EMERGENCY),
launch respawned the server in under 8 s, and the mow completed and docked.

## Keyframe COG heading method + magnetometer removal (2026-11-03, branch ws/heading-estimator, unit + offline replay)

Stock heading maintenance (`fusion_localization_ros/src/common/heading_estimator.h`, `gnss_initor.h`):
- RTK keyframes every 0.05 m, sliding window of 10, chord between window ends 0.3-0.8 m.
- A segment counts only while the IMU yaw is steady: spread <= 0.17, sigma <= 0.08, end-to-end <= 0.08 rad.
- Updates every 0.2 s, weight 0.7, measurement noise 3.6e-5 rad^2, corrections > 0.8 rad rejected.
- A yaw-jump checker resets the window. The ESKF pose is fed back to detect heading offsets.
- No magnetometer anywhere in the stock heading path.

New: `heading_logic.KeyframeHeadingEstimator`, selected by `cog_method: keyframes` (default stays
`window`). It uses the vendor defaults (`kf_*` params, radians) and only RTK-fixed positions. Seeding,
source precedence, drift rate and quality are the same as for `window`. Replay tool:
`scripts/heading_replay.py <bagdir>`. The rings have no /imu/data, so IMU yaw = EKF yaw minus the
live offset (checked against the status imu_yaw, p95 0.25 deg). Truth = RTK track over >= 1 m straights.
On mow 20261102-030022 (28 min) window/keyframes gave 196/945 updates, a 3.0/0.3 s cadence, and
2 deg convergence after 447/106 s. Error against the track was p50 1.01/1.10 and p95 3.42/2.97 deg
(live 1.21/2.16). The two methods differed by p50 1.2 and p95 5.5 deg. On mow 20261102-021049
(9 min, mostly stopped) the drift-rate term took the window offset 10 deg away during a stop. The
keyframe rate term (scaled by gain/alpha) barely moves. Default not flipped: field test first.

Magnetometer seed experiment removed (revert of 185aab6). The LIS2MDL has a DT node but no ACK on
I2C on this unit, and the stock firmware never reads it. `docs/HEADING_RECEIVER_2026-11.md` stays.

## Heading drift-rate freeze while parked (2026-11-03, branch ws/drift-freeze, unit + offline replay)

The COG drift-rate term (both `cog_method`s) is now integrated only while RTK positions show
motion: at least `rate_motion_min` 0.1 m over the last `rate_motion_window` 5 s. When the robot is
parked the offset holds and the rate estimate is kept. Total extrapolation between COG fixes is
capped at `rate_max_extrapolation` 2 deg. Replay of 20261102-021049 (robot parked from minute 5):
the window final offset moved from -45.3 to -39.0 deg and the window-keyframes gap from 10.8 to
3.7 deg p50. On 20261102-030022, error against the RTK track was p95 2.90 deg for window (3.42
before) and 1.66 deg for keyframes (2.97 before). Keyframes convergence to within 2 deg of its
final value went from 106 to 824 s, because the final value moved.

## Short skipped stretches no longer make a mow incomplete (2026-11-03, dev-image tested only)

Field 2026-11-03, area 5: the end-of-area pass logged "stretch sub-path 4 poses 140 -> 145 not re-tried (0.23 m < 0.50 m)", counted it as "1 stretch blocked by obstacles", and ended a 99.8 % mow MOWING_INCOMPLETE. Now a skipped stretch shorter than `stretch_min_retry_m` is dropped from the area's skipped list as too short to matter (logged, its metres added to the mission's `ignored_short_m`). It never counts as blocked or NOT mowed, so the area completes. Stretches next to an unbypassable spot are still reported blocked, a 0.6 m stretch is still retried, and a retry stopped by a static obstacle is still excluded as blocked_static. The outcome text now carries the percentage mowed (planned minus NOT mowed sub-paths, skipped / blocked_static stretches and ignored short ones, over the planned metres of the finished area runs), e.g. "99.8 % mowed, 0.23 m skipped as too short". It goes on MOWING_INCOMPLETE sub_state and on MOWING_COMPLETE when anything was left out. No sim gate: this only changes completion bookkeeping, not driving.

## Bag ring compresses its own splits; wifi_monitor /proc fallback (2026-11-03, branch ws/recorder-compress, unit-tested only)

2026-11-03: the 2026-11-01 single-thread fix did not cover a stop: a ring-profile switch sent
SIGINT while rosbag2 was compressing the last closed split (started 48 s before "Pausing
recording"), and rosbag2 does not finish an in-flight compression, so split `_21` came out
"premature end". `ros2 bag record` now runs with `--compression-mode none` and bag_recorder
compresses closed splits itself (`mower_control/bag_compress.py`, nice-10 thread): `.tmp`,
full decompress check, `os.replace` to `.db3.zstd`, delete the `.db3`, rewrite
`metadata.yaml` to the `.zstd` names (and `compression_mode: FILE` once all are compressed).
The open split is never touched, leftovers are resumed on start, a split that fails 3 times
stays plain with a `.nozstd` marker. Checked in the dev image: a real 3-split recording
compressed by the worker opens with `ros2 bag info <session>`. The readers (`wheel_cal.py`,
`wheel_bag_metrics.py`, `heading_replay.py`) glob `*.db3` / `*.db3.zstd` and need no change.
The image gets `python3-zstandard` and `zstd` (Dockerfile.jazzy: needs an image rebuild; the
`zstd` binary fallback works without it only where that binary exists). Details:
`docs/crash_recovery.md`, "Rolling bag ring".

wifi_monitor: with no `iw`, or `iw` failing for more than `stale_samples` samples, the status
comes from `/proc/net/wireless` + `operstate` + `carrier` (connected with the /proc signal,
bitrate 0, BSSID/SSID empty). New `WifiStatus.source` field (`iw` | `proc`); logged once as
"iw not available: signal from /proc/net/wireless only". No roam is counted across a `proc`
sample. The GUI chip tooltip says "Signal only" for `proc`; the Go side decodes by name, no
change. The 2026-11-02 rule "iw failing past stale_samples = unknown" now applies only to
those `stale_samples`; after them the /proc signal is reported.

## Graded RTK-loss policy (2026-11-03, unit-tested; RTK-loss sim scenario pending)

Branch `ws/rtk-loss` (REVIEW_2026-11-02 step 7). The monitor's `ok` also needs the ekf_map xy
sigma <= `loc_sigma_ok_m` 0.15 m. When degraded, both controllers are capped at 0.15 m/s
through the tilt-caution speed mechanism, and a mow follow finishes its straight swath only
while est_drift + 0.3 m fits the swath's edge clearance; otherwise it holds. When lost, it
holds. After 120 s held it retraces up to 3 m of the driven track toward the last RTK-fixed
pose. After 600 s it goes to MOWING_INCOMPLETE, docks if RTK is fixed
(`return_home_on_incomplete`), and otherwise stays stopped with the mowIncomplete alert
(`loc_hold_max_s` is replaced by `loc_lost_dock_s`). A hold never charges the retry or
no-progress counters. Dock transits and docking are exempt unless lost for more than 60 s.
Details: docs/localization.md "RTK-loss policy". The sim cannot inject an RTK loss yet, so
this has only had unit tests and a plain no-regression gate.

2026-11-03, branch `ws/rtk-stale`: the new sim RTK-loss injection showed `/fix_status`
never went stale in sim. Five nodes compared the pump's node-clock receipt times with
`time.monotonic()`. They now use the node clock, and production behaviour is unchanged.
New `lost_after_s` (30 s): no RTK FIXED/FLOAT that long reads `lost` ("no RTK for N s").
The retrace trail is a 10 m ring recorded in every phase. Gates (`-d -s 2 -x 3`): a 60 s
dropout gives k PASS (lost at 30 s, resumed 3 s after the fix returned). A 700 s dropout
gives lost at 30 s, a 2.91 m retrace, then MOWING_INCOMPLETE "no RTK fixed, stopping in
place" at 600 s, with no dock. The earlier run docked only because the stale RTK still
read `fixed`. Its k FAIL (not resumed) is expected, because the policy has it stay put.



## Stock-parity small behaviours (2026-11-03, branch ws/stock-parity, unit-tested only)

The small stock Tron behaviours we lacked (OTA excluded); evidence in
`docs/audit_2026-10-05/vendor_mission_layer.md` 1.2 / 1.4 and `mower_docs/reference/IotNotice.msg`.
No sim gate: none of these changes the drive path on a healthy run (each needs a hot battery,
a flat battery, a blocked lens, a moved dock, a repeated stuck spot or an RTK-float recording).
All alerts go through the new `/mower_alerts/notice` (JSON, `mower_alerts/rules.py parse_notice`).

* **Battery temperature** (stock "battery temp high", -112): START refused outside
  `battery_temp_min_start_c` 0 .. `battery_temp_max_start_c` 45 (from `/battery` temperature,
  mcu_node BatteryInfo); mid-mow above `battery_temp_max_mow_c` 55 for 10 s the robot docks
  and resumes once <= 45. Vendor thresholds are not in the recovered code: ours.
* **Critical battery** (stock ShutdownSoon 900023): < `battery_critical_percent` 5 off the dock
  -> stop, blade off, priority-5 alert; still off the dock after `battery_critical_grace_s` 120
  -> `/base_keys/request_poweroff` (new Trigger service: the power-key power-off sequence).
* **Charger moved**: after a successful dock with RTK fixed and a measured dock pose, docked
  pose vs stored dock pose > `dock_moved_warn_m` 0.5 -> alert + latched `/mower_docking/dock_moved`
  JSON (for the GUI dock card's re-calibration offer; wired on 2026-11-04, see "GUI small
  batch" below).
* **Camera blocked** (stock CameraBlocked 100105): new `mower_vision camera_blocked` node; dark /
  washed-out / uniform front image for 10 s while MOWING in `daylight_hours` -> latched
  `/vision/camera_blocked`, alert, mission caps both controllers at `camera_blocked_speed_mps` 0.15.
* **Stuck spots -> no-go proposal**: two STUCK_NEEDS_HELP / stuck-escape events within 1.0 m,
  across sessions (`/userdata/ros2/stuck_spots.json`), propose a pending 1.0 m no-go disc
  (`auto: stuck spot #id`) for owner Accept; it survives the session-end drop.
* **Recorded-vertex RTK quality**: recordings save `<recording_quality_dir>/<name>.json` with
  fix type + hdop per vertex; `record_require_fixed` warns and flags vertices without RTK fixed
  (the GUI map colours them since 2026-11-04, see "GUI small batch" below).

## Row-end turns vs pivot time (2026-11-03, branch ws/turns, sim only)

Pivot share on real_run5 is 27 %. Row-end pivots account for 7 % of moving time, edge-lap
corners for 4 %, and transit route corners and sub-path starts for 15 %. A small area with
racetrack rows becomes 16 transits. With a 0.60 m nose and row ends 0.45 m from the line,
neither forward arcs nor bulb turns fit the 0.26 m body limit, and a pivot is the fastest
turn that does fit. `mower_coverage_node` param `turn_mode` (pivot | arc | fishtail | auto,
default pivot) lets swath turns use the edge-lap band, with a body-outline check. A gate
with auto gives 27 % and regresses d and e, so pivot stays the default. Details and next
levers (fewer transits on small areas, tangent transit ends):
docs/analysis/2026-11-03_turns.md.


## CPU diet: map_server executor + point test, MPPI batch 600 (2026-11-03, branch ws/cpu-diet, sim gate)

Review 2026-11-02 step 10. The field premise was partly stale: map_server already reads
`/odometry/filtered_map` through the SubscriptionPump (/tf is sampled, drained only by the two
rare lookups) and the boundary check is already capped at `boundary_check_rate_hz` 10.
Measured on the mower (docked, read-only /proc per thread): map_server main (spin) thread
32 % of a core at 19 wakes/s (~17 ms per wake), pump thread 33 % at 27 wakes/s (odometry);
gui_bridge 15 % in all, node_host_gui < 3 %, cmd_vel_slew tick thread 2.5 % at 24 Hz
(~1 ms/tick). So gui_bridge and cmd_vel_slew got no change (nothing hot to cut).

Changes:
- `areas.point_in_polygon`: scalar even-odd loop instead of 1-element numpy arrays per
  vertex (same rule; equivalence test against `points_in_polygon` incl. vertices/edges).
  It is the bulk of the per-odometry boundary check (robot_area_status, outside_distance).
- map_server publishers' QoS event handlers in `sub_pump.executor_hidden_group()`: every wake
  of the spin thread walked ~60 publisher event waitables (dev host: 1.23 -> 0.52 ms/wake for
  28 publishers + 30 services). Loss: this node no longer logs incompatible-QoS subscribers.
- MPPI `batch_size` 1000/800/800 -> 600/600/600 (time_steps 40 unchanged).

Gate real_run5 `-d -s 2 -x 3`, per-node CPU = `top` 60 s average during MOWING (sim time 3x):

| | base run 1 | base run 2 | after (all three) |
|---|---|---|---|
| map_server_node | 34.0 % | 35.8 % | 21.8 % |
| controller_server | 38.2 % | 44.4 % | 30.8 % |
| gui_bridge | 46.1 % | 46.8 % | 44.8 % |
| cmd_vel_slew | 20.5 % | 22.2 % | 21.2 % |
| coverage_action_server | 55.7 % | 58.4 % | 53.9 % |
| controller loop Hz median / p10 | 20 / 20 | 20 / 18 | 20 / 20 |
| "missed desired rate" warnings | 185 | 1318 | 15 |
| a max outside (m) | 0.135 | 0.114 | 0.074 |
| e corner max dev (m) | 0.119 | 0.139 | 0.119 |
| g pivot / moving | 27 % | 26 % | 27 % |
| h p99 line / arc | 0.103 / 0.067 | 0.087 / 0.066 | 0.091 / 0.062 |
| i side / nose / sweep | 0.134 / 0.056 / 0.086 | 0.093 / 0.067 / 0.081 | 0.104 / 0.055 / 0.090 |
| b, c, d, f | pass | pass | pass |

a-i are inside the documented +-0.05 m run-to-run band (the baselines themselves FAIL a/h/i
on this seed). One after run covers all three changes, so the per-change split is inferred:
map_server from the two map_server commits, controller_server from the batch size.
Sim in-container CPU is not the mower's (3x message rates, /clock wakes every node).
Projected mower saving: map_server ~65 % -> ~40 % of a core, controller_server ~25 % less
(~7 % of a core): about 0.3 core of 8. To verify on the mower: per-thread `top -H`.

## Mission parameter diet, first pass (2026-11-03, branch ws/param-diet, sim gate)

Source: the 2026-11-02 params audit. Mission `Params` 369 -> 338 fields, mission.yaml 385 -> 354
keys (-31 each). mower_mission source 15 356 -> 15 302 lines (209 deleted, 124 added; the
added lines are mostly re-wrapped comments, the startup warning and the unified params).

- Tests run what ships: the FSM test `Harness` no longer pins route_transits /
  path_conditioning / hard_boundary_body / hard_boundary_snapshot false, detour_unclassified_after
  -1 or 1.0 / 3.0 m detour skips. FSM-driven tests on the shipped flags: 0 of 690 -> 643 of 693;
  the other 50 set a flag explicitly, marked `# revert path` (or a pinned tuning value with its
  reason). The toy squares answer plan_route "no route graph" (auto_route), and a conditioning
  pivot completes when a test asks for the next action (auto_spin).
- Deleted uncalled code: `parity._speed_cap_params`, `nogo.wrap`, `external_rain.parse_rain_payload`.
- Retired 17 flags, shipped path only: the costmap-clear trio (always local, selective),
  require_blade_confirmation + blade_spinup_s (also out of the GUI param catalog), route_variation
  (area setting transit_variation still applies), obstacle_path_filter, requeue_failed_transits,
  requeue_skipped, blocked_pivot_requeue, resume_skip_failing, edge_blocked_skip,
  sweep_blocked_skip, retry_each_skipped_stretch (with stretch_block_radius_m and the unbypassable
  shortcut it gated), exclude_marked_obstacles, turn_leg_end_by_distance, blade_keep_on_obstacles
  (owner rule). Kept: manual_blade_requires_enable (owner toggles it), route_transits,
  path_conditioning, hard_boundary_*, heading calibration flags.
- Unified: `slow_maneuver_mps` 0.10 (7 speeds + the hard-coded `min(v, 0.10)` clamps in
  boundary.py / spin_drift.py), `caution_speed_mps` 0.15 (tilt / camera blocked / loc degraded),
  `backout_m` 0.30 (5 distances; pivot_backout_m was 0.35, no test needed it),
  `heading_status_timeout_s` 5 s is also the post-calibration COG wait (was 3 s).
- Startup WARN for each of hard_boundary, stuck_guard, tilt_guard, loc_hold,
  manual_blade_requires_enable set false.
- Dock timeout: the docking server enforces the Dock goal's timeout_s itself; its worst case is
  1 + 8 retries x (docking_timeout_s 60 + final 10 s) + 8 x retry_timeout_s 15 = 750 s, so the old
  480 s cut the stock 8-retry cycle off around retry 5. `dock_timeout_s` 480 -> 900,
  `dock_action_timeout_s` 540 -> 960; max_retries stays 8 (stock, owner choice 2026-10-21).

Gate real_run5 `-d -s 2 -x 3`: a 0.078 m / 0, b-h PASS, i 0.111 / 0.077 / 0.080, transit 0.388
(FAIL as on the baseline: 0.101 / 0.098 / 0.082, 0.364), g 25 %, controller 20 Hz. Same as the
baseline within run-to-run spread. Not field tested.

## GUI small batch (2026-11-04, branch ws/small1104, vitest + Go tests; not run against the sim or the mower)

Open GUI items from the open-work audit (2.2, 2.4, 2.6) and `REVIEW_2026-11-02.md` section 5
"Also:". Paths are under `third_party/mowglinext/gui/`.

* **Charger moved.** `/mower_docking/dock_moved` is in the provider topic map and the multiplex
  allowlist as `dockMoved`. The vision dock card (Settings > Docking) shows a warning with the
  distance and points at "Set dock = current robot pose" while the last check found the charger
  moved and the stored dock pose is still more than `warn_m` from where the robot docked. The
  bell titles `dockMoved`, `cameraBlocked` and `batteryTemp` notices.
* **Recorded-vertex RTK quality.** `GET /api/mowglinext/recording-quality` reads the mission's
  sidecars from the shared maps volume (`/ros2_ws/maps/record_quality`). The map draws the
  recorded vertices of every area, no-go zone and path whose name is still in the map: view
  mode only the vertices without RTK fixed (amber float, red worse), Edit Map all of them
  (green fixed). The match is by name, so a renamed item loses its marks and a new item that
  reuses an old recording's name shows the old marks.
* **F12.** The dialog after drawing a polygon is a corner panel (bottom sheet on phones), so
  the polygon stays visible; every type takes a name; the saved shape is the draw store's
  current copy; no-go zones are labelled on the map ("No-go zone N" without a name). The
  "vanishes for 4-6 s" part was not reproduced (no sim GUI run in this batch).
* **F14.** Path and navigation-area labels sit on the middle of the path's centreline (or on the
  polygon), not on the centroid. Tapping a path opens a path sheet (name, width, length, Edit
  path) instead of the Mow settings drawer.
* **F24.** The `/rosout` hub subscribes at GUI start (`RosoutHub.Pin`), so the Logs page opens
  with up to 1,000 earlier lines. Cost: `/rosout` now flows through foxglove_bridge all the time.
* **Naming.** The map polygon is a "No-go zone" everywhere; "obstacle" is kept for detected
  things (tracked obstacles, obstacle detection). The boundary line's "keep-out width" reads
  "clearance from the line"; the French no-go sheet is translated.
* **Mow angle.** The planner takes `mow_angle_deg` as a map-frame heading (0 deg east, 90 deg
  north, counter-clockwise), but the area picker drew it as a compass bearing (0 deg north,
  clockwise). The picker now uses the planner's convention; stored values and what the mower
  does with them are unchanged. Area Mow settings, Settings > Mowing, the parameter catalog and
  the live binding now describe the same order: the area's own direction, else (area on Auto)
  the slope-aware angle, else the global default, else the planner's choice.
* **Diagnostics.** Camera freshness statuses ("/left_oa_camera/image_raw topic status") are
  grouped under "cameras" instead of "unknown".
* **Schedule rules.** Checked, nothing to do: `components/schedule/scheduleRules.ts` reads
  `rain_mode`, `battery_low_action` and `battery_low_percent` (ws/ux-batch).
* **B11 drive-load detector.** Not wired into any node. `motor_load.replay()` and
  `scripts/drive_load_events.py` run it over a mow bag (`/mower_sensor_info`, `/cmd_vel`,
  `/fix`) and print the hits, so "log-only for 2 mows" can be done from the bags with no effect
  on the robot. The rule uses the sum of both drive currents, so B10 does not change it (tested).
  Smoke run on one 2026-10-28 incident bag: 240 motor frames, 0 events.

## Night dock: SEARCH filter heading from RTK (2026-11-04, branch ws/dusk1104, unit + sim; not field tested)

Field 2026-10-31 20:02 night dock: 77 s and 5 approaches, against 27 s by day. The scene was
black and only the lit marker was visible. Detection stayed at 100 %, so the exposure stepper
never stepped, but the marker yaw swung +-20 deg per frame. Each SEARCH seeded the filter from
the median marker yaw: -13.5, -7.5, -13.3 and -21.2 deg, against RTK -3..+9 deg. Each approach
therefore started 0.1-0.2 m off in lateral, which cost 2 pose realigns, 2 near realigns and a
retry.

- `search_seed_rtk_heading` (mower_docking, default true). Three conditions must hold: the dock
  pose is measured, RTK is fixed, and `/heading_aligner/status` reports `quality` in
  `heading_trusted_qualities` (['good'], fresher than `heading_status_timeout_s` 5 s). The
  filter then starts on the RTK docked heading, and the accepted detections are re-headed onto
  it (`rehead_errors`), so the marker centres stay the position source. Otherwise the seed is the
  marker median as before. The log line is "filter heading seeded from RTK".
- Tried and dropped: picking the `solvePnPGeneric` IPPE solution nearest the RTK / filter
  heading. On mower_sim-rendered frames it gave 3-5 approaches where the seed alone gave 1. The
  0.055 m marker read at `marker_size` 0.04 biases the true solution's normal, so the mirror
  solution is often the nearer one (mower_docking README, "Night dock").
- mower_sim: the world key `rear_camera: {dusk, bloom_px, corner_noise_px, seed}` renders a black
  scene (brightness about 10) with a blooming lit marker. Per-frame corner jitter swings the
  marker yaw by +-15-20 deg. World `dock_dusk` = `dock_approach` with that scene.
- Tests: `test_dock_dusk_2026_10_31.py`. On the night-like stream, the master seed lands 12-16 deg
  and 0.2-0.3 m off, and the RTK seed within 1 deg and 2 cm. When the dock pose is not measured,
  RTK is not fixed or the heading is not trusted, the seed is exactly master's. Over 10 seeds in
  closed loop, master needs 24 realigns and the RTK seed 17.

Results (master-equivalent = this branch with `search_seed_rtk_heading: false`, which is
master's code path):

| Run | Master | Branch |
|---|---|---|
| mower_sim `dock_dusk` seed 1 | docked 41 s, 3 approaches, 2 near realigns | docked 40 s, 3 approaches, 2 near realigns |
| mower_sim `dock_dusk` seed 2 | docked 16 s, 1 approach | docked 14 s, 1 approach |
| Offline loop on rendered dusk frames, 0.5 px jitter, seeds 1-6 from 1.2 m | 5/6 docked, 14 approaches, 30 s mean | 6/6, 11 approaches, 22 s |
| Same, no jitter (the sim's fixed yaw bias only) | 6/6, 18 approaches, 34 s | 6/6, 6 approaches, 12 s |
| Gate real_run5 `-d -s 1 -x 1` | 41e61e3 (fix1104 `master-d1`): a 0.078 / 0 PASS, h FAIL 0.084 / 0.104, i FAIL; dock 3 approaches | c81a556: a FAIL 0.121 m / 85 samples (MOWING at (-5.28, -0.88), the spot where master's body worst lies), h PASS 0.099 / 0.092, i FAIL; b-g PASS; docked and charging, 3 approaches |

The gate differences are in MOWING, where the docking change does nothing, so they are
run-to-run spread. The mower_sim dock runs show no gain on these two seeds. The SEARCH
acceptance already rejects most clusters 20 deg off, and the near-field check still realigns on
the raw marker heading. The fix removes the seed error that the night showed, but the
near-field check at night is still open. Dusk field test: dock in the dark with RTK fixed and a
'good' heading. Look for "filter heading seeded from RTK" and fewer pose realigns. Note the
near-field check outcomes (raw-heading trips) to decide whether that check needs the filter
heading next. A darker night exposure preset is not built.


## A3 coverage map, slice 1: mowed raster, honest % mowed, resume by the raster (2026-11-05, branch ws/cov1105, unit + sim; not field tested)

Before this, "% mowed" was plan progress, nothing recorded what was actually cut, and resume
used a plan cursor plus (since ws/fix1104) the mowed polylines of the plan. Stock resumes from
position plus a mowed raster (`analysis/2026-10-21_stock_firmware_findings.md` "3. Resume after
STUCK").

**What is recorded.** `mower_mission/mowed_raster.py` keeps one `MowedRaster` per area:
- 0.05 m boolean cells over the area's bounding box plus 0.5 m. The 0.22 m disc is about 4.4
  cells, so its edge lands within 2.5 cm.
- The mission (`fsm/coverage.py`) stamps the disc swept since the last tick, on every 10 Hz
  tick, when all of these hold:
  - the cutter REPORTS cutting (`MowerBaseDevStatus.is_cutting`, not the command);
  - the localisation is trusted: `/localization/status` ok, or never received (no
    localization_monitor, as the hold reads it). Degraded (RTK float, sigma over the limit),
    lost and a stale status are not stamped;
  - the blade point is known: base_link -> blade_link from `/tf_static` (the URDF
    `blade_joint`, 0.23 m today; mission_node chains the static tree) and the width is
    `cut_width_m`. Without the transform nothing is stamped and one WARN says so.
- A blade jump of more than 1 m between two ticks is not swept.
- Cost: 18 us per tick; the percentage is under 0.01 ms after the first call (the mask is
  cached).

**Persistence and clear rules.**
- The raster is an `area_raster <idx> <key> ...` row of the resume cursor (zlib + base64; it
  replaces the `area_mowed` polyline rows). It is saved with every cursor save and at least
  every 30 s while cutting. Upstream readers ignore the row.
- Key: area index (what resume.py uses) plus a CRC of the area's outer boundary. The map has no
  persistent area id. An edited boundary, or an index that now names a different area (an
  area deleted before it), therefore clears the raster with a WARN. Holes (accepted zones) do
  not change the key: a new zone does not wipe what was cut.
- Empty again on: a new full mow (a new cursor), `clear_coverage_resume`, each repeat / cross
  run (a new area cursor), and mission complete.

**Honest %.** `~/mow_progress` gets `mowed_area_percent` next to the unchanged `percent`. It is
the cut share of the mowable polygon: outer minus the accepted obstacles and the mid-mow re-plan
zones. Every area run logs `area N: X % of the mowable area cut (mowed raster)`. The GUI
fields and their meaning are unchanged, and the GUI does not show the new value yet. The value
stays a few points under the plan's 99.9 %: the blade disc cannot reach the strip along the
boundary. On real_run5's 19.3 m2 lawn that is 3.6 points, about a 3.6 cm strip.

**Resume by position.**
- A plan pose counts as mowed when 75 % of its footprint cross-section is cut: the swath
  width, sampled across the path at the blade point of base_link on that pose, with its yaw.
  - Why 75 %: of the five 5 cm samples across a 0.18 m swath, one may be missing (a 5 cm
    tracking offset, the quantised disc edge), two (half the swath) may not.
- A sub-path at least 90 % mowed by length is done. The mowed lead-in (at least 0.5 m) of
  the first sub-path left is the resume point. Both rules are kept from `_cursor_remap` and
  the mid-mow re-plan.
- Where the raster is used:
  - **Same plan as the cursor:** it also skips the sub-paths the raster shows mowed (a failed
    one, one a detour finished). It moves the resume point forward to the end of the mowed
    lead-in when that lies ahead of the cursor. It never moves it back; an uncut gap is the
    patch report's.
  - **`_cursor_remap` (changed plan) and the mid-mow no-go re-plan:** both map the new plan
    onto the raster instead of onto the plan polylines.
  - Before `/tf_static` has given the blade offset, the raster is not read at all, so the
    cursor decides.

**Missed patches, report only.** `missed_patch_min_m2` (default 0 = off). At the end of each
area run, the uncut parts of the mowable polygon that the blade fits into are found. An
opening by half the blade width drops the boundary strip and the seams. Those of at least that
size are logged as `uncut patch(es)` with size and centre, plus a `missed_patches` incident.
Driving them is slice 2.

**Removed for A6.** The raster replaces the polyline mechanism:
- the `area_mowed` rows and `AreaCursor.mowed`;
- `_cursor_mowed_update`, `mowed_pieces`, `simplify_polyline`, `_nogo_mowed`;
- `nogo.mowed_progress` with its point grid;
- the `nogo_progress_tol_m` parameter.

`mission.yaml`: 367 keys before and after (`nogo_progress_tol_m` out, `missed_patch_min_m2` in).
`cut_width_m`, "display only" before, now sizes the raster disc.

**Limits of slice 1.**
- map_server still paints its own `~/mow_progress` OccupancyGrid for the GUI map. That grid is
  not loc-gated and not persisted.
- Covered stretches in the middle of a sub-path are re-driven; only the lead-in and whole
  sub-paths are skipped.
- A cursor saved by an older build has no raster: a remap then logs the "could not be mapped"
  error and starts the area over.

**Tests.**
- New: `test_mowed_raster.py` (20). It covers stamping, blade off, untrusted localisation, no
  transform, %, file round trip and torn rows, the 30 s save, the clear rules (new mow, edited
  boundary, repeat run), resume skip and lead-in, the no-go re-plan on the raster, the GUI
  field and the patch report. They all fail on master, which has no raster.
- Changed: the old polyline tests are removed; the harness drives the pose along the path (the
  raster needs the track) and puts the blade point at base_link.
- `scripts/sim_gate`: evaluate.py reports, without judging, the lawn's cut share from ground
  truth (same raster, blade at the deck centre while the sim reports cutting), the mission's
  raster value and plan % and, after a STOP -> START, the blade path in MOWING over ground cut
  before the STOP.

CI (`scripts/ci_test.sh`, 154fa66): colcon PASS (3846 cases, 0 failed), scripts PASS, replay PASS.

**Sim (real_run5, one run at a time, performance profile).**

All runs: "desired rate of 60" appears in every `-x 3` launch.log, the controller loop held a
20 Hz median and p10, and every bag was pruned. The yesterday baselines are from the tasking
notes (code now on master). The master runs of 2026-11-05 use f8bbbc7 on the same host, the
same day.

| Gate | master (yesterday) | master f8bbbc7 (today) | branch | new failures |
|---|---|---|---|---|
| `-d -s 1 -x 1` | a 0.092, e 0.101, h 0.078 / 0.088 PASS, i 0.129 FAIL | a FAIL 0.128 m / 53 samples at (-0.52, 3.18), e 0.104, h 0.089 / 0.073 PASS, i FAIL 0.157 | run 1: a 0.094, e FAIL 0.175 (lap-0 corner pair (-1.24, 5.06)), h FAIL 0.108 / 0.192, i FAIL 0.229. Started right after the build, load1 4.05. Run 2: a FAIL 0.114 m / 57 at (-5.26, -0.90), e 0.111, h FAIL 0.088 / 0.114, i FAIL 0.141 | none consistent, see below |
| `-l -x 3 -s 1` | PASS: a 0.066, e 0.096, i 0.076 | FAIL: a 0.057, e 0.097, h 0.081 / 0.056, i FAIL 0.108 | FAIL: a 0.091, e 0.120, h 0.087 / 0.067, i FAIL 0.112 | none (i level with master today) |

The `-d` failures move between a, e and h from run to run on both codes. Master today failed
a; the branch failed e+h in run 1 and a+h in run 2. Branch run 2's a excursion is at the west
corner where `ws/dusk1104` (no mowing change) also failed a with 0.121 m. On a fresh mow the
branch changes nothing in the motion path. It adds the stamping (18 us per tick), a cursor save
every 30 s and log lines; the plan, the resume cursor use and every guard are master's. The e
0.175 of run 1 is the one value outside the spread seen so far, and it did not repeat on the
calm-host rerun.

**Coverage (reported).** Ground truth uses the same raster, with the blade at the deck centre
while the sim reports cutting:

| Run | lawn cut, ground truth | mission raster | plan % |
|---|---|---|---|
| `-d` run 1 | 96.2 % | 96.2 % | |
| `-d` run 2 | 96.0 % | 96.2 % | 99.8 % |
| `-l -x 3` | 96.4 % | 96.9 % | 99.9 % |
| resume scenario | 96.3 % | 96.6 % | 99.8 % |

- The raster matches ground truth within 0.5 point (`-l` localisation drift: the EKF is not
  the ground truth).
- It reads 3.2-3.7 points under the plan %. That is the strip along the boundary the disc
  cannot reach (see "Honest %").
- Stated tolerance: raster vs ground truth <= 1 point; raster vs plan = plan minus the boundary
  strip, 3-4 points on this 19 m2 lawn.

**Resume scenario.** `--at 500:cmd:8 --at 530:cmd:1 -l -x 3 -s 1 -t 3600` on real_run5:
- STOP mid sub-path 3, START 30 s later. The same plan came back and the log read `area 0:
  resuming (3/11 sub-paths done, cursor (3, 471))`. MOWING_COMPLETE, docked, charging.
- a 0.075, e 0.093, h 0.073 / 0.069 PASS; i FAIL 0.100, as master today.
- In the first 12 s after the resume, 37 of 1042 blade samples lay on ground cut before the STOP
  (under 0.1 m, the disc overlap at the stop point).
- Share of the blade path in MOWING over ground cut more than 20 s earlier (row-end pivots and
  infill sweep over the neighbouring swath):

| Window | resume run | `-l -x 3 -s 1` run, no STOP |
|---|---|---|
| before +500 s | 14 % | 16 % |
| after the resume / after +500 s | 20 % | 21 % |

  The resume re-cut no more than an uninterrupted mow does.
- This scenario resumes on the same plan, so it exercises the raster save and the same-plan
  path. The changed-plan mapping (`_cursor_remap`) is covered by the unit tests only.

**Slice 2.**
- Drive the missed patches: plan the patch inside the area and reach it with the transit /
  route.
- Skip covered stretches in the middle of a sub-path (a jump like `_excluded_jump`).
- GUI: show `mowed_area_percent`, and draw the mission's raster in place of map_server's
  unpersisted, ungated paint (then retire that paint).
- A resume scenario with a changed plan in the sim.
- A field mow.

## Corner pockets: pocket spurs in the gap fill (2026-11-05, branch ws/corner1105, unit + sim; not field tested)

Field (b4437d0): the owner saw an uncut triangle in the west lobe of area 3 ("further backyard
lawn", the sim's owner_far Area 6) inside the third lap. Analysis, gap table and PNG method:
`analysis/2026-11-05_corner_pockets.md`.

**Cause.**
- Rows under 1.2 m are dropped, and the lobe is too narrow for longer ones.
- A fill pass must be at least 1.0 m long and add 35 % new area.
- The fill search picked the best pass by gain and only then applied those limits. When the
  best pass failed them, the search gave up on the pocket.
- On the live polygon a fill pass happens to cross the lobe, but fill passes run after every
  row. At 59 % the mower had not reached it yet. On the owner_far copy there is none: the
  sim ground truth leaves 0.633 m2 there.

**Change (`mower_coverage` `fillGaps`, edge layout only).**
- The fill search picks the best pass among those that meet the limits.
- A pocket no fill pass fits gets in-and-out **pocket spurs** if the blade fits into at least
  0.03 m2 of it:
  - forward along a line of the row region (base_link at least 0.45 in) into the pocket;
  - pivot about at the end: the sweep of a row end, at most 0.195 m past the line;
  - back out on the same line, or straight to the next spur's mouth (a "V").
- A spur is at least 0.5 m long and adds 35 % of its strip.
- A spur is planned path: two swaths for the bridge. The mission sees ordinary pivots.
- No new parameters.

**Tests.**
- Unit: `EdgeLayoutSpurs.LobePocketsGetSpurs` (synthetic lobe) and `OwnerFarPocketsShrink`
  (owner_far Area 6 and heart). Both fail on master. They also check the body outline at
  every spur end through the about-turn (at most 0.26 out).
- Offline chain (planner, bridge splitter, mission conditioning, idealised drive, blade
  raster), blade-fit patches:
  - owner_far Area 6 0.635 m2 to none;
  - heart 0.133 to none;
  - real_run5 unchanged 0.063 (that lobe has no inner laps).

**Sim gate (-l -x 3 -s 1).**
- real_run5: a 0.089, e 0.091, h PASS, i FAIL 0.086 (master 0.112). Ground truth 97.0 %
  (master 96.4 %), MOWING 777 s (master 740 s).
- owner_far: a 0.079, h PASS, b 0, no STUCK. Ground truth 99.2 % (master 97.0 %), MOWING
  3238 s (master 3122 s, +3.7 %).
- owner_far (e) FAIL 0.181 is at a sub-path identical to master's: drift noise, shown by
  re-evaluating sub-paths 0-5. (i) fails as on master. Over the spur legs, nose / sweep over
  plan is 0.12 / 0.15.
- real_run5 -d -s 1 -x 1: a 0.087, b 0, no STUCK, ground truth 96.0 %. (e) 0.155 is at a
  TRANSIT route corner, not a spur. (e), (h) and (i) fail here run to run on master too.

**Not fixed (next levers, owner's call).**
- Inner laps opened by `lap_corner_radius` 0.7 vanish in features narrower than about 2 m.
  The real_run5 SW lobe, live Area 5 north and live Area 4 have only lap 0. Area 4, a strip
  about 1 m wide, has no rows and cuts 42 % offline.
- Acute lobe tips that the row region does not reach: live closer backyard lawn, 0.273 m2.
- The thin seams in the field raster are real tracking or localisation offsets between rows
  driven at different times, against the 0.04 m nominal overlap. They are not sampling
  artefacts. Isolated cells are the raster's 2.5 cm disc quantisation.

`mission.yaml` and the planner parameters are unchanged (A6).
