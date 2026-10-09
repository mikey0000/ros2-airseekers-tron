> Audit date: 2026-10-05. Produced by a Claude Opus subagent (read-only audit of `ros2_stack/`).
> Point-in-time snapshot: several defects listed here were fixed the same day. See `docs/STATUS.md` for the current state.

# Audit of the Airseekers Tron ROS 2 Jazzy stack (`/home/michael/git/airseekers-decompile/ros2_stack`)

I read the code, configs and launch files and ran the pure-python tests. I did not modify anything, did not run colcon, and did not touch the mower. `jbcontext` was unavailable (MCP connection failed), so all discovery was done with grep and direct reads.

**Bottom line:** the drivers are fairly well developed. Everything above them is either a disconnected piece or missing. What is launched today cannot move the robot autonomously or run the blade, and several nodes would crash or fail to build on real Humble even though their tests pass.

## 1. Package table

LOC counts include docstrings. "Tests" means test functions found / result of `python3 -m pytest` run here on a host without rclpy (stubbed ROS).

| Package | Purpose | Lang | LOC | Tests | State | Pub / Sub / Srv | Launched by |
|---|---|---|---|---|---|---|---|
| `mower_mcu_driver` | A5…5A MCU codec, odom, battery | py | 1965 | 24 → **24 pass** | Most complete; no services, cutter, e-stop or rain | pub `/battery` `/odom` `/imu` (placeholder) `/mower_sensor_info`; sub `/cmd_vel` **TwistStamped** (`mcu_node.py:428-444`) | bringup |
| `wit_imu_driver` | JY61P IMU | py | 321 | **0** (README claims 25) | Suspect parser (see §4) | pub `/imu` (BEST_EFFORT), `/imu/temperature_c`; TF off by default (`wit_node.py:47`) | bringup |
| `um960_gps_driver` | UM960 GNSS | py | 2350 | 47 → **28 pass, 19 fail to collect** | Good, but no NTRIP | pub `/fix` `/vel` `/heading` `/fix_status` `/nmea` | bringup |
| `mower_localization` | `gps_gate` + ekf/navsat YAML | py | 597 | 25 → **25 pass**, but stubbed wrongly (§4) | **Crashes on Humble** | `/fix` → `/fix_gated` | nav2 |
| `mower_control` | cmd_vel slew, slip detector, imu_cal | py | 721 | **0** | Written, never wired | slew `/cmd_vel_raw`→`/cmd_vel` (Stamped); slip →`/dig_stall`; imu_cal sub `/imu/data` | **none** |
| `bumper_controller` | back up and rotate after a bumper hit | C++ | 386 | 0 | **Does not compile** (§4) | sub `/mower_base/status`, `/odom`; pub `/cmd_vel` **Twist**, `/bumper_cloud`; srv `/test_bumper_service` | bringup |
| `mower_coverage` | Fields2Cover 2.1 boustrophedon planner | C++ | 1129 | 15 gtest (not run; agent14 log claims 58) | Plausible, unbuilt | srv `/coverage/plan` (frame `map`) | **none** |
| `mower_interfaces` | 11 msg + 7 srv files | idl | – | – | **`MappingControl.srv` and `SetLoRa.srv` are on disk but not in `CMakeLists.txt`** | – | – |
| `base_ble` | BLE UART app bridge (teleop) | py | 711 | 14 → **14 pass** (framing only) | Will fail to import | pub `/cmd_vel` **Twist**; clients `/clear_estop` `/logic/cutter_control` `/mapping_control` `/mower_gps_node/set_lora` | **none** |
| `base_keys` | evdev key presses → button info | py | 217 | 0 | WIP; `python3-evdev` not declared or installed | pub `/mower_base/button_info` | **none** |
| `mower_proto` | generated `*_pb2.py` | py (gen) | 881 | 0 | Generated with protobuf ≥3.20 (`builder` import); Jammy's `python3-protobuf` is 3.12, so likely an ImportError (high confidence, unverified) | – | – |
| `mower_rknn` | rknn-lite wrapper | py | 192 | 0 | Library; rknn-toolkit-lite2 not in the image | – | – |
| `det_ros` | YOLOv8 on the NPU | py | 437 | 4 → **4 pass** | Not deployable: relative `model/…` path, no models | sub `/left_oa_camera/image_raw`…; pub `/ai/det/*` | **none** |
| `seg_ros` | PP-LiteSeg on the NPU | py | 218 | 0 | Same as det_ros | pub `/ai/seg/*` | **none** |
| `mower_cameras` | Metoak stereo split + rear camera | py | 246 | 0 | WIP | pub `/vio/left` `/vio/right` `/rear_camera` | camera.launch |
| `stereo_vio_bridge` | stereo + ICM IMU → VIO input | py | 769 | 6 → **6 pass** | WIP; IMU only ~10 Hz (`perception_vio.md` §Blockers 2) | pub `/vio/{left,right}/image_raw`, `camera_info`, `/vio/imu` | vio |
| `open_vins` | symlink to the handoff clone | – | ext | – | **Symlink breaks inside the container** (§4) | – | vio |

**Never built here.** There are no `build/`, `install/` or `log/` directories anywhere under the stack, and `.gitignore` is absent. Commits `422a569` ("Fix runtime bugs") and `75a5353`/`4316cab` suggest the first three drivers were colcon-built and run once somewhere else. `bumper_controller` and the Humble constant problems show the later packages never were.

Other repo facts:
- `launch/__pycache__/*.pyc` is committed.
- `launch/vio.launch.py` has uncommitted changes and `config/vio/` is untracked.
- `agent20_vio_wire.log` is a single line, a rate-limit error. `agent17` and `agent19` also died on rate limits.

## 2. End-to-end wiring as launched

**`launch/bringup.launch.py`** starts `mcu_node`, `wit_node`, `um960_node` and `bumper_controller_node`. It does not start `base_keys`, `base_ble`, any `mower_control` node, `robot_state_publisher` or `foxglove_bridge`.

**`launch/nav2.launch.py`** starts `robot_state_publisher` (xacro), `gps_gate`, `navsat_transform_node`, `ekf_node`, `controller_server`, `planner_server` and `lifecycle_manager`. There is no `bt_navigator`, `behavior_server`, `velocity_smoother`, `collision_monitor`, `smoother_server`, `map_server` or `nav2_params.yaml`.

**`launch/vio.launch.py`** starts `stereo_vio_bridge` and `ov_msckf/run_subscribe_msckf`.

### Topic and frame mismatches

| Issue | Evidence |
|---|---|
| **No one publishes `/imu/data`.** `wit` publishes `/imu`, but `ekf_node`, `navsat` and `imu_cal` read `/imu/data`. The remap is applied to `ekf_node`, which does not subscribe to `/imu`, so it does nothing. | `wit_node.py:67`; `ekf.yaml imu0: /imu/data`; `nav2.launch.py:261,286,289-292`; `imu_cal.py:109` |
| **Two `/imu` publishers**: wit (BEST_EFFORT) and the MCU placeholder. | `mcu_node.py:430`, `wit_node.py:67` |
| **`/cmd_vel` has two message types.** `mcu_node` subscribes TwistStamped; `bumper_controller` and `base_ble` publish plain Twist on the same topic. They will never connect, so bumper recovery and BLE teleop cannot drive the robot. | `mcu_node.py:444`; `bumper_controller.cpp:39`; `base_ble_node.py:77` |
| **Nothing consumes Nav2's output.** Nav2 publishes `/cmd_vel_nav`; `cmd_vel_slew` (unlaunched) expects `/cmd_vel_raw` as TwistStamped. | `nav2.launch.py:330`; `cmd_vel_slew.py:84` |
| **GPS frame name mismatch.** The driver stamps `gps` but the URDF link is `gps_link`. `nav2.launch.py:73-76` and the xacro header both claim bringup overrides it to `gps_link`; it does not. | `bringup.launch.py:46`; `mower.urdf.xacro:203` |
| **`bumper_controller` waits on a topic nobody publishes.** It needs `/mower_base/status` (MowerBaseDevStatus); `mcu_node` publishes `/mower_sensor_info` (MowerSensorInfo). The bringup docstring admits it is dormant. | `bumper_controller_node.cpp:56`; `bringup.launch.py:7-9` |
| **No cutter path at all.** `MOD_CUTTER` is marked "not sent yet", there is no `/cutter_control` server, and `base_ble` calls `/logic/cutter_control`. | `mcu_node.py:123`; `base_ble_node.py:79` |
| **`/clear_estop` does not exist.** `base_ble` calls it as Trigger; the vendor contract uses Empty; no server anywhere. | `base_ble_node.py:78` |
| **No `map` frame**, yet the coverage path and the planner's global costmap default to `map`. | `nav2.launch.py:50-56`; `mower_coverage_node.cpp:86` |
| **Camera topics collide.** All three `v4l2_camera` nodes have `namespace=''`, so they all publish `/image_raw` and `/camera_info`. `det_ros` and `seg_ros` subscribe `/left_oa_camera/image_raw`, which never appears. | `cameras.launch.py:23-24`; `det_ros_node.py:39` |
| **Two competing stereo producers on one device.** `mower_cameras` publishes `/vio/left` while OpenVINS expects `/vio/left/image_raw`; both default to `/dev/video11`. | `camera_node.py:97`; `kalibr_imucam_chain.yaml:25` |
| **Nav2 `controller_server` would die on configure.** `FollowPath`, `AlignToHeading` and `general_goal_checker` are declared without `.plugin` types (Humble aborts with "plugin param not defined"), and `use_stamped_odom` is not a Humble parameter. With no costmap params the defaults apply, so the global costmap waits on `/map` and the `map` TF forever. | `nav2.launch.py:304-348` |
| **`navsat_transform_node` gets no input**: `gps_gate` crashes (§4) and `/imu/data` is missing. | – |

## 3. Gaps to a minimum viable autonomous mow (in dependency order)

**Direct answers to your checklist:**
- **Mission/task state machine or behaviour tree:** none. No BT, no `bt_navigator`, no task node; `PlannerTaskSet.srv` has no server.
- **Coverage planner wired to Nav2:** no. `/coverage/plan` exists but is unlaunched, and nothing sends its `nav_msgs/Path` to `controller_server`'s `follow_path` action.
- **Docking behaviour:** none. No `opennav_docking`, `ChargingControl.srv` has no server, and dock pose and charge state are not handled.
- **Map store (polygons/GeoJSON):** none. There is no zone store and no lat/lon to `map` datum conversion.
- **User-facing control surface:** effectively none. `base_ble` exists but is unlaunched and its imports are broken. `foxglove_bridge` may be installed (`Dockerfile.jazzy`, guarded) but is never launched. No HTTP API, no MQTT, no web UI.
- **E-stop / safety chain:** host side, none. Details in step 4.

**Ordered gap list:**

0. **Boot.** `mower-ros2.service.example` runs `compose up -d`, and compose's `command: bash` launches no ROS nodes. `run_stack.sh` only does `ros2 launch <pkg> <file>`, but the top-level `launch/*.py` files belong to no package. Nothing starts the stack at boot.
1. **Image.** Missing: `ros-jazzy-robot-localization` (still absent despite `nav2.launch.py:36`), `xacro`, `robot-state-publisher` (explicitly), `v4l2-camera`, `python3-evdev`, rknn-toolkit-lite2, and protobuf ≥3.20. `/userdata` is not mounted, so the `stereo_vio_bridge` calibration directory is missing.
2. **Drivers that actually run on Humble:**
   - Fix the `gps_gate` constants and the `um960` RTK status mapping.
   - Fix or verify the WIT 11-byte frame parser.
   - Make `/cmd_vel` one type across all producers.
   - Fix the `bumper_controller` compile error.
   - Register `MappingControl` and `SetLoRa` in `mower_interfaces`.
3. **MCU contract.** Characterise the heartbeat period (a guess today, `mcu_node.py:63-67`). Add a cutter command (`MOD_CUTTER`) with `/cutter_control`, plus `/clear_estop`, charging state, rain, and a `MowerBaseDevStatus` publisher. `linear_max` defaults to 1.5 m/s against the vendor's 0.3.
4. **Safety chain.** The design relies entirely on unverified MCU firmware cut-offs. On the host:
   - No lift, tilt or bumper interlock gates `/cmd_vel` or the blades.
   - No `collision_monitor` or `twist_mux` priority lanes.
   - `/dig_stall` has no consumer.
   - The BLE 8 s blade watchdog lives in an unlaunched node.
   - No `/estop` topic.
5. **Localization.**
   - No `map→odom` publisher (dual EKF or a UTM datum decision is still open).
   - The JY61P is a 6-axis IMU, so its yaw is not earth-referenced. `navsat_transform` needs an absolute heading (`yaw_offset`, or use `/heading` if the UM960 is dual-antenna — unverified).
   - Lever arms in the URDF are TODO.
6. **Nav2 configuration.** Write a real `nav2_params.yaml` with costmaps in `map` or a GPS frame, controller plugins, a goal checker, `behavior_server`, `velocity_smoother`, `collision_monitor`, then `cmd_vel_slew` → `mcu_node`.
7. **Map store**: GeoJSON zones, boundary, keep-outs and dock pose, with a datum.
8. **Coverage execution.** Launch `mower_coverage`, then a task node calls `/coverage/plan` and sends the path to `follow_path`, controlling the blade per segment.
9. **Mission state machine / BT**: idle → undock → transit → mow → (low battery, rain or fault → return) → dock → charge → resume.
10. **Docking**: `opennav_docking` or an equivalent, a dock pose, and charging detection from the MCU.
11. **Control surface**: at minimum, launch `foxglove_bridge` and add start/stop/e-stop services. Later, fix the BLE bridge or port the vendor HTTP API from `openapi.json`.

Perception (det/seg/VIO) is not on the critical path for a basic RTK mow, and nothing consumes its output anyway (`perception_vio.md` Blockers 3-4).

## 4. Quality red flags

**Tests that pass only because the stubs are wrong:**
- **`mower_localization/test/conftest.py:33-42`** invents `NavSatStatus` values (`FIX=1`, `DGPS_FIX=2`, `RTK_FIX=4`, `RTK_FLOAT=5`…).
  - Real Humble only has `NO_FIX=-1`, `FIX=0`, `SBAS_FIX=1`, `GBAS_FIX=2`.
  - So `gps_gate.py:67-72` will raise AttributeError at import on Humble.
  - The launch default `min_fix_status=1` (`nav2.launch.py:195`) also rejects every `STATUS_FIX`.
  - The 25 passing tests prove nothing about Humble behaviour.
- **`um960_gps_driver`.** `test/conftest.py:36` still stubs `STATUS_DGPS_FIX` after `um960_node.py:52` switched to `STATUS_GBAS_FIX`, so all 19 node tests fail to collect.
  - Separately, the node maps RTK fixed (GGA quality 4) to `STATUS_FIX`, below DGPS (`um960_node.py:49-56`). No downstream gate can prefer RTK by status.

**Build and parser defects:**
- **`bumper_controller` compile error.** `bumper_controller.cpp:17-22` uses `integral_`, `prev_error_`, `out_max_` and `out_min_`; the header declares `integral`, `prev_error`, `out_max` and `out_min` (`bumper_controller.h:63-67`).
- **`wit_imu_driver` parser looks wrong** (high confidence; no tests and no hardware check):
  - It assumes 5-byte `0x55|type|lo|hi|sum` packets (`wit_node.py:112-140`, `docs/wit_imu.md`).
  - The standard WIT protocol uses 11-byte frames with 4 int16 values plus a sum. The vendor config script uses standard `FF AA` WIT commands, which matches the standard protocol.
  - `all(self.angle.values())` (`wit_node.py:170`) never fires while any axis reads exactly 0.
- `mower_coverage/CMakeLists.txt` calls `ament_package()` before the `BUILD_TESTING` block (minor).

**Stale claims in docs and agent logs:**
- README and `handoff_gap_analysis.md` say wit has 25 tests; it has none.
- `nav2.launch.py:95-115` and `localization_control_plan.md` §3 still describe the orientation-is-zero blocker. `wit_node.py:205-219` now fills orientation; the real problems are the parser and the non-absolute yaw.
- `agent14` claims 58 ported gtests; the file has 15.
- `handoff_gap_analysis.md` describes 8 packages and an empty `config/`; both are out of date.

**Hardcoded paths:**
- `/home/airseekers/ros2_stack` (`mower-ros2.service.example:9-10`).
- `file:///work/config/cameras/…` (`cameras.launch.py:30`).
- `/userdata/ros2/calibration` (`stereo_vio_bridge.py:162`, not mounted).
- `/dev/video11` in three places, against `/dev/video22` in `metoak_sdk.py:80`.
- Relative `model/*.rknn` paths in the det/seg YAML.
- **`src/open_vins` → `../../ros2_port_handoff/...`** resolves to `/ros2_port_handoff` inside the container (compose mounts only `ros2_stack` at `/work`), so it is dangling there.
- `install_udev.sh:8` looks for `config/cameras/99-mower-cameras.rules`; the file is in `scripts/`, so the camera rules are never installed.

**TODO/FIXME counts:**
- `mower_mcu_driver` 15, `base_ble` 3, `mower_localization` 2.
- Launch files 1, `config/` 7 (calibration TODOs in the URDF).
- All other packages 0. The real gaps there are documented in prose (e.g. the "Known gaps" docstring in `mcu_node.py`) rather than as TODOs.

**Nodes with no tests:** `wit_node`, `cmd_vel_slew`, `slip_detector`, `imu_cal`, `bumper_controller`, `base_keys`, `seg_ros`, `mower_cameras`, `mower_rknn`.
- `base_ble` tests cover only framing, never the node or its imports.
- `det_ros` tests cover only post-processing.
- Every Python test runs against hand-written ROS stubs; none runs with real rclpy or real message definitions. That is exactly why the constant bugs above slipped through.

**Uncertain:** the protobuf 3.12 incompatibility and Nav2 Humble's exact fatal-on-missing-plugin behaviour come from my knowledge of those versions; I did not reproduce them, because there is no ROS install on this host.
