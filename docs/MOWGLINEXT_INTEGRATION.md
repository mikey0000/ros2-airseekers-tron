# Upstreaming the Airseekers Tron port into MowgliNext: a plan

Status: plan only, 2026-10-13. Nothing here has been offered upstream yet except PR #1.

**Purpose.** Turn the fork (`ros2_stack/`, branch `migration/jazzy`) into a series of small
upstream PRs against MowgliNext `dev`. The end state is a fork that holds only what cannot be
published.

**Baselines read for this plan:**

- Upstream: `upstream-pr/mowglinext` at `origin/dev` = `be927ebb`. The checkout is on
  `feat/custom-map-tiles` (dev + 3 GUI commits).
- Ours: `ros2_stack/src/*` and the GUI fork `ros2_stack/third_party/mowglinext/gui`. The GUI fork
  is based on upstream `main` @ `faf658b` (`third_party/mowglinext/UPSTREAM_COMMIT`), so it is
  far behind `dev`. Against `dev` it has 238 changed files, about 150 files of its own, and is
  missing 204 files that `dev` has.

**Path convention.** Upstream paths are relative to the MowgliNext repo root. Ours are relative to
`ros2_stack/`. "Not checked" means I did not read the code.

**Earlier docs.** `docs/mowglinext_handoff.md` describes the old "two containers side by side"
idea. This plan supersedes it as the route to *upstream*.

---

## 1. The key finding: upstream already has a hardware-plugin boundary

MowgliNext `dev` is no longer STM32-only. It has a **hardware backend** abstraction, and our
port should land as one more backend. A port that runs beside the stack is not the right shape.

| Upstream mechanism | Where |
|---|---|
| `HARDWARE_BACKEND` env → `hardware_backend` launch arg. Only `mowgli` starts the in-stack `hardware_bridge_node`. Every other backend runs a **sidecar container** that publishes the same `/hardware_bridge` contract. | `ros2/src/mowgli_bringup/launch/mowgli.launch.py:59-72, 233-240` (`SUPPORTED_HARDWARE_BACKENDS = ("mowgli","mavros","openmower")`) |
| Reference sidecar: OpenMower v1. It talks to stock firmware that is never reflashed, which is exactly our situation. | `sensors/openmower/` (C++ `mowgli_openmower_bridge`, `README.md`, `sim/openmower_v1_emulator.py`, CI `sensors-openmower.yml`) |
| Per-backend **default overrides**. A backend may replace defaults but never add keys. The layering is template ← backend ← sparse installed file. | `ros2/src/mowgli_bringup/config/backends/openmower.yaml`, `launch/robot_config_util.py:465-495`, `gui/asserts/backend_defaults.json` (`go run ./cmd/gen-backend-defaults`) |
| Compose modules chosen by the installer | `install/compose/stack.json` (`options.backend`), `install/compose/docker-compose.openmower.yml` |
| GUI backend gating and per-model presets | `gui/web/src/constants/hardwareBackends.ts` (`appliesToBackend`), `gui/web/src/constants/mowerModels.ts` (`MOWER_MODELS`, guarded by `model_preset_parity_test.go`) |
| The contract itself | `docs/claude/ros-interfaces.md`: `/hardware_bridge/{status,emergency,power,dig_escalated,cmd_vel_applied}`, `/battery_state`, `/imu/data`, `/wheel_odom`, `/wheel_ticks`, services `mower_control`, `emergency_stop`, `reboot_board`, `set_firmware_debug`, `/cmd_vel` **TwistStamped** |

**So the Tron becomes `HARDWARE_BACKEND=airseekers`.** This shrinks the integration problem a lot.

- **Above the bridge:** fusion_graph, Nav2/FTC, the BT, `mowgli_map`, `mowgli_coverage`,
  opennav_docking and the GUI run upstream's code unchanged. Our parallel versions of all of these
  get retired, not upstreamed.
- **Below the bridge:** our drivers become one sidecar.
- **Additions:** what upstream lacks entirely, chiefly camera perception, goes in as optional
  features.

## 2. Hard conflicts with upstream invariants (root `CLAUDE.md`)

These hold no matter what gets upstreamed. The fork has to move to upstream's side before any
backend PR makes sense.

| # | Upstream rule | Fork today | Resolution |
|---|---|---|---|
| C1 | Inv. 1/2 and "What NOT to do": `fusion_graph` is the **sole** localizer and owns `map→odom` and `odom→base_footprint`. robot_localization is removed and must not come back. | `mower_localization`: dual EKF (`ekf_node` + `ekf_map`), `navsat_transform`, `gps_gate`, `heading_aligner`. EKF is the default. | Adopt fusion_graph. Our EKF stack stays fork-only during the transition and is then deleted. The old concern was GTSAM on 4 GB aarch64. Upstream's own reference board looks like an RK3588S Orange Pi 5B (per `ros2/src/mowgli_leds/README.md`), so this should be measurable. **Measure CPU/RAM on the Tron first.** |
| C2 | Inv. 3: Cyclone DDS, never FastRTPS | `docker/docker-compose.yml:25` `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | Switch to Cyclone. Upstream compose files mount `cyclonedds.xml`. |
| C3 | `/cmd_vel` and all twist_mux lanes are `TwistStamped` (`twist_mux.yaml use_stamped: true`). There is no `locks:` block unless the bridge publishes the lock. | Unstamped `Twist` end to end (`src/mower_teleop/config/twist_mux.yaml use_stamped: false`, `locks: estop /estop`). | The sidecar subscribes to `TwistStamped`. Our lanes go away with our nodes. The `/estop` lock is dropped: e-stop goes through `/hardware_bridge/emergency_stop`. |
| C4 | Inv. 2 / REP-105: Nav2 uses `base_footprint`. `base_link` sits at the rear axle. | `src/mower_navigation/config/nav2_params.yaml:31,48`: `robot_base_frame: base_link` (a documented deviation). The axle convention already matches. | Retired together with our Nav2 params. |
| C5 | Inv. 15 / `gui/CLAUDE.md:4`: no defaults in the GUI. Defaults live in the ROS template and are mirrored into `mower_config.schema.json`. | The GUI fork's `web/src/constants/robotProfiles.ts` hard-codes `DEFAULT_TELEOP {0.25, 0.6}`, camera topics, battery behaviour and feature flags. `param_bindings.go` pushes settings to nodes that do not read `mowgli_robot.yaml`. | Move the values into `backends/airseekers.yaml` and template keys. Feature flags become backend capabilities (P5). `param_bindings` is retired once our nodes read the merged config through `robot_config_util`, the same way the OpenMower bridge mounts `/config:ro`. |
| C6 | Safety: STM32 firmware is the blade authority. On a backend whose firmware has no such policy, the bridge is in the actuation path (`blade_policy.hpp`, kill test in `sensors/openmower/sim`). | The vendor AT32 latches lift. Our host-side interlock is split across `mower_mcu_driver` (lift/stop interlock, e-stop latch) and `mower_gui_bridge` (blade only in HL 2/4). | The sidecar must implement the OpenMower safety model: zero on emergency, stale link or stale `cmd_vel`; blade only in AUTONOMOUS/MANUAL_MOWING with a fresh `HighLevelStatus`; SIGKILL test against the emulator. The **heartbeat-failsafe measurement** (`mowglinext_handoff.md` §10 #2) is a prerequisite. |
| C7 | Inv. 16: dig detection lives in the hardware bridge and is GNSS-anchored. | `mower_control/slip_detector` publishes `/dig_stall` (commanded vs measured, RTK-gated). | Port into the sidecar by reusing upstream's pure `mowgli_hardware/include/mowgli_hardware/dig_detector.hpp`. That needs P6. Until then the sidecar publishes `dig_escalated=false` as OpenMower does. |
| C8 | `HighLevelStatus` codes 0-4 and BT-owned `state_name`. The `behavior_tree_node` name is reserved. | `mower_mission` impersonates `behavior_tree_node` with its own state set. | Adopt `mowgli_behavior`. Our mission is retired (§3). |
| C9 | ROS 2 **Lyrical** / Ubuntu 26.04 images. Installer and updater own the host (compose, systemd, `/boot` edits). | Jazzy, and the mower host is untouchable (no host package changes, `DOCKER_BUILDKIT=0`, `fuse-overlayfs`, no `CONFIG_POSIX_MQUEUE`, root fs full). | Two open questions: (a) whether upstream's Lyrical images run on the vendor 20.04 kernel — **not checked**; (b) the installer needs a "no host changes" mode for a locked-down host. Raise as an upstream issue before the backend PRs. |
| C10 | Message types bit-identical | `src/mowgli_interfaces` is a stale `faf658b` copy. `Status`, `MapArea`, `MapObstacleInfo`, `PlanCoverage`, `DigEvent` and others have drifted. We added `srv/RemoveObstacle.srv`. | Fork-side step F1: replace it with dev's copy. Upstream `RemoveObstacle` only if `mowgli_map` needs it (it does not today). |

## 3. Classification of every fork package

Key: **(a)** generic, upstreamable; **(b)** hardware, goes into the Airseekers backend sidecar;
**(c)** replaces something upstream has; **(d)** stays in the fork.

| Package (ours) | Class | Upstream landing place / decision | Parameterise / notes |
|---|---|---|---|
| `mower_mcu_driver` | **b** | `sensors/airseekers/` sidecar, node name `hardware_bridge`. It publishes the contract directly, which retires `mower_gui_bridge`'s projection. | Serial port, wheel track, clamps (0.3 m/s, 0.3 rad/s) → `airseekers_*` keys or `backends/airseekers.yaml` defaults. **Licence check:** the A5…5A protocol was recovered from vendor builds, and the handoff §9 excludes "protocol material recovered from vendor builds". Confirm that the clean-room driver may be published. Language: upstream sidecars are C++, and the reusable safety core (P6) is C++. Recommend porting the framing (`mower_mcu_driver/*protocol*`) to C++ and keeping the Python version as the test oracle. **Decision needed.** |
| `wit_imu_driver` | **b** | Same sidecar; publishes `/imu/data`. | Port and baud as backend keys. No TF (already `publish_tf:=false`). |
| `um960_gps_driver` | **c → b** | Upstream GNSS is the Universal GNSS sidecar, and Unicore support exists (`sensors/gps/universal_gnss_topic_bridge.py:92`, UM980/UM982 profiles). The UM960 sits **behind an AT32 `rtk_rover` board** that switches between LoRa and network RTK (`docs/um960.md:12,212`), and universal-gnss cannot drive that board. | Short term: the sidecar publishes `/gps/fix` (raw NavSatFix) and `/gps/status` (GnssStatus), with `GNSS_STACK=none`. Long term: a universal-gnss UM960 profile plus a pre-config hook for the rover board. UM960 support in universal-gnss: **not checked** (submodule not checked out). |
| `base_keys` | **b** | Sidecar, using OpenMower's panel mapping: buttons → `high_level_control`, long press → reset emergency. | Power-off helper stays fork-only (host script). |
| `bumper_controller` | **b / d** | It publishes a twist_mux lane, and upstream lanes are fixed in `mowgli_bringup/config/twist_mux.yaml`. The bridge layer "only REDUCES motion" (Don't-list, `EscapeStartBlocked`). | **Upstream first:** decide whether bumper back-off is a BT recovery (preferred: bumper → `Emergency`/`Status` bit, BT backs up on `/cmd_vel_nav`) or a backend lane. Until then it stays in the fork. |
| `mower_lights` (+ `_interfaces`) | **c** | `ros2/src/mowgli_leds` already drives a WS2812 over SPI from `HighLevelStatus`. | Merge features: per-robot device, LED count, layout and a vendor pattern table. Whether `mowgli_leds` has those knobs: **not checked**. |
| `mower_control` | split | `cmd_vel_slew` → **c**: upstream has a merged-command slew in the bridge (`mowgli_hardware/cmd_vel_slew.hpp`); reuse it in the sidecar. `slip_detector` → **b** via C7. `imu_cal` → **b**: copy OpenMower's `imu_bias_estimator.hpp` (dock-armed). `tilt_monitor` → **b**: the vendor firmware has no tilt trip, so it becomes a bridge emergency source. `supervisor`, `crash_record`, `blackbox` → **d**. `bag_recorder` (rolling ring plus incident pinning) → **a**, later (P11). | Straight-driving trim, deadband and heading hold compensate for the vendor MCU, so they are backend keys. |
| `mower_gui_bridge` | **c** (retire) | Its contract projection moves into the sidecar. `set_datum` duplicates `navsat_to_absolute_pose`. The `/gui/*` low-rate copies exist only for the GUI fork. | None. |
| `mower_teleop` | **c** (drop) | Upstream `mowgli_bringup/scripts/cmd_vel_ws_relay.py` (ours is a copy). | None. |
| `mower_localization` | **c** (adopt theirs) | `fusion_graph` + `cog_to_imu_node` + `navsat_to_absolute_pose`. | Ideas worth offering later as **a**: `localization_monitor` drift budget since last RTK-FIXED with BT hold/resume (P12), and `rtk_quality` covariance classes. Each must be argued against fusion_graph's own gates; upstream's Don't-list is strict about GPS gates. |
| `mower_navigation` | **c** | `nav2_params_base.yaml` + overlays. | Tron-specific tuning (yaw ≤ 0.3 rad/s, FTC `speed_angular` 15°/s, `src/mowgli_nav2_plugins/config/ftc_tron.yaml`) should become template keys (P3) plus `backends/airseekers.yaml` defaults, not a Nav2 overlay. |
| `mowgli_nav2_plugins` (ours) | **c + a** | Ours is a Jazzy backport of an old FTC. Upstream's FTC has since gained the lattice, turn fallback and authorized transit. Adopt theirs. | Offer `scale_linear_on_angular_saturation` (`src/ftc_controller.cpp:246,571`, absent upstream) and `test/test_ftc_tracking_sim.cpp` (closed loop with a yaw-rate-limited unicycle) as P2. `LegGoalChecker` goes away along with our route graph. |
| `mower_map` | **c** (adopt `mowgli_map`) | `map_server_node` (C++). It already has navigation areas, obstacle tracker, proposals and the datum migration. | Possible later **a**: `route_graph.py` transit along drawn paths. Upstream has `AuthorizedTransitPlanner`, so compare the two first (not checked in depth). `nongrass` memory layer (needs seg). `import_vendor_geojson.py` becomes a GUI importer like `gui/pkg/api/openmower_import.go`; that one is **b**, a vendor format. |
| `mower_coverage`, `mower_coverage_bridge` | **c** (drop) | `mowgli_coverage`. Upstream pins F2C 3.0 and forbids any other server. Ours is F2C 2.1. | None. |
| `mower_mission` | **c** (adopt `mowgli_behavior`) | BehaviorTree.CPP v4. Ours is a Python port of an older tree. | Features worth upstreaming as BT nodes once the backend runs: localization hold (P12), tilt caution/limit (from sidecar `Status`), and obstacle-policy wait/resume. The last overlaps with `ftc_obstacle_wait.hpp`, so check first. The vendor cutter-start check and retries belong in the sidecar's blade policy, not the BT. |
| `mower_docking` | **c + a** | Upstream uses `opennav_docking` `SimpleChargingDock`: forward, GPS dock pose, BackUp undock. | The Tron docks **in reverse on an ArUco marker**. Use `dock_backwards: true` plus SimpleChargingDock external detection fed by a small generic `dock_marker_detector` node (rear camera → `detected_dock_pose`), offered as P10. Rear-corridor hold (`/vision/rear_blocked`) comes later with perception. `is_docking_done` → `Status.is_charging` in the sidecar (Inv. 12). |
| `mower_cameras` | **b** | Optional `sensors/airseekers-cameras` (v4l2/rkisp OA cams, rear cam, Metoak stereo + depth). | Device paths are backend keys. Needs P7 (URDF and observation-source extension points). |
| `mower_rknn`, `det_ros`, `det_ros_cpp`, `seg_ros` | **b** (opt-in, RK3588-only) | Optional perception sidecar. **Models are proprietary vendor exports and cannot ship**, so upstream gets "bring your own `.rknn`". | Model paths, classes, NPU core → config. Upstream has **no camera perception at all** (grep for `vision_msgs`/`Detection2D` under `ros2/src` and `gui/pkg` finds nothing). |
| `det_range`, `mower_vision` (`obstacle_guard`) | **a** (later) | `det_range` (detections + depth → points) is generic. It would feed the local costmap / collision_monitor as one more observation source (P7). | `obstacle_guard`'s emergency burst plus `/cutter_off` overlaps collision_monitor and the bridge emergency path. Re-cut it as "person in corridor → `emergency_stop`" or drop it. |
| `stereo_vio_bridge`, OpenVINS | **d** | fusion_graph could take a VIO factor some day. Not now. | None. |
| `mower_alerts` | **a** | Theft-geofence, lift and incident rules feeding the GUI notifier. Upstream `gui/pkg/providers/notify_*.go` exists but has no `theft` kind (grep). | Land as GUI provider rules, or in `mowgli_monitoring`. Theft needs the dock geofence from `mowgli_robot.yaml`. |
| `mower_sim` | **d** (derive) | Upstream sim is Webots plus a kinematic `fake_hardware_bridge`. | For CI, write a **wire-level** Airseekers MCU emulator on a pty, modelled on `sensors/openmower/sim/openmower_v1_emulator.py`. Reuse `mower_sim/kinematics.py`. |
| `mower_bringup` | **b** (shrinks) | `sensors/airseekers/launch/airseekers_bridge.launch.py`, `install/compose/docker-compose.airseekers.yml`, `stack.json` option, `backends/airseekers.yaml`, a `MOWER_MODELS` "AirseekersTron" preset. | `config/urdf/mower.urdf.xacro` camera frames need P7. Everything else in `mower.launch.py` dies. |
| `mower_interfaces`, `mower_proto`, `base_ble` | **d** | Vendor contract and vendor protobuf bindings (not redistributable). | `mower_interfaces` may stay **sidecar-internal**, but it must not appear on the public ROS graph. |
| `mowgli_interfaces` (our copy) | **c** | Replace with dev's (C10). | None. |
| GUI fork (`third_party/mowglinext/gui`) | split | **a**, generic: `/rosout` log stream (`pkg/api/rosout.go`, `web/src/pages/rosoutLog.ts`; upstream has no `rosout` anywhere in `gui/`), WS heartbeat/reconnect (`pkg/api/ws_heartbeat.go`, `hooks/reconnect.ts`), `SafeDrawPolygonMode`, polygon rotation handle (upstream `en.json` has one rotation string; check), path tool, mission-progress/why-stopped panels, cutter-height control (if a `Status` or `mower_control` field backs it upstream; **not checked**). **b**: Perception page, cameras proxy (`pkg/api/cameras.go`), GNSS correction-source and LoRa cards, vision-dock card, AirseekersTron model preset. **c**: `profileGates.ts`/`robotProfiles.ts` → upstream's backend gating plus P5 capabilities. `param_bindings.go` → retire (C5). | Every GUI PR has to be **re-cut on `dev`**. The fork base is `faf658b`, so cherry-picks will not apply. |
| PR #1, custom map tiles | **a** | Branch `feat/custom-map-tiles` (3 commits, +1922 lines incl. Go and vitest tests). | Ready. |

## 4. What upstream has to change first (plugin points)

| Id | Upstream change | Why the Tron needs it | Generic value |
|---|---|---|---|
| U1 | Add `airseekers` to `SUPPORTED_HARDWARE_BACKENDS` (`mowgli.launch.py:62`), `normalizeHardwareBackend` (`hardwareBackends.ts`), `stack.json` `options.backend`, and the installer menu. | Backend selection | Pattern already exists. Trivial once the sidecar exists. |
| U2 | **Shared bridge core.** Move the backend-independent halves of `mowgli_openmower_bridge` (`blade_policy.hpp`, `emergency_tracker.hpp`, `power_semantics.hpp`, `imu_bias_estimator.hpp`) and `mowgli_hardware` (`dig_detector.hpp`, `dig_escalation.hpp`, `cmd_vel_slew.hpp`, `cmd_vel_validation.hpp`) into one header-only package, e.g. `ros2/src/mowgli_bridge_core`. | Gives the sidecar the same safety semantics without copying files. | OpenMower's README already plans this reuse for v2. It also gives OpenMower dig detection, which it lacks today. |
| U3 | **Backend capabilities.** `backends/<name>.yaml` (or a sibling `capabilities`) declares which features a backend has: stm32_firmware, lidar, cameras, dock kind, drive tuning, and so on. `/settings/hardware-backend` returns them and the GUI gates on them. | Replaces the fork's `profileGates.ts` without GUI-side defaults (C5). | Replaces the scattered `appliesToBackend` lists. |
| U4 | **URDF extension.** An `extra_urdf` xacro include (path from backend config) for sensor frames the template does not model. | Camera frames for perception and the docking marker | Any accessory sensor |
| U5 | **Extra observation sources.** A config-listed PointCloud2 source for local costmap and collision_monitor, alongside the LiDAR. | Stereo depth and `det_range` points | Any depth camera |
| U6 | **Robot yaw-rate cap.** A template key (`max_angular_vel`) injected into FTC (`max_cmd_vel_ang`, `speed_angular`), RPP and docking `v_angular_max`. | Tron MCU clamps at 0.3 rad/s. Live 10-06: the FTC carrot ran 0.54 m off the ring. | Every slow-turning chassis. Upstream already warns about it (`robot_config_util.py:757-772`). |
| U7 | **Installer "locked host" mode.** No `/boot` or systemd edits, no host updater, compose only. | The vendor host must not be modified. | Other closed-OS mowers |
| U8 | Bumper policy (§3 `bumper_controller`) | Bumper back-off | Mowers with a host-visible bumper |

## 5. Proposed PR series

Size is the rough diff including tests. "CI" names the gating job from upstream
`docs/claude/testing-ci.md`. Fork-side steps (F*) are not PRs, but they come first.

**Fork-side prerequisites (no upstream PR):**

| Step | What |
|---|---|
| F1 | Sync `mowgli_interfaces` to `dev` |
| F2 | Cyclone DDS |
| F3 | Measure fusion_graph + Nav2 + BT CPU/RAM on the Tron (bench, no motion) |
| F4 | Heartbeat-failsafe measurement (handoff §10 #2) |
| F5 | Licence clearance for the MCU protocol driver |

**Upstream PRs:**

| # | Title | Scope | Size | Depends on | CI / tests |
|---|---|---|---|---|---|
| **1** | GUI: custom XYZ/MBTiles map tiles | `gui/pkg/api/imagery.go`, `web/src/hooks/useImagery.ts`, `ImageryLayers/Panel.tsx`, i18n, `wiki/GUI.md` | ~1.9k (done) | none | `gui-ci.yml` (tsc, lint, vitest), Go `go test ./...` (`imagery_test.go`) |
| **2** | FTC: keep curvature when angular saturates, plus a yaw-limited closed-loop tracking test | `mowgli_nav2_plugins` `ftc_controller.cpp` (`scale_linear_on_angular_saturation`, default **false**: no behaviour change), new gtest from our `test_ftc_tracking_sim.cpp`, re-based on the lattice FTC | ~250 | none | `ros2-ci.yml` build-and-test (gtest; use `ament_add_ros_isolated_gtest` if it inits rclcpp), clang-format 18. **Safety-critical label** (motion). |
| **3** | Robot `max_angular_vel` cap (U6) | `mowgli_bringup/config/mowgli_robot.yaml` template key, `gui/asserts/mower_config.schema.json`, `navigation.launch.py` injection and clamp, docs `parameters.md` | ~200 | none (pairs with 2) | `test_launch_injection.py`, `test_nav2_params.py`, `schema_template_parity_test.go` |
| **4** | GUI: `/rosout` log stream on the Logs page | `gui/pkg/api/rosout.go` + test, `pages/rosoutLog.ts` + test, Logs page tab. Useful for any backend without a docker socket. | ~600 | none | gui-ci + Go tests. Re-cut on `dev`'s `LogsPage.tsx`/`logBatcher.ts`. |
| **5** | Backend capabilities (U3) | `backends/*.yaml` capability block, `settings_backend.go`, `hardwareBackends.ts`, replace ad-hoc `appliesToBackend` uses | ~500 | none | Go + vitest. `backend_defaults` generator parity test. |
| 6 | Shared bridge core (U2) | New `ros2/src/mowgli_bridge_core`. `mowgli_hardware` and `mowgli_openmower_bridge` include from it, no logic change. | ~400 moved | none | `ros2-ci.yml`, `sensors-openmower.yml` (test + simulate) unchanged green |
| 7 | URDF and observation-source extension points (U4, U5) | `mowgli.urdf.xacro` `extra_urdf` arg, `navigation.launch.py` extra sources, defaults empty | ~250 | none | `test_urdf_xacro.py`, `test_tf_ownership.py`, `test_nav2_params.py` |
| 8 | GUI generic batch(es) | WS heartbeat/reconnect, SafeDrawPolygonMode, rotation handle, path tool. **One PR each.** | 150-600 each | none | gui-ci |
| 9 | **Airseekers backend** (split into 9a-9d) | 9a: protocol lib + pty emulator + unit tests (no ROS). 9b: `airseekers_bridge_node` on `mowgli_bridge_core` (contract, blade policy, emergency, tilt, IMU, panel keys, dig) + `backends/airseekers.yaml`. 9c: compose module, `stack.json`, installer (U1, U7), `MOWER_MODELS` preset, GUI enum. 9d: GNSS (UM960 + rover-board mode) as sidecar output or universal-gnss profile. | 9a ~1.5k, 9b ~2.5k, 9c ~600, 9d ~1.5k | 5, 6 (+ F1-F5) | New `sensors-airseekers.yml` via `_sensor-docker.yml`: build, smoke, emulator **simulate** job including the SIGKILL watchdog test (mirror `sensors-openmower.yml`), installer bash suite (not CI-gated; run by hand). **Safety-critical.** |
| 10 | Marker docking detection for opennav_docking | Generic `dock_marker_detector` (ArUco/AprilTag → `detected_dock_pose`), `dock_backwards` per backend default | ~800 | 7 | gtest/pytest on synthetic images, `test_nav2_params.py` |
| 11 | Rolling bag ring + incident pinning | From `mower_control/bag_recorder.py`, `bag_ring.py`, as `mowgli_monitoring` or a `tools/` sidecar; GUI Diagnostics hook | ~900 | none | pytest; check overlap with `gui/pkg/api/rosbag.go` first |
| 12 | BT: localization-drift hold | Condition node from `localization_monitor`'s drift budget, gated on `/gps/status`, not on graph covariance (Don't-list) | ~400 | F3 | `mowgli_behavior` gtests |
| 13 | Alerts: theft geofence and lift | GUI notifier kinds + rules from `mower_alerts/rules.py` | ~700 | none | Go tests |
| 14+ | Perception (cameras, det, det_range, Perception page) | Optional RK3588 sidecar with BYO models, gated by capability `cameras` | large | 5, 7 | sensor-image smoke; no model in CI |

**Fork shrink path.** After 9b runs on the Tron, delete these from `ros2_stack/src` together with
`mower.launch.py`:

- `mower_gui_bridge`, `mower_teleop`, `mower_coverage*`
- `mower_navigation`, `mower_map`, `mower_mission`, `mower_docking`, `mower_localization`
- our `mowgli_*` copies
- the GUI fork

The fork would then be:

- `mower_proto` and `base_ble` (vendor bindings)
- the power-off host helper
- VIO
- the NPU model install
- sidecar overrides not yet upstreamed (bumper, perception)

## 6. Main blockers, in order

1. **Localizer swap (C1).** Nothing above the bridge can be upstream's until the Tron runs
   fusion_graph. GPS-only mode exists (`nav2_params_no_lidar.yaml`, `lidar_enabled: false`). The
   open question is the JY61P (no magnetometer, drifting yaw) with COG only. Measure (F3) before
   writing any backend code.
2. **Distro and host (C9).** Lyrical / 26.04 images on the vendor 20.04 kernel with
   `DOCKER_BUILDKIT=0`, fuse-overlayfs, no mqueue and a full root fs. Upstream's installer assumes
   it may modify the host. **Not checked**; needs a bench try of upstream's published images and
   an upstream issue (U7).
3. **Safety model and licence for the MCU driver (C6, F4, F5).** The sidecar is in the blade
   actuation path. The heartbeat failsafe is unmeasured. Publishing the reverse-engineered
   protocol needs a clear yes.
4. **GUI fork divergence.** It is based on `main@faf658b`, with 238 changed files and 204 dev
   files missing. Every GUI PR is a re-implementation on `dev`, not a cherry-pick. Prioritise the
   generic ones (4, 8) and let backend gating (5) replace `profileGates`.
5. **No-GUI-defaults (C5)** blocks upstreaming the AirseekersTron profile as written: teleop caps,
   camera topics and battery handling must come from ROS config.
6. **Bumper (U8)** and **perception** have no upstream home yet. They need a design discussion
   (issue), not a PR.
