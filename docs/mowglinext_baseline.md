# MowgliNext — research baseline for the Airseekers-Tron ROS 2 port

Purpose: everything the four driver packages we are about to build
(`mower_interfaces`, `mower_mcu_driver`, `wit_imu_driver`, `um960_gps_driver`) should
inherit, copy or deliberately *not* copy from the MowgliNext reference stack.

Research date: 2026-10-05. All upstream claims below were read from `main` at the time of
fetching unless a commit/tag is named; upstream `ros-interfaces.md` states it was generated
2026-09-03 at `f21729e9`.

---

## 1. Repo identity

| Field | Value |
|---|---|
| Canonical URL | **https://github.com/mowglinext/mowglinext** |
| Owner | GitHub **organisation** `mowglinext` (id 299422744), created 2026-04-03 |
| Homepage / docs site | https://mowgli.garden/ |
| Clone | `git clone --recurse-submodules https://github.com/mowglinext/mowglinext.git` |
| Default branch | `main` |
| Latest release | `v1.4.0` (`4fcabb3`); tags `v1.0.2 … v1.4.0` |
| Primary language | C++ (also Go GUI, TypeScript/React web, Python, C firmware, shell) |
| Stars / forks / open issues | 41 / 29 / 78 at fetch time — first public beta |
| Licence | **Dual**: GPLv3 for open/personal/educational/non-profit use, **separate commercial licence required** for any commercial use (`contact@mowgli.garden`). SPDX `NOASSERTION` because of the dual licence. |
| Topics | `autonomous behavior-tree docker lidar nav2 openmower robot-mower ros2 rtk-gps slam` |
| ROS distro | **ROS 2 "Lyrical" / Ubuntu 26.04** (rebuilt in v1.4.0). **Not** Humble. |
| Target compute | ARM64 SBCs: RK3566/RK3588, Raspberry Pi 4/5. Webots sim image is amd64-only. |

### 1.1 The path in the brief does not exist

`https://github.com/ClemensElflein/mowglinext` returns **404**. ClemensElflein is the
OpenMower author (`openmower.de`), and MowgliNext explicitly credits
`https://github.com/ClemensElflein/open_mower_ros` as its inspiration — that is a different
project, and `mowgli.hardware` / `mowgli_bringup` even descend from it (see
`docs/claude/codemaps/mowgli_hardware.md`: “`ll_datatypes.hpp` is a port of OpenMower's
`ll_datatypes.h`”). GitHub repo search for `mowglinext` returns exactly one canonical repo
plus community forks (`cedbossneo/mowgli-ros2`, archived → moved to MowgliNext;
`arcidodo/mowglinext-ha`, `juditech3D/MowgliNext-ha-bridge`).

**Use `mowglinext/mowglinext`.** Related upstream used by the GNSS stack:
`https://github.com/Pepeuch/universal-gnss` (vendored as a git submodule).

### 1.2 Monorepo layout

| Directory | Contents |
|---|---|
| `ros2/` | **The ROS 2 stack we care about** — 12 colcon packages under `ros2/src/`, plus `ros2/scripts/`, `ros2/foxglove/`, `ros2/systemd/mowgli.service` |
| `gui/` | React + Go web interface (532 files) |
| `firmware/` | STM32 motor control / IMU / blade safety (718 files, `firmware/stm32/`) |
| `install/` | Interactive installer, hardware presets, modular compose fragments (83 files) |
| `docker/` | Generated/manual compose, DDS settings, orchestration |
| `sensors/` | Dockerised **GNSS sidecar** + 3 LiDAR containers |
| `docs/` | mowgli.garden GitHub Pages + `docs/claude/` agent reference index |
| `tools/motor/` | `mowgli_tools` (drive-PID tuner) copied into the runtime image |

---

## 2. ROS 2 package list (the baseline we model)

Twelve packages in `ros2/src/`:

| Package | Executables | Role | Maps to our batch? |
|---|---|---|---|
| `mowgli_interfaces` | — | 16 `.msg`, 15 `.srv`, 3 `.action` on disk (README says 15/14/3) + shared C++ helpers (`gnss_status_utils.hpp`, `wgs84_projection.hpp`, `motion_yaw_fit.hpp`, …) | **`mower_interfaces`** |
| `mowgli_hardware` | `hardware_bridge_node` | The single ROS↔STM32 boundary: serial + COBS/CRC16 framing, IMU + wheel odom + status publishing, `cmd_vel` in, blade/e-stop out, runtime PID push, at-rest IMU bias cal, wheel-slip “dig” detector | **`mower_mcu_driver`** (partly `wit_imu_driver`) |
| `mowgli_bringup` | `empty_static_map_pub.py`, `cmd_vel_ws_relay.py` | Launch, Nav2 params, URDF/xacro, `twist_mux`, `mowgli_robot.yaml` template | later `mower_bringup` |
| `mowgli_localization` | `wheel_odometry_node`, `navsat_to_absolute_pose_node`, `localization_monitor_node`, `cog_to_imu`, `mag_yaw_publisher`, `calibrate_imu_yaw_node`, `scan_deskew_node`, `costmap_scan_filter_node`, `gps_dock_detection_node` | GPS→ENU pose, mode monitor, yaw helpers, deskew | later `mower_localization` |
| `fusion_graph` | `fusion_graph_node` | **Sole localizer** — GTSAM iSAM2 Pose2 factor graph owning both `map→odom` and `odom→base_footprint` | later (replaces EKF) |
| `mowgli_behavior` | `behavior_tree_node` | BehaviorTree.CPP v4 mission executor (`main_tree.xml`) | later `mower_behavior` |
| `mowgli_coverage` | `mowgli_coverage` (node `coverage_server`) | Fields2Cover v3 headland+swath planner, `plan_coverage` action | later `mower_coverage` |
| `mowgli_map` | `map_server_node`, `obstacle_tracker_node` | GridMap areas, keepout mask, mow progress, obstacle promotion | later `mower_map` |
| `mowgli_nav2_plugins` | — (plugin lib) | `FTCController` (follow-the-carrot coverage controller) + `PathProgressGoalChecker` | later `mower_nav2_plugins` |
| `mowgli_monitoring` | `diagnostics_node`, `mqtt_bridge_node` | 9-status `/diagnostics` aggregator, MQTT bridge | later `mower_monitoring` |
| `mowgli_leds` | `led_ring_node` | Optional WS2812 ring over SPI, off by default | later (Airseekers has `libws_2812.so`) |
| `mowgli_simulation` | `fake_hardware_bridge_node`, `sim_actuation_node` (+ py helpers) | Webots world + fake hardware bridge + sensor-degradation sims | later `mower_simulation` |

Out-of-tree packages the runtime image still ships:

- `ros2/src/external/universal-gnss` — git submodule, universal GNSS runtime (u-blox F9P,
  Unicore UM98x, generic NMEA). Migrating to a dedicated `mowgli-gps` sidecar.
- `opennav_coverage` — git submodule, linked as `opennav_coverage_msgs` (action defs only
  by default); its server/BT/navigator packages are optional and need Fields2Cover.
- `mowgli_gnss_bridge` (`sensors/gps/`) — adapter layer converting the Universal GNSS
  internals into the public topic contract.
- `mowgli_tools` (`tools/motor/`) — GUI invokes `ros2 run mowgli_tools tune_drive_pid`
  *inside* the running container, so it must be in the runtime workspace.
- LiDAR containers: `sensors/lidar-ldlidar`, `sensors/lidar-rplidar`, `sensors/lidar-stl27l`
  (exactly one composed, selected by `LIDAR_TYPE`).

---

## 3. Architecture

```
MowgliNext GUI (Go backend = Foxglove client + React)
        │ Foxglove WS :8765  +  teleop relay :8766
behavior_tree_node (BehaviorTree.CPP v4, main_tree.xml)
   guards: Emergency → Sensors → Boundary → Localization → GPS mode → Nav2 resume
   actions: Undock → PlanCoverage → Transit → Mow → Dock (+resume)
        │
        ├── map_server / obstacle_tracker (GridMap, keepout mask, mow progress)
        └── Nav2  (SmacPlanner2D, RPP transit, FTC coverage, RotationShim,
                  collision_monitor, keepout_filter, docking_server)
                    │
             fusion_graph_node  (GTSAM iSAM2 — sole map+odom localizer)
                    │  /gps/fix + /imu/data + /wheel_odom → Pose2 factor graph
             hardware_bridge_node  (COBS+CRC16 over USB serial)
                    │
             STM32 firmware (IMU, wheel encoders, blade ESC, e-stop, rain, panel)
```

Invariants worth stealing verbatim (they are what make this stack debuggable):

- **Sole TF ownership (Inv 2).** Only `fusion_graph_node` may broadcast `map→odom` or
  `odom→base_footprint`. The driver publishes **no TF at all**. A CI test
  (`mowgli_bringup/test/test_tf_ownership.py`) fails the build if any other file constructs
  a `tf2_ros::TransformBroadcaster`.
- **Sole serial ownership.** `hardware_bridge` is the only node allowed to touch the STM32
  link; remaps live in the launch file, never in C++.
- **Map frame == GPS frame**, anchored only by `datum_lat`/`datum_lon` (no SLAM back-end;
  slam_toolbox, robot_localization dual-EKF, `navsat_transform_node` and Kinematic-ICP were
  all removed).
- **Firmware keeps sole blade/e-stop authority.** The host may *request* emergency; the board
  latches it. `blade_gate.hpp` may suppress an ENABLE but never swallow a DISABLE.
- REP-105 chain `map → odom → base_footprint → base_link (rear axle) → sensors`;
  `base_link` sits at the centre of the rear drive axle, **not** the chassis centre
  (`chassis_center_x` default 0.18 m).

TF frames: `base_footprint`, `base_link`, `blade_link`, `imu_link`, `gps_link`, `lidar_link`,
`gps` (identity alias), plus continuous joint frames for the two drive wheels and two front
casters.

---

## 4. MCU protocol approach (the part we copy hardest)

MowgliNext owns both ends of its link, which is the single biggest structural difference from
Airseekers: **they wrote the STM32 firmware too** (`firmware/stm32/ros_usbnode`, PlatformIO,
608 source files, USB CDC). We cannot — the Airseekers MCUs are Artery AT32 blobs with a
vendor protocol we reverse-engineered.

### 4.1 Framing

```
0x00 | COBS(payload ++ CRC16) | 0x00
payload = packed struct bytes, first field is the packet type/id, last 2 bytes are
          CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) little-endian over all preceding bytes
```

- Every struct is `#pragma pack(1)`; little-endian is native on both Cortex-M3 and arm64.
- `ros2/src/mowgli_hardware/firmware/mowgli_protocol.h` is the *single source of truth*,
  deliberately plain C99 so the firmware (C) and the ROS 2 bridge (C++) include the same
  definitions, with `_Static_assert` on every size/offset.
- Host mirror: `ll_datatypes.hpp` (`PacketId` enum, packed `Ll*` structs, size asserts).
- Implementation: `cobs.cpp`, `crc16.cpp` (host) / `cobs.c`, `crc16.c` (firmware),
  `packet_handler.cpp` (0x00-delimited deframer, 512 B max packet, rx/crc/overflow/cobs
  counters), `serial_port.cpp` (termios RAII, 8N1, `O_NONBLOCK`, `write_all` with EAGAIN
  retry — port is reopened on the next read tick after any error).

### 4.2 Packet table (host `ll_datatypes.hpp`, `kMowgliProtocolVersion = 6`)

| Id | Struct (bytes) | Direction | Purpose |
|---|---|---|---|
| `0x01` | `LlStatus` (38) | FW→host | status bitmask, 5× USS ranges, emergency bitmask, `v_charge`, `v_system`, `charging_current`, `batt_percentage`; ~25 Hz |
| `0x02` | `LlImu` (41) | FW→host | `dt_millis`, accel, gyro, mag; 50–100 Hz |
| `0x03` | `LlUiEvent` (5) | FW→host | panel button + press duration |
| `0x04` | `LlOdometry` (17) | FW→host | `dt_millis`, int32 L/R ticks, int16 L/R mm/s; 50 Hz |
| `0x05` | `LlBladeStatus` (16) | FW→host | consumed internally |
| `0x06` | `LlResetCause` (5) | FW→host | boot cause + WWDG breadcrumb stage |
| `0x11` / `0x12` | `LlConfigReq` (4) / `LlConfigRsp` (8) | host→FW / FW→host | protocol-version + semver handshake, 5 s timeout → `firmware_compatible=false` |
| `0x42` | `LlHeartbeat` (5) | host→FW | 4 Hz (~250 ms); firmware declares e-stop if absent past its timeout; carries `emergency_requested` / `emergency_release_requested` |
| `0x43` | `LlHighLevelState` (5) | host→FW | mode (`HL_MODE_IDLE/AUTONOMOUS/RECORDING/MANUAL_MOW/…`) + `gps_quality`, 2 Hz |
| `0x50` | `LlCmdVel` (11) | host→FW | **linear x + angular z only** |
| `0x51` | `LlCmdBlade` (5) | host→FW | blade enable |
| `0x52` | `LlReboot` (4, magic `0xB0`) | host→FW | board reboot |
| `0x54` | `LlSetDrivePid` (27) | host→FW | runtime `ticks_per_meter`, kp/ki/kd, integral limit, `pwm_per_mps` |
| `0x55` | `LlSetYawPid` (21) | host→FW | yaw loop + `gyro_bias_radps` |
| `0x56` | `LlSetKinematics` (11) | host→FW | `wheel_base`, `max_mps` |
| `0x57` | `LlSetSafetyLimits` (21) | host→FW | charge V/I, lift/estop timeouts; firmware clamps so the wire can only tighten |

### 4.3 The lessons behind that design (apply them to Airseekers)

1. **Version handshake before anything else.** The bridge sends `0x11` on every (re)connect
   and refuses to be “compatible” without a `0x12` inside 5 s. Re-arm on reconnect, and burst
   re-send runtime tuning (`pid_resend_count_ = 5`) after each reconnect.
2. **One-time push of all tunables at connect** so no param lives only on the host.
3. **Firmware owns the fast loops.** The per-wheel velocity **PI loop and `ticks_per_meter`
   live in the firmware**; the host sends linear/angular velocity only. Airseekers is the
   same shape (MCU takes linear/angular; host-side `libpid_controller` PID in the vendor
   stack) — one of the few places our port lines up almost exactly.
4. **Deadband/gain sanity gate** (`drive_gain_sanity.hpp`): before pushing PID gains, verify
   the loop can actually break stiction at 0.03 m/s, else fall back to template gains and log
   ERROR. This was a real field failure (stiction lock, 2026-09-15).
5. **Timestamp translation is explicit**: `clock_fit.hpp` `HostFirmwareClockFit` does a
   100-sample sliding linear fit from firmware `dt_millis` to host ns, resetting on a
   >5 s gap. Do not stamp IMU/odom with arrival time.
6. **Odometry hygiene**: int32 ticks unwrapped at 16 bits, per-packet spike limit (>100 ticks
   dropped), 50 ms aggregation window, `vy` variance set tiny (non-holonomic), forced to zero
   while charging.
7. **At-rest IMU bias calibration with a persisted, validated file**
   (`# mowgli_imu_calibration_v1`): abort if wheels move or the charger drops; discard
   windows whose |accel| is not gravity or whose gyro variance is exactly zero; reject a
   loaded file with all-zero covariances or |gyro offset| > 0.2 rad/s.
8. **Dig (wheel-slip) detection is a pure, unit-tested header**
   (`dig_detector.hpp`): compare *worst-wheel* travelled distance against commanded tyre
   speed, gate on GNSS RTK-fixed accuracy (`DigTrustSigma` from `/gps/status`, never the
   fused covariance), raise a latched `DigEvent` plus an escape manoeuvre
   (`dig_escape*`, backward motion) and a pending keepout proposal. Companion
   `dig_escalation.hpp` clears the latch on charger contact or 2× escape displacement.
9. **`cmd_vel` shaping before the wire** (`cmd_vel_slew.hpp`): immediate exact stop, bounded
   decel through zero before reversal, separate accel/decel limits, sub-deadband clamp, and
   echo the exact encoded command on `/hardware_bridge/cmd_vel_applied` for rosbag analysis.
10. **Validate what the firmware will silently re-clamp.** Param bounds mirror the firmware's
    `pid_constrain()` so `ros2 param set` cannot be accepted-then-ignored.
11. **Host heartbeat participates in the safety story** and `twist_mux` timeouts are tuned so
    a dead `collision_monitor` cannot keep refreshing the firmware watchdog.

### 4.4 Their own protocol-drift traps (avoid repeating them)

- `ros2/src/mowgli_hardware/firmware/` is a **hand-copied mirror stuck at v3** while the host
  is at **v6** — nothing compiles it and the drift CI guard does not check it.
  Authoritative: `firmware/stm32/ros_usbnode/`.
- `HL_MODE_*` is triplicated (msg ↔ bridge C++ ↔ firmware header).
- Bumping a struct requires editing, in lockstep: host version constant, firmware constant,
  `static_assert`s, `test_protocol.cpp`, `protocol_baseline.json`, and the C mirror.
- CI has a `protocol-version-drift.yml` job that fingerprints the wire tokens and fails if the
  wire changed without a version bump. **We should build the equivalent for the Airseekers
  frame decoder/encode pairs.**

### 4.5 Airseekers contrast (from `ros2_port_handoff/01_mcu_protocol`)

| | MowgliNext | Airseekers Tron |
|---|---|---|
| Framing | `0x00` + COBS + CRC-16-CCITT + `0x00` | `A5 | total_len | {sub_len type mod payload}+ | sum&0xFF | 5A` with `F5 01/02/03` byte-stuffing for `A5/5A/F5` |
| Dispatch | single typed packet id | **module id** inside each sub-packet: speed 4, charge 3, battery 5, imu 9, sensor 10, cutter 11, version 12, motors 13, calib 18, log 19, bms_version 24 |
| Direction | separate FW→host / host→FW ids | `type=0` host→MCU, `type=8` MCU→host, heartbeat `type=255` |
| Link | USB CDC `/dev/mowgli` @115200 | `/dev/serial_mower` → `ttyS9` @115200 8N1 (RK3588 UART9 `0xfebc0000`) |
| Firmware | theirs, in-repo, PlatformIO | vendor Artery AT32 blobs (chassis 0.6.34, cutter 0.6.36, BMS 1.18.0, rtk 0.9.50) — **never reflash** |
| IMU | on the MCU link (`LlImu` 0x02) | **separate UART** `/dev/serial_imu` (`ttyS1`), WIT JY61P 0x55 frames |
| GNSS | USB/UART sidecar, 921600 default | `/dev/serial_rtk` (`ttyS4`), Unicore UM960, NMEA + Unicore binary |
| Camera | 2D LiDAR (LD19/STL27L/RPLIDAR) | Metoak stereo depth (no MowgliNext analogue) |
| E-stop | firmware latch, host requests via heartbeat | vendor MCU enforces estop/lift/bumper cut-offs; host heartbeat period/failsafe still to be characterised |

---

## 5. Topics / services contract

### 5.1 Driver-owned surface (what `mower_mcu_driver` should reproduce)

| Topic | Type | Dir | Rate | Notes to copy |
|---|---|---|---|---|
| `/hardware_bridge/status` | `mowgli_interfaces/Status` | pub | ~10 Hz | Widest fan-out in the stack (BT, fusion_graph, map_server, diagnostics, MQTT, GUI). Carries firmware version/protocol version/compat. |
| `/hardware_bridge/emergency` | `mowgli_interfaces/Emergency` | pub | ~1 Hz | latch, stop-button, wheel-lift bits |
| `/hardware_bridge/power` | `mowgli_interfaces/Power` | pub | ~1 Hz | `charger_enabled` = firmware charging bit |
| `/battery_state` | `sensor_msgs/BatteryState` | pub | ~1 Hz | absolute topic; `current = abs(charging_current)` while charging else 0; consumed by Nav2 docking |
| `/imu/data` | `sensor_msgs/Imu` | sub of the driver | — | **RELIABLE QoS(10) on purpose** even though it is sensor data |
| `/imu/mag_raw` | `sensor_msgs/MagneticField` | pub | with imu | µT→T, published only if a subscriber exists |
| `/wheel_odom` | `nav_msgs/Odometry` | pub | ~20 Hz | `odom`→`base_link`, twist only, zeroed while charging |
| `/wheel_ticks` | `mowgli_interfaces/WheelTick` | pub | per packet | raw encoder deltas |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | **sub** | — | `SystemDefaultsQoS`; input is `twist_mux` output |
| `/cmd_vel_applied` | `geometry_msgs/TwistStamped` | pub | per packet | exact float32 command put on the wire |
| `/hardware_bridge/dig_event` | `mowgli_interfaces/DigEvent` | pub | on event | `QoS(10).transient_local()` **both ends** |
| `/hardware_bridge/dig_escalated` | `std_msgs/Bool` | pub | latch | `QoS(1).transient_local()` both ends |
| `/gps/status` | `mowgli_interfaces/GnssStatus` | **sub** | — | dig trust gate + firmware LED |
| `/gps/absolute_pose` | `mowgli_interfaces/AbsolutePose` | **sub** | — | independent veto on dig false positives |
| `/odometry/filtered_map` | `nav_msgs/Odometry` | **sub** | — | primary dig reference + event position, `SensorDataQoS` |
| `/behavior_tree_node/high_level_status` | `mowgli_interfaces/HighLevelStatus` | **sub** | — | mirrored to the firmware as `HL_STATE` |

Services hosted by the driver: `~/mower_control` (blade enable),
`~/emergency_stop`, `~/reboot_board`, `~/set_firmware_debug` (`std_srvs/SetBool`). It is a
**client** of `~/high_level_control` (panel buttons → `COMMAND_START`/`COMMAND_HOME`). No
actions.

### 5.2 Broader stack surface (context for later packages)

Published: `/gps/fix` (`NavSatFix`, `SensorDataQoS` — *this*, not `/gps/pose_cov`, feeds the
localizer), `/gps/pose_cov`, `/odometry/filtered` (odom-frame DR) and
`/odometry/filtered_map` (map frame), `/coverage/full_plan`, `/keepout_mask` +
`/costmap_filter_info`, `/map_server_node/{mow_progress,docking_pose,boundary_violation,
lethal_boundary_violation,replan_needed}`, `/obstacle_tracker/obstacles`,
`/mowgli/localization/mode{,_id}`, `/scan`, `/scan_deskewed`, `/scan_costmap`,
`/scan_collision`, `/collision_monitor_state`, `/diagnostics` (9 statuses at 1 Hz),
`/behavior_tree_node/high_level_status`.

Services: `/behavior_tree_node/high_level_control` (the one command entry point for
GUI/MQTT/HomeKit/scheduler/buttons — `COMMAND_START=1`, `COMMAND_HOME=2`,
`COMMAND_RECORD_AREA=3`, …, `COMMAND_STOP=8`, `COMMAND_RESET_EMERGENCY=254`) plus the
`/map_server_node/*` CRUD family.

Actions: `/plan_coverage` (`PlanCoverage`), `/dock_robot`, `/undock_robot`,
`/navigate_to_pose`, `/follow_path` (`controller_id="FollowCoveragePath"`), `/backup`,
`/calibrate_imu_yaw_node/calibrate_dock`.

### 5.3 Conventions that make the stack maintainable

- C++ nodes declare **`~/name`** private topics; **all remapping happens in the launch file**.
- Shared system topics are **absolute strings in source**.
- Naming is a function of the *node name* in the launch file — `hardware_bridge_node` exe run
  as node `hardware_bridge` is why `~/status` is `/hardware_bridge/status`. (MowgliNext was
  bitten by this: renaming the node breaks `map_server`'s `/hardware_bridge/dig_event`.)
- QoS rules: sensor data `SensorDataQoS`; commands reliable; status/diagnostics `QoS(10)`;
  anything a late joiner must see on connect `QoS(1).transient_local()`. Two deliberate
  exceptions: `/imu/data` reliable, `/hardware_bridge/dig_event` transient-local both ends.
- `twist_mux` with `use_stamped: true` and **no `locks:` block** — e-stop is the firmware
  latch reached through a service, not a mux lock. Lanes: `navigation` 10 (0.6 s),
  `docking` 15, `teleop` 20, `tuning` 30, `emergency` 100 (0.2 s).

---

## 6. Nav2 usage

| Aspect | MowgliNext choice |
|---|---|
| Global planner | `SmacPlanner2D` |
| Transit controller | `RegulatedPurePursuitController` (`primary_controller.max_linear_vel` = `transit_speed`) + `RotationShimController` |
| Coverage controller | **custom `FTCController`** in `mowgli_nav2_plugins` — follow-the-carrot, 3-axis PID, claims <10 mm lateral accuracy on swaths; loaded by `controller_server` via `ftc_controller_plugin.xml` |
| Goal checker | custom `PathProgressGoalChecker` (`coverage_goal_checker`) |
| Odometry input | `controller_server.odom_topic` → `/wheel_odom` (never leave it unset — Nav2's default `/odom` has no publisher) |
| Global costmap | `static_layer` (`/map` or `/no_lidar_static_map`), `obstacle_layer` (`/scan_costmap`), **`keepout_filter`** from `/keepout_mask` + `/costmap_filter_info`; driven in **delta mode** |
| Local costmap | `/scan_costmap` filtered scan |
| Safety | `collision_monitor` in the `cmd_vel` chain: `cmd_vel_nav` → monitor → `cmd_vel_monitored`; polygon stop/slow/approach topics |
| Scan conditioning | `scan_deskew_node` (IMU) → `costmap_scan_filter_node` (gravity-aware ground filter) → `/scan_deskewed` → `/scan_costmap`; unfiltered `/scan_collision` keeps *every* near-field return |
| Docking | `docking_server` (`opennav_docking`) + `SimpleChargingDock`, `battery_topic=/battery_state`, `staging_x_offset`, `max_retries`; `gps_dock_detection_node` optionally approaches the dock off RTK-Fixed `/gps/absolute_pose` |
| Undock | `nav2_msgs/BackUp` — explicitly **not** `UndockRobot` (Invariant 10) |
| Lifecycle | `lifecycle_manager_navigation`; the BT PAUSEs Nav2 on the dock and re-activates it (`Nav2ResumeGuard`) |
| Recovery | `ClearCostmap` actions, `FollowStripRetry` ×5 with `StuckBackoff`, `EscapeStartBlocked`, `DynamicObstacleSkip`, `AreaUnreachable` |
| Mission logic | BehaviorTree.CPP v4 `main_tree.xml`, root `ReactiveSequence` so all guards re-evaluate every tick |
| Coverage | Fields2Cover v3 `plan_coverage` → continuous hole-free `drivable_subpaths`; each sub-path is ONE `FollowPath` goal; progress grid survives restarts |

### Humble deltas we will hit (verify each against the Humble Nav2 version we pin)

- **Distro gap is the big one:** upstream is Lyrical/Ubuntu 26.04, we are Humble/Ubuntu 22.04
  on aarch64. Every source file must be re-checked against Humble APIs.
- **No docking server in Humble.** `opennav_docking` (and its `nav2_docking` predecessor) is
  not available; docking/undocking has to be a custom BT node or a small state machine in
  `mower_behavior`. Update 2026-10-08: done as the standalone `mower_docking` package
  (`/mower_docking/dock`, `/mower_docking/undock` actions).
- **Costmap filter / behaviour-server parameter renames** exist upstream (e.g.
  `local_costmap_topic`, `local_footprint_topic` vs Humble's `costmap_topic`/`footprint_topic`;
  a wrong key is *silently ignored*). Check `nav2_params_*.yaml` key-by-key for Humble.
- **`nav2_msgs/CollisionMonitorState`** is newer than Humble — our `IsObstacleStuck` guard
  needs an alternative source.
- `keepout_filter` + `filter_info_topic` and `collision_monitor` do exist in Humble, but
  polygon parameter names differ; confirm.
- `smac_planner`/`nav2_smac_planner` and `RPP` exist in Humble; `RotationShimController` exists.
- ~~`TwistStamped` and `twist_mux use_stamped` exist in Humble — good, the mux design ports as-is.~~
  Correction 2026-10-08: Humble's twist_mux (4.3.0, what we run) has **no `use_stamped`**; our
  whole `/cmd_vel` chain is unstamped `geometry_msgs/Twist` (`src/mower_teleop/config/twist_mux.yaml`,
  `cmd_vel_slew`, `mcu_node`). MowgliNext's stamped mux needs a TwistStamped→Twist shim.
- **GTSAM and Fields2Cover must be built from source for arm64 in the image** (upstream does
  exactly this: `gtsam-builder`, `fields2cover-v3-builder` stages). Expect long build times.
- RMW: upstream uses **`rmw_cyclonedds_cpp`** in the runtime image for ARM service discovery
  without shared-memory issues — copy that choice for RK3588.
- Upstream is honest that Webots sim images are **amd64 only**; on our aarch64 box we will need
  a fake-node/sim harness instead (their `fake_hardware_bridge_node` pattern is the model).

---

## 7. Mapping table — MowgliNext package → Airseekers-Tron driver package

Target hardware: MCU on `/dev/serial_mower`, IMU WIT JY61P, GNSS Unicore UM960, Metoak stereo
camera, RK3588 aarch64, ROS 2 Humble in Docker.

### 7.1 This batch (four packages)

| MowgliNext package / artefact | Airseekers-Tron package | What we take | What we must change / write fresh |
|---|---|---|---|
| **`mowgli_hardware`** (`hardware_bridge_node`, `mowgli_hardware_core` static lib: `serial_port`, `packet_handler`, `cobs`, `crc16`, `clock_fit`, `odometry_publisher`) | **`mower_mcu_driver`** | Architecture: one node owns the link; private `~/` topics + launch remaps; read tick timer separate from publish rate; reconnect with backoff; counters (`rx_ok`, `rx_crc_errors`, `rx_overflow`, `rx_stuff_errors`); header/C++ split (pure framing + pure logic unit-testable); `/wheel_odom` at a fixed aggregation window with spike rejection and forced-zero while charging; `cmd_vel_slew` before the wire + `/cmd_vel_applied` echo; `blade_gate` (ENABLE-only suppression); heartbeat + emergency request/release; runtime gain push; dig detector (`dig_detector.hpp`/`dig_escalation.hpp`) gated on `/gps/status` RTK-fixed; 15-style gtest suite incl. a protocol/layout test | Framing: `A5 … 5A` + `F5` byte-stuffing + `sum&0xFF`, module-id dispatch (speed 4 / charge 3 / battery 5 / imu 9 / sensor 10 / cutter 11 / version 12 / motors 13 / calib 18 / log 19 / bms 24) instead of COBS+CRC16 and typed ids. Port defaults `/dev/serial_mower`, 115200 8N1. No firmware version handshake packet — use the **version (mod 12)** sub-packet; host heartbeat is `type=255` and its timeout/failsafe is *unchartered*. Add the vendor topics from the ROS 1 contract: `/mower_sensor_info` (33 Hz), `/mower_base/{status,motor_info,button_info,battery_health,net_status}`, `/battery`, `/notice_code`, `/notice_info`, `/calibration_result`, and services `/cutter_control`, `/charging`, `/fake_charging`, `/enable_bumper`, `/clear_estop`, `/reset_odom`, `/poweroff`, `/fill_light_control`. Keep the vendor MCU as the sole e-stop authority. **No firmware source, no `0x54`-style runtime tuning packets** — gains stay host-side unless the vendor protocol allows it |
| **IMU half of `mowgli_hardware`** (`LlImu` 0x02 handling, `imu_liveness.hpp`, at-rest bias calibration, persisted `# …_v1` calibration file, µT→T conversion) | **`wit_imu_driver`** | The whole role, moved to its own node because our IMU is a different UART: parse → `/imu/data` (reliable `QoS(10)`, frame `imu_link`), `/imu/temperature_c` (we already expose temperature; MowgliNext does not — keep ours), `/imu/mag_raw` (update 2026-10-08: **not published yet**; the 0x54 frame is parsed in `wit_protocol.py` but `wit_node.py` has no `MagneticField` publisher); dead-sample/liveness watchdog (`IsImuSampleDead`, 45-sample threshold) rejecting a hung JY61P bus; at-rest bias calibration with the same *plausibility* rules (|accel|≈g, non-zero gyro variance) and a persisted validated file; host clock fit for timestamps; bias is pushed into `/odometry/filtered_map` yaw handling downstream | Wire format is the **public WIT 0x55 frame set** (0x51 accel, 0x52 gyro, 0x53 angle, 0x54 mag, …) — no COBS, no CRC, no packet ids. Port `/dev/serial_imu` (`ttyS1`, 115200, 100 Hz, 98 Hz BW per `wit_imu_config.py`). Publishes **no TF**; supplies `imu_link` extrinsics to the URDF instead. The `LlImu`-equivalent `mod 9` IMU sub-packet from the MCU is still parsed by `mower_mcu_driver` — decide which source wins (recommend JY61P as `/imu/data`, MCU stream as `/mower_sensor_info`-only, and characterise whether the MCU *requires* `SendImuData`) |
| **`sensors/gps/` sidecar** (`mowgli_gnss_bridge` adapter + vendored `universal-gnss` runtime, `gnss_receiver_family: auto\|ublox\|unicore\|nmea`) | **`um960_gps_driver`** | The public contract exactly: `/gps/fix` (`sensor_msgs/NavSatFix`, `SensorDataQoS`, covariance from the receiver's own reported horizontal/vertical accuracy as `COVARIANCE_TYPE_APPROXIMATED`, never invented → `COVARIANCE_TYPE_UNKNOWN` and no `/gps/pose_cov` downstream), `/gps/status` (`mowgli_interfaces/GnssStatus` via a shared `gnss_status_utils` so BT/LED/driver cannot disagree), `/rtcm` (`rtcm_msgs/Message` public mirror of the NTRIP hop), `/diagnostics`; YAML → env → default config resolution | Receiver is **Unicore UM960** on `/dev/serial_rtk` (`ttyS4`), NMEA **plus Unicore binary** — the vendor `mower_gps_driver_node` parsed both. UM960 sits behind the vendor `rtk_rover` AT32 board + LoRa M4 (`ota.py -t 21 -m 37`), so **analyse what actually appears on the wire before choosing a parser**. `universal-gnss` supports UM98x, not UM960 — either add a UM960 parser or write the driver directly. NTRIP from `/userdata/mower/ntrip.yaml` (credentials deliberately excluded from the bundle). Publish `/fix_fused`-style and `/rtk_input` debug topics only if the port needs them |
| **`mowgli_interfaces`** (16 msg / 15 srv / 3 action + shared headers) | **`mower_interfaces`** | Split and rename by domain instead of one grab-bag: `Status`, `Emergency`, `Power`, `DigEvent`, `WheelTick`, `ESCStatus`, `ImuRaw`, `HighLevelStatus`, `HighLevelControl`, `MowerControl`, `EmergencyStop`, `GnssStatus` + `gnss_status_utils.hpp`, `AbsolutePose`, `mow_progress`/map messages. Keep the `~/`-relative-node-name + launch-remap convention, the QoS table, and the “documented-but-dead” discipline | Carry over the Airseekers ROS 1 message set (`mower_msgs`, 88 msg/srv/action, `mower_gps_msgs`) field-for-field so the behaviour tree and GUI contract survive: `/mower_sensor_info`, `/mower_base/motor_info`, `/wheel_vel`, `/bumper_cloud`, `/notice_*`, `/gps_dev_status`, `/mower_gps_node/info`. Add only what Nav2 needs. Drop everything GUI-only. Provide `wgs84_projection`/datum helpers for the later localizer |

### 7.2 Later batches (named now so boundaries stay clean)

| MowgliNext package / artefact | Planned Airseekers-Tron package | Notes |
|---|---|---|
| `fusion_graph` (`fusion_graph_node`, GTSAM iSAM2) | `mower_localization` (or reuse `robot_localization` + `navsat_transform` on Humble) | Our `/fix` + `/imu/data` + `/wheel_odom` are the same three inputs; decide factor-graph vs EKF **after** the drivers work. Build GTSAM for aarch64 |
| `mowgli_localization` | `mower_localization` | `navsat_to_absolute_pose`, `cog_to_imu`, `mag_yaw_publisher`, `calibrate_imu_yaw_node`, `localization_monitor_node` all port conceptually |
| `mowgli_bringup` | `mower_bringup` | URDF for the Tron chassis (`m2-11…m2-16` variants exist in the handoff), `twist_mux` lanes, Nav2 YAML for Humble |
| `mowgli_nav2_plugins` (`FTCController`, `PathProgressGoalChecker`) | `mower_nav2_plugins` | Follow-the-carrot coverage controller — the piece that gives sub-cm swath accuracy; port from their `.hpp`/`.cpp` |
| `mowgli_behavior` (`behavior_tree_node`, `main_tree.xml`) | `mower_behavior` | Guards map onto our services (`/clear_estop`, `/charging`, `/mower_sensor_info` rain bit). Undock must be `BackUp`; docking has **no Humble `docking_server`** → custom node |
| `mowgli_coverage` (`coverage_server`, Fields2Cover v3) | `mower_coverage` | Needs F2C built from source on arm64 |
| `mowgli_map` (`map_server_node`, `obstacle_tracker_node`) | `mower_map` | Area CRUD, keepout mask, mow progress. Vendor already has `LocatorSaveMap`/`LoadMap`/`GetMapInfo` services and a map DB at `/userdata/mower/robot.db` |
| `mowgli_monitoring` | `mower_monitoring` | `/diagnostics` aggregation |
| `mowgli_leds` | `mower_leds` | Vendor has `mower_light_sound_node` + `libws_2812.so` on `/dev/spidev3.0` — binary-only, may stay binary or be replaced |
| `mowgli_simulation` (`fake_hardware_bridge_node`) | `mower_simulation` | Our substitute for Webots; a `fake_mower_port` replay node already exists in the vendor tree |
| `mowgli_tools` (`tools/motor/mowgli_tools`) | `mower_tools` | Drive-PID tuner, wheel radius/yaw cal (`ros2/scripts/wheel_radius_cal.py`, `wheel_yaw_cal.py` upstream) |
| `sensors/lidar-*` (LD19 / RPLIDAR / STL27L) | **no equivalent** | Airseekers has no 2D LiDAR |
| — | **new: `metoak_stereo_driver`** | **No MowgliNext analogue at all.** Metoak "Archer" stereo module: `/dev/videoSimor`, `/dev/videoIsp`, OA `video53`/`video44`, I²C-8 (`xc9080` ISP, `icm40608` IMU, `lis2mdl` mag), SDK-only depth (`libMoGeneralSDK`), camera SN `IC17SZ01011`, calibration in `08_calibration_identity/`. Plan: convert depth to an obstacle layer (`pointcloud_to_obstacles` / `sensor_msgs/PointCloud2` → costmap) and/or a MetaOAK obstacle msg. Needs its own package and its own risk register |
| `firmware/stm32/ros_usbnode` (+ `mowgli_hardware/firmware/*.c`) | **nothing** | Explicitly out of scope: we do not reflash. Only borrow the *C-source-of-truth discipline* for our decoder, and write the Airseekers analogue of `protocol_version_guard.py` |
| `gui/` | out of scope for now | Vendor HTTP API (`05-maps-and-http-api.md`, `openapi.json`) is the UI contract to preserve |
| `install/` + `docker/` + `sensors/*/Dockerfile` | `ros2_stack/docker/` | Same pattern: multi-stage `ros:humble-*-base`, GTSAM/F2C builders, Cyclone DDS runtime, compose fragments, `--device /dev/serial_mower -v /dev/serial_imu …` passthrough, on aarch64 |

### 7.3 Interface mapping cheat-sheet (MowgliNext → ours)

| MowgliNext | Airseekers-Tron | Source of truth on our side |
|---|---|---|
| `/hardware_bridge/status` | `/mower_base/status` | MCU `sensor`/`motors`/`version` sub-packets |
| `/hardware_bridge/emergency` | `/mower_base/emergency` (new msg; estop/lift/bumper) | MCU status bitmask + `/clear_estop` |
| `/hardware_bridge/power` | `/mower_base/charge` (new) | MCU `charge` (mod 3) |
| `/battery_state` | `/battery` (keep vendor name) | MCU `battery` (mod 5) + `bms_version` (mod 24) |
| `/imu/data` | `/imu` (vendor) **or** `/imu/data` (Nav2-friendly) | JY61P via `wit_imu_driver` |
| `/imu/mag_raw` | `/imu/mag_raw` (new; not published yet as of 2026-10-08) | JY61P mag frame |
| `/wheel_odom`, `/wheel_ticks` | `/odom`, `/wheel_vel` | MCU `speed` (mod 4) + IMU yaw, host-side |
| `/cmd_vel` | `/cmd_vel` + `/cmd_vel_stamped` | both (vendor ROS 1 node subscribed to both) |
| `/hardware_bridge/dig_event` | `/mower_base/dig_event` (new) | `mower_mcu_driver`, gated on `/gps/status` |
| `/gps/fix` | `/fix` (vendor) or `/gps/fix` | `um960_gps_driver` |
| `/gps/status` | `/gps_dev_status` / `/gps/status` | `um960_gps_driver` |
| `/mower_control`, `/emergency_stop` | `/cutter_control`, `/clear_estop` | `mower_mcu_driver` |
| — | `/mower_sensor_info`, `/mower_base/motor_info`, `/bumper_cloud`, `/notice_code`, `/notice_info`, `/calibration_result`, `/factory_test_result` | `mower_mcu_driver` (no MowgliNext analogue) |
| — | `/rain` (`/dev/rain` ADC), `/mower_base/button_info` (`/dev/keyboard`), `/mower_base/net_status`, `/fill_light_control`, `/charging`, `/fake_charging`, `/enable_bumper`, `/reset_odom`, `/poweroff` | `mower_mcu_driver` + later packages |

---

## 8. Takeaways for the four packages

1. **One node per serial link, ever.** `mower_mcu_driver` owns `/dev/serial_mower`;
   `wit_imu_driver` owns `/dev/serial_imu`; `um960_gps_driver` owns `/dev/serial_rtk`. No
   cross-talk with the vendor services (`mower-base.service` etc. must be stopped before
   testing — two writers on one port).
2. **Publish no TF from a driver.** Adopt MowgliNext's Invariant 2 and add a CI check.
3. **Structure like `mowgli_hardware_core`**: framing (`serial_port`, `packet_handler`,
   `cobs`→`stuffing`, `crc16`→`sum8`) in a static lib, pure logic in headers, one `rclcpp::Node`.
   The vendor protocol already has a validated Python decoder
   (`mcu_frame_decode.py`, 172 850 frames, zero checksum errors) — use it as the oracle for
   our C++ tests.
4. **Version/identity before control.** MowgliNext's `0x11/0x12` handshake is the right shape;
   our substitute is the `version` (mod 12) sub-packet plus a published firmware-compat flag.
5. **Heartbeat discipline is a safety feature, not a nicety** — but its timeout and failsafe
   behaviour are the #1 unresolved unknown in the handoff (`(?)` items). Characterise it with
   the vendor's own `mower_sdk_test` before we ever send motion.
6. **Characterise, don't assume.** Units of `BatteryInfo.current` / `MotorInfo.*`, meaning of
   module id 7, `SensorInfoControl` semantics, and whether `SendImuData` is mandatory are all
   open. Encode every assumption as a named parameter with a documented default.
7. **Keep the "dig" concept** — Airseekers' 4-wheel + caster chassis on grass with RTK
   localisation will dig, and the vendor stack has exactly this failure mode. It is the most
   valuable single piece of logic to port.
8. **Publish the exact commanded value** (`/cmd_vel_applied`-style) — it is the cheapest
   possible debugging tool on a real mower.
9. **Distro discipline.** Humble target: `TwistStamped` + `twist_mux use_stamped` do **not**
   port unchanged (Humble twist_mux 4.3 has no `use_stamped`; we run unstamped `Twist`); the docking server, `CollisionMonitorState`, and several Nav2 YAML keys do not
   exist in Humble — plan substitutes now rather than at bring-up time.

---

## 9. Sources

Fetched 2026-10-05:

- Repo root README — `https://raw.githubusercontent.com/mowglinext/mowglinext/main/README.md`
- ROS 2 stack README (architecture, packages, topics/services/actions, config, Docker, Nav2) —
  `https://raw.githubusercontent.com/mowglinext/mowglinext/main/ros2/README.md`
- Wire protocol header (packet ids, packed structs, protocol version) —
  `https://raw.githubusercontent.com/mowglinext/mowglinext/main/ros2/src/mowgli_hardware/firmware/mowgli_protocol.h`
- Driver codemap (node internals, params, packet table, pitfalls) —
  `.../main/docs/claude/codemaps/mowgli_hardware.md`
- Exhaustive ROS interface index (topics/services/actions/twist_mux/TF/QoS) —
  `.../main/docs/claude/ros-interfaces.md`
- Sensors sidecar contract (GNSS + LiDAR containers) — `.../main/sensors/README.md`
- Repo metadata / file tree / releases — `api.github.com/repos/mowglinext/mowglinext/…`

Local, read-only, for the Airseekers side:

- `ros2_port_handoff/README.md` (device map, protocol TL;DR, ROS 1 contract, safety cautions)
- `ros2_port_handoff/01_mcu_protocol/` (`protocol_definitions_recovered.h`,
  `mcu_frame_decode.py`, `mower_base_pkg/`, traffic logs)
- `ros2_port_handoff/03_ros_interfaces/` (`mower_msgs`, `mower_gps_msgs`)
- `ros2_stack/README.md` (this port's ground rules: Humble, Docker, no writes to the mower)