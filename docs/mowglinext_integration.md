# MowgliNext integration — reference stack for the Airseekers-Tron ROS 2 port

Companion to [`mowglinext_baseline.md`](mowglinext_baseline.md) (deep research
notes) and [`interfaces_map.md`](interfaces_map.md) (our `mower_interfaces`
subset). This doc is the *integration* record: exactly what was cloned, which
parts of its contract our three drivers already satisfy, what must be ported,
and a concrete Docker/compose plan to run its stack alongside ours.

Research date: 2026-10-05. All facts below were re-verified against the local
clone `/tmp/mowglinext-src` (read-only reference) and our `ros2_stack/`.

> **Status note — 2026-10-08.** The research above stands, but several of *our*
> facts in it were true only when written. Superseded by the code:
>
> - **§3 "Two conflicts" — both resolved.** `wit_imu_driver` now publishes
>   `/imu/data` directly and `mower_mcu_driver`'s copy is on `/mcu/imu`, so there
>   is no duplicate name. `wit_imu_driver publish_tf` defaults to `false`, so the
>   URDF owns `base_link → imu_link`.
> - **§3 "Our driver hosts no services yet" — false now.** `mcu_node` serves
>   `/cutter_control`, `/charging`, `/clear_estop`, `/cutter_off`,
>   `/cutter/set_height`; `mower_gui_bridge` serves the `hardware_bridge` and
>   `behavior_tree_node` services and publishes `/hardware_bridge/{status,emergency,power}`,
>   `/gps/{fix,status}`, `/wheel_odom`, `/odometry/filtered_map`.
> - **§3 `/gps/status` type** — already `mowgli_interfaces/GnssStatus`, converted
>   from the driver's `/fix_status` String by `mower_gui_bridge`.
> - **§7 risks 6, 7, 8** — see below; risk 7 and 8 are closed, risk 6 still holds
>   for the MCU's BEST_EFFORT IMU echo.
> - Topic names elsewhere in this doc (`wit_imu_driver` → `/imu`, `um960` →
>   `/fix`) were the pre-rename state.
>
> Current state of the integration surface: [`mowglinext_handoff.md`](mowglinext_handoff.md).

---

## 1. Repo identity (exact)

| Field | Value |
|---|---|
| URL | **https://github.com/mowglinext/mowglinext** |
| Clone used | `git clone --recurse-submodules https://github.com/mowglinext/mowglinext /tmp/mowglinext-src` |
| Branch / commit | `main` @ **`faf658bc57b92410ee820adff9953e663b5b0230`** — Tue 2026-09-15 21:37:56 +0200, "Merge pull request #624 from mowglinext/dev" |
| Submodule 1 | `ros2/src/external/universal-gnss` @ **`ab32f673da6e9e6ffa8eac9a57b08656f8843645`** (from https://github.com/mowglinext/universal-gnss.git) |
| Submodule 2 | `ros2/src/opennav_coverage` @ **`d6e41a298c5e9767cc2a42c9fafa7ca41b9bf089`** (from https://github.com/open-navigation/opennav_coverage.git) |
| Latest release | `v1.4.0` (`4fcabb3732b52cf339a1339ecdb97d612f1aec09`) — first public beta |
| ROS distro | **ROS 2 Lyrical / Ubuntu 26.04** (rebuilt in v1.4.0). **Not Humble** — see §4.4 |
| Licence | Dual: GPLv3 (open/personal/educational/non-profit) + **separate commercial licence** (`contact@mowgli.garden`) for commercial use. Clear this before any product integration. |

Local checkout (read-only reference): `/tmp/mowglinext-src`.

---

## 2. Major packages

The ROS 2 stack lives in `ros2/src/` — 12 colcon packages plus the two git
submodules. (The repo is a monorepo: `gui/` = web UI, `firmware/` = STM32,
`install/` + `docker/` + `sensors/` = deployment.)

| Package | Executables | Role | Our counterpart |
|---|---|---|---|
| `mowgli_interfaces` | — | 16 `.msg` / 15 `.srv` / 3 `.action` (`CalibrateDock`, `CoverageTask`, `PlanCoverage`) + shared C++ headers (`gnss_status_utils.hpp`, `wgs84_projection.hpp`) | `mower_interfaces` |
| `mowgli_bringup` | `empty_static_map_pub.py`, `cmd_vel_ws_relay.py`, `wait_for_tf.py` | Launch (`full_system.launch.py`, `mowgli.launch.py`, `navigation.launch.py`, `foxglove_bridge.launch.py`), Nav2 params, URDF/xacro, `twist_mux` | `mower_bringup` (later) |
| `mowgli_hardware` | `hardware_bridge_node` | The single ROS↔STM32 boundary: COBS/CRC16 framing, status/IMU/wheel-odom publishing, `cmd_vel` in, blade/e-stop out, runtime PID push | **`mower_mcu_driver`** (+ IMU half → `wit_imu_driver`) |
| `mowgli_localization` | `wheel_odometry_node`, `navsat_to_absolute_pose_node`, `localization_monitor_node`, `cog_to_imu`, `mag_yaw_publisher`, `calibrate_imu_yaw_node`, `scan_deskew_node`, `costmap_scan_filter_node`, `gps_dock_detection_node` | GPS→ENU pose, mode monitor, yaw helpers, scan conditioning | `mower_localization` (later) |
| `fusion_graph` | `fusion_graph_node` | **Sole localizer** — GTSAM iSAM2 Pose2 factor graph owning both `map→odom` and `odom→base_footprint` | `mower_localization` (later) |
| `mowgli_nav2_plugins` | — (plugin lib) | `FTCController` (follow-the-carrot coverage) + `PathProgressGoalChecker`, loaded via `ftc_controller_plugin.xml` | `mower_nav2_plugins` (later) |
| `mowgli_behavior` | `behavior_tree_node` | BehaviorTree.CPP v4 mission executor (`main_tree.xml`) | `mower_behavior` (later) |
| `mowgli_coverage` | `coverage_server` | Fields2Cover v3 headland+swath planner, `plan_coverage` action | `mower_coverage` (later) |
| `mowgli_map` | `map_server_node`, `obstacle_tracker_node` | GridMap areas, keepout mask, mow progress, obstacle promotion | `mower_map` (later) |
| `mowgli_monitoring` | `diagnostics_node`, `mqtt_bridge_node` | `/diagnostics` aggregator, MQTT/Home-Assistant bridge | `mower_monitoring` (later) |
| `mowgli_leds` | `led_ring_node` | Optional WS2812 ring over SPI, off by default | `mower_leds` (later) |
| `mowgli_simulation` | `fake_hardware_bridge_node`, `sim_actuation_node` | Webots world + fake hardware bridge (amd64-only images) | `mower_simulation` (later) |
| `external/universal-gnss` (submodule) | GNSS runtime | u-blox F9P, **Unicore UM98x**, generic NMEA | **`um960_gps_driver`** |
| `opennav_coverage` (submodule) | msgs by default | `opennav_coverage_msgs` action defs; server/BT/navigator optional (need Fields2Cover) | — |

Out-of-tree but shipped in the runtime image: `mowgli_gnss_bridge`
(`sensors/gps/`, adapts universal-gnss to the public topic contract),
`mowgli_tools` (`tools/motor/`, drive-PID tuner invoked via `ros2 run` inside
the container), and exactly one LiDAR container selected by `LIDAR_TYPE`
(LD19 / STL27L / RPLIDAR A1).

**The stack's invariants (adopt them):** only `fusion_graph_node` broadcasts
`map→odom`/`odom→base_footprint` (CI test fails any other
`tf2_ros::TransformBroadcaster`); only `hardware_bridge` touches the serial
link; all remapping lives in launch files, nodes declare `~/private` topics.

---

## 3. What our drivers already satisfy

Target contract = MowgliNext's driver-facing surface (baseline doc §5.1).
**Our three drivers already cover the sensor/command half of it**, mostly via
topic renames:

| MowgliNext contract | Type | Our source today | Status |
|---|---|---|---|
| `/gps/fix` | `sensor_msgs/NavSatFix` | `um960_gps_driver` → `/fix` (param `fix_topic`) | ✅ rename only (`/fix` → `/gps/fix`). Our publisher is RELIABLE(10); MowgliNext uses `SensorDataQoS` — a RELIABLE pub is compatible with best-effort subs, so this is safe, optionally switch to `SensorDataQoS` for parity |
| `/imu/data` | `sensor_msgs/Imu` | `wit_imu_driver` → `/imu/data` (JY61P, param `imu_topic`) | ✅ same name, and RELIABLE depth 10 like MowgliNext (`wit_node.py:87-91`). Orientation and covariances are filled |
| `/imu/mag_raw` | `sensor_msgs/MagneticField` | `wit_imu_driver` (mag frame 0x54) | ❌ not published. `wit_protocol.py` parses the 0x54 frame (and the sensor's mag output is off by default in `cmd_set_output`), but `wit_node.py` has no `MagneticField` publisher |
| `/wheel_odom` | `nav_msgs/Odometry` (`odom`→`base_link`) | `mower_mcu_driver` → `/odom` | ✅ rename; our frames (`odom_frame`/`base_frame` params, default `odom`/`base_link`) already match. Our `/odom` is RELIABLE(10) — fine |
| `/battery_state` | `sensor_msgs/BatteryState` | `mower_mcu_driver` → `/battery` | ✅ rename (RELIABLE(10) — fine) |
| `/cmd_vel` (input) | `geometry_msgs/TwistStamped` | `twist_mux` → `/cmd_vel_raw` → `cmd_vel_slew` → `/cmd_vel` → `mower_mcu_driver` | ❌ **unstamped `geometry_msgs/Twist`** end to end: Humble's twist_mux 4.3 has no `use_stamped` (`src/mower_teleop/config/twist_mux.yaml`), `cmd_vel_slew` is `Twist` in/out, and `mcu_node` takes `Twist` on `/cmd_vel` (it also accepts `TwistStamped` on `/cmd_vel_stamped`). Any MowgliNext producer needs a TwistStamped→Twist shim |
| `/mower_sensor_info`, `/mower_base/status`, `/mower_base/motor_info` | `mower_interfaces/*` | `mower_mcu_driver` | ✅ vendor contract preserved — these are *ours*, no MowgliNext analogue; keep them |
| `/imu/temperature_c` | `std_msgs/Float32` | `wit_imu_driver` | ✅ extra we keep (MowgliNext has no temperature topic) |
| `um960` extras `/vel`, `/heading`, `/fix_status`, `/nmea` | `TwistStamped`, `Float32`, `String` | `um960_gps_driver` | ✅ keep as debug/COG sources (`/heading` ≈ COG input for `cog_to_imu`) |

**Two conflicts to resolve before any MowgliNext consumer runs — both since closed:**

1. ~~**Duplicate `/imu`.**~~ **Resolved:** `wit_imu_driver` (JY61P) owns
   `/imu/data` via its `imu_topic` parameter; `mower_mcu_driver`'s copy of the
   same data (the MCU echoing back what the host forwarded it) publishes on
   `/mcu/imu`. No name collision.
2. ~~**`wit_imu_driver` broadcasts TF.**~~ **Resolved:** `publish_tf` defaults to
   `false`, and the URDF provides `base_link → imu_link`. No driver broadcasts TF,
   so MowgliNext Invariant 2 holds.

**The MowgliNext-only surface.** Update 2026-10-08: most of it is now served by
`mower_gui_bridge` (defaults in `gui_bridge_node.py:62-100`): `/hardware_bridge/status`
(`mowgli_interfaces/Status`), `/hardware_bridge/emergency` (`Emergency`),
`/hardware_bridge/power` (`Power`), `/gps/fix`, `/gps/status` (`GnssStatus`, converted
from the driver's `/fix_status` String), `/wheel_odom`, `/odometry/filtered_map`, and the
services `/hardware_bridge/mower_control`, `/hardware_bridge/emergency_stop` and
`/hardware_bridge/reboot_board`. **Still not provided:** `/wheel_ticks` (`WheelTick`),
`/cmd_vel_applied` (`cmd_vel_slew` logs the applied value instead of publishing it),
`/hardware_bridge/dig_event`/`dig_escalated`, `/gps/absolute_pose` (`AbsolutePose`),
`/rtcm` (`rtcm_msgs/Message`) and `~/set_firmware_debug`. Our driver *does* host services now:
`mcu_node` serves `/cutter_control`, `/charging`, `/clear_estop`, `/cutter_off`
and `/cutter/set_height`, and `mower_gui_bridge` serves the `hardware_bridge` and
`behavior_tree_node` ones above; the vendor ROS 1 stack's `/cutter_control`,
`/clear_estop`, `/charging`,
`/enable_bumper`, `/reset_odom`, `/poweroff` are the functional equivalents to
expose.

---

## 4. What needs porting

### 4.1 MCU protocol differences (the hard part)

`mowgli_hardware` is unusable against our MCU as-is — it speaks MowgliNext's
own firmware protocol, and we must never reflash the vendor Artery AT32 MCUs:

| | MowgliNext | Airseekers Tron (ours) |
|---|---|---|
| Framing | `0x00` + COBS(payload++CRC16-CCITT) + `0x00`, 512 B max, packet-id dispatch | `A5 \| len \| {sub_len type mod payload}+ \| sum&0xFF \| 5A`, `F5 01/02/03` byte-stuffing, **module-id dispatch** (speed 4, charge 3, battery 5, imu 9, sensor 10, cutter 11, version 12, motors 13, calib 18, log 19, bms_version 24) |
| Direction | separate FW→host / host→FW packet ids | `type=0` host→MCU, `type=8` MCU→host, heartbeat `type=255` |
| Link | USB CDC `/dev/mowgli` @115200 | `/dev/serial_mower` → `ttyS9` @115200 8N1 |
| Version handshake | `0x11`/`0x12` semver handshake, 5 s timeout → `firmware_compatible` | `version` sub-packet (mod 12) — use as the compat flag source |
| Heartbeat | `0x42` 4 Hz, carries estop request/release | `type=255` — **timeout/failsafe behaviour uncharacterised** (#1 open unknown; characterise with vendor `mower_sdk_test` before sending motion) |
| Runtime tuning | `0x54`/`0x55`/`0x56`/`0x57` push PID/kinematics/safety to FW | Not available — gains stay host-side |
| Firmware | theirs, in-repo (PlatformIO) | vendor blobs (chassis 0.6.34, cutter 0.6.36, BMS 1.18.0, rtk 0.9.50) |

Port = reuse `mowgli_hardware`'s **architecture** (one node per serial link,
`~/` private topics + launch remaps, read-tick vs publish-rate separation,
reconnect with backoff, rx/crc/overflow counters, `cmd_vel_slew` +
`/cmd_vel_applied` echo, dig detector gated on RTK-fixed `/gps/status`, 15-test
gtest suite) with our `A5…5A` framing. The validated Python oracle
(`mcu_frame_decode.py`, 172 850 frames, zero checksum errors) is the test
reference. Already done: `mower_mcu_driver` implements framing + the core
publishers above. Remaining: the §3 "nothing satisfies yet" surface.

### 4.2 Message schemas

`mowgli_interfaces` (16 msg / 15 srv / 3 action) vs our `mower_interfaces`
(11 msg / 10 srv / 2 action — msgs: `BatteryHealthInfo`, `ControlInfo`, `MotorControl`,
`MotorInfo`, `MotorStatus`, `MowerBaseButtonInfo`, `MowerBaseDevInfo`, `MowerBaseDevStatus`,
`MowerBaseMotorsInfo`, `MowerSensorInfo`, `PlannerOption`; srvs: `BumperControl`,
`ChargingControl`, `CutterControl`, `GetAreaSettings`, `MappingControl`, `PlanCoverage`,
`PlannerTaskSet`, `SetAreaSettings`, `SetCutterHeight`, `SetLoRa`; actions: `Dock`, `Undock`).
Update 2026-10-08: `mowgli_interfaces` itself is now copied into `src/mowgli_interfaces`, so
the list below is satisfied by that package rather than by additions to `mower_interfaces`.
To consume MowgliNext packages unmodified, add to `mower_interfaces` (or a
thin `mowgli_shim` package):

- **Required by consumers:** `Status`, `Emergency`, `Power`, `GnssStatus`
  (+`gnss_status_utils.hpp` so BT/LED/driver cannot disagree), `WheelTick`,
  `DigEvent`, `HighLevelStatus`, `HighLevelControl`, `MowerControl`,
  `EmergencyStop`, `AbsolutePose`.
- **Keep our vendor schema field-for-field** for the ROS 1 contract
  (`interfaces_map.md`) — do *not* rename `/battery`, `/mower_sensor_info`
  etc. inside `mower_mcu_driver`; do the MowgliNext renames in the **launch
  file** (their own convention).
- Actions to port: `PlanCoverage`, `Dock`/`Undock` (undock = `nav2_msgs/BackUp`
  upstream, not `UndockRobot`), `NavigateToPose`, `FollowPath`.

### 4.3 GNSS receiver support

`universal-gnss` supports u-blox F9P, **Unicore UM98x**, and generic NMEA —
**not the UM960**. Options, in order of preference:
1. Keep `um960_gps_driver` as the publisher of `/gps/fix` + `/gps/status`
   (add a `GnssStatus` publisher mapping our `/fix_status` string + fix mode),
   and skip universal-gnss entirely.
2. Add a UM960 parser to the vendored universal-gnss (its UM98x parser is the
   template) — only if we want their NTRIP/`/rtcm` mirror and config tooling.
The UM960 sits behind the vendor `rtk_rover` AT32 board + LoRa M4, so
**analyse the wire before choosing** (baseline §7.1).

### 4.4 Distro / dependency deltas (Lyrical → Humble)

- Nav2: `SmacPlanner2D`, RPP, `RotationShimController`, `keepout_filter`,
  `collision_monitor` exist in Humble; Humble's twist_mux (4.3, what we run) has
  **no `use_stamped`**, so our `/cmd_vel` chain is unstamped `Twist`. Docking does not use
  `opennav_docking`: this repo ships its own `mower_docking` package (rclpy action server,
  `/mower_docking/dock` and `/mower_docking/undock`, ArUco-guided reverse docking), and no
  image installs `opennav_docking`; `nav2_msgs/CollisionMonitorState` is newer than Humble;
  several costmap YAML keys renamed (`costmap_topic` vs
  `local_costmap_topic`) — wrong keys are *silently ignored*, so check
  `nav2_params_*.yaml` key-by-key.
- Source builds needed on aarch64: **GTSAM** and **Fields2Cover v3** (upstream
  does exactly this in `gtsam-builder` / `fields2cover-v3-builder` stages —
  expect long build times). BehaviorTree.CPP v4, grid_map and twist_mux are
  apt-installable in Humble (twist_mux without `use_stamped`, see above).
- RMW: upstream runtime uses **`rmw_cyclonedds_cpp`** + `cyclonedds.xml`
  (ARM service discovery without shared-memory issues). Our image pins
  `rmw_fastrtps_cpp`. Both containers on the same host/DOMAIN_ID can mix only
  if every node uses the same RMW — pick Cyclone for both, or FastRTPS for
  both.
- Webots sim images are amd64-only; on the RK3588 use their
  `fake_hardware_bridge_node` pattern instead.

### 4.5 Hardware / sensors

No 2D LiDAR on Airseekers (Metoak stereo depth instead — **no MowgliNext
analogue**, needs its own `metoak_stereo_driver` package); IMU is a separate
UART (JY61P) not the MCU link; GNSS is UM960 not UM98x/F9P; no STM32
firmware to build. All of `sensors/lidar-*`, `firmware/stm32/`, and
`mowgli_simulation`'s Webots world are out of scope.

---

## 5. How the web UI is served

| Aspect | Detail |
|---|---|
| Framework | **Go backend** (Gin, module `github.com/mowglinext/mowglinext`, entry `gui/main.go`, built with Go 1.25) + **React 19 frontend** (`gui/web/`, Vite + TypeScript, antd v5 + Formily, Mapbox GL, ~532 files total; TS API types generated from the Go API via `generate_ts_types.sh`) |
| Container | `mowgli-gui` from `ghcr.io/mowglinext/mowglinext/mowglinext-gui:main` (or `:dev`; built by `cd gui && make build`) |
| URL / port | **`http://<board-ip>:4006`** — the Go server's default `system.api.addr` is `:4006`, serving the built web app from `system.api.webDirectory` (`/app/web`) plus a REST API under `/api` and Swagger at `/swagger/`. Host networking, so "the container's port 4006" *is* the host's |
| ROS 2 access | GUI is a **Foxglove WS client**: it connects to `foxglove_bridge` running **inside the `mowgli-ros2` container** on **`ws://<board-ip>:8765`** (env `FOXGLOVE_URL`, default `ws://localhost:8765`; config `install/config/mowgli/foxglove_bridge.yaml` → `port: 8765`, `address: 0.0.0.0`). Teleop relay on **:8766** (`cmd_vel_ws_relay.py` accepts `TwistStamped` JSON). **No rosbridge** — both GUI and Foxglove Studio speak the Foxglove WebSocket |
| Privileges | `network_mode: host`, `pid: host`, `privileged: true` — the power menu `nsenter -t 1` into the host to run `systemctl reboot`/`poweroff`; runs as root |
| Mounts | Docker socket (`DOCKER_HOST=unix:///var/run/docker.sock`), `/dev`, bitcask DB dir (`DB_PATH=/db`), mower config dirs (`MOWER_CONFIG_FILE`, `MOWER_YAML_CONFIG_FILE`, `MOWER_RUNTIME_ENV_FILE`), maps volume |
| Optional | MQTT bridge `:1883` / WS `:9001` (Home Assistant), HomeKit provider, Tailscale/remote-access sidecar, in-GUI updates (Settings → Updates), NTRIP, weather |

For us: the GUI container is optional for bringup — Foxglove Studio can talk to
`ws://<ip>:8765` directly (our `Dockerfile.jazzy` already tries to install
`ros-jazzy-foxglove-bridge`, guarded). Porting the *vendor* HTTP API
(`openapi.json`, mower_docs/05) is a separate, later exercise; the GUI's value
to us now is as a reference for the UI contract and the compose fragment
patterns.

---

## 6. Concrete plan: run the MowgliNext stack in our Docker Humble image

Goal: build `ros2/src` from `/tmp/mowglinext-src` **inside our existing
`mower:jazzy` image** (or a derived image), mount its sources as a Docker
volume, and run it side-by-side with our driver stack — without touching
`ros2_stack/src`.

### 6.1 Image additions (extend `docker/Dockerfile.jazzy`, or a `Dockerfile.mowgli` derived from it)

```dockerfile
FROM mower:jazzy
# Humble-apt-able deps (BT.CPP v4, grid_map, twist_mux, docking, Cyclone DDS)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-behaviortree-cpp ros-jazzy-grid-map ros-jazzy-twist-mux \
      ros-jazzy-opennav-docking ros-jazzy-cyclonedds \
      ros-jazzy-rtcm-msgs libeigen3-dev && rm -rf /var/lib/apt/lists/*
# GTSAM + Fields2Cover v3 must be built from source for arm64 (long builds;
# mirror upstream's gtsam-builder / fields2cover-v3-builder stages and cache
# the results in a derived image, NOT in the runtime container layer).
```

### 6.2 Docker volume for its src

Two equivalent options; the named volume is the one for the mower device:

```yaml
volumes:
  mowgli_src: {}     # named volume, populated once from /tmp/mowglinext-src
```

Populate it (dev host, once, after any `git pull`):

```bash
docker run --rm -v mowgli_src:/dst -v /tmp/mowglinext-src/ros2/src:/src:ro \
  alpine sh -c 'cp -a /src/. /dst/'
```

or, for dev iteration, bind-mount the checkout directly (zero-copy, live edit)
— note this nests inside our existing `..:/work` bind, which is fine:

```yaml
volumes:
  - /tmp/mowglinext-src/ros2/src:/work/mowgli/src:ro
```

The submodules (`external/universal-gnss`, `opennav_coverage`) come along
because the clone used `--recurse-submodules`.

### 6.3 docker-compose override

Drop this next to `docker/docker-compose.yml` and run with
`docker compose -f docker/docker-compose.yml -f docker/docker-compose.mowgli.yml`:

```yaml
# docker-compose.mowgli.yml — MowgliNext stack alongside our drivers.
# Usage: docker compose -f docker/docker-compose.yml \
#                       -f docker/docker-compose.mowgli.yml <cmd>
volumes:
  mowgli_src:

services:
  # Our existing service, plus the mowgli sources and Cyclone DDS config.
  mower_jazzy:
    environment:
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # must match mowgli_ros2
    volumes:
      - mowgli_src:/work/mowgli/src:ro
      - ../mowgli/cyclonedds.xml:/cyclonedds.xml:ro   # copy from
                                                       # mowglinext-src/docker/fastdds.xml
                                                       # (or upstream cyclonedds.xml)

  # The MowgliNext side: same image, separate workspace, own launch.
  mowgli_ros2:
    build:
      context: ..
      dockerfile: docker/Dockerfile.mowgli        # §6.1
      network: host
    image: mower:jazzy-mowgli
    container_name: mowgli_ros2
    platform: linux/arm64
    privileged: true
    network_mode: host          # same LAN DDS domain as mower_jazzy
    ipc: host
    stdin_open: true
    tty: true
    environment:
      - ROS_DISTRO=jazzy
      - ROS_DOMAIN_ID=0         # same domain as mower_jazzy
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - UNIVERSAL_GNSS_PATH=/work/mowgli/src/external/universal-gnss
    volumes:
      - mowgli_src:/work/mowgli/src:ro
      - ../mowgli/cyclonedds.xml:/cyclonedds.xml:ro
      - /dev:/dev
    devices:
      - /dev/ttyS9:/dev/ttyS9   # serial_mower (MCU)
      - /dev/ttyS1:/dev/ttyS1   # serial_imu (JY61P)
      - /dev/ttyS4:/dev/ttyS4   # serial_rtk (UM960)
    command: bash
```

> **Port conflict note:** only one container may own each serial device at a
> time. During MowgliNext bringup, run either `mower_jazzy`'s drivers or
> `mowgli_ros2`'s `hardware_bridge_node` against `ttyS9` — never both
> (baseline §8.1: stop vendor `mower-base`/`mower-logic`/`mower-controller`
> services too). The end state is **our drivers** feeding **their stack**:
> disable `mowgli_hardware` in `mowgli_bringup` and remap our topics onto
> their contract in a shim launch file (§3).

### 6.4 Build and run

```bash
# 1. One-time: copy the upstream DDS config next to the override
mkdir -p ros2_stack/mowgli
cp /tmp/mowglinext-src/docker/fastdds.xml ros2_stack/mowgli/cyclonedds.xml

# 2. Populate the named volume (§6.2), then build the mowgli workspace
docker compose -f docker/docker-compose.yml \
               -f docker/docker-compose.mowgli.yml build mowgli_ros2
docker compose -f docker/docker-compose.yml \
               -f docker/docker-compose.mowgli.yml run --rm mowgli_ros2 \
  bash -c 'source /opt/ros/jazzy/setup.bash && \
           rosdep install --from-paths /work/mowgli/src --ignore-src \
             --rosdistro jazzy -y \
             --skip-keys "grid_map_core grid_map_ros grid_map_msgs beluga_ros \
               nav2_smac_planner webots_ros2_driver opennav_coverage \
               opennav_coverage_bt opennav_coverage_demo \
               opennav_coverage_navigator opennav_row_coverage universal-gnss" && \
           cd /work/mowgli && colcon build --symlink-install'

# 3. Run OUR drivers first (they own the serial links and satisfy §3)
docker compose -f docker/docker-compose.yml \
               -f docker/docker-compose.mowgli.yml run --rm mower_jazzy \
  bash -c 'source /opt/ros/jazzy/setup.bash && source /work/install/setup.bash && \
           ros2 launch mower_bringup bringup.launch.py'   # (or our launch pkg)

# 4. Then run the MowgliNext consumers against the remapped topics,
#    with mowgli_hardware disabled in a shim launch file:
docker compose -f docker/docker-compose.yml \
               -f docker/docker-compose.mowgli.yml run --rm mowgli_ros2 \
  bash -c 'source /opt/ros/jazzy/setup.bash && \
           source /work/mowgli/install/setup.bash && \
           ros2 launch mowgli_bringup navigation.launch.py'

# 5. Verify the contract
ros2 topic list | grep -E 'gps/fix|imu/data|wheel_odom|battery_state'
ros2 topic echo /gps/fix --once
```

### 6.5 Sequencing / milestones

1. **Shim first, stack second.** Port nothing but the launch remaps + a
   `mower_interfaces` → `mowgli_interfaces` adapter node; prove
   `fusion_graph_node` can consume our `/gps/fix` + `/imu/data` +
   `/wheel_odom` on a bench (no motion) before touching Nav2.
2. Characterise the MCU heartbeat (`type=255`) timeout/failsafe with the
   vendor SDK **before** enabling any `cmd_vel` path.
3. Port the missing driver surface (§3 "nothing satisfies yet") into
   `mower_mcu_driver`, including `dig_event` — the single most valuable piece
   of MowgliNext logic for a 4-wheel RTK mower on grass.
4. Only then bring up `mowgli_behavior` + `mowgli_coverage` (needs GTSAM +
   Fields2Cover v3 arm64 builds) and substitute a custom dock state machine
   for the missing Humble `docking_server`.

---

## 7. Risks / open questions

| # | Item | Note |
|---|---|---|
| 1 | Licence | Dual GPLv3/**commercial** — resolve before any commercial integration |
| 2 | Distro gap | Lyrical vs Humble: API renames, missing docking server, newer `CollisionMonitorState`; re-check every source file |
| 3 | Heartbeat failsafe | Vendor `type=255` timeout behaviour uncharacterised — safety-critical unknown |
| 4 | UM960 support | universal-gnss has no UM960 parser; our driver is the pragmatic path |
| 5 | Build time | GTSAM + Fields2Cover v3 source builds on aarch64 are long; cache in a derived image |
| 6 | QoS | `wit_imu_driver` publishes `/imu/data` RELIABLE(10), matching MowgliNext; only the MCU's `/mcu/imu` echo is BEST_EFFORT (`mcu_node.py:716-724`). MowgliNext expects `transient_local` on dig events, which we don't publish. Our `/battery` and `/odom` are already RELIABLE(10) |
| 7 | ~~Duplicate `/imu`~~ | **Closed:** `/imu/data` from `wit_imu_driver`, `/mcu/imu` from `mower_mcu_driver` |
| 8 | ~~Driver TF~~ | **Closed:** `wit_imu_driver publish_tf:=false`; the URDF owns `base_link → imu_link` |
| 9 | RAM/CPU | RK3588 already runs the vendor stack + stereo; GTSAM iSAM2 + Nav2 + BT is heavy — budget before full bringup |

---

## 8. Sources

- `/tmp/mowglinext-src` — full clone @ `faf658b` with submodules (universal-gnss
  @ `ab32f67`, opennav_coverage @ `d6e41a2`)
- `README.md`, `ros2/README.md`, `gui/README.md`, `docker/README.md`,
  `install/compose/docker-compose.{base,gui,foxglove}.yml`,
  `install/config/mowgli/foxglove_bridge.yaml` (`port: 8765`),
  `install/lib/checks.sh` (port-4006 reachability check),
  `gui/main.go` + `gui/pkg/api/api.go` (Gin, `system.api.addr` default `:4006`),
  `gui/pkg/providers/ros.go` (Foxglove WS client, `ws://localhost:8765`,
  topic map), `gui/pkg/providers/cmd_vel_relay.go` + `ros2/src/mowgli_bringup/scripts/cmd_vel_ws_relay.py`
  (relay :8766), `gui/Dockerfile` (Go 1.25 + node:22 build stages),
  `gui/web/package.json` (React 19 + Vite + antd/Formily)
- Our side: `ros2_stack/docker/{Dockerfile.jazzy,docker-compose.yml}`,
  `ros2_stack/launch/bringup.launch.py`, `ros2_stack/src/{mower_mcu_driver,
  wit_imu_driver,um960_gps_driver}`, `ros2_stack/src/mower_interfaces`
  (11 msg / 10 srv / 2 action), and docs `mowglinext_baseline.md`, `interfaces_map.md`,
  `mcu_protocol_spec.md`, `wit_imu.md`, `um960.md`, `deployment.md`
