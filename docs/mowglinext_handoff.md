# MowgliNext integration handoff — Airseekers Tron ROS 2 stack

**For:** the MowgliNext developer evaluating an integration.
**From:** the Airseekers Tron ROS 2 port team.
**Date:** 2026-10-08.
**Read alongside:** `mowglinext_baseline.md` and `mowglinext_integration.md`
(our research notes and the long integration record). Three further docs are
relevant but **internal**, so their substance is restated here instead of
linked: `mcu_protocol_spec.md`, `interfaces_map.md`, `wheel_control_semantics.md`
(§0).

Everything below is scoped to **open-source components only** — §0 says exactly
what that includes and what has been deliberately left out.

---

## 0. Scope: what is in this document, what is not

**Included (open source).**

| Group | Contents | Licence |
|---|---|---|
| Our ROS 2 packages | 30 of the 31 packages with a `package.xml` under `ros2_stack/src/` | Apache-2.0 (21), BSD-3-Clause (1), MIT (2), GPL-3.0 (3), GPL-3.0-or-later (3) — see §9. (`mower_proto` is excluded, see below) |
| Our launch/config/URDF/tests | `ros2_stack/launch/`, `config/`, `scripts/`, `docker/`, and each package's `test/` | same as the packages they serve |
| Our docs | `ros2_stack/docs/`, **except** the three listed as internal below | same |
| MowgliNext itself | a **subset** of your repo, pinned in-tree at `third_party/mowglinext/` @ `faf658bc57b92410ee820adff9953e663b5b0230`: `gui`, `mowgli_behavior`, `mowgli_bringup`, `mowgli_coverage`, `mowgli_map`, `mowgli_monitoring`, `mowgli_nav2_plugins` (no `mowgli_interfaces`, `mowgli_hardware`, localization or `fusion_graph`). The `gui` tree carries our Airseekers patches (robot profile, `/gui/*` topics, `/cutter/*`, …) | **GPLv3** (non-commercial) / commercial licence |
| Open-source upstreams we use | OpenVINS (git submodule `third_party/open_vins`, pinned `rpng/open_vins` @ `69488123`), Fields2Cover, Nav2, `robot_localization`, RKNN runtime, Fast DDS | their own upstream licences |

**Excluded (not open source / not ours to share), and therefore not described
anywhere below:**

- Vendor AT32 firmware images (chassis 0.6.34, cutter 0.6.36, BMS 1.18.0, RTK 0.9.50)
  and every other vendor binary or blob.
- Vendor ROS 1 package sources and the decompilation of them.
- Protocol documents recovered out of vendor debug info / DWARF. Concretely,
  **`docs/mcu_protocol_spec.md`, `docs/interfaces_map.md` and
  `docs/wheel_control_semantics.md` are internal** — the first two are largely
  recovered vendor material, the third quotes the decompiled host directly.
  Where integration genuinely needs to know how the serial link behaves, this
  document restates the behaviour in our own words (§5, §8.3); the recovered
  headers, the validated frame-decode oracle and the vendor SDK sources stay
  private.
- `ros2_stack/src/mower_proto` — generated protobuf bindings for the vendor
  BLE/cloud payload schema. Not ours to redistribute, so it is out of scope.
  Consequence: `base_ble` (§2.1) `exec_depend`s on it and imports its modules,
  so `base_ble` does not run without it.
- NTRIP/RTK credentials, cloud and BLE keys, field maps, site calibration files.
- Anything under `ros2_port_handoff/` or the original vendor `src/`.

Everything in §1–§8 is built from code that is either Apache-2.0/BSD/MIT and
originally ours, or GPL-3.0(-or-later) and originally yours.

---

## 1. The short version

We ported the Airseekers Tron mower (RK3588 brain, Artery AT32 motor MCUs,
UM960 RTK, WIT IMU, Metoak stereo, RK3588 NPU) from the vendor's ROS 1 stack to
**ROS 2 Humble**, in Docker, modelled on MowgliNext. 30 packages plus a
per-package test suite (per-package test counts in `docs/STATUS.md`),
one entry point: `ros2 launch mower_bringup mower.launch.py`.

For integration the important fact is: **we have the hardware half, you have
the user/mission half, and the seam is already partly wired.** We built
`mower_gui_bridge` specifically to speak your GUI's contract
(`/hardware_bridge/*`, `/gps/*`, `/wheel_odom`, `/odometry/filtered_map`,
`/behavior_tree_node/*`), and we ported your `mowgli_interfaces` message set.
(That contract is your *stock* GUI's. Our patched copy in
`third_party/mowglinext/gui` additionally reads `/gui/{status,pose,emergency,gnss_status,detections}`
and `/cutter/height_mm`, and calls `/cutter/set_height`.)

What integration actually costs is four things:

1. **Run us side by side, not merged.** Two containers, `network_mode: host`,
   `ipc: host`, same `ROS_DOMAIN_ID`, same RMW. Your stack never touches the
   serial ports. (§3.1)
2. **Finish the adapter**: topic remaps, `Twist` ↔ `TwistStamped` on the
   command path, and the ~12 MowgliNext-only messages/services we don't publish
   yet. (§3.2–3.4)
3. **Do not use `mowgli_hardware` against this machine.** It speaks your
   firmware's COBS/CRC protocol; our boards are locked vendor blobs we will
   never reflash. Our `mower_mcu_driver` replaces it. (§3.5)
4. **Decide the distro question.** You target ROS 2 Lyrical / Ubuntu 26.04; we
   are on Humble on a 20.04 host. Different containers make this a non-issue
   for DDS, but every package you want to run natively needs a Humble port.
   (§3.6)

---

## 2. What we have built, and what you would actually need

Legend — **Needed**: required for MowgliNext to run on this hardware.
**Optional**: only if you want our version of that subsystem instead of yours.
**Drop**: superseded by a MowgliNext component.

### 2.1 Drivers (hardware → ROS). Your stack cannot run without these.

| Package | Licence | What it does | |
|---|---|---|---|
| `mower_mcu_driver` | Apache-2.0 | The only open-source driver for the AT32 chassis/cutter/BMS serial link. Publishes `/odom`, `/battery`, `/mower_base/status`, `/mower_sensor_info`, `/mcu/{measured,commanded,sent}_speed`; subscribes `/cmd_vel` (and `/cmd_vel_stamped`), `/estop_request`; serves `/cutter_control`, `/charging`, `/clear_estop`, `/cutter_off`, `/cutter/set_height`. Owns `ttyS9`. | **Needed** |
| `wit_imu_driver` | Apache-2.0 | WIT JY61P on `ttyS1` → `/imu/data` (100 Hz, RELIABLE, orientation + covariances filled, no TF) + `/imu/temperature_c`. The 0x54 magnetic frame is parsed by the protocol module but not published. | **Needed** |
| `um960_gps_driver` | MIT | Unicore UM960 RTK → `/fix` (NavSatFix, 10 Hz), `/fix_status` (String), `/vel`, `/heading`, `/nmea`, `/gps/corrections` (1 Hz JSON correction-health summary), `/ntrip/status`. RTCM corrections in via NTRIP or LoRa, written straight to the receiver's serial port. | **Needed** |
| `bumper_controller` | BSD-3-Clause | Bumper back-off routing: on a hit in `/mower_base/status`, backs up and rotates clear via `/cmd_vel_bumper` (a `twist_mux` input), publishes `/bumper_cloud` and `/mower_base/bumper_routing_status`. The instant bumper cut-off is the MCU's. | **Needed (safety)** |
| `base_keys` | Apache-2.0 | Top-panel buttons via evdev → `/mower_base/key_pressed`, plus `HighLevelControl` START/STOP/HOME mapping. | **Needed (physical UI)** |
| `mower_cameras` | Apache-2.0 | OA stereo L/R, rear cam, Metoak hardware depth, stereo IMU. | Optional (perception) |
| `base_ble` | Apache-2.0 | BLE bridge to the base station. Needs the excluded `mower_proto` (§0). | Optional |
| `mower_lights` | Apache-2.0 | `light_controller`: LED strip modes. | Optional |

### 2.2 Interfaces

| Package | Licence | What it does | |
|---|---|---|---|
| `mowgli_interfaces` | GPL-3.0 | **Your** message set, ported: 16 msg / 15 srv / 3 action (`PlanCoverage`, `CoverageTask`, `CalibrateDock`). | **Needed** |
| `mower_interfaces` | Apache-2.0 | The vendor ROS 1 contract kept field-for-field (11 msg / 10 srv / 2 action: `MowerSensorInfo`, `MowerBaseDevStatus`, `MotorInfo`, `ControlInfo`, `CutterControl`, `ChargingControl`, `SetCutterHeight`, `BumperControl`, `Dock`, `Undock`, …). Consumers of the ROS 1 behaviour need this unchanged. | **Needed** |
| `mower_lights_interfaces` | Apache-2.0 | `LightMode.msg` and `LightControl.srv` for `light_controller`. | Optional |

> **Action item:** verify our `mowgli_interfaces` copy still matches upstream
> `main` @ `faf658b` field-for-field. If it has drifted, the CDR types will not
> interoperate across DDS and nothing will appear on either side.

### 2.3 The integration surface

| Package | Licence | What it does | |
|---|---|---|---|
| `mower_gui_bridge` | Apache-2.0 | Presents *our* topics under *your* GUI's hard-coded names: `/hardware_bridge/{status,emergency,power}`, services `/hardware_bridge/{mower_control,emergency_stop,reboot_board}` and `/navsat_to_absolute_pose/set_datum`, `/gps/{fix,status}`, `/wheel_odom`, `/odometry/filtered_map` (frame `map`). With `serve_high_level:=true` (only when launched with `mission:=false`) it also serves a stub mission layer `/behavior_tree_node/{high_level_control,high_level_status,start_in_area,clear_coverage_resume,coverage_resume_available}` with states IDLE / IDLE_DOCKED / RECORDING / MANUAL_MOWING / EMERGENCY (your `state_name` strings); STOP is accepted as a command, START and HOME are refused. Blade-on is refused unless the high-level state is 2 or 4. | **Needed — this is the seam** |
| `mower_coverage_bridge` | Apache-2.0 | Serves **your** `mowgli_interfaces/action/PlanCoverage` on top of our Fields2Cover planner, so `PlanCoverageArea` → `FollowStrip` → one Nav2 `FollowPath` per sub-path runs unchanged. | **Needed if you keep our planner** |

### 2.4 Control and safety

| Package | Licence | What it does | |
|---|---|---|---|
| `mower_control` | Apache-2.0 | `cmd_vel_slew` (`/cmd_vel_raw` → `/cmd_vel`, exact-stop watchdog, applied-command echo), `slip_detector` (commanded vs measured twist, RTK-gated, latched), `imu_cal` (at-rest bias, persisted + validated file). | **Needed** |
| `mower_teleop` | GPL-3.0 | `twist_mux` + WebSocket `cmd_vel_ws_relay` — the same architecture as your `cmd_vel_relay`. | **Needed** |
| `mower_control` (cont.) | Apache-2.0 | `supervisor` (health/diagnostics, `~/dump` service) and `mow_recorder`. | **Needed** |
| `mower_vision` | Apache-2.0 | `obstacle_guard`: consumption policy for detections → emergency zero burst + `/cutter_off`, published policy, markers. | Recommended (safety) |
| `mower_localization` | MIT | `gps_gate` (fix-worthiness gate decided from the message, never from a filter that already fused it), `heading_aligner`, `vio_gate`, EKF config, `navsat_transform_node` config. 93 tests. | **Optional — see §3.7** |
| `mower_navigation` | Apache-2.0 | Nav2 params tuned for a 4-wheel RTK mower on grass. | Optional (you bring your own) |
| `mower_docking` | Apache-2.0 | `/mower_docking/dock` and `/mower_docking/undock` actions + `NavigateToPose` client + calibration file. | Recommended — you have no Humble docking server |
| `mower_mission` | GPL-3.0-or-later | The full mission layer, **on by default** (`mission:=true`). Runs as node `behavior_tree_node` and serves `/behavior_tree_node/*` (the gui_bridge stub is then off). A plain Python state machine ported from `mowgli_behavior`'s `main_tree.xml` / FollowStrip / coverage persistence / recording: START/HOME, undock, wait for RTK, plan coverage (`/plan_coverage` client), mow, transit, dock, record area, resume, manual blade, plan preview. | Optional (you bring `mowgli_behavior`) — **it uses the same node name, so run only one of the two** |
| `mower_map` | GPL-3.0-or-later | Map server / zone storage. | Optional |
| `mower_bringup` | Apache-2.0 | **The single entry point**: `mower.launch.py` + `bringup.launch.py`, `config/urdf/mower.urdf.xacro`, all group toggles. | **Needed** |

### 2.5 Perception (NPU)

| Package | Licence | What it does | |
|---|---|---|---|
| `mower_rknn` | Apache-2.0 | Shared `rknn-toolkit-lite2` wrapper + preprocess. | Needed for §2.5 |
| `det_ros` / `det_ros_cpp` | Apache-2.0 | YOLOv8 `*.rknn` on the NPU → `/ai/det/detections` (`Detection2DArray`). Both run as node name `det_ros`; `cpp` is default, `python` the fallback. | Needed for §2.5 |
| `det_range` | GPL-3.0-or-later | Fuses detections + `/stereo_depth/depth/image_raw` + `/vio/right/camera_info` + static TF → `/ai/det/{detections_ranged,obstacles,obstacle_points}`. | Needed for §2.5 |
| `seg_ros` | Apache-2.0 | PP-LiteSeg → `/ai/seg/{mask,traversability}`. Nothing consumes it yet. | Optional |
| `stereo_vio_bridge` | Apache-2.0 | `vio_odom_bridge` (OpenVINS `/ov_msckf/odomimu` → `/odometry/vio`, with `vio:=true`) plus a legacy Metoak capture node (only with `vio_capture:=bridge`; by default `mower_cameras/stereo_cam` feeds `/vio/*`). | Optional |
| OpenVINS | GPL-3.0 (upstream) | VIO back-end. Not in `src/`: git submodule `third_party/open_vins` (upstream `rpng/open_vins` @ `69488123`), built only by `scripts/build_openvins_mower.sh` with `config/vio/patches/*.patch` applied. | Optional |

### 2.6 Superseded by MowgliNext — drop these, or keep only as a porting reference

| Package | Licence | Replace with |
|---|---|---|
| `mower_coverage` | Apache-2.0 | `mowgli_coverage` + `opennav_coverage` (or keep ours and use `mower_coverage_bridge`) |
| `mowgli_nav2_plugins` | GPL-3.0 | yours — this *is* a port of your plugins; keep it as the reference for the Humble port |

---

## 3. What integration requires

### 3.1 Architecture: two containers, DDS is the seam

We deliberately keep two workspaces and glue them over DDS rather than merging
source trees. This is already sketched in `mowglinext_integration.md` §6.
The YAML below is the **target**, not what runs today: our `mower_humble`
container currently uses `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` with a Fast DDS
profile (`docker/docker-compose.yml`), so one side has to switch before the two
stacks can discover each other.

```yaml
# mower_humble  — ours: drivers, control, perception, GUI adapter
  network_mode: host
  ipc: host
  environment: [ROS_DOMAIN_ID=0, RMW_IMPLEMENTATION=rmw_cyclonedds_cpp]   # today: rmw_fastrtps_cpp
  devices: [/dev/ttyS9, /dev/ttyS1, /dev/ttyS4]

# mowgli_ros2   — yours: mowgli_behavior, mowgli_coverage, mowgli_map, GUI, Nav2
  network_mode: host
  ipc: host
  environment: [ROS_DOMAIN_ID=0, RMW_IMPLEMENTATION=rmw_cyclonedds_cpp]
  volumes: [/dev:/dev]   # NO serial devices — our container owns them
```

Rules that fall out:

- **One owner per serial device.** Only `mower_humble` may open `ttyS9`
  (chassis), `ttyS1` (IMU), `ttyS4` (RTK). `mowgli_hardware` must be disabled in
  `mowgli_bringup`.
- **RMW must match** on both sides or you get silent discovery failures. We
  are on Fast DDS today; agree on one (CycloneDDS in the target above) and
  switch the other side.
- **Message types must be bit-identical.** Same `mowgli_interfaces` commit, no
  local field edits.
- No custom bridge process is needed for topics — only remaps (§3.2) and, for
  type mismatches, a ~200-line adapter node (§3.4).

### 3.2 Remaps your side expects vs what we publish

| MowgliNext contract | Type | Ours today | Work |
|---|---|---|---|
| `/gps/fix` | `sensor_msgs/NavSatFix` | driver → `/fix` @10 Hz; `mower_gui_bridge` relays `/gps/fix` @2 Hz (GUI-only) | ✅ present. Feed fusion from `/fix`, or set `gps_fix_rate_hz: 0` |
| `/gps/status` | `mowgli_interfaces/GnssStatus` | `mower_gui_bridge` → `/gps/status`, converted from the driver's `/fix_status` (`std_msgs/String`) | ✅ type already matches |
| `/imu/data` | `sensor_msgs/Imu` | `wit_imu_driver` → `/imu/data` @100 Hz | ✅ already correct |
| `/wheel_odom` | `nav_msgs/Odometry` (`odom`→`base_link`) | driver → `/odom` @50 Hz; `mower_gui_bridge` relays `/wheel_odom` @5 Hz | ✅ present. Fusion should use `/odom` |
| `/battery_state` | `sensor_msgs/BatteryState` | `mower_mcu_driver` → `/battery` | one remap |
| `/cmd_vel` (input) | `geometry_msgs/TwistStamped` | `/cmd_vel_raw` in, `/cmd_vel` out of `cmd_vel_slew` — unstamped `geometry_msgs/Twist` end to end (Humble `twist_mux` 4.3 has no `use_stamped`; the driver reads `Twist` on `/cmd_vel`, `TwistStamped` only on `/cmd_vel_stamped`) | ❌ needs a `TwistStamped` → `Twist` conversion (§3.4) |
| `/hardware_bridge/{status,emergency,power}` | `mowgli_interfaces/*` | `mower_gui_bridge` (5 / 5 / 2 Hz) | ✅ implemented |
| `/odometry/filtered_map` | `nav_msgs/Odometry` | `mower_gui_bridge`, frame `map` | ✅ implemented (frame `map` == `odom` today) |
| `/wheel_ticks`, `/cmd_vel_applied` | `mowgli_interfaces/WheelTick`, `Twist` | not published | see §3.3 |

### 3.3 What `mower_gui_bridge` already gives you, and what nobody gives you

`mower_gui_bridge` (§2.3) is the component that translates our vendor-shaped
topics into your contract. Already implemented:

| Provided | Notes |
|---|---|
| `/hardware_bridge/{status,emergency,power}` | 5 / 5 / 2 Hz. `emergency` is clamped to ≥2 Hz because your BT treats >2 s silence as an emergency |
| `/gps/fix`, `/gps/status` (`GnssStatus`), `/wheel_odom`, `/odometry/filtered_map` | rate-limited for the GUI on purpose — `foxglove_bridge` cannot rate-limit, and these were built as GUI-only copies. Feed fusion from the originals (`/fix`, `/odom`) or set the `*_rate_hz` params to `0` |
| `/hardware_bridge/{mower_control,emergency_stop,reboot_board}` | services |
| `/behavior_tree_node/{high_level_control,start_in_area,clear_coverage_resume}` | served by `mower_mission` by default; the gui_bridge stub (`serve_high_level: true`) only runs with `mission:=false` |
| `/navsat_to_absolute_pose/set_datum` | calls robot_localization's `/datum` and persists the datum into `/userdata/ros2/datum.env` (and `mowgli_robot.yaml` when `robot_yaml_path` is set) |
| `/gui/{pose,status,emergency,gnss_status,detections}`, `/diagnostics` | low-rate web-UI copies + `DiagnosticArray` |

**Still missing — nothing publishes these today:**

- `/hardware_bridge/dig_event` and `dig_escalated`. `mower_control/slip_detector`
  computes the equivalent (commanded vs measured twist, RTK-gated) and latches
  it, but not on your topic or your `DigEvent` type. One publisher.
- `/cmd_vel_applied`. `cmd_vel_slew` logs the applied command instead of
  publishing it — one line to change.
- `/wheel_ticks` (`mowgli_interfaces/WheelTick` is already in our tree, unused).
  **This one has no hardware source.** The wire carries only a body twist (§5);
  the MCU reports instantaneous per-wheel RPM, not cumulative counters, and
  `/odom` is open-loop integration. `WheelTick` would have to be synthesised by
  integrating RPM — we deliberately do not, because it would look like real
  encoder odometry when it is not.
- `/gps/absolute_pose` — needs a small node over `/gps/fix` + the datum. The
  WGS84→ENU projection helper already exists as `mowgli_interfaces`
  `wgs84_projection.hpp`.
- `/rtcm` — RTCM is not republished on ROS at all: the driver writes it
  straight to the receiver's serial port. (`/gps/corrections` is a 1 Hz JSON
  correction-health summary, not RTCM.)
- `~/set_firmware_debug` — no such vendor frame exists.

**Practical reading:** everything on the "already" list means you can run your
GUI and BT against our stack with remaps only. The "missing" list is where the
actual coding is, and only the first two items are cheap.

### 3.4 Adapter node (the only new code we would write first)

Once §3.2's two remaps are in, the things left that need a node are:

- **required:** convert your `TwistStamped` commands to our unstamped `Twist`
  (our whole chain — `twist_mux`, `cmd_vel_slew`, `cmd_vel_ws_relay`, the
  driver's `/cmd_vel` — is `Twist`). Note our patched GUI already hits this:
  when its cmd_vel relay (:8766) is not connected it falls back to publishing
  `/cmd_vel_teleop` as `TwistStamped` through `foxglove_bridge`, which does not
  match our `Twist` `twist_mux` input;
- the §3.3 producers (`/cmd_vel_applied`, `dig_event`, `/gps/absolute_pose`).

IMU QoS needs nothing: `/imu/data` is already RELIABLE(10).

The `/gps/status` type conversion turned out to be unnecessary:
`mower_gui_bridge` already publishes `mowgli_interfaces/GnssStatus`.

### 3.5 The MCU link: why `mowgli_hardware` cannot be used

This is the hard boundary, and it is not negotiable — the AT32 firmware is a
vendor blob we will not reflash.

| | MowgliNext | Airseekers Tron |
|---|---|---|
| Framing | `0x00` + COBS(payload + CRC16-CCITT) + `0x00`, 512 B max | `A5 \| len \| {sub_len type mod payload}+ \| sum&0xFF \| 5A` with `F5 01/02/03` byte-stuffing |
| Dispatch | packet-id | module-id (speed 4, charge 3, battery 5, imu 9, sensor 10, cutter 11, version 12, motors 13, calib 18, log 19, bms 24) |
| Direction | separate FW→host / host→FW packet ids | `type=0` host→MCU, `type=8` MCU→host, heartbeat `type=255` |
| Link | USB CDC `/dev/mowgli` @115200 | `/dev/serial_mower` → `ttyS9` @115200 8N1, **cutter board is the gateway**, chassis hangs behind it over a second UART |
| Version handshake | `0x11`/`0x12` semver, 5 s timeout | `version` sub-packet (mod 12) |
| Runtime PID/kinematics push (`0x54`–`0x57`) | yes | not available — gains are host-side |
| Firmware | yours, in-repo, PlatformIO | vendor blobs |

**So: port `mowgli_hardware`'s *architecture*, not its protocol.** What we
already adopted from it: one node per serial link, `~/` private topics with
launch remaps, read-tick vs publish-rate separation, reconnect with backoff,
rx/crc/overflow counters, `cmd_vel_slew` + applied-command echo, and the
RTK-gated dig detector. What we wrote ourselves: the `A5…5A` framing, escaping,
checksum and module dispatch in `mower_mcu_driver`.

Do not attempt to send MowgliNext firmware packets to these boards.

### 3.6 Distro gap

You target **ROS 2 Lyrical / Ubuntu 26.04**; we run **Humble** in a container
on the stock Ubuntu 20.04 host kernel (the mower OS is not being upgraded).

- Across DDS, Lyrical ↔ Humble interop works for identical `.idl` types — that
  is why §3.1 (two containers) is the recommended path and §3.6 does not block it.
- Any *native* run of your packages on our container needs a Humble port:
  API renames, your `docking_server` does not exist on Humble, and
  `CollisionMonitorState` is newer than ours. Re-check every file you port.
- The alternative — Ubuntu 26.04 container with Lyrical — is possible
  (containers carry their own userland) but puts both stacks on a glibc newer
  than the host; untested by us.

### 3.7 Localization: one deliberate divergence

MowgliNext runs a GTSAM iSAM2 factor graph and removed `robot_localization`.
We kept the EKF (`gps_gate` → `navsat_transform_node` → `ekf_node`, 20 Hz,
sole owner of `odom → base_link`) because:

- GTSAM needs an aarch64 source build and the RK3588 has 4 GB RAM;
- our three sources (wheel odom, IMU, RTK) are exactly the case an EKF handles;
- we have **no 2D LiDAR** — there is nothing to scan-match against.

Invariants kept regardless of which backend wins: sole TF ownership (no driver
broadcasts TF), `map` == GPS frame, `base_link` at the rear axle. If you want
GTSAM here, the swap point is `src/mower_localization/config/ekf.yaml` —
everything upstream of it (`gps_gate` on `/fix`, `/imu/data`, `/odom`) already matches what your
`fusion_graph_node` wants.

### 3.8 Known conflicts to resolve before any consumer runs

1. **IMU ownership — already settled.** `wit_imu_driver` (JY61P) owns
   `/imu/data`; `mower_mcu_driver`'s copy of the same data (the MCU echoes back
   what the host forwarded it) is deliberately on `/mcu/imu`, so there is no
   duplicate name. Note the MCU copy is BEST_EFFORT sensor QoS.
2. **TF from a driver — resolved.** `wit_imu_driver`'s `publish_tf` defaults
   to false and the launch files do not override it; `imu_link` comes from the
   URDF static transform.
3. **QoS — resolved.** `/imu/data`, `/battery` and `/odom` are RELIABLE(10).
4. **`map → odom` has no real publisher.** Currently identity (Phase A), so the
   stack localizes in `odom` only. Topology decision needed.

---

## 4. Reference: nodes, topics, services, actions, layout

> **Provenance note:** the stack was deliberately stopped while this document
> was written, and we are not restarting it — there is an open motion issue
> under investigation (§8.3). So the lists below are **derived from the launch
> files**, not captured from a live `ros2 node list`. We can produce a live
> capture (plus `ros2 topic/service/action list -t`) on request once motion
> testing is cleared.

### 4.1 Package and launch layout

```
ros2_stack/
├── launch/                     # mower_bringup installs these (src/mower_bringup/launch -> ../../launch)
│   ├── mower.launch.py         # THE entry point (group toggles below)
│   ├── bringup.launch.py       # drivers: mower_mcu_driver, wit_imu_driver, um960_gps_driver,
│   │                           #   bumper_controller, base_keys, light_controller, fill_light
│   ├── cameras.launch.py       # OA cameras, rear cam, Metoak stereo_cam / stereo_imu / stereo_depth
│   ├── camera.launch.py        # OpenCV fallback camera driver (mower_cameras/camera_node)
│   ├── nav2.launch.py          # localization only: gps_gate → navsat → ekf (Nav2 is mower_navigation)
│   ├── vio.launch.py           # ov_msckf + vio_odom_bridge
│   └── robot_settings.py       # launch-time settings loader
├── src/                        # 31 packages, 30 in scope (§2, §9)
├── third_party/                # mowglinext/ (vendored subset), open_vins/ (submodule); COLCON_IGNOREd
├── config/                     # calibration, cameras, gui, systemd, udev, urdf, vio, vio_metoak, vio_wit
├── docker/                     # Dockerfile.humble, docker-compose{,.gui,.video}.yml, dev-amd64, fields2cover
├── scripts/                    # build.sh, dev_build.sh, deploy_to_mower.sh, run_stack.sh, install_mower_service.sh, …
└── docs/                       # this file and ~30 others
```

Tests live in each package's own `test/` directory; there is no top-level
`test/`.

`mower.launch.py` group arguments (all `true` unless noted; `--show-args` for
the full list):

| Arg | Starts |
|---|---|
| `drivers` | `bringup.launch.py`: `mower_mcu_driver`, `wit_imu_driver`, `um960_gps_driver`, `bumper_controller`, `base_keys`, `light_controller`, `fill_light` (respawn 2 s) |
| *(always)* | `robot_state_publisher` from `config/urdf/mower.urdf.xacro` |
| `publish_static_map_odom` | identity `map → odom` (Phase A) |
| `control` | `cmd_vel_slew`, `slip_detector`, `imu_cal` |
| `supervisor` | `supervisor` (`mower_control`) |
| `mow_recorder` | `mow_recorder` (`mower_control`) |
| `teleop` | `twist_mux` + WebSocket `cmd_vel_ws_relay` |
| `gui_bridge` | `gui_bridge` (`serve_high_level` = `mission != true`) |
| `foxglove` | `foxglove_bridge` on :8765 |
| `localization` | `nav2.launch.py`: `gps_gate`, `navsat_transform_node`, `heading_aligner`, `ekf_node` |
| `navigation` | Nav2 (`mower_navigation/navigation.launch.py`) |
| `map_server` | `mower_map` `map_server_node` |
| `coverage` | `mower_coverage_bridge`: `mower_coverage_node` + `coverage_server` |
| `docking` | `mower_docking` |
| `mission` | `mower_mission` (`behavior_tree_node`) |
| `cameras` | `cameras.launch.py` |
| `stereo` | Metoak `stereo_cam`, `stereo_imu`, `stereo_depth` |
| `stereo_costmap` | stereo depth as a Nav2 local-costmap source |
| `vio` *(default false)* | `vio.launch.py` (needs `ov_msckf`, see §2.5) |
| `video` | `web_video_server` on :8080 |
| `perception` | `mower_vision/perception.launch.py`: `det_ros`, `obstacle_guard` (`seg_ros` off) |
| `det_range` | `det_range` (also requires `perception`) |

Included launch files: `bringup.launch.py`, `nav2.launch.py`,
`cameras.launch.py`, `vio.launch.py` (stack `launch/`), and from packages
`mower_coverage_bridge`, `mower_docking`, `mower_map`, `mower_mission`,
`mower_navigation`, `mower_teleop`, `mower_vision/perception`.
`um960_gps_driver` is a plain node in `bringup.launch.py`, not a sub-launch.

### 4.2 Nodes (from the launch files)

Names are the running **node names**; executables in parentheses where they
differ.

**Drivers:** `mower_mcu_driver` (`mcu_node`), `wit_imu_driver` (`wit_node`),
`um960_gps_driver` (`um960_node`), `bumper_controller`, `base_keys`,
`mower_cameras` (`camera_node`, `stereo_cam`, `stereo_depth`, `stereo_imu`),
`light_controller`, `fill_light`, `gui_bridge`.

**Control:** `cmd_vel_slew`, `slip_detector`, `imu_cal`, `twist_mux`,
`cmd_vel_ws_relay`, `supervisor`, `mow_recorder`.

**Localization:** `gps_gate`, `navsat_transform_node`, `ekf_node`,
`static_map_odom`, `heading_aligner`, `vio_gate`, `vio_odom_bridge`,
`stereo_vio_bridge`.

**Navigation / mission:** `planner_server`, `controller_server`,
`behavior_server`, `bt_navigator`, `velocity_smoother`, `costmap_filter_info_server`,
`lifecycle_manager_navigation`, `lifecycle_manager_costmap_filters`,
`map_server_node`, `mower_docking` (`docking_server`), `behavior_tree_node`
(`mission_node`, package `mower_mission`), `coverage_server`
(`coverage_action_server`), `mower_coverage_node`.

**Perception:** `det_ros` (`det_ros` / `det_ros_cpp`), `seg_ros`, `det_range`,
`obstacle_guard`.

**Infra:** `robot_state_publisher`, `foxglove_bridge`, `web_video_server`.

### 4.3 Topics (grouped)

| Area | Topics |
|---|---|
| Drive | `/cmd_vel` (`Twist`, unstamped), `/cmd_vel_raw`, `/cmd_vel_emergency`, `/odom`, `/mcu/{measured,commanded,sent}_speed` |
| Status | `/mower_base/status`, `/mower_sensor_info`, `/mower_base/button_info`, `/mower_base/key_pressed`, `/battery`, `/estop` |
| IMU | `/imu/data` (wit, 100 Hz), `/mcu/imu` (MCU echo), `/imu/temperature_c`, `/bias_status` |
| GNSS | `/fix` (10 Hz), `/fix_status`, `/vel`, `/heading`, `/nmea` (off by default), `/gps/corrections`, `/ntrip/status`, `/fix_gated`, `/odometry/gps` |
| Filtered pose | `/odometry/filtered`, `/odometry/filtered_map` (frame `map`) |
| Vision | `/left_oa_camera/{image_raw,camera_info}`, `/right_oa_camera/...`, `/rear_camera/...`, `/stereo_depth/{points,ground_plane,stats}` |
| AI | `/ai/det/detections`, `/ai/det/image_annotated`, `/<camera_ns>/image_annotated`, `/ai/det/{detections_ranged,obstacles,obstacle_points}`, `/ai/seg/{mask,traversability}`, `/vision/obstacle_close`, `/vision/obstacle_markers` |
| VIO | `/vio/*` |
| GUI contract | `/hardware_bridge/{status,emergency,power}`, `/behavior_tree_node/{high_level_control,high_level_status,start_in_area,clear_coverage_resume,coverage_resume_available}` (`dig_event`/`dig_escalated` are contract-only — not yet published, §3.3) |
| GUI relay | `/gps/fix` (2 Hz), `/gps/status`, `/wheel_odom` (5 Hz), `/gui/{pose,status,emergency,gnss_status,detections}` |
| Diagnostics | `/diagnostics`, `/mcu/{measured,commanded,sent}_speed`, `/mcu/imu` |

### 4.4 Services

| Service | Type | Node |
|---|---|---|
| `/cutter_control` | `mower_interfaces/CutterControl` | `mower_mcu_driver` |
| `/charging` | `mower_interfaces/ChargingControl` | `mower_mcu_driver` |
| `/clear_estop` | `std_srvs/Empty` | `mower_mcu_driver` |
| `/cutter_off` | `std_srvs/Trigger` | `mower_mcu_driver` |
| `/cutter/set_height` | `mower_interfaces/SetCutterHeight` | `mower_mcu_driver` |
| `/hardware_bridge/{mower_control,emergency_stop,reboot_board}` | `mowgli_interfaces/*`, `Trigger` | `gui_bridge` |
| `/behavior_tree_node/{high_level_control,start_in_area,clear_coverage_resume}` | `mowgli_interfaces/*`, `Trigger` | `behavior_tree_node` (`mower_mission`); `gui_bridge` stub only with `mission:=false` |
| `/navsat_to_absolute_pose/set_datum` | `std_srvs/Trigger` | `gui_bridge` |
| `/base_ble/init` | `std_srvs/Trigger` | `base_ble` |
| `/fill_light_control`, `/fill_light/set_auto`, `/fill_light/get_state` | `SetBool`, `SetBool`, `Trigger` | `fill_light` (`mower_mcu_driver/fill_light_node`) |
| `/supervisor/dump` | `std_srvs/Trigger` | `supervisor` (`mower_control`) |

### 4.5 Actions

| Action | Type | Server |
|---|---|---|
| `/mower_docking/dock`, `/mower_docking/undock` | `mower_interfaces/{Dock,Undock}` | `mower_docking` |
| `/plan_coverage` | `mowgli_interfaces/PlanCoverage` | `coverage_server` (`mower_coverage_bridge`); `mower_mission` is a client |
| `navigate_to_pose`, `follow_path`, `backup` | `nav2_msgs/action/*` | Nav2 |

### 4.6 Processes / boot

Host is Ubuntu 20.04 with the vendor services present but **stopped** —
`mower-base`, `mower-logic`, `mower-controller`, `mower-ros2` are all
`inactive`, and nothing but our container holds a serial device.

| Unit | Purpose |
|---|---|
| `mower-ros2.service` | `docker compose … up -d` / `down` for `mower_humble` (example in `config/systemd/`) |
| `mower-stereo-watchdog.service` | re-runs the Metoak init script when the front stereo delivers no frames |

Docker: data-root `/userdata/docker`, `fuse-overlayfs` storage driver,
`bridge: none`, default runtime `runc-nomqueue` (the mower kernel lacks
`CONFIG_POSIX_MQUEUE`), `DOCKER_BUILDKIT=0`. The workspace is bind-mounted
`ros2_stack → /work`; the install is symlinked back to `/work/src`, so **editing
Python and restarting the container is enough — no colcon rebuild.**
Images: `mower:humble` (device), `mower:humble-dev-amd64` (host dev/CI).

---

## 5. The RK3588 ↔ AT32 link, described behaviourally

Integration-relevant facts only; the recovered protocol documents are not part
of this handoff.

- **Port:** `/dev/serial_mower` → `ttyS9`, 115200 8N1, no flow control.
- **Topology:** RK3588 ↔ cutter board over UART, and the chassis board
  (wheels, bumper, lift, stop) hangs behind the cutter board on a second UART.
  One host serial port, three MCUs behind it. Firmware versions: cutter 0.6.36,
  chassis 0.6.34, BMS 1.18.0.
- **Framing:** a length-prefixed, byte-stuffed, additive-checksum packet with
  sub-packets inside a frame (the MCU batches several sub-packets per frame).
  Frames carry a type field for direction and a module field for the payload
  kind.
- **Payloads that matter to you:** speed (both directions, MCU reports ~60 Hz),
  battery (~1 Hz), sensors (lift/stop/bumper, ~100 Hz), cutter state, motor
  currents/RPMs, version, IMU (host→MCU), charge control.
- **Command semantics:** the host sends a body twist — `linear` m/s and
  `angular` rad/s — **there is no per-wheel command on the wire.** The MCU runs
  the wheel loop.
- **Heartbeat:** a host→MCU keepalive. **Its timeout/failsafe behaviour is not
  characterised — this is the single most important open safety unknown**, and
  it must be measured (with the vendor test tool, wheels-up) before any
  consumer of ours sends motion.
- Other serial devices on the box: `ttyS1` (WIT IMU), `ttyS4` (UM960 RTK),
  `serial_ble`, plus the vendor log capture at `/userdata/log/mcu.dat`.

---

## 6. Cameras, LiDAR, GNSS — ROS interfaces

**There is no LiDAR.** Obstacle depth comes from the Metoak stereo pair.

| Sensor | Device | Topics | Notes |
|---|---|---|---|
| Front OA stereo L/R | `/dev/left_oa_camera` → `video53`, `/dev/right_oa_camera` → `video44` | `/left_oa_camera/{image_raw,camera_info}`, `/right_oa_camera/...` | Also the YOLO inputs (§7) |
| Metoak hardware depth | `/dev/video11` | `/stereo_depth/points`, `/stereo_depth/depth/image_raw`, `/stereo_depth/ground_plane`, `/stereo_depth/stats` | Noise-filtered, RANSAC ground removal; the depth image feeds `det_range`, the points optionally the Nav2 local costmap (`stereo_costmap` toggle) |
| Rear camera | `/dev/rear_camera` → `video62` | `/rear_camera/{image_raw,camera_info}` | Also `http://<mower>:8080/stream?topic=/rear_camera/image_raw` |
| Stereo IMU | ICM-40608 (kernel `inv_icm42600` driver, IIO mode) | `/stereo_imu/data` (via `stereo_imu_node`), bias latched | VIO input — deliberately **not** `/imu/data` |
| RTK GNSS | UM960 on `ttyS4` (param `port`, default `/dev/serial_rtk`) | `/fix`, `/fix_status`, `/vel`, `/heading`, `/nmea`, `/gps/corrections`, `/ntrip/status`; RTCM in/out | NTRIP client **or** LoRa base; `gps_gate` republishes a fusion-worthy subset as `/fix_gated` |
| Chassis IMU | WIT JY61P on `ttyS1` | `/imu/data` (100 Hz), `/imu/temperature_c` | Owns `/imu/data`. EKF consumes `/imu/data_aligned` (see below) |
| Buttons | `/dev/input/event5` | `/mower_base/button_info`, `/mower_base/key_pressed` | `base_keys` |

A watchdog service re-initialises the stereo when frames stop.

**What fusion consumes:** `wit_imu_driver` fills `Imu.orientation` from the JY61P
0x53 angle frame with realistic per-axis covariances, and `heading_aligner`
re-publishes it as `/imu/data_aligned` with yaw in ENU (course-over-ground, dock
pose, persisted offset). `ekf_node` is configured `imu0: /imu/data_aligned`, not
the raw topic — subscribe to the aligned one too if you want to agree with our
filter. Caveat that survives any driver fix: the JY61P is a 6-axis unit, so its
yaw is gyro-integrated and drifts; there is no absolute yaw source yet.

---

## 7. YOLO: inputs, outputs, and how detections are consumed

**Input.** `det_ros` (or `det_ros_cpp`, the default — same node name `det_ros`,
config `det_ros/config/det.yaml`) subscribes to image topics, defaulting to
`/left_oa_camera/image_raw` and `/right_oa_camera/image_raw`, plus the
`extra_topics` array, which in the shipped `det.yaml` already adds
`/vio/right/image_color` (the front stereo right eye). Parameters: `models_dir` (falls back when
`/userdata/ros2/models/<file>` is absent), `classes`, `img_size` `[480, 640]`,
`obj_thresh` 0.25, `nms_thresh` 0.45, `core_mask`/`core_masks` for NPU core
pinning, `max_rate_hz` 5.0 per camera (0 = every frame), `dry_run` (stay alive
and publish nothing if rknnlite or the model is missing).

**Output.**

| Topic | Type | Meaning |
|---|---|---|
| `/ai/det/detections` | `vision_msgs/Detection2DArray` | one entry per box, class id → label via `det_ros/labels.py` |
| `/ai/det/image_annotated` | `sensor_msgs/Image` | debug; encoded **only when subscribed** |
| `/<camera_ns>/image_annotated`, e.g. `/left_oa_camera/image_annotated` | `sensor_msgs/Image` | per-camera, same on-demand pattern (what the GUI perception page streams) |

**Consumption.**

1. `det_range` fuses `/ai/det/detections` + `/stereo_depth/depth/image_raw` +
   `/vio/right/camera_info` + `/tf_static` → `/ai/det/detections_ranged`,
   `/ai/det/obstacles`, `/ai/det/obstacle_points` (this is what turns a 2D
   box into "how many metres away").
2. `obstacle_guard` applies the policy: it publishes a **latched** `String`
   policy, `/vision/obstacle_close` (`Bool`), an emergency `Twist` on the
   emergency topic, and `visualization_msgs/ImageMarker` on
   `/vision/obstacle_markers`; with `stop_on_close` it emits the zero
   burst and calls `/cutter_off`. Respawned with a 2 s delay.
3. Mission/BT layer sees the resulting state through
   `/behavior_tree_node/*` and `/hardware_bridge/*`.

Launch: `mower_vision/launch/perception.launch.py`, args `det_backend`
(`cpp`|`python`), `det`, `seg` (off by default — nothing consumes it),
`obstacle_guard`, `models_dir`, `dry_run`, `stop_on_close`. Models are
`*.rknn` on the RK3588 NPU at `/userdata/ros2/models/`.

Retraining is a separate pipeline (`docs/yolo_retrain_pipeline.md`) and is out
of scope here.

---

## 8. How navigation, docking and safety are split between Linux and the MCU

### 8.1 Linux (RK3588) owns

Localization (EKF or, if you prefer, GTSAM), Nav2 planning/control, coverage
planning, docking logic, the behavior tree / mission layer, the GUI, obstacle
perception, RTK/NTRIP, all calibration. **Linux decides where to go.**

### 8.2 MCU (AT32) owns

The wheel closed loop (current/FOD control with encoders), lift/stop/bumper
interlocks, the e-stop latch, battery and charging, blade enable, and the
failsafe when the host goes quiet. **The MCU decides whether it is safe to
move at all.** It accepts only a body twist — no per-wheel setpoints, no
runtime PID tuning over the wire.

### 8.3 The wire contract, and one open issue

- `/cmd_vel` → exactly one speed frame per received message. The vendor host is
  a **pass-through**: no timer, no PID, no timeout, and **no zero frame sent on
  its own while idle** (our measurement of the vendor host's transmit policy;
  the supporting capture is internal).
- Our driver follows that policy: one frame per `/cmd_vel`, three zeros
  100 ms apart (~200 ms in all) on stop/timeout/interlock, then **silence**, clamped ±0.3 m/s,
  host-side PID **removed** (the MCU owns the fast loop).
- **Open issue under investigation:** while an earlier build of our driver
  streamed a continuous `0,0` at 20 Hz, the wheels turned even though every
  commanded velocity was zero. With the host fully silent the wheels are
  still, and the MCU reports `linear=0`, `rpm=0` throughout — so the MCU is
  reacting to something in the host's transmission, not running a program of
  its own. The suspected cause is precisely the non-vendor behaviour in the
  previous bullet (streaming `0,0` instead of going quiet). **Motion testing is
  paused until this is closed.** Any MowgliNext consumer of `/cmd_vel` must
  inherit the same rule: do not stream zeros.
- **Safety layers, in order of authority:** MCU interlock (lift / stop /
  bumper / e-stop latch) → `bumper_controller` → `obstacle_guard` (zero burst +
  `/cutter_off`) → `cmd_vel_slew` (slew + exact-stop watchdog) →
  `twist_mux` (teleop vs autonomy priority) → `mower_gui_bridge` (refuses
  blade-on unless the high-level state is 2 or 4) → Nav2.
- `/clear_estop` is the only thing that releases a latched e-stop.

### 8.4 What that means for you

You get `/cmd_vel` with a normal Nav2 `twist_mux` in front of it, and you get
status back on `/mower_base/status` and `/mower_sensor_info`. Everything
safety-critical below `/cmd_vel` is ours and already enforced on both sides of
the link. The one thing we still need from the hardware side before declaring
the seam safe is the heartbeat-failsafe measurement in §5.

---

## 9. Licences

Our packages, so you can see at a glance what you are pulling in:

| Licence | Packages |
|---|---|
| **Apache-2.0** (21) | `base_ble`, `base_keys`, `det_ros`, `det_ros_cpp`, `mower_bringup`, `mower_cameras`, `mower_control`, `mower_coverage`, `mower_coverage_bridge`, `mower_docking`, `mower_gui_bridge`, `mower_interfaces`, `mower_lights`, `mower_lights_interfaces`, `mower_mcu_driver`, `mower_navigation`, `mower_rknn`, `mower_vision`, `seg_ros`, `stereo_vio_bridge`, `wit_imu_driver` |
| **BSD-3-Clause** (1) | `bumper_controller` |
| **MIT** (2) | `mower_localization`, `um960_gps_driver` |
| **GPL-3.0** (3) | `mower_teleop`, `mowgli_interfaces`, `mowgli_nav2_plugins` |
| **GPL-3.0-or-later** (3) | `det_range`, `mower_map`, `mower_mission` |

Total: 30 packages. (`mower_proto` also carries `<license>Apache-2.0</license>`
in its manifest but is excluded from scope per §0, because it is generated from
a vendor schema.)

**Compatibility:** GPLv3 (your project) can absorb our Apache-2.0 / BSD / MIT
code — that combination is fine, and the resulting combined work stays GPLv3.
The reverse is not true: nothing of ours may be relicensed away from its
notice. The packages that are *already* GPL-3.0 (`mowgli_interfaces`,
`mowgli_nav2_plugins`) are ports of your code and simply inherit your terms.

**Not covered by any of the above, and therefore not shared:** vendor firmware
and binaries, vendor ROS 1 sources, protocol material recovered from vendor
builds, `mower_proto`'s generated vendor schema bindings, credentials, maps and
calibration files (§0).

---

## 10. Open questions and risks

| # | Item | Note |
|---|---|---|
| 1 | **Licence** | MowgliNext is dual GPLv3 / commercial. Resolve the commercial question before any non-personal integration. |
| 2 | **Heartbeat failsafe** | Vendor `type=255` timeout behaviour uncharacterised — safety-critical, must be measured wheels-up before motion. |
| 3 | **Zero-command wheel motion** | §8.3, under investigation, motion tests paused. |
| 4 | **Distro** | Lyrical vs Humble: API renames, no `docking_server`, newer `CollisionMonitorState`. Side-by-side containers dodge this; a native merge does not. |
| 5 | **`mowgli_interfaces` drift** | Verify our copy matches upstream `faf658b` exactly, or DDS types will not interoperate. |
| 6 | **QoS + duplicate IMU** | Resolved (§3.8 items 1 and 3). |
| 7 | **TF ownership** | Resolved (§3.8 item 2) — no driver broadcasts TF. |
| 7a | **Command type + RMW** | Our `/cmd_vel` chain is unstamped `Twist` and we run Fast DDS; both need a decision/adapter before the two stacks talk (§3.1, §3.4). |
| 7b | **`behavior_tree_node` clash** | `mower_mission` and `mowgli_behavior` both use that node name; run only one. |
| 8 | **`map → odom`** | Still identity; the GPS-anchored publisher is the missing piece. |
| 9 | **UM960** | `universal-gnss` has no UM960 parser; our driver is the pragmatic path. |
| 10 | **RAM/CPU** | 4 GB total, already running vendor stack + stereo + NPU. GTSAM iSAM2 + Nav2 + BT together is the budget risk; measure before full bring-up. |
| 11 | **Build time** | GTSAM + Fields2Cover source builds on aarch64 are long — cache in a derived image. |

---

## 11. Suggested first step

Bench, wheels-up, no motion:

1. Start `mower_humble` (our drivers + `gui_bridge`).
2. Start `mowgli_ros2` with `mowgli_hardware` disabled and the §3.2 remaps.
3. `ros2 topic echo /gps/fix`, `/imu/data`, `/wheel_odom`, `/battery_state`,
   `/hardware_bridge/status` — if all five appear with sane values, the seam
   works and everything after it is adapter code.

Cost estimate for that step: the remap launch file and nothing else. The
adapter node (§3.4) and the §3.3 surface are the next increments.
