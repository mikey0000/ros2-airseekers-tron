> **Historical document (superseded 2026-10-05).** The current state of the port, the fixed
> topic/frame contract and the roadmap live in `STATUS.md`. The full audits that replaced this
> analysis are in `audit_2026-10-05/` (`ros2_stack_audit.md`, `mowglinext_reuse_study.md`,
> `vendor_mission_layer.md`). Everything below is kept for history and is out of date in places
> (package count, `config/` contents, test counts).

# Handoff gap analysis — `ros2_port_handoff/` vs `ros2_stack/`

Date: 2026-10-05. Method: read `ros2_port_handoff/README.md` plus the top-level
(and second-level) file lists of every numbered subdir, and all of
`ros2_stack/src/*`, `ros2_stack/docs/*`, `launch/`, `scripts/`, `docker/`.
Nothing outside this file was modified.

`ros2_port_handoff/` is the read-only bundle captured from the mower
(firmware `v1.3.27-rc.1-tron+20260713`, 2026-10-05, 4,090 files, ~1.1 GB
incl. the excluded-identity dir). `ros2_stack/` is the ROS 2 Humble port
under construction (git repo, 11 commits, 8 source packages). This document
inventories each handoff subdirectory against what the stack already contains
and ends with a prioritised, path-concrete "bring into ros2_stack" list.

## Current state of `ros2_stack/` (baseline for the comparison)

| Package | Type | What it is | Tests |
|---|---|---|---|
| `src/mower_interfaces` | ament_cmake | **11 msg** (`MotorStatus`, `MotorInfo`, `MowerBaseMotorsInfo`, `MowerSensorInfo`, `ControlInfo`, `MowerBaseDevStatus`, `PlannerOption`, `BatteryHealthInfo`, `MotorControl`, `MowerBaseButtonInfo`, `MowerBaseDevInfo`) + **4 srv** (`CutterControl`, `ChargingControl`, `BumperControl`, `PlannerTaskSet`) | — |
| `src/mower_mcu_driver` | ament_python | `mcu_node.py` (A5…5A codec port of `mcu_frame_decode.py` + recovered structs), `fake_mcu.py`, `scripts/loopback_test.sh`. Publishes `/battery`, `/odom`, `/imu` (placeholder), `/mower_sensor_info`; subscribes `/cmd_vel`. **No services, no TF, no rain/keyboard** | 24 |
| `src/wit_imu_driver` | ament_python | `wit_node.py` — JY61P 0x55 frames → `/imu` (BEST_EFFORT) + `/imu/temperature_c`; broadcasts `base_link→imu_link`. **Blocker: `Imu.orientation` never populated (0x53 frame parsed but only debug-logged); all-zero covariances.** No `test/` dir in the tree (README claims 25 tests — stale) | 0 in tree |
| `src/um960_gps_driver` | ament_python | `um960_node.py` + `parsers.py` + `serial_port.py` (termios, no pyserial), `config/um960.yaml`, `launch/um960.launch.py`. `/fix`, `/vel`, `/heading`, `/fix_status`, `/nmea` | 47 |
| `src/mower_localization` | ament_python | `gps_gate.py` (`/fix`→`/fix_gated`), `config/ekf.yaml`, `config/navsat.yaml` | 25 |
| `src/mower_control` | ament_python | `cmd_vel_slew.py`, `slip_detector.py`, `imu_cal.py` | 0 |
| `src/base_keys` | ament_python | `base_keys_node.py` — evdev (`/dev/keyboard` → `event5`) → `/mower_base/button_info` (`MowerBaseButtonInfo`) | 0 |

Also present: `launch/bringup.launch.py` (the three serial drivers),
`launch/nav2.launch.py` (`gps_gate` → `navsat_transform_node` → `ekf_node` →
Nav2 controller/planner + lifecycle), `docker/{Dockerfile.humble,docker-compose.yml}`,
`scripts/{build,run_stack,preflight,setup_mower}.sh`, and an **empty top-level
`config/` dir**.

Deliberately absent everywhere in `ros2_stack/`: any URDF/xacro, any udev rule,
any systemd unit, any calibration YAML, any proto definition, any vendor SDK
binary, and drivers for BLE, cameras, stereo, LEDs, charge, network, LTE.

---

## 00_docs — the 2026-10-02 system survey

**Contains:** the 10-document survey (`01-system-overview` … `10-hardware`,
plus `README.md`), a `reference/` dir (`ble_develop.md`, `CutterControl.srv`,
`IotCmd.msg`, `IotNotice.msg`, `Locator{GetMapInfo,LoadMap,SaveMap}.srv`,
`MappingControl.srv`, `mower_map_geojson_README.md`, `openapi.json`,
`proto/` with 14 BLE/MQTT protobuf files, `tools/{mower_client.py,poc_13344.py}`).

**Already represented:** only by citation. `docs/deployment.md` cites
`02-ros2-migration.md`; `docs/mcu_protocol_spec.md` cites `10-hardware.md`;
`docs/yolo_retrain_pipeline.md` cites `03-yolo-perception.md`;
`docs/mowglinext_baseline.md` is the research successor of the reuse strategy
(`09-reuse-strategy.md`). The documents themselves are **not** in
`ros2_stack/`.

**Missing / reusable:**
- The 10 survey docs — the canonical system reference; everything in
  `ros2_stack/docs/` ultimately derives from them.
- `reference/proto/*.proto` (14 files) — the BLE/MQTT payload contract;
  also present (as a 19-file set) in `03_ros_interfaces/mower_proto/`.
- `reference/openapi.json` — the vendor HTTP API contract (needed if the
  HTTP/API surface is ever ported; `mower_docs/05-maps-and-http-api.md`).
- `tools/mower_client.py` — HTTP API client, useful for QA against the device.

## 01_mcu_protocol — the core MCU bus recovery

**Contains:**
- `protocol_definitions_recovered.h` (10.7 KB) — the stripped vendor header
  regenerated from DWARF in `libmower_sdk.a` (all struct layouts, offsets,
  both enums).
- `mcu_frame_decode.py` — the validated frame codec oracle (172,850 live
  frames, zero checksum errors).
- `sdk_static_lib/` — `libmower_sdk.a` (5.4 MB), extracted `.o` files with
  full DWARF, `nm` symbol list; `dwarf/` (raw type dumps + `dwarf2hdr.py`).
- `bin/` — stripped `mower_base_node` (4.3 MB) plus two standalone non-ROS
  test tools (`mower_sdk_test`, `mower_sdk_ncurses`).
- `libs/` — the 4 private `.so` the node links (`libbase_imu`,
  `libbase_keys`, `libbumper_controller`, `libpid_controller`).
- **`mower_base_pkg/`** (364 KB) — a complete **ROS 2 (Humble, ament_cmake,
  BSD-3-Clause) reimplementation** of the vendor driver: `mower_sdk/` with
  *real C++ source* (`protocol.cpp` 201 L, `protocol_manager.cpp` 197 L,
  `serial_device.cpp` 111 L + headers incl.
  `protocol/protocol_definitions.h`), a ROS-free core in
  `include/mower_base/` (`odometry.h`, `motors.h`, `dev_health.h` —
  heartbeat watchdog, `power_manager.h`, `dev_status.h`), a 577-line
  `mower_base_node.cpp` wiring the **full** ROS 1 contract (odom, battery,
  sensor info, bumper cloud, all services), `config/base.yaml`,
  `launch/{mower_base.launch,mower_base_debug.launch}`, `urdf/` (9 URDFs +
  `urdf.py`), `test/` (`fake_{mower,imu,serial}_port.sh`,
  `test_{battery,bumper,speed}.py`), `docs/aichw_mower.md` (Chinese design
  doc with mermaid class diagrams of the original Humble architecture),
  `docs/{publisher,subscriber}_example.md`.
- `base_imu_pkg/` — vendor ROS 2 package for the WIT JY61P: CMakeLists,
  package.xml, `launch/base_imu.launch`, `scripts/wit_imu_config.py`
  (configures the sensor: 100 Hz, 98 Hz bandwidth).
- `mcu_traffic_logs/` — two raw MCU→host captures (5.5 MB total).
- `ota_tools/` — `ota.py`, `install.py/.sh`, `mcu_m4_ota.py`,
  `wit_imu_config.py`. `mower_base_node_strings.txt`, `libbase_imu_strings.txt`.

**Already represented:**
- The protocol knowledge: `docs/mcu_protocol_spec.md` (165-line spec derived
  from the header + decode script) and `mcu_node.py` (faithful Python port of
  the codec, header comment points at both).
- Test approach: `fake_mcu.py` + `test/loopback_test.sh` reimplement the
  vendor's socat loopback idea.

**Missing / reusable:**
- The C++ `mower_sdk` source and the 577-line `mower_base_node.cpp` — the
  only complete reference for the **service** wiring (`/cutter_control`,
  `/charging`, `/enable_bumper`, `/clear_estop`, `/reset_odom`, `/poweroff`,
  `/factory_test_req`, `/fill_light_control`), the `DevHealthHandler`
  heartbeat watchdog, host-side odometry (`sensor/odometry.cpp`), and the
  MCU status→ROS enum mapping. `ros2_stack`'s driver implements none of the
  services.
- `config/base.yaml` — the vendor parameter surface (`rain_min/max` 2000/4000,
  linear/angular PID kp/ki/kd 0.9/0.3/1.0 & 0.9/0.3/0.5, `linear_max`
  ±0.3, `rotate_timeout`/`linear_timeout` 20 s, ports/baud). `mcu_node.py`
  has equivalent params inline but no file, and no rain thresholds at all.
- `wit_imu_config.py` — the JY61P register configuration; `wit_node.py`
  assumes the sensor is already configured.
- The vendor launch files (incl. the `robot_state_publisher` + URDF pattern)
  and `test/` harness.
- `mcu_frame_decode.py` itself (not copied — only ported) and the traffic
  captures (ideal regression data).
- `ota.py` + `mcu_m4_ota.py` document the bootloader protocol (reference
  only; never run).

## 02_mcu_firmware — MCU firmware binaries + OTA flasher

**Contains:** `mcu_bin_v1.3.27/` (chassis 0.6.14–0.6.34, cutter 0.6.21–0.6.36,
BMS 1.14.0–1.18.0, rtk_rover 0.7.x/0.8.x/0.9.x per board rev, rtk_rover_m4
LoRa `B039.01.02.dwn`, rtk_station, `emmc/emmc.img` DDR-frequency blob
flashed at eMMC sector 0x40, SN-gated), `ota.py`, `install.py/.sh`,
`mcu_m4_ota.py`, README; `older_rtk_rover_from_v1.0.60/`; `ota.py.v1.0.60`.
All Artery AT32 raw `.bin` (20–65 KB). Running versions: cutter 0.6.36,
chassis 0.6.34, BMS 1.18.0, rtk 0.9.50.

**Already represented:** the running-version facts, in
`docs/mcu_protocol_spec.md` §1. Nothing else — and that is correct: the
handoff caution is "do not flash anything until the driver works", and
`ros2_stack/README.md` forbids touching the mower.

**Missing / reusable:** nothing to bring in. Keep as read-only reference for
version-pinning checks (e.g. `mcu_node` could assert `VersionInfo` against
these once the driver runs). The `emmc.img` is platform-level (DDR config),
not a ROS concern.

## 03_ros_interfaces — vendor ROS interface packages

**Contains:** `mower_msgs` (52 msg + 32 srv + 4 action = 88 files),
`mower_gps_msgs` (10 msg incl. `UnicoreNav`, `UnicoreMessageHeader`,
`UnicoreExtendedSolutionStatus` + `SetLoRa.srv`), `mower_proto` (19 `.proto`
BLE/MQTT payloads + `cmake/protobuf-generate.cmake` + `docs/ble.md`),
`vanjee_lidar_msg`, and `devel_generated_headers/` (generated headers for the
binary-only packages: `mower_logic`, `mower_bt_nodes`, `mpc_controller`,
`global_planner`, `lidar_localization_ros`, `vanjee_lidar_sdk`).

**Already represented:** `src/mower_interfaces` — the core subset (11 msg +
4 srv listed above), with `docs/interfaces_map.md` mapping each conversion
and a **Deferred** list (48 msg, 32 srv, 4 actions). Note the map doc is
stale: it says "7 msg" but the tree has 11 (`BatteryHealthInfo`,
`MotorControl`, `MowerBaseButtonInfo`, `MowerBaseDevInfo` were added after
the doc was written).

**Missing / reusable:**
- The deferred interface definitions (see P2 list below for the priority
  order — `MowerLocalizationInfo` first, because the vendor base node
  subscribes to it).
- `mower_gps_msgs` — the full Unicore/GNSS contract (the UM960 driver
  currently publishes only standard types).
- `mower_proto` — the 19 protobuf definitions for the BLE/MQTT bridge.
- `vanjee_lidar_msg` — only relevant if a Vanjee LiDAR is ever added (the
  Tron has none).
- `devel_generated_headers/` — the only surviving interface surface of the
  binary-only packages; reference for a future C++ port of
  `mower_logic`/`mpc_controller`.

## 04_full_src_tree — the complete vendor `/workspace/src`

**Contains:** the full 14-package vendor tree (202 MB, no `.git`/`.rknn`):
`mower`, `mower_charge`, `mower_common`, `mower_controller`, `mower_drivers`
(base_ble, base_cameras, base_imu, base_keys, base_lidar, base_net, base_rtk,
base_stereo, mower_base), `mower_localization` (eskf, fusion, lidar, vision),
`mower_msgs`, `mower_perception` (det_ros, seg_ros, vslam_ros, rknn2_runtime),
`mower_planner` (astar, coverage, global, slic3r), `mower_pnc`, `mower_proto`,
`mower_simulation`, `mower_task` (bt_nodes, core, light_sound, logic, map,
mqtt), `perception/mower_explore`; plus `install.sh` (the OTA installer),
`profile/`, `build_version.yaml` (`1.3.27-rc.1-tron+20260713.cnb-2fg-1jtdt6r4s`).
Every `package.xml`, `CMakeLists.txt`, launch, param YAML and ~900 surviving
headers are here, plus the vendor SDK libs (Metoak `libMoGeneralSDK`
`sdk_v2745/lib`, `librknnrt.so`, Rockchip MPP `base_cameras/lib/mpp`,
`libws_2812.so`). 52 of 72 packages are binary-only.

**Already represented:** only as architecture reference — the MowgliNext
mapping in `docs/mowglinext_baseline.md` §2 and
`docs/mowglinext_integration.md` §2 was derived from it, and
`docs/yolo_retrain_pipeline.md` cites its `det_ros`/`seg_ros` internals.

**Missing / reusable:**
- The vendor PNC/logic/task/map/perception packages — the entire "later"
  layer of the stack (`mower_behavior`, `mower_coverage`, `mower_map`,
  `mower_nav2_plugins`, `mower_monitoring` in MowgliNext terms). All closed
  or header-only; they define the topic/service contract the drivers must
  ultimately serve (e.g. `mower_localization`'s `MowerLocalizationInfo`
  publisher, `mower_task`'s `mower_light_sound`).
- Param YAMLs and launch files of every closed node — the ground truth for
  what our launch files must reproduce.
- `install.sh` — **caution**: this is the OTA flasher that replaces
  `/workspace` and re-flashes MCUs; reference only, never run.
- `build_version.yaml` — the exact build identity (already quoted in the
  handoff README).

## 05_driver_binaries — closed hardware-abstraction nodes

**Contains:** 11 nodes with no source — `base_ble_node`,
`mower_gps_driver_node(+ntrip)`, `base_cameras_node`, `mower_light_sound_node`,
`base_keys_node`, `base_net_node`, `mower_charge`, `stereo_ros`,
`fake_rtk_node`, `fake_mower_node`, `gatt-service`; `libs/` with their private
`.so`; `ldd_report.txt` (external deps per binary).

**Already represented (reimplemented in `ros2_stack/src/`):**
- `base_keys_node` → `src/base_keys` (evdev → `/mower_base/button_info`).
- `mower_gps_driver_node` → `src/um960_gps_driver` (NTRIP half **not**
  ported — `docs/um960.md` §NTRIP covers what the vendor client does).
- `mower_base_node` → `src/mower_mcu_driver` (MCU half only; services,
  bumper cloud, rain ADC, fill-light PWM all still missing).
- `fake_mower_node` → `src/mower_mcu_driver/fake_mcu.py` (loopback only).

**Missing / reusable (no ROS 2 counterpart yet):**
- `base_ble_node` + `gatt-service` (BLE link to the app, `/dev/serial_ble`
  → ttyS3; framing source survives in `04_full_src_tree/src/mower_drivers/base_ble`).
- `base_cameras_node` (rear UVC + OA cameras, `/dev/{rear_camera,left_oa_camera,right_oa_camera}`).
- `stereo_ros` (Metoak "Archer" stereo, `/dev/videoSimor`/`videoIsp`;
  depth is SDK-only — `metoak_stereo_driver` is a known new-work item,
  `docs/localization_control_plan.md` §4).
- `mower_light_sound_node` (WS2812 on `/dev/spidev3.0` via `libws_2812.so`).
- `base_net_node`, `mower_charge` (dock/charge state machine),
  `fake_rtk_node` (simulator).
- `ldd_report.txt` — the dependency map for any future in-container reuse of
  these binaries (they need the 20.04 host ABI, so they stay host-side).

## 06_vendor_sdks — dockerless vendor SDKs

**Contains:** Metoak camera tools + kernel modules (`ko/`: `mo_simor`,
`mo_xc9080`, `mo_trig_flash`, `inv-icm42600`, `st_mag40`, `at24`,
`video_rkcif/rkisp`), the custom OpenCV 4.2 build (`metoak_opencv_4.2.0-gcc930/`,
i.e. `/usr/metoak/4.2.0-gcc930`), AP6256 BT/Wi-Fi firmware (`BCM4345C5.hcd`,
`brcm_patchram_plus1`, `fw_bcm43456c5_ag.bin`, `nvram_ap6256.txt`),
Rockchip `librga` 2.x + headers, and a README.

**Already represented:** nothing in `ros2_stack/` — correctly so. These are
**dockerless by necessity**: they are built for the stock 20.04/aarch64 host
ABI and the 5.10 BSP kernel, and the Humble container is a Jammy userland.
They must remain on the host (`/usr/metoak`, `/lib/modules/...`); the
container only needs them if a closed vendor binary is ever run host-side.

**Missing / reusable:**
- The AP6256 firmware + `brcm_patchram_plus1` — the alternate on-SoC BLE
  path (`bluetooth-ap6256-bridge.service`, currently disabled on the device).
  Only relevant if the external BLE UART module is ever replaced.
- The Metoak `ko/` modules + `mo_init.sh` — needed only if the stereo
  camera driver (`metoak_stereo_driver`) is ever ported; the module set is
  what makes `videoSimor`/`videoIsp` appear.
- `librga` + headers — 2D accelerator, only for image pipelines.
- Action: copy the README's *facts* (what each SDK is, where it lives on the
  host) into `docs/deployment.md`'s host-prerequisites section; the binaries
  themselves stay in the handoff.

## 07_system_config — udev rules, service units, host config

**Contains:** `udev/mower.rules` (the device-name mapping: `ttyS9→serial_mower`,
`ttyS1→serial_imu`, `ttyS4→serial_rtk`, `ttyS3→serial_ble`, `ttyUSB2→serial_lte`,
`video44/53→right/left_oa_camera`, `event5→keyboard`, saradc→`/dev/rain`),
18 `mower-*.service` units + `rosmaster.service` + `foxglove.service` +
`bluetooth-ap6256-bridge.service` + `unit_enablement_state.txt`,
`cron.d_mower`, `logrotate.d_mower`, `NetworkManager_conf.d/10-unmanaged.conf`,
`etc/99-mower.conf`, `mower_cam_keeper.py` (on-demand camera capture),
`userdata_mower_timezone`.

**Already represented:** only as *knowledge* — `docs/deployment.md` documents
the serial/device map and the compose `devices:` pins; `scripts/preflight.sh`
checks whether the vendor services (`mower-base`, `mower-logic`,
`mower-controller`) hold the ports. The **files themselves are absent** from
`ros2_stack/`.

**Missing / reusable:**
- `udev/mower.rules` — the container's `/dev/serial_*` symlinks only exist
  if this rule is installed on the host. This is a hard bringup dependency
  that currently lives only in the handoff.
- The service units — the templates for the stack's own units
  (`mower-humble.service` running `run_stack.sh`, plus a preflight/camera
  keeper equivalent); `unit_enablement_state.txt` records what to disable.
- `mower_cam_keeper.py` — the on-demand camera capture logic (start/stop
  capture services) that `docs/yolo_retrain_pipeline.md` §2.1 depends on.
- NetworkManager `10-unmanaged.conf` (keeps the modem/LTE interfaces out of
  NM), cron/logrotate snippets (log hygiene for `mcu.dat` etc.).

## 08_calibration_identity — calibration YAMLs, URDFs, camera identity

**Contains:** stereo intrinsics/extrinsics for SN `IC17SZ01011` (`cam0.yaml`,
`cam1.yaml`, `stereo_params.yaml` — PINHOLE_FULL model, 640×480, baseline
60.05 mm), the Metoak stereo EEPROM calibration dump
(`metoak_stereo_eeprom_calibration.data`), `base_cameras_config/`
(`camera_param.yaml`, `mower_cameras.yaml`, `left_oa_info.yaml`,
`right_oa_info.yaml`, `trinocular_camera_right_info.yaml`, `usb_cam_info.yaml`,
`usb_cam_zvcai.yaml`), and `mower_base_urdf/` — 9 URDFs (`m2-11`…`m2-16`,
`mower.urdf`, `prototype-1/3/5`) + `urdf.py` (selects by `HOSTNAME`; on the
device the hostname *is* the serial number, so this is SN-based in practice —
the handoff README's "picks by SN").

**Already represented:** **nothing.** This is the largest concrete gap:
`docs/localization_control_plan.md` §5 item 4 and `launch/nav2.launch.py`
both flag "no URDF yet" — without `base_link→gps`, `navsat_transform_node`
logs the lookup failure and silently assumes the receiver sits at the robot
origin (a lever-arm error), and without `base_link→imu_link` in the URDF the
only source of that transform is `wit_imu_driver`'s (Invariant-2-violating)
broadcast.

**Missing / reusable:**
- The 9 URDFs + the selection script — directly reusable for a new
  `mower_bringup` package (extend `urdf.py` to prefer SN over hostname).
- The stereo calibration triple — input for `14_vio_replacement` and any
  future `metoak_stereo_driver` / VIO back-end.
- `base_cameras_config/` — the camera node configs the closed
  `base_cameras_node`/`stereo_ros` run with (topic names, formats, rates).
- The EEPROM dump — the authoritative per-unit calibration; the YAMLs are its
  decoded form.

## 09_platform — platform facts

**Contains:** `device-tree.dtb` (256 KB, "decompile with `dtc -I dtb -O dts`"
— not yet done), `cmdline.txt`, `dt-serial.txt`, `dt_top_level_nodes.txt`,
`dt-model.txt`, `emmc_partition_map.txt`, `partitions.txt`, `df.txt`,
`meminfo.txt`, `uname.txt`, `os-release.txt`, `dpkg-l.txt`,
`ros_noetic_debs.txt`, `lsmod.txt`, `lsusb.txt`, `input-devices.txt`,
`i2c8-devices.txt`, `dev_nodes.txt`, `ttyS-sysfs.txt`, `tty_settings_live.txt`,
`udev-symlinks.txt`, `dmc_freqs.txt`.

**Already represented:** the load-bearing facts are already cited in
`docs/deployment.md` (OS, kernel, arch, disk, ROS debs, device nodes, tty
settings) and `docs/yolo_retrain_pipeline.md` (meminfo, df, lsusb).

**Missing / reusable:**
- The device-tree decompile — the UART/I²C/SARADC pinmux truth
  (`dt-serial.txt` is the summary; the full DTS would confirm e.g. that
  `ttyS9` is UART9 @ `0xfebc0000` and that nothing else claims it).
- The raw files stay in the handoff as the citable source; a short
  `docs/reference/platform_facts.md` in `ros2_stack/` would stop every future
  doc from reaching back into the bundle.

## 10_ros_live_snapshot — the running ROS 1 system, captured

**Contains:** `nodes.txt`, `topics_verbose.txt` (38 KB), `services.txt`
(6.3 KB), per-node pub/sub/service tables for the driver nodes
(`node_info_{base_driver_node,base_ble_node,mower_charge,mower_gps_node,mower_light_sound_node}.txt`),
one sample message per MCU-side topic (`sample__{odom,imu,battery,wheel_vel,mower_sensor_info,mower_base_status,mower_base_motor_info}.txt`),
and measured `topic_rates.txt`.

**Already represented:**
- `docs/mcu_protocol_spec.md` §10 — the topic/service inventory + rates
  transcribed from this snapshot.
- `docs/interfaces_map.md` §"Live topic / service reference".
- The sample messages informed the message conversions in `mower_interfaces`.

**Missing / reusable:**
- The sample messages as **test fixtures** (`ros2_stack/src/*/test/fixtures/`)
  — golden files for codec round-trip and publisher sanity tests.
- The per-node tables for `base_ble_node`, `mower_charge`,
  `mower_light_sound_node` — the contracts for the not-yet-ported drivers.
- `topic_rates.txt` — the acceptance criteria for driver publish rates
  (`/odom` `/imu` `/wheel_vel` 100 Hz vendor; our `mcu_node` runs `/odom`
  at 50 Hz by default — a deliberate divergence worth recording).

## 11_perception_models — the five NPU models

**Contains:** `best_large_0208.rknn` (28.8 MB), `best_small_0208.rknn`
(11.1 MB), `model_1.rknn` (2.9 MB, light classifier), `pplite-seg_20260606.rknn`
and `pplite-seg_20260630-1-6cls.rknn` (9.5 MB each). 59 MB total.

**Already represented:** the models' *I/O contract* is fully extracted in
`docs/yolo_retrain_pipeline.md` §1 (input `images` int8 NHWC 1×3×640×480;
9 outputs; 22 score channels; S = 80×60/40×30/20×15) — read out of the
`.rknn` metadata via `strings`.

**Missing / reusable:** the weights themselves — **do not bring into
`ros2_stack/`** (59 MB of binary blobs in a git repo; no source). Keep in
the handoff, reference by path. The conversion path (rknn-toolkit2 2.3.0) and
the retrain pipeline design are the reusable part, and that is already a doc.

## 12_excluded_secrets_identity — credentials, identity, owner data

**Contains (collected with `collect_excluded.sh`, redact=0):** `system/`
(`opt_mower_init.sh` with Tailscale auth key, `wpa_supplicant.conf`,
`meta_sn`, `meta_mac_addr`, `hostname`, `{airseekers,root}_authorized_keys`,
`auto.crt`/`auto.key`, `rosparam_dump.yaml`, `webshell_agent`),
`userdata/` (`ntrip.yaml`, `lte.yaml`, `mower_cloud_config.yaml`,
`mower_mqtt/{ca,cert,private,config.yaml}`), `RobotData/` (`robot.db`,
GeoJSON maps, reports, 4 rosbags, `vslam/` calibration copies, `track.txt`,
`active_map.json`). 488 MB.

**Already represented:** nothing — deliberately. `docs/deployment.md` and the
handoff README describe what was excluded and why.

**Missing / reusable:**
- The **redacted templates** (`opt_mower_init.sh`, `ntrip.yaml` with
  `<REDACTED>` placeholders) — useful as install checklists for whoever
  re-enables Tailscale/NTRIP on the device; safe to copy as `.example` files.
- `rosparam_dump.yaml` — the full live ROS 1 parameter set; the best
  available cross-check for our launch parameter defaults.
- The MQTT cert/key and device SN must be re-collected from the device;
  they are **not** to be committed anywhere.

## 13_open_source_upstreams — cloned upstreams (reuse strategy)

**Contains:** shallow clones (1.1 GB total) of `VINS-Mono` (replaces
`libcamera_models`/camodocal), `rknn_model_zoo` (NPU deployment for
`det_ros`/`seg_ros`), `open_mower_ros` (architectural reference for
`mower_logic`/`mower_map`/`slic3r_coverage_planner`), `teb_local_planner`
(replaces `teb_controller`), `polygon_coverage_planning` (BSD),
`spatio_temporal_voxel_layer` (LGPL costmap layer), plus a README with the
licensing table and "still to clone" list (PaddleSeg, ultralytics, WIT JY61P
samples).

**Already represented:** `docs/mowglinext_baseline.md` and
`docs/mowglinext_integration.md` are the MowgliNext-side successors of the
reuse strategy; `docs/yolo_retrain_pipeline.md` uses `rknn_model_zoo`. None
of the clones are wired into `ros2_stack/`.

**Missing / reusable:**
- `polygon_coverage_planning` (BSD — safe to link) → the coverage planner
  for `mower_coverage`.
- `teb_local_planner`, `spatio_temporal_voxel_layer` → Nav2 plugin wiring.
- `VINS-Mono` → the back-end for `14_vio_replacement`'s configs.
- `open_mower_ros` → **reference only** (GPL-3.0; architecture, not code).
- Keep as external clones/vendor dirs (`ros2_stack/third_party/…`), not
  copies inside the git repo.

## 14_proprietary_drivers — decompiled driver reimplementations

**Contains:** three rclcpp packages recovered from the stripped `.so`s:
- `base_keys/` (CMakeLists, package.xml, `include/base_keys/`,
  `src/{base_keys.cpp,base_keys_node.cpp}`) — the `libbase_keys.so` logic.
- `bumper_controller/` (`include/bumper_controller/`,
  `src/{bumper_controller.cpp,bumper_controller_node.cpp}`) — the
  `libbumper_controller.so` logic (bumper event handling; consumes the PID
  rotate primitive).
- `pid_controller/` (CMakeLists, package.xml, `include/pid_controller/`,
  `src/{pid_controller.cpp,pid_controller_ros.cpp,pid_controller_ros_node.cpp}`)
  — the `libpid_controller.so` logic; the standalone node is a minimal
  rotate/stop primitive, superseded by `opennav_docking` upstream.

**Already represented:**
- `base_keys` → `ros2_stack/src/base_keys` (Python reimplementation, same
  contract, keycode mapping exposed as params — the physical keycode mapping
  could not be recovered and needs `evtest` on device).
- `pid_controller` → **deliberately not ported**: `docs/control.md` §"Deliberately
  not ported" (the MCU owns the fast loop; closing the gap means
  reverse-engineering gains, not porting code).

**Missing / reusable:**
- `bumper_controller` — **no counterpart in `ros2_stack/`**. It owns the
  bumper event semantics behind `/bumper_cloud` and `/enable_bumper`
  (both in the vendor contract, `mcu_protocol_spec.md` §10).
- `pid_controller` as a reference for the host-side PID gains/structure
  (the vendor ran it against wheel-velocity feedback; `mcu_node.py` applies
  a plain scale+clamp and documents the gap).

## 14_vio_replacement — VINS-Fusion configs from the Metoak calibration

**Contains:** `generate_vio_config.py` (reads `08_calibration_identity/`
cam0/cam1/stereo_params + the fixed IMU↔cam0 extrinsic recovered from the
vendor VIO configs; reorders distortion coefficients from the vendor's
`{k1,k2,k3,p1,p2}` to OpenCV's `{k1,k2,p1,p2,k3}`) and its output
`config/{left.yaml,right.yaml,stereo_imu_config.yaml}`.

**Already represented:** nothing. VIO is the named fallback localization
back-end ("Revisit if Metoak stereo depth becomes a second back-end",
`docs/localization_control_plan.md` §1).

**Missing / reusable:** the generator + the three configs, ready to run with
`13_open_source_upstreams/VINS-Mono` once the stereo driver exists.

## Handoff top-level files

`MANIFEST.sha256` (640 KB, integrity manifest of the bundle), `TREE.txt`
(size summary per dir), `collect_excluded.sh` (the redaction collector —
reuse the redaction patterns), `mcu_frame_protocol_decode.log` (30 KB decode
log). Reusable: `MANIFEST.sha256` + `TREE.txt` as the bundle's own
inventory; `collect_excluded.sh` as the template for any future redacted
collection.

---

## Cross-cutting gaps (not owned by any single handoff dir)

1. **No services anywhere.** `mower_interfaces` defines `CutterControl`,
   `ChargingControl`, `BumperControl`, `PlannerTaskSet`, but no node in
   `ros2_stack/` exposes or consumes them (`mowglinext_integration.md` §3:
   "our `mower_interfaces` srv set is not yet wired to any node"). The vendor
   contract adds `clear_estop`, `reset_odom`, `poweroff`, `factory_test_req`,
   `fill_light_control` as std_srvs/Empty equivalents.
2. **Rain sensor missing entirely.** `/dev/rain` (SARADC `in_voltage4_raw`,
   thresholds 2000/4000) is read by the vendor `mower_base_node`; nothing in
   `ros2_stack/` touches it.
3. **Heartbeat unknown.** `mcu_node.py` ships `heartbeat_period: 0.1` as a
   *guess*; the real period/timeout/failsafe is the #1 safety unknown
   (`mcu_protocol_spec.md` §11) and must be characterised with the vendor's
   `mower_sdk_test` before any motion.
4. **`wit_imu_driver` orientation + covariances** — the stack-wide blocker
   (`localization_control_plan.md` §3); also publishes `/imu` while
   `mcu_node.py` publishes a placeholder `/imu` (duplicate publisher), and
   broadcasts TF against MowgliNext Invariant 2 (`mowglinext_integration.md`
   §3 conflicts 1–2).
5. **Open protocol questions** (`mcu_protocol_spec.md` §13): `BatteryInfo.current`
   units, `MotorInfo` units/per-board scaling, module id 7 semantics,
   `SensorInfoControl` semantics, `SendImuData` required-ness, cutter
   `MotorControl.speed` scale (1000 seen), `Heartbeat.year` epoch offset.
6. **`interfaces_map.md` staleness** — says 7 msg; the tree has 11 msg + 4 srv.
7. **`ros2_stack/README.md` test counts** — claims 25 tests for
   `wit_imu_driver`, but the package has no `test/` directory in the tree.
8. **No `config/` content** — the top-level `ros2_stack/config/` dir is
   empty; all configs currently live inside packages.
9. **Dependency inconsistency** — `wit_imu_driver` uses `pyserial` while
   `um960_gps_driver` uses raw `termios`; the Dockerfile installs
   `python3-serial`, so it works, but the two drivers should agree.

---

## Prioritised "bring into ros2_stack" list

Concrete target paths; `P0` = bringup blocker, `P1` = contract/safety
completeness, `P2` = full-contract / later phase, `P3` = reference only.

### P0 — bringup blockers

| # | From | To | Why |
|---|---|---|---|
| 1 | `07_system_config/udev/mower.rules` | `ros2_stack/config/udev/mower.rules` (+ install step in `scripts/setup_mower.sh`) | The container's `/dev/serial_mower|serial_imu|serial_rtk|serial_ble` symlinks exist only if this rule is on the host. Currently the file lives only in the handoff while `docs/deployment.md` assumes it. |
| 2 | `08_calibration_identity/mower_base_urdf/*` | `ros2_stack/src/mower_bringup/urdf/{m2-11..m2-16,mower,prototype-1,prototype-3,prototype-5}.urdf` + `urdf.py` (extend hostname picker to prefer `/meta/sn`) | No URDF exists anywhere in `ros2_stack/`; `navsat_transform_node` silently assumes the GPS antenna is at the robot origin (lever-arm error) and `base_link→imu_link` has no static source. New `mower_bringup` package (ament_python) also gets the `robot_state_publisher` launch the vendor `mower_base.launch` demonstrates. |
| 3 | `01_mcu_protocol/mower_base_pkg/config/base.yaml` | `ros2_stack/src/mower_mcu_driver/config/base.yaml` | Vendor parameter surface: `rain_min/max` 2000/4000, PID kp/ki/kd (reference only — host PID deliberately not ported), `linear/angular_max` ±0.3, timeouts. Load from `launch/bringup.launch.py`. Mark the unrecovered heartbeat values `(?)`. |
| 4 | `01_mcu_protocol/base_imu_pkg/scripts/wit_imu_config.py` | `ros2_stack/src/wit_imu_driver/scripts/wit_imu_config.py` (+ call it from a new `src/wit_imu_driver/launch/wit_imu.launch.py`) | The JY61P must be configured (100 Hz, 98 Hz BW) at startup; `wit_node.py` currently assumes a pre-configured sensor. |
| 5 | — (internal) | `ros2_stack/src/wit_imu_driver/wit_imu_driver/wit_node.py` | Fill `Imu.orientation` from the 0x53 frame as a quaternion and set realistic covariances; move TF to a URDF static transform. The stack-wide blocker per `localization_control_plan.md` §3. Not a handoff item, but it gates everything the handoff enables. |

### P1 — contract and safety completeness

| # | From | To | Why |
|---|---|---|---|
| 6 | `01_mcu_protocol/mower_base_pkg/mower_sdk/` + `src/mower_base_node.cpp` + `include/mower_base/` | `ros2_stack/docs/reference/mower_base_pkg/` (copy) **and/or** `ros2_stack/src/mower_sdk/` (new ament_cmake package porting `protocol.cpp`, `protocol_manager.cpp`, `serial_device.cpp`) | The only complete reference for the service wiring, heartbeat watchdog (`dev_health.h`), host odometry, and status-enum mapping. The Python driver already implements the codec, so this is either a reference copy or a C++ acceleration path. |
| 7 | `10_ros_live_snapshot/services.txt` + `mower_base_node.cpp` service handlers | `ros2_stack/src/mower_mcu_driver/mower_mcu_driver/mcu_node.py` | Wire the existing srv types: `/cutter_control`, `/charging`, `/enable_bumper` (→ `SensorInfoControl`), `/factory_test_req` (→ MCCalib), plus `std_srvs/Empty` `/clear_estop`, `/reset_odom`, `/poweroff`. |
| 8 | `01_mcu_protocol/mower_base_pkg/test/` | `ros2_stack/src/mower_mcu_driver/test/{fake_mower_port.sh,fake_imu_port.sh,fake_serial_port.sh,test_battery.py,test_bumper.py,test_speed.py}` | The vendor's socat loopback harness; complements `fake_mcu.py` + `loopback_test.sh`. |
| 9 | `01_mcu_protocol/mcu_frame_decode.py` + `mcu_traffic_logs/` | `ros2_stack/src/mower_mcu_driver/test/mcu_frame_decode.py` (+ `test/data/mcu.dat.*`, 5.5 MB) | The validated oracle (172,850 frames, 0 checksum errors) as a regression test and golden capture data. |
| 10 | `07_system_config/systemd/*` + `unit_enablement_state.txt` | `ros2_stack/config/systemd/mower-humble.service` (+ `mower-preflight.service`); `unit_enablement_state.txt` → `docs/reference/` | Replace the vendor units with ones that run `scripts/run_stack.sh` in the container; the state file records what must be disabled to free the serial ports. |
| 11 | `07_system_config/mower_cam_keeper.py` | `ros2_stack/scripts/mower_cam_keeper.py` | On-demand camera capture (start/stop services) — required by the YOLO capture design (`yolo_retrain_pipeline.md` §2.1). |
| 12 | `01_mcu_protocol/mower_base_pkg/src/mower_base_node.cpp` rain/keyboard handling | `ros2_stack/src/mower_mcu_driver/` (rain ADC reader; keyboard already covered by `src/base_keys`) | `/dev/rain` SARADC input with the `base.yaml` thresholds; part of the vendor node's auxiliary-device contract, currently missing entirely. |

### P2 — full contract and later phases

| # | From | To | Why |
|---|---|---|---|
| 13 | `03_ros_interfaces/mower_msgs/msg|srv|action` (deferred set) | `ros2_stack/src/mower_interfaces/{msg,srv,action}/` | Priority order: `MowerLocalizationInfo` (the vendor base node subscribes to it), `IotCmd`/`IotNotice` (BLE/cloud), `MowerControllerInfo`, `NavStatus`, then the `Map*`/`Planner*`/`TypePath` planning set, then the 4 actions (`Docking`, `NavigatePath`, `NavigateToPose`, `Undock`). Update `interfaces_map.md` (stale at 7 msg) in the same change. |
| 14 | `03_ros_interfaces/mower_gps_msgs/` | `ros2_stack/src/mower_gps_interfaces/` (new package) | Full Unicore contract (`UnicoreNav`, `UnicoreMessageHeader`, `Info`, `Gga`, `Gsv`, `Quality`, `Satellite`, `SetLoRa`) for GPS contract parity; also the source of truth for the BESTNAV field-layout discrepancies noted in `docs/um960.md`. |
| 15 | `03_ros_interfaces/mower_proto/` (19 `.proto`) + `00_docs/reference/proto/` | `ros2_stack/src/mower_proto/proto/` (+ `cmake/protobuf-generate.cmake`) | The BLE/MQTT payload definitions — prerequisite for any `mower_mqtt`/BLE bridge. |
| 16 | `04_full_src_tree/src/mower_drivers/base_rtk/ntrip_client/` + Microstrain `ntrip_client` ros2 branch | `ros2_stack/src/ntrip_client/` + `ros2_stack/config/ntrip.yaml.example` (template from `12_excluded_secrets_identity/userdata/ntrip.yaml`, credentials stripped) | RTCM correction injection on the same UART; `docs/um960.md` §NTRIP documents the vendor behaviour. Without it, `/fix` stays single-point (~2.5 m). |
| 17 | `14_proprietary_drivers/bumper_controller/` | `ros2_stack/src/bumper_controller/` (port, C++ or Python) | Bumper event semantics behind `/bumper_cloud` + `/enable_bumper`; no counterpart today. Keep `14_proprietary_drivers/pid_controller/` as `docs/reference/pid_controller/` only (host PID deliberately not ported). |
| 18 | `13_open_source_upstreams/{polygon_coverage_planning,teb_local_planner,spatio_temporal_voxel_layer}` | `ros2_stack/third_party/…` (git submodule or vendored copy) | Coverage planning (BSD, safe to link), TEB controller, costmap layer. `open_mower_ros` stays reference-only (GPL-3.0). |
| 19 | `08_calibration_identity/{cam0,cam1,stereo_params}.yaml` + `base_cameras_config/` | `ros2_stack/config/calibration/` | Stereo intrinsics/extrinsics + camera node configs for the future `metoak_stereo_driver` / VIO back-end. |
| 20 | `14_vio_replacement/` | `ros2_stack/scripts/generate_vio_config.py` + `ros2_stack/config/vio/{left,right,stereo_imu_config}.yaml` | VINS-Fusion configs ready for the fallback localization back-end. |
| 21 | `00_docs/mower_docs_survey_2026-10-02/` + `reference/` | `ros2_stack/docs/reference/survey/` (10 docs + `openapi.json` + `proto/` + `tools/mower_client.py`) | Make the stack self-contained instead of reaching back into the bundle; `openapi.json` is the HTTP API contract for a future API port. |
| 22 | `10_ros_live_snapshot/sample__*.txt` | `ros2_stack/src/*/test/fixtures/` | Golden messages for codec/publisher tests; `topic_rates.txt` becomes the rate acceptance criteria. |
| 23 | `00_docs/reference/{Locator*,MappingControl,CutterControl}.srv` etc. | covered by #13/#7 | Map save/load services belong with the `mower_map` phase. |

### P3 — reference only, do **not** bring in

| Handoff dir | Why not |
|---|---|
| `02_mcu_firmware/` (5.8 MB) | Flashing is forbidden until the driver works; version facts already in `mcu_protocol_spec.md` §1. |
| `04_full_src_tree/` (202 MB) | Architecture reference only; `install.sh` is the OTA flasher (caution). |
| `05_driver_binaries/` (54 MB) | Closed binaries needing the 20.04 host ABI; cannot run in the Jammy container. |
| `06_vendor_sdks/` (111 MB) | Dockerless host-side SDKs (Metoak OpenCV 4.2, kmods, AP6256 firmware, librga) — wrong ABI for the container; document in `docs/deployment.md` host prerequisites instead. |
| `09_platform/` (752 KB) | Facts already cited; pending action: `dtc -I dtb -O dts device-tree.dtb > ros2_stack/docs/reference/device-tree.dts`. |
| `11_perception_models/` (59 MB) | Binary weights; keep in the handoff, reference by path (I/O contract already extracted in `yolo_retrain_pipeline.md`). |
| `12_excluded_secrets_identity/` (488 MB) | Credentials, device identity, owner maps/bags. Only the redacted templates (`ntrip.yaml`, `opt_mower_init.sh`) are safe to copy, as `.example` files. |
| `13_open_source_upstreams/` (1.1 GB) | External clones — wire in as `third_party/` submodules, not copies. |

## Suggested first three commits

1. `config/udev/mower.rules` + `setup_mower.sh` install hook (P0 #1).
2. `src/mower_bringup/` with the 9 URDFs + `urdf.py` + a
   `bringup_full.launch.py` that adds `robot_state_publisher` to the existing
   driver bringup (P0 #2).
3. `src/mower_mcu_driver/config/base.yaml` + `src/wit_imu_driver/scripts/wit_imu_config.py`
   + the wit orientation/covariance fix (P0 #3–#5) — then re-run the pytest
   suites and update `README.md`/`interfaces_map.md` test counts and message
   counts in the same commit.
