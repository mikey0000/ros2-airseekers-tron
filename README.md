# Airseekers Tron — ROS 2 (Humble) port stack

Baseline: MowgliNext (ROS 2 mower stack). Target: runs on the mower via Docker (Humble).

`docs/STATUS.md` is the dated status report; this file is the stable "how the stack is put
together" overview. Start here, then `--show-args` on the entry point for what is actually
launchable.

Source assets (read-only, do NOT modify):
- ../ros2_port_handoff/ (README.md first, then 00_docs, 01_mcu_protocol, 03_ros_interfaces, 10_ros_live_snapshot)
- ../mower_docs/ (02-ros2-migration.md, 05-maps-and-http-api.md, 10-hardware.md)

Write all new files under ros2_stack/ only.

## Getting everything

```bash
git clone --recurse-submodules <this repo>     # or, in an existing clone:
git submodule update --init                    # third_party/open_vins (rpng/open_vins, pinned)
./scripts/build_f2c.sh                         # mower-f2c:v3-amd64 (Fields2Cover, fetched from GitHub at a pinned ref)
ARCH=arm64 ./scripts/build_f2c.sh              # mower-f2c:v3-arm64, needed by docker/Dockerfile.humble
./scripts/dev_build.sh build && ./scripts/dev_build.sh test   # amd64 dev image + colcon
./gui/build.sh                                 # GUI image; needs VITE_MAPBOX_TOKEN in
                                               # third_party/mowglinext/gui/web/.env (copy .env.example)
./scripts/install_models.sh --runtime          # + librknnrt.so 2.3.0 from airockchip/rknn-toolkit2
```

**Not in this repo** (vendor material; cannot be redistributed):

| What | Needed for | How to supply it |
|---|---|---|
| NPU models (`best_*_0208.rknn`, `pplite-seg_*.rknn`, `model_1.rknn`) — vendor exports, proprietary | `det_ros`, `seg_ros` (perception) | Copy them off your own mower / vendor firmware into a directory, then `MODELS_SRC=<dir> ./scripts/install_models.sh`. Without them run `perception:=false det_range:=false`. |
| Vendor `.proto` sources | Only to *regenerate* `mower_proto` | Not needed to build: the generated `*_pb2.py` are committed. `generate_proto.sh` expects them in `../mower_docs/reference/proto/`. |
| A vendor `map.geojson` | One optional `mower_map` test | `MOWER_VENDOR_MAP_SAMPLE=<path>`; the test is skipped otherwise. |

## Rules on the mower (192.168.1.105)

The stack is deployed and run there — that is the point of the port — but within these limits:

- **Deploy target is `/userdata/ros2_stack`**, bind-mounted to `/work` in the container.
  The install is symlinked back to `/work/src`, so editing Python and restarting the
  container is enough — no colcon rebuild. Typical change: `rsync` the package, then
  `docker restart mower_humble`.
- **Never reflash the Artery AT32 MCUs** and never write vendor firmware. The boards keep
  their vendor blobs; only the host side is ours.
- **No host package changes**: no `apt`, no OS upgrade, no `systemctl` edits — except the
  opt-in `scripts/install_mower_service.sh`, which installs the boot unit from
  `config/systemd/mower-ros2.service.example`. The host stays Ubuntu 20.04 / Noetic;
  everything new runs in containers.
- **No motion commands without an explicit go-ahead.** Bench work is wheels-up.
- Build flags that must not change: `DOCKER_BUILDKIT=0` (the kernel lacks
  `CONFIG_POSIX_MQUEUE`), Docker data-root `/userdata/docker`, storage driver
  `fuse-overlayfs`, `bridge: none`, default runtime `runc-nomqueue`.
- The root filesystem `/` is 100% full — put everything under `/userdata`.

## Perception / VSLAM / NPU

Metoak VIO is replaced by **OpenVINS**; YOLOv8 + PP-LiteSeg run on the RK3588S NPU.
Full plan + blockers in [docs/perception_vio.md](docs/perception_vio.md). Launched by
`mower.launch.py perception:=true` → `src/mower_vision/launch/perception.launch.py`
(det_ros, seg_ros, obstacle_guard); `det_range` is a separate node (`det_range:=true`).

| Package | What |
|---|---|
| `mower_rknn` | shared rknn-toolkit-lite2 wrapper + preprocess |
| `det_ros` / `det_ros_cpp` | YOLOv8 `best_*_0208.rknn` → `/ai/det/detections`. Executables `det_ros` / `det_ros_cpp`, both launched as node name `det_ros`; `det_backend:=cpp` (default) uses the RKNN C API, `python` the fallback |
| `det_range` | detections + `/stereo_depth/depth/image_raw` + `/vio/right/camera_info` + `/tf_static` → `/ai/det/detections_ranged`, `/ai/det/obstacles`, `/ai/det/obstacle_points` |
| `mower_vision` | `obstacle_guard` — policy for what to do about a detection (emergency zero burst + `/cutter_off`), plus the perception launch file |
| `seg_ros` | PP-LiteSeg `pplite-seg…6cls.rknn` → `/ai/seg/{mask,traversability}` (`seg:=false` by default: nothing consumes it yet) |
| `mower_cameras` | `v4l2_cam` (OA cameras), `camera_node` (rear), `stereo_cam` (Metoak stereo → `/vio/{left,right}/image_raw`), `stereo_imu` (ICM-40608), `stereo_depth` (→ `/stereo_depth/points`, `/stereo_depth/depth/image_raw`) |
| `stereo_vio_bridge` | `vio_odom_bridge` (`/ov_msckf/odomimu` → `/odometry/vio`, runs with `vio:=true`) + legacy stereo/IMU capture (only with `vio_capture:=bridge`) |
| OpenVINS | git submodule `third_party/open_vins` (rpng/open_vins, pinned). `third_party/` is `COLCON_IGNORE`d, so the normal build skips it; `scripts/build_openvins_mower.sh` applies `config/vio/patches/*.patch` and builds `ov_core`/`ov_init`/`ov_msckf` on the mower |

OpenVINS configs: `config/vio_wit/` (default, `vio_imu:=wit`) and `config/vio_metoak/`
(`vio_imu:=metoak`), selected in `launch/vio.launch.py`. Camera bringup is
`launch/cameras.launch.py`.

## Running

**Dev build (x86_64 host, no mower):** `./scripts/dev_build.sh` builds the whole workspace in
the `mower:humble-dev-amd64` image (`./scripts/dev_build.sh build --packages-select <pkg>`,
`./scripts/dev_build.sh test`, `./scripts/dev_build.sh shell`). Every mode sets
`ROS_LOCALHOST_ONLY=1` and `ROS_DOMAIN_ID=77` (override with `DEV_ROS_DOMAIN_ID`) — a dev-host
run must never reach the robot's DDS graph over the LAN (it did once, on 2026-10-07, and
latched a boundary stop). Needs the `mower-f2c:v3-amd64` image (`./scripts/build_f2c.sh`).

**Deploy (from the dev host):** `./scripts/deploy_to_mower.sh sync|image|build|up|down|logs|shell|all`
rsyncs the tree to `/userdata/ros2_stack`, builds the image and runs colcon on the mower
(low parallelism, OpenVINS skipped), and starts the stack plus the GUI compose file
(`VIDEO=1` adds the RTSP relay). Details in [docs/deployment.md](docs/deployment.md).

**Device build (RK3588, arm64):** `./scripts/build.sh` builds `docker/Dockerfile.humble`
(`mower:humble`, needs the `mower-f2c:v3-arm64` image first: `ARCH=arm64 ./scripts/build_f2c.sh`)
and runs colcon inside it. Then either `docker compose -f
docker/docker-compose.yml up -d` (what `config/systemd/mower-ros2.service.example` does at
boot; `restart: unless-stopped`) or `./scripts/run_stack.sh [launch args]` in the foreground.
Calibration and maps persist under `/userdata/ros2` (maps also at `/ros2_ws/maps`).

**Extra compose files:** `docker/docker-compose.gui.yml` — `mower_gui` (MowgliNext GUI,
`http://<mower>:4006`, image from `./gui/build.sh`, see [docs/gui.md](docs/gui.md));
`docker/docker-compose.video.yml` — `mower_mediamtx` RTSP relay (`rtsp://<mower>:8554/rear`).
The GUI's sources are the vendored MowgliNext subset in `third_party/mowglinext/`
(`COLCON_IGNORE`d; the stack builds its own copy of `mowgli_interfaces` from `src/`).

**Entry point:** `ros2 launch mower_bringup mower.launch.py [arg:=value ...]`. `mower_bringup`
is launch-only: it installs `launch/` and `config/` (symlinked into the package; re-run colcon
after adding a file there).

It declares **36 arguments** of its own; `--show-args` (the authoritative list) prints ~128,
because it also lists the arguments of the launch files it includes. The group toggles:

| Arg | Default | Starts |
|---|---|---|
| `drivers` | `true` | `bringup.launch.py`: `mcu_node`, `wit_node`, `um960_node` (node names `mower_mcu_driver`, `wit_imu_driver`, `um960_gps_driver`), `bumper_controller`, `base_keys`, `light_controller`, `fill_light_node` (respawn 2 s) |
| `control` | `true` | `cmd_vel_slew` (`/cmd_vel_raw` → `/cmd_vel`), `slip_detector`, `imu_cal` |
| `supervisor` | `true` | `mower_control/supervisor`: node liveness (`/supervisor/status`), crash records |
| `mow_recorder` | `false` | legacy `mower_control/mow_recorder` (rosbag2 per mow); off: `bag_recorder` pins each mow from its ring |
| `teleop` | `true` | `mower_teleop/teleop.launch.py` (twist_mux + WebSocket relay) |
| `gui_bridge` | `true` | `mower_gui_bridge/gui_bridge` |
| `foxglove` | `true` | `foxglove_bridge` on `foxglove_port` (8765); `foxglove_all_topics:=false` advertises only the GUI's topic whitelist |
| `localization` | `true` | `nav2.launch.py` localization: `gps_gate`, `heading_aligner`, `navsat_transform_node`, `ekf_node` |
| `navigation` | `true` | `mower_navigation/navigation.launch.py` (Nav2: bt_navigator, controller/planner/behavior; FTC controller from `mowgli_nav2_plugins`) |
| `map_server` | `true` | `mower_map/map_server.launch.py` (zones, keepout mask, dock pose); `maps_dir` |
| `coverage` | `true` | `mower_coverage` (Fields2Cover planner node) + `mower_coverage_bridge` (`/plan_coverage` action); `cut_width_m`, `swath_overlap_m` |
| `docking` | `true` | `mower_docking/docking.launch.py` (`/mower_docking/dock`, `/mower_docking/undock`) |
| `mission` | `true` | `mower_mission/mission.launch.py` (the `/behavior_tree_node` mission layer) |
| `cameras` | `true` | `cameras.launch.py`: OA cameras (`mower_cameras/v4l2_cam`, `oa_driver`), rear (`camera_node`, `rear_driver:=opencv`) |
| `stereo` | `true` | `stereo_cam` + `stereo_imu` + `stereo_depth` (Metoak front stereo) |
| `video` | `true` | `web_video_server` MJPEG on :8080 |
| `perception` | `true` | `mower_vision/perception.launch.py` (`det_ros` + `obstacle_guard`; `seg_ros` with `seg:=true`); `det_backend`, `stop_on_close` |
| `det_range` | `true` | `det_range` (also needs `perception:=true`) |
| `publish_static_map_odom` | `true` | identity `map → odom` (Phase A: map == odom) |
| `vio` | `false` | `launch/vio.launch.py` (OpenVINS + `vio_odom_bridge`); `vio_capture` (`stereo_cam` default, or `bridge`) |

Plus `urdf_file`, `robot_settings_file` (GUI settings → node params, `launch/robot_settings.py`),
`imu_calibration_file` (default `/userdata/ros2/calibration/imu_calibration.yaml`),
`datum_lat` / `datum_lon` / `datum_yaw` / `datum_env_file` (map origin), `maps_dir`, and
`stereo_costmap` (feed `/stereo_depth/points` + `det_range` into the Nav2 local costmap).

Not launched by anything: `base_ble` (BLE bridge to the vendor app protocol, uses
`mower_proto`). Interface packages: `mower_interfaces`, `mowgli_interfaces`,
`mower_lights_interfaces`; `mower_proto` holds the vendor protobuf bindings.

## Localization and control

Localization and control, merged from `docs/mowglinext_baseline.md` §3, §4.3, §6 and §7.2.
Full write-up in [docs/localization_control_plan.md](docs/localization_control_plan.md).

**Localization** — `src/mower_localization`, slam-less, no `slam_toolbox`:

| Piece | What it is |
|---|---|
| `gps_gate` | Republishes `/fix` → `/fix_gated` only when the fix is worth fusing: `NavSatFix.status` at/above `min_fix_status`, a `used_fixes` whitelist, and a position covariance inside `max_position_covariance`. Decides from the message itself, never from a filter that already consumed the fix. No hardware needed. |
| `heading_aligner` | `/imu/data` → `/imu/data_aligned`, yaw in ENU (course-over-ground, dock pose, persisted offset). Latched `/heading_aligner/status` for the mission preflight. |
| `src/mower_localization/config/ekf.yaml` | 20 Hz `ekf_node`, `world_frame: odom`, `publish_tf: true`. `odom0` = `/odom`, `imu0` = `/imu/data_aligned`, plus `/odometry/gps`. Sole publisher of `odom → base_link`. (`vio:=true` overlays `ekf_vio.yaml`: odom2 = `/odometry/vio_gated`.) |
| `src/mower_localization/config/navsat.yaml` | 10 Hz `navsat_transform_node`, publishing `/odometry/gps` for the EKF. `broadcast_utm_transform: false`, `wait_for_datum: false`. |

Frames published today: `map → odom` (identity, `static_map_odom`),
`odom → base_link` (`ekf_node`), and `base_link → {imu_link, gps_link, base_footprint,
blade_link, cutter_link, bumper, wheels, stereo/OA/rear camera frames}` from
`robot_state_publisher` + the URDF. **No driver broadcasts TF** —
`wit_imu_driver publish_tf` defaults to `false`. Intended tree is
`map` == `gps` → `odom` → `base_link` (rear drive axle, not chassis centre) → `imu_link`,
`gps`, `blade_link`, anchored by the receiver alone with no SLAM back-end.

We deliberately diverge from the baseline here: MowgliNext runs a GTSAM iSAM2 factor graph
and removed `robot_localization` entirely. We keep the EKF, because GTSAM needs an aarch64
source build and our three sources (wheel odom, IMU, RTK) are the case an EKF handles well —
we have no 2D LiDAR for scan matching. Revisit if Metoak stereo depth becomes a second
back-end. Invariants kept regardless: sole TF ownership, map == GPS frame, `base_link` at the
rear axle.

**Control** — `src/mower_control`, three baseline §4.3 ports (details in
[docs/control.md](docs/control.md)): `cmd_vel_slew` (slew before the wire, exact-stop
watchdog, applied command as a log line — not yet the `/cmd_vel_applied` topic), `slip_detector`
(the dig detector — commanded vs measured twist, RTK-gated, latched), `imu_cal` (at-rest bias
cal with a persisted, validated file). Also `supervisor` (process liveness) and `mow_recorder`.
Not ported: the host-side wheel PID (the MCU owns the fast loop — see
[docs/wheel_control_semantics.md](docs/wheel_control_semantics.md)) and the deadband/gain
sanity gate (needs a runtime-tuning path the vendor protocol does not offer).

**Wiring** — `launch/nav2.launch.py` is localization only: `gps_gate` → `heading_aligner` →
`navsat_transform_node` → `ekf_node` (`static_map_odom` is in `mower.launch.py`). Nav2 itself is
`src/mower_navigation/launch/navigation.launch.py`. The header comment of `nav2.launch.py`
carries the frame tree with per-transform ownership.

**Open blockers, in order:**

1. **`map → odom` has no GPS anchor.** Phase A publishes it as identity (map == odom), so the
   stack localizes in `odom` only. Topology decision still needed.
2. **JY61P yaw is gyro-integrated, not earth-referenced**, so it drifts. `wit_imu_driver`
   reports it with a large `orientation_yaw_covariance` and `heading_aligner` applies
   course-over-ground / dock-pose offsets — but there is no absolute yaw source yet.
3. **The MowgliNext GUI contract is only partly wired.** `mower_gui_bridge` provides
   `/hardware_bridge/{status,emergency,power}`, `/gps/{fix,status}`, `/wheel_odom`,
   `/odometry/filtered_map` and the high-level services; `dig_event`, `cmd_vel_applied` and
   `wheel_ticks` are not published. See [docs/mowglinext_handoff.md](docs/mowglinext_handoff.md) §3.3.
4. ~~No URDF~~ — `config/urdf/mower.urdf.xacro` now provides `base_link → gps_link`,
   `base_footprint`, `blade_link` and the sensor/camera frames; the lever-arm values are still uncalibrated vendor numbers.
5. ~~`ros-humble-robot-localization` missing from `docker/Dockerfile.humble`~~ — added.

Resolved since the first status report: `wit_imu_driver` now fills `Imu.orientation` from the
JY61P 0x53 angle frame with realistic covariances (it used to leave the quaternion at all
zeros), and its `publish_tf` parameter defaults to `false` so the URDF owns `base_link → imu_link`.

## Tests

`./scripts/dev_build.sh test` (colcon inside the amd64 image) is the source of truth.
Host-side, without ROS: `python3 -m pytest -q src/<pkg>/test`, which the stubbed-ROS
`conftest.py` in each Python package makes work (C++ packages test only under colcon) — current results on 2026-10-08:

| Package | Result |
|---|---|
| `mower_mcu_driver` | 77 passed, 3 skipped (the skips need real `rclpy`) |
| `um960_gps_driver` | 96 passed, 2 skipped |
| `mower_localization` | 92 passed, 2 skipped |
| `wit_imu_driver` | 13 passed, 2 skipped |

Known host-side gaps (the image does not have them): `base_keys` fails to import `base_keys`
when pytest is run from the repo root, and `stereo_vio_bridge/test/test_odom_convert.py`
imports `rclpy` unguarded. Both pass under `dev_build.sh test`.

The three driver packages (`mower_mcu_driver`, `wit_imu_driver`, `um960_gps_driver`) all run
on the mower; `docs/STATUS.md` records what was verified there.
