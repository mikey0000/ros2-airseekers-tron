# Airseekers Tron — ROS 2 (Humble) port stack

Baseline: MowgliNext (ROS 2 mower stack). Target: runs on the mower via Docker (Humble).

Source assets (read-only, do NOT modify):
- ../ros2_port_handoff/ (README.md first, then 00_docs, 01_mcu_protocol, 03_ros_interfaces, 10_ros_live_snapshot)
- ../mower_docs/ (02-ros2-migration.md, 05-maps-and-http-api.md, 10-hardware.md)

Write all new files under ros2_stack/ only.
Do NOT run anything on the actual mower (192.168.1.105); no apt changes, no flashing, no systemctl.

## Perception / VSLAM / NPU

Metoak VIO is replaced by **OpenVINS**; YOLOv8 + PP-LiteSeg run on the RK3588S NPU.
Full plan + blockers in [docs/perception_vio.md](docs/perception_vio.md). Packages:

| Package | What |
|---|---|
| `mower_rknn` | shared rknn-toolkit-lite2 wrapper + preprocess |
| `det_ros` | YOLOv8 `best_*_0208.rknn` → `/ai/det/detections` |
| `seg_ros` | PP-LiteSeg `pplite-seg…6cls.rknn` → `/ai/seg/{mask,traversability}` |
| `stereo_vio_bridge` | Metoak stereo + ICM-42600 IMU → `/vio/*` (VIO input) |

OpenVINS configs live in `../ros2_port_handoff/14_vio_replacement/open_vins/`; the clone is
in `../ros2_port_handoff/13_open_source_upstreams/open_vins/`. OA/rear camera bringup is
`launch/cameras.launch.py`.

## Running

**Dev build (x86_64 host, no mower):** `./scripts/dev_build.sh` builds the whole workspace in
the `mower:humble-dev-amd64` image (`./scripts/dev_build.sh build --packages-select <pkg>`,
`./scripts/dev_build.sh test`, `./scripts/dev_build.sh shell`).

**Device build (RK3588, arm64):** `./scripts/build.sh` builds `docker/Dockerfile.humble`
(`mower:humble`) and runs colcon inside it. Then either `docker compose -f
docker/docker-compose.yml up -d` (what `config/systemd/mower-ros2.service.example` does at
boot; `restart: unless-stopped`) or `./scripts/run_stack.sh [launch args]` in the foreground.
Calibration and maps persist under `/userdata/ros2` (maps also at `/ros2_ws/maps`).

**Entry point:** `ros2 launch mower_bringup mower.launch.py [arg:=value ...]`. `mower_bringup`
is launch-only: it installs `launch/` and `config/` (symlinked into the package; re-run colcon
after adding a file there). Group toggles (`--show-args` for the full list):

| Arg | Default | Starts |
|---|---|---|
| `drivers` | `true` | `bringup.launch.py`: `mcu_node`, `wit_node`, `um960_node`, `bumper_controller` (respawn 2 s) |
| — | always | `robot_state_publisher` from `config/urdf/mower.urdf.xacro` |
| `publish_static_map_odom` | `true` | identity `map → odom` (Phase A: map == odom) |
| `control` | `true` | `cmd_vel_slew` (`/cmd_vel_raw` → `/cmd_vel`), `slip_detector`, `imu_cal` |
| `teleop` | `true` | `mower_teleop/teleop.launch.py` (twist_mux + WebSocket relay) |
| `gui_bridge` | `true` | `mower_gui_bridge/gui_bridge` |
| `foxglove` | `true` | `foxglove_bridge` on `foxglove_port` (8765) |
| `localization` | `true` | `nav2.launch.py` localization only: `gps_gate`, `navsat_transform_node`, `ekf_node` |
| `navigation` | `false` | `mower_navigation/navigation.launch.py` (Nav2) |

Also `urdf_file`, `imu_calibration_file` (default `/userdata/ros2/calibration/imu_calibration.yaml`).

## Localization and control

Localization and control, merged from `docs/mowglinext_baseline.md` §3, §4.3, §6 and §7.2.
Full write-up in [docs/localization_control_plan.md](docs/localization_control_plan.md).

**Localization** — `src/mower_localization`, slam-less, no `slam_toolbox`:

| Piece | What it is |
|---|---|
| `gps_gate` | Republishes `/fix` → `/fix_gated` only when the fix is worth fusing: `NavSatFix.status` at/above `min_fix_status`, a `used_fixes` whitelist, and a position covariance inside `max_position_covariance`. Decides from the message itself, never from a filter that already consumed the fix. 25 tests, no hardware. |
| `config/ekf.yaml` | 30 Hz `ekf_node`, `world_frame: odom`, `publish_tf: true`. Fuses `/odom` (x, y, vx, vy, yaw rate), `/imu/data` (yaw, yaw rate, ax, ay) and `/odometry/gps` as `odom1`. Sole publisher of `odom → base_link`. |
| `config/navsat.yaml` | 10 Hz `navsat_transform_node`, publishing `/odometry/gps` for the EKF. `broadcast_utm_transform: false`, `wait_for_datum: false`. |

Frames actually published today: `odom → base_link`, plus `base_link → imu_link` from
`wit_imu_driver`. `map → odom` has no publisher yet — see the blockers below. Intended tree is
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
watchdog, echo the applied command), `slip_detector` (the dig detector — commanded vs
measured twist, RTK-gated, latched), `imu_cal` (at-rest bias cal with a persisted,
validated file). Not ported: the host-side wheel PID (MCU owns the fast loop) and the
deadband/gain sanity gate (needs a runtime-tuning path the vendor protocol may not offer).

**Wiring** — `launch/nav2.launch.py` starts `gps_gate` → `navsat_transform_node` → `ekf_node`,
then the Nav2 controller stack. Its header comment block lists the required apt packages
(`ros-humble-robot-localization` is the only one not already in `docker/Dockerfile.humble`)
and the frame tree with per-transform ownership.

**Blocking, in order** (details in the plan doc §3):

1. `wit_imu_driver` never populates `Imu.orientation` — it parses the JY61P 0x53 angle frame
   and only debug-logs it — and publishes all-zero covariances. `navsat_transform_node` builds
   its heading from that quaternion, so the GPS correction is currently wrong and the EKF
   over-trusts the IMU. Fix the driver before tuning anything else.
2. No GPS-anchored `map → odom` publisher. Phase A: `mower.launch.py` publishes it as identity
   (map == odom), so the stack still localizes in `odom` only. Topology decision needed.
3. `/imu` vs `/imu/data` is settled only by a remap inside `nav2.launch.py`.
4. ~~No URDF~~ — `config/urdf/mower.urdf.xacro` now provides `base_link → gps_link`,
   `base_footprint`, `blade_link`; the lever-arm values are still uncalibrated vendor numbers.
5. ~~`ros-humble-robot-localization` missing from `docker/Dockerfile.humble`~~ — added.

None of the driver packages (`mower_mcu_driver`, `wit_imu_driver`, `um960_gps_driver`) has run
against the mower. Verified with pytest only: 24 + 25 + 47 tests respectively.
