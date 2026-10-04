# Airseekers Tron — ROS 2 (Humble) port stack

Baseline: MowgliNext (ROS 2 mower stack). Target: runs on the mower via Docker (Humble).

Source assets (read-only, do NOT modify):
- ../ros2_port_handoff/ (README.md first, then 00_docs, 01_mcu_protocol, 03_ros_interfaces, 10_ros_live_snapshot)
- ../mower_docs/ (02-ros2-migration.md, 05-maps-and-http-api.md, 10-hardware.md)

Write all new files under ros2_stack/ only.
Do NOT run anything on the actual mower (192.168.1.105); no apt changes, no flashing, no systemctl.

## Next batch

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
2. No `map → odom` publisher, so the stack localizes in `odom` only. Topology decision needed.
3. `/imu` vs `/imu/data` is settled only by a remap inside `nav2.launch.py`.
4. No URDF yet, so `base_link → gps` does not exist and navsat assumes the receiver sits at
   the robot origin.
5. `ros-humble-robot-localization` is not in `docker/Dockerfile.humble` yet.

None of the driver packages (`mower_mcu_driver`, `wit_imu_driver`, `um960_gps_driver`) has run
against the mower. Verified with pytest only: 24 + 25 + 47 tests respectively.
