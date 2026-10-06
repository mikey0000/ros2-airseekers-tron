# Visual-inertial odometry (VIO)

Goal: keep a usable pose when RTK drops (under trees, near walls) by fusing stereo VIO
**velocity** into the EKF while RTK is not FIXED. Status 2026-10-06: **wired, disabled
(`vio:=false` default), not yet built for arm64, not yet validated on real imagery.**
Do not enable it on the mower until the owner approves (section 6).

## 1. Pipeline

```
mower_cameras/stereo_cam ──/vio/{left,right}/image_raw (bgr8 640x480, best effort)──┐
wit_imu_driver ───────────/imu/data (100 Hz, gravity included)──────────────────────┤
                                                                                     v
                         ov_msckf/run_subscribe_msckf  (OpenVINS, config/vio_wit/)
                                    │ /ov_msckf/odomimu  (frame "global", twist in IMU body)
                                    v
                 stereo_vio_bridge/vio_odom_bridge  -> /odometry/vio
                    odom / base_link, body vx,vy,vz; var = max(10 x OpenVINS var, 0.04)
                                    │
                                    v
     mower_localization/vio_gate (+ /fix_status, /odom) -> /odometry/vio_gated, /vio_gate/state
                                    │
                                    v
           ekf_node odom2 (config/ekf_vio.yaml overlay): twist vx, vy only
```

| Piece | File |
|---|---|
| OpenVINS config for the WIT IMU | `config/vio_wit/{estimator_config,kalibr_imu_chain,kalibr_imucam_chain}.yaml` |
| (old) config for the Metoak ICM-40608 on `/vio/imu` | `config/vio/` (that IMU is not exposed, see 3.4) |
| OpenVINS patch (image QoS) | `config/vio/patches/0001-ov_msckf-sensor-data-qos-for-images.patch` |
| arm64 build | `scripts/build_openvins_mower.sh` |
| Producers launch | `launch/vio.launch.py` (args `capture`, `config_dir`, `cov_scale`) |
| Bridge | `src/stereo_vio_bridge/stereo_vio_bridge/{vio_odom_bridge,odom_convert}.py` |
| Gate | `src/mower_localization/mower_localization/{vio_gate,vio_gate_logic}.py` |
| EKF overlay | `src/mower_localization/config/ekf_vio.yaml` |
| Tests | `src/mower_localization/test/test_vio_gate.py`, `src/stereo_vio_bridge/test/test_odom_convert.py` |

Launch switch: `mower.launch.py vio:=true` includes `vio.launch.py` and passes `vio:=true`
to `nav2.launch.py`, which then starts `vio_gate` and gives `ekf_node` the
`ekf_vio.yaml` overlay. With `vio:=false` (default) the EKF node is exactly the old one
(only `config/ekf.yaml`), no VIO node starts, and `cameras.launch.py` behaves as before.
`vio_capture:=bridge` restores the legacy `stereo_vio_bridge` capture node (it then owns
`/dev/video22` and `stereo_cam` is not started); default `stereo_cam` shares the device
with `stereo_depth` and the GUI.

### Design choices

* **Velocity, not position.** OpenVINS' pose is in its own gravity-aligned frame with
  arbitrary yaw and an origin wherever it initialised; fusing it as absolute x/y/yaw
  would fight `/odometry/gps` and the aligned IMU yaw the moment RTK comes back. Body
  vx/vy are frame-free. vy is the valuable bit: the wheels do not measure lateral slide.
* **Low weight.** Wheel vx variance is 0.02; the bridge floors VIO at 0.04, so VIO has at
  most ~1/3 of the vx weight. `odom2_twist_rejection_threshold: 3.0` adds a Mahalanobis
  gate in the EKF itself.
* **Gate** (`vio_gate_logic.py`): passes only when (a) RTK is not FIXED for >= 1 s
  (from `/fix_status`: `quality=RTK_FIXED` or Unicore `solution=*_INT`; NavSatFix cannot
  tell FIXED from FLOAT, both map to `STATUS_GBAS_FIX`), or `/fix_status` is silent > 2 s;
  (b) >= 20 consecutive healthy VIO messages (no gap > 0.5 s, finite, |v| < 1.2 m/s,
  var < 0.25); (c) VIO vx agrees with wheel vx within 0.4 m/s (1 s of disagreement latches
  "diverged" until 3 s of agreement). Health is tracked while RTK is FIXED too, so VIO is
  warm when RTK drops. RTK FIXED blocks immediately. `/vio_gate/state` is a 1 Hz JSON
  summary.
* **No TF from OpenVINS** (`publish_global_to_imu_tf: false`, `publish_calibration_tf:
  false`): the EKF stays the only `odom -> base_link` owner; `map -> odom` is unchanged.

## 2. What exists on the mower (2026-10-06)

* `ov_msckf` is **not** in the mower image (`/work/install` has no `ov_*`), and
  `src/open_vins` on the mower is a dangling symlink (`ros2_port_handoff` is not synced).
* All build deps are in `mower:humble` (OpenCV 4.5, Eigen, Boost, cv_bridge,
  image_transport, message_filters).
* `stereo_cam` publishes `/vio/{left,right}/image_raw` (bgr8, best effort, stamped with
  host time, frame `vio_camera`); nobody publishes `/vio/imu`.

## 3. Findings from the 60 s docked bag

Bag: `/userdata/ros2/bags/vio_static_dock` on the mower (277 MB, copy in
`airseekers-decompile/bags/vio_static_dock`), 2026-10-06 ~23:07 local, state CHARGING.
Topics: `/vio/{left,right}/image_raw`, `/vio/*/camera_info`, `/imu/data`,
`/imu/data_aligned`, `/odom`, `/wheel_odom`, `/fix`, `/odometry/filtered`, `/tf_static`.

1. **It was night: the frames are pure sensor noise** (mean brightness 8.8/255). OpenVINS
   ran (patched amd64 build) and correctly never initialised ("not enough feats" /
   "platform moving too much" from noise features). Drift-at-rest and init success
   therefore could **not** be measured from this bag; repeat in daylight (5.1).
2. **Stereo rate is far too low.** `stereo_cam` delivered 2.6 Hz left / 2.1 Hz right
   (cap is 5 Hz) and only **1.7 Hz of exact-stamp stereo pairs**, with gaps up to 1.9 s.
   OpenVINS needs paired frames at >= 5 Hz, ideally 10 Hz, for a mower turning in place.
   Fix belongs in `mower_cameras/stereo_node.py` (not changed here; other agents own it):
   publish both eyes from the same capture as one unit, publish `mono8` for VIO (3x less
   conversion/serialisation), raise `stereo_fps` to 10 when `vio:=true`.
3. **WIT gyro reads exactly 0.0 at rest on all axes** (6198 samples). No software
   deadband in `wit_imu_driver`; the JY61P firmware clamps small rates. Slow rotations
   below the clamp are lost to VIO. Accel at rest: mean (-0.28, -0.07, 9.84) m/s^2 (1.7 deg
   tilt on the dock), std 0.010-0.036 m/s^2. Noise values in `config/vio_wit` are
   inflated accordingly; re-identify from the drive bag.
4. **Metoak's own IMU (ICM-40608, i2c 8-0068) is not exposed:** `inv_icm42600` is loaded
   but not bound (no IIO device). The original `config/vio` expected it on `/vio/imu`.
   Binding it (device-tree/new_device) would give a 200 Hz IMU rigidly on the camera;
   not attempted (hardware change).
5. **OpenVINS image QoS mismatch:** its `message_filters` stereo subscribers default to
   reliable QoS; `stereo_cam` publishes best effort -> ov_msckf would get no images.
   Patched (`config/vio/patches/0001-*.patch`, applied by the build script).
6. **Possible eye swap:** the Metoak factory calibration gives `T_cam0_cam1` x = -0.060 m
   (cam1 LEFT of cam0). Either `/vio/left` is physically the right eye or the naming is
   swapped. Check on the daylight bag (5.1): if stereo triangulation fails / features
   have negative depth, swap the two `rostopic` lines in `config/vio_wit/kalibr_imucam_chain.yaml`.
7. **Extrinsics:** `config/vio_wit` uses the URDF's measured `base_link ->
   stereo_camera_optical` (0.466 0 0.240, 0.9 deg down-tilt) for cam0 and the factory
   cam0->cam1 transform; the WIT IMU is identity at base_link (URDF TODO(calib)).
   `calib_cam_timeoffset: true` lets OpenVINS estimate the camera/IMU latency.

**CPU / memory (offline, amd64 dev box, 63.6 s bag at 1x):** ov_msckf used 3.5 CPU-s
(5 % of one x86 core) and 164 MB peak RSS, but it was only tracking noise and never
initialised, so this is a floor. Expect 20-50 % of one A76 core at 5-10 Hz stereo with
`num_opencv_threads: 1`, `num_pts: 150`, `max_clones: 8`, `max_slam: 30` (the budget is
< 1 core, < 300 MB). Measure on the mower in 5.2.

## 4. Building OpenVINS for the mower (arm64)

Never inside the running stack (load-190 incident). With the owner present:

```bash
./scripts/deploy_to_mower.sh down                      # stack STOPPED (mission CHARGING/IDLE_DOCKED)
SSHPASS=... ./scripts/build_openvins_mower.sh check     # refuses if mower_humble is up / < 3 GB free
SSHPASS=... ./scripts/build_openvins_mower.sh build     # 30-60 min, one-off container, -j1, 2.5 GB cap
./scripts/deploy_to_mower.sh up                         # vio stays off
```

The Python side (bridge entry point `vio_odom_bridge`, `vio_gate`, `ekf_vio.yaml` data
file) needs the usual `colcon build --packages-select stereo_vio_bridge
mower_localization` (pure Python, fine in the dev image or during the same stop).
amd64 validation: built in `mower:humble-dev-amd64` (4 min at -j6), patch applied, runs.

## 5. Test plan

### 5.1 Static, daylight, docked (no motion; any time)
Record with the stack running, mission CHARGING:
```bash
ros2 bag record -o vio_static_day --max-bag-size 1500000000 \
  /vio/left/image_raw /vio/right/image_raw /vio/left/camera_info /vio/right/camera_info \
  /imu/data /imu/data_aligned /odom /wheel_odom /fix /fix_status /odometry/filtered /tf_static
```
Offline (dev box): replay into ov_msckf (`ov_ws/tools/run_offline.sh`). Pass criteria:
mean brightness > 40; >= 80 features tracked; at rest OpenVINS will NOT initialise
(static init waits for an accel "jerk" by design), so this bag checks tracking and the
stereo pairing/eye order only.

### 5.2 First manual drive (owner, 2 min) — the bag we need
Owner drives with the remote (no mission), RTK FIX at the start, daylight:
0-10 s parked; drive straight 5 m; rotate in place 360 deg slowly; drive a 3x3 m square
back to start; if possible pass under a tree / along a wall so RTK drops to FLOAT; park
10 s. Same topic list as 5.1 plus `/odometry/gps`; `stereo_fps` raised to 10 if 3.2 is fixed.

What the bag should show when replayed through OpenVINS + bridge + gate:
* init within 1-2 s of the first jerk (driving off); `/ov_msckf/odomimu` at the camera rate;
* `/odometry/vio` vx tracks wheel vx within 0.1 m/s on the straight; vy ~ 0 +- 0.05 on flat
  ground; at rest |v| < 0.02 m/s (ZUPT on);
* VIO path (integrated in base_link) closes the square to < 0.5 m and < 5 deg yaw vs RTK FIX;
* under RTK FLOAT the gate log shows "PASSING VIO" after 1 s, "RTK fixed" when FIX returns;
  no "diverged" latch on normal driving;
* ov_msckf CPU < 1 core, RSS < 300 MB on the mower (measure with `top -p`, live run with
  `vio:=true` but the EKF overlay can be judged offline first).

### 5.3 Enable on the mower (owner approval)
`vio:=true` with a mission over an area with known RTK shadow; compare `/odometry/filtered`
against RTK on re-acquisition (jump size) with and without VIO.

## 6. Not done / needs the owner

* arm64 build (requires stopping the stack): `scripts/build_openvins_mower.sh build`.
* Daylight static bag + the 2-min manual drive bag (5.1, 5.2).
* Stereo pairing/rate fix in `stereo_cam` (3.2) — prerequisite for useful VIO.
* Decide whether to bind the Metoak ICM-40608 (3.4).
* Approval to run with `vio:=true` (EKF behaviour changes only then).
