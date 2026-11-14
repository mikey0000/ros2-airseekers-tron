# Visual-inertial odometry (VIO)

Goal: keep a usable pose when RTK drops (under trees, near walls) by fusing stereo VIO
**velocity** into the EKF while RTK is not FIXED. Status 2026-10-06: **wired, disabled
(`vio:=false` default), not yet built for arm64, not yet validated on real imagery.**
Do not enable it on the mower until the owner approves (section 6).

## 1. Pipeline

```
mower_cameras/stereo_cam ──/vio/{left,right}/image_raw (mono8 pair, 10 Hz, §7)───┐
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
| OpenVINS config for the Metoak ICM-40608 (`/stereo_imu/data`) | `config/vio_metoak/` (section 7; old `config/vio/` superseded) |
| OpenVINS patches (image QoS, deep IMU queue, serial-runner CMake option, Jazzy `.hpp` headers) | `config/vio/patches/000{1,2,3,4}-*.patch`, staged by `scripts/vio/stage_openvins.sh` |
| arm64 build | `scripts/build_openvins_mower.sh` |
| amd64 build (dev box, offline evaluation) | `scripts/vio/build_openvins_local.sh` |
| Recording + offline evaluation (section 8) | `scripts/vio/{record_vio_bag.sh,vio_preflight.py,eval_vio_bag.sh,eval_vio_bag.py,vio_offline.py}`, `config/vio/serial/ros2_serial_msckf.cpp` |
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

* `ov_msckf` is **not** in the mower image (`/work/install` has no `ov_*`). Sources are the
  `third_party/open_vins` git submodule (rpng/open_vins, pinned; `git submodule update --init`);
  `deploy_to_mower.sh sync` does not copy it, `build_openvins_mower.sh` stages a patched copy.
  (Older trees had a dangling `src/open_vins` symlink here; `sync` removes it.)
* All build deps are in `mower:jazzy` (OpenCV 4.5, Eigen, Boost, cv_bridge,
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

Never inside the running stack (load-190 incident). With the owner present; the exact,
re-checked sequence is in section 8.6:

```bash
./scripts/deploy_to_mower.sh down                      # stack STOPPED (mission CHARGING/IDLE_DOCKED)
SSHPASS=... ./scripts/build_openvins_mower.sh check     # refuses if mower_jazzy is up / < 3 GB free
SSHPASS=... ./scripts/build_openvins_mower.sh build     # 30-60 min, one-off container, -j1, 2.5 GB cap
./scripts/deploy_to_mower.sh up                         # vio stays off
```

The Python side (bridge entry point `vio_odom_bridge`, `vio_gate`, `ekf_vio.yaml` data
file) needs the usual `colcon build --packages-select stereo_vio_bridge
mower_localization` (pure Python, fine in the dev image or during the same stop).
amd64 validation: built in `mower:jazzy-dev-amd64` (4 min at -j6), patch applied, runs.

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

* arm64 build (requires stopping the stack): `scripts/build_openvins_mower.sh build` (8.6).
* Daylight static bag + the 2-min manual drive bag (5.1, 5.2): superseded by the 2026-11-14
  recording session (section 8), which records both drive and mow in one go.
* ~~Stereo pairing/rate fix in `stereo_cam` (3.2)~~ done 2026-10-07 (section 7.1).
* ~~Bind the Metoak ICM-40608 (3.4)~~ done 2026-10-07 (section 7.2); persisting the two host
  settings across reboots needs approval (7.4).
* Approval to run with `vio:=true` (EKF behaviour changes only then).

## 7. Stereo pair + Metoak IMU, clean room (2026-10-07)

### 7.1 Synchronised 10 Hz mono8 pair (`mower_cameras/stereo_cam`)

* `/vio/{left,right}/image_raw`: mono8 640x480, both eyes cut from ONE `/dev/video22` buffer
  (Y plane only, no colour conversion, straight into pre-serialised Images), SAME header stamp =
  V4L2 buffer timestamp (kernel CLOCK_MONOTONIC) mapped to ROS time (`stereo_pair.ClockMap`).
  `/vio/{left,right}/camera_info` (factory cam0 / cam1) carry the same stamp.
  `/vio/right/image_color` (bgr8, 3 Hz, only while subscribed) feeds det_ros (`det.yaml`
  `extra_topics`); `publish_color:false` turns it off. The GUI tiles show the mono8 topics.
* Rate: `fps` 10 is an average decimation on buffer stamps (`DecimationGate`): the source runs
  23.8 Hz (XC9080 subdev says 30 fps, trigger IRQ `asic1_trig` 25 Hz), kept frames alternate
  84/126 ms.
* **Where the rate was lost** (old node: 2.6/2.1 Hz, 1.7 Hz pairs): not CPU, not capture. The
  container runs Fast DDS UDP-only (`fastdds_no_shm.xml`) and the host capped socket buffers at
  `net.core.rmem_max = 212992`: one 300 KB (mono8) / 921 KB (bgr8) image fills a subscriber's
  receive buffer and the second eye, sent right after it, is dropped. Measured with the new node
  before/after `rmem_max/wmem_max = 8 MiB`: right eye 2.9 -> 10.0 Hz, exact pairs 2.9 -> 10.01 Hz.
  `scripts/setup_stereo_host.sh` sets it (runtime only). The old node additionally converted both
  eyes to BGR in Python and used a 1-slot hand-off that could split pairs.
* Live (docked, stack running, 40 s): 401 exact-stamp pairs = 10.01 Hz, 0 unpaired, max gap
  126 ms, 401/401 camera_info stamps match; stamp -> subscriber receipt 25 ms median.
  CPU `stereo_cam` 7.2 % of one core (incl. 3 Hz colour for det_ros); `stereo_depth` 20.4 %
  (18.5 % before: unaffected, separate `/dev/video11` node).

### 7.2 Metoak IMU (ICM-40608) -> `/stereo_imu/data` (`mower_cameras/stereo_imu`)

* Why it looked "loaded but not attached": `8-0068` IS bound (i2c driver name `icm40608` =
  `inv_icm42600_i2c`), but `mo_init.sh` (cam.service) loads the Metoak build of `inv_icm42600`
  with `Repot_m=1` = **netlink report mode**: samples go to a vendor netlink socket, no IIO
  device. (`modinfo`: `Repot_m: 0 - iio, 1 - netlink`; `TimeStamp_m: 1 - ktime_get_ts64`.)
  Unbind/bind fails (`netlink_kernel_create error`, probe -1: the socket leaks); reloading the
  module with `Repot_m=0` gives `icm40608-gyro` + `icm40608-accel` IIO devices.
  `scripts/setup_stereo_host.sh` does it (idempotent, `revert` restores netlink mode).
* Driver quirks found: timestamps are CLOCK_MONOTONIC regardless of `current_timestamp_clock`
  (says realtime); every sample of one FIFO interrupt group (4-9 samples, ~20 ms) carries the
  IRQ time; gyro and accel interrupt separately; one read can split a group. `iio_imu.BatchStamper`
  holds back the open group, spreads closed groups at the ODR estimated over ~8 s (true 200.78 Hz)
  with a low-gain phase loop (IRQ latency jitter 0.2-3 ms -> 0.15 ms), then gyro/accel are paired.
* Node: sysfs setup (200 Hz, +-500 dps, +-8 g, x/y/z + timestamp), `/dev/iio:deviceN` buffers
  (no sysfs polling), gyro bias from the first still 3 s, pre-serialised Imu publishing,
  best-effort depth-200 QoS. Frame `stereo_camera_imu` (URDF: factory body_T_cam0 inverted).
* Live (deep-queue subscriber, 30 s): 200.35 Hz, dt p1/median/p99 = 4.89/5.00/5.03 ms, max
  5.1 ms, 0 non-monotonic. Bias -0.0074/-0.0093/-0.0018 rad/s. At rest: gyro std
  0.00066/0.00093/0.00061 rad/s (= 4.7e-5/6.6e-5/4.3e-5 rad/s/sqrt(Hz), datasheet 6.6e-5), accel
  std 0.022/0.019/0.015 m/s^2, mean (-0.024, -9.782, 0.318), |a| 9.787. CPU 7.3 % of one core.
  Receipt latency 37 ms median (one held-back interrupt group + 4-sample watermark).
* OpenVINS subscribes IMU with `SensorDataQoS()` (depth 5); IMU leaves in groups of 4-9:
  `config/vio/patches/0002-ov_msckf-deep-imu-queue.patch` (depth 1000).

### 7.3 Clean-room check (no Metoak SDK)

| VIO needs | Ours | Status |
|---|---|---|
| synced stereo pair, 10 Hz | one side-by-side buffer -> both eyes, same stamp | yes, 10.01 Hz, 0 unpaired |
| mono8 | Y plane of the YUYV/YVYU frame | yes |
| camera_info / rectification | factory cam0/cam1 K + plumb_bob, stamps match; R/P identity (OpenVINS uses K/D + extrinsics; image_proc stereo rectification would need `stereoRectify` from stereo_params R/T, not done) | yes for OpenVINS |
| IMU >= 100 Hz | ICM-40608 via kernel IIO, 200 Hz | yes |
| IMU/image same clock | both kernel CLOCK_MONOTONIC (V4L2 buffer ts / IIO ts), same ClockMap | yes; residual constant offset unknown (below) |
| hardware disparity | `/dev/video11` decode (stereo_depth) | yes, unchanged |

How the vendor did it (strings of `libMoGeneralSDK.so` 2.7.4.5 and `stereo_ros`): the IMU is
NOT read over USB/UVC on this MIPI module. The SDK talks to the ICM-406xx from userspace over
i2c-dev (`/dev/i2c_imu` -> `/dev/i2c-8`, InvenSense eMD `inv_icm406xx_*` incl. FIFO,
`enable_fsync`, `timestamp_resolution`), applies an IMU rectification matrix stored on the module
(`moLocalGetImuRectifyParameter`, `IMU_RECT_MATRIX_*`), and aligns image stamps with the frame
trigger (`MO_ENV_TRIGGER_TIMESTAMP_ADJUST`, `getNearestTimeStamp`; trigger timestamps from
`mo_trig_flash.ko` -> `/dev/trig_flash`, `asic1_trig` IRQ). `MoUVCMsgManager` exists for UVC
variants only. `mo_iio_test` shows Metoak also used the IIO path.

What we cannot do (yet) and the compensation:
* **Exposure-time image stamps / FSYNC.** We stamp at the rkcif buffer (DMA) time, the vendor
  could tag the trigger. The offset is constant per mode (exposure/2 + readout + ISP); OpenVINS
  `calib_cam_timeoffset: true` estimates it (stays on in vio_metoak). `/dev/trig_flash` returns
  EINVAL to plain `read()` (needs an unknown ioctl/size): reverse it if the online estimate is
  unstable. The IMU stamps carry the mean IRQ latency (~1-2 ms), also absorbed there.
* **Exposure/gain control.** Not exposed through V4L2 on video22 (the XC9080 runs its own AE;
  vendor used `mo_asicrw` i2c pokes). Accept auto exposure; `histogram_method: HISTOGRAM`.
* **IMU factory rectification matrix** (misalignment/scale, in the module EEPROM blob, format
  unknown): identity used; residual misalignment ~1 deg (dock tilt check). Could be estimated by
  OpenVINS `calib_imu_intrinsics` on a rich drive, left off.
* Host settings are runtime-only (7.4).

### 7.4 Persisting across reboot (needs owner approval: vendor/host files)

* `/usr/metoak/metoak/mo_init.sh` line 49: `Repot_m=1` -> `Repot_m=0` (or run
  `setup_stereo_host.sh` from a unit after `cam.service`).
* `/etc/sysctl.d/90-ros2-dds.conf`: `net.core.rmem_max=8388608`, `net.core.wmem_max=8388608`.
Until then: run `sudo /userdata/ros2_stack/scripts/setup_stereo_host.sh` after each boot, then
restart `mower_jazzy` (stereo_imu retries with back-off until the IIO devices exist; DDS
participants keep the buffer size they were created with).

### 7.5 VIO with the Metoak IMU (not enabled)

`vio.launch.py vio_imu:=metoak` -> `config/vio_metoak/` (factory T_imu_cam, `/stereo_imu/data`
200 Hz, measured noise x2-3) and the bridge with `imu_to_base_rpy [-1.586797, 0, 1.570796]`,
`imu_xyz_in_base [0.451, -0.0514, 0.2352]` (lever arm removed: turning in place at 0.5 rad/s
would otherwise read as 0.23 m/s lateral slip). Chain check: the measured IMU gravity maps to
base (-0.47, -0.02, 9.78) m/s^2 (level in roll, 2.8 deg nose-down on the dock).

## 8. Recording session (2026-11-14)

Step 2 of "keep mowing through RTK loss". The dead-reckoning study
(docs/analysis/2026-11-13_dead_reckoning_drift.md) found that wheels + gyro reach a p90 error of
0.30 m after about 14 m on flat ground but only 2.75 m on the 11 deg slope, and cannot see wheel
slip at all. VIO is meant to fix exactly those. Everything for the evaluation is ready offline;
one recording session gives the whole evaluation. **VIO stays off on the mower** (`vio:=false`):
the recorder only adds a `ros2 bag record` next to the running stack.

### 8.1 Before the session (owner)

Daylight (the 2026-10-06 night bag was pure sensor noise), RTK FIXED at the start, stack up.

```bash
# dev box, this checkout (ws/vio1114): copy only the recorder (nothing else on the mower changes)
rsync -a scripts/vio/ root@192.168.1.105:/userdata/ros2_stack/scripts/vio/

# mower host (ssh root@192.168.1.105)
/userdata/ros2_stack/scripts/setup_stereo_host.sh status   # 8 MiB buffers + "IIO mode: ..."?
# only if it says otherwise (runtime settings, lost at every reboot):
/userdata/ros2_stack/scripts/setup_stereo_host.sh && docker restart mower_jazzy
/userdata/ros2_stack/scripts/vio/record_vio_bag.sh check   # preflight only, records nothing
```

`check` (and every recording) refuses to start unless: 8 MiB socket buffers and the Metoak IMU
in IIO mode; load below 10; exact stereo pairs >= 9 Hz with <= 5 % unpaired; `/stereo_imu/data`
>= 190 Hz; `/odom` and `/fix` alive; RTK FIXED now; and the planned minutes fit on /userdata
while keeping bag_recorder's 3 GB floor + 1 GB. Each failure prints what to do.

**Idle cap (found 2026-11-14 on the first `check`).** While `/mission/activity` is `docked_idle`
or `idle`, the camera nodes run their idle duty cycle on purpose (`mower_cameras/activity_gate.py`):
1 stereo pair/s and the stereo IMU at 100 Hz. A `check` on the dock therefore sees ~1 Hz pairs.
The preflight now reads the activity and, while idle, only tests that frames flow and pair
(>= 0.8 Hz, IMU >= 90 Hz) and says so. Full rate (10 Hz pairs, 200 Hz IMU) starts as soon as
the activity is `manual` (GUI manual mode = MANUAL_MOWING) or `mowing`. Plain teleop without the
GUI's manual mode may stay `idle`, so drive part A in the GUI's **manual mode**, blade off.

### 8.2 What to record

About **380 MB per minute** (two 300 KB mono8 eyes at 10 Hz = 6.1 MB/s, everything else
0.25 MB/s). Part A + B below = about 23 min = about 9 GB; `check` prints how many minutes fit.

**Part A, manual drive, 8 min** (`record_vio_bag.sh drive`, then switch the GUI to **manual mode**
with the blade off and drive; it stops by itself after 8 min, Ctrl-C stops it earlier):

1. Parked 20 s in open sky (IMU bias, ZUPT).
2. Flat ground: straight 5 m forward, 5 m back in reverse; one slow 360 deg pivot each way;
   three 90 deg pivots.
3. On the slope: up and down the fall line twice (about 10 m each way).
4. **Across the slope** (the drop side, which the drift study lacked): drive along the slope
   twice in each direction, about 10 m each, so the low side is left once and right once. Stop
   for 10 s mid-slope while across it.
5. Two 90 deg pivots on the slope (the slip case: on 2026-11-12 a pivot sequence on the slope
   slid 1.4 m with the wheels reading 0).
6. If there is a tree or a wall: 30-60 s along / under it (real RTK float), then back to open
   sky until RTK is FIXED again.
7. Parked 20 s.

**Part B, a normal mow, 15 min** (`record_vio_bag.sh mow`, default 15 min, then start the mow in
the GUI): pick the area with the slope (Area 1 or the Area 4 strip) so the bag has slope, edge
laps, pivots and reverse legs with RTK truth. Let it mow; the recorder stops after 15 min (the
mow itself can go on).

```bash
/userdata/ros2_stack/scripts/vio/record_vio_bag.sh drive               # part A
/userdata/ros2_stack/scripts/vio/record_vio_bag.sh mow                  # part B (15 min), then start the mow
```

Keep the terminal open (or run it in `tmux`/`screen`): the recorder stops with the script.
Bags go to `/userdata/ros2/vio_bags/<YYYYmmdd-HHMMSS>_<drive|mow>/` (sqlite3, 60 s splits),
not to the bag_recorder ring, which would prune them. The script prints where the bag is, the
`ros2 bag info` head, any dropped-message lines from the recorder log, and the copy command.

Storage choice: sqlite3, no compression, 60 s splits, 16 MiB write cache, `nice 10` and
best-effort I/O class 7. mcap is not in the mower image, and ekf_replay reads .db3 directly.
zstd gains little on sensor-noise grass images (about 1.3x) and would cost a sustained share of
a core while the stack mows (load-190 incident). The recorder takes the lowest priority so that
under contention it drops messages, not the stack. The 60 s splits mean a power cut loses at most one split.
Stack-side cost, not yet measured on the mower: rosbag2's cost is mostly per message (the
ring's curated list is estimated at 12-15 % of a core, bag_recorder.py); this recorder adds about
400 messages/s (both IMUs, odometry, tf) plus 20 images/s (over SHM, not copied through UDP) and
6 MB/s of sequential eMMC writes. Watch `top` during part A; Ctrl-C if the load climbs.

### 8.3 Copy it off (owner)

```bash
# dev box, from this checkout (about 10 GB: 15-30 min over Wi-Fi)
mkdir -p ../bags/vio
rsync -a --info=progress2 root@192.168.1.105:/userdata/ros2/vio_bags/ ../bags/vio/
# then on the mower, once copied:  rm -r /userdata/ros2/vio_bags/<name> /userdata/ros2/vio_bags/<name>.log
```

### 8.4 Offline evaluation (dev box)

```bash
scripts/vio/build_openvins_local.sh                 # once: amd64 OpenVINS into .vio_build/ (~4 min at -j6)
scripts/vio/eval_vio_bag.sh ../bags/vio/<name>      # -> ../bags/vio/<name>.eval/{report,drift}.md
```

Steps (all in `mower:jazzy-dev-amd64`, `--network none`):

1. **OpenVINS, serially**: `ros2_serial_msckf` (config/vio/serial/, built only for the dev box,
   `-DOV_ROS2_SERIAL=ON`) reads the bag with rosbag2_cpp in recorded order and calls
   ROS2Visualizer's IMU and stereo callbacks directly, with OpenVINS' threading off.
   Upstream OpenVINS has a serial runner only for ROS 1. Playing the bag (`ros2 bag play` into
   `run_subscribe_msckf`) would drop or reorder frames whenever the dev box stalls, so two runs
   would differ and the result would depend on the play rate. Serially, every stereo pair is
   processed, the output is the same on every run, and it runs about 20-30x real time.
   Output: odomimu.csv (OpenVINS' /ov_msckf/odomimu content), updates.csv (one row per
   processed pair: wall time, initialised, features, camera-IMU time offset).
2. **Bridge + gate**: `vio_odom_bridge`'s conversion (`odom_convert`, Metoak rotation and
   lever arm) and `vio_gate`'s logic (`vio_gate_logic`), the same code the nodes run, on the
   recorded /fix_status and /odom.
3. **ekf_map replay** (`scripts/replay/ekf_replay.py`, step 1's tool): the same simulated RTK
   dropouts as the drift study (100 s every 120 s, 6 phases), ekf_map's real inputs
   (`--live-inputs`: /odom and /imu/data_aligned are in this bag), variants `v<k>` wheel only
   and `o<k>` + `ekf_dual_vio.yaml`'s gated VIO. The gate runs per variant with that variant's
   dropouts as its RTK loss, so it opens 1 s into each dropout as it would live.
4. **drift.md** (`drift_analyse.py`): position error by distance, and distance until p90 reaches
   0.10 / 0.20 / 0.30 m for wheel only against wheel + VIO, split flat / slope / side slope /
   pivot / reverse / slip.
5. **report.md**: VIO health: init time after the first image and after the first motion,
   processed pair rate (must be 10 Hz), odomimu rate, update wall time (CPU share on the dev box;
   expect 3-4x on one A76 core), MSCKF / SLAM feature counts, the camera-IMU time offset estimate,
   VIO vx and vy against RTK and against the wheels per terrain class (incl. side slope and the
   RTK vy the wheels miss), and gate statistics (on the recorded RTK, and inside the simulated
   dropouts: passing, warming up, diverged latches).

`ORACLE=1 scripts/vio/eval_vio_bag.sh BAG` replaces step 1 by RTK body velocity plus 3 cm/s noise
(no images needed): a plumbing test on any field bag and an upper bound on what a VIO that
is right whenever RTK was could give the EKF. It is **not** a VIO result.

### 8.5 Found while preparing (2026-11-14)

* **`track_frequency: 10` dropped every other stereo pair.** ROS2Visualizer drops a pair that is
  closer than 1/track_frequency to the previous one; stereo_cam's 10 Hz is an average of 84 / 126 ms
  intervals, so half the pairs were dropped (4.8 Hz). Shown with `ros2_serial_msckf` on a
  synthetic bag with that timing: 93 of 186 pairs dropped at 10, none at 20.
  `config/vio_metoak` now has 20 (`config/vio_wit`, the legacy WIT config, still has 10).
* **Current Jazzy packages removed the deprecated headers** `image_transport/image_transport.h`,
  `cv_bridge/cv_bridge.h` and `tf2_geometry_msgs/tf2_geometry_msgs.h`; ov_msckf no longer built
  in the dev image (the 2026-10-06 build predates that). Patch 0004 switches to the `.hpp`
  headers, which also exist in older Jazzy releases. The mower image will hit the same error
  the next time it is built with current packages.
* **odomimu rate.** OpenVINS publishes odomimu from each IMU callback only when
  `fast_state_propagate` finds two IMU samples between the last update and that time (plus the
  estimated camera-IMU offset). On the synthetic bag that was about one row per processed pair,
  not 200 Hz. The report prints the real rate. The EKF weight assumption ("VIO has at most a third
  of the vx weight") holds only at similar rates: robot_localization fuses every message, so
  200 Hz of VIO at variance 0.04 against 50 Hz of wheels at 0.02 would give VIO two thirds.
  `vio_hz=N` in ekf_replay (`VIO_HZ` in eval_vio_bag.sh) tests thinner streams.
* **`mower.launch.py` does not pass `vio_imu`** to vio.launch.py, so `vio:=true` today runs the
  WIT config (`config/vio_wit`, /imu/data at 100 Hz), not the Metoak one this evaluation uses.
  Needs a `vio_imu` pass-through (default unchanged) before enabling.

### 8.6 Building ov_msckf on the mower, later (owner; not part of the recording)

`scripts/build_openvins_mower.sh`, re-checked 2026-11-14 without running it: image `mower:jazzy`,
container mounts and the 2.5 GB / 2 CPU / `-j1` caps are current; Ceres, Eigen, OpenCV, cv_bridge
and image_transport are in `docker/Dockerfile.jazzy`; it stages through `scripts/vio/stage_openvins.sh`
(all four patches; the serial runner is not compiled there). `deploy_to_mower.sh sync` excludes
`third_party/open_vins/` and `.vio_build/`, and `build` skips the `ov_*` packages, so a later sync
or stack build leaves the installed ov_msckf alone.

```bash
# dev box, this checkout, owner present, mower docked (mission CHARGING / IDLE_DOCKED)
git submodule update --init third_party/open_vins
./scripts/deploy_to_mower.sh down                      # stack stopped
SSHPASS=... ./scripts/build_openvins_mower.sh check     # refuses while mower_jazzy runs / < 3 GB MemAvailable
SSHPASS=... ./scripts/build_openvins_mower.sh build     # 30-60 min, then prints the ov_* install sizes
SSHPASS=... ./scripts/build_openvins_mower.sh verify
./scripts/deploy_to_mower.sh up                         # vio stays off (vio:=false default)
```

### 8.7 Smoke test of the pipeline (2026-11-14, no daylight image bag exists yet)

These runs test the plumbing only. They are **not** a VIO quality test.

* `ros2_serial_msckf` on the 2026-10-06 docked night bag (`bags/vio_static_dock`, WIT config): reads
  the bag, pairs the eyes by stamp (109 pairs, 83 unpaired: the old 2 Hz node), processes every
  pair (17.5 ms mean on the dev box), never initialises (night noise), 28x real time.
* On a generated 20 s bag (random texture, a still IMU, stereo_cam's 84 / 126 ms timing):
  initialises after 1.0-1.2 s, and showed the `track_frequency` drop (8.5).
* `vio_preflight.py` against that bag played in a private DDS domain: pairs 9.5 Hz,
  `/stereo_imu/data` 200 Hz pass; missing /odom, /fix and RTK fail with their hints.
* `ORACLE=1 eval_vio_bag.sh` on a 2026-11-13 ring bag (8 min, mostly flat, docked part
  included): all five steps ran; gate 99.9 % "RTK fixed" on the recorded status, 98.6-98.9 %
  passing inside the simulated dropouts (the rest is the 1 s hold-off); 22 dropout windows per
  variant set; distance to p90 0.10 m 0.75 m wheel only against 3.0 m with the oracle VIO (too few
  windows for the other thresholds). Upper bound only, as said in 8.4.
