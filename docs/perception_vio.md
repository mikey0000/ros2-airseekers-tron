# Perception / VSLAM / NPU — Airseekers Tron ROS 2 port

Status of the perception + VSLAM + localization phase (priority: GPS → vision → VSLAM →
NPU detection). GPS gate + gating exist (`mower_localization`); this doc covers vision,
VSLAM, and NPU.

## Component map (closed binary → replacement)

| Closed binary | Replacement | Where |
|---|---|---|
| `stereo_ros` / `as_vio_node` (Metoak VIO) | **OpenVINS** (approved) | `13_open_source_upstreams/open_vins/` + `14_vio_replacement/open_vins/` |
| `libcamera_models.so` (camodocal) | OpenVINS `ov_core` camera models | same clone |
| `det_ros` + `libdet.so` (YOLOv8) | `det_ros` (Python, rknn-lite) | `src/det_ros/` |
| `seg_ros`/`seg_oacam`/`libppseg.so` (PP-LiteSeg) | `seg_ros` (Python, rknn-lite) | `src/seg_ros/` |
| `libMoGeneralSDK` + `mo_simor.ko` (Metoak SDK) | **bypassed** — V4L2 + recovered calib | `src/stereo_vio_bridge/` |
| `base_cameras` (V4L2 manager) | `v4l2_camera` + `launch/cameras.launch.py` (rear MJPEG fallback: `mower_cameras`) | `launch/`, `src/mower_cameras/` |
| closed OA consumer of `det_ros` | `mower_vision/obstacle_guard` (danger-zone stop) | `src/mower_vision/` |
| `web_video_server` (MJPEG for the app) | `web_video_server` (+ mediamtx RTSP/WebRTC relay) | `launch/cameras.launch.py`, `docker/docker-compose.video.yml` |

## Data flow

```
Metoak stereo front ──(V4L2)──> stereo_vio_bridge ──/vio/imu,/vio/{left,right}/image_raw──> OpenVINS ──/ov_msckf/...──> TF /odom-in-vio
left_oa_camera  ──(V4L2)──> det_ros (YOLOv8) ──/ai/det/detections──> obstacle_guard ──/vision/obstacle_close, /cmd_vel_emergency, /cutter_off
                      └────> seg_ros (PP-LiteSeg) ──/ai/seg/{mask,traversability}──> (planner — TODO)
right_oa_camera ──(V4L2)──> det_ros + seg_ros
```

## Key findings this phase

1. **Metoak SDK is not needed** on the VIO/det/seg path. `stereo_vio_bridge` grabs raw
   V4L2 frames; calibration is fully recovered (`08_calibration_identity/`). The closed
   SDK only adds on-chip depth and only runs on the stock Rockchip 5.10.66 BSP kernel.
2. **Model input shapes recovered from `.rknn` metadata** (not guessed):
   - `best_*_0208.rknn`: 480 wide × 640 tall (static `[1,3,640,480]` NCHW).
   - `pplite-seg_20260630-1-6cls.rknn`: 640 wide × 480 tall (`[1,3,480,640]` NCHW).
   The det and seg inputs are transposed relative to each other — worth re-confirming on
   the first real inference.
3. **NPU core split** preserved: det core 0, seg core 1, core 2 spare.

## Metoak data path (recovered from the mower runtime `metoak/` + `iqfiles/`)

The user pulled the mower's Metoak SDK runtime directory (`metoak/`) and Rockchip ISP IQ
tunings (`iqfiles/`) into the workspace. Everything below is now concrete, not inferred.

### i2c-8 peripherals (the whole stereo module hangs off `feca0000.i2c`)

| addr | device | role |
|---|---|---|
| `0x06` | simor | stereo-matching / depth ASIC |
| `0x1b` | xc9080 | **ISP / disparity engine** (not the sensor — it sits ahead of the SC132GS pair) |
| `0x1e` | lis2mdl / IIS2MDC | magnetometer |
| `0x48` | tmp112 | temperature |
| `0x50` | 24c128 (AT24 EE) | **calibration EEPROM** |
| `0x68` | icm40608 | 6-axis IMU (driven by `inv-icm42600` kmod) |

### Frame formats (from `mo_init.sh` + sample file sizes)

| stream | size | format |
|---|---|---|
| ISP stereo (combined) | **1280×480** | YUV422 (`YVYU`) — 2×640×480 side-by-side |
| Simor raw | 640×360 | `SBGGR8` Bayer |
| Simor RGB | 640×360 | RGB888 |
| Simor YUV | 640×360 | NV12 |
| Simor depth | 640×360 | uint16 |

`MoImage::SpliteImage` (in the SDK) splits the 1280×480 into left/right. So VIO input is
one combined frame split in software — **not** two independent 640×480 V4L2 devices.

### Calibration

Stored in the 24c128 EEPROM at `0x50`, dumped by `local_dump_epr` (source
`local_dump_epr.cpp`) → `metoak_stereo_eeprom_calibration.data` (magic `-Metoak-[Archer]`).
SDK EPR codec funcs recovered from symbols: `MoBuildEprxFile`, `MoConvertEpr2SaveInfo`,
`MoConvertSave2EprInfo`, with structs `MoEprInfo`/`MoEprSaveInfo`/`MoStreamInfo`/
`MoDepthFrmInfo` (DWARF layouts extractable — binaries are **not stripped**).

### Clean-room re-impl (metoak_reimpl/) — fold-in

The `metoak_reimpl/` directory (produced in parallel) is a clean-room, open-source
re-implementation of the MP021 stack. It supersedes the earlier reverse-engineering and
gives us three concrete assets:

1. **`include/metoak.h`** — reconstructed C API (~170 `moLocal*`/`Mo*` functions + DWARF-
matching structs `MoFrame`/`MoIMUData`/`MoEprInfo`/`MoDepthFrmInfo`/…). This is the
contract the `stereo_vio_bridge` `source_mode: sdk` shim targets (see below).
2. **`kernel/regs_xc9080.h`** — verbatim per-resolution init tables extracted from the
vendor `mo_xc9080.ko`. `isp_xc9080_1280x480_regs` is exactly the **VIO 1280×480 stream**
(the others are 960×360 / 2160×720 / 2160×1080 / 1080×480 variants). `regs_simor.h` is
empty/sentinel (Simor init is driven by XC9080/FPGA).
3. **`docs/simor_meta.md` + `docs/API.md`** — decode of `simor.meta` (1280 B) and the
**EPR** (400 KB, `MoEprInfo` header + CRC + backup slot) holding per-unit intrinsics +
rectification + disparity offset — the exact calibration VSLAM needs.

Also resolved this phase: the vendor dev-board `.ko` are **5.10.66** (`readelf -p
.modinfo`); the mower runs **5.10.209** with its own matching modules, so the stock-
kernel + container path is viable and the 6.1 re-impl drivers are a contingency only
(see `deployment.md`).

### SDK + init

- SDK version **2.7.4.1**; camera model **"Mp021" MIPI** (`MoMp021MipiCamera`/`MoMp021MipiV4l2Manager`).
- Sensor init is a short i2c sequence via `mo_asicrw 8 xc` + `mo_asicrw 8 s 0051 …` (bus 8).
- GPIOs: 103 = trigger enable, 42 = Simor reset, 113 = rear-camera USB power.

### Sensors → iqfiles (resolved from `device-tree.dtb` + `iqfiles/`)

The device tree names the physical sensors exactly:

| camera | sensor | i2c | driver | iqfile |
|---|---|---|---|---|
| left OA | GalaxyCore **GC2093** | `/i2c@feab0000/gc2093b_1@37` | `gc2093.ko` | `gc2093_MY_default.json` (1920×1080) |
| right OA | GalaxyCore **GC2093** | `/i2c@fead0000/gc2093b_1@37` | `gc2093.ko` | `gc2093_MY_default.json` (1920×1080) |
| stereo front | ISP **XC9080** + **2× SmartSens SC132GS** (1 MP global-shutter mono pair) | `/i2c@feca0000/XC9080@1b` | `mo_xc9080.ko` | none (custom sensor) |

The other IQ files (`imx327`, `os04a10`, `imx415`, `imx464`, `gc8034`, …) are for other
Rockchip modules not on this unit. For Ubuntu 22 the OA cameras need `gc2093.ko` +
rkaiq (`camera_engine_rkaiq`) + `gc2093_MY_default.json` installed — see
`scripts/setup_camera_iq.sh`.

## Blockers / next

1. **OpenVINS build in the Humble image** — add the `open_vins` repo to the colcon
   workspace (symlink under `src/`), `rosdep install`, add `libceres-dev` if rosdep can't
   resolve it on arm64. Configs are ready in `14_vio_replacement/open_vins/`.
2. **IMU rate** — `stereo_vio_bridge` emits IMU on the image tick (~10 Hz); VIO wants
   ~200 Hz. Configure the ICM-40608 IIO hrtimer/buffer (`update_rate: 200` assumed).
3. **Obstacle-avoidance consumer** — first consumer done: `mower_vision/obstacle_guard`
   (whitelisted classes in an image-space danger zone -> `/vision/obstacle_close`, optional
   zero burst on `/cmd_vel_emergency` + `/cutter_off`). A planner-level avoidance (steer
   around, not just stop) is still to be written. Thresholds are untuned (no field data).
4. **`seg_ros` fusion** — the original fused seg masks into obstacle pointclouds via a
   per-camera `points.txt` monocular-depth mapping; that fusion step (`fusion_ros_node`)
   needs a replacement or a drop decision.

## Prerequisites in the container

Full deploy steps (models, runtime, compose lines, video feeds): `docs/cameras_and_video.md`.

- `rknn-toolkit-lite2` **2.3.0** (aarch64 cp310; the `.rknn` models were compiled with
  toolkit 2.3.0, the stock `/usr/lib/librknnrt.so` is 2.1.0 and the vendor workspace copy
  2.2.0) + a matching `librknnrt.so` bind-mounted at `/usr/lib/librknnrt.so`.
  Appended to `docker/Dockerfile.jazzy` (guarded pip install).
- `ros-jazzy-v4l2-camera`, `ros-jazzy-cv-bridge` (already in the image),
  `ros-jazzy-vision-msgs` and `ros-jazzy-web-video-server` (appended to
  `docker/Dockerfile.jazzy`; `vision_msgs` is NOT part of ros-base, and is also missing
  from `Dockerfile.dev-amd64`).
- Models on the device: `scripts/install_models.sh` -> `/userdata/ros2/models/`.
