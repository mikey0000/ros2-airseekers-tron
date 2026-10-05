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
| `base_cameras` (V4L2 manager) | `v4l2_camera` + `launch/cameras.launch.py` | `launch/` |

## Data flow

```
Metoak stereo front ──(V4L2)──> stereo_vio_bridge ──/vio/imu,/vio/{left,right}/image_raw──> OpenVINS ──/ov_msckf/...──> TF /odom-in-vio
left_oa_camera  ──(V4L2)──> det_ros (YOLOv8) ──/ai/det/detections──> (obstacle avoidance — TODO)
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

## Blockers / next

1. **OpenVINS build in the Humble image** — add the `open_vins` repo to the colcon
   workspace (symlink under `src/`), `rosdep install`, add `libceres-dev` if rosdep can't
   resolve it on arm64. Configs are ready in `14_vio_replacement/open_vins/`.
2. **IMU rate** — `stereo_vio_bridge` emits IMU on the image tick (~10 Hz); VIO wants
   ~200 Hz. Configure the ICM-42600 IIO hrtimer/buffer (`update_rate: 200` assumed).
3. **Obstacle-avoidance consumer** — `det_ros`/`seg_ros` publish standard
   `vision_msgs`/Image surfaces; the node that turns these into drive commands is still
   to be written (the closed OA/fusion nodes used vendor-specific pointcloud topics).
4. **`seg_ros` fusion** — the original fused seg masks into obstacle pointclouds via a
   per-camera `points.txt` monocular-depth mapping; that fusion step (`fusion_ros_node`)
   needs a replacement or a drop decision.

## Prerequisites in the container

- `rknn-toolkit-lite2` (aarch64, matches `librknnrt` 2.1.0) — see `src/mower_rknn/README.md`.
- `ros-humble-v4l2-camera`, `ros-humble-vision-msgs`, `ros-humble-cv-bridge`
  (the latter two ship with ros-base; `vision_msgs` is in `common_interfaces`).
