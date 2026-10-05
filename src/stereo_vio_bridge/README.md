# stereo_vio_bridge

Feeds the VSLAM/VIO estimator from the **stereo front** camera (Metoak XC9080 pair +
on-board "Simor" depth ASIC (+ its embedded TDK ICM-40608 IMU at i2c-8/0x68).

The stereo front is delivered by the RK ISP as **one combined 1280×480 YUV422 stream**
(two 640×480 images side-by-side). This node splits it (the role of the SDK's
`MoImage::SpliteImage`), optionally stereo-rectifies with the recovered calibration, and
publishes mono8 VIO inputs.

Publishes (topics declared by the OpenVINS config in
`ros2_port_handoff/14_vio_replacement/`):

```
/vio/imu               sensor_msgs/Imu
/vio/left/image_raw    sensor_msgs/Image
/vio/right/image_raw   sensor_msgs/Image
/vio/left/camera_info  sensor_msgs/CameraInfo
/vio/right/camera_info sensor_msgs/CameraInfo
```

## Camera topology (whole mower)

| Camera | Sensor | Role | Resolution |
|---|---|---|---|
| **stereo front** (Metoak) | XC9080 + Simor ASIC | VSLAM / VIO | combined 1280×480 YUV422 (2×640×480) |
| **left OA** | GalaxyCore **GC2093** | YOLOv8 / PP-LiteSeg | 1920×1080 UYVY |
| **right OA** | GalaxyCore **GC2093** | YOLOv8 / PP-LiteSeg | 1920×1080 UYVY |
| rear | USB UVC (32e6:9221) | recording / telemetry | 1920×1080 MJPEG/YUV422 |

## Capture layout

`stereo_layout: combined` (default) opens one device and splits it:

```bash
ros2 run stereo_vio_bridge stereo_vio_bridge \
  --ros-args -p stereo_layout:=combined \
  -p stereo_device:=/dev/video11 \
  -p combined_width:=1280 -p combined_height:=480 \
  -p calib_dir:=/userdata/ros2/calibration
```

`stereo_layout: separate` is the legacy two-device mode (`left_device`/`right_device`).

Use `swap_lr:=true` if the ISP emits right-half-first; `publish_mono:=false` to keep bgr8.

> Prerequisite (combined): `mo_init.sh` must have run first (`mo_xc9080.ko` +
> `mo_simor.ko` + `video_rkisp.ko` loaded and the i2c init sequence executed) so the ISP
> exposes the 1280×480 stream as a V4L2 node. Confirm the actual node with
> `v4l2-ctl --list-devices` on hardware.

## Capture paths

`source_mode: v4l2` (default) grabs frames via V4L2 and stereo-rectifies in software using
the recovered calibration (`cam0/cam1/stereo_params.yaml`). Whether the raw frames need
rectifying (`rectify:=true/false`) depends on what the RK ISP exposes — confirm on hardware.

`source_mode: sdk` consumes the clean-room `libmetoak.so` (the reconstructed `metoak.h` C
API in `metoak_reimpl/`) instead of the closed `libMoGeneralSDK`. Build it with
`cd metoak_reimpl/user && make` on the target, then run with `-p metoak_lib:=/path/to/libmetoak.so`.
It produces pre-split 640x480 mono8 + IMU directly from `moLocalGetOneFrame` /
`moLocalSplitRGBFrame` / `moLocalGetIMUData`. This is the "open SDK replacement" path;
`source_mode: v4l2` (default) grabs raw V4L2 and rectifies in software.

> Both modes still require `mo_init.sh` to have run (the ISP exposes the 1280x480
> stream as a V4L2 node); the sdk mode only changes the userspace reader, not the
> kernel bring-up.

Calibration is in `ros2_port_handoff/08_calibration_identity/{cam0,cam1,stereo_params}.yaml`.
