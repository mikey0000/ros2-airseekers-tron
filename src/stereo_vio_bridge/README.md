# stereo_vio_bridge

Feeds the VSLAM/VIO estimator from the **stereo front** camera (Metoak XC9080 pair +
on-board "Simor" depth ASIC) and its embedded TDK ICM-42600 IMU.

Publishes (topics declared by the VINS config in `ros2_port_handoff/14_vio_replacement/`):

```
/vio/imu               sensor_msgs/Imu
/vio/left/image_raw    sensor_msgs/Image
/vio/right/image_raw   sensor_msgs/Image
/vio/left/camera_info  sensor_msgs/CameraInfo
/vio/right/camera_info sensor_msgs/CameraInfo
```

## Camera topology (whole mower)

| Camera | Role | Resolution | Driver |
|---|---|---|---|
| **stereo front** (Metoak) | VSLAM / VIO | 640×480 | this node (`v4l2` or `sdk`) |
| **left OA** | obstacle detection (YOLOv8) / segmentation (PP-LiteSeg) | 1920×1080 UYVY | `v4l2_camera`/`usb_cam` + `rknn_model_zoo` |
| **right OA** | obstacle detection / segmentation | 1920×1080 UYVY | `v4l2_camera`/`usb_cam` + `rknn_model_zoo` |
| rear | recording / telemetry | 1920×1080 MJPEG/YUV422 | `v4l2_camera`/`usb_cam` |

## Capture paths

`source_mode: v4l2` (default) grabs the two sensors via V4L2 and stereo-rectifies in
software using the recovered calibration (`cam0/cam1/stereo_params.yaml`). Whether the
raw frames actually need rectifying (`rectify:=true/false`) depends on what the RK ISP /
sensor exposes — confirm on hardware.

`source_mode: sdk` (not implemented here) would consume already-rectified frames + IMU
from the Metoak SDK. That needs a `libMoGeneralSDK` shim — the SDK ships no headers, but
its full API was recovered in `native_decompile/metoak/MoGeneralSDK.api.txt`. The SDK only
works on the **Rockchip 5.10.66 BSP kernel** the mower ships (`mo_simor.ko` has no source),
so the V4L2 path is preferred for a clean Ubuntu 22 port.

## Run

```bash
ros2 run stereo_vio_bridge stereo_vio_bridge \
  --ros-args -p left_device:=/dev/video0 -p right_device:=/dev/video1 \
  -p calib_dir:=/userdata/ros2/calibration
```

Calibration is in `ros2_port_handoff/08_calibration_identity/{cam0,cam1,stereo_params}.yaml`.
