# Cameras, NPU vision and video feeds

This covers the OA and rear cameras, the NPU detection and segmentation nodes, the
`obstacle_guard` safety consumer, and the MJPEG/RTSP/WebRTC feeds. The front Metoak
stereo module belongs to VIO (`launch/vio.launch.py`, `stereo_vio_bridge`), so it is not
covered here. See `docs/perception_vio.md`.

## 1. Devices and topics

The stock udev rules are in `ros2_port_handoff/07_system_config/udev/mower.rules`. The live
symlinks are in `09_platform/udev-symlinks.txt`. `scripts/99-mower-cameras.rules`
reproduces them.

| Camera | Device (symlink → node) | Sensor / format (vendor config) | Producer (canonical) | Topics |
|---|---|---|---|---|
| left OA | `/dev/left_oa_camera` → `video53` | GC2093 MIPI → rkcif → rkisp `rkisp_mainpath` (multi-planar), UYVY 1920x1080, sensor 30 fps, published at 15 | `mower_cameras/v4l2_cam` (`cameras.launch.py`, `oa_driver:=mower_cameras`) | `/left_oa_camera/image_raw`, `/left_oa_camera/camera_info` |
| right OA | `/dev/right_oa_camera` → `video44` | same as left | `mower_cameras/v4l2_cam` | `/right_oa_camera/image_raw`, `/right_oa_camera/camera_info` |
| rear | `/dev/rear_camera` → `video62` | USB UVC "FHD webcam" 32e6:9221, **MJPG only**, 30 fps at every size; default 1280x720, published at 15 | `mower_cameras/camera_node` (`rear_driver:=opencv`, the default) | `/rear_camera/image_raw`, `/rear_camera/image_raw/compressed`, `/rear_camera/camera_info` |
| Metoak stereo | `/dev/video11` (`videoSimor`), `/dev/video22` (`videoIsp`, SDK) | side-by-side | `stereo_vio_bridge` (`vio.launch.py`) | `/vio/{left,right}/image_raw` |

- Calibration comes from `config/cameras/<camera>_info.yaml`, which is installed to
  `share/mower_bringup/config/cameras/`. `cameras.launch.py` passes
  `camera_info_url:=file://<that dir>/<camera>_info.yaml`. You can override the directory
  with `camera_info_dir:=`. All three files are vendor copies with the same intrinsics; the
  rear file is the vendor `usb_cam_info.yaml`.
- `output_encoding:=bgr8` is the default. The driver converts UYVY or YUYV once, so
  `det_ros` and `seg_ros` (which both subscribe) decode it with `cv_bridge`. Use
  `yuv422` to save driver CPU at the cost of per-subscriber conversion.
- Only one process can stream a V4L2 device:
  - `mower_cameras` is off by default (`enable_rear` and `enable_stereo` are both `false`).
    `rear_driver:=opencv` swaps the rear producer; it does not add a second one.
  - `mower_cameras`' stereo output is now `/vio/{left,right}/image_raw`, the same topic
    names that OpenVINS consumes. It is a debug fallback, and you must never run it
    together with `stereo_vio_bridge`.
  - On a stock rootfs, the vendor `base_cameras` node, `mower-webcam.service` and
    `mower-cam-keeper.service` own the same devices and port 8080. Stop them before you
    start the Humble cameras.
- The OA nodes are V4L2 **multi-planar** (`rkisp_v6`, `Video Capture Multiplanar`).
  `v4l2_camera` 0.6 and OpenCV 4.5.4 only speak single-planar, so `v4l2_camera` sees no
  formats, logs `Requesting format: 0x0 UYVY` → EINVAL, then `Failed mapping device
  memory`. The "0x0" is the empty single-planar `G_FMT`, not the `image_size` parameter
  (which arrives correctly as `[1920, 1080]`; it is now typed `List[int]` anyway).
  `mower_cameras/v4l2_cam` (`mower_cameras/v4l2.py`, a pure-Python ioctl+mmap reader that
  handles single- and multi-planar devices) replaces it. The container has no GStreamer
  plugins (only `coreelements`), so `v4l2src` is not an option.
- The rear camera is MJPG-only, which `v4l2_camera` cannot decode. `camera_node`
  captures in a thread with the same reader (`rear_backend:=v4l2`, the default), decodes
  with `cv2.imdecode` for `image_raw`, and passes the camera's JPEG straight through to
  `image_raw/compressed`. `rear_backend:=opencv` uses `cv2.VideoCapture(CAP_V4L2)`, which
  also works (27.9 fps at 1080p). Do **not** set `CAP_PROP_BUFFERSIZE=1`: it halves the
  rate to 13.5 fps.
- Host prerequisites for the OA cameras are already met by the stock rootfs at boot:
  `rkaiq_3A.service` (`/usr/bin/rkaiq_3A_server`, the 3A for both ISPs), the IQ file
  `/etc/iqfiles/gc2093_MY_default.json`, and the media links `rkcif-mipi-lvds{,5}` →
  `rkisp-isp-subdev` → `rkisp_mainpath` (already `[ENABLED]`; no `media-ctl -l` needed).
  `scripts/setup_cameras_device.sh` checks all of this on the host (read-only;
  `start-3a` starts rkaiq the vendor way if it is not running). Without rkaiq, rkisp still
  streams, but exposure and white balance are not regulated.
- Large images and DDS: with Fast DDS defaults (512 KiB SHM segment) a best-effort
  6.2 MB bgr8 frame is fragmented and mostly dropped. A synthetic 15 Hz 1080p publisher
  was received at 5.66 Hz. `cameras.launch.py` therefore sets
  `FASTRTPS_DEFAULT_PROFILES_FILE=config/cameras/fastdds_camera_shm.xml` (a 16 MiB SHM
  segment, 8 MiB max message) for the camera nodes only (a scoped `GroupAction`). The profile is needed on
  the publisher only, and with it delivery is 14.95 Hz. Remote (UDP) subscribers are
  still limited by the host's `net.core.rmem_max=212992`, so use `image_raw/compressed`
  off-board.
- UPDATE 2026-10-06 (current): the SHM transport is disabled for the whole container.
  `docker-compose.yml` sets `FASTRTPS_DEFAULT_PROFILES_FILE=.../config/cameras/fastdds_no_shm.xml`
  (UDPv4 only) and `cameras.launch.py` defaults `camera_dds_profile` to the same file, so
  /dev/shm stays at 0 bytes (was 145 MB per run, 1 GB after several restarts). Measured on the
  mower over UDP loopback: OA ~7-9 Hz, rear ~13 Hz, /mower_base/status 99.8 Hz. The old SHM
  profile `fastdds_camera_shm.xml` is kept for A/B (pass `camera_dds_profile:=` that file
  and remove the compose env). The section below describes that SHM-era analysis.
- /dev/shm budget (SHM era, measured 2026-10-06): only the 4 camera publisher processes (2x v4l2_cam,
  camera_node, stereo_cam) get the profile, so only they own large segments (formerly 4 x
  32 MiB). Every other participant (~27) owns a default 512 KiB segment (549,408 B files) plus
  per-participant port/lock files (52,416 B + 32 B x3); one clean run is ~150 files / ~145 MB and
  stays flat. Subscribers map every segment but allocate nothing. The 890 files / 1 GB seen was
  leakage across restarts (Fast DDS 2.6 does not unlink segments of killed participants; each
  restart adds ~145 MB), not growth within a run. `scripts/shm_gc.sh` (run by the compose command
  at start with MIN_AGE=0, then every 5 min) removes only `fastrtps_*` files that no process maps
  and no fd holds. It only sees this container's PID namespace: do not run other Fast DDS
  containers on the same host /dev/shm. compose `shm_size`/tmpfs cannot cap this because /dev is
  bind-mounted from the host. Note the compose `command` change needs `docker compose up -d`
  (recreate), not just `docker restart`.
- The rear USB power is gated by GPIO 113.

### `cameras.launch.py` arguments

- `left_oa_camera`, `right_oa_camera` (true).
- `oa_driver`: `mower_cameras` (default) or `v4l2_camera` (does not work on rkisp).
- `oa_width`, `oa_height`, `oa_pixel_format`: 1920, 1080, UYVY. UYVY, NV12, NV21 and YUYV
  are converted; any size from 32x32 to 1920x1080 in steps of 8 works, and the ISP scales.
- `oa_fps` (15.0) caps the publish rate; frames above it are dropped before conversion.
- `oa_compressed` (false).
- `rear_driver`: `opencv` (default; `mower_cameras/camera_node`), `v4l2` or `none`.
- `rear_width`, `rear_height`: 1280, 720. Calibration is 1080p; `camera_info` is
  rescaled for 16:9 modes, assuming the UVC mode is a full-FOV scale.
- `rear_fps` (15.0) and `rear_compressed` (true).
- `<camera>_device`, `output_encoding` (bgr8; `v4l2_camera` only), `camera_info_dir`.
- `camera_dds_profile`: the Fast DDS profile for the camera publishers; `""` uses the RMW
  default.
- `web_video_server` (true; `mower.launch.py video`, also true) and `video_port` (8080).
  The GUI proxies it as `:4006/api/cameras/<id>/stream` and `/snapshot` (quality 50,
  640x360, ≤ 5 fps per viewer; see `gui/pkg/api/cameras.go`). web_video_server only
  subscribes and encodes while a client is connected, so the idle cost is about zero.
  See "On-demand video" below for the whole chain (GUI, web_video_server, drivers).

## 2. Models (deploy flow)

The models are the `.rknn` files in `ros2_port_handoff/11_perception_models/`. There is no
source for them. Header metadata:

| File | Used by | Toolkit | Input (NCHW) | Notes |
|---|---|---|---|---|
| `best_large_0208.rknn` | `det_ros` (default) | 2.3.0 | `[1,3,640,480]` | 22 classes (`det_ros/labels.py`) |
| `best_small_0208.rknn` | — | 2.3.0 | `[1,3,640,480]` | class tensors are 1 channel wide, so it is a single-class model with an unknown label |
| `pplite-seg_20260630-1-6cls.rknn` | `seg_ros` (default) | 2.3.0 | `[1,3,480,640]` | 6 classes |
| `pplite-seg_20260606.rknn` | — | 2.3.0 | `[1,3,480,640]` | older seg |
| `model_1.rknn` | — | 2.3.2 | `[1,3,224,224]` | light classifier |

Deploy flow, in addition to `scripts/deploy_to_mower.sh sync|image|build`:

```bash
./scripts/install_models.sh                 # rsync models -> airseekers@192.168.1.105:/userdata/ros2/models/
./scripts/install_models.sh --runtime       # + librknnrt.so 2.3.0 -> /userdata/ros2/lib/librknnrt.so
./scripts/install_models.sh --local DIR     # local copy (then models_dir:=DIR)
./scripts/install_models.sh --dry-run       # show the commands
```

The default `model_path` for `det_ros` and `seg_ros` is `/userdata/ros2/models/<file>`.
`docker-compose.yml` already bind-mounts `/userdata/ros2`. When that file is missing, the
node tries `<models_dir>/<basename>`, where `models_dir` is a parameter and also a launch
argument of `perception.launch.py`.

## 3. NPU runtime requirements

- **Python runtime:** `rknn-toolkit-lite2` **2.3.0** (aarch64, cp310), from
  `airockchip/rknn-toolkit2`. `rockchip-linux/rknn-toolkit2` stops at 1.6.0.
  `docker/Dockerfile.humble` installs it as an appended step. That step is guarded, so an
  unreachable URL only prints a warning. The wheel's dependencies are `numpy` (the system
  1.21.5 satisfies it), `psutil` and `ruamel.yaml`; aarch64 wheels exist for both.
- **C runtime `librknnrt.so`:** the wheel does not include it, so it must be bind-mounted
  at `/usr/lib/librknnrt.so`. The versions involved:

  | Source | Version |
  |---|---|
  | Stock rootfs `/usr/lib/librknnrt.so` (`mower_docs/10-hardware.md`) | 2.1.0 |
  | Vendor workspace copy (`rknn2_runtime/librknn_api/aarch64/librknnrt.so`) | 2.2.0 |
  | Toolkit that compiled the models | 2.3.0 |

  Use the 2.3.0 runtime:

  ```yaml
  # docker/docker-compose.yml, service mower_humble, volumes:
        - /userdata/ros2/lib/librknnrt.so:/usr/lib/librknnrt.so:ro   # 2.3.0 from install_models.sh --runtime
  ```

  Alternative: the stock runtime (2.1.0). It is older than the models, so expect a
  version warning or an init failure.

  ```yaml
        - /usr/lib/librknnrt.so:/usr/lib/librknnrt.so:ro
  ```

  To check a library's version: `strings <lib> | grep 'librknnrt version'`. The kernel
  driver is RKNPU v0.9.8.
- **Device nodes:** the container is `privileged` with `/dev:/dev`, so the RKNPU device
  (`/dev/dri/renderD*` or `/dev/rknpu`, depending on the BSP) is visible.
- **ROS packages:** `ros-humble-vision-msgs` and `ros-humble-web-video-server` are
  appended to `Dockerfile.humble`. `vision_msgs` was not in either image.
  `Dockerfile.dev-amd64` still lacks both. Add them there too if dev images should run
  `det_ros` and `obstacle_guard`; the verification below installed them at run time.
- **Startup behaviour:**
  - If `rknnlite` or the model is missing, `det_ros` and `seg_ros` log one FATAL line that
    names the fix, then exit 1.
  - With `dry_run:=true`, the node logs one WARN, stays alive and publishes nothing.
  - If `vision_msgs` or `cv_bridge` is missing, the node exits 2 with an install hint.
- **NPU cores:** `det_ros` uses core 0, `seg_ros` uses core 1.
- **Frame-rate caps:** `max_rate_hz` defaults to 10 for det and 5 for seg, per camera.

## 4. Perception and `obstacle_guard`

```bash
ros2 launch mower_vision perception.launch.py                  # det + obstacle_guard (seg:=false)
ros2 launch mower_vision perception.launch.py seg:=true stop_on_close:=true models_dir:=/userdata/ros2/models
```

### `det_ros` output

`det_ros` publishes `/ai/det/detections` as `vision_msgs/Detection2DArray` (Humble 4.x
layout):

- `header.frame_id` is the camera frame (`left_oa_camera` or `right_oa_camera`).
- Each detection's `results[0].hypothesis.class_id` is the class name and `score` is the
  confidence.
- `bbox` is in source-image pixels.

It also publishes `/ai/det/image_annotated`.

### `obstacle_guard`

Config: `src/mower_vision/config/obstacle_guard.yaml`.

- **Classes:** whitelist `[person, dog, cat, hedgehog, rabbit]` with `min_score` 0.4. Set
  `whitelist: ['*']` to react to all classes.
- **Close rule:** a detection is close when its bbox bottom edge is greater than
  `y_frac`·H (0.6) **and** its bbox width is greater than `w_frac`·W (0.15).
  `Detection2DArray` carries no image size, and `det_ros` reports bbox pixels in its
  input image's space. The guard therefore subscribes to `camera_info_topics`
  (`/{left,right}_oa_camera/camera_info`, scaled by the camera node) and uses the
  width/height of the matching `header.frame_id` for every detection frame (logged once per
  frame as `image size for frame ...`). `image_width`/`image_height` (960x540) are only the
  fallback until a camera_info arrives. The rule is fractional, so 1920x1080 and 960x540
  give identical decisions (unit-tested).
- **Debounce:** `hold_s` (0.5 s) per camera. The overall state is true when either camera
  is close.
- **Output:** `/vision/obstacle_close` (`std_msgs/Bool`), published on every detection
  frame and when the hold expires.
- **Markers:** `/vision/obstacle_markers` (`visualization_msgs/ImageMarker`, LINE_LIST in
  image pixels). Red means close, yellow means whitelisted, grey is other, and the blue
  line is the danger-zone edge. In Foxglove, add it as an annotation topic on the camera
  Image panel.
- **`stop_on_close`:** defaults to false, so it only logs a warning. When it is true, the
  rising edge of the close state triggers two actions:
  - Zero `Twist` messages on `/cmd_vel_emergency` at 20 Hz for `burst_s` (1 s). This is
    the twist_mux emergency input (priority 100, 0.2 s timeout).
  - One `std_srvs/Trigger` call to `/cutter_off` (`mower_mcu_driver`).

  If the obstacle stays in view after the burst, `obstacle_guard` does not send another
  burst. A new burst needs a new rising edge, which happens after the hold expires.

## 5. Video feeds

| Feed | URL | Port | Source |
|---|---|---|---|
| MJPEG (HTTP) | `http://<mower>:8080/stream?topic=/rear_camera/image_raw&qos_profile=sensor_data` (also `&width=1280&height=720&quality=70`). `qos_profile=sensor_data` is required: the camera drivers publish best-effort, and web_video_server's default (reliable) subscription receives nothing. Do not percent-encode the `/` in `topic`. | 8080/tcp | `web_video_server` (`cameras.launch.py`, on by default) |
| Snapshot | `http://<mower>:8080/snapshot?topic=/rear_camera/image_raw&qos_profile=sensor_data` | 8080/tcp | same |
| Topic list | `http://<mower>:8080/` | 8080/tcp | same |
| RTSP H.264 | `rtsp://<mower>:8554/rear`, `rtsp://<mower>:8554/left_oa` | 8554/tcp, 8000-8001/udp | mediamtx (`docker/docker-compose.video.yml`) |
| WebRTC | `http://<mower>:8889/rear` (WHEP `/rear/whep`) | 8889/tcp, 8189/udp | mediamtx |

The ports don't clash with the rest of the stack: Foxglove uses 8765, the teleop
WebSocket 8766 and the GUI 4006. HLS, RTMP, SRT, the API and MoQ are disabled in
`mediamtx.yml`.

### Default RTSP path: from ROS

mediamtx starts an ffmpeg on demand. ffmpeg reads the `web_video_server` MJPEG at 720p,
encodes it with `libx264` (ultrafast, zerolatency, baseline, 10 fps, about 1.5 Mbit/s) and
publishes `rtsp://127.0.0.1:8554/<path>`. The encode runs only while an RTSP or WebRTC
client is connected, and stops 10 s after the last one leaves.

This path never touches `/dev/video*`, so it can't conflict with `v4l2_camera`.

```bash
# on the mower, from /userdata/ros2_stack
docker compose -f docker/docker-compose.yml -f docker/docker-compose.video.yml up -d
```

CPU estimate for the RK3588S (not measured on the mower):

| Load | Estimate |
|---|---|
| `libx264` 720p10 | about 35-60 % of one A76 core |
| `libx264` 1080p15 | about 100-150 % |
| `web_video_server` JPEG encode at 720p10 | about 15-30 % |
| `v4l2_camera` YUYV→bgr8 at 1080p | about 10-20 % per camera |

The RK3588 hardware encoder (rkvenc through `/dev/mpp_service`) is not used:

- The stock ffmpeg in the mediamtx image has no `h264_rkmpp`.
- Rockchip exposes no V4L2 M2M encoder, so `h264_v4l2m2m` does not apply.
- `09_platform/dev_nodes.txt` lists no encoder nodes; it was filtered to tty, i2c and
  video.

Using the hardware encoder would need a ffmpeg-rockchip build with `/dev/mpp_service`,
`/dev/rga` and `/dev/dri` passed through.

### Option B: direct device (not the default)

The `rear_direct` block is commented out in `mediamtx.yml`. In this option, ffmpeg opens
`/dev/rear_camera` itself with `-f v4l2 -input_format mjpeg`. CPU is lower, because there
is no ROS conversion and no JPEG re-encode. The cost is that the ROS rear camera must be
off (`rear_driver:=none`), because a device can be streamed by only one process. It also
needs the `devices:` entry in the compose file.

### Home Assistant and Frigate

- **Home Assistant:** use the Generic Camera integration with the stream source
  `rtsp://<mower>:8554/rear`. Alternatively, use the MJPEG IP Camera integration with the
  `:8080/stream?...` URL.
- **Frigate:**

  ```yaml
  cameras:
    mower_rear:
      ffmpeg:
        inputs:
          - path: rtsp://<mower>:8554/rear
            roles: [detect, record]
      detect: {width: 1280, height: 720, fps: 5}
  ```

## 6. Integration (coordinator)

`launch/mower.launch.py` is not edited here. These are the include lines for it:

```python
# arguments
arg('cameras', 'true', 'Include cameras.launch.py (OA + rear v4l2_camera).'),
arg('video', 'true', 'web_video_server MJPEG on :8080 (GUI camera page via /api/cameras; source for the RTSP relay).'),
arg('perception', 'false', 'Include mower_vision/perception.launch.py (det_ros + obstacle_guard).'),
arg('stop_on_close', 'false', 'obstacle_guard: zero burst + cutter off on close obstacle.'),

cameras = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cameras.launch.py')),
    launch_arguments={'web_video_server': LaunchConfiguration('video')}.items(),
    condition=enabled('cameras'),
)
perception = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('mower_vision'), 'launch', 'perception.launch.py'])),
    launch_arguments={'stop_on_close': LaunchConfiguration('stop_on_close')}.items(),
    condition=enabled('perception'),
)
# ... + [cameras, perception] in the returned LaunchDescription
```

## 7. Verified off-robot (amd64 dev container `mower:humble-dev-amd64`)

- `colcon build` succeeded for `mower_cameras`, `det_ros`, `seg_ros`, `mower_rknn`,
  `mower_vision` and `mower_bringup`.
- `ros2 launch mower_bringup cameras.launch.py` ran. It produced the three namespaced
  nodes and the topics `/{left_oa_camera,right_oa_camera,rear_camera}/{image_raw,camera_info}`.
  The generated parameter files were correct:
  - `image_size` was the int array `[1920,1080]`.
  - `camera_info_url` was `file:///…/share/mower_bringup/config/cameras/…`.
  - `video_device` was the udev symlink.
  The nodes then logged "Failed opening device", as expected without hardware.
  `rear_driver:=opencv` started `mower_cameras` with `enable_rear=True` and the installed
  rear calibration path.
- pytest passed on the host and in the container.
- `det_ros` and `seg_ros` without `rknnlite` logged one FATAL line and exited 1.
  `dry_run:=true` stayed alive. `perception.launch.py dry_run:=true seg:=true` brought up
  all three nodes.
- `obstacle_guard` fake run with `stop_on_close:=true`:
  - Input was a synthetic `person` `Detection2DArray` with its bottom edge at 1000 px and
    a width of 400 px.
  - Results: `/vision/obstacle_close` was true, 21 zero `Twist` messages arrived on
    `/cmd_vel_emergency` (1 s at 20 Hz), a fake `/cutter_off` server received the call,
    and one `ImageMarker` with 10 points was published.
  - The same box labelled `stone` gave false.
- Video chain:
  - `mower_vision fake_camera` published 1280x720 on `/rear_camera/image_raw`.
    `web_video_server` (installed with apt) served the snapshot (HTTP 200, `image/jpeg`)
    and the `multipart/x-mixed-replace` MJPEG stream.
  - mediamtx v1.21.1 with `docker/mediamtx/mediamtx.yml` served
    `rtsp://127.0.0.1:8554/rear` as H.264 Constrained Baseline 1280x720 at 10 fps, read by
    an ffmpeg client. The WebRTC page on `:8889/rear` answered.

## 8. Only testable on the mower

1. (Done 2026-10-06, see below.) Device formats and rates.
2. Whether the udev numbering (`video53`, `video44`, `video62`) holds after a re-flash or
   kernel change.
3. NPU inference end to end:
   - `rknnlite` 2.3.0 with the bind-mounted `librknnrt.so` (2.3.0 versus the stock
     2.1.0).
   - Model load and `core_mask`.
   - That the det input really is 480 wide by 640 tall in RGB (the old code fed BGR; it
     now feeds RGB).
   - The output layout that the postprocess assumes.
   - The class order of `best_large`.
4. Real latency and CPU of det and seg at 1080p from Python (`max_rate_hz` caps).
5. The `obstacle_guard` thresholds (`y_frac`, `w_frac`, `min_score`) on real footage. Also
   that twist_mux honours the burst, and that `/cutter_off` reaches the MCU.
6. The arm64 image build of the appended Dockerfile lines (the wheel URL from the device
   network, and apt for `vision-msgs` and `web-video-server`).
7. Video CPU cost on the RK3588S, WebRTC through the LAN (ICE on 8189/udp), and the
   arm64 mediamtx image.
8. Conflicts with the vendor services if any are still enabled (`base_cameras`,
   `mower-webcam`, `mower-cam-keeper`, port 8080).

## Verified on the mower (2026-10-06)

- `rknn-toolkit-lite2` 2.3.0 + `librknnrt.so` 2.3.0 (mounted from `/userdata/ros2/lib`): `best_large_0208.rknn`
  loads, `init_runtime(NPU_CORE_0_1_2)` succeeds, inference on a 640x480x3 uint8 input takes **86 ms**
  and returns the expected 9 outputs `[1,64,80,60] [1,22,80,60] [1,1,80,60] ... [1,64,20,15] [1,22,20,15] [1,1,20,15]`.
  RKNN driver 0.9.8.
- Cameras (all three streaming in `mower_humble`, measured with `ros2 topic hz`
  inside the container):

  | Topic | Rate | Size / format | Producer CPU (10 s `top`, % of one core) |
  |---|---|---|---|
  | `/rear_camera/image_raw` | 14.97 Hz | 1280x720 bgr8 (2.76 MB) | `camera_node` 29 % |
  | `/rear_camera/image_raw/compressed` | 14.82 Hz | camera JPEG passthrough (~0.2 MB) | (same) |
  | `/left_oa_camera/image_raw` | 15.02 Hz | 1920x1080 bgr8 (6.2 MB) | `v4l2_cam` 44 % |
  | `/right_oa_camera/image_raw` | 14.98 Hz (raw subscriber) | 1920x1080 bgr8 | `v4l2_cam` 43 % |
  | `/{left_oa,right_oa,rear}_camera/camera_info` | 15.0 Hz | from `config/cameras` | |

  The whole container ran at about 65 % user CPU on 8 cores, with a load average of about
  10, mostly from the non-camera nodes. `/mower_base/status` stayed at 100.0 Hz and
  `/imu/data` at 100.0 Hz.
- Raw device rates (`v4l2-ctl --stream-mmap` on the host):
  - rear MJPG: 29.88 fps at both 1920x1080 and 1280x720.
  - OA UYVY 1920x1080: 30.00 fps (bytesperline 3840, 1 plane, 4147200 B).
  - The `mower_cameras.v4l2` reader gets the same rates from the container, plus the
    per-frame conversion costs below.

  | Conversion | Cost per frame |
  |---|---|
  | UYVY→BGR 1080p | 11 ms |
  | UYVY→BGR or NV12→BGR 720p | 10 ms |
  | MJPEG decode 1080p | 35 ms |
  | MJPEG decode 720p | 26 ms |

  Frame means are about 125/118/111 (BGR), so 3A is active.
- Root cause of the old "rear opens but never publishes": reads worked, but
  `msg.data = frame.tobytes()` makes the Humble rclpy setter range-check every byte in
  Python. That took 0.998 s per 1080p frame, so the node ran at about 1 Hz and the
  `ros2 topic hz` CLI dropped the huge messages. Assigning `array.array('B')` instead takes
  0.005 s.
- Vendor camera services: `rkaiq_3A.service` is active (and is required). `cam.service`
  (Metoak `mo_init.sh`), `mower-webcam.service` and `mower-cam-keeper.service` are
  inactive, which is what we want.


## OA image size (960 px by default)

Fast DDS fragmenting 6 MB 1080p bgr8 images was about 75 % of the per-frame camera CPU.
`cameras.launch.py` now defaults `oa_publish_width:=960` (integer decimation before colour
conversion, `camera_info` scaled to 960x540, 1.5 MB frames). Expected `v4l2_cam` CPU while
subscribed: about 42 % -> about 15 % per OA camera. Measured on the mower (10 s `top`, one
core): 27-38 % for one subscribed camera (`hz` only, 10.02 Hz, other camera idle at 2-7 %),
and 29-32 % per camera with det_ros consuming both. The reduction is real but smaller than
estimated: the remaining cost is capture, ISP dequeue and decimation, not DDS. Frames are still only converted while subscribed
(`oa_on_demand`). `det_ros` letterboxes any input to 480x640, so detection quality is
unchanged by the downscale apart from the lower input resolution, and its bboxes are now in
960x540 pixel space. `obstacle_guard` follows `camera_info` (see above). Use
`oa_publish_width:=0` for native 1920x1080 (e.g. recording full-resolution data); Foxglove
image panels and `ImageMarker` overlays work at either size.


## On-demand video (2026-10-09)

Goal: a camera runs, and is encoded, only while somebody consumes it.

Measured on the mower before the change (load average 13 on 8 cores): one desktop browser
had the Perception page open (it streamed every camera as soon as the page opened, and
kept streaming in a background tab on desktop): 3 MJPEG streams (left_oa and right_oa
annotated, rear), plus 1 Hz snapshot polling of both front stereo eyes (tiles beyond the
third), plus the map PiP at 15 fps, plus 5 health polls/s. Each health poll fetched
web_video_server's index page, which enumerates the ROS graph, so there were hundreds of
TIME_WAIT sockets on :8080. web_video_server used about 35 % CPU. No mediamtx relay was
running, and neither foxglove_bridge nor the bag recorder subscribed to an image topic.
The OA cameras are also subscribed by det_ros (detection; legitimate), and their
camera_info by gui_bridge (diagnostics freshness).

The chain now:

1. **GUI** (`gui/web`):
   - The Perception page streams only the first camera on open. The user starts the
     others with the tile's play button, and the choice is remembered per browser
     (`localStorage` `perception.startedCameras`).
   - Every stream, and the detections feed, stops while the browser tab is hidden (on
     desktop too) and resumes when the tab is shown again. The map PiP behaves the same.
   - MJPEG `<img>`s go through `MjpegImg`, which points the element at a `data:` URL when
     it unmounts or its URL changes. This aborts the HTTP request at once; a merely
     detached `<img>` can keep loading until it is garbage-collected.
   - The Go proxy caches the web_video_server index probe for 2 s, so the health polls
     share one upstream request.
2. **web_video_server** 3.1.0 (unchanged, verified in the dev image): it subscribes when
   an HTTP client connects and drops the subscription about 0.5 s after the client leaves.
   A snapshot subscribes only for that one frame.
3. **Drivers** (`mower_cameras` `v4l2_cam`, `camera_node` rear, `stereo_cam`; parameter
   `close_when_unused_s`, default 5 s):
   - With `publish_on_demand` set, the device is not opened at start-up until a
     subscriber appears.
   - The device is closed when the topic has had no subscriber for 5 s. That stops the
     sensor and ISP stream, the 3A, and the dequeue loop.
   - The device reopens on the next subscriber. The poll runs every 0.2 s, and then the
     device open time adds to the delay.
   - The 5 s linger keeps a reconnecting viewer from cycling the device.
   - `0` gives the old behaviour: the device keeps streaming, and frames are dequeued and
     dropped.

Consumers that keep a camera open by design: det_ros (OA cameras and the front right eye,
at 1 Hz while the mission is idle), det_ros on the rear camera only while
`/vision/rear_watch` is open, docking (rear, while a vision dock runs), and gui_bridge's
camera_info freshness check on the OA cameras.

To check on the robot: `GET :4006/api/cameras` shows `viewers` per topic, and the topic
subscriber list should show no `web_video_server` while no GUI view is open. The driver
logs `<dev>: no subscribers (device closed); opening on demand` and later `resuming`.
