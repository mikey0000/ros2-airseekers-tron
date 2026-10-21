# Front camera colour cast ("looks quite yellow"), 2026-10-22

> **Correction (ws/camera-colour2).** The first version of this note (branch
> `ws/camera-colour`, merged as abdfc66) said the XC9080 sends YVYU and switched
> `stereo_cam` to decode YVYU. **That was wrong.** Stock decodes YUYV, and live frames
> decode correctly as YUYV. With YVYU the sky turned orange and the grass teal, which
> is what the owner reported as "worse". `color_order` is back to `YUYV` (a9edc35, and
> this branch). The remaining mild warm cast comes from the XC9080 ISP set-up, not
> from the decode.

## Which stream
The GUI "Front stereo left/right" tiles (`third_party/mowglinext/gui/pkg/api/cameras.go:186-187`)
show `/vio/{left,right}/image_color`. These come from `mower_cameras/stereo_cam`
(`stereo_node.py`), which reads `/dev/video22` (`/dev/videoIsp`, rkcif-mipi-lvds3, behind
the XC9080). `det_ros` and `seg_ros` consume `/vio/right/image_color` too.

## What stock does (confirmed from the binaries)
| Step | Stock | Evidence |
|---|---|---|
| Node | `vslam_ros/stereo_ros` (ROS1). It uses `libmoMowerAPI` + `libMoGeneralSDK` 2.7.5.10, not V4L2 directly. | `ota_.../src/mower_perception/vslam_ros/CMakeLists.txt`, `src/stereo_ros.h` |
| Device open | `MoMp021MipiV4l2Manager::OpenAllDevices` opens `/dev/videoIsp` with fourcc `0x56595559` = **`V4L2_PIX_FMT_YUYV`**, 0x500x0x1e0 (1280x480). It opens `/dev/videoSimor` with `RGB3`. | `libMoGeneralSDK_static.a` member `MoMp021MipiV4l2Manager.cpp.o`, Ghidra `OpenAllDevices` (`local_424 = 0x456595559`) |
| Eye split | `moLocalSplitRGBFrame(&RGBFrm, &L, &R, 1)` calls `MoImage::SpliteImage`: per row, `memcpy` of the first half of the bytes to L and the second half to R. No byte reordering. | `MoImage.cpp.o` `SpliteImage` @0x3058 |
| Colour | `cv::cvtColor(matYuv422, matRGB, 0x74)`. **116 = `COLOR_YUV2BGR_YUY2` (= YUYV)**, on a `CV_8UC2` 640x480 Mat per eye, published as `bgr8`. There is no colour matrix, white balance or gain in the node. | `native_decompile/dec/stereo_ros/stereo_ros.c:5695,5706` (`process_rgb` @0x178750) |
| Sensor/ISP init | `MoMp021IspSensor::Init` (SDK, at every open) writes `__MoXc9080StreamOffRegData`, then **`__Mp021Xc9080InitData`, 3917 XC9080 registers** over I2C. It then writes `__Mp021Sc132InitData` (SC132GS), sleeps 0.5 s, runs `MoIspSensor::Init`, and checks XC9080 reg `0x0137 == 0x88` ("isp firmware value", otherwise "ISP maybe not work"). The 2.7.5.10 CHANGELOG says "更新ISP初始化序列，V14". 2.7.5.0 added this check "to fix the ISP green-image problem". | `MoMp021IspSensor.cpp.o` relocations @0x4a0-0x7f8; table at `libMoGeneralSDK.so.2.7.5.10` vaddr 0x1959a0 |

Where the YVYU claim came from: `metoak/mo_init.sh` *echoes* `"Isp: V4L2_PIX_FMT_YVYU 1280 480
/dev/video11"`. That line is only a label, and it names the wrong device too. Our clean-room
`metoak_reimpl/user/metoak.c:184` copied it. Neither file is what the SDK does.

## Live-frame check (2026-10-22 ~06:4x mower time, read-only)
* One `/vio/{left,right}/image_color` + `/vio/right/image_raw` set was taken from the
  running mower. The node was running `color_order YVYU`.
* The packed bytes were rebuilt from the published bgr8: inverse BT.601, then the chroma
  pair averaged and re-packed in YVYU order. The rebuilt luma matches `image_raw` to a mean
  of 1.2 LSB, so the rebuild is faithful.
* The bytes were then decoded as YUYV, YVYU, UYVY and VYUY:
  - **YUYV: natural.** Blue sky (R133 G208 B251), white clouds, green grass, and a brown
    timber deck.
  - **YVYU (abdfc66): orange sky** (R237 G177 B118), teal grass and a blue deck.
  - **UYVY/VYUY: garbage.** Luma is taken from the chroma bytes, giving green/magenta stripes.
  - NV12/NV21 cannot apply: the buffer is 1280x480x2 packed.
* Images are in `ros2_stack-wt/accuracy-simout/camera_colour2/`:
  `live1_right_grid_YUYV_YVYU__UYVY_VYUY.png` (top left YUYV, top right YVYU) and
  `live1_left_as_YUYV.png`.
* The vendor sample `metoak/xc9080.yuv` that looked right as YVYU is not representative of
  this unit's stream. It came from a dev board with a different XC9080 set-up.

## The yellow cast (before abdfc66)
* With the correct YUYV decode, neutral objects are slightly short of blue:
  - grey stone: R205 G201 B164;
  - far white fence: R167 G171 B138;
  - grass: R176 G185 B102.
* That is a mild warm/yellow white balance. It varies with the scene, because the XC9080
  runs its own on-chip AWB/AE.
* Likely cause: our stack never runs the SDK. The XC9080 only gets the
  `mo_xc9080.ko` stream-on table (`isp_xc9080_1280x480_regs`, 2299 regs). Stock overwrites
  it with the SDK's V14 table (3917 regs) at every open.
* The two tables differ heavily (difflib ratio 0.26), including a large patched block at
  0x2100-0x22ff and different values for 0x8010, 0x1800 and 0x0007/0x0016/0x0017. AWB and
  CCM tuning are among the differences that would plausibly matter. This is **inferred**:
  the register map is not documented.
* Dump: `accuracy-simout/camera_colour2/sdk_xc9080_v2.7.5.10.txt` (`addr val` per line, in
  write order).

## Fix
* `stereo_cam` `color_order` default is `YUYV` again, the same as stock. YVYU stays
  available as an override.
* The docstrings and docs (`perception_vio.md`, `stereo_depth_plan.md`,
  `yolo_retrain_pipeline.md`) are corrected.
* `test/test_stereo_split.py` adds:
  - a live-frame fixture (`test/data/xc9080_live_patches.npz`: sky, stone and grass
    patches) that must decode blue, roughly neutral and green as YUYV;
  - a check that the node default stays YUYV.

## Proposed follow-up (not done; needs a supervised test on the mower)
* Replay the stock XC9080 init on our stack: a one-shot `xc9080_init` that does what
  `MoMp021IspSensor::Init` does, before `stereo_cam` opens video22:
  - stream-off regs;
  - the 3917-entry V14 table via `/dev/i2c-8` addr 0x1b (16-bit reg, 8-bit val);
  - SC132GS table through the XC9080 bypass;
  - wait 0.5 s;
  - verify `0x0137 == 0x88`.
* Risks:
  - it races the kernel driver's own stream-on writes;
  - the order relative to `mo_xc9080.ko` must be checked;
  - stereo_depth (video11) shares the XC9080.
* Simpler alternative to try first: fix `mo_xc9080.ko`'s 1280x480 table to the SDK's.

## Effect on perception
* While abdfc66 was live (YVYU), `seg_ros`/`det_ros` saw teal grass and orange sky. Grass
  segmentation is colour-driven, so expect it to have been degraded during that window.
* Stereo depth (`video11`) and VIO (luma) were never affected.

## Owner action
* Deploy `mower_cameras` and `mower_bringup` from this branch (or `mower_cameras` from
  master a9edc35, which already defaults to YUYV), then restart the camera launch.
* The new launch argument `stereo_color_order` (default `YUYV`) selects the order at launch.
* `stereo_cam` reads `color_order` only at start-up, so `ros2 param set` at run time
  changes nothing: always restart the node.
* Immediate revert with no rebuild: in the container, change the default in the installed
  `mower_cameras/stereo_node.py` (`d('color_order', 'YVYU')` -> `'YUYV'`), then restart the
  camera launch.

## Front camera "flicked off" (20:2x-20:3x UTC)
`stereo_depth` logs (`/userdata/ros2/log/python3_392_*.log`) show `/mission/activity` toggling
between mowing and idle:
* idle at 20:26:53, 20:28:49, 20:29:53, 20:30:49 and 20:31:46;
* mowing or manual again within 2-90 s.

On each idle, `stereo_depth` closes video11 (the matching dmesg `stream OFF`/`ON` lines are
at 04:30-04:31 local time = UTC+8), and `stereo_cam` drops to its 1 pair/s idle cap. These
times match the boundary incidents.

There is no crash, no USB reset and no OOM in dmesg. This is the designed activity gating,
not a fault. The analysis made one read-only ROS query at about 20:52 UTC, which was after
the dropout.
