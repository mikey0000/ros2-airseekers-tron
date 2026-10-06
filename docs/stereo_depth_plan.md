# Front stereo depth into Nav2 (plan, 2026-10-06)

Status: investigation only; no code yet. Goal: feed Metoak stereo depth into the Nav2 local
costmap `obstacle_layer` as PointCloud2 (user picked this over bbox ground-projection).

## Findings (read-only probe of the live device + decompile)
- The Metoak module computes disparity in hardware. The vendor `stereo_ros` called
  `moLocalGetOneRGBDFrame -> moLocalDecodeRGBDFrame -> moLocalEnhanceRemoveDupTexture ->
  moLocalOutlierRemove` and published `/vio/depth/image_raw` (mono16, frame `stereo_camera`,
  15 fps). Source: `native_decompile/dec/stereo_ros/stereo_ros.c:5461-5613`.
- Media graph: Simor (`m02_b_mosimor`, i2c 8-0006) -> rkcif-mipi-lvds2 -> `/dev/video11`
  (`/dev/videoSimor`), bus SBGGR8 1920x360 @30fps, current format RGB3 640x360 = 691,200 B.
  Nothing streams it today. XC9080 -> `/dev/video22` YVYU 1280x480 is the L/R pair used by
  `stereo_cam`.
- The 1920x360 frame is a packed RGBD frame, not Bayer. Vendor samples in `metoak/`:
  `simor.yuv` (640x360 NV12), `simor.depth` (640x360 uint16, max 2807), `simor.rgb`.
  Hypothesis: 345,600 B NV12 + 345,600 B 12-bit disparity (unverified; `parseRgbDFrame` in
  `metoak/mo_simor` has DWARF). Sample is 96 % zeros: expect sparse disparity. Disparity
  scale unknown (d/16 or d/64): check `GetPointCloudFromDispImg` in
  `src/mower_drivers/base_stereo/sdk_v2745/lib/libMoGeneralSDK_static.a` or a target at a
  known distance.
- Calibration: `ros2_port_handoff/08_calibration_identity/stereo_params.yaml` (base 60.047 mm,
  bxf 22698.4 -> f ~378 px), `cam0.yaml`/`cam1.yaml` 640x480 intrinsics; EEPROM AT24C128 at
  `/dev/i2c_simor0_eeprom` (dump in repo). `stereo_vio_bridge/camera_info_builder.py` parses
  the yaml; `DispOffset` (MoDepthFrmInfo) not parsed yet.
- Vendor SDK via ctypes is possible (structs in `metoak_reimpl/include/metoak.h`) but it
  re-inits the ISP, must own both video nodes (replaces `stereo_cam`), closed source: not the
  main path.
- `stereo_cam` runs ~2.5 Hz against a 5 fps cap: the sensor triggers at ~25 Hz; the cap is the
  Python YUYV->BGR of 1280x480 plus `CaptureLoop._keep` gating at 0.9 of the period
  (`v4l2_node.py:135`) on a loaded CPU. Fix: publish mono8 Y or move capture to C++.

## Plan (~3-4 days)
1. Probe video11 (0.5 d): capture raw 691,200 B buffers with plain V4L2 (does not touch
   video22), decode offline, verify packing, scale and DispOffset at a known distance.
   Risk: Simor depth may need SDK register setup.
2. `mower_cameras/stereo_depth` node (1.5-2 d): read video11 with `v4l2.py` CaptureLoop,
   unpack with numpy, Z = bxf / (d/scale), downsample (~14k pts), range 0.2-4 m, publish
   `/stereo/depth/image_raw` + `/stereo/points` (PointCloud2, frame `stereo_camera`) 5-10 Hz.
3. URDF + costmap (0.5 d): `stereo_camera` link exists (`config/urdf/mower.urdf.xacro:252`,
   ~4 deg down); `stereo_cam` stamps `vio_camera`, which is not in the URDF: fix. Add a
   PointCloud2 observation source to the local `obstacle_layer` (min_obstacle_height ~0.05 m
   to reject grass, clearing on). Check 640x360 vs 640x480 ROI / `IfOriginImg720P`.
4. Fallback (2-3 d): host StereoBM on rectified 320x240 pairs from video22 inside `stereo_cam`.
