# Front camera colour cast ("looks quite yellow"), 2026-10-22

## Which stream
The GUI "Front stereo left/right" tiles (`third_party/mowglinext/gui/pkg/api/cameras.go:186-187`)
show `/vio/{left,right}/image_color`. These come from `mower_cameras/stereo_cam`
(`stereo_node.py`), which reads the Metoak module on `/dev/video22` (rkcif-mipi-lvds3, behind
the XC9080 bridge). `det_ros` (YOLO) and `seg_ros` (grass segmentation) consume
`/vio/right/image_color` too (`det.yaml:31`, `seg.yaml:27`).

## Cause: U/V swap (confidence high)
* The XC9080 outputs packed **YVYU** (Y0 V Y1 U). The vendor SDK opens video22 with
  `V4L2_PIX_FMT_YVYU` (`metoak_reimpl/user/metoak.c:184`). `HARDWARE.md`, `perception_vio.md`
  and `stereo_depth_plan.md` all say the same.
* `stereo_cam` asks for `YUYV` and decodes colour with `cv2.COLOR_YUV2BGR_YUYV`. rkcif DMAs the
  bridge bytes unchanged, so the fourcc label does not reorder them, and U and V are swapped.
  Luma, and therefore `image_raw`, VIO and stereo, is unaffected.
* Evidence from the vendor capture `metoak/xc9080.yuv` (an office scene):
  - Decoded as YVYU: a wooden table, white walls, a black gimbal and a green tissue box, all
    natural colours.
  - Decoded as YUYV, as we do now: a blue table, red rings on the black gimbal, and warm walls.
  - Images: `ros2_stack-wt/accuracy-simout/camera_colour/xc9080_sample_decoded_as_{yuyv,yvyu}.png`
    and `right_eye_image_color_{yuyv_current,yvyu_fixed}.png`.
* The modules are **colour** SC132GS parts. The "mono" claims in `metoak_reimpl/docs` are wrong.
* **The cast is scene-dependent, as the owner saw after the turn.** A swap mirrors every
  chroma vector across U=V:
  - blue sky and cool/shaded whites (U>0, V<0) become yellow-orange;
  - yellow-green grass shifts toward teal.
  The colour of the output therefore depends on what is in frame: sky versus grass, and
  backlit versus front-lit. On top of that, the XC9080's own on-chip AE/AWB re-balances per
  scene. That gives "yellowish again after it turned around" without any change in white
  balance gains. A plain WB error would be a constant tint instead.
* There were no frames to correlate with EKF yaw or sun azimuth: the live capture was stopped
  at the coordinator's request, so that correlation was not done.

## Ruled out
* **RK ISP / rkaiq / IQ files.** video22 is an rkcif node, so the RK ISP is not in the path.
  The stock OTA contains no rkaiq or 3A server, and `iqfiles/` has no SC132GS/XC9080 entry
  (these are generic Rockchip SDK files). None of this applies.
* **AWB.** It runs inside the XC9080, from the register tables in `regs_xc9080.h`. video22
  exposes no V4L2 WB control (`docs/vio.md` "Exposure/gain control"). The XC9080 AWB is
  plausibly fine. Re-check this after the fix.
* **IR-cut filter.** The cast is not magenta or pink foliage, and the vendor sample has
  natural colours.

## Fix (branch `ws/camera-colour`)
* `v4l2.py`: `to_bgr`, `eye_to_bgr` and decimation accept `YVYU`. A new `colour_format()`
  helper does the swap.
* `stereo_cam` gets a new parameter `color_order` (default `YVYU`), used only for
  `image_color`. Capture negotiation (`pixel_format: YUYV`) is unchanged, so there is no risk
  to the proven device setup.
* Unit tests are in `test/test_stereo_split.py`.

## Effect on perception
* `seg_ros` and `det_ros` run on swapped colour today:
  - grass hue is shifted;
  - sky and blue or orange objects are inverted.
* The impact is most likely on grass/non-grass segmentation, which is colour-driven. YOLO
  detection is less affected, since it is mostly shape-driven, but its scores may still drop.
* Stereo depth (`video11`) and VIO (mono8 luma) are unaffected.

## Owner action
Rebuild and deploy `mower_cameras`, then restart the camera launch. Check that the front
tiles in the GUI show natural colours, then re-check the seg mask on grass. Nothing on the
mower needs to be set.

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
