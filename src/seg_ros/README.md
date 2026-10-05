# seg_ros — PP-LiteSeg terrain segmentation (RK3588S NPU)

Replaces the closed `seg_ros` / `seg_oacam` binaries. Runs the shipped 6-class
`pplite-seg_20260630-1-6cls.rknn` via `rknn-toolkit-lite2`, pinned to NPU **core 1**
(the original pinned itself to core 1 with `rknn_set_core_mask(RKNN_NPU_CORE_1)`).

## Model

| Model | Classes | Input | Size |
|---|---|---|---|
| `pplite-seg_20260630-1-6cls.rknn` | 6 | **640×480** (W×H) int8 NHWC | 9.5 MB |

Input shape recovered from the `.rknn` static metadata (`shape [1,3,480,640]` NCHW →
480 tall × 640 wide). Class order (from the original `ppseg.cc`):

| idx | class | traversable |
|---|---|---|
| 0 | other / BG | no |
| 1 | grass | **yes** |
| 2 | passway | no |
| 3 | soil | **yes** |
| 4 | obstacle | no |
| 5 | shrub | no (soft obstacle) |

## Topics

| Direction | Topic | Type |
|---|---|---|
| in | `/left_oa_camera/image_raw` | `sensor_msgs/Image` |
| in | `/right_oa_camera/image_raw` | `sensor_msgs/Image` |
| out | `/ai/seg/mask` | `sensor_msgs/Image` mono8 (class index 0..5) |
| out | `/ai/seg/traversability` | `sensor_msgs/Image` mono8 (255 nav / 0 obstacle) |
| out | `/ai/seg/image_overlay` | `sensor_msgs/Image` bgr8 (debug) |

## Run

```bash
ros2 run seg_ros seg_ros \
  --ros-args -p model_path:=model/pplite-seg_20260630-1-6cls.rknn \
  -p core_mask:="1"
```

## OA fusion note

The original fused segmentation into obstacle pointclouds and line segments using a
per-camera `points.txt` depth mapping (monocular geometric projection). That fusion
node (`fusion_ros_node`) is a separate piece; this package provides the raw
class/traversability masks as the clean input to a replacement fusion step.
