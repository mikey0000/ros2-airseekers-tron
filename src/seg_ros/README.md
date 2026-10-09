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
| in | `/vio/right/image_color` (`extra_topics`, default since 2026-10-09) | `sensor_msgs/Image` |
| in | `/left_oa_camera/image_raw`, `/right_oa_camera/image_raw` (vendor; blank in seg.yaml) | `sensor_msgs/Image` |
| out | `/ai/seg/mask` | `sensor_msgs/Image` mono8 (class index 0..5) |
| out | `/ai/seg/confidence` | `sensor_msgs/Image` mono8 (top-1 softmax x 255, same stamp/frame) |
| out | `/ai/seg/traversability` | `sensor_msgs/Image` mono8 (255 nav / 0 obstacle) |
| out | `/ai/seg/image_overlay` | `sensor_msgs/Image` bgr8 (debug) |

## Run

```bash
# models: scripts/install_models.sh -> /userdata/ros2/models/ on the device
ros2 launch mower_vision perception.launch.py seg:=true
ros2 run seg_ros seg_ros --ros-args -p models_dir:=/some/dir -p core_mask:="1" -p dry_run:=true
```

Default `model_path` is `/userdata/ros2/models/pplite-seg_20260630-1-6cls.rknn`;
`models_dir` is tried with the same basename when that is missing. No `rknnlite` / no
model: one FATAL line + exit 1, or with `dry_run:=true` alive and silent. Runtime
requirements (rknn-toolkit-lite2 2.3.0 + `librknnrt.so`): `docs/cameras_and_video.md`.

## Non-grass projector (2026-10-09)

`nongrass_projector` (`seg_ros/ground_projection.py`, pure + unit-tested) projects the mask of
the front stereo right colour eye onto the ground and publishes one non-grass / grass vote per
0.1 m map cell per frame on `/ai/seg/ground_cells` (PointCloud2, map frame). mower_map's
map_server_node turns the votes into a remembered, confirmed, SOFT Nav2 cost
(`~/nongrass_cost` -> global costmap `nongrass_layer`). Launch:
`perception.launch.py nongrass:=true` (also starts seg_ros) or `mower.launch.py nongrass:=true`.
Design, thresholds and live checks: `docs/grass_segmentation.md`.

## OA fusion note

The original fused segmentation into obstacle pointclouds and line segments using a
per-camera `points.txt` depth mapping (monocular geometric projection). That fusion
node (`fusion_ros_node`) is a separate piece; this package provides the raw
class/traversability masks as the clean input to a replacement fusion step.
