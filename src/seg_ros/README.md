# seg_ros — PP-LiteSeg terrain segmentation (RK3588S NPU)

Replaces the closed `seg_ros` / `seg_oacam` binaries. Runs the shipped 6-class
`pp37.rknn` via `rknn-toolkit-lite2`, pinned to NPU **core 1**
(the original pinned itself to core 1 with `rknn_set_core_mask(RKNN_NPU_CORE_1)`).

## Model

| Model | Classes | Input | Size |
|---|---|---|---|
| `pp37.rknn` (default since 2026-11-25) | 6 | **640×480** (W×H) int8 NHWC | 9.5 MB |
| `pplite-seg_20260630-1-6cls.rknn` (previous default) | 6 | **640×480** (W×H) int8 NHWC | 9.5 MB |

`pp37.rknn` is the stock v1.3.33 retrain. Its embedded metadata matches the old model (input
`x` [1,3,480,640], mean/std 127.5, no rgb2bgr, one [1,6,480,640] output), and the stock
`ppseg.cc` class table did not change. Only the output quant range is narrower (logits
[-11.6, 12.1] against [-13.7, 16.4]), so the top-1 softmax confidence runs lower and
`nongrass_projector` `min_conf_nongrass` 0.8 may need re-tuning from a live overlay.
To go back: `model_path:=/userdata/ros2/models/pplite-seg_20260630-1-6cls.rknn`.

Deploy (models are not in git): the stock v1.3.33 install keeps it on the mower at
`/workspace/src/mower_perception/seg_ros/model/pp37.rknn` (`/workspace` links to the
versioned stock dir). Either copy it there, on the mower:
`cp /workspace/src/mower_perception/seg_ros/model/pp37.rknn /userdata/ros2/models/`,
or push it from an extracted stock tree:
`MODELS_SRC=<v1.3.33>/src/mower_perception/seg_ros/model ./scripts/install_models.sh`
(it lists the models it does not find there and copies the rest).

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

Default `model_path` is `/userdata/ros2/models/pp37.rknn`;
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
