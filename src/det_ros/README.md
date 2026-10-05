# det_ros — YOLOv8 obstacle detection (RK3588S NPU)

Replaces the closed `det_ros` binary (see `mower_docs/03-yolo-perception.md`). Runs the
shipped `.rknn` models on the NPU via `rknn-toolkit-lite2`.

## Models (in `ros2_port_handoff/11_perception_models/`)

| Model | Classes | Input | Size |
|---|---|---|---|
| `best_large_0208.rknn` | 22 (see `config/det.yaml`) | **480×640** (W×H) int8 NHWC | 28.8 MB |
| `best_small_0208.rknn` | 2 (`stone`, `leaf`) | 480×640 int8 NHWC | 11.1 MB |

Input shape was recovered directly from the `.rknn` static metadata
(`shape [1,3,640,480]` NCHW → 640 tall × 480 wide) — **not** the 640×640 square the
earlier docs guessed. The node letterboxes to this size and maps boxes back.

## Topics

| Direction | Topic | Type |
|---|---|---|
| in | `/left_oa_camera/image_raw` | `sensor_msgs/Image` |
| in | `/right_oa_camera/image_raw` | `sensor_msgs/Image` |
| out | `/ai/det/detections` | `vision_msgs/Detection2DArray` |
| out | `/ai/det/image_annotated` | `sensor_msgs/Image` (bgr8, debug) |

The `Detection2DArray.header.frame_id` carries the source camera frame so the planner/OF
layer can tell left from right. Detection `results[i].id` is the class name; `class_id`
is the integer index.

## OA obstacle contract (to reconcile)

The closed OA/planner nodes consumed vendor-specific foxglove/custom pointcloud topics
(`/ai/det/pointcloud_box_ob`, `/ai/det/bgr_draw_box`, etc.). This node publishes the
standard ROS 2 `vision_msgs` surface instead; the consumer that turns detections into
drive commands (obstacle avoidance) still needs to be written to this contract. Kept
deliberately standard so it stays reusable.

## Run

```bash
ros2 run det_ros det_ros \
  --ros-args -p model_path:=model/best_large_0208.rknn \
  -p core_mask:="0"
```

## NPU core allocation

- `det_ros` → core 0 (`core_mask: "0"`)
- `seg_ros` → core 1 (`core_mask: "1"`)
- (spare) core 2

Matches the original split (det on core 0, seg pinned to core 1).

## Prerequisite

`rknn-toolkit-lite2` (aarch64, matches on-device `librknnrt` 2.1.0) must be installed in
the Humble container — see `mower_rknn` package README. The import is guarded so the
package still imports (and logs a clear error) away from the NPU.
