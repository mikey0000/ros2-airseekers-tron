"""``det_ros`` — YOLOv8 obstacle detection on the RK3588S NPU.

Replaces the closed ``det_ros`` binary (YOLOv8, see `mower_docs/03-yolo-perception.md`).
Runs the shipped ``best_{large,small}_0208.rknn`` on the left/right OA camera streams.
"""
from .yolo_postprocess import post_process  # noqa: F401
