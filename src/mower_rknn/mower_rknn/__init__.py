"""``mower_rknn`` — shared RK3588S NPU inference helpers for the perception nodes.

Thin wrapper over Rockchip's ``rknn-toolkit-lite2`` (on-device runtime) so ``det_ros``
(YOLOv8) and ``seg_ros`` (PP-LiteSeg) share one model-load/inference path.
"""
from .rknn_runner import RknnRunner, RknnUnavailable, CORE_MASK  # noqa: F401
from .preprocess import letterbox, bgr_to_rgb_nhwc  # noqa: F401
