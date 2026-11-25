"""``seg_ros`` — PP-LiteSeg terrain segmentation on the RK3588S NPU.

Replaces the closed ``seg_ros`` / ``seg_oacam`` binaries. Runs the shipped
``pp37.rknn`` (6 classes) on the left/right OA camera streams.
"""
from .ppseg_postprocess import argmax_mask, traversability_mask  # noqa: F401
