"""PP-LiteSeg post-processing.

The shipped ``pplite-seg_20260630-1-6cls.rknn`` has a single NCHW output
``[1, 6, H, W]`` (6 classes, dequantised to float). We argmax over the class axis to
produce a class-index mask, matching the original `draw_segment_image` in
`src/mower_perception/seg_ros/src/ppseg.cc`.

Class semantics (from the original `cityscapes_label` table):
0 other/BG, 1 grass (navigable), 2 passway, 3 soil (navigable), 4 obstacle, 5 shrub.
"""

from __future__ import annotations

from typing import List

import numpy as np

# Class index -> (label, is_navigable)
CLASSES: List[tuple] = [
    ("other", False),
    ("grass", True),
    ("passway", False),
    ("soil", True),
    ("obstacle", False),
    ("shrub", False),
]


def argmax_mask(logits: np.ndarray) -> np.ndarray:
    """``logits`` is ``[C, H, W]`` (or ``[1, C, H, W]``) -> ``[H, W]`` uint8 class indices."""
    if logits.ndim == 4:
        logits = logits[0]
    return np.argmax(logits, axis=0).astype(np.uint8)


def traversability_mask(class_mask: np.ndarray) -> np.ndarray:
    """Class-index mask -> uint8 traversability (255 = navigable, 0 = obstacle)."""
    trav = np.zeros_like(class_mask, dtype=np.uint8)
    for idx, (_label, nav) in enumerate(CLASSES):
        if nav:
            trav[class_mask == idx] = 255
    return trav


def colorize(class_mask: np.ndarray) -> np.ndarray:
    """Class-index mask -> bgr8 debug overlay (navigable white, obstacle colour-coded)."""
    # Distinct BGR colours so the classes are visually separable.
    palette = np.array([
        [0, 0, 0],        # other
        [255, 255, 255],  # grass
        [0, 0, 128],      # passway
        [0, 255, 0],      # soil
        [0, 0, 255],      # obstacle (red)
        [0, 255, 255],    # shrub (yellow)
    ], dtype=np.uint8)
    return palette[class_mask]


def resize_mask(mask: np.ndarray, size) -> np.ndarray:
    """Nearest-neighbour a mask up to ``(w, h)`` (keeps class indices exact)."""
    import cv2
    return cv2.resize(mask, size, interpolation=cv2.INTER_NEAREST)
