"""Image preprocessing shared by the RKNN perception nodes.

Both shipped models expect an int8 **NHWC** RGB tensor (no 1/255 normalisation — the
calibration baked the scale into the quantised weights). See
`mower_docs/03-yolo-perception.md` ("Input: one tensor, int8, NHWC").
"""

from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np


def letterbox(img_bgr: np.ndarray, size: Tuple[int, int],
              pad_color=(0, 0, 0)) -> Tuple[np.ndarray, float, Tuple[int, int]]:
    """Resize ``img_bgr`` to fit ``size=(w, h)`` preserving aspect ratio, pad to square.

    Returns ``(padded_bgr, scale, (pad_x, pad_y))``. ``scale`` is ``new/orig`` on the
    side that was scaled; ``pad_x/pad_y`` are the left/top padding added, so a detection
    box in resized space maps back with ``(x - pad_x)/scale``.
    """
    w, h = size
    ih, iw = img_bgr.shape[:2]
    scale = min(w / iw, h / ih)
    nw, nh = int(round(iw * scale)), int(round(ih * scale))
    resized = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_x = (w - nw) // 2
    pad_y = (h - nh) // 2
    padded = np.full((h, w, 3), pad_color, dtype=np.uint8)
    padded[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
    return padded, scale, (pad_x, pad_y)


def bgr_to_rgb_nhwc(img_bgr: np.ndarray) -> np.ndarray:
    """BGR -> RGB and layout to H,W,C (NHWC) uint8. No normalisation/mean subtraction."""
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb)
