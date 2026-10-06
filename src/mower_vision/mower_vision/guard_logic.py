"""Pure decision logic for ``obstacle_guard`` (no ROS imports; unit-tested on any host).

Danger-zone rule (image coordinates, origin top-left, y down): a detection of a
whitelisted class is *close* when

    bbox bottom edge  > y_frac * image_height     (low in the image = near the mower)
    AND bbox width    > w_frac * image_width      (large = near)

When a detection carries a measured range (``Box.range_m``, from ``det_range`` via
``/ai/det/detections_ranged``) the image-space rule is replaced by
``range_m <= stop_range_m``; boxes without range keep the image-space heuristic.

A per-camera hold (``hold_s``) debounces flicker: a camera stays "close" for ``hold_s``
after its last close detection. The overall state is the OR over cameras; a rising edge of
the overall state triggers the stop actions (zero-twist burst + cutter off).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# Must match det_ros/labels.py BEST_LARGE_CLASSES (best_large_0208.rknn export order).
DEFAULT_CLASSES: List[str] = [
    'person', 'dog', 'cat', 'sports ball', 'hedgehog', 'rabbit', 'stone', 'hoe',
    'shovel', 'manhole', 'brick', 'trashbin', 'toycars', 'potted plant', 'cans',
    'bottle', 'book', 'backpack', 'wood', 'chair', 'trunk', 'dock',
]
# Living things the mower must never drive into / mow over.
DEFAULT_WHITELIST: List[str] = ['person', 'dog', 'cat', 'hedgehog', 'rabbit']


@dataclass(frozen=True)
class Box:
    label: str
    score: float
    cx: float
    cy: float
    w: float
    h: float
    range_m: Optional[float] = None     # measured range (m), None = unknown

    @property
    def x1(self) -> float:
        return self.cx - self.w / 2.0

    @property
    def y1(self) -> float:
        return self.cy - self.h / 2.0

    @property
    def x2(self) -> float:
        return self.cx + self.w / 2.0

    @property
    def y2(self) -> float:
        return self.cy + self.h / 2.0


@dataclass
class GuardConfig:
    whitelist: Sequence[str] = field(default_factory=lambda: list(DEFAULT_WHITELIST))
    y_frac: float = 0.6
    w_frac: float = 0.15
    min_score: float = 0.4
    image_width: int = 1920
    image_height: int = 1080
    stop_range_m: float = 1.0


def with_image_size(cfg: GuardConfig, width: int, height: int) -> GuardConfig:
    """``cfg`` for an actual published image size (bbox pixels are in that image's space);
    non-positive sizes keep the configured fallback."""
    if int(width) <= 0 or int(height) <= 0:
        return cfg
    return replace(cfg, image_width=int(width), image_height=int(height))


def class_label(class_id: str, classes: Sequence[str]) -> str:
    """det_ros publishes names; accept a bare integer index too (mapped via ``classes``)."""
    cid = str(class_id).strip()
    if cid.lstrip('-').isdigit():
        idx = int(cid)
        if 0 <= idx < len(classes):
            return classes[idx]
    return cid


def is_whitelisted(label: str, whitelist: Iterable[str]) -> bool:
    wl = {w.strip().lower() for w in whitelist}
    return '*' in wl or label.strip().lower() in wl


def in_danger_zone(box: Box, cfg: GuardConfig) -> bool:
    return (box.y2 > cfg.y_frac * cfg.image_height
            and box.w > cfg.w_frac * cfg.image_width)


def is_close(box: Box, cfg: GuardConfig) -> bool:
    """Measured range when present, else the image-space danger zone."""
    if box.range_m is not None:
        return box.range_m <= cfg.stop_range_m
    return in_danger_zone(box, cfg)


def classify(boxes: Iterable[Box], cfg: GuardConfig) -> List[Tuple[Box, bool, bool]]:
    """``[(box, relevant, close)]``: relevant = whitelisted and score >= min_score."""
    out = []
    for b in boxes:
        relevant = b.score >= cfg.min_score and is_whitelisted(b.label, cfg.whitelist)
        out.append((b, relevant, relevant and is_close(b, cfg)))
    return out


def any_close(boxes: Iterable[Box], cfg: GuardConfig) -> bool:
    return any(close for _, _, close in classify(boxes, cfg))


class GuardState:
    """Per-source hold + overall rising-edge detection + stop-burst window."""

    def __init__(self, hold_s: float = 0.5, burst_s: float = 1.0):
        self.hold_s = float(hold_s)
        self.burst_s = float(burst_s)
        self._last_close: Dict[str, float] = {}
        self._was_close = False
        self.burst_until: Optional[float] = None

    def is_close(self, now: float) -> bool:
        return any(now - t <= self.hold_s for t in self._last_close.values())

    def update(self, source: str, close_now: bool, now: float) -> Tuple[bool, bool]:
        """Feed one detection frame. Returns ``(overall_close, rising_edge)``."""
        if close_now:
            self._last_close[source] = now
        return self.tick(now)

    def tick(self, now: float) -> Tuple[bool, bool]:
        """Re-evaluate without new data (hold expiry). Returns ``(close, rising_edge)``."""
        close = self.is_close(now)
        rising = close and not self._was_close
        self._was_close = close
        if rising:
            self.burst_until = now + self.burst_s
        return close, rising

    def burst_active(self, now: float) -> bool:
        return self.burst_until is not None and now < self.burst_until


def box_outline(box: Box) -> List[Tuple[float, float]]:
    """8 points (4 segments) for a LINE_LIST outline of ``box``."""
    a, b, c, d = (box.x1, box.y1), (box.x2, box.y1), (box.x2, box.y2), (box.x1, box.y2)
    return [a, b, b, c, c, d, d, a]


def danger_line(cfg: GuardConfig) -> List[Tuple[float, float]]:
    y = cfg.y_frac * cfg.image_height
    return [(0.0, y), (float(cfg.image_width), y)]
