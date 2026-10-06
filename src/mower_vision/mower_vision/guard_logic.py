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
# /obstacle_policy: classes that move away by themselves (mission: stop, blade off, wait).
# Every other class is static (mission: blade-off detour around it).
DEFAULT_DYNAMIC: List[str] = ['person', 'dog', 'cat']


@dataclass(frozen=True)
class Box:
    label: str
    score: float
    cx: float
    cy: float
    w: float
    h: float
    range_m: Optional[float] = None     # measured range (m), None = unknown
    bearing_deg: Optional[float] = None  # measured bearing (deg, base_link, left +)

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


# ---------------------------------------------------------------------------
# /obstacle_policy (consumed by mower_mission)
# ---------------------------------------------------------------------------
# Priority order. 'unranged' is advisory only (mower_mission ignores it).
POLICY_KINDS = ('dynamic', 'static', 'unranged')
NONE_POLICY = {'kind': 'none', 'class': '', 'distance_m': None, 'bearing_deg': None}


# ---------------------------------------------------------------------------
# Camera roles + motion relevance (owner rule: "if something is not in front, or to its
# side while turning, or behind while reversing, it should move")
# ---------------------------------------------------------------------------
CAM_FRONT, CAM_LEFT, CAM_RIGHT, CAM_REAR = 'front', 'left', 'right', 'rear'
CAMERA_ROLES = (CAM_FRONT, CAM_LEFT, CAM_RIGHT, CAM_REAR)
# Detection2DArray header.frame_id -> role. The rear camera ('rear_camera') is not run
# through the detector yet; once det_ros publishes it, its boxes count while reversing.
DEFAULT_CAMERA_FRAMES: Dict[str, List[str]] = {
    CAM_FRONT: ['vio_camera'],
    CAM_LEFT: ['left_oa_camera'],
    CAM_RIGHT: ['right_oa_camera'],
    CAM_REAR: ['rear_camera'],
}


def camera_role(frame_id: str, frames: Dict[str, Sequence[str]]) -> str:
    """Role of a detection frame; unknown frames are treated as front (conservative)."""
    fid = (frame_id or '').strip().lstrip('/')
    for role in CAMERA_ROLES:
        if fid in {str(x).strip().lstrip('/') for x in frames.get(role, ())}:
            return role
    return CAM_FRONT


@dataclass
class MotionConfig:
    lin_deadband_mps: float = 0.03    # |v| below this: not driving forward / backward
    ang_deadband_rps: float = 0.15    # |w| below this: not turning (controller jitter)
    hold_s: float = 0.6               # a command keeps counting this long after it


class MotionState:
    """Last commanded /cmd_vel (unstamped Twist) -> cameras relevant to the motion."""

    def __init__(self, cfg: Optional[MotionConfig] = None):
        self.cfg = cfg or MotionConfig()
        self._last_fwd: Optional[float] = None
        self._last_rev: Optional[float] = None
        self._last_left: Optional[float] = None
        self._last_right: Optional[float] = None

    def update(self, linear_x: float, angular_z: float, now: float) -> None:
        c = self.cfg
        if linear_x > c.lin_deadband_mps:
            self._last_fwd = now
        elif linear_x < -c.lin_deadband_mps:
            self._last_rev = now
        if angular_z > c.ang_deadband_rps:
            self._last_left = now
        elif angular_z < -c.ang_deadband_rps:
            self._last_right = now

    def _recent(self, t: Optional[float], now: float) -> bool:
        return t is not None and now - t <= self.cfg.hold_s

    def relevant(self, now: float) -> set:
        """Front counts unless the robot is only reversing (standing still / about to drive
        off forward keeps the front camera armed); a side counts while turning toward it;
        rear while reversing."""
        out = set()
        rev, fwd = self._recent(self._last_rev, now), self._recent(self._last_fwd, now)
        if fwd or not rev:
            out.add(CAM_FRONT)
        if rev:
            out.add(CAM_REAR)
        if self._recent(self._last_left, now):
            out.add(CAM_LEFT)
        if self._recent(self._last_right, now):
            out.add(CAM_RIGHT)
        return out


# ---------------------------------------------------------------------------
# Per-area obstacle_detection level (mower_mission pushes it as a parameter)
# ---------------------------------------------------------------------------
OBSTACLE_LEVELS = ('none', 'standard', 'sensitive')
DEFAULT_LEVEL = 'standard'


@dataclass
class PolicyConfig:
    dynamic_classes: Sequence[str] = field(default_factory=lambda: list(DEFAULT_DYNAMIC))
    min_score: float = 0.4
    dynamic_range_m: float = 1.0      # dynamic: same reach as the guard stop
    static_range_m: float = 1.5       # static: a little further (explains a controller abort)
    # Unranged boxes (side cameras, or stereo ranging failed): close when the bbox is at
    # least this fraction of the image height tall (person ~within 1.5 m).
    unranged_h_frac: float = 0.45
    level: str = DEFAULT_LEVEL        # none | standard | sensitive
    any_camera: bool = False          # True: every camera counts regardless of motion
    # STATIC needs a RANGED box within static_range_m (2026-10-07: unranged "shovel" / "hoe"
    # boxes from the side cameras made the mission detour around nothing). An unranged
    # static-class box only yields the advisory kind 'unranged' (the mission never acts on
    # it). True (sensitive level, owner opt-in via LevelTable.sensitive_static_unranged)
    # lets a tall unranged box produce 'static' again.
    static_unranged: bool = False


@dataclass
class LevelTable:
    """Thresholds the levels map to (obstacle_guard parameters)."""
    standard_stop_range_m: float = 1.0
    standard_min_score: float = 0.4
    sensitive_stop_range_m: float = 1.5
    sensitive_min_score: float = 0.35
    static_range_m: float = 1.5
    unranged_h_frac: float = 0.45
    # Owner opt-in (obstacle_guard param sensitive_static_unranged): at level 'sensitive'
    # unranged static-class boxes (bbox-height proxy) produce 'static' (-> detours).
    sensitive_static_unranged: bool = False


def level_policy(level: str, base: PolicyConfig, table: LevelTable) -> PolicyConfig:
    """PolicyConfig for ``level`` (unknown -> standard)."""
    lvl = level if level in OBSTACLE_LEVELS else DEFAULT_LEVEL
    if lvl == 'sensitive':
        stop, score, anyc = table.sensitive_stop_range_m, table.sensitive_min_score, True
        sunr = bool(table.sensitive_static_unranged)
    else:
        stop, score, anyc = table.standard_stop_range_m, table.standard_min_score, False
        sunr = False
    return replace(base, level=lvl, min_score=float(score), dynamic_range_m=float(stop),
                   static_range_m=max(float(table.static_range_m), float(stop)),
                   unranged_h_frac=float(table.unranged_h_frac), any_camera=anyc,
                   static_unranged=sunr)


def camera_counts(role: str, relevant: Optional[set], cfg: PolicyConfig) -> bool:
    """Whether a camera's detections may produce a policy at all (level + motion)."""
    if cfg.level == 'none':
        return False
    if cfg.any_camera or relevant is None:
        return True
    return role in relevant


def box_close(b: Box, reach: float, cfg: PolicyConfig, image_height: float) -> bool:
    """Ranged: stereo range within ``reach``; unranged: bbox-height proxy."""
    if b.range_m is not None:
        return b.range_m <= reach
    return image_height > 0 and b.h >= cfg.unranged_h_frac * image_height


def frame_policy(boxes: Iterable[Box], cfg: PolicyConfig, guard_cfg: GuardConfig,
                 role: str = CAM_FRONT, relevant: Optional[set] = None) -> dict:
    """Closest relevant detection of one frame: any dynamic one wins over static ones.

    Nothing counts at level ``none`` or when camera ``role`` is not in ``relevant`` (the
    cameras that matter for the current motion, :class:`MotionState`; None = all), unless
    ``cfg.any_camera`` (sensitive). Ranged boxes count within ``dynamic_range_m`` /
    ``static_range_m``; unranged ones need bbox height >= ``unranged_h_frac`` * image
    height (``guard_cfg.image_height``). A static-class box must be RANGED to give
    ``static``; a close unranged one gives the advisory ``unranged`` (lowest priority)
    unless ``cfg.static_unranged``."""
    if not camera_counts(role, relevant, cfg):
        return dict(NONE_POLICY)
    dyn = {c.strip().lower() for c in cfg.dynamic_classes}
    best = {}
    for b in boxes:
        if b.score < cfg.min_score:
            continue
        kind = 'dynamic' if b.label.strip().lower() in dyn else 'static'
        reach = cfg.dynamic_range_m if kind == 'dynamic' else cfg.static_range_m
        close = box_close(b, reach, cfg, float(guard_cfg.image_height))
        if not close:
            continue
        if kind == 'static' and b.range_m is None and not cfg.static_unranged:
            kind = 'unranged'
        key = b.range_m if b.range_m is not None else float('inf')
        cur = best.get(kind)
        if cur is None or key < cur[0]:
            best[kind] = (key, b)
    for kind in POLICY_KINDS:
        if kind in best:
            b = best[kind][1]
            return {'kind': kind, 'class': b.label,
                    'distance_m': None if b.range_m is None else round(b.range_m, 2),
                    'bearing_deg': None if b.bearing_deg is None else round(b.bearing_deg, 1)}
    return dict(NONE_POLICY)


class PolicyState:
    """Per-source hold of the last non-none policy (debounces detector flicker); the
    combined policy prefers dynamic, then the closest."""

    def __init__(self, hold_s: float = 0.5):
        self.hold_s = float(hold_s)
        self._last: Dict[str, Tuple[float, dict]] = {}

    def update(self, source: str, policy: dict, now: float) -> None:
        if policy.get('kind', 'none') != 'none':
            self._last[source] = (now, dict(policy))

    def current(self, now: float) -> dict:
        live = [p for t, p in self._last.values() if now - t <= self.hold_s]
        for kind in POLICY_KINDS:
            cands = [p for p in live if p['kind'] == kind]
            if cands:
                return min(cands, key=lambda p: p['distance_m']
                           if p['distance_m'] is not None else float('inf'))
        return dict(NONE_POLICY)


# Activity mode (/mission/activity, mower_mission/activity.py): only these two allow an idle
# duty cycle; anything else (or no message yet) keeps the full rate.
LOW_POWER_ACTIVITIES = frozenset(('docked_idle', 'idle'))


def tick_rate_hz(activity, busy, burst_rate_hz, idle_rate_hz):
    """Guard tick rate: ``burst_rate_hz`` unless the robot is idle (activity low power) and
    nothing is pending (``busy`` = close latched or a zero burst running)."""
    if busy or activity not in LOW_POWER_ACTIVITIES or idle_rate_hz <= 0.0:
        return float(burst_rate_hz)
    return min(float(idle_rate_hz), float(burst_rate_hz))
