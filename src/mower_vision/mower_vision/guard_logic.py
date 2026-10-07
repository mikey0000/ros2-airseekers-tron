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
    # Side cameras (standard level, owner rule 2026-10-07: "left and right obstacle camera
    # shouldn't affect direct path mowing unless it's close and the mower is turning"):
    # the APPLIED /cmd_vel must turn toward that side above side_turn_min_radps
    # continuously for side_turn_min_s, AND odometry must show the robot actually turning.
    side_turn_min_radps: float = 0.15
    side_turn_min_s: float = 0.5
    odom_turn_min_radps: float = 0.1
    odom_hold_s: float = 0.5          # older odom = unknown = not turning


class MotionState:
    """Last commanded /cmd_vel (unstamped Twist) -> cameras relevant to the motion."""

    def __init__(self, cfg: Optional[MotionConfig] = None):
        self.cfg = cfg or MotionConfig()
        self._last_fwd: Optional[float] = None
        self._last_rev: Optional[float] = None
        self._last_left: Optional[float] = None
        self._last_right: Optional[float] = None
        self._turn_side: Optional[str] = None     # side of the current sustained turn
        self._turn_since: Optional[float] = None
        self._turn_last: Optional[float] = None
        self._odom_w: Optional[float] = None
        self._odom_t: Optional[float] = None

    def update_odom(self, angular_z: float, now: float) -> None:
        self._odom_w, self._odom_t = float(angular_z), now

    def odom_turning(self, now: float) -> bool:
        return (self._odom_t is not None and now - self._odom_t <= self.cfg.odom_hold_s
                and abs(self._odom_w) > self.cfg.odom_turn_min_radps)

    def sustained_turn(self, side: str, now: float) -> bool:
        """Commanded turn toward ``side`` above side_turn_min_radps for >= side_turn_min_s
        (unbroken) and still current (last such command within hold_s)."""
        c = self.cfg
        return (self._turn_side == side and self._turn_since is not None
                and now - self._turn_last <= c.hold_s
                and self._turn_last - self._turn_since >= c.side_turn_min_s - 1e-9
                if self._turn_last is not None else False)

    def side_active(self, side: str, now: float) -> bool:
        return self.sustained_turn(side, now) and self.odom_turning(now)

    def side_reason(self, side: str, now: float) -> str:
        """Why a side camera does not count right now ('' = it counts)."""
        if not self.sustained_turn(side, now):
            return 'not turning'
        if not self.odom_turning(now):
            return 'not turning (odom)'
        return ''

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
        side = (CAM_LEFT if angular_z > c.side_turn_min_radps
                else CAM_RIGHT if angular_z < -c.side_turn_min_radps else None)
        if side is None:
            self._turn_side = self._turn_since = self._turn_last = None
        else:
            if side != self._turn_side or self._turn_last is None \
                    or now - self._turn_last > c.hold_s:
                self._turn_side, self._turn_since = side, now
            self._turn_last = now

    def _recent(self, t: Optional[float], now: float) -> bool:
        return t is not None and now - t <= self.cfg.hold_s

    def relevant(self, now: float, strict_sides: bool = True) -> set:
        """Front counts unless the robot is only reversing (standing still / about to drive
        off forward keeps the front camera armed); rear while reversing; a side counts only
        while turning toward it: ``strict_sides`` (standard) = :meth:`side_active` (sustained
        commanded turn + odometry turning), else any recent command toward it."""
        out = set()
        rev, fwd = self._recent(self._last_rev, now), self._recent(self._last_fwd, now)
        if fwd or not rev:
            out.add(CAM_FRONT)
        if rev:
            out.add(CAM_REAR)
        for side, t in ((CAM_LEFT, self._last_left), (CAM_RIGHT, self._last_right)):
            if self.side_active(side, now) if strict_sides else self._recent(t, now):
                out.add(side)
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
    # 'standard' strict rules (2026-10-07, side-camera "cat" during swath-end pivots):
    # side cameras: ranged within side_stop_range_m, or unranged with bbox height >=
    # side_unranged_h_frac AND score >= side_unranged_min_score AND persistence.
    # front unranged dynamic: score >= front_unranged_min_score AND persistence.
    # persist_frames = consecutive frames (same camera + class) a box must qualify in.
    strict: bool = False
    side_stop_range_m: float = 0.8
    side_unranged_h_frac: float = 0.60
    side_unranged_min_score: float = 0.75
    front_unranged_min_score: float = 0.6
    persist_frames: int = 2


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
    side_stop_range_m: float = 0.8
    side_unranged_h_frac: float = 0.60
    side_unranged_min_score: float = 0.75
    front_unranged_min_score: float = 0.6
    persist_frames: int = 2


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
                   static_unranged=sunr, strict=(lvl == 'standard'),
                   side_stop_range_m=float(table.side_stop_range_m),
                   side_unranged_h_frac=float(table.side_unranged_h_frac),
                   side_unranged_min_score=float(table.side_unranged_min_score),
                   front_unranged_min_score=float(table.front_unranged_min_score),
                   persist_frames=int(table.persist_frames))


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


class Persistence:
    """Consecutive-frame counter per (source, class): a class seen qualifying in frame N
    of a source and again in frame N+1 has count 2. A frame without it resets it."""

    def __init__(self):
        self._n: Dict[Tuple[str, str], int] = {}

    def step(self, source: str, labels: Iterable[str]) -> Dict[str, int]:
        labels = {str(x).strip().lower() for x in labels}
        prev = {k: v for k, v in self._n.items() if k[0] == source}
        for k in prev:
            del self._n[k]
        out = {}
        for lab in labels:
            n = prev.get((source, lab), 0) + 1
            self._n[(source, lab)] = n
            out[lab] = n
        return out


def _policy_dict(kind: str, b: Box, role: str, image_height: float) -> dict:
    return {'kind': kind, 'class': b.label,
            'distance_m': None if b.range_m is None else round(b.range_m, 2),
            'bearing_deg': None if b.bearing_deg is None else round(b.bearing_deg, 1),
            'camera': role, 'score': round(float(b.score), 2),
            'bbox_h_frac': round(b.h / image_height, 2) if image_height > 0 else None,
            'ranged': b.range_m is not None}


def frame_policy(boxes: Iterable[Box], cfg: PolicyConfig, guard_cfg: GuardConfig,
                 role: str = CAM_FRONT, relevant: Optional[set] = None,
                 persistence: Optional[Persistence] = None, source: str = '',
                 ignored: Optional[list] = None, motion_reason: str = '') -> dict:
    """Closest relevant detection of one frame: any dynamic one wins over static ones.

    Nothing counts at level ``none`` or when camera ``role`` is not in ``relevant`` (the
    cameras that matter for the current motion, :class:`MotionState`; None = all), unless
    ``cfg.any_camera`` (sensitive). Ranged boxes count within ``dynamic_range_m`` /
    ``static_range_m``; unranged ones need bbox height >= ``unranged_h_frac`` * image
    height (``guard_cfg.image_height``). A static-class box must be RANGED to give
    ``static``; a close unranged one gives the advisory ``unranged`` (lowest priority)
    unless ``cfg.static_unranged``.

    ``cfg.strict`` (standard level) tightens it: side cameras need a ranged box within
    ``side_stop_range_m`` or an unranged one with bbox height >= ``side_unranged_h_frac``,
    score >= ``side_unranged_min_score`` and ``persist_frames`` consecutive frames; an
    unranged front dynamic needs score >= ``front_unranged_min_score`` and persistence.
    Persistence is counted in ``persistence`` (keyed by ``source``); None = not required.
    Whitelisted-relevant boxes dismissed by these rules are appended to ``ignored`` as
    ``(box, reason)`` (``motion_reason`` explains a camera excluded by motion)."""
    boxes = list(boxes)
    ih = float(guard_cfg.image_height)
    dyn = {c.strip().lower() for c in cfg.dynamic_classes}
    if not camera_counts(role, relevant, cfg):
        if persistence is not None:
            persistence.step(source, ())
        if ignored is not None and cfg.level != 'none':
            for b in boxes:
                if b.score >= cfg.min_score and b.label.strip().lower() in dyn:
                    ignored.append((b, motion_reason or 'camera not relevant to motion'))
        return dict(NONE_POLICY)
    side = role in (CAM_LEFT, CAM_RIGHT)
    cands = []        # (kind, box, needs_persistence)
    for b in boxes:
        if b.score < cfg.min_score:
            continue
        kind = 'dynamic' if b.label.strip().lower() in dyn else 'static'
        reach = cfg.dynamic_range_m if kind == 'dynamic' else cfg.static_range_m
        need_persist = False
        if cfg.strict and side:
            if b.range_m is not None:
                close = b.range_m <= min(reach, cfg.side_stop_range_m)
                why = 'ranged %.2f m > %.2f m' % (b.range_m, cfg.side_stop_range_m)
            else:
                hf = b.h / ih if ih > 0 else 0.0
                close = (hf >= cfg.side_unranged_h_frac
                         and b.score >= cfg.side_unranged_min_score)
                why = 'unranged, score %.2f, height %d%%' % (b.score, round(hf * 100))
                need_persist = True
        else:
            close = box_close(b, reach, cfg, ih)
            why = 'too far'
            if close and cfg.strict and b.range_m is None and kind == 'dynamic':
                if b.score < cfg.front_unranged_min_score:
                    close = False
                    why = 'unranged, score %.2f' % b.score
                need_persist = True
        if not close:
            if ignored is not None and kind == 'dynamic':
                ignored.append((b, why))
            continue
        if kind == 'static' and b.range_m is None and not cfg.static_unranged:
            kind = 'unranged'
        cands.append((kind, b, need_persist))
    counts = (persistence.step(source, [b.label for _, b, np_ in cands if np_])
              if persistence is not None else {})
    best = {}
    for kind, b, need_persist in cands:
        if need_persist and persistence is not None:
            n = counts.get(b.label.strip().lower(), 0)
            if n < cfg.persist_frames:
                if ignored is not None and kind == 'dynamic':
                    ignored.append((b, 'unranged, score %.2f, %d/%d frames'
                                    % (b.score, n, cfg.persist_frames)))
                continue
        key = b.range_m if b.range_m is not None else float('inf')
        cur = best.get(kind)
        if cur is None or key < cur[0]:
            best[kind] = (key, b)
    for kind in POLICY_KINDS:
        if kind in best:
            return _policy_dict(kind, best[kind][1], role, ih)
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
