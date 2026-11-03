"""Drive the mission FSM from a recorded incident fixture (no ROS graph).

A fixture (``fixtures/<date>_<slug>.json``, docs/DEVELOPMENT.md "Incident -> regression") is
the input sequence of one field incident window, extracted from a bag + the mission log by
``scripts/replay/fsm_replay.py extract``::

    {"id", "bag", "log", "window_utc": [from, until], "t0": <epoch s of t = 0>,
     "fix": "<branch / commit>", "pre_fix": "<commit>", "notes": [...],
     "areas": [{"index", "name", "outer": [[x, y], ...], "obstacles": [...]}],
     "events": [{"t": s, "topic": "/heading_aligner/status", "msg": {...}},
                {"t": s, "log": "state A -> B (...)"}, ...]}

Topic events go through the real ``MissionNode._on_*`` handlers (bound to a shim holding
the Harness' FSM), so a replay exercises the same message -> ``Inputs`` parsing as the
robot. Log events are the recorded mission log (outcomes, transitions) for comparison and
documentation; they are not fed. Inputs the bag did not record directly are derived by the
extractor and carry ``"derived": "<how>"`` (e.g. /gps/status from /fix_status).
"""
import contextlib
import json
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES = os.path.join(HERE, 'fixtures')
sys.path.insert(0, os.path.dirname(HERE))           # test/ (Harness)

from mower_mission import mission_fsm as f  # noqa: E402

# recorded topic -> MissionNode handler (the mission.yaml default topic names)
HANDLERS = {
    '/heading_aligner/status': '_on_heading',
    '/localization/status': '_on_localization',
    '/tilt/status': '_on_tilt',
    '/obstacle_policy': '_on_obstacle_policy',
    '/obstacle_tracker/nogo_zones': '_on_nogo_zones',
    '/supervisor/status': '_on_supervisor',
    '/odometry/filtered_map': '_on_odom',
    '/odom': '_on_wheel_odom',
    '/gps/status': '_on_gnss',
    '/map_server_node/outside_m': '_on_outside',
    '/map_server_node/lethal_boundary_violation': '_on_lethal',
}
RECEIPT = {'_on_outside'}           # handlers that take (msg, receipt)
# Fields a handler reads that older fixtures did not record (recorded values win). hdop NaN
# = unknown: _on_gnss maps it to Inputs.hdop None, as for a receiver that reports none.
MSG_DEFAULTS = {
    '/gps/status': {'fix_type': 0, 'quality_percent': 0.0, 'hdop': float('nan')},
}


def load(name):
    path = name if os.path.isabs(name) or os.path.exists(name) else \
        os.path.join(FIXTURES, name if name.endswith('.json') else name + '.json')
    with open(path) as fh:
        return json.load(fh)


def ns(obj):
    """dict -> attribute namespace, recursively (what the handlers read off a message)."""
    if isinstance(obj, dict):
        return types.SimpleNamespace(**{k: ns(v) for k, v in obj.items()})
    if isinstance(obj, list):
        return [ns(v) for v in obj]
    return obj


def odom_msg(x, y, yaw):
    return {'pose': {'pose': {'position': {'x': x, 'y': y, 'z': 0.0},
                              'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2),
                                              'w': math.cos(yaw / 2)}}}}


def _msg(topic, m):
    """Expand the compact fixture forms."""
    if topic in ('/odometry/filtered_map', '/odom') and 'yaw' in m:
        return odom_msg(m['x'], m['y'], m['yaw'])
    return m


class _Log:
    def __init__(self):
        self.lines = []

    def _add(self, lvl, text, *a, **k):
        self.lines.append((lvl, str(text)))

    def info(self, t, *a, **k):
        self._add('I', t)

    def warn(self, t, *a, **k):
        self._add('W', t)

    warning = warn

    def error(self, t, *a, **k):
        self._add('E', t)

    def debug(self, t, *a, **k):
        pass


class Replay:
    """Feed fixture events into a test Harness at their recorded times (10 Hz ticks between
    them, as the node's PeriodicRunner)."""

    def __init__(self, h, fx, base=None):
        from mower_mission import mission_node as mn
        self.h, self.fx = h, fx
        self.base = h.t if base is None else base     # harness time of fixture t = 0
        self.events = sorted((e for e in fx['events'] if 'topic' in e),
                             key=lambda e: e['t'])
        self.k = 0
        self.log = _Log()
        clk = types.SimpleNamespace(now=lambda: h.t)
        self.node = types.SimpleNamespace(
            fsm=h.fsm, _lock=contextlib.nullcontext(), _clk=clk, get_logger=lambda: self.log,
            get_name=lambda: 'behavior_tree_node', _mp_policy=None, _bumper_on=False,
            _hw_rain=False, _base_rain=False, _hw_charging=False, _base_charging=False)
        self._fn = {}
        for topic, name in HANDLERS.items():
            fn = getattr(mn.MissionNode, name, None)
            if fn is not None:                    # older commits lack some inputs
                self._fn[topic] = (fn, name in RECEIPT)

    def apply(self, e):
        hit = self._fn.get(e['topic'])
        if hit is None:
            return
        fn, receipt = hit
        raw = _msg(e['topic'], e['msg'])
        if isinstance(raw, dict) and e['topic'] in MSG_DEFAULTS:
            raw = {**MSG_DEFAULTS[e['topic']], **raw}
        msg = ns(raw)
        fn(self.node, msg, self.h.t) if receipt else fn(self.node, msg)

    def run_until(self, t_rel, tick=True):
        """Apply every event with t <= t_rel, ticking the FSM in between; returns self."""
        while self.k < len(self.events) and self.events[self.k]['t'] <= t_rel + 1e-9:
            e = self.events[self.k]
            self._advance(self.base + e['t'], tick)
            self.apply(e)
            self.k += 1
        self._advance(self.base + t_rel, tick)
        return self

    def _advance(self, t_abs, tick):
        h = self.h
        while tick and h.t + 0.1 <= t_abs + 1e-9:
            h.tick()
        if h.t < t_abs:
            h.t = t_abs

    def last(self, topic, t_rel=None):
        """The last recorded message on ``topic`` at or before t_rel (None = any)."""
        out = None
        for e in self.events:
            if e['topic'] == topic and (t_rel is None or e['t'] <= t_rel):
                out = e
        return out


def recorded_log(fx, needle):
    """[(t, text)] of the recorded mission log lines containing ``needle``."""
    return [(e['t'], e['log']) for e in fx['events'] if 'log' in e and needle in e['log']]


def area_dict(a):
    return {'name': a['name'], 'outer': [tuple(p) for p in a['outer']],
            'obstacles': [[tuple(p) for p in o] for o in a.get('obstacles', [])],
            'is_navigation_area': bool(a.get('is_navigation_area', False))}


# ---- footprint geometry (independent of mower_mission.body_boundary, which the fixes
# changed): the 0.60 x 0.47 m body of FIELD_PTS in test_body_boundary.
BODY = {'front_left': (0.60, 0.235), 'front_right': (0.60, -0.235),
        'rear_left': (-0.12, 0.235), 'rear_right': (-0.12, -0.235)}


def _inside(pt, poly):
    x, y = pt
    c = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[i - 1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def _seg_dist(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L = dx * dx + dy * dy
    u = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - ax - u * dx, p[1] - ay - u * dy)


def outside_m(pt, poly):
    """Distance of a point outside the polygon (0 inside)."""
    if _inside(pt, poly):
        return 0.0
    return min(_seg_dist(pt, poly[i - 1], poly[i]) for i in range(len(poly)))


def worst_corner(pose, poly):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return max(outside_m((x + c * bx - s * by, y + s * bx + c * by), poly)
               for bx, by in BODY.values())
