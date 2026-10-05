"""ROS-free helpers for the cmd_vel WebSocket relay.

Adapted from MowgliNext (GPL-3.0) cmd_vel_ws_relay.py.
"""
import json
import math


def clamp(v, limit):
    """Clamp v to [-limit, limit]."""
    return max(-limit, min(limit, v))


def _num(section, key):
    if not isinstance(section, dict):
        raise ValueError(f"expected object, got {type(section).__name__}")
    val = section.get(key, 0.0)
    if isinstance(val, bool) or not isinstance(val, (int, float, str)):
        raise ValueError(f"non-numeric value for {key!r}: {val!r}")
    try:
        f = float(val)
    except (TypeError, ValueError):
        raise ValueError(f"non-numeric value for {key!r}: {val!r}") from None
    if not math.isfinite(f):
        raise ValueError(f"non-finite value for {key!r}: {val!r}")
    return f


def twist_fields_from_json(raw, max_linear, max_angular):
    """Decode and clamp an untrusted frame.

    Returns dict(linear=(x, y, z), angular=(x, y, z)). Raises ValueError on
    malformed JSON, wrong structure or non-numeric/non-finite values.
    Any header field is ignored.
    """
    try:
        d = json.loads(raw)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError(f"bad JSON: {exc}") from None
    if not isinstance(d, dict):
        raise ValueError("frame must be a JSON object")
    t = d.get("twist", {})
    if not isinstance(t, dict):
        raise ValueError("'twist' must be an object")
    lin = t.get("linear", {})
    ang = t.get("angular", {})
    return dict(
        linear=tuple(clamp(_num(lin, k), max_linear) for k in "xyz"),
        angular=tuple(clamp(_num(ang, k), max_angular) for k in "xyz"),
    )


class LeaseTracker:
    """Tracks freshness of the last received command."""

    def __init__(self, lease):
        self.lease = lease
        self._last = None

    def update(self, now):
        self._last = now

    def clear(self):
        self._last = None

    def should_replay(self, now):
        """True while a command was received and its lease is still fresh."""
        return self._last is not None and (now - self._last) <= self.lease

    def expired(self, now):
        """True if a command is outstanding and its lease has run out."""
        return self._last is not None and (now - self._last) > self.lease
