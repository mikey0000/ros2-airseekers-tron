# SPDX-License-Identifier: GPL-3.0-or-later
"""Rate caps for the GUI-only ``/gui/*`` relays (ROS-free, unit-tested).

foxglove_bridge spends a few ms per delivered message per client and has no
per-topic rate limit, so the GUI subscribes to low-rate copies of the busy
topics instead of the originals (whose rates the motion path needs):
``/gui/pose`` (filtered_map, 5 Hz), ``/gui/status`` (2 Hz), ``/gui/emergency``
(immediately on change, else 1 Hz), ``/gui/detections`` (per camera 2 Hz,
only while something subscribes to it), ``/gui/obstacle_close`` (the 13 Hz vision Bool:
on change, else a 1 Hz keep-alive) and ``/gui/diagnostics`` (the ~9 Hz /diagnostics from
six publishers merged to one array per second, latest status per name).
"""


class Throttle:
    """``due(now, key, changed)``: True at most once per ``period`` per key, or at once
    when ``changed`` (a state change the GUI must see without delay). period <= 0 = all."""

    def __init__(self, period):
        self.period = float(period)
        self._last = {}

    def due(self, now, key=None, changed=False):
        last = self._last.get(key)
        if changed or self.period <= 0.0 or last is None or now - last >= self.period:
            self._last[key] = now
            return True
        return False


def period_for(rate_hz):
    """Rate in Hz -> period with 5 % slack (a jittery source still yields the rate)."""
    rate_hz = float(rate_hz)
    return 0.95 / rate_hz if rate_hz > 0.0 else 0.0


_UNSET = object()


class ChangeOrKeepalive:
    """``due(value, now)``: True when ``value`` differs from the previous one (first value
    included), else at most once per ``period`` (keep-alive)."""

    def __init__(self, period):
        self._thr = Throttle(period)
        self._last = _UNSET

    def due(self, value, now):
        changed = self._last is _UNSET or value != self._last
        self._last = value
        return self._thr.due(now, changed=changed)


class LatestByName:
    """Statuses received since the last ``take()``, latest wins per name (first-seen order).
    Only what was updated in the window is forwarded, so the GUI's per-name receive time
    (and its stale greying) still follows the real publishers."""

    def __init__(self):
        self._items = {}

    def add(self, name, item):
        self._items[name] = item

    def take(self):
        out, self._items = list(self._items.values()), {}
        return out

    def __len__(self):
        return len(self._items)
