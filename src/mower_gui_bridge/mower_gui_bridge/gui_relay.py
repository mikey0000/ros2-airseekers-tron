# SPDX-License-Identifier: GPL-3.0-or-later
"""Rate caps for the GUI-only ``/gui/*`` relays (ROS-free, unit-tested).

foxglove_bridge spends a few ms per delivered message per client and has no
per-topic rate limit, so the GUI subscribes to low-rate copies of the busy
topics instead of the originals (whose rates the motion path needs):
``/gui/pose`` (filtered_map, 5 Hz), ``/gui/status`` (2 Hz), ``/gui/emergency``
(immediately on change, else 1 Hz) and ``/gui/detections`` (per camera 2 Hz,
only while something subscribes to it).
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
