# SPDX-License-Identifier: GPL-3.0-or-later
"""Change-or-period publish gate (pure) and the /mission/activity idle test."""

LOW_POWER_ACTIVITIES = ('docked_idle', 'idle')


def activity_low_power(activity):
    """True only for the explicit idle modes of /mission/activity (unknown -> active)."""
    return activity in LOW_POWER_ACTIVITIES


class ChangeOrPeriodGate:
    """``allow(key, now, period)``: True when ``key`` changed since the last allowed call
    or ``period`` seconds have passed (period <= 0: always). A change is never delayed."""

    def __init__(self):
        self._key = object()
        self._t = None

    def allow(self, key, now, period):
        if period <= 0.0 or key != self._key or self._t is None or now - self._t >= period:
            self._key, self._t = key, now
            return True
        return False
