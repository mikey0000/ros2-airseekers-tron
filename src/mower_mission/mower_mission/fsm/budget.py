# SPDX-License-Identifier: GPL-3.0-or-later
"""``Budget``: retry budgets and back-off schedules (2026-10-23 code review 1.7).

Two shapes recur across the mission's retry counters:

* a **windowed budget**: events younger than ``period_s`` count, ``limit`` of them spends it
  (stuck escapes ``stuck_max_recoveries`` per ``stuck_recovery_period_s``, steep-slope
  back-outs ``tilt_max_backouts`` per ``tilt_backout_period_s``);
* a **back-off schedule**: the wait before retry ``n`` (1-based) is ``schedule[n-1]``, the
  last entry repeating, ``default`` when the schedule is empty (transit retries
  ``transit_retry_backoff_s``, route re-asks ``route_plan_retry_backoff_s``).

The history lists stay where they are (``_stuck_hist``, ``_tilt_hist``: entries start with
their time); this only holds the arithmetic once.
"""


class Budget:
    def __init__(self, period_s, limit):
        self.period_s = float(period_s)
        self.limit = int(limit)

    def window(self, hist, now):
        """The entries of ``hist`` (``[t, ...]``) inside the window ending at ``now``."""
        horizon = now - self.period_s
        return [h for h in hist if h[0] >= horizon]

    def spent(self, hist):
        return len(hist) >= self.limit

    @staticmethod
    def backoff(schedule, n, default):
        """Wait before retry ``n`` (1-based; None = ``default``)."""
        b = list(schedule or [])
        if not b or n is None:
            return float(default)
        return float(b[min(max(int(n), 1), len(b)) - 1])
