# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Costmap clearing and the timed retry of a phase.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (SRV_CLEAR_COSTMAPS)  # noqa: F401
from mower_mission.fsm.budget import Budget


class RetryMixin:
    """Costmap clearing and the timed retry of a phase."""

    # ---- retries ---------------------------------------------------------
    def _clear_costmaps(self):
        # global only when clear_global_costmap_on_retry (review 2026-10-09): the global costmap
        # now holds the sensor obstacle marks the detour / replan must route around; wiping it
        # on every retry sent the robot straight back through marks it could no longer see.
        if self.p.clear_costmaps_on_retry:
            self._call(SRV_CLEAR_COSTMAPS, {'global': bool(self.p.clear_global_costmap_on_retry)},
                       track=False)

    def _retry_delay(self, n):
        """Back-off before retry ``n`` (1-based): transit_retry_backoff_s[n-1], the
        last entry repeating; transit_retry_delay_s when the list is empty."""
        return Budget.backoff(self.p.transit_retry_backoff_s, n, self.p.transit_retry_delay_s)

    def _schedule_retry(self, phase, sub, delay=None):
        """Blade off, clear the costmaps, wait ``delay`` (default transit_retry_delay_s),
        then re-send."""
        m = self.mission
        self._blk = None
        self._soft_blade_off('retry')
        self._clear_costmaps()
        m.step = 'retry_wait'
        m.retry_at = self._now + float(self.p.transit_retry_delay_s if delay is None else delay)
        self._log('warn', sub)
        self._go(phase, sub)

    def _retry_fire(self):
        m = self.mission
        m.step = None
        if m.transit_target is not None and self._transit_arrived() and \
                (m.recovering or m.transit_fails):
            self._log('info', 'retry: robot already within %.2f m of the transit target'
                      % self.p.transit_arrived_radius_m)
            m.recovering = False
            self._begin_blade()
            return
        self._dispatch(force_transit=m.recovering)
