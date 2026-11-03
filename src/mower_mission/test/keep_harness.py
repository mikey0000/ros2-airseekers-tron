# SPDX-License-Identifier: GPL-3.0-or-later
"""Harness for maneuvers the shipped blade_keep_on_obstacles rule (owner 2026-10-23) runs with
the blade on: a pivot back-up / no-go step / K-turn back-up inside the area by the body rules.
test_mission_fsm.Harness asserts no back-up / drive starts with the blade on; this one allows
it exactly where the FSM's own rules say the blade may stay on (_blade_keep_ok; the slope
pre-drive's 'slope_pivot' step, which blade_allowed lists)."""
from mower_mission import mission_fsm as f

from test_mission_fsm import Harness

KEEP_ACTIONS = (f.ACT_BACKUP, f.ACT_DRIVE)


class KeepHarness(Harness):
    def _apply(self, fx):
        batch = []
        for e in fx:
            if isinstance(e, f.StartAction) and e.name in KEEP_ACTIONS:
                super()._apply(batch)              # the blade state before this start
                batch = []
                if self.blade:
                    m = self.fsm.mission
                    assert self.fsm._blade_keep_ok() or (
                        m is not None and m.step == 'slope_pivot'
                        and f.blade_allowed(self.fsm)), 'blade on while starting %s' % e.name
                    self.fx.append(e)
                    continue
            batch.append(e)
        super()._apply(batch)
        return fx
