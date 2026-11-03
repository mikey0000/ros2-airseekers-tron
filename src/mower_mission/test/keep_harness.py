# SPDX-License-Identifier: GPL-3.0-or-later
"""Harness for maneuvers the owner rule (2026-10-23: blade stays on through obstacle handling
inside the area) runs with the blade on: a no-go step / slope pre-drive (ACT_DRIVE) as well as
a back-up. test_mission_fsm.Harness allows only a mission back-up with the body inside; this
one also allows a drive, exactly where the FSM's own rules say the blade may stay on
(_blade_keep_ok, which lists the slope pre-drive's 'slope_pivot' step like blade_allowed)."""
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
                    assert self.fsm._blade_keep_ok(), 'blade on while starting %s' % e.name
                    self.fx.append(e)
                    continue
            batch.append(e)
        super()._apply(batch)
        return fx
