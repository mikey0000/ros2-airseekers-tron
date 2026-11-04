# SPDX-License-Identifier: GPL-3.0-or-later
"""``Hold``: the shared shape of the three waits in place (2026-10-23 code review 1.7).

The dynamic-obstacle stop (``_dyn``), the path-blocked wait (``_blk``) and the localization
hold (``_loc``) stop the robot and keep the interrupted goal's context (``ctx`` follow /
transit / dock, ``phase``, ``purpose``, ``sub``) to resume it unchanged. They share:

* **stop**: the running action cancelled (its reason) and a zero burst;
* **show the wait**: the sub-state counts the seconds, re-published only when it changes;
* **resume**: a dock goal is sent again, a transit re-sends its goal, a follow
  re-dispatches from its progress.

What differs stays in the holds, through explicit hooks: when each starts and clears, its
time limit and timeout reaction, the blade (soft vs hard off), and ``transit_phase`` (the
blocked wait resumes a transit in TRANSIT; the others in the phase they hold). Behaviour is
unchanged: every effect is emitted in the order the copies used.
"""

from mower_mission.fsm.common import CancelActions, ZeroBurst


class Hold:
    """Stateless helpers; the hold state lives in ``fsm._dyn`` / ``_blk`` / ``_loc``."""

    @staticmethod
    def stop(fsm, why):
        """Cancel the running action (``why``) and zero the wheels. A coverage pivot cut
        short (step corner_pivot: an in-place Spin, a K-turn about-turn, the pivot after a
        slope pre-drive) is no longer "done": ``_cov_pivot`` marks it done when it STARTS,
        so the resume (``_dispatch`` -> ``_cov_pivot``) would follow the leg from the half-
        turned heading (field 2026-11-04 23:42 / 23:53: loc hold mid-pivot, resume sent
        /follow_path with no spin, BOUNDARY OUT 8 s later). Cleared, the resume pivots the
        rest of the way from the pose then (nothing when within the pivot threshold)."""
        m = fsm.mission
        if m is not None and fsm.phase == 'MOWING' and m.step == 'corner_pivot':
            m.pivot_done, m.pivot_target = None, None
        if fsm._action is not None:
            fsm._fx.append(CancelActions(why))
            fsm._action = None
        fsm._fx.append(ZeroBurst())

    @staticmethod
    def show_wait(fsm, sub):
        if sub != fsm.sub_state:
            fsm._go(fsm.phase, sub)

    @staticmethod
    def resume(fsm, h, transit_phase=None):
        """Resume the held goal ``h`` ({'ctx', 'phase', 'purpose', 'sub'})."""
        m = fsm.mission
        if h['ctx'] == 'dock':
            fsm._dock(h['phase'], h['purpose'], sub=h['sub'])
        elif m is None:
            return
        elif h['ctx'] == 'transit':
            m.step = 'transit'
            fsm._go(transit_phase or fsm.phase, h['sub'])
            fsm._send_transit()
        else:
            m.step = None
            fsm._dispatch()
