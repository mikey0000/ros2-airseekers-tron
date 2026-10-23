# SPDX-License-Identifier: GPL-3.0-or-later
"""``Escape``: the shared shape of the three recovery manoeuvres (2026-10-23 code review 1.7).

The stuck escape (``_stk``), the steep-slope back-out (``_tob``) and the hard-boundary retrace
(``_hb``) each drive the robot under the mission's control while the stuck guard is
suppressed. They share:

* **begin**: the state dict is set, every action cancelled, the blade off, a zero burst, the
  slip_detector's /stuck suppressed, optionally a terrain incident;
* **pause the mission**: a running follow keeps its progress as the resume point, the step is
  cleared;
* **watch**: an emergency or a phase change aborts the manoeuvre (zero, guard re-armed);
* **stop**: the state cleared, optionally a zero command, the guard re-armed;
* **resume** after success: back to the dock phase, or idle when nothing is left to resume
  (the guard-specific continuation is the caller's);
* **needs help**: the latched STUCK_NEEDS_HELP with its operator clear.

What differs stays in the guards, through explicit hooks: the attribute that holds the state,
the label used for cancels / blade / interrupts, the abort log text, the STUCK_NEEDS_HELP
sub-state, and a ``before_interrupt`` hook (the stuck guard skips the failing piece first).
The manoeuvres themselves (escape steps, track back-out, retrace / inward / back move) are
not shared and stay in their guards. Behaviour is unchanged: every effect is emitted in the
order the three copies used.
"""

from mower_mission.fsm.common import (DOCK_PHASES, DriveCmd, RecordIncident,
                                      SuppressStuckGuard, ZeroBurst)


class Escape:
    """One recovery manoeuvre kind (stateless; the state lives in ``fsm.<attr>``)."""

    def __init__(self, attr, label, abort_text, help_sub, before_interrupt=None,
                 begin_label=None):
        self.attr = attr                    # MissionFSM attribute holding the state dict
        self.label = label                  # cancel / blade-off / interrupt reason (give-up)
        self.begin_label = begin_label or label   # cancel / blade-off reason at the start
        self.abort_text = abort_text        # 'stuck escape', 'steep-slope back-out', ...
        self.help_sub = help_sub            # STUCK_NEEDS_HELP sub-state, '%.2f, %.2f' slots
        self.before_interrupt = before_interrupt   # fsm -> None, before the mission interrupt

    def state(self, fsm):
        return getattr(fsm, self.attr)

    def begin(self, fsm, state, incident=None):
        """Take over: state set, actions cancelled, blade off, zero, /stuck suppressed;
        ``incident`` = (kind, x, y, detail) for a terrain incident."""
        setattr(fsm, self.attr, state)
        fsm._cancel_all(self.begin_label)
        fsm._blade_off(self.begin_label)
        fsm._fx.append(ZeroBurst())
        fsm._fx.append(SuppressStuckGuard(True))
        if incident is not None:
            fsm._fx.append(RecordIncident(*incident))

    @staticmethod
    def pause_mission(fsm, follow):
        """A running follow resumes from its progress; the mission step is cleared."""
        m = fsm.mission
        if m is not None:
            if follow:
                m.start_local = fsm._track_progress()
            m.step = None

    def watch(self, fsm):
        """Abort when something else took over (emergency, STOP, phase change)."""
        k = self.state(fsm)
        if k is None:
            return
        if fsm._emergency_cause() or fsm.phase != k['phase']:
            fsm._log('warn', '%s aborted (%s)' % (
                self.abort_text, fsm._emergency_cause() or 'state %s' % fsm.phase))
            setattr(fsm, self.attr, None)
            fsm._fx.append(SuppressStuckGuard(False))
            fsm._fx.append(DriveCmd(0.0, 0.0))

    def stop(self, fsm, zero=True):
        """End the manoeuvre: state cleared, (zero command,) /stuck re-armed."""
        setattr(fsm, self.attr, None)
        if zero:
            fsm._fx.append(DriveCmd(0.0, 0.0))
        fsm._fx.append(SuppressStuckGuard(False))

    @staticmethod
    def resume_dock_or_idle(fsm, k, dock_sub, idle_why):
        """Common resume head: a dock phase docks again, no mission left goes idle.
        True = handled; False = the guard continues the mission itself."""
        if k['phase'] in DOCK_PHASES:
            fsm._dock(k['phase'], k['dock_purpose'], dock_sub)
            return True
        m = fsm.mission
        if m is None or m.area_idx is None or not m.subpaths:
            fsm._go_idle(idle_why)
            return True
        return False

    def needs_help(self, fsm, why, pose):
        """Latched STUCK_NEEDS_HELP (STOP / START / MANUAL clear it)."""
        x, y = float(pose[0]), float(pose[1])
        fsm._log('error', 'STUCK_NEEDS_HELP: %s at (%.2f, %.2f): motion stopped, blade off '
                 '(STOP / START / MANUAL to clear)' % (why, x, y))
        if fsm.mission is not None:
            if self.before_interrupt is not None:
                self.before_interrupt(fsm)
            fsm._interrupt_mission(self.label)
        fsm._cancel_all(self.label)
        fsm._blade_off(self.label)
        fsm._fx.append(ZeroBurst())
        fsm._dock_purpose = None
        fsm._resume_after = None
        setattr(fsm, self.attr, None)
        fsm._go('STUCK_NEEDS_HELP', self.help_sub % (x, y))
