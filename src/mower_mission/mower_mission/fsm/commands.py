# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Operator / GUI commands.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (CMD_HOME, CMD_MANUAL_MOW, CMD_RECORD_AREA,
    CMD_RECORD_CANCEL, CMD_RECORD_FINISH, CMD_RECORD_OBSTACLE, CMD_RECORD_PATH,
    CMD_RESET_EMERGENCY, CMD_START, CMD_STOP, CMD_UNDOCK, DOCK_PHASES, PublishTrajectory,
    SRV_CLEAR_ESTOP, STATE_AUTONOMOUS, SUB_MANUAL_BLADE_OFF, SUB_MANUAL_BLADE_ON, ZeroBurst)  # noqa: F401


class CommandsMixin:
    """Operator / GUI commands."""

    # ==================================================================
    # commands
    # ==================================================================
    def _command(self, cmd):
        self._stuck_operator(cmd)
        if cmd == CMD_RESET_EMERGENCY:
            self._call(SRV_CLEAR_ESTOP, {}, track=False)
            self._log('info', 'reset emergency: /clear_estop requested')
            if self.phase == 'BOUNDARY_EMERGENCY_STOP':
                self._go_idle('boundary stop reset')
            return True
        if cmd == CMD_STOP:
            self._stop()
            return True
        if self._emergency or self.phase == 'EMERGENCY':
            self._log('warn', 'command %d refused: emergency active' % cmd)
            return False
        if self.inputs.emergency_stamp is None:
            self._log('warn', 'command %d refused: no /hardware_bridge/emergency message yet '
                              '(emergency feed not up)' % cmd)
            return False
        if self.phase == 'BOUNDARY_EMERGENCY_STOP':
            self._log('warn', 'command %d refused: boundary emergency stop latched '
                              '(send STOP or RESET_EMERGENCY)' % cmd)
            return False
        if cmd == CMD_START:
            return self._start(single=None)
        if cmd == CMD_HOME:
            return self._home()
        if cmd == CMD_RECORD_AREA:
            return self._record_start()
        if cmd == CMD_RECORD_PATH:
            return self._record_start(kind='path')
        if cmd == CMD_RECORD_OBSTACLE:
            return self._record_start(kind='obstacle')
        if cmd == CMD_RECORD_FINISH:
            return self._record_finish()
        if cmd == CMD_RECORD_CANCEL:
            if self.phase != 'RECORDING':
                self._log('warn', 'RECORD_CANCEL refused: not recording')
                return False
            self._track = []
            self._record_poly = None
            self._enum = None
            self._service = None
            self._log('info', 'recording cancelled, trajectory discarded')
            self._fx.append(PublishTrajectory([]))
            self._go_idle('recording cancelled')
            return True
        if cmd == CMD_MANUAL_MOW:
            return self._manual_mow()
        if cmd == CMD_UNDOCK:
            return self._manual_undock()
        self._log('warn', 'command %d not supported' % cmd)
        return False

    def _leave_current_mode(self, why):
        """Blade off and drop recording / holds before entering another mode."""
        if self.phase == 'MANUAL_MOWING' or self.blade_on:
            self._blade_off(why)
        if self.phase == 'RECORDING':
            self._track = []
            self._record_poly = None
            self._enum = None
            self._service = None
            self._fx.append(PublishTrajectory([]))
        self._resume_after = None

    def _stop(self):
        self._log('info', 'STOP: cancel actions, blade off, zero velocity')
        if self.mission is not None:
            self._interrupt_mission('stop')
        self._cancel_all('stop')
        self._blade_off('stop')
        self._fx.append(ZeroBurst())
        if self.phase == 'RECORDING':
            self._track = []
            self._record_poly = None
            self._fx.append(PublishTrajectory([]))
        self._resume_after = None
        self._dock_purpose = None
        self._manual_undocking = False
        if self.phase != 'EMERGENCY':
            self._go_idle('stopped')

    def _home(self):
        if self.phase in DOCK_PHASES:
            return True
        self._hb_count = 0
        self._hb_spots = []
        self._leave_current_mode('home')
        if self.mission is not None:
            self._interrupt_mission('home')
        self._dock('RETURNING_HOME', 'home')
        return True

    def _manual_mow(self):
        if self.state == STATE_AUTONOMOUS:
            self._log('warn', 'MANUAL_MOW refused: autonomous run active (STOP first)')
            return False
        if self.phase == 'RECORDING':
            self._leave_current_mode('manual mow')
        self._resume_after = None
        if self.p.manual_blade_requires_enable:
            # Two-step manual: drive first; the blade needs manual_blade(True).
            if self.blade_on:
                self._blade_off('manual mow (re-entered)')
            self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_OFF)
            return True
        # Upstream order: publish state 4 first, then enable the blade.
        self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_ON)
        self._blade_on()
        return True

    def _manual_blade_veto(self):
        """Why the blade may not run in MANUAL_MOWING right now ('' = allowed)."""
        cause = self._emergency_cause()
        if cause:
            return 'emergency: %s' % cause
        if self.inputs.is_charging:
            return 'charging'
        if self.inputs.docked:
            return 'docked'
        if self._tilt_band() in ('limit', 'critical'):
            return 'steep slope (%s)' % self._tilt_text()
        return ''

    def _manual_blade(self, enable):
        if self.phase != 'MANUAL_MOWING':
            return False, 'not in MANUAL_MOWING (%s)' % self.phase
        if not enable:
            if self.blade_on:
                self._blade_off('manual: blades stop requested')
            self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_OFF)
            return True, 'blade off'
        veto = self._manual_blade_veto()
        if veto:
            return False, 'refused: %s' % veto
        if not self.blade_on:
            self._blade_on()
        self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_ON)
        return True, 'blade on'

    def _manual_tick(self):
        """Two-step manual: a docked / charging robot never keeps the blade on
        (emergency, lift and stop button are handled by the guards)."""
        if self.p.manual_blade_requires_enable and self.blade_on:
            veto = self._manual_blade_veto()
            if veto:
                self._log('warn', 'manual: blade off (%s)' % veto)
                self._blade_off('manual: %s' % veto)
                self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_OFF)
