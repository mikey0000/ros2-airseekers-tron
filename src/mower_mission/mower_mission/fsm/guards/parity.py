# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: stock-parity guards (2026-11-03).

Small behaviours the stock Tron firmware has (docs/audit_2026-10-05/vendor_mission_layer.md
1.2 / 1.4, mower_docs/reference/IotNotice.msg):

* battery temperature: START refused outside the start band (start.py _preflight); above
  battery_temp_max_mow_c mid-mission the robot docks and resumes once cooled (stock
  ChargeEnvCheck "battery temperature high" -> auto-charge flow, pseudo-state
  ``battery_temp_highing``).

Every guard raises / clears a mower_alerts notice (:class:`Notice`).
"""

from mower_mission.fsm.common import MISSION_PHASES, Notice


class ParityGuardsMixin:
    """Stock-parity guards, called from ReactiveGuardsMixin._guards."""

    def _notice(self, incident, active, text='', kind='blocked', priority=3, message=''):
        """Emit a mower_alerts notice when its state changes (text changes re-send it)."""
        sent = self.__dict__.setdefault('_notices_sent', {})
        key = (bool(active), text if active else '')
        if sent.get(incident) == key:
            return
        sent[incident] = key
        self._fx.append(Notice(incident, bool(active), text, kind, int(priority),
                               message or incident))

    # ---- battery temperature --------------------------------------------
    def _battery_temp_tick(self):
        """True when it took over the tick (docking for a hot battery)."""
        p, i = self.p, self.inputs
        t = i.battery_temp_c
        if t is not None and self._battery_temp_refusal(p.battery_temp_max_start_c) == '':
            self._notice('battery_temp', False)
        hot = t is not None and t > float(p.battery_temp_max_mow_c)
        if not (hot and self.phase in MISSION_PHASES and self.mission is not None):
            self._batt_hot_since = None
            return False
        since = self.__dict__.get('_batt_hot_since')
        if since is None:
            self._batt_hot_since = since = self._now
        if self._now - since < float(p.battery_temp_debounce_s):
            return False
        self._batt_hot_since = None
        why = self._battery_temp_refusal(p.battery_temp_max_mow_c)
        self._log('warn', '%s: docking until it is <= %.0f C' % (why, p.battery_temp_max_start_c))
        self._notice('battery_temp', True, 'Mowing paused: %s.' % why, 'battery', 4,
                     'batteryTempHigh')
        self._interrupt_mission('battery temperature')
        self._dock('RETURNING_HOME', 'battery_temp', 'battery temperature %.0f C' % t)
        return True

    def _battery_temp_wait_tick(self):
        """Docked after a hot-battery pause: resume once inside the start band."""
        if self._battery_temp_refusal(self.p.battery_temp_max_start_c):
            return
        self._log('info', 'battery temperature %.0f C <= %.0f C: resuming mowing'
                  % (self.inputs.battery_temp_c or 0.0, self.p.battery_temp_max_start_c))
        self._resume('battery temperature')
