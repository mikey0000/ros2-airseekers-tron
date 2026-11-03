# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: stock-parity guards (2026-11-03).

Small behaviours the stock Tron firmware has (docs/audit_2026-10-05/vendor_mission_layer.md
1.2 / 1.4, mower_docs/reference/IotNotice.msg):

* critical battery: below battery_critical_percent off the dock -> stop, blade off, alert,
  then after battery_critical_grace_s the host power-off via base_keys (stock ShutdownSoon
  900023 + PowerManager power-off; the lithium pack must not be run flat);
* camera blocked: while mower_vision flags the front camera blocked, both controllers are
  capped at camera_blocked_speed_mps (the tilt caution cap path);
* battery temperature: START refused outside the start band (start.py _preflight); above
  battery_temp_max_mow_c mid-mission the robot docks and resumes once cooled (stock
  ChargeEnvCheck "battery temperature high" -> auto-charge flow, pseudo-state
  ``battery_temp_highing``).

Every guard raises / clears a mower_alerts notice (:class:`Notice`).
"""

from mower_mission.fsm.common import (AREA_SETTINGS_DEFAULTS, IDLE_NAMES,
    PARAM_NODE_CONTROLLER, SRV_SET_PARAMS, MISSION_PHASES, Notice, SRV_POWEROFF,
    ZeroBurst)


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

    # ---- camera blocked ---------------------------------------------------
    def _camera_blocked_tick(self):
        p = self.p
        want = (bool(self.inputs.camera_blocked) and float(p.camera_blocked_speed_mps) > 0.0
                and self.phase in MISSION_PHASES and self.mission is not None)
        if want == self._camera_capped:
            return
        self._camera_capped = want
        if want:
            self._log('warn', 'front camera blocked: controller speed capped at %.2f m/s'
                      % self._speed_cap())
        else:
            self._log('info', 'front camera ok: controller speed %s'
                      % ('restored' if self._speed_cap() is None else
                         'capped at %.2f m/s (other caps kept)' % self._speed_cap()))
        self._push_speed_caps()

    # ---- critical battery -------------------------------------------------
    def _battery_critical_tick(self):
        """True on the tick it stopped the robot."""
        p, i = self.p, self.inputs
        b, limit = i.battery_percent, float(p.battery_critical_percent)
        low = (limit > 0.0 and b is not None and 0.0 < b < limit and not self._at_dock())
        since = self.__dict__.get('_crit_since')
        if not low:
            if since is not None:
                self._log('info', 'battery critical cleared (%s): no power-off'
                          % ('on the dock' if self._at_dock() else '%.0f%%' % (b or 0.0)))
                self._notice('battery_critical', False)
            self._crit_since = None
            self._crit_requested = False
            return False
        if since is None:
            self._crit_since = self._now
            self._crit_requested = False
            off = ('powering off in %.0f s unless it is put on the dock'
                   % p.battery_critical_grace_s if p.battery_critical_poweroff else 'no power-off')
            self._log('error', 'battery critical %.0f%% < %.0f%% off the dock: stopping, %s'
                      % (b, limit, off))
            self._notice('battery_critical', True, 'Battery critical (%.0f %%): robot stopped, '
                         '%s.' % (b, off), 'battery', 5, 'batteryCriticalStop')
            if self.mission is not None:
                self._interrupt_mission('battery critical')
            if self.phase not in IDLE_NAMES:
                self._cancel_all('battery critical')
                self._blade_off('battery critical')
                self._fx.append(ZeroBurst())
                self._dock_purpose = None
                self._go_idle('battery critical %.0f%%: stopped' % b)
                return True
            return False
        if p.battery_critical_poweroff and not self.__dict__.get('_crit_requested') \
                and self._now - since >= float(p.battery_critical_grace_s):
            self._crit_requested = True
            self._log('error', 'battery critical for %.0f s off the dock: requesting host '
                      'power-off' % (self._now - since))
            self._call(SRV_POWEROFF, {'reason': 'battery critical %.0f%%' % b}, track=False)
        return False

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
