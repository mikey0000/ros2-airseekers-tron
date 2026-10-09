"""ROS-free power-off sequencing for the power-key long press (``power_long_action:=sequence``).

The 3 s hold itself is detected by the vendor kernel module (``init_test.ko``: 40 ms GPIO
poll, KEY_L after 75 ticks while still held; see docs/buttons.md), so this module only decides
what the stack does once KEY_POWER_LONG arrives:

    on_power_long  -> estop, hlc STOP, cutter_off, light POWER_OFF      (immediately)
    tick           -> after settle_s: light POWER_OFF again, write_request  (host powers off)

Guards: ignored before ``min_uptime_s`` of host uptime (a power-on press still held when the
module loads at ~16 s would otherwise read as a long press), and while a sequence is already
running. If the host helper is not installed (request directory missing) only the safety stop
runs and the sequence returns to idle so a later press can retry.

The vendor did the same steps in ``TaskManager::sensorStateCallback`` (stopTask, notice 900005,
sleep 3000 ms, ``/poweroff`` -> MCU, ``shutdown -h now``), except that it cut MCU power before
the OS had halted; we leave the MCU cut to the host's system-shutdown hook.
"""
from dataclasses import dataclass

IDLE, SETTLING, REQUESTED = 'idle', 'settling', 'requested'


@dataclass(frozen=True)
class Step:
    kind: str       # 'estop' | 'hlc_stop' | 'cutter_off' | 'light_poweroff' | 'write_request' | 'log' | 'error'
    note: str = ''


class PowerOffSequencer:
    def __init__(self, settle_s=3.0, min_uptime_s=30.0):
        self.settle_s = float(settle_s)
        self.min_uptime_s = float(min_uptime_s)
        self.state = IDLE
        self._t0 = None

    def on_power_long(self, now, uptime_s, helper_installed):
        """KEY_POWER_LONG received. ``now`` is a monotonic time, ``uptime_s`` host uptime."""
        if self.state != IDLE:
            return [Step('log', 'power long ignored: power-off already %s' % self.state)]
        if uptime_s < self.min_uptime_s:
            return [Step('log', 'power long ignored: %.1f s after boot (< %.0f s)'
                         % (uptime_s, self.min_uptime_s))]
        safety = [Step('estop', 'power off: latch e-stop (motion + blade off)'),
                  Step('hlc_stop', 'power off: mission STOP'),
                  Step('cutter_off', 'power off: cutter off')]
        if not helper_installed:
            return safety + [Step('error', 'power off: host helper not installed (no request '
                                  'directory); mower stopped but NOT powered off. Install with '
                                  'scripts/install_power_button.sh; short press clears the e-stop')]
        self.state = SETTLING
        self._t0 = float(now)
        return safety + [Step('light_poweroff', 'power off: PowerOff light')]

    def tick(self, now):
        if self.state == SETTLING and float(now) - self._t0 >= self.settle_s:
            self.state = REQUESTED
            # the e-stop flips the auto light to WarnSensorTrigged; re-assert PowerOff
            return [Step('light_poweroff', 'power off: PowerOff light'),
                    Step('write_request', 'power off: asking the host to shut down')]
        return []

    @property
    def busy(self):
        return self.state == SETTLING

    def request_failed(self):
        """write_request raised: go back to idle so the user can retry."""
        self.state = IDLE
        self._t0 = None
