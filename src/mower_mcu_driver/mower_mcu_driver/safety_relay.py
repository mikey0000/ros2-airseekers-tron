# SPDX-License-Identifier: GPL-3.0-or-later
"""MCU-side ``/hardware_bridge/emergency`` source (review 2026-11-02, H1).

The mission treats more than ``emergency_timeout_s`` (2 s) without an Emergency message as an
emergency. Until 2026-11-03 gui_bridge published the topic, so a crash of anything sharing
its process (coverage server, WebSocket relay) silenced it and latched EMERGENCY. The topic
now comes from the process that owns the MCU link (mcu_node; mower_sim's sim_node on the
bench), so its silence means what the rule assumes: the driver is gone or hung.

Pure Python (no ROS): the node passes a ``publish(fields)`` callback and calls

* :meth:`SafetyRelay.frame` on every SensorInfo frame (MCU telemetry received),
* :meth:`SafetyRelay.set_inputs` whenever the e-stop / stop button / lift state is evaluated
  (publishes at once when the folded fields change),
* :meth:`SafetyRelay.tick` from its main loop; publishes every ``period_s`` (10 Hz).

``fields`` is ``(active, latched, lift_warning, reason)``, the Emergency message fields with
the meaning gui_bridge used: ``active`` = e-stop OR stop button OR lift OR telemetry stale,
``latched`` = e-stop, ``lift_warning`` = lift.
"""

import time


def emergency_fields(estop, stop_triggered, lift_triggered, mcu_stale=False):
    """Fold the raw inputs into ``(active, latched, lift_warning, reason)``."""
    causes = []
    if estop:
        causes.append('estop latched/interlock')
    if stop_triggered:
        causes.append('stop button')
    if lift_triggered:
        causes.append('lift')
    if mcu_stale:
        causes.append('mcu telemetry stale')
    return bool(causes), bool(estop), bool(lift_triggered), ', '.join(causes)


class SafetyRelay:
    def __init__(self, publish, period_s=0.1, stale_s=1.0, clock=time.monotonic):
        self._publish = publish
        self._period_s = float(period_s)
        self._stale_s = float(stale_s)
        self._clock = clock
        self._inputs = (False, False, False)
        self._frame_t = None          # receipt time of the last SensorInfo frame
        self._sent_fields = None      # what the last message said
        self._sent_t = None

    def frame(self):
        self._frame_t = self._clock()

    def stale(self):
        return self._frame_t is None or self._clock() - self._frame_t > self._stale_s

    def fields(self):
        return emergency_fields(*self._inputs, mcu_stale=self.stale())

    def set_inputs(self, estop, stop_triggered, lift_triggered):
        self._inputs = (bool(estop), bool(stop_triggered), bool(lift_triggered))
        if self.fields() != self._sent_fields:
            self._send()

    def tick(self):
        fields = self.fields()
        if fields != self._sent_fields or self._sent_t is None or \
                self._clock() - self._sent_t >= self._period_s * 0.95:
            self._send(fields)

    def _send(self, fields=None):
        self._sent_fields = self.fields() if fields is None else fields
        self._sent_t = self._clock()
        self._publish(self._sent_fields)
