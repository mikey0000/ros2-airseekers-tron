"""Keep controller_server's controller_frequency at base x rate_factor (sim faster than real time).

Nav2 Jazzy (1.3.13) controller_server runs ``WallRate(controller_frequency)``. At rate_factor k
mower_sim sets controller_frequency = base x k through the server's dynamic parameter, but the
server only holds that callback while ACTIVE: ``on_deactivate`` removes it and
``on_activate`` does not re-read the parameter. activity_gated_lifecycle_manager pauses nav2
right after start-up (docked, idle) and resumes it on START, so a set that lands after the
pause reports success, the parameter reads back base x k, and the loop still runs at the base
rate (7 Hz sim at 3x: the 2026-11-01..03 INVALID gates). So the set is re-sent on EVERY
transition into active, and retried while it fails (the server refuses parameter changes
while a goal is running).

Pure logic, no rclpy: ``SimNode`` feeds it lifecycle states (``/controller_server/
transition_event`` and a GetState poll) and set results.
"""
from __future__ import annotations

from typing import Callable

STATE_ACTIVE = 3      # lifecycle_msgs/msg/State.PRIMARY_STATE_ACTIVE


class CtrlRateKeeper:
    """Decides when to send ``controller_frequency``; ``send(hz)`` issues the request and
    ``result(ok)`` reports its outcome."""

    def __init__(self, send: Callable[[float], None], hz: float) -> None:
        self._send = send
        self._hz = hz
        self._active: bool | None = None    # None: no state seen yet
        self._pending = False               # active and not yet set since the activation
        self._in_flight = False
        self._activations = 0

    @property
    def known(self) -> bool:
        return self._active is not None

    @property
    def activations(self) -> int:
        return self._activations

    def observe(self, state_id: int) -> None:
        """A lifecycle state of the controller_server (primary or transitional id)."""
        active = state_id == STATE_ACTIVE
        if active and self._active is not True:
            self._activations += 1
            self._pending = True
            self._active = True
            self._try_send()
            return
        self._active = active
        if not active:
            self._pending = False

    def result(self, ok: bool) -> None:
        """Outcome of the last ``send``."""
        self._in_flight = False
        if ok:
            self._pending = False

    def tick(self) -> None:
        """Periodic retry of a failed set while the server stays active."""
        self._try_send()

    def _try_send(self) -> None:
        if self._active and self._pending and not self._in_flight:
            self._in_flight = True
            self._send(self._hz)
