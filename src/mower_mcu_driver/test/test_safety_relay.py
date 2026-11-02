# SPDX-License-Identifier: GPL-3.0-or-later
"""safety_relay: the MCU-side /hardware_bridge/emergency source (review 2026-11-02 H1)."""

import os
import sys

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from mower_mcu_driver.safety_relay import SafetyRelay, emergency_fields  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make(period_s=0.1, stale_s=1.0):
    clk, sent = Clock(), []
    relay = SafetyRelay(lambda f: sent.append((clk.t, f)), period_s=period_s, stale_s=stale_s,
                        clock=clk)
    return relay, clk, sent


def drive(relay, clk, seconds, frames=True, dt=0.01):
    """100 Hz SensorInfo frames (optional) + the 10 Hz main-loop tick."""
    end = clk.t + seconds
    next_tick = clk.t
    while clk.t < end - 1e-9:
        if frames:
            relay.frame()
            relay.set_inputs(False, False, False)
        if clk.t >= next_tick - 1e-9:
            relay.tick()
            next_tick += 0.1
        clk.t = round(clk.t + dt, 6)


def test_fields_match_the_gui_bridge_semantics():
    assert emergency_fields(False, False, False) == (False, False, False, '')
    assert emergency_fields(True, True, False) == \
        (True, True, False, 'estop latched/interlock, stop button')
    assert emergency_fields(False, False, True) == (True, False, True, 'lift')
    assert emergency_fields(False, False, False, True) == \
        (True, False, False, 'mcu telemetry stale')


def test_cadence_is_10_hz_with_frames_and_without():
    relay, clk, sent = make()
    drive(relay, clk, 5.0)
    healthy = [t for t, f in sent if not f[0]]
    assert 48 <= len(healthy) <= 52                  # 10 Hz for 5 s
    gaps = [b - a for a, b in zip(healthy, healthy[1:])]
    assert max(gaps) <= 0.11
    # MCU link silent: the relay keeps publishing (from the main loop), now 'stale'
    n = len(sent)
    drive(relay, clk, 3.0, frames=False)
    tail = sent[n:]
    assert 28 <= len(tail) <= 32
    assert all(not f[0] for t, f in tail if t - tail[0][0] <= 1.0 - 0.02)
    stale = [f for t, f in tail if t - tail[0][0] > 1.05]
    assert stale and all(f == (True, False, False, 'mcu telemetry stale') for f in stale)


def test_never_seen_mcu_is_stale():
    relay, clk, sent = make()
    relay.tick()
    assert sent[-1][1] == (True, False, False, 'mcu telemetry stale')


def test_change_publishes_immediately():
    relay, clk, sent = make()
    relay.frame()
    relay.set_inputs(False, False, False)
    n = len(sent)
    relay.set_inputs(False, False, False)            # unchanged: rate limited to the tick
    assert len(sent) == n
    relay.set_inputs(False, False, True)             # lift: at once, no tick needed
    assert len(sent) == n + 1 and sent[-1][1] == (True, False, True, 'lift')
    relay.set_inputs(True, True, False)
    assert sent[-1][1] == (True, True, False, 'estop latched/interlock, stop button')
