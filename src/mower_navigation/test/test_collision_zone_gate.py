#!/usr/bin/env python3
# Copyright 2026 ROS 2 port team
# SPDX-License-Identifier: Apache-2.0
"""ZoneGate (pure decision logic of scripts/collision_zone_gate.py); no ROS needed."""
import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "collision_zone_gate", os.path.join(HERE, "..", "scripts", "collision_zone_gate.py"))
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
ZoneGate = _mod.ZoneGate


def test_initial_state_is_on():
    assert ZoneGate().wanted is True      # safe default: zones active


def test_forward_on_reverse_and_pivot_off():
    g = ZoneGate()
    assert g.on_cmd(0.2, 0.0) is True
    assert g.on_cmd(-0.1, 0.1) is False   # reverse leg must not hit the front stop zone
    assert g.on_cmd(0.0, 0.2) is False    # pivot


def test_hysteresis_band():
    g = ZoneGate()                        # on 0.08, off 0.06
    g.on_cmd(-0.1, 0.0)
    assert g.on_cmd(0.07, 0.1) is False   # inside the band: stays off
    g.on_cmd(0.2, 0.2)
    assert g.on_cmd(0.07, 0.3) is True    # inside the band: stays on
    assert g.on_cmd(0.05, 0.4) is False


def test_idle_rearm():
    g = ZoneGate()
    assert g.on_cmd(-0.1, 0.0) is False
    assert g.on_tick(0.5) is False
    assert g.on_tick(1.0) is True         # no command for idle_rearm_s -> back on


def test_tick_without_command_stays_on():
    g = ZoneGate()
    assert g.on_tick(100.0) is True


def test_off_above_on_rejected():
    with pytest.raises(ValueError):
        ZoneGate(forward_on=0.05, forward_off=0.08)


def test_retry_backoff_after_refusal():
    # 2026-10-13: an unconfigured collision_monitor refused "not declared" at 5 Hz for hours
    b = _mod.RetryBackoff(5.0)
    assert b.ready(0.0)
    assert b.failed(0.0) is True          # first of a streak: log it
    assert not b.ready(4.9)
    assert b.ready(5.0)
    assert b.failed(5.0) is False         # streak continues: throttled
    assert not b.ready(9.0)
    assert b.succeeded() is True          # recovered: one INFO line
    assert b.ready(9.0)
    assert b.succeeded() is False


def test_tight_slow_arc_off_wide_fast_arc_on():
    """2026-10-21: slow arcs (v 0.03-0.09, |w| 0.1-0.3) kept the stop zone armed in grass."""
    g = ZoneGate(0.08, 0.06, 1.0, max_curvature_on=1.0, max_curvature_off=1.4)
    assert g.on_cmd(0.3, 0.0, 0.2) is True      # r 1.5 m: forward, armed
    assert g.on_cmd(0.09, 0.1, 0.2) is False    # curvature 2.2 > 1.4: off
    g2 = ZoneGate(0.08, 0.06, 1.0, max_curvature_on=1.0, max_curvature_off=1.4)
    assert g2.on_cmd(0.09, 0.0, 0.2) is False   # r 0.45 m: off
    assert g2.on_cmd(0.09, 0.1, 0.11) is False  # curvature 1.22: band, stays off
    assert g2.on_cmd(0.09, 0.2, 0.08) is True   # curvature 0.9: on
    assert g2.on_cmd(0.09, 0.3, -0.11) is True  # band: stays on
    assert g2.on_cmd(0.09, 0.4, -0.2) is False  # curvature 2.2: off


def test_behavior_holds_zones_off_then_idle_rearm():
    g = ZoneGate(0.08, 0.06, 1.0, behavior_hold_s=2.0)
    assert g.on_behavior(True, 0.0) is False
    assert g.on_tick(5.0) is False               # Spin running: no idle re-arm
    assert g.on_cmd(0.3, 5.1) is False           # nor a forward command
    assert g.on_behavior(False, 6.0) is False
    assert g.on_tick(7.9) is False               # inside behavior_hold_s
    assert g.on_cmd(0.3, 8.1) is True            # after the hold: normal gating
    g.on_cmd(0.0, 8.2, 0.3)
    assert g.on_tick(9.3) is True                # idle re-arm still works afterwards


def test_behavior_end_without_command_rearms_after_hold():
    g = ZoneGate(0.08, 0.06, 1.0, behavior_hold_s=2.0)
    g.on_behavior(True, 0.0)
    g.on_behavior(False, 1.0)
    assert g.on_tick(2.9) is False
    assert g.on_tick(3.1) is True
