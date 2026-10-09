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
