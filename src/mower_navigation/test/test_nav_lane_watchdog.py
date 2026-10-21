#!/usr/bin/env python3
# Copyright 2026 ROS 2 port team
# SPDX-License-Identifier: Apache-2.0
"""LaneWatchdog (pure logic of scripts/nav_lane_watchdog.py); no ROS needed."""
import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "nav_lane_watchdog", os.path.join(HERE, "..", "scripts", "nav_lane_watchdog.py"))
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
LaneWatchdog = _mod.LaneWatchdog


def drive(wd, t0, t1, smoothed=True, nav=True, v=0.175, w=-0.08):
    t = t0
    while t <= t1 + 1e-9:
        wd.on_raw(v, w, t)
        if smoothed:
            wd.on_smoothed(t)
        if nav:
            wd.on_nav(t)
        t += 0.1
    return t


def test_healthy_lane_never_flags():
    wd = LaneWatchdog()
    drive(wd, 0.0, 30.0)
    assert wd.check(30.0) is None


def test_collision_monitor_silently_dropping_is_flagged():
    # the r5 hard leg 26 pattern: controller moving, smoother forwarding, /cmd_vel_nav silent
    wd = LaneWatchdog(stall_s=2.0)
    drive(wd, 0.0, 5.0)
    drive(wd, 5.1, 6.0, nav=False)
    assert wd.check(6.0) is None            # not yet 2 s
    drive(wd, 6.1, 7.3, nav=False)
    assert wd.check(7.3) == 'collision_monitor'


def test_smoother_silent_is_named():
    wd = LaneWatchdog(stall_s=2.0)
    drive(wd, 0.0, 3.0, smoothed=False, nav=False)
    assert wd.check(3.0) == 'velocity_smoother'


def test_a_real_collision_stop_is_left_alone():
    wd = LaneWatchdog(stall_s=2.0)
    wd.on_cm_state(1)                       # STOP: an obstacle in the stop zone
    drive(wd, 0.0, 5.0, nav=False)
    assert wd.check(5.0) is None
    wd.on_cm_state(0)
    assert wd.check(5.0) == 'collision_monitor'


def test_stopped_controller_is_not_a_stall():
    wd = LaneWatchdog(stall_s=2.0)
    drive(wd, 0.0, 5.0, nav=False, v=0.0, w=0.0)   # zero commands: nothing to forward
    assert wd.check(5.0) is None


def test_recoveries_are_bounded():
    wd = LaneWatchdog(max_recoveries=3, window_s=600.0, cooldown_s=10.0)
    assert wd.may_recover(0.0)
    assert not wd.may_recover(5.0)          # cooldown
    assert wd.may_recover(11.0) and wd.may_recover(22.0)
    assert not wd.may_recover(40.0)         # 3 in the window
    assert wd.may_recover(700.0)            # window passed


def test_node_starts_and_stays_up():
    """The ROS node must construct and spin (2026-10-21: assigning self.clients, a read-only
    rclpy Node property, crash-looped it every 4 s on the mower; the logic tests above missed it)."""
    import signal
    import subprocess
    import sys
    import time
    script = os.path.join(HERE, "..", "scripts", "nav_lane_watchdog.py")
    p = subprocess.Popen([sys.executable, script, "--ros-args", "-r", "__node:=wd_smoke"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        time.sleep(4.0)
        alive = p.poll() is None
        if not alive:
            raise AssertionError(p.stdout.read().decode(errors="replace")[-2000:])
    finally:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
