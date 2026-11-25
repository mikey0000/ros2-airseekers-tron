# SPDX-License-Identifier: Apache-2.0
"""Toggling the docking vision subscriptions off from a worker thread must not kill the node.

Field 2026-11-25: every failed dock ended with ``_enable_vision(False)`` on the action thread
while the 6-thread executor built its wait set, and rclpy raised InvalidHandle from a
subscription's default QoS event handler, which took the node down. Needs a real ROS 2
Jazzy environment; skipped elsewhere. Isolated ROS domain.
"""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mower_interfaces.action')

from rclpy.executors import MultiThreadedExecutor  # noqa: E402

DOMAIN = 88


def test_vision_subscriptions_have_no_event_handlers_and_survive_toggling():
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(domain_id=DOMAIN)
    from mower_docking.docking_node import DockingServer
    node = DockingServer()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    errors = []

    def spin():
        try:
            while rclpy.ok() and not stop.is_set():
                ex.spin_once(timeout_sec=0.001)
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)
    stop = threading.Event()
    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    try:
        assert node._enable_vision(True)
        assert node._image_sub.event_handlers == [] and node._info_sub.event_handlers == []
        for _ in range(300):
            node._enable_vision(False)
            node._enable_vision(True)
        time.sleep(0.2)
        assert spinner.is_alive() and not errors, errors
    finally:
        stop.set()
        spinner.join(2.0)
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
