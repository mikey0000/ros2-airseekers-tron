# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionNode Nav2-readiness gate: a nav2-backed goal waits for the lifecycle stack to
report active before it is sent (the activity gate PAUSEs it while the robot is idle).
Needs real rclpy (skipped elsewhere); isolated domain."""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs.action')
pytest.importorskip('std_srvs.srv')
pytest.importorskip('mower_interfaces.msg')
pytest.importorskip('mowgli_interfaces.action')

from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

DOMAIN = 89
SERVICE = '/lifecycle_manager_navigation/is_active'


class _Stub:
    """Just enough of MissionNode for _await_nav2_ready: client, params, lock, cancels."""

    def __init__(self, cli, node, timeout_s):
        self._nav2_active_cli = cli
        self._p = {'nav2_ready_timeout_s': timeout_s}
        self._lock = threading.RLock()
        self._cancelled = set()
        self._node = node

    def get_logger(self):
        return self._node.get_logger()


def test_nav2_ready_waits_for_active():
    from mower_mission.mission_node import MissionNode
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(domain_id=DOMAIN)
    srv = rclpy.create_node('nav2_ready_srv')
    cli_node = rclpy.create_node('nav2_ready_cli')
    t0 = time.monotonic()

    def is_active(_req, resp):
        resp.success = (time.monotonic() - t0) >= 0.6    # paused, then resumes
        return resp
    srv.create_service(Trigger, SERVICE, is_active)

    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(srv)
    ex.add_node(cli_node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        cli = cli_node.create_client(Trigger, SERVICE)
        assert cli.wait_for_service(timeout_sec=5.0)
        stub = _Stub(cli, cli_node, 5.0)

        start = time.monotonic()
        MissionNode._await_nav2_ready(stub, 1)           # blocked until ~0.6 s
        assert time.monotonic() - start >= 0.5

        t0 = time.monotonic() - 10.0                     # now always active
        start = time.monotonic()
        MissionNode._await_nav2_ready(stub, 2)
        assert time.monotonic() - start < 0.3

        stub._cancelled.add(3)                           # a cancelled goal must not wait
        start = time.monotonic()
        MissionNode._await_nav2_ready(stub, 3)
        assert time.monotonic() - start < 0.3
    finally:
        ex.shutdown()
        srv.destroy_node()
        cli_node.destroy_node()
        rclpy.try_shutdown()
