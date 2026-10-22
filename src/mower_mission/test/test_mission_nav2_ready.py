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
from mower_common.sub_pump import WALL_CLOCK


class _Stub:
    """Just enough of MissionNode for _await_nav2_ready: client, params, lock, cancels."""

    def __init__(self, cli, node, timeout_s):
        self._nav2_active_cli = cli
        self._p = {'nav2_ready_timeout_s': timeout_s}
        self._lock = threading.RLock()
        self._cancelled = set()
        self._node = node
        self._clk = WALL_CLOCK

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
        assert MissionNode._await_nav2_ready(stub, 2) == ''
        assert time.monotonic() - start < 0.3

        stub._cancelled.add(3)                           # a cancelled goal must not wait
        start = time.monotonic()
        assert MissionNode._await_nav2_ready(stub, 3) == ''
        assert time.monotonic() - start < 0.3

        # still not active after the whole budget: the reason is returned, not swallowed
        t0 = time.monotonic() + 1e6                      # never active
        stub._p['nav2_ready_timeout_s'] = 1.0
        start = time.monotonic()
        assert MissionNode._await_nav2_ready(stub, 4) == 'Nav2 stack not active after 1s'
        assert 0.9 <= time.monotonic() - start < 4.0
    finally:
        ex.shutdown()
        srv.destroy_node()
        cli_node.destroy_node()
        rclpy.try_shutdown()


def test_nav2_down_explains_only_unaccepted_goals_and_docking():
    from mower_mission import mission_fsm as f
    from mower_mission.mission_node import _nav2_down_explains
    for name in (f.ACT_FOLLOW, f.ACT_NAV, f.ACT_BACKUP, f.ACT_DOCK):
        assert _nav2_down_explains(name, f.REJECTED)
        assert _nav2_down_explains(name, f.UNAVAILABLE)
        assert not _nav2_down_explains(name, f.SUCCEEDED)
        assert not _nav2_down_explains(name, f.CANCELED)
    # an accepted Nav2 goal means the stack came back: a later abort is not "Nav2 down"
    assert not _nav2_down_explains(f.ACT_FOLLOW, f.ABORTED)
    assert not _nav2_down_explains(f.ACT_NAV, f.ABORTED)
    # the docking server accepts, then navigates with Nav2
    assert _nav2_down_explains(f.ACT_DOCK, f.ABORTED)
