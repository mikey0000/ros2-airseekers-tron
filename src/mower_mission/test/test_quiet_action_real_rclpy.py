# SPDX-License-Identifier: GPL-3.0-or-later
"""QuietActionClient: results still arrive, unused feedback is dropped, a registered
feedback callback still fires. Needs real rclpy (skipped elsewhere); isolated domain."""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs.action')

from nav2_msgs.action import BackUp  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

DOMAIN = 88


def test_quiet_client_results_and_feedback():
    from mower_mission.quiet_action import QuietActionClient
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(domain_id=DOMAIN)
    srv_node = rclpy.create_node('quiet_srv')
    cli_node = rclpy.create_node('quiet_cli')

    def execute(gh):
        for _ in range(50):
            gh.publish_feedback(BackUp.Feedback())
            time.sleep(0.002)
        gh.succeed()
        return BackUp.Result()

    ActionServer(srv_node, BackUp, '/quiet_test', execute_callback=execute)
    cli = QuietActionClient(cli_node, BackUp, '/quiet_test')
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(srv_node)
    ex.add_node(cli_node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        assert cli.wait_for_server(timeout_sec=5.0)
        for with_cb in (False, True):
            seen = []
            fut = cli.send_goal_async(BackUp.Goal(),
                                      feedback_callback=seen.append if with_cb else None)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 5.0:
                time.sleep(0.01)
            gh = fut.result()
            assert gh.accepted
            rf = gh.get_result_async()
            while not rf.done() and time.monotonic() - t0 < 10.0:
                time.sleep(0.01)
            assert rf.done() and rf.result().status == 4    # SUCCEEDED
            assert bool(seen) == with_cb
    finally:
        ex.shutdown()
        srv_node.destroy_node()
        cli_node.destroy_node()
        rclpy.try_shutdown()
