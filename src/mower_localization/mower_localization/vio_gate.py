"""``vio_gate``: forward VIO odometry to the EKF only when RTK is not fixed and VIO is healthy.

    /odometry/vio        nav_msgs/Odometry  (stereo_vio_bridge/vio_odom_bridge)  in
    /fix_status          std_msgs/String    (um960_gps_driver)                    in
    /odom                nav_msgs/Odometry  (mower_mcu_driver, wheel twist)       in
    /odometry/vio_gated  nav_msgs/Odometry  -> ekf_node odom2 (config/ekf_vio.yaml) out
    /vio_gate/state      std_msgs/String    1 Hz JSON summary for the GUI / logs   out

Decision logic lives in :mod:`mower_localization.vio_gate_logic` (tested on the host).
Started only with ``vio:=true`` (launch/nav2.launch.py); see docs/vio.md.
"""

from __future__ import annotations

import json
import time
from typing import List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from mower_localization.vio_gate_logic import VioGateLogic, VioGateParams

WARN_PERIOD_S = 10.0
# Odometry.twist.covariance is 6x6 row-major (vx, vy, vz, wx, wy, wz).
VX_VAR, VY_VAR = 0, 7


class VioGateNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("vio_gate", **kwargs)
        d = self.declare_parameter
        d("input_topic", "/odometry/vio")
        d("output_topic", "/odometry/vio_gated")
        d("fix_status_topic", "/fix_status")
        d("wheel_odom_topic", "/odom")
        defaults = VioGateParams()
        for name, value in vars(defaults).items():
            d(name, value)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        params = VioGateParams(**{
            name: type(value)(g(name)) for name, value in vars(defaults).items()})
        self.logic = VioGateLogic(params)

        self.pub = self.create_publisher(Odometry, str(g("output_topic")), 10)
        self.state_pub = self.create_publisher(String, "~/state", 1)
        self.create_subscription(Odometry, str(g("input_topic")), self._on_vio, 10)
        self.create_subscription(String, str(g("fix_status_topic")), self._on_status, 10)
        self.create_subscription(Odometry, str(g("wheel_odom_topic")), self._on_wheel, 10)
        self.create_timer(1.0, self._publish_state)

        self.received = 0
        self.passed = 0
        self.last_reason: Optional[str] = "no VIO yet"
        self.get_logger().info(
            "vio_gate: %s -> %s (RTK from %s, wheel check on %s)"
            % (g("input_topic"), g("output_topic"), g("fix_status_topic"),
               g("wheel_odom_topic")))

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _on_status(self, msg: String) -> None:
        self.logic.on_fix_status(msg.data, self._now())

    def _on_wheel(self, msg: Odometry) -> None:
        self.logic.on_wheel(msg.twist.twist.linear.x, self._now())

    def _on_vio(self, msg: Odometry) -> None:
        self.received += 1
        t = msg.twist
        reason = self.logic.on_vio(
            self._now(), t.twist.linear.x, t.twist.linear.y,
            t.covariance[VX_VAR], t.covariance[VY_VAR])
        if reason != self.last_reason:
            self.get_logger().info("vio_gate: %s" % (reason or "PASSING VIO to the EKF"))
        self.last_reason = reason
        if reason is None:
            self.passed += 1
            self.pub.publish(msg)

    def _publish_state(self) -> None:
        now = self._now()
        self.state_pub.publish(String(data=json.dumps({
            "passing": self.last_reason is None,
            "reason": self.last_reason,
            "rtk_fixed": self.logic.rtk_available(now),
            "healthy_streak": self.logic.healthy_streak,
            "diverged": self.logic.diverged,
            "received": self.received,
            "passed": self.passed,
        })))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = VioGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
