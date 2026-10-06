"""``vio_odom_bridge``: ``/ov_msckf/odomimu`` -> ``/odometry/vio`` (odom frame, body twist).

Output is a nav_msgs/Odometry with header.frame_id ``odom`` and child_frame_id
``base_link`` whose TWIST is the VIO body velocity rotated from the VIO IMU frame into
base_link (``imu_to_base_rpy``; identity for the WIT IMU, which is mounted with base_link
axes), with a covariance inflated from OpenVINS' own (``cov_scale``, ``min_twist_var``).
The pose is zero with 1e6 variance: VIO position/yaw are in OpenVINS' drifting
``global`` frame and are NOT fed to the EKF. The lever arm (omega x r) is ignored:
the WIT IMU sits at base_link (URDF imu_xyz 0 0 0).

Health/gating is ``mower_localization/vio_gate`` downstream. This node only drops
messages that are older than ``max_age_s`` (OpenVINS stalls) and counts them.
Started only with ``vio:=true`` (launch/vio.launch.py); see docs/vio.md.
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from stereo_vio_bridge.odom_convert import (body_twist_covariance, rotate,
                                            unused_pose_covariance)


def _rpy_matrix(r: float, p: float, y: float):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


class VioOdomBridge(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("vio_odom_bridge", **kwargs)
        d = self.declare_parameter
        d("input_topic", "/ov_msckf/odomimu")
        d("output_topic", "/odometry/vio")
        d("odom_frame", "odom")
        d("base_frame", "base_link")
        d("imu_to_base_rpy", [0.0, 0.0, 0.0])
        d("cov_scale", 10.0)        # OpenVINS velocity covariance is optimistic
        d("min_twist_var", 0.0025)  # (0.05 m/s)^2 floor
        d("max_age_s", 0.5)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.odom_frame = str(g("odom_frame"))
        self.base_frame = str(g("base_frame"))
        self.r_base_imu = _rpy_matrix(*[float(v) for v in g("imu_to_base_rpy")])
        self.cov_scale = float(g("cov_scale"))
        self.min_twist_var = float(g("min_twist_var"))
        self.max_age_s = float(g("max_age_s"))
        self.pub = self.create_publisher(Odometry, str(g("output_topic")), 10)
        self.create_subscription(Odometry, str(g("input_topic")), self._on_odom, 10)
        self.forwarded = 0
        self.stale = 0
        self.get_logger().info("vio_odom_bridge: %s -> %s (%s, child %s)" % (
            g("input_topic"), g("output_topic"), self.odom_frame, self.base_frame))

    def _on_odom(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.max_age_s > 0.0 and now - stamp > self.max_age_s:
            self.stale += 1
            self.get_logger().warn("dropping VIO odometry %.2fs old" % (now - stamp),
                                   throttle_duration_sec=10.0)
            return
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        v = rotate(self.r_base_imu, (lin.x, lin.y, lin.z))
        w = rotate(self.r_base_imu, (ang.x, ang.y, ang.z))
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.orientation.w = 1.0
        out.pose.covariance = unused_pose_covariance()
        t = out.twist.twist
        t.linear.x, t.linear.y, t.linear.z = v
        t.angular.x, t.angular.y, t.angular.z = w
        out.twist.covariance = body_twist_covariance(
            list(msg.twist.covariance), self.cov_scale, self.min_twist_var)
        self.forwarded += 1
        self.pub.publish(out)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = VioOdomBridge()
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
