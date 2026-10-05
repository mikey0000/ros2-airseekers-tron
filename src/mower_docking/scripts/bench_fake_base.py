#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Bench fake for mower_docking (NO hardware): integrates /cmd_vel_docking into
/odom, publishes /mower_base/status (20 Hz) with is_docking_done true while the
robot is within 1 cm of the dock at x = 0 (a hard stop), serves /charging and
latches /map_server_node/docking_pose at the origin.  Starting docked and
undocking 0.8 m means the contact flips after 0.8 m of reverse on the way back.

  python3 bench_fake_base.py --start-docked
  python3 bench_fake_base.py --with-nav --start-x 2.0 --start-y 0.6 --start-yaw 3.0
      (--with-nav also follows /cmd_vel_nav while the docking lane is idle, like
      twist_mux, and publishes /odometry/filtered plus TF map->odom->base_link
      so Nav2 can run against the fake)
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mower_interfaces.msg import MowerBaseDevStatus
from mower_interfaces.srv import ChargingControl
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped


class Fake(Node):
    def __init__(self, a):
        super().__init__('bench_fake_base')
        self.a = a
        self.x = 0.0 if a.start_docked else a.start_x
        self.y = 0.0 if a.start_docked else a.start_y
        self.yaw = 0.0 if a.start_docked else a.start_yaw
        self.v = self.w = 0.0
        self.last_cmd = self.get_clock().now()
        self.nav_v = self.nav_w = 0.0
        self.last_nav = self.get_clock().now()
        if a.with_nav:
            from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
            self.tf = TransformBroadcaster(self)
            self.stf = StaticTransformBroadcaster(self)
            st = TransformStamped()
            st.header.frame_id, st.child_frame_id = 'map', 'odom'
            st.transform.rotation.w = 1.0
            self.stf.sendTransform(st)
            self.filt_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
            self.create_subscription(Twist, '/cmd_vel_nav', self.on_nav, 10)
        self.charging = a.start_docked
        self.create_subscription(Twist, '/cmd_vel_docking', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.st_pub = self.create_publisher(MowerBaseDevStatus, '/mower_base/status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.dock_pub = self.create_publisher(PoseStamped, '/map_server_node/docking_pose', latched)
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.orientation.w = 1.0
        self.dock_pub.publish(ps)
        self.create_service(ChargingControl, '/charging', self.on_charging)
        self.dt = 0.05
        self.create_timer(self.dt, self.tick)

    def on_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z
        self.last_cmd = self.get_clock().now()

    def on_nav(self, m):
        self.nav_v, self.nav_w = m.linear.x, m.angular.z
        self.last_nav = self.get_clock().now()

    def on_charging(self, req, resp):
        self.charging = req.enable_charging
        self.get_logger().info('/charging enable_charging=%s' % req.enable_charging)
        return resp

    def tick(self):
        now = self.get_clock().now()
        if (now - self.last_cmd).nanoseconds > 5e8:  # twist_mux timeout
            self.v = self.w = 0.0
            if self.a.with_nav and (now - self.last_nav).nanoseconds < 6e8:
                self.v, self.w = self.nav_v, self.nav_w  # lower-priority nav lane
        # the dock is at x=0 and is a hard stop
        nx = self.x + self.v * math.cos(self.yaw) * self.dt
        self.x = max(nx, 0.0 if self.v < 0 and self.x >= 0 else nx)
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.w * self.dt
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)
        if self.a.with_nav:
            self.filt_pub.publish(o)
            t = TransformStamped()
            t.header.stamp = o.header.stamp
            t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation = o.pose.pose.orientation
            self.tf.sendTransform(t)
        s = MowerBaseDevStatus()
        s.header.stamp = o.header.stamp
        s.is_docking_done = self.x <= 0.01
        s.is_charging = s.is_docking_done and self.charging
        s.is_moving = abs(self.v) > 1e-3
        self.st_pub.publish(s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start-docked', action='store_true')
    ap.add_argument('--start-x', type=float, default=0.8)
    ap.add_argument('--start-y', type=float, default=0.0)
    ap.add_argument('--start-yaw', type=float, default=0.0)
    ap.add_argument('--with-nav', action='store_true')
    a, rest = ap.parse_known_args()
    rclpy.init(args=rest)
    rclpy.spin(Fake(a))


if __name__ == '__main__':
    main()
