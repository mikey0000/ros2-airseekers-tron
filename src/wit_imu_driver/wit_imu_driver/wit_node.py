#!/usr/bin/env python3
"""WIT-Motion JY61P IMU driver (ROS 2 Humble).

Publishes
    /imu/data            sensor_msgs/Imu   (orientation, angular velocity, linear acceleration)
    /imu/temperature_c   std_msgs/Float32

Frame decoding lives in :mod:`wit_imu_driver.wit_protocol` (unit-tested, ROS-free).

Notes
- The JY61P is a 6-axis unit: its yaw is gyro-integrated, not earth-referenced, and
  drifts. ``navsat_transform_node`` therefore needs ``yaw_offset`` or a GPS heading;
  the EKF is configured to fuse yaw *rate* from this topic and absolute yaw only
  with a large covariance (``orientation_yaw_covariance`` parameter).
- One Imu message is published per received 0x53 angle frame (the last frame of each
  WIT output burst), so the ROS rate follows the sensor's configured rate exactly.
- ``publish_tf`` defaults to False: robot_state_publisher owns base_link->imu_link.
"""
import time

import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import tf2_ros

from wit_imu_driver.wit_protocol import (
    CMD_SAVE, CMD_UNLOCK, TYPE_ANGLE, WitParser, cmd_set_output, cmd_set_rate,
    euler_to_quaternion)


def _diag3(a, b, c):
    return [a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c]


class WitImuNode(Node):
    def __init__(self):
        super().__init__('wit_imu_driver')

        self.declare_parameter('port', '/dev/serial_imu')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate', 100.0)               # sensor output rate to request
        self.declare_parameter('configure_sensor', False)   # send FF AA rate/output commands at start
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('orientation_rp_covariance', 0.02)
        self.declare_parameter('orientation_yaw_covariance', 1.0)  # drifting gyro yaw: barely trusted
        self.declare_parameter('angular_velocity_covariance', 0.005)
        self.declare_parameter('linear_acceleration_covariance', 0.05)
        self.declare_parameter('stale_warn_s', 1.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.port, self.baud, self.frame_id = g('port'), int(g('baud')), g('frame_id')
        self.rate = float(g('rate'))
        self.configure_sensor = bool(g('configure_sensor'))
        self.publish_tf = bool(g('publish_tf'))
        self.stale_warn_s = float(g('stale_warn_s'))

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.imu_pub = self.create_publisher(Imu, g('imu_topic'), qos)
        self.temp_pub = self.create_publisher(Float32, '/imu/temperature_c', qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        rp, yaw = float(g('orientation_rp_covariance')), float(g('orientation_yaw_covariance'))
        self.orientation_cov = _diag3(rp, rp, yaw)
        av = float(g('angular_velocity_covariance'))
        self.angular_velocity_cov = _diag3(av, av, av)
        la = float(g('linear_acceleration_covariance'))
        self.linear_acceleration_cov = _diag3(la, la, la)

        self.parser = WitParser()
        self.ser = None
        self._last_open_attempt = 0.0
        self._last_frame_mono = 0.0
        self._published = 0
        self._last_stale_warn = 0.0

        self.create_timer(0.005, self._poll)          # 200 Hz serial poll
        self.create_timer(5.0, self._health)
        self.get_logger().info('wit_imu_driver: %s @ %d, publishing %s (frame %s)'
                               % (self.port, self.baud, g('imu_topic'), self.frame_id))

    # ------------------------------------------------------------------ serial
    def _open(self):
        now = time.monotonic()
        if now - self._last_open_attempt < 2.0:
            return
        self._last_open_attempt = now
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.get_logger().info('opened %s' % self.port)
            if self.configure_sensor:
                self._configure()
        except (serial.SerialException, OSError) as exc:
            self.ser = None
            self.get_logger().error('cannot open %s: %s' % (self.port, exc),
                                    throttle_duration_sec=10.0)

    def _configure(self):
        rate = int(round(self.rate))
        try:
            for cmd in (CMD_UNLOCK, cmd_set_rate(rate), cmd_set_output(), CMD_SAVE):
                self.ser.write(cmd)
                time.sleep(0.05)
            self.get_logger().info('sensor configured: %d Hz, acc+gyro+angle' % rate)
        except KeyError:
            self.get_logger().error('unsupported WIT rate %s; leaving sensor config alone' % rate)

    def _poll(self):
        if self.ser is None:
            self._open()
            return
        try:
            n = self.ser.in_waiting
            if n <= 0:
                return
            data = self.ser.read(n)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error('serial read failed: %s; reopening' % exc)
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return
        for ftype in self.parser.feed(data):
            self._last_frame_mono = time.monotonic()
            if ftype == TYPE_ANGLE and self.parser.sample.complete:
                self._publish()

    # ---------------------------------------------------------------- publish
    def _publish(self):
        s = self.parser.sample
        stamp = self.get_clock().now().to_msg()
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        if s.quat is not None:
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = s.quat
        else:
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = \
                euler_to_quaternion(*s.rpy)
        msg.orientation_covariance = self.orientation_cov
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = s.gyro
        msg.angular_velocity_covariance = self.angular_velocity_cov
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = s.acc
        msg.linear_acceleration_covariance = self.linear_acceleration_cov
        self.imu_pub.publish(msg)
        self._published += 1

        if s.temperature_c is not None:
            self.temp_pub.publish(Float32(data=float(s.temperature_c)))

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'base_link'
            t.child_frame_id = self.frame_id
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

    def _health(self):
        if self.ser is None:
            return
        age = time.monotonic() - self._last_frame_mono
        if self._last_frame_mono and age > self.stale_warn_s:
            self.get_logger().warn('no IMU frames for %.1f s (bad checksums so far: %d)'
                                   % (age, self.parser.bad_checksums))
        elif self.parser.bad_checksums:
            self.get_logger().debug('frames ok, %d bad checksums total' % self.parser.bad_checksums)


def main(args=None):
    rclpy.init(args=args)
    node = WitImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
