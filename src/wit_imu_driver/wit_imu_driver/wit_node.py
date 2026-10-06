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
- Yaw continuity: this driver publishes the sensor's own yaw untouched (no unwrap, no
  offset, no zeroing) and by default sends NOTHING to the sensor (``configure_sensor``
  false; even then only unlock/rate/output/save, never the 0x01 calibration register that
  zeroes the Z angle). The JY61P yaw therefore survives driver/container restarts and only
  restarts on a sensor power cycle (measured 2026-10-07: 164.191 deg before a container
  restart, 164.185 deg after). The first yaw after every open is logged for that check.

CPU (RK3588): serial bytes are read by a plain thread blocked in ``select`` on the tty fd
(no 200 Hz rclpy poll timer: each executor wake costs 1.5-4 ms of Python here), and
``/imu/data`` + ``/imu/temperature_c`` are published as pre-serialized CDR bytes
(:mod:`wit_imu_driver.imu_cdr`, byte-identical to rclpy serialization, unit tested;
``fast_publish:=false`` restores the typed path). The executor only serves the 5 s health
timer and the parameter services.
"""
import math
import os
import select
import threading
import time

import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import tf2_ros

from wit_imu_driver.imu_cdr import ImuSerializer, float32_bytes
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
        self.declare_parameter('fast_publish', True)        # pre-serialized CDR publish
        # after the first byte of a burst, wait this long before reading so the whole
        # acc+gyro+angle burst (33 bytes = 2.9 ms @115200) is read in one go (0 = off)
        self.declare_parameter('read_coalesce_s', 0.003)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.port, self.baud, self.frame_id = g('port'), int(g('baud')), g('frame_id')
        self.rate = float(g('rate'))
        self.configure_sensor = bool(g('configure_sensor'))
        self.publish_tf = bool(g('publish_tf'))
        self.stale_warn_s = float(g('stale_warn_s'))

        # RELIABLE: robot_localization's ekf_node / navsat subscribe with the default (reliable)
        # profile; a BEST_EFFORT publisher is incompatible and they would never receive IMU data.
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
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

        self.fast_publish = bool(g('fast_publish'))
        self.read_coalesce_s = max(0.0, float(g('read_coalesce_s')))
        self._temp_check = 0.0
        self._temp_subs = True
        self._imu_ser = ImuSerializer(self.frame_id, self.orientation_cov,
                                      self.angular_velocity_cov,
                                      self.linear_acceleration_cov)

        self.parser = WitParser()
        self.ser = None
        self._last_open_attempt = 0.0
        self._last_frame_mono = 0.0
        self._published = 0
        self._last_stale_warn = 0.0
        self._first_yaw_logged = False

        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, name='wit_reader', daemon=True)
        self._reader.start()
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

    def _close_serial(self):
        ser, self.ser = self.ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    def _read_loop(self):
        """Serial reader thread: block in select() on the tty, feed whatever arrived."""
        ctx = self.context
        while not self._stop.is_set() and ctx.ok():
            if self.ser is None:
                self._open()
                if self.ser is None:
                    self._stop.wait(0.5)
                    continue
            try:
                fd = self.ser.fileno()
                ready, _, _ = select.select([fd], [], [], 0.5)
                if not ready:
                    continue
                if self.read_coalesce_s:
                    time.sleep(self.read_coalesce_s)
                data = os.read(fd, 4096)
                if not data:   # readable but empty: the device went away
                    raise OSError('EOF on %s' % self.port)
            except BlockingIOError:
                continue
            except (serial.SerialException, OSError, ValueError, TypeError) as exc:
                if self._stop.is_set():
                    break
                self.get_logger().error('serial read failed: %s; reopening' % exc)
                self._close_serial()
                continue
            try:
                self._feed(data)
            except Exception as exc:  # noqa: BLE001 - keep reading
                if self._stop.is_set() or not ctx.ok():
                    break
                self.get_logger().error('IMU frame handling failed: %r' % exc,
                                        throttle_duration_sec=10.0)
        self._close_serial()

    def _feed(self, data):
        for ftype in self.parser.feed(data):
            self._last_frame_mono = time.monotonic()
            if ftype == TYPE_ANGLE and self.parser.sample.complete:
                if not self._first_yaw_logged:
                    self._first_yaw_logged = True
                    self.get_logger().info('first JY61P yaw %.2f deg (sensor-side; continuous '
                                           'unless the sensor lost power)'
                                           % math.degrees(self.parser.sample.rpy[2]))
                self._publish()

    # ---------------------------------------------------------------- publish
    def _publish(self):
        s = self.parser.sample
        quat = s.quat if s.quat is not None else euler_to_quaternion(*s.rpy)
        if self.fast_publish and self.tf_broadcaster is None:
            stamp_ns = self.get_clock().now().nanoseconds
            try:
                self.imu_pub.publish(self._imu_ser.serialize(stamp_ns, quat, s.gyro, s.acc))
                # /imu/temperature_c: only serialized + sent while somebody listens
                if s.temperature_c is not None and self._temp_wanted():
                    self.temp_pub.publish(float32_bytes(s.temperature_c))
                self._published += 1
                return
            except TypeError:   # rclpy without publish(bytes): use the typed path from now on
                self.fast_publish = False
        self._publish_typed(s, quat)

    def _temp_wanted(self):
        now = time.monotonic()
        if now >= self._temp_check:      # graph query at most once per second
            self._temp_check = now + 1.0
            try:
                self._temp_subs = self.temp_pub.get_subscription_count() > 0
            except AttributeError:       # test doubles
                self._temp_subs = True
        return self._temp_subs

    def _publish_typed(self, s, quat):
        stamp = self.get_clock().now().to_msg()
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat
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

    def destroy_node(self):
        self._stop.set()
        if self._reader.is_alive():
            self._reader.join(timeout=2.0)
        super().destroy_node()


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
