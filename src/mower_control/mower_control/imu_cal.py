#!/usr/bin/env python3
"""At-rest IMU bias calibration with a persisted, validated file (ROS 2 Humble).

Port of the MowgliNext at-rest bias-calibration pattern -- see
``ros2_stack/docs/mowglinext_baseline.md`` section 4.3, item 7: collect IMU
samples while the mower is at rest, keep only plausible windows (|accel| ~ g,
small but non-zero gyro variance), average them into gyro/accel biases and
persist them to a YAML file that is validated on load (reject all-zero
biases / |gyro offset| > 0.2 rad/s).

Subscribed topics
-----------------
``/imu/data``   sensor_msgs/Imu   from wit_imu_driver (reliable QoS(10))

Published topics
----------------
``/bias_status``  std_msgs/String   calibration state/progress,
                                       QoS(1).transient_local()

Parameters
----------
``calibration_file``  ``~/mower_control/imu_calibration.yaml``  output/input file
``gravity``            ``9.81``    m/s^2 expected |accel| at rest
``accel_tolerance``    ``0.5``     m/s^2 -- |(|accel| - g)| must be below this
``gyro_variance_max``  ``0.0001``  (rad/s)^2 -- window gyro variance must be
                                     below this (window is "still")
``gyro_bias_max``      ``0.2``     rad/s -- reject windows whose |gyro bias| is
                                     larger (MowgliNext validation rule)
``min_samples``        ``100``     samples per evaluation window
``window_timeout``     ``60.0``    s before giving up with FAILED
``status_rate``        ``1.0``     Hz /bias_status publish rate

Plausibility rules (per window, MowgliNext style)
-------------------------------------------------
* ``| |accel| - g | < accel_tolerance``  -- the mower is at rest, not accelerating
* every axis has non-zero gyro variance  -- the IMU is alive, not stuck
* max axis gyro variance < gyro_variance_max  -- the mower is not moving
* ``|gyro bias| <= gyro_bias_max``  -- sanity, mirrors the file-load validation

File format
-----------
Flat YAML with a version header, written atomically (tmp + rename)::

    # mower_control_imu_calibration_v1
    accel_bias_mps2: [0.01, 0.02, -0.03]
    calibrated_at: '2026-10-05T12:00:00+00:00'
    gravity_mps2: 9.81
    gyro_bias_radps: [0.001, -0.002, 0.003]
    samples: 100

An existing file is validated on startup (all-zero biases, |gyro bias| >
``gyro_bias_max``, missing keys or unreadable YAML are rejected with a warning)
and then overwritten once a plausible window completes.
"""

import math
import os
import time
from datetime import datetime, timezone

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from sensor_msgs.msg import Imu
from std_msgs.msg import String

_CALIBRATION_HEADER = '# mower_control_imu_calibration_v1'


class ImuCalNode(Node):
    """Calibrates gyro/accel biases at rest and persists them to YAML."""

    def __init__(self):
        super().__init__('imu_cal')

        self.declare_parameter('calibration_file',
                               '~/mower_control/imu_calibration.yaml')
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('accel_tolerance', 0.5)
        self.declare_parameter('gyro_variance_max', 0.0001)
        self.declare_parameter('gyro_bias_max', 0.2)
        self.declare_parameter('min_samples', 100)
        self.declare_parameter('window_timeout', 60.0)
        self.declare_parameter('status_rate', 1.0)

        self._calibration_file = os.path.expanduser(
            str(self.get_parameter('calibration_file').value))
        self._gravity = float(self.get_parameter('gravity').value)
        self._accel_tolerance = float(self.get_parameter('accel_tolerance').value)
        self._gyro_variance_max = float(self.get_parameter('gyro_variance_max').value)
        self._gyro_bias_max = float(self.get_parameter('gyro_bias_max').value)
        self._min_samples = int(self.get_parameter('min_samples').value)
        self._window_timeout = float(self.get_parameter('window_timeout').value)
        self._status_rate = float(self.get_parameter('status_rate').value)

        self._state = 'COLLECTING'   # COLLECTING -> CALIBRATED | FAILED
        self._window = []             # [(ax, ay, az, gx, gy, gz), ...]
        self._start_time = time.monotonic()
        self._gyro_bias = None
        self._accel_bias = None
        self._samples = 0

        qos10 = QoSProfile(depth=10,
                           reliability=QoSReliabilityPolicy.RELIABLE,
                           history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(Imu, '/imu/data', self._on_imu, qos10)

        status_qos = QoSProfile(depth=1,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, '/bias_status', status_qos)

        self.create_timer(1.0 / self._status_rate, self._tick)
        self._load_existing()
        self.get_logger().info(
            'imu_cal ready: file=%s, min_samples=%d, window_timeout=%.1f s'
            % (self._calibration_file, self._min_samples, self._window_timeout))

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_imu(self, msg):
        if self._state != 'COLLECTING':
            return
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self._window.append((a.x, a.y, a.z, g.x, g.y, g.z))
        if len(self._window) >= self._min_samples:
            self._evaluate_window()

    def _tick(self):
        if (self._state == 'COLLECTING' and
                time.monotonic() - self._start_time > self._window_timeout):
            self._state = 'FAILED'
            self.get_logger().error(
                'calibration FAILED: no plausible at-rest window within %.1f s '
                '(is the mower stationary?)' % self._window_timeout)
        self._publish_status()

    # ------------------------------------------------------------------
    # calibration logic
    # ------------------------------------------------------------------
    def _evaluate_window(self):
        n = len(self._window)
        mean_a = [sum(w[i] for w in self._window) / n for i in range(3)]
        mean_g = [sum(w[3 + i] for w in self._window) / n for i in range(3)]
        mag_a = math.sqrt(sum(v * v for v in mean_a))
        var_g = [sum((w[3 + i] - mean_g[i]) ** 2 for w in self._window) / n
                 for i in range(3)]
        bias_mag = math.sqrt(sum(v * v for v in mean_g))

        accel_ok = abs(mag_a - self._gravity) < self._accel_tolerance
        alive = all(v > 0.0 for v in var_g)          # no stuck axis
        still = max(var_g) < self._gyro_variance_max
        bias_ok = bias_mag <= self._gyro_bias_max

        if accel_ok and alive and still and bias_ok:
            # Accel bias = mean specific force minus g along the measured
            # gravity direction (works for any IMU orientation).
            g_dir = [v / mag_a for v in mean_a]
            self._gyro_bias = mean_g
            self._accel_bias = [mean_a[i] - self._gravity * g_dir[i]
                                for i in range(3)]
            self._samples = n
            self._state = 'CALIBRATED'
            self._save()
            self.get_logger().info(
                'calibration done: gyro_bias=%s accel_bias=%s from %d samples'
                % (self._gyro_bias, self._accel_bias, n))
        else:
            self.get_logger().warn(
                'implausible window discarded: |accel|=%.3f (expected %.3f), '
                'gyro_var=%s, |gyro_bias|=%.4f'
                % (mag_a, self._gravity,
                   ['%.2e' % v for v in var_g], bias_mag))
            self._window = []

    # ------------------------------------------------------------------
    # persistence
    # ------------------------------------------------------------------
    def _save(self):
        data = {
            'gyro_bias_radps': [float(v) for v in self._gyro_bias],
            'accel_bias_mps2': [float(v) for v in self._accel_bias],
            'gravity_mps2': self._gravity,
            'samples': self._samples,
            'calibrated_at': datetime.now(timezone.utc).isoformat(),
        }
        directory = os.path.dirname(self._calibration_file)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp = self._calibration_file + '.tmp'
        with open(tmp, 'w') as f:
            f.write(_CALIBRATION_HEADER + '\n')
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)
        os.replace(tmp, self._calibration_file)

    def _load_existing(self):
        """Validate any existing calibration file (MowgliNext-style checks)."""
        try:
            with open(self._calibration_file) as f:
                data = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warn(
                'no usable calibration file at %s (%s); will calibrate fresh'
                % (self._calibration_file, exc))
            return
        try:
            gyro = [float(v) for v in data['gyro_bias_radps']]
            accel = [float(v) for v in data['accel_bias_mps2']]
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn('calibration file %s is malformed; ignoring it'
                                   % self._calibration_file)
            return
        if not any(gyro) and not any(accel):
            self.get_logger().warn(
                'calibration file %s has all-zero biases; ignoring it'
                % self._calibration_file)
            return
        if math.sqrt(sum(v * v for v in gyro)) > self._gyro_bias_max:
            self.get_logger().warn(
                'calibration file %s has |gyro bias| > %.3f rad/s; ignoring it'
                % (self._calibration_file, self._gyro_bias_max))
            return
        self.get_logger().info(
            'loaded existing calibration from %s: gyro_bias=%s accel_bias=%s'
            % (self._calibration_file, gyro, accel))

    # ------------------------------------------------------------------
    # status
    # ------------------------------------------------------------------
    def _publish_status(self):
        if self._state == 'CALIBRATED':
            text = ('state=CALIBRATED gyro_bias_radps=%s accel_bias_mps2=%s '
                    'samples=%d file=%s' % (self._gyro_bias, self._accel_bias,
                                            self._samples,
                                            self._calibration_file))
        elif self._state == 'FAILED':
            text = 'state=FAILED reason=timeout'
        else:
            text = 'state=COLLECTING samples=%d/%d' % (len(self._window),
                                                        self._min_samples)
        self._status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = ImuCalNode()
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
