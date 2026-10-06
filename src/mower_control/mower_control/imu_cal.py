#!/usr/bin/env python3
"""At-rest IMU bias calibration with a persisted, validated file (ROS 2 Humble).

Port of the MowgliNext at-rest bias-calibration pattern -- see
``ros2_stack/docs/mowglinext_baseline.md`` section 4.3, item 7: collect IMU
samples while the mower is at rest, keep only plausible windows (|accel| ~ g,
small gyro/accel variance), average them into gyro/accel biases and persist
them to a YAML file that is validated on load (reject all-zero biases /
|gyro offset| > 0.2 rad/s).

Subscribed topics
-----------------
``/imu/data``   sensor_msgs/Imu   from wit_imu_driver (reliable). Only while
                                   calibrating: with a valid calibration file
                                   and ``recalibrate_on_boot`` false the node
                                   never subscribes.

Published topics
----------------
``/bias_status``  std_msgs/String   calibration state/progress,
                                       QoS(1).transient_local()

Parameters
----------
``calibration_file``     ``~/mower_control/imu_calibration.yaml``  output/input file
``recalibrate_on_boot``  ``false``   calibrate even when a valid file exists
``gravity``              ``9.81``    m/s^2 expected |accel| at rest
``accel_tolerance``      ``0.5``     m/s^2 -- |(|accel| - g)| must be below this
``gyro_variance_max``    ``0.0001``  (rad/s)^2 -- max axis gyro variance of a
                                      "still" window
``accel_variance_max``   ``0.01``    (m/s^2)^2 -- max axis accel variance of a
                                      "still" window (the only motion check
                                      left when the gyro is dead-banded)
``require_gyro_noise``   ``false``   also require non-zero gyro variance on
                                      every axis (the MowgliNext "alive" rule).
                                      Off by default: the WIT JY61P outputs
                                      exactly 0.0 rad/s on all axes at rest
                                      (firmware static zeroing), which made
                                      every window fail. Liveness is checked on
                                      the accelerometer instead.
``gyro_bias_max``        ``0.2``     rad/s -- reject windows whose |gyro bias| is
                                      larger (MowgliNext validation rule)
``min_samples``          ``100``     samples per evaluation window
``sample_rate``          ``20.0``    Hz /imu/data sampling (newest message per
                                      tick; the topic itself is 100 Hz)
``window_timeout``       ``60.0``    s of collecting before an attempt FAILs
``retry_delay``          ``60.0``    s before the first retry after a FAILED
                                      attempt; doubles per failure ...
``retry_delay_max``      ``600.0``   ... up to this
``status_rate``          ``1.0``     Hz /bias_status publish rate while collecting

Plausibility rules (per window, MowgliNext style)
-------------------------------------------------
* ``| |accel| - g | < accel_tolerance``  -- the mower is at rest, not accelerating
* the IMU is alive: some accel variance (and, with ``require_gyro_noise``,
  non-zero gyro variance on every axis)
* max axis gyro variance < gyro_variance_max and max axis accel variance <
  accel_variance_max  -- the mower is not moving
* ``|gyro bias| <= gyro_bias_max``  -- sanity, mirrors the file-load validation

The statistics of the first window are logged once at INFO (and the last
rejected window with its reasons on every FAILED) so the thresholds can be
tuned against the real sensor.

CPU (RK3588): /imu/data is a sampled input of a ``SubscriptionPump`` read by a
``PeriodicRunner`` at ``sample_rate`` (no executor wake per 100 Hz message);
the subscription is destroyed while waiting for a retry and once calibrated.

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
``gyro_bias_max``, missing keys or unreadable YAML are rejected with a warning).
A valid file is used as is unless ``recalibrate_on_boot`` is set; otherwise the
node calibrates and overwrites it once a plausible window completes.
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

from mower_control.fast_msgs import string_bytes
from mower_control.sub_pump import PeriodicRunner, SubscriptionPump, parse_imu

_CALIBRATION_HEADER = '# mower_control_imu_calibration_v1'
_ZERO_VAR = 1e-12   # below this a variance is rounding noise of a constant signal


# ----------------------------------------------------------------------
# pure helpers (unit tested without ROS)
# ----------------------------------------------------------------------
def window_stats(window):
    """Mean/variance statistics of ``[(ax, ay, az, gx, gy, gz), ...]``."""
    n = len(window)
    mean = [sum(w[i] for w in window) / n for i in range(6)]
    var = [sum((w[i] - mean[i]) ** 2 for w in window) / n for i in range(6)]
    mean_a, mean_g = mean[:3], mean[3:]
    return {
        'n': n,
        'mean_a': mean_a,
        'mean_g': mean_g,
        'var_a': var[:3],
        'var_g': var[3:],
        'mag_a': math.sqrt(sum(v * v for v in mean_a)),
        'bias_mag': math.sqrt(sum(v * v for v in mean_g)),
    }


def window_rejections(stats, gravity, accel_tolerance, gyro_variance_max,
                      accel_variance_max, gyro_bias_max, require_gyro_noise):
    """Reasons ``stats`` is not a plausible at-rest window (empty list = plausible)."""
    reasons = []
    if not abs(stats['mag_a'] - gravity) < accel_tolerance:
        reasons.append('|accel| %.3f not within %.3f of %.3f'
                       % (stats['mag_a'], accel_tolerance, gravity))
    if not any(v > _ZERO_VAR for v in stats['var_a']):
        reasons.append('accelerometer frozen (zero variance)')
    if require_gyro_noise and not all(v > _ZERO_VAR for v in stats['var_g']):
        reasons.append('gyro axis with zero variance (require_gyro_noise)')
    if not max(stats['var_g']) < gyro_variance_max:
        reasons.append('gyro variance %.2e >= %.2e' % (max(stats['var_g']), gyro_variance_max))
    if not max(stats['var_a']) < accel_variance_max:
        reasons.append('accel variance %.2e >= %.2e'
                       % (max(stats['var_a']), accel_variance_max))
    if not stats['bias_mag'] <= gyro_bias_max:
        reasons.append('|gyro bias| %.4f > %.4f' % (stats['bias_mag'], gyro_bias_max))
    return reasons


def biases_from_stats(stats, gravity):
    """(gyro_bias, accel_bias): accel bias = mean specific force minus g along the measured
    gravity direction (works for any IMU orientation)."""
    mag = stats['mag_a']
    g_dir = [v / mag for v in stats['mean_a']]
    return (list(stats['mean_g']),
            [stats['mean_a'][i] - gravity * g_dir[i] for i in range(3)])


def retry_delay(failures, initial, maximum):
    """Backoff before retry number ``failures`` (1 = after the first FAILED)."""
    return min(maximum, initial * 2.0 ** max(0, failures - 1))


def validate_calibration(data, gyro_bias_max):
    """``(gyro, accel)`` from a loaded calibration dict; ValueError with the reason."""
    try:
        gyro = [float(v) for v in data['gyro_bias_radps']]
        accel = [float(v) for v in data['accel_bias_mps2']]
    except (KeyError, TypeError, ValueError):
        raise ValueError('malformed') from None
    if len(gyro) != 3 or len(accel) != 3:
        raise ValueError('malformed')
    if not any(gyro) and not any(accel):
        raise ValueError('all-zero biases')
    if math.sqrt(sum(v * v for v in gyro)) > gyro_bias_max:
        raise ValueError('|gyro bias| > %.3f rad/s' % gyro_bias_max)
    return gyro, accel


def _fmt(values, spec='%.3e'):
    return '[' + ', '.join(spec % v for v in values) + ']'


class ImuCalNode(Node):
    """Calibrates gyro/accel biases at rest and persists them to YAML."""

    def __init__(self):
        super().__init__('imu_cal')

        self.declare_parameter('calibration_file',
                               '~/mower_control/imu_calibration.yaml')
        self.declare_parameter('recalibrate_on_boot', False)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('accel_tolerance', 0.5)
        self.declare_parameter('gyro_variance_max', 0.0001)
        self.declare_parameter('accel_variance_max', 0.01)
        self.declare_parameter('require_gyro_noise', False)
        self.declare_parameter('gyro_bias_max', 0.2)
        self.declare_parameter('min_samples', 100)
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('window_timeout', 60.0)
        self.declare_parameter('retry_delay', 60.0)
        self.declare_parameter('retry_delay_max', 600.0)
        self.declare_parameter('status_rate', 1.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._calibration_file = os.path.expanduser(str(p('calibration_file')))
        self._recalibrate = bool(p('recalibrate_on_boot'))
        self._gravity = float(p('gravity'))
        self._thresholds = dict(
            gravity=self._gravity,
            accel_tolerance=float(p('accel_tolerance')),
            gyro_variance_max=float(p('gyro_variance_max')),
            accel_variance_max=float(p('accel_variance_max')),
            gyro_bias_max=float(p('gyro_bias_max')),
            require_gyro_noise=bool(p('require_gyro_noise')),
        )
        self._gyro_bias_max = self._thresholds['gyro_bias_max']
        self._min_samples = int(p('min_samples'))
        self._sample_rate = float(p('sample_rate'))
        self._window_timeout = float(p('window_timeout'))
        self._retry_delay = float(p('retry_delay'))
        self._retry_delay_max = float(p('retry_delay_max'))
        self._status_rate = float(p('status_rate'))

        # COLLECTING -> CALIBRATED | FAILED (-> COLLECTING after the backoff)
        self._state = 'COLLECTING'
        self._source = 'calibration'
        self._window = []             # [(ax, ay, az, gx, gy, gz), ...]
        self._start_time = None
        self._gyro_bias = None
        self._accel_bias = None
        self._samples = 0
        self._failures = 0
        self._retry_at = None
        self._noise_logged = False
        self._last_stats = None
        self._last_reasons = None
        self._pump = None
        self._runner = None

        status_qos = QoSProfile(depth=1,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, '/bias_status', status_qos)

        loaded = self._load_existing()
        if loaded is not None and not self._recalibrate:
            self._gyro_bias, self._accel_bias, self._samples = loaded
            self._state = 'CALIBRATED'
            self._source = 'file'
            self._publish_status()
            self.get_logger().info(
                'imu_cal: using the existing calibration, /imu/data not subscribed '
                '(set recalibrate_on_boot:=true to recalibrate)')
            return

        self._runner = PeriodicRunner(
            self, [(1.0 / self._sample_rate, self._sample_tick),
                   (1.0 / self._status_rate, self._status_tick)], 'imu_cal')
        self._start_collecting()
        self._runner.start()
        self.get_logger().info(
            'imu_cal ready: file=%s, min_samples=%d @ %.1f Hz, window_timeout=%.1f s, '
            'thresholds=%s' % (self._calibration_file, self._min_samples,
                               self._sample_rate, self._window_timeout, self._thresholds))

    def shutdown(self):
        if self._runner is not None:
            self._runner.stop()
        self._stop_collecting()

    # ------------------------------------------------------------------
    # collection lifecycle (runs on the PeriodicRunner thread after __init__)
    # ------------------------------------------------------------------
    def _start_collecting(self):
        self._state = 'COLLECTING'
        self._window = []
        self._start_time = time.monotonic()
        self._retry_at = None
        self._pump = SubscriptionPump(self, 'imu_cal_inputs')
        self._pump.subscribe(Imu, '/imu/data', self._on_imu,
                             QoSProfile(depth=1,
                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                        history=QoSHistoryPolicy.KEEP_LAST),
                             parser=parse_imu, sampled=True)
        self._pump.start()   # sampled only: no thread

    def _stop_collecting(self):
        pump, self._pump = self._pump, None
        if pump is not None:
            pump.close()

    def _on_imu(self, msg):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self._window.append((a.x, a.y, a.z, g.x, g.y, g.z))

    def _sample_tick(self):
        if self._state != 'COLLECTING' or self._pump is None:
            return
        self._pump.poll()
        if len(self._window) >= self._min_samples:
            self._evaluate_window()

    def _status_tick(self):
        now = time.monotonic()
        if self._state == 'COLLECTING':
            if now - self._start_time > self._window_timeout:
                self._fail(now)
            self._publish_status()
        elif self._state == 'FAILED' and now >= self._retry_at:
            self.get_logger().info('imu_cal: retrying calibration (attempt %d)'
                                   % (self._failures + 1))
            self._start_collecting()
            self._publish_status()

    def _fail(self, now):
        self._failures += 1
        delay = retry_delay(self._failures, self._retry_delay, self._retry_delay_max)
        self._retry_at = now + delay
        self._state = 'FAILED'
        self._stop_collecting()
        if self._last_stats is None:
            detail = 'no complete window (is /imu/data publishing?)'
        else:
            detail = 'last window: %s; rejected: %s' % (
                self._describe(self._last_stats), '; '.join(self._last_reasons))
        self.get_logger().error(
            'calibration FAILED (attempt %d): no plausible at-rest window within %.1f s '
            '(is the mower stationary?) -- %s. Retrying in %.0f s.'
            % (self._failures, self._window_timeout, detail, delay))

    # ------------------------------------------------------------------
    # calibration logic
    # ------------------------------------------------------------------
    @staticmethod
    def _describe(stats):
        return ('|accel|=%.3f accel_mean=%s accel_var=%s gyro_mean=%s gyro_var=%s (n=%d)'
                % (stats['mag_a'], _fmt(stats['mean_a'], '%.3f'), _fmt(stats['var_a']),
                   _fmt(stats['mean_g']), _fmt(stats['var_g']), stats['n']))

    def _evaluate_window(self):
        stats = window_stats(self._window)
        self._window = []
        if not self._noise_logged:
            self._noise_logged = True
            self.get_logger().info('measured IMU statistics (first window, %.1f Hz): %s'
                                   % (self._sample_rate, self._describe(stats)))
        reasons = window_rejections(stats, **self._thresholds)
        self._last_stats, self._last_reasons = stats, reasons
        if reasons:
            self.get_logger().debug('implausible window discarded: %s -- %s'
                                    % (self._describe(stats), '; '.join(reasons)))
            return
        self._gyro_bias, self._accel_bias = biases_from_stats(stats, self._gravity)
        self._samples = stats['n']
        self._state = 'CALIBRATED'
        self._source = 'calibration'
        self._stop_collecting()
        self._save()
        self._publish_status()
        if self._runner is not None:
            self._runner.stop()   # nothing left to do; /bias_status is latched
        self.get_logger().info(
            'calibration done: gyro_bias=%s accel_bias=%s from %d samples'
            % (self._gyro_bias, self._accel_bias, self._samples))

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
        """Validate any existing calibration file (MowgliNext-style checks).

        Returns ``(gyro, accel, samples)`` or None.
        """
        try:
            with open(self._calibration_file) as f:
                data = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warn(
                'no usable calibration file at %s (%s); will calibrate fresh'
                % (self._calibration_file, exc))
            return None
        try:
            gyro, accel = validate_calibration(data, self._gyro_bias_max)
        except ValueError as exc:
            self.get_logger().warn('calibration file %s rejected (%s); ignoring it'
                                   % (self._calibration_file, exc))
            return None
        self.get_logger().info(
            'loaded existing calibration from %s: gyro_bias=%s accel_bias=%s'
            % (self._calibration_file, gyro, accel))
        try:
            samples = int(data.get('samples', 0))
        except (TypeError, ValueError):
            samples = 0
        return gyro, accel, samples

    # ------------------------------------------------------------------
    # status
    # ------------------------------------------------------------------
    def _publish_status(self):
        if self._state == 'CALIBRATED':
            text = ('state=CALIBRATED gyro_bias_radps=%s accel_bias_mps2=%s '
                    'samples=%d file=%s source=%s'
                    % (self._gyro_bias, self._accel_bias, self._samples,
                       self._calibration_file, self._source))
        elif self._state == 'FAILED':
            text = ('state=FAILED reason=timeout attempts=%d retry_in_s=%.0f'
                    % (self._failures, max(0.0, self._retry_at - time.monotonic())))
        else:
            text = 'state=COLLECTING samples=%d/%d' % (len(self._window),
                                                        self._min_samples)
        self._status_pub.publish(string_bytes(text))


def main(args=None):
    rclpy.init(args=args)
    node = ImuCalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
