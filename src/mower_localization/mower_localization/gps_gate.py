#!/usr/bin/env python3
"""GNSS quality gate: republishes ``/fix`` only when it is a usable fix.

Sits between ``um960_gps_driver`` (``/fix``, 10 Hz) and
``navsat_transform_node``:

    /fix  --[gps_gate]-->  /fix_gated  -->  navsat_transform_node

A fix is republished on ``/fix_gated`` only when it is worth fusing:

1. ``NavSatFix.status.status >= min_fix_status`` (default
   ``NavSatStatus.STATUS_FIX``).  The UM960 driver maps GGA quality 0
   to ``STATUS_NO_FIX`` and everything else to ``STATUS_FIX`` /
   ``STATUS_DGPS_FIX``; this keeps no-fix, estimated and simulated
   "fixes" out of the filter.
2. The largest diagonal ``position_covariance`` entry is
   ``<= max_position_covariance`` (m^2), when the receiver reports a
   known covariance.  A 10 Hz stream of 500 m^2 single-point "fixes"
   would drag the EKF around; a gated silence lets it coast on wheel
   odometry instead.  ``0`` disables the check.
3. ``status.status`` is one of ``used_fixes`` (empty list disables
   the check): the set of fix types this stack uses.  Defaults to
   plain GPS fix, DGPS, RTK-fixed and RTK-float; set ``[4]`` for
   RTK-fixed-only operation.

The decision is deliberately made from the message itself
(``status`` + ``position_covariance``), never from a filter that has
already consumed the fix — MowgliNext's dig-detector rule: gate on
the receiver's own reported truth.

Rejected messages are counted and logged (throttled) and *not*
published, so downstream nodes see a clean stop rather than a stream
of bad fixes.  ``NavSatFix`` carries no satellite count, so a
"satellites used" gate has to live in the driver (the UM960 parsers
track ``num_sats_used``).

The message is forwarded with its stamp and ``header.frame_id`` intact
so ``navsat_transform_node``'s TF lookups and latency handling keep
working.

4. 2026-10-09 (``quality_covariance``, default true): the RTK solution
   class from ``/fix_status`` (:mod:`mower_localization.rtk_quality`;
   NavSatFix cannot tell fixed from float) sets the horizontal
   covariance the filter sees: fixed keeps the receiver sigma (3 cm
   floor), float is inflated x4 with a 0.4 m sigma floor, DGPS/single
   are dropped (``single_policy: reject``) or given a 3 m floor
   (``loose``). navsat_transform_node copies this covariance into
   ``/odometry/gps``, so ``ekf_map`` leans on wheel + IMU (+ VIO) during
   float instead of chasing decimetre noise. Without a fresh
   /fix_status the class falls back to the NavSatStatus (GBAS = float).
   ``quality_covariance: false`` restores the pass-through behaviour.

Parameters
----------
``input_topic``            ``/fix``       raw NavSatFix from um960_gps_driver
``output_topic``           ``/fix_gated`` gated NavSatFix for navsat_transform_node
``min_fix_status``         ``1``          (``NavSatStatus.STATUS_FIX``)
``max_position_covariance``  ``100.0``    m^2 (10 m sigma); ``0`` disables
``used_fixes``             ``[STATUS_FIX, STATUS_DGPS_FIX, STATUS_RTK_FIX,
                            STATUS_RTK_FLOAT]``

QoS is reliable, KEEP_LAST(10) on both sides: it matches the UM960
driver's ``/fix`` profile, and ``navsat_transform_node`` subscribes
with the default (reliable) profile, so the gated topic connects to
both.  (A best-effort publisher here would silently drop every fix.)

Run:

    ros2 run mower_localization gps_gate
"""

import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from mower_localization import rtk_quality as rq

# NavSatStatus values (sensor_msgs/msg/NavSatStatus, ROS 2 Jazzy):
#   -1 NO_FIX, 0 FIX (autonomous), 1 SBAS_FIX (differential), 2 GBAS_FIX (RTK fixed/float)
# um960_gps_driver maps GGA quality 2 -> SBAS_FIX and 4/5 -> GBAS_FIX; RTK fixed vs float
# is only visible through the position covariance, which is why the gate also checks that.
DEFAULT_USED_FIXES = (
    NavSatStatus.STATUS_FIX,
    NavSatStatus.STATUS_SBAS_FIX,
    NavSatStatus.STATUS_GBAS_FIX,
)

# Log a rejection at most once per this many seconds.
WARN_PERIOD_S = 5.0


def _max_position_covariance(msg: NavSatFix) -> Optional[float]:
    """Largest diagonal position covariance in m^2, or ``None`` when unknown.

    ``COVARIANCE_TYPE_UNKNOWN`` (what um960_gps_driver reports when the
    receiver gave no accuracy numbers) means "no information": the gate
    cannot check it and lets the message through.
    """
    if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        return None
    covariance = msg.position_covariance
    return max(covariance[0], covariance[4], covariance[8])


def _reject_reason(
    msg: NavSatFix,
    min_fix_status: int,
    max_position_covariance: float,
    used_fixes: frozenset,
) -> Optional[str]:
    """Return why ``msg`` must not be republished, or ``None`` if it passes.

    Pure: no ROS state, so the whole safety decision is unit-testable.
    """
    if msg.status.status < min_fix_status:
        return ("status %d is below min_fix_status %d"
                % (msg.status.status, min_fix_status))
    if used_fixes and msg.status.status not in used_fixes:
        return "status %d is not one of the used fixes %s" % (
            msg.status.status, sorted(used_fixes))
    if max_position_covariance > 0.0:
        covariance = _max_position_covariance(msg)
        if (covariance is not None
                and covariance > max_position_covariance):
            return ("position covariance %.2f m^2 exceeds "
                    "max_position_covariance %.2f m^2"
                    % (covariance, max_position_covariance))
    return None


def _apply_quality(msg: NavSatFix, rtk_class: Optional[str],
                   policy: rq.CovariancePolicy) -> Optional[str]:
    """Rewrite ``msg``'s horizontal covariance for its RTK class (in place).

    Returns why the fix must be dropped instead, or ``None``. Pure, like
    :func:`_reject_reason`. Vertical variance is kept (navsat zero_altitude).
    """
    rx = _max_horizontal_covariance(msg)
    var = rq.fused_variance(rtk_class, rx, policy)
    if var is None:
        return "RTK class %s is not fused (single_policy %s)" % (
            rtk_class, policy.single_policy)
    cov = list(msg.position_covariance)
    if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        cov = [0.0] * 9
        cov[8] = var * 4.0
    cov[0] = cov[4] = var
    cov[1] = cov[3] = 0.0
    msg.position_covariance = cov
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return None


def _max_horizontal_covariance(msg: NavSatFix) -> Optional[float]:
    if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        return None
    return max(msg.position_covariance[0], msg.position_covariance[4])


class GpsGateNode(Node):
    """Republish ``/fix`` on ``/fix_gated`` only when it is a usable fix."""

    def __init__(self, **kwargs) -> None:
        super().__init__("gps_gate", **kwargs)

        self.declare_parameter("input_topic", "/fix")
        self.declare_parameter("output_topic", "/fix_gated")
        self.declare_parameter("min_fix_status", NavSatStatus.STATUS_FIX)
        self.declare_parameter("max_position_covariance", 100.0)
        # Non-empty default so rclpy types this as an integer array.
        self.declare_parameter("used_fixes", list(DEFAULT_USED_FIXES))

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.min_fix_status = int(self.get_parameter("min_fix_status").value)
        self.max_position_covariance = float(
            self.get_parameter("max_position_covariance").value)
        self.used_fixes = frozenset(
            int(value) for value in self.get_parameter("used_fixes").value)

        # 2026-10-09: fix-quality covariance (item 6). Policy keys = rtk_quality fields.
        self.declare_parameter("quality_covariance", True)
        self.declare_parameter("fix_status_topic", "/fix_status")
        self.declare_parameter("fix_status_timeout", 2.0)
        defaults = rq.CovariancePolicy()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.quality = bool(self.get_parameter("quality_covariance").value)
        self.fix_status_timeout = float(self.get_parameter("fix_status_timeout").value)
        self.policy = rq.CovariancePolicy(**{
            name: type(value)(self.get_parameter(name).value)
            for name, value in vars(defaults).items()})
        self._rtk_class: Optional[str] = None
        self._rtk_t: Optional[float] = None

        fix_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.fix_pub = self.create_publisher(
            NavSatFix, self.output_topic, fix_qos)
        self.create_subscription(
            NavSatFix, self.input_topic, self._on_fix, fix_qos)
        if self.quality:
            self.create_subscription(
                String, str(self.get_parameter("fix_status_topic").value),
                self._on_fix_status, fix_qos)

        # Counters for bring-up; a diagnostics hookup lands with
        # mower_monitoring (mowglinext_baseline.md section 7.2).
        self.received = 0
        self.passed = 0
        self.rejected = 0

        self.get_logger().info(
            "gps_gate: %s -> %s (min_status=%d, max_cov=%.1f m^2, "
            "used_fixes=%s)"
            % (self.input_topic, self.output_topic, self.min_fix_status,
               self.max_position_covariance,
               sorted(self.used_fixes) if self.used_fixes else "any"))

    # -- gating ----------------------------------------------------------------
    def _on_fix_status(self, msg: String) -> None:
        cls = rq.classify_fix_status(msg.data)
        if cls is not None:
            self._rtk_class, self._rtk_t = cls, time.monotonic()

    def rtk_class(self, msg: NavSatFix, now: Optional[float] = None) -> str:
        """Fresh /fix_status class, else the conservative NavSatStatus class."""
        now = time.monotonic() if now is None else now
        if (self._rtk_class is not None and self._rtk_t is not None
                and now - self._rtk_t <= self.fix_status_timeout
                and self._rtk_class != rq.RTK_STALE):
            return self._rtk_class
        return rq.class_from_navsat(msg.status.status)

    def _on_fix(self, msg: NavSatFix) -> None:
        self.received += 1
        reason = _reject_reason(
            msg, self.min_fix_status, self.max_position_covariance,
            self.used_fixes)
        if reason is not None:
            self.rejected += 1
            covariance = _max_position_covariance(msg)
            covariance_str = ("unknown" if covariance is None
                              else "%.2f m^2" % covariance)
            self.get_logger().warn(
                "dropping fix from frame %s (status=%d, max_cov=%s): %s"
                % (msg.header.frame_id, msg.status.status, covariance_str,
                   reason),
                throttle_duration_sec=WARN_PERIOD_S)
            return
        if self.quality:
            cls = self.rtk_class(msg)
            reason = _apply_quality(msg, cls, self.policy)
            if reason is not None:
                self.rejected += 1
                self.get_logger().warn("dropping fix: %s" % reason,
                                       throttle_duration_sec=WARN_PERIOD_S)
                return
        self.passed += 1
        self.fix_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('gps_gate')
    except ImportError:
        pass
    rclpy.init(args=args)
    node = GpsGateNode()
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
