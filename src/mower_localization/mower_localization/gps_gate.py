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

The message is forwarded unchanged (stamp and ``header.frame_id``
intact) so ``navsat_transform_node``'s TF lookups and latency
handling keep working.

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

# NavSatStatus values (sensor_msgs/msg/NavSatStatus, ROS 2 Humble):
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

        fix_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.fix_pub = self.create_publisher(
            NavSatFix, self.output_topic, fix_qos)
        self.create_subscription(
            NavSatFix, self.input_topic, self._on_fix, fix_qos)

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
        self.passed += 1
        self.fix_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
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
