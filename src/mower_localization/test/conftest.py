"""Test configuration for mower_localization.

The gate-decision logic in ``gps_gate._reject_reason`` is pure and
runs anywhere. The node tests need ``rclpy``; when it is not
importable (plain Python, no ROS install) a small shim stands in,
mirroring ``um960_gps_driver/test/conftest.py`` so the same tests run
unchanged inside the Humble container.
"""

import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _install_rclpy_shim() -> None:
    """Register minimal stand-ins for the rclpy/sensor_msgs APIs gps_gate uses."""

    class _Stamp:
        def __init__(self, value=0):
            self.sec = value
            self.nanosec = 0

        def to_msg(self):
            return _Stamp(self.sec)

    class Header:
        def __init__(self):
            self.stamp = _Stamp()
            self.frame_id = ""

    class NavSatStatus:
        STATUS_NO_FIX = 0
        STATUS_FIX = 1
        STATUS_DGPS_FIX = 2
        STATUS_PPS_FIX = 3
        STATUS_RTK_FIX = 4
        STATUS_RTK_FLOAT = 5
        STATUS_ESTIMATED = 6
        STATUS_MANUAL = 7
        STATUS_SIMULATION = 8
        SERVICE_GPS = 1

    class NavSatFix:
        COVARIANCE_TYPE_UNKNOWN = 0
        COVARIANCE_TYPE_APPROXIMATED = 1
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
        COVARIANCE_TYPE_KNOWN = 3

        def __init__(self, latitude=0.0, longitude=0.0):
            self.header = Header()
            self.latitude = latitude
            self.longitude = longitude
            self.altitude = 0.0
            self.status = NavSatStatus()
            self.position_covariance = [0.0] * 9
            self.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    class Publisher:
        def __init__(self):
            self.msgs = []

        def publish(self, msg):
            self.msgs.append(msg)

    class Subscription:
        def __init__(self, msg_type, topic, callback, qos=None):
            self.msg_type = msg_type
            self.topic = topic
            self.callback = callback
            self.qos = qos

    class Node:
        """Records parameters; callbacks are driven explicitly by the test."""

        def __init__(self, name, **kwargs):
            self._params = {}
            self.pubs = {}
            self.subs = {}
            for parameter in kwargs.get("parameter_overrides") or []:
                self._params[parameter.name] = parameter.value

        def declare_parameter(self, name, value):
            self._params.setdefault(name, value)

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self._params[name])

        def create_publisher(self, _msg_type, topic, qos=None):
            publisher = Publisher()
            publisher.qos = qos
            self.pubs[topic] = publisher
            return publisher

        def create_subscription(self, msg_type, topic, callback, qos=None):
            subscription = Subscription(msg_type, topic, callback, qos)
            self.subs[topic] = subscription
            return subscription

        def get_logger(self):
            logger = types.SimpleNamespace()
            for level in ("debug", "info", "warn", "error", "fatal"):
                setattr(logger, level, lambda *_args, **_kwargs: None)
            return logger

        def destroy_node(self):
            return True

    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: True
    rclpy.init = lambda **_kwargs: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda *_a, **_k: None

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.NavSatFix = NavSatFix
    sensor_msgs_msg.NavSatStatus = NavSatStatus
    sensor_msgs.msg = sensor_msgs_msg

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = Node
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = lambda **kwargs: kwargs
    qos_mod.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(
        RELIABLE=1, BEST_EFFORT=2)

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": node_mod,
            "rclpy.qos": qos_mod,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
        }
    )


try:  # pragma: no cover - depends on the environment
    import rclpy as _real_rclpy  # noqa: F401
    import sensor_msgs.msg  # noqa: F401

    HAS_RCLPY = True
except ImportError:  # pragma: no cover
    _install_rclpy_shim()
    HAS_RCLPY = False
