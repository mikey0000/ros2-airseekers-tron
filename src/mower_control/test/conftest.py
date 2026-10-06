"""Test configuration for mower_control.

The node logic under test (slew/watchdog step, slip step, IMU window evaluation, fast CDR
helpers) is exercised directly. When ``rclpy`` is not importable (plain host Python) a small
shim registers the module names the nodes import, so the same tests run on the host and
inside the Humble container; tests that need real serialization skip on the shim.
"""

import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _install_shim():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class _Any:
        def __init__(self, *args, **kwargs):
            pass

    class _Policy:
        RELIABLE = BEST_EFFORT = KEEP_LAST = KEEP_ALL = VOLATILE = TRANSIENT_LOCAL = 0

    def _deserialize(_data, _type):
        raise RuntimeError('rclpy shim: no deserialize_message')

    mod('rclpy', init=lambda **_k: None, ok=lambda: False, shutdown=lambda: None,
        spin=lambda *_a, **_k: None, SHIM=True)
    mod('rclpy.node', Node=_Any)
    mod('rclpy.qos', QoSProfile=_Any, QoSReliabilityPolicy=_Policy,
        QoSHistoryPolicy=_Policy, QoSDurabilityPolicy=_Policy)
    mod('rclpy.callback_groups', CallbackGroup=_Any)
    mod('rclpy.serialization', deserialize_message=_deserialize)
    mod('rclpy.parameter', Parameter=_Any)
    mod('rcl_interfaces')
    mod('rcl_interfaces.msg', SetParametersResult=_Any)
    for pkg, names in (('geometry_msgs', ('Twist', 'TwistStamped')),
                       ('nav_msgs', ('Odometry',)),
                       ('sensor_msgs', ('Imu',)),
                       ('std_msgs', ('Bool', 'String'))):
        mod(pkg)
        mod(pkg + '.msg', **{n: type(n, (_Any,), {}) for n in names})


try:  # pragma: no cover - depends on the environment
    import rclpy  # noqa: F401
    import geometry_msgs.msg  # noqa: F401

    HAS_RCLPY = True
except ImportError:  # pragma: no cover
    _install_shim()
    HAS_RCLPY = False
