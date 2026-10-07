"""Minimal ROS stand-ins so the driver can be imported and smoke-tested without ROS.

``ros:humble`` (and any colcon test environment) provides the real packages; this module only
kicks in when they are missing, which lets the tests in this directory run on a plain host
with nothing but CPython 3.

Everything here is deliberately dumb: constants and message *shapes* the driver touches, plus
record-keeping (published messages, subscriptions, timers) that the tests assert on.
"""

import collections
import sys
import types


class _Param:
    def __init__(self, value):
        self._value = value

    @property
    def value(self):
        return self._value

    def get_parameter_value(self):
        return types.SimpleNamespace(
            string_value=self._value if isinstance(self._value, str) else '',
            double_value=float(self._value) if isinstance(self._value, (int, float)) else 0.0,
            bool_value=bool(self._value) if isinstance(self._value, bool) else False,
            integer_value=int(self._value) if isinstance(self._value, int) else 0)


class _Logger:
    def _log(self, *args, **kwargs):
        pass

    info = warn = warning = error = debug = fatal = _log


class _Stamp:
    def __init__(self):
        self.sec = 0
        self.nanosec = 0


class _Clock:
    def now(self):
        return types.SimpleNamespace(to_msg=lambda: _Stamp())


class _Publisher:
    def __init__(self, node, topic):
        self._node = node
        self.topic = topic

    def publish(self, msg):
        self._node.published[self.topic].append(msg)


class Node:
    """Enough of ``rclpy.node.Node`` for :class:`mower_mcu_driver.mcu_node.McuNode`."""

    def __init__(self, name=''):
        self.node_name = name
        self._params = {}
        self.published = collections.defaultdict(list)
        self.subscriptions = {}
        self.timers = []

    def declare_parameter(self, name, default=None):
        self._params.setdefault(name, default)

    def get_parameter(self, name):
        return _Param(self._params[name])

    def create_publisher(self, msg_type, topic, qos):
        return _Publisher(self, topic)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return callback

    def create_service(self, srv_type, name, callback):
        if not hasattr(self, 'services'):
            self.services = {}
        self.services[name] = callback
        return callback

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return types.SimpleNamespace(period=period, callback=callback)

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        pass


class QoSProfile:
    def __init__(self, depth=10, reliability=None, history=None, **kwargs):
        self.depth = depth
        self.reliability = reliability
        self.history = history


class _Policy:
    RELIABLE = BEST_EFFORT = 0
    KEEP_LAST = KEEP_ALL = 0


# --------------------------------------------------------------------------
# message stubs (only the fields the driver reads or writes)
# --------------------------------------------------------------------------
class Header:
    def __init__(self):
        self.stamp = _Stamp()
        self.frame_id = ''


class _Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _Pose:
    def __init__(self):
        self.position = _Vector3()
        self.orientation = _Quaternion()


class _Twist:
    def __init__(self):
        self.linear = _Vector3()
        self.angular = _Vector3()


class TwistStamped:
    def __init__(self):
        self.header = Header()
        self.twist = _Twist()


class _PoseWithCovariance:
    def __init__(self):
        self.pose = _Pose()
        self.covariance = [0.0] * 36


class _TwistWithCovariance:
    def __init__(self):
        self.twist = _Twist()
        self.covariance = [0.0] * 36


class Odometry:
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ''
        self.pose = _PoseWithCovariance()
        self.twist = _TwistWithCovariance()


class BatteryState:
    POWER_SUPPLY_STATUS_UNKNOWN = 0
    POWER_SUPPLY_STATUS_CHARGING = 1
    POWER_SUPPLY_STATUS_DISCHARGING = 2
    POWER_SUPPLY_STATUS_NOT_CHARGING = 3
    POWER_SUPPLY_STATUS_FULL = 4
    POWER_SUPPLY_HEALTH_UNKNOWN = 0
    POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0

    def __init__(self):
        self.header = Header()
        self.voltage = 0.0
        self.current = 0.0
        self.charge = 0.0
        self.capacity = 0.0
        self.design_capacity = 0.0
        self.percentage = 0.0
        self.power_supply_status = 0
        self.power_supply_health = 0
        self.power_supply_technology = 0
        self.present = False
        self.cell_voltage = []
        self.cell_temperature = []
        self.location = ''
        self.serial_number = ''


class Imu:
    def __init__(self):
        self.header = Header()
        self.orientation = _Quaternion()
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity = _Vector3()
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration = _Vector3()
        self.linear_acceleration_covariance = [0.0] * 9


def _mod(name):
    module = sys.modules.get(name) or types.ModuleType(name)
    sys.modules[name] = module
    return module


def is_stubbed():
    """True when ``rclpy`` in ``sys.modules`` is this module's stand-in (not real ROS).

    ``install()`` only returns True on the call that registers the stubs, so use this for
    skip decisions that must hold for every test module.
    """
    return getattr(sys.modules.get('rclpy'), '__file__', None) is None


def install():
    """Register the stubs, but never shadow a real ROS installation.

    With a real ROS, ``rclpy.init()`` is called here because the tests build ``Node``
    instances directly without spinning a context of their own.
    """
    try:
        import rclpy  # noqa: F401
    except ImportError:
        pass
    else:
        if not rclpy.ok():
            import atexit
            rclpy.init(args=None)
            atexit.register(lambda: rclpy.shutdown() if rclpy.ok() else None)
        return False

    rclpy = _mod('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.ok = lambda: False

    node_module = _mod('rclpy.node')
    node_module.Node = Node

    qos_module = _mod('rclpy.qos')
    qos_module.QoSProfile = QoSProfile
    qos_module.QoSReliabilityPolicy = _Policy
    qos_module.QoSHistoryPolicy = _Policy

    _mod('geometry_msgs')
    _mod('geometry_msgs.msg').TwistStamped = TwistStamped
    _mod('geometry_msgs.msg').Twist = _Twist
    _mod('nav_msgs')
    _mod('nav_msgs.msg').Odometry = Odometry
    _mod('sensor_msgs')
    sensor_msgs = _mod('sensor_msgs.msg')
    sensor_msgs.BatteryState = BatteryState
    sensor_msgs.Imu = Imu

    class Bool:
        def __init__(self, data=False):
            self.data = data

    _mod('std_msgs')
    _mod('std_msgs.msg').Bool = Bool

    class Int16:
        def __init__(self, data=0):
            self.data = data

    _mod('std_msgs.msg').Int16 = Int16
    _mod('std_msgs.msg').String = Bool.__class__('String', (), {'__init__': Bool.__init__})

    class _Srv:
        class Request:
            pass

        class Response:
            def __init__(self):
                self.success = False
                self.message = ''

    _mod('std_srvs')
    std_srvs = _mod('std_srvs.srv')
    std_srvs.Empty = _Srv
    std_srvs.Trigger = _Srv
    return True
