"""fill_light_node: the vendor ``/fill_light_control`` on a host sysfs PWM, plus auto mode.

See fill_light.py for the vendor reverse-engineering. Interfaces:

Services
  /fill_light_control   std_srvs/SetBool   manual on (true) / off (false)
  /fill_light/set_auto  std_srvs/SetBool   true -> auto mode, false -> off
  /fill_light/get_state std_srvs/Trigger   message = JSON status
Topics
  /fill_light/state     std_msgs/Bool      read-back (latched); mcu_node copies it into
                                           /mower_sensor_info.is_fill_light_on
  /fill_light/status    std_msgs/String    JSON status (latched, on change + 1/10 Hz)
Subscribes
  /behavior_tree_node/high_level_status  (mowgli_interfaces/HighLevelStatus.state_name)

Auto (``fill_light_auto: true``): on while the mission is MOWING/TRANSIT and the sun is
below ``dark_below_deg`` at the datum (``datum_env_file`` or the latitude/longitude
params). No camera, no network.
"""

import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

try:
    from mowgli_interfaces.msg import HighLevelStatus
except ImportError:  # pragma: no cover
    HighLevelStatus = None

from mower_mcu_driver.fill_light import (DarknessDetector, PwmFillLight, VENDOR_CHIP,
                                         parse_datum_env, status_json, want_on)


class FillLightNode(Node):
    def __init__(self):
        super().__init__('fill_light')
        d = self.declare_parameter
        d('pwm_chip_path', VENDOR_CHIP)
        d('pwm_channel', 0)
        d('period_ns', 1000000)
        d('brightness', 60.0)              # % of period; 60 -> vendor's 600000 ns
        d('fill_light_auto', True)
        d('active_states', ['MOWING', 'TRANSIT'])
        d('dark_below_deg', -3.0)          # sun elevation; civil twilight is -6
        d('dark_hysteresis_deg', 1.0)
        d('datum_env_file', '/userdata/ros2/datum.env')
        d('latitude', 0.0)                 # used when datum_env_file is unreadable
        d('longitude', 0.0)
        d('refresh_s', 10.0)               # vendor re-writes every 10 s
        d('high_level_status_topic', '/behavior_tree_node/high_level_status')
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.pwm = PwmFillLight(g('pwm_chip_path'), g('pwm_channel'), g('period_ns'))
        self.brightness = float(g('brightness'))
        self.active_states = tuple(s.upper() for s in g('active_states'))
        self.dark = DarknessDetector(g('dark_below_deg'), g('dark_hysteresis_deg'))
        self.mode = 'auto' if g('fill_light_auto') else 'off'
        self.state_name = ''
        self.requested = False
        self.on = False
        self._last_write = 0.0
        self._last_status = None
        self._lat, self._lon = self._datum(g)

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Bool, '/fill_light/state', latched)
        self.status_pub = self.create_publisher(String, '/fill_light/status', latched)
        if HighLevelStatus is not None:
            self.create_subscription(HighLevelStatus, g('high_level_status_topic'),
                                     self._on_hl, 10)
        self.create_service(SetBool, '/fill_light_control', self._srv_control)
        self.create_service(SetBool, '/fill_light/set_auto', self._srv_auto)
        self.create_service(Trigger, '/fill_light/get_state', self._srv_get)
        self.create_timer(1.0, self._tick)
        self._refresh_s = float(g('refresh_s'))

        self.get_logger().info(
            'fill light on %s/pwm%d (%s), mode %s, datum %s'
            % (self.pwm.chip, self.pwm.channel,
               'present' if self.pwm.available() else 'ABSENT: no fill-light PWM on this '
               'device tree, commands will be refused', self.mode,
               '%.5f,%.5f' % (self._lat, self._lon) if self._lat is not None else 'unknown'))
        self._apply(force=True)

    def _datum(self, g):
        try:
            with open(g('datum_env_file')) as f:
                ll = parse_datum_env(f.read())
            if ll:
                return ll
        except OSError:
            pass
        lat, lon = float(g('latitude')), float(g('longitude'))
        return (lat, lon) if (lat or lon) else (None, None)

    # ---------------------------------------------------------------- logic
    def _on_hl(self, msg):
        name = str(msg.state_name)
        if name != self.state_name:
            self.state_name = name
            self._apply()

    def _tick(self):
        self._apply()

    def _apply(self, force=False):
        dark = self.dark.update(self._lat, self._lon, time.time())
        want = want_on(self.mode, self.state_name, dark, self.active_states)
        now = time.monotonic()
        if force or want != self.requested or now - self._last_write >= self._refresh_s:
            if want != self.requested:
                self.get_logger().info('fill light -> %s (mode %s, state %s, dark %s)'
                                       % ('ON' if want else 'off', self.mode,
                                          self.state_name or '?', dark))
            self.requested = want
            self._last_write = now
            if self.pwm.available() or want:
                self.on = self.pwm.set(want, self.brightness)
            else:
                self.on = False
                self.pwm.error = ''
        self._publish()

    def _status(self):
        return status_json(self.mode, self.on, self.requested, self.pwm.available(),
                           self.pwm.error, self.dark.dark, self.dark.elevation,
                           self.state_name, self.brightness, self.pwm.chip)

    def _publish(self):
        s = self._status()
        if s == self._last_status:
            return
        self._last_status = s
        self.state_pub.publish(Bool(data=bool(self.on)))
        self.status_pub.publish(String(data=s))

    # ------------------------------------------------------------- services
    def _result(self, resp):
        self._apply(force=True)
        resp.success = self.on == self.requested and not self.pwm.error
        resp.message = self._status()
        return resp

    def _srv_control(self, req, resp):
        self.get_logger().info('fill light control, data: %s' % ('on' if req.data else 'off'))
        self.mode = 'on' if req.data else 'off'
        return self._result(resp)

    def _srv_auto(self, req, resp):
        self.mode = 'auto' if req.data else 'off'
        return self._result(resp)

    def _srv_get(self, _req, resp):
        resp.success = True
        resp.message = self._status()
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = FillLightNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.pwm.set(False)  # never leave a light burning with no owner
        except Exception:  # pragma: no cover
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
