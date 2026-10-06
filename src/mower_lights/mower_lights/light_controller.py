"""light_controller: ROS 2 port of the vendor status-LED manager (mower_light_sound).

Serves   /light_control       mower_lights_interfaces/srv/LightControl  (vendor-shaped)
Pubs     /light_info          std_msgs/Int32   current LightMode (vendor topic)
         ~/state              std_msgs/String  JSON readback (mode, per-area colours,
                                               frames sent, kernel SPI counters)
Subs     /behavior_tree_node/high_level_status  mowgli_interfaces/HighLevelStatus
         /hardware_bridge/emergency             mowgli_interfaces/Emergency
         /mower_base/status                     mower_interfaces/MowerBaseDevStatus
         /battery                               sensor_msgs/BatteryState (fallback %)

Hardware: WS2812 chain on /dev/spidev3.0 (see docs/lights.md). Never commands motion
or the cutter.

CPU (RK3588): the inputs are only consumed by the 5 Hz auto tick, so they are *sampled*
subscriptions of a :class:`sub_pump.SubscriptionPump` (depth 1, newest message taken when
the tick runs; /mower_base/status alone is 100 Hz and cost one executor wake per message
before). The auto tick and the state publisher run on a plain ``PeriodicRunner`` thread;
the executor only serves /light_control and the parameter services.
"""
import json
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32, String

from mower_lights_interfaces.srv import LightControl

from . import led_hw
from . import lights_logic as ll
from .sub_pump import PeriodicRunner, SubscriptionPump, flat_parser

try:
    from mowgli_interfaces.msg import Emergency, HighLevelStatus
except ImportError:  # pragma: no cover
    Emergency = HighLevelStatus = None
try:
    from mower_interfaces.msg import MowerBaseDevStatus
except ImportError:  # pragma: no cover
    MowerBaseDevStatus = None
try:
    from sensor_msgs.msg import BatteryState
except ImportError:  # pragma: no cover
    BatteryState = None

DEFAULTS = {
    'auto_mode': True,
    'dry_run': False,
    'spi_device': '/dev/spidev3.0',
    'spi_speed_hz': ll.SPI_SPEED_HZ,
    'spi_mode': ll.SPI_MODE,
    'spi_bits_per_word': ll.SPI_BITS,
    'spi_stats_dir': '/sys/class/spi_master/spi3/statistics',
    'sn_path': '/meta/sn',
    'front_led_num': 0,            # 0 = from SN like the vendor (fallback 14)
    'top_led_num': ll.TOP_LED_NUM,
    'buffer_leds': ll.BUFFER_LEDS,
    'brightness': 100,             # vendor config SetLightBrightness default
    'frame_rate_hz': 25.0,
    'lane_rate_hz': 20.0,
    'startup_mode': ll.POWER_ON,
    'shutdown_mode': ll.POWER_OFF,
    'shutdown_anim_s': 2.0,
    'high_level_status_topic': '/behavior_tree_node/high_level_status',
    'emergency_topic': '/hardware_bridge/emergency',
    'base_status_topic': '/mower_base/status',
    'battery_topic': '/battery',
    'light_info_topic': '/light_info',
    'service_name': '/light_control',
    'state_rate_hz': 2.0,
    'low_battery_percent': 20.0,
    'charge_full_percent': 100.0,
    'warn_hold_s': 2.0,
    'status_timeout_s': 5.0,
    'auto_period_s': 0.2,
    'keepalive_s': 1.0,            # re-send an unchanged frame this often (0 = never)
}


class LightController(Node):
    def __init__(self):
        super().__init__('light_controller')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.g = g

        front = int(g('front_led_num'))
        sn = None
        if front <= 0:
            try:
                with open(g('sn_path')) as f:
                    sn = f.readline().strip()
            except OSError:
                sn = None
            front = ll.front_led_num_from_sn(sn)
        self.get_logger().info('sn %s front led num %d, top %d' % (sn, front, int(g('top_led_num'))))
        pixels = ll.Pixels(front, int(g('top_led_num')), int(g('brightness')), int(g('buffer_leds')))

        if g('dry_run'):
            backend = led_hw.DryRunBackend()
        else:
            backend = led_hw.SpiBackend(g('spi_device'), int(g('spi_speed_hz')),
                                        int(g('spi_mode')), int(g('spi_bits_per_word')))
        self._err_logged = 0.0
        self.engine = led_hw.LightEngine(backend, pixels, int(g('startup_mode')),
                                         float(g('frame_rate_hz')), float(g('lane_rate_hz')),
                                         on_error=self._on_hw_error,
                                         keepalive_s=float(g('keepalive_s')))
        try:
            self.engine.start()
        except OSError as e:
            self.get_logger().error('cannot open %s: %s (falling back to dry run)'
                                    % (g('spi_device'), e))
            self.engine.backend = led_hw.DryRunBackend()
            self.engine.start()
        self.get_logger().info('light manager start (%s, dry_run=%s)'
                               % (g('spi_device'), bool(g('dry_run'))))

        self.auto = ll.AutoMode(ll.MappingParams(
            low_battery_percent=float(g('low_battery_percent')),
            charge_full_percent=float(g('charge_full_percent')),
            warn_hold_s=float(g('warn_hold_s')),
            status_timeout_s=float(g('status_timeout_s'))))
        self._startup_done = False
        self._last_mode_logged = None

        self.info_pub = self.create_publisher(Int32, g('light_info_topic'), 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.srv = self.create_service(LightControl, g('service_name'), self._on_service)

        # Inputs: latest-value state read by the auto tick -> sampled (newest message only).
        self._lock = threading.Lock()          # auto tick vs. service handler
        self._pump = SubscriptionPump(self, 'lights_inputs') if SubscriptionPump.available() \
            else None
        self._base_sub = None

        def sub(msg_type, topic, cb, flat=False, best_effort=False):
            if self._pump is None:
                return self.create_subscription(msg_type, topic, cb, QoSProfile(depth=10))
            qos = QoSProfile(depth=1)
            if best_effort:     # no per-message ACKNACK traffic for a 100 Hz sampled input
                qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
            return self._pump.subscribe(msg_type, topic, cb, qos, sampled=True,
                                        parser=flat_parser(msg_type) if flat else None)

        if HighLevelStatus is not None:
            sub(HighLevelStatus, g('high_level_status_topic'), self._on_hl)
            sub(Emergency, g('emergency_topic'), self._on_emergency)
        if MowerBaseDevStatus is not None:
            self._base_sub = sub(MowerBaseDevStatus, g('base_status_topic'), self._on_base,
                                 flat=True, best_effort=True)
        if BatteryState is not None:
            sub(BatteryState, g('battery_topic'), self._on_battery)

        self._periodic = PeriodicRunner(self, [
            (float(g('auto_period_s')), self._auto_tick),
            (1.0 / max(0.1, float(g('state_rate_hz'))), self._publish_state),
        ], 'lights_periodic')
        self._periodic.start()

    # ------------------------------------------------------------------ inputs
    def _on_hl(self, m):
        i = self.auto.inputs
        i.hl_state, i.hl_state_name = int(m.state), str(m.state_name)
        i.hl_emergency, i.hl_is_charging = bool(m.emergency), bool(m.is_charging)
        if m.battery_percent > 0.0:
            i.battery_percent = float(m.battery_percent)
        i.hl_time = time.monotonic()

    def _on_emergency(self, m):
        self.auto.inputs.estop_active = bool(m.active_emergency or m.latched_emergency
                                             or getattr(m, 'lift_warning', False))

    def _on_base(self, m):
        i = self.auto.inputs
        i.stop, i.lift = bool(m.stop_triggered), bool(m.lift_triggered)
        i.bumper = bool(m.bumper_triggered)
        i.base_charging = bool(m.is_charging)
        self.engine.set_cutter_running(bool(m.is_cutting))   # vendor statusCallback

    def _on_battery(self, m):
        if self.auto.inputs.battery_percent is None and m.percentage == m.percentage:
            pct = float(m.percentage)
            self.auto.inputs.battery_percent = pct * 100.0 if pct <= 1.0 else pct

    # ------------------------------------------------------------------ control
    def _apply(self, mode, source):
        irq = self.engine.request(mode)
        self.get_logger().info('setLightMode %d (%s) <- %s%s' % (
            mode, ll.mode_name(mode), source, ' [preempt]' if irq.top else ''))
        self.info_pub.publish(Int32(data=int(self.engine.mode)))

    def _on_service(self, req, resp):
        mode = int(req.mode.light_mode)
        self.get_logger().info('service recv: %d' % mode)
        with self._lock:
            if self._pump is not None and self._base_sub is not None:
                self._pump.poll((self._base_sub,))   # fresh is_cutting for the cutter gate
            self._apply(mode, 'service')
        return resp

    def _auto_tick(self):
        with self._lock:
            if self._pump is not None:
                self._pump.poll()
            self._auto_tick_locked()

    def _auto_tick_locked(self):
        if not self._startup_done:
            # let the PowerOn animation finish (it hands over to Idle itself)
            if self.engine.mode == ll.POWER_ON:
                return
            self._startup_done = True
        if not self.g('auto_mode'):
            return
        mode = self.auto.evaluate(time.monotonic())
        if mode is not None and mode != self.engine.mode:
            self._apply(mode, 'auto(%s)' % (self.auto.inputs.hl_state_name or 'no status'))

    def _on_hw_error(self, e):
        now = time.monotonic()
        if now - self._err_logged > 10.0:
            self._err_logged = now
            self.get_logger().error('LED write failed: %s' % e)

    def readback(self):
        eng = self.engine
        px = eng.pixels
        with eng.pixel_lock:
            top = ll.summarize_area(px, ll.AREA_TOP)
            tail = ll.summarize_area(px, ll.AREA_TAIL)
            buf = px.snapshot()
        last = getattr(eng.backend, 'last_frame', None)
        wire_ok = last is not None and ll.decode_frame(last, px.n) == buf
        return {
            'mode': int(eng.mode), 'mode_name': ll.mode_name(eng.mode),
            'top': top, 'tail': tail, 'buffer_hex': buf[:3 * (2 * px.front + px.top)].hex(),
            'frames_sent': eng.frames_sent, 'last_frame_matches_buffer': bool(wire_ok),
            'backend': type(eng.backend).__name__, 'last_error': eng.last_error,
            'spi_stats': led_hw.read_spi_stats(self.g('spi_stats_dir')),
            'auto_mode': bool(self.g('auto_mode')),
            'stack_state': self.auto.inputs.hl_state_name,
        }

    def _publish_state(self):
        st = self.readback()
        self.state_pub.publish(String(data=json.dumps(st)))
        self.info_pub.publish(Int32(data=st['mode']))
        if st['mode'] != self._last_mode_logged:
            self._last_mode_logged = st['mode']
            self.get_logger().info('current light mode %d %s | top: %s | tail: %s'
                                   % (st['mode'], st['mode_name'], st['top'], st['tail']))

    def shutdown_lights(self):
        self._periodic.stop()
        if self._pump is not None:
            self._pump.stop()
        mode = int(self.g('shutdown_mode'))
        try:
            self.engine.play_blocking(mode, float(self.g('shutdown_anim_s')))
            st = self.readback()
            print('[light_controller] shutdown: mode %d %s | top: %s | tail: %s | frames %d'
                  % (st['mode'], st['mode_name'], st['top'], st['tail'], st['frames_sent']),
                  flush=True)
        finally:
            self.engine.stop()


def main(args=None):
    rclpy.init(args=args)
    node = LightController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown_lights()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
