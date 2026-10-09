"""Airseekers Tron top-panel buttons: evdev reader + key -> action mapping.

Replaces the vendor ``mower_base::Buttons`` (``button.cpp``, publishes
``/mower_base/button_info``) and the key branch of the vendor
``TaskManager::sensorStateCallback`` (``mower_logic/src/task_manager.cpp``).
See ``docs/buttons.md`` and ``base_keys/keys_logic.py`` for the recovered tables.

Publishes
  /mower_base/button_info  mower_interfaces/MowerBaseButtonInfo  key bit on press, 0 on release
  /mower_base/key_pressed  std_msgs/UInt8                         same bitmask (for MowerSensorInfo.key_pressed)
Subscribes
  /behavior_tree_node/high_level_status  mowgli_interfaces/HighLevelStatus
Calls (each one can be disabled by parameter)
  /clear_estop                           std_srvs/Empty          power short
  /behavior_tree_node/high_level_control mowgli_interfaces/HighLevelControl
  /poweroff                              std_srvs/Empty          power long, only if power_long_action=poweroff_service
  /cutter_off                            std_srvs/Trigger        power long (sequence)
  /light_control                         mower_lights_interfaces/LightControl  power long (sequence)
Publishes (power_long_action=sequence)
  /estop_request                         std_msgs/Bool           true: mcu_node latches motion + blade off
Writes (power_long_action=sequence)
  <power_request_dir>/shutdown_request   JSON read by the host's mower-poweroff.path/.service
"""
import errno
import os
import queue
import select
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, UInt8
from std_srvs.srv import Empty, Trigger
from mower_interfaces.msg import MowerBaseButtonInfo
from mowgli_interfaces.msg import HighLevelStatus
from mowgli_interfaces.srv import HighLevelControl

from base_keys import keys_logic as kl
from base_keys import power_sequence as ps
from base_keys import poweroff_host as ph

try:
    from mower_lights_interfaces.msg import LightMode
    from mower_lights_interfaces.srv import LightControl
except ImportError:  # pragma: no cover - lights interfaces not built
    LightMode = LightControl = None

PROC_INPUT_DEVICES = '/proc/bus/input/devices'


class EvdevReader:
    """Background reader of one /dev/input/eventN node (raw struct input_event,
    no python-evdev needed). Pushes ('event', t, type, code, value) and
    ('closed', reason) tuples into ``out_q``. Mirrors vendor
    ``mower_drivers::BaseKeys`` (open O_RDONLY, poll 1000 ms, read 24 bytes)."""

    def __init__(self, path, out_q):
        self.path = path
        self.q = out_q
        self.fd = None
        self._run = False
        self._th = None

    def open(self):
        self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        self._run = True
        self._th = threading.Thread(target=self._loop, name='base_keys_reader', daemon=True)
        self._th.start()

    def close(self):
        self._run = False
        if self._th is not None and self._th is not threading.current_thread():
            self._th.join(timeout=1.5)
        self._th = None
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def _loop(self):
        pending = b''
        poller = select.poll()
        poller.register(self.fd, select.POLLIN | select.POLLERR | select.POLLHUP)
        while self._run:
            try:
                evs = poller.poll(500)
            except InterruptedError:
                continue
            if not evs:
                continue
            if any(m & (select.POLLERR | select.POLLHUP | select.POLLNVAL) for _, m in evs):
                self.q.put(('closed', 'poll error/hangup on %s' % self.path))
                return
            try:
                buf = os.read(self.fd, kl.INPUT_EVENT_SIZE * 64)
            except BlockingIOError:
                continue
            except OSError as e:
                self.q.put(('closed', '%s: %s' % (self.path, os.strerror(e.errno or errno.EIO))))
                return
            if not buf:
                self.q.put(('closed', 'EOF on %s' % self.path))
                return
            events, pending = kl.decode_events(pending + buf)
            for (t, etype, code, value) in events:
                if etype == kl.EV_KEY:
                    # monotonic stamp for timing (event timeval is wall clock)
                    self.q.put(('event', time.monotonic(), etype, code, value))


class BaseKeysNode(Node):
    def __init__(self):
        super().__init__('base_keys')
        dp = self.declare_parameter
        # device selection
        dp('device', '')                         # explicit /dev/input/eventN; '' = auto
        dp('device_name', kl.VENDOR_DEVICE_NAME)  # /proc/bus/input/devices N: Name=
        dp('fallback_device', kl.VENDOR_DEVICE_PATH)  # vendor path (udev symlink)
        dp('reopen_period_s', 2.0)
        # key codes (vendor values; see docs/buttons.md)
        dp('key_power', kl.VENDOR_KEY_POWER)
        dp('key_power_long', kl.VENDOR_KEY_POWER_LONG)
        dp('key_work_pause', kl.VENDOR_KEY_WORK_PAUSE)
        dp('key_go_docking', kl.VENDOR_KEY_GO_DOCKING)
        dp('key_dock_and_pause', kl.VENDOR_KEY_DOCK_AND_PAUSE)
        # timing
        dp('long_press_ms', 3000)   # software long press on key_power; 0 = only the device's KEY_L
        dp('debounce_ms', 50)
        # actions
        dp('actions_enabled', True)
        dp('on_power_short', True)
        dp('power_short_reset_emergency', True)
        dp('on_work_pause', True)
        dp('on_go_docking', True)
        dp('on_dock_and_pause', False)
        dp('power_long_action', 'sequence')       # sequence | log_only | poweroff_service | shutdown
        dp('shutdown_hook_file', '')               # power_long_action=shutdown: file to write (host watches it)
        # power_long_action=sequence (docs/buttons.md "Power off")
        dp('power_request_dir', ph.REQUEST_DIR)    # bind-mounted /userdata/ros2/power; host watches it
        dp('power_settle_s', 3.0)                  # stop -> request delay (vendor slept 3000 ms)
        dp('power_min_uptime_s', 30.0)             # ignore long presses this early after boot
        dp('cutter_off_service', '/cutter_off')
        dp('light_control_service', '/light_control')
        # names
        dp('clear_estop_service', '/clear_estop')
        dp('poweroff_service', '/poweroff')
        dp('high_level_control_service', '/behavior_tree_node/high_level_control')
        dp('high_level_status_topic', '/behavior_tree_node/high_level_status')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        code_map = {
            int(g('key_power')): kl.KEY_POWER_SHORT,
            int(g('key_power_long')): kl.KEY_POWER_LONG,
            int(g('key_work_pause')): kl.KEY_WORKING_OR_PAUSE,
            int(g('key_go_docking')): kl.KEY_GO_DOCKING,
            int(g('key_dock_and_pause')): kl.KEY_DOCK_AND_PAUSE,
        }
        self._required_codes = (int(g('key_power')), int(g('key_work_pause')), int(g('key_go_docking')))
        self.sm = kl.KeyStateMachine(code_map, power_code=int(g('key_power')),
                                     long_press_s=g('long_press_ms') / 1000.0,
                                     debounce_s=g('debounce_ms') / 1000.0)
        pla = g('power_long_action')
        if pla not in kl.POWER_LONG_ACTIONS:
            self.get_logger().warn('power_long_action=%r unknown, using log_only' % pla)
            pla = 'log_only'
        self.cfg = kl.ActionConfig(
            on_power_short=g('on_power_short'),
            power_short_reset_emergency=g('power_short_reset_emergency'),
            on_work_pause=g('on_work_pause'),
            on_go_docking=g('on_go_docking'),
            on_dock_and_pause=g('on_dock_and_pause'),
            power_long_action=pla)
        self.actions_enabled = g('actions_enabled')
        self.shutdown_hook_file = g('shutdown_hook_file')

        self.pub_info = self.create_publisher(MowerBaseButtonInfo, '/mower_base/button_info', 10)
        self.pub_key = self.create_publisher(UInt8, '/mower_base/key_pressed', 10)
        self.hl_state = None
        self.create_subscription(HighLevelStatus, g('high_level_status_topic'), self._on_hl, 10)
        self.cli_clear = self.create_client(Empty, g('clear_estop_service'))
        self.cli_poweroff = self.create_client(Empty, g('poweroff_service'))
        self.cli_hlc = self.create_client(HighLevelControl, g('high_level_control_service'))
        self.power_request_dir = g('power_request_dir')
        self.power_seq = ps.PowerOffSequencer(settle_s=g('power_settle_s'),
                                              min_uptime_s=g('power_min_uptime_s'))
        self.pub_estop = self.create_publisher(Bool, '/estop_request', 10)
        self.cli_cutter_off = self.create_client(Trigger, g('cutter_off_service'))
        self.cli_light = (self.create_client(LightControl, g('light_control_service'))
                          if LightControl is not None else None)

        self.q = queue.Queue()
        self.reader = None
        self._last_open_try = -1e9
        self._last_fail_reason = None
        self.reopen_period = float(g('reopen_period_s'))
        self._try_open()
        # Key handling runs on a plain thread that blocks on the reader queue instead of a
        # 50 Hz rclpy timer (50 executor wakes/s cost ~10 % of a core with no key pressed).
        # It still ticks every 20 ms while a key is held (power long-press) and retries a
        # missing device every reopen_period_s; events are handled as soon as they arrive.
        self._stop = threading.Event()
        self._loop_th = threading.Thread(target=self._key_loop, name='base_keys_loop',
                                         daemon=True)
        self._loop_th.start()

    # ---------------------------------------------------------------- device
    def _resolve_device(self):
        explicit = self.get_parameter('device').value
        if explicit:
            return explicit, 'parameter device'
        try:
            with open(PROC_INPUT_DEVICES) as f:
                text = f.read()
        except OSError as e:
            text = ''
            self.get_logger().warn('cannot read %s: %s' % (PROC_INPUT_DEVICES, e))
        path, why = kl.find_device(text, self.get_parameter('device_name').value, ())
        if path:
            return path, why
        fb = self.get_parameter('fallback_device').value
        if fb and os.path.exists(fb):
            return os.path.realpath(fb), 'fallback %s' % fb
        return kl.find_device(text, '', self._required_codes)

    def _try_open(self):
        self._last_open_try = time.monotonic()
        path, why = self._resolve_device()
        if not path:
            self._log_fail(why)
            return
        r = EvdevReader(path, self.q)
        try:
            r.open()
        except OSError as e:
            self._log_fail('open %s (%s) failed: %s' % (path, why, e.strerror))
            return
        self.reader = r
        self._last_fail_reason = None
        self.get_logger().info('opened %s via %s' % (path, why))

    def _log_fail(self, reason):
        if reason != self._last_fail_reason:     # don't spam every retry
            self.get_logger().error('button device unavailable: %s (retrying every %.1fs)'
                                    % (reason, self.reopen_period))
            self._last_fail_reason = reason

    # ---------------------------------------------------------------- loop
    def _key_loop(self):
        ctx = self.context
        while not self._stop.is_set() and ctx.ok():
            if self.sm.held_bits:
                timeout = 0.02                       # long-press tick while a key is held
            elif self.power_seq.busy:
                timeout = 0.1                        # power-off settle timer
            elif self.reader is None:
                timeout = max(0.0, self.reopen_period
                              - (time.monotonic() - self._last_open_try))
            else:
                timeout = 0.5                        # only to notice shutdown
            try:
                first = self.q.get(timeout=timeout)
            except queue.Empty:
                first = None
            if self._stop.is_set() or not ctx.ok():
                break
            try:
                self._spin_once(first)
            except Exception as exc:  # noqa: BLE001 - keep the buttons alive
                self.get_logger().error('key handling failed: %r' % (exc,),
                                        throttle_duration_sec=5.0)

    def _spin_once(self, first=None):
        now = time.monotonic()
        while True:
            if first is not None:
                item, first = first, None
            else:
                try:
                    item = self.q.get_nowait()
                except queue.Empty:
                    break
            if item[0] == 'closed':
                self.get_logger().warn('button device lost: %s; will reopen' % item[1])
                if self.reader is not None:
                    self.reader.close()
                    self.reader = None
                self.sm.reset()
                self._publish(0)
                continue
            _, t, etype, code, value = item
            held_before = self.sm.held_bits
            bits = self.sm.feed(etype, code, value, t)
            if self.sm.unknown_codes:
                for c in self.sm.unknown_codes:
                    if c == kl.VENDOR_KEY_POWER_LONG_RELEASE:
                        self.get_logger().info('power long press released (KEY_R; the kernel '
                                               'module now runs /usr/bin/sys_close)')
                    else:
                        self.get_logger().info('unknown key code %d (no mapping)' % c)
                self.sm.unknown_codes.clear()
            for b in bits:
                self._emit(b)
            if value == kl.KEY_RELEASE and held_before and not self.sm.held_bits:
                self._publish(0)      # vendor: release -> key 0
        for b in self.sm.tick(now):
            self._emit(b)
        for step in self.power_seq.tick(now):
            self._power_step(step)
        if self.reader is None and now - self._last_open_try >= self.reopen_period:
            self._try_open()

    def _publish(self, bits):
        m = MowerBaseButtonInfo()
        m.header.stamp = self.get_clock().now().to_msg()
        m.key_value = bits
        self.pub_info.publish(m)
        self.pub_key.publish(UInt8(data=bits))

    def _emit(self, bit):
        self.get_logger().info('key %s (%d) triggered' % (kl.BIT_NAMES.get(bit, '?'), bit))
        self._publish(bit)
        if not self.actions_enabled:
            return
        for a in kl.map_key(bit, self.hl_state, self.cfg):
            self._do(a)

    # ---------------------------------------------------------------- actions
    def _on_hl(self, msg):
        self.hl_state = msg.state

    def _call(self, cli, req, what):
        if not cli.service_is_ready():
            self.get_logger().warn('%s: service %s not available, skipped' % (what, cli.srv_name))
            return
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f: self._done(f, what))

    def _done(self, fut, what):
        exc = fut.exception()
        if exc is not None:
            self.get_logger().error('%s failed: %s' % (what, exc))
            return
        res = fut.result()
        ok = getattr(res, 'success', True)
        (self.get_logger().info if ok else self.get_logger().warn)(
            '%s -> %s' % (what, 'ok' if ok else 'rejected'))

    def _do(self, a):
        log = self.get_logger()
        if a.kind == 'log':
            log.info(a.note)
        elif a.kind == 'clear_estop':
            log.info(a.note)
            self._call(self.cli_clear, Empty.Request(), 'clear_estop')
        elif a.kind == 'hlc':
            log.info(a.note)
            self._call(self.cli_hlc, HighLevelControl.Request(command=a.arg),
                       'high_level_control(%d)' % a.arg)
        elif a.kind == 'power_sequence':
            log.warn(a.note)
            steps = self.power_seq.on_power_long(time.monotonic(), ph.read_uptime(),
                                                 os.path.isdir(self.power_request_dir))
            for step in steps:
                self._power_step(step)
        elif a.kind == 'poweroff_service':
            log.warn(a.note)
            self._call(self.cli_poweroff, Empty.Request(), 'poweroff')
        elif a.kind == 'shutdown_hook':
            if not self.shutdown_hook_file:
                log.warn('power_long_action=shutdown but shutdown_hook_file is empty: '
                         'shutdown from inside the container is not implemented (see docs/buttons.md)')
                return
            try:
                with open(self.shutdown_hook_file, 'w') as f:
                    f.write('%f\n' % time.time())
                log.warn('%s: wrote %s' % (a.note, self.shutdown_hook_file))
            except OSError as e:
                log.error('shutdown hook write failed: %s' % e)

    def _power_step(self, step):
        log = self.get_logger()
        if step.kind == 'log':
            log.warn(step.note)
        elif step.kind == 'error':
            log.error(step.note)
        elif step.kind == 'estop':
            log.warn(step.note)
            self.pub_estop.publish(Bool(data=True))
        elif step.kind == 'hlc_stop':
            log.warn(step.note)
            self._call(self.cli_hlc, HighLevelControl.Request(command=kl.CMD_STOP),
                       'high_level_control(STOP)')
        elif step.kind == 'cutter_off':
            log.warn(step.note)
            self._call(self.cli_cutter_off, Trigger.Request(), 'cutter_off')
        elif step.kind == 'light_poweroff':
            if self.cli_light is not None:
                self._call(self.cli_light,
                           LightControl.Request(mode=LightMode(light_mode=LightMode.POWER_OFF)),
                           'light_control(PowerOff)')
        elif step.kind == 'write_request':
            try:
                path = ph.write_request(ph.make_request(
                    'base_keys', ph.read_boot_id(), ph.read_uptime(),
                    hl_state=self.hl_state), self.power_request_dir)
                log.warn('%s: wrote %s' % (step.note, path))
            except OSError as e:
                log.error('power off request write failed: %s' % e)
                self.power_seq.request_failed()

    def destroy_node(self):
        self._stop.set()
        if self._loop_th is not threading.current_thread():
            self._loop_th.join(2.0)
        if self.reader is not None:
            self.reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseKeysNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:          # SIGTERM: context shut down under spin (RCLError / ExternalShutdownException)
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
