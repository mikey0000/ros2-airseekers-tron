"""Airseekers Tron base_keys driver — reads the top-panel buttons (evdev) and
publishes ``mower_interfaces/MowerBaseButtonInfo`` on ``/mower_base/button_info``.

Reimplementation of the lost ``libbase_keys.so`` + ``base_keys_node``. The original
``mower_drivers::BaseKeys`` class opened a ``/dev/input`` evdev node, polled it in a
background thread, and surfaced ``input_event`` callbacks plus a ``Key`` queue
(``getPressedKey()`` returns 0xffffffff when empty). The node side reduced the raw
keycodes to the ``key_value`` bitmask defined by ``MowerBaseButtonInfo``:

    KEY_WORKING_OR_PAUSE = 1      (pause/resume mowing)
    KEY_GO_DOCKING      = 2       (go dock)
    KEY_POWER_SHORT     = 4       (power button, short press)
    KEY_POWER_LONG      = 8       (power button, long press)
    KEY_DOCK_AND_PAUSE  = 16      (dock + pause pressed together)

The physical keycode -> semantic mapping is hardware-specific and could not be fully
recovered from the stripped binary, so it is exposed as ROS parameters (see the
``declare_parameter`` calls) and must be confirmed on device with ``evtest``.
"""
import os
import select
import struct
import threading
import time

from rclpy.node import Node

from std_msgs.msg import Header
from mower_interfaces.msg import MowerBaseButtonInfo


# Linux input event (linux/input.h). 64-bit timeval layout.
INPUT_EVENT_FMT = 'llHHi'          # tv_sec, tv_usec, type, code, value
INPUT_EVENT_SIZE = struct.calcsize(INPUT_EVENT_FMT)

EV_SYN = 0
EV_KEY = 1
EV_KEY_PRESS = 1
EV_KEY_RELEASE = 0

# Default linux/input-event-codes.h keycodes (confirm with `evtest` on device).
DEFAULT_KEY_WORK_PAUSE = 119   # KEY_PAUSE
DEFAULT_KEY_GO_DOCKING = 102   # KEY_HOME
DEFAULT_KEY_POWER = 116        # KEY_POWER


class BaseKeys:
    """Blocking reader of a raw evdev ``/dev/input`` node, mirroring the recovered
    ``mower_drivers::BaseKeys`` API (open/read/poll in a loop). Callers provide an
    ``on_event`` callback invoked with each ``(type, code, value)``."""

    def __init__(self, device, on_event):
        self.device = device
        self.on_event = on_event
        self.fd = None
        self._running = False
        self._thread = None

    def open(self) -> bool:
        try:
            self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
            return True
        except OSError:
            self.fd = None
            return False

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def start_reading(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop_reading(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _read_loop(self):
        while self._running:
            if self.fd is None:
                time.sleep(0.05)
                continue
            ready, _, _ = select.select([self.fd], [], [], 1.0)
            if not ready:
                continue
            try:
                data = os.read(self.fd, INPUT_EVENT_SIZE)
            except OSError:
                break
            while len(data) >= INPUT_EVENT_SIZE:
                tv_sec, tv_usec, etype, code, value = struct.unpack(INPUT_EVENT_FMT, data[:INPUT_EVENT_SIZE])
                data = data[INPUT_EVENT_SIZE:]
                self.on_event(etype, code, value, tv_sec + tv_usec * 1e-6)


class BaseKeysNode(Node):
    def __init__(self):
        super().__init__('base_keys_node')
        self._pub = self.create_publisher(MowerBaseButtonInfo, '/mower_base/button_info', 10)

        self.declare_parameter('device', '/dev/input/event0')
        self.declare_parameter('key_work_pause', DEFAULT_KEY_WORK_PAUSE)
        self.declare_parameter('key_go_docking', DEFAULT_KEY_GO_DOCKING)
        self.declare_parameter('key_power', DEFAULT_KEY_POWER)
        self.declare_parameter('long_press_ms', 3000)

        self.device = self.get_parameter('device').value
        self.key_map = {
            self.get_parameter('key_work_pause').value: MowerBaseButtonInfo.KEY_WORKING_OR_PAUSE,
            self.get_parameter('key_go_docking').value: MowerBaseButtonInfo.KEY_GO_DOCKING,
            self.get_parameter('key_power').value: MowerBaseButtonInfo.KEY_POWER_SHORT,
        }
        self.long_press_s = self.get_parameter('long_press_ms').value / 1000.0

        self._power_key_code = self.get_parameter('key_power').value
        self._press_start = {}      # keycode -> press timestamp (monotonic)
        self._pressed_bits = 0      # current bitmask of held *semantic* buttons

        self.keys = BaseKeys(self.device, self._on_event)
        if not self.keys.open():
            self.get_logger().error(
                'Failed to open %s (is the device node correct? see `evtest`)', self.device)
            self.keys = None
            return
        self.keys.start_reading()
        self.get_logger().info(
            'base_keys up on %s; keycode mapping is DEFAULT and must be confirmed with evtest',
            self.device)

    def _on_event(self, etype, code, value, timestamp):
        if etype != EV_KEY:
            return
        if code not in self.key_map:
            return

        if value == EV_KEY_PRESS:
            self._press_start[code] = timestamp
            if code != self._power_key_code:
                self._pressed_bits |= self.key_map[code]
            self._publish()
        elif value == EV_KEY_RELEASE and code in self._press_start:
            held = timestamp - self._press_start.pop(code)
            if code == self._power_key_code:
                bit = (MowerBaseButtonInfo.KEY_POWER_LONG if held >= self.long_press_s
                       else MowerBaseButtonInfo.KEY_POWER_SHORT)
                # Power is transient: emit the short/long bit now, then clear it.
                self._publish_extra(bit)
                self._publish()  # clear the transient bit
            else:
                self._pressed_bits &= ~self.key_map[code]
                self._publish()

    def _publish_extra(self, bit):
        msg = MowerBaseButtonInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.key_value = bit
        self._pub.publish(msg)

    def _publish(self):
        msg = MowerBaseButtonInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.key_value = self._pressed_bits
        self._pub.publish(msg)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = BaseKeysNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
