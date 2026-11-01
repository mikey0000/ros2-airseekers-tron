#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Wi-Fi link monitor (ROS 2 Jazzy): signal, bitrate and roams of ``iface`` (wlan0).

2026-11-01. The mower roams badly (bcmdhd firmware roaming under NetworkManager) and the
stack kept no record of the link. Everything here is read without root; parsing and roam
tracking live in ``wifi_logic.py`` (pure, unit tested). A missing ``iw`` or interface
publishes ``connected=false``, never crashes.

Published
---------
``/wifi/status`` mower_interfaces/WifiStatus every ``period_s`` (1 s).
``/wifi/event``  std_msgs/String JSON on connect / roam / disconnect / reconnect:
                 {event, iface, ssid, old:{bssid,freq_mhz,signal_dbm}, new:{...}, roam_count}.
"""

import json
import subprocess
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mower_interfaces.msg import WifiStatus

from mower_control import wifi_logic


def run_cmd(argv, timeout_s):
    """stdout of argv, or '' on a missing binary, timeout or non-zero exit."""
    try:
        res = subprocess.run(argv, capture_output=True, text=True, timeout=timeout_s,
                             check=False)
    except (OSError, subprocess.SubprocessError):
        return ''
    return res.stdout if res.returncode == 0 else ''


def read_file(path):
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            return f.read()
    except OSError:
        return ''


class WifiSampler:
    """Collects one sample from the system (injectable runners for tests)."""

    def __init__(self, iface, station_period_s=10.0, cmd_timeout_s=0.8,
                 run=run_cmd, read=read_file, clock=time.monotonic):
        self._iface = iface
        self._station_period_s = station_period_s
        self._cmd_timeout_s = cmd_timeout_s
        self._run = run
        self._read = read
        self._clock = clock
        self._station = None
        self._last_station = None

    def sample(self):
        iface = self._iface
        now = self._clock()
        if self._last_station is None or now - self._last_station >= self._station_period_s:
            self._last_station = now
            self._station = wifi_logic.parse_station_dump(
                self._run(['iw', 'dev', iface, 'station', 'dump'], self._cmd_timeout_s))
        return wifi_logic.build_sample(
            iface,
            self._read('/proc/net/wireless'),
            self._run(['iw', 'dev', iface, 'link'], self._cmd_timeout_s),
            self._read('/sys/class/net/%s/operstate' % iface),
            self._station)


def fill_status(msg, sample, roam_count):
    msg.iface = sample['iface']
    msg.connected = bool(sample['connected'])
    msg.ssid = sample['ssid']
    msg.bssid = sample['bssid']
    msg.freq_mhz = float(sample['freq_mhz'])
    msg.signal_dbm = float(sample['signal_dbm']) if sample['signal_dbm'] is not None \
        else float('nan')
    msg.tx_bitrate_mbps = float(sample['tx_bitrate_mbps'])
    msg.rx_bitrate_mbps = float(sample['rx_bitrate_mbps'])
    msg.tx_retries = int(sample['tx_retries'])
    msg.tx_failed = int(sample['tx_failed'])
    msg.roam_count = int(roam_count)
    msg.link_state = sample['link_state']
    return msg


def describe_event(ev):
    """One log line for an event."""
    o, n = ev['old'], ev['new']

    def side(s):
        if not s['bssid']:
            return '-'
        sig = '?' if s['signal_dbm'] is None else '%.0f dBm' % s['signal_dbm']
        return '%s %.0f MHz %s' % (s['bssid'], s['freq_mhz'], sig)
    return 'wifi %s on %s: %s -> %s (roams %d)' % (
        ev['event'], ev['iface'], side(o), side(n), ev['roam_count'])


class WifiMonitorNode(Node):

    def __init__(self):
        super().__init__('wifi_monitor')
        dp = self.declare_parameter
        dp('iface', 'wlan0')
        dp('period_s', 1.0)
        dp('station_period_s', 10.0)
        dp('cmd_timeout_s', 0.8)
        dp('status_topic', '/wifi/status')
        dp('event_topic', '/wifi/event')
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self._sampler = WifiSampler(gp('iface'), float(gp('station_period_s')),
                                    float(gp('cmd_timeout_s')))
        self._tracker = wifi_logic.RoamTracker()
        self._status_pub = self.create_publisher(WifiStatus, gp('status_topic'), 10)
        self._event_pub = self.create_publisher(String, gp('event_topic'), 10)
        self.create_timer(float(gp('period_s')), self._tick)

    def _tick(self):
        sample = self._sampler.sample()
        for ev in self._tracker.update(sample):
            text = describe_event(ev)
            if ev['event'] == 'disconnect':
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
            self._event_pub.publish(String(data=json.dumps(ev)))
        msg = fill_status(WifiStatus(), sample, self._tracker.roam_count)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = sample['iface']
        self._status_pub.publish(msg)


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('wifi_monitor')
    except ImportError:
        pass
    rclpy.init(args=args)
    node = WifiMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
