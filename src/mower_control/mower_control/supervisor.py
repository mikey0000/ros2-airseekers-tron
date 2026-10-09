# SPDX-License-Identifier: GPL-3.0-or-later
"""Supervisor: node liveness + black-box recorder for the Tron stack (ROS 2 Jazzy).

Recovery itself is launch's job (``respawn=True`` on every node, Nav2 servers re-activated
by ``nav2_lifecycle_manager`` with ``attempt_respawn_reconnection``). This node makes
crashes *visible* and *recorded*:

Liveness (``supervisor_logic.LivenessTracker``)
    polls the ROS graph at ``poll_hz`` (``get_node_names_and_namespaces``). A node that was
    up and vanished is DOWN until it reappears (respawn -> restart counted).
    Publishes ``~/status`` (std_msgs/String JSON, latched, on change + every 5 s)::

        {"ok": bool, "down": [...], "critical_down": [...], "restarts": {...}, "nodes": {...}}

    and ``/diagnostics`` entries ``supervisor: <node>`` (ERROR while down, OK once back),
    which the MowgliNext GUI raises as alerts (it merges /diagnostics by entry name).
    The mission subscribes ``~/status`` and goes to EMERGENCY (safe stop) when a node in
    ``critical_nodes`` is down.

Black box (``blackbox.SnapshotBuffer``)
    keeps the last ``window_s`` seconds of ``blackbox_topics`` (``topic|type|max_hz``) as
    serialized bytes and dumps them as a rosbag2 to ``<crash_dir>/<ts>_<reason>/`` on:
    a node going DOWN, an emergency rising edge (``/hardware_bridge/emergency``), a lethal
    boundary violation, a mission "docking failed"/"undock failed"/boundary stop log line,
    or the ``~/dump`` service (std_srvs/Trigger). Dumps are rate limited (``dump_cooldown_s``).

Every dump dir: ``bag/`` (``ros2 bag play``), ``reason.json`` (trigger, supervisor status,
buffer stats, deployed revision), ``launch_log_tail.txt`` (last 500 lines of the current
launch log) and copies of crash records written in the last 10 minutes.
"""

import glob
import json
import os
import shutil
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import Log
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from mower_control import crash_record
from mower_control.blackbox import SnapshotBuffer, dump_bag, make_dump_dir
from mower_control.sub_pump import SubscriptionPump
from mower_control.supervisor_logic import DEFAULT_IGNORE, LivenessTracker

DEFAULT_TOPICS = [
    '/cmd_vel|geometry_msgs/msg/Twist|20',
    '/cmd_vel_raw|geometry_msgs/msg/Twist|20',
    '/cmd_vel_nav|geometry_msgs/msg/Twist|10',
    '/cmd_vel_teleop|geometry_msgs/msg/Twist|10',
    '/cmd_vel_docking|geometry_msgs/msg/Twist|10',
    '/cmd_vel_bumper|geometry_msgs/msg/Twist|10',
    '/cmd_vel_emergency|geometry_msgs/msg/Twist|10',
    '/odometry/filtered_map|nav_msgs/msg/Odometry|5',
    '/mower_base/status|mower_interfaces/msg/MowerBaseDevStatus|2',
    '/behavior_tree_node/high_level_status|mowgli_interfaces/msg/HighLevelStatus|2',
    '/hardware_bridge/emergency|mowgli_interfaces/msg/Emergency|2',
    '/obstacle_policy|std_msgs/msg/String|5',
    '/fix_status|std_msgs/msg/String|1',
    '/motion_enabled|std_msgs/msg/Bool|2',
    '/cmd_vel_slew/gate_status|std_msgs/msg/String|5',
    '/map_server_node/lethal_boundary_violation|std_msgs/msg/Bool|2',
    '/diagnostics|diagnostic_msgs/msg/DiagnosticArray|0',
    '/rosout|rcl_interfaces/msg/Log|0',
]
DEFAULT_CRITICAL = ['/mower_mcu_driver', '/cmd_vel_slew', '/twist_mux', '/behavior_tree_node']
# mission log lines (from /rosout) that trigger a dump
DUMP_LOG_PATTERNS = ('docking failed', 'undock failed', 'BOUNDARY_EMERGENCY_STOP')
EMERGENCY_TOPIC = '/hardware_bridge/emergency'
LETHAL_TOPIC = '/map_server_node/lethal_boundary_violation'


def _import_type(type_name):
    pkg, kind, name = type_name.split('/')
    module = __import__('%s.%s' % (pkg, kind), fromlist=[name])
    return getattr(module, name)


def launch_log_dir():
    base = os.environ.get('ROS_LOG_DIR') or os.path.join(
        os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')), 'log')
    latest = os.path.join(base, 'latest')
    return latest if os.path.isdir(latest) else None


def tail_lines(path, n=500, max_bytes=2 * 1024 * 1024):
    try:
        with open(path, 'rb') as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - max_bytes))
            data = f.read()
    except OSError:
        return ''
    return b'\n'.join(data.splitlines()[-n:]).decode('utf-8', 'replace') + '\n'


class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        dp = self.declare_parameter
        dp('poll_hz', 1.0)
        dp('critical_nodes', DEFAULT_CRITICAL)
        dp('ignore_nodes', list(DEFAULT_IGNORE))
        dp('adopt_after_s', 10.0)
        dp('down_after_s', 2.5)
        dp('startup_grace_s', 90.0)
        dp('crash_dir', crash_record.crash_dir())
        dp('window_s', 120.0)
        dp('max_buffer_mb', 24.0)
        dp('blackbox_topics', DEFAULT_TOPICS)
        dp('dump_cooldown_s', 30.0)
        dp('max_dumps', 50)               # oldest dump dirs beyond this are deleted
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self._crash_dir = str(gp('crash_dir'))
        self._cooldown = float(gp('dump_cooldown_s'))
        self._max_dumps = int(gp('max_dumps'))
        self._critical = list(gp('critical_nodes'))
        self._lock = threading.Lock()
        self._tracker = LivenessTracker(
            critical=self._critical, ignore=list(gp('ignore_nodes')),
            adopt_after_s=gp('adopt_after_s'), down_after_s=gp('down_after_s'),
            startup_grace_s=gp('startup_grace_s'), now=time.monotonic())
        self._buffer = SnapshotBuffer(float(gp('window_s')),
                                      int(float(gp('max_buffer_mb')) * 1024 * 1024))
        self._last_dump = None
        self._dumping = False
        self._emergency_active = False
        self._lethal = False
        self._ever_down = set()
        self._status_json = None
        self._status_pub_t = 0.0

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self._status_pub = self.create_publisher(String, '~/status', latched)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_service(Trigger, '~/dump', self._srv_dump)

        best_effort = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._pump = SubscriptionPump(self, 'supervisor_blackbox')
        for spec in gp('blackbox_topics'):
            try:
                topic, type_name, rate = (spec.split('|') + ['0'])[:3]
                msg_type = _import_type(type_name)
            except (ValueError, ImportError, AttributeError) as exc:
                self.get_logger().warn('blackbox: skipping %r (%s)' % (spec, exc))
                continue
            self._buffer.add_topic(topic, type_name, float(rate or 0))
            self._pump.subscribe(msg_type, topic, self._make_cb(topic, msg_type), best_effort,
                                 raw=True)
        self._pump.start()

        self.create_timer(1.0 / max(0.2, float(gp('poll_hz'))), self._poll)
        self.get_logger().info(
            'supervisor: critical=%s, black box %d topics / %.0f s -> %s'
            % (self._critical, len(self._buffer.topics()), float(gp('window_s')),
               self._crash_dir))

    # ---------------------------------------------------------------- black box input
    def _make_cb(self, topic, msg_type):
        def cb(data):
            with self._lock:
                stored = self._buffer.push(topic, time.time_ns(), data)
            if topic in (EMERGENCY_TOPIC, LETHAL_TOPIC, '/rosout'):
                self._check_trigger(topic, msg_type, data)
            return stored
        return cb

    def _check_trigger(self, topic, msg_type, data):
        try:
            msg = deserialize_message(bytes(data), msg_type)
        except Exception:  # noqa: BLE001
            return
        if topic == EMERGENCY_TOPIC:
            active = bool(msg.active_emergency)
            if active and not self._emergency_active:
                self.request_dump('emergency', getattr(msg, 'reason', ''))
            self._emergency_active = active
        elif topic == LETHAL_TOPIC:
            if msg.data and not self._lethal:
                self.request_dump('lethal_boundary', '')
            self._lethal = bool(msg.data)
        elif topic == '/rosout' and msg.name == 'behavior_tree_node':
            for pat in DUMP_LOG_PATTERNS:
                if pat in msg.msg:
                    self.request_dump(pat.replace(' ', '_'), msg.msg)
                    break

    # ---------------------------------------------------------------- liveness
    def _poll(self):
        now = time.monotonic()
        try:
            names = ['%s/%s' % ('' if ns == '/' else ns, n)
                     for n, ns in self.get_node_names_and_namespaces()]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('graph query failed: %s' % exc)
            return
        events = self._tracker.update(now, names)
        for ev in events:
            if ev[0] == 'down':
                self._ever_down.add(ev[1])
                critical = ev[1] in self._critical
                (self.get_logger().error if critical else self.get_logger().warn)(
                    'node %s DOWN (%s)%s' % (ev[1], ev[2], ' [critical]' if critical else ''))
                self._event_record({'event': 'down', 'node': ev[1], 'reason': ev[2],
                                    'critical': critical})
                self.request_dump('node_down_%s' % ev[1].strip('/').replace('/', '_'), ev[2])
            else:
                self.get_logger().info('node %s back UP after %.1f s (respawned)'
                                       % (ev[1], ev[2]))
                self._event_record({'event': 'up', 'node': ev[1], 'downtime_s': round(ev[2], 1)})
        st = self._tracker.status(now)
        text = json.dumps(st, sort_keys=True)
        if text != self._status_json or now - self._status_pub_t >= 5.0:
            self._status_json, self._status_pub_t = text, now
            self._status_pub.publish(String(data=text))
        self._publish_diagnostics(st)

    def _publish_diagnostics(self, st):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        summary = DiagnosticStatus(name='supervisor: nodes', hardware_id='supervisor')
        if st['critical_down']:
            summary.level = DiagnosticStatus.ERROR
            summary.message = 'critical node down: ' + ', '.join(st['critical_down'])
        elif st['down']:
            summary.level = DiagnosticStatus.ERROR
            summary.message = 'node down: ' + ', '.join(st['down'])
        else:
            summary.level = DiagnosticStatus.OK
            summary.message = '%d nodes alive' % len(st['nodes'])
        summary.values = [KeyValue(key='restarts', value=json.dumps(st['restarts']))]
        arr.status.append(summary)
        for name in sorted(self._ever_down):
            info = st['nodes'].get(name)
            if info is None:
                continue
            s = DiagnosticStatus(name='supervisor: %s' % name, hardware_id=name)
            if not info['alive']:
                s.level = DiagnosticStatus.ERROR
                s.message = 'DOWN %s s (%s)' % (info['down_for_s'], info['reason'])
            else:
                s.level = DiagnosticStatus.OK
                s.message = 'running (restarted %d x)' % info['restarts']
            arr.status.append(s)
        self._diag_pub.publish(arr)

    def _event_record(self, rec):
        rec = dict(rec, time=time.strftime('%Y-%m-%dT%H:%M:%S'), source='supervisor')
        try:
            os.makedirs(self._crash_dir, exist_ok=True)
            with open(os.path.join(self._crash_dir, 'events.jsonl'), 'a',
                      encoding='utf-8') as f:
                f.write(json.dumps(rec) + '\n')
        except OSError as exc:
            self.get_logger().warn('cannot write events.jsonl: %s' % exc)

    # ---------------------------------------------------------------- dumps
    def request_dump(self, reason, detail='', force=False):
        """Start a dump in a worker thread. Returns (started, message)."""
        now = time.monotonic()
        with self._lock:
            if self._dumping:
                return False, 'a dump is already running'
            if not force and self._last_dump is not None \
                    and now - self._last_dump < self._cooldown:
                return False, 'cooldown (%.0f s left)' % (self._cooldown - (now - self._last_dump))
            self._dumping = True
            self._last_dump = now
            messages = self._buffer.snapshot(time.time_ns())
            stats = self._buffer.stats()
        path = make_dump_dir(self._crash_dir, crash_record.timestamp(), reason)
        threading.Thread(target=self._dump_worker, name='blackbox_dump', daemon=True,
                         args=(path, reason, detail, messages, stats)).start()
        return True, path

    def _dump_worker(self, path, reason, detail, messages, stats):
        try:
            n = 0
            try:
                n = dump_bag(messages, os.path.join(path, 'bag'))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('blackbox: bag write failed: %s' % exc)
            info = {'reason': reason, 'detail': detail,
                    'time': time.strftime('%Y-%m-%dT%H:%M:%S'),
                    'revision': crash_record.git_revision(), 'messages': n,
                    'buffer': stats,
                    'supervisor_status': json.loads(self._status_json or '{}')}
            with open(os.path.join(path, 'reason.json'), 'w', encoding='utf-8') as f:
                json.dump(info, f, indent=2, sort_keys=True)
            log_dir = launch_log_dir()
            if log_dir:
                with open(os.path.join(path, 'launch_log_tail.txt'), 'w',
                          encoding='utf-8') as f:
                    f.write(tail_lines(os.path.join(log_dir, 'launch.log')))
            cutoff = time.time() - 600
            for rec in glob.glob(os.path.join(self._crash_dir, '*.txt')):
                if os.path.getmtime(rec) >= cutoff:
                    shutil.copy2(rec, path)
            self.get_logger().warn('blackbox: %s -> %s (%d messages)' % (reason, path, n))
            self._event_record({'event': 'dump', 'reason': reason, 'path': path,
                                'messages': n})
            self._prune()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('blackbox: dump failed: %s' % exc)
        finally:
            with self._lock:
                self._dumping = False

    def _prune(self):
        dirs = sorted(d for d in glob.glob(os.path.join(self._crash_dir, '2*_*'))
                      if os.path.isdir(d))
        for d in dirs[:max(0, len(dirs) - self._max_dumps)]:
            shutil.rmtree(d, ignore_errors=True)

    def _srv_dump(self, _req, resp):
        ok, msg = self.request_dump('manual', 'supervisor/dump service', force=True)
        resp.success, resp.message = ok, msg
        return resp

    def destroy_node(self):
        self._pump.stop()
        return super().destroy_node()


def main(args=None):
    crash_record.install('supervisor')
    rclpy.init(args=args)
    node = Supervisor()
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
