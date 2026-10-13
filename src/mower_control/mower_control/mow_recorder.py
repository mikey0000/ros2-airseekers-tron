# SPDX-License-Identifier: GPL-3.0-or-later
"""Per-mow rosbag2 recorder (ROS 2 Jazzy).

Replaces the ad-hoc ``/userdata/ros2/logs/rec.py`` CSV recorder (it was started with
``docker exec -d`` and so died silently with every container recreate; ``pgrep -f rec.py``
then only matched the pgrep's own shell).

Records ``topics`` (``topic|type[|latched]``; latched = reliable transient-local, to get
the last value of a latched publisher at the start) to a rosbag2 under ``<root>/<YYYYmmdd-HHMMSS>/bag``
for each mission: starts when ``/behavior_tree_node/high_level_status.state_name`` leaves
the idle states (IDLE, IDLE_DOCKED, CHARGING), stops when it has been back in one for
``stop_after_idle_s``. Storage is mcap when the image has the plugin, else sqlite3.
Retention: at most ``max_mows`` recordings and ``max_total_gb`` in total; the oldest go
first, the recording in progress is never deleted.

This complements the supervisor's black box (``blackbox.py``): that one keeps the last
120 s in RAM and dumps on a crash/emergency to /userdata/ros2/crashes; this one writes
whole mows to disk. They share no state.

Messages are stored serialized (raw subscriptions, no deserialize). Subscription
callbacks only queue; a writer thread drains the queue into rosbag2_py.
"""

import collections
import os
import shutil
import threading
import time

IDLE_STATES = frozenset(('IDLE', 'IDLE_DOCKED', 'CHARGING'))

DEFAULT_TOPICS = [
    '/cmd_vel_nav|geometry_msgs/msg/Twist',
    '/cmd_vel|geometry_msgs/msg/Twist',
    '/odom|nav_msgs/msg/Odometry',
    '/odometry/filtered_map|nav_msgs/msg/Odometry',
    '/imu/data_aligned|sensor_msgs/msg/Imu',
    '/plan|nav_msgs/msg/Path',
    '/obstacle_policy|std_msgs/msg/String',
    '/behavior_tree_node/high_level_status|mowgli_interfaces/msg/HighLevelStatus',
    '/map_server_node/boundary_status|std_msgs/msg/String',
    '/cmd_vel_slew/gate_status|std_msgs/msg/String|latched',
    '/fix_status|std_msgs/msg/String',
    # terrain memory inputs (offline re-import of a mow, docs/terrain_aware_planning.md)
    '/dig_stall|std_msgs/msg/Bool|latched',
    '/mission/incident|std_msgs/msg/String',
    '/map_server_node/terrain_summary|std_msgs/msg/String|latched',
]


# --------------------------------------------------------------------------- pure logic
class MowSession:
    """Start/stop decisions from the mission state (pure; times in seconds)."""

    def __init__(self, stop_after_idle_s=5.0):
        self.stop_after_idle_s = float(stop_after_idle_s)
        self.recording = False
        self._idle_since = None

    def update(self, state_name, now):
        """Returns 'start', 'stop' or None."""
        idle = str(state_name) in IDLE_STATES
        if not self.recording:
            if not idle:
                self.recording = True
                self._idle_since = None
                return 'start'
            return None
        if not idle:
            self._idle_since = None
            return None
        if self._idle_since is None:
            self._idle_since = now
        if now - self._idle_since >= self.stop_after_idle_s:
            self.recording = False
            self._idle_since = None
            return 'stop'
        return None


def dir_size(path):
    total = 0
    for base, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(base, f))
            except OSError:
                pass
    return total


def retention_victims(entries, max_mows, max_bytes, keep=None):
    """``entries`` = [(name, bytes)]; names sort chronologically (YYYYmmdd-HHMMSS).
    Returns the names to delete (oldest first) so that at most ``max_mows`` remain and
    their total is <= ``max_bytes``. ``keep`` (the active recording) is never returned."""
    ordered = sorted(entries)
    victims = []
    count = len(ordered)
    total = sum(b for _n, b in ordered)
    for name, size in ordered:
        if count <= max_mows and total <= max_bytes:
            break
        if name == keep:
            continue
        victims.append(name)
        count -= 1
        total -= size
    return victims


def apply_retention(root, max_mows, max_bytes, keep=None, rmtree=shutil.rmtree):
    try:
        names = [n for n in os.listdir(root) if os.path.isdir(os.path.join(root, n))]
    except OSError:
        return []
    entries = [(n, dir_size(os.path.join(root, n))) for n in names]
    victims = retention_victims(entries, max_mows, max_bytes, keep)
    for n in victims:
        rmtree(os.path.join(root, n), ignore_errors=True)
    return victims


def session_dir_name(wall_time):
    return time.strftime('%Y%m%d-%H%M%S', time.localtime(wall_time))


def pick_storage(available):
    return 'mcap' if 'mcap' in available else 'sqlite3'


def available_storage():
    """Storage plugins rosbag2 can load in this install (mcap only if installed)."""
    out = {'sqlite3'}
    prefixes = os.environ.get('AMENT_PREFIX_PATH', '/opt/ros/jazzy').split(os.pathsep)
    for p in prefixes:
        if os.path.isdir(os.path.join(p, 'share', 'rosbag2_storage_mcap')):
            out.add('mcap')
    return out


# --------------------------------------------------------------------------- node
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    import rosbag2_py

    from mower_control.supervisor import _import_type

    class MowRecorder(Node):
        def __init__(self):
            super().__init__('mow_recorder')
            dp = self.declare_parameter
            dp('root', '/userdata/ros2/mows')
            dp('topics', DEFAULT_TOPICS)
            dp('status_topic', '/behavior_tree_node/high_level_status')
            dp('stop_after_idle_s', 5.0)
            dp('max_mows', 10)
            dp('max_total_gb', 2.0)
            gp = lambda n: self.get_parameter(n).value  # noqa: E731
            self._root = str(gp('root'))
            self._max_mows = int(gp('max_mows'))
            self._max_bytes = int(float(gp('max_total_gb')) * 1024 ** 3)
            self._storage = pick_storage(available_storage())
            self._session = MowSession(float(gp('stop_after_idle_s')))
            self._status_topic = str(gp('status_topic'))
            self._lock = threading.Lock()
            self._queue = collections.deque()
            self._writer = None
            self._current = None
            self._types = {}
            self._hl_type = None
            self._latched = {}        # latched topic -> last (data, t_ns): written at open

            self._qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            # Streaming topics (odom, imu, cmd_vel, ...) are subscribed ONLY while a mow is
            # being recorded: idle/docked the node only listens to the mission status and the
            # (rare) latched topics, so it costs ~nothing (it used to deserialize-free but
            # still wake for every 50-100 Hz sample, ~50 % CPU docked).
            self._stream_specs = []   # (topic, msg_type) subscribed while recording
            self._stream_subs = []
            for spec in gp('topics'):
                try:
                    parts = spec.split('|')
                    topic, type_name = parts[:2]
                    msg_type = _import_type(type_name)
                except (ValueError, ImportError, AttributeError) as exc:
                    self.get_logger().warn('skipping %r (%s)' % (spec, exc))
                    continue
                self._types[topic] = type_name
                if topic == self._status_topic:
                    self._hl_type = msg_type
                if 'latched' in parts[2:]:
                    self._latched[topic] = None
                    self.create_subscription(msg_type, topic, self._make_cb(topic),
                                             latched, raw=True)
                elif topic == self._status_topic:
                    self.create_subscription(msg_type, topic, self._make_cb(topic),
                                             self._qos, raw=True)
                else:
                    self._stream_specs.append((topic, msg_type))
            if self._hl_type is None:
                self._hl_type = _import_type('mowgli_interfaces/msg/HighLevelStatus')
                self.create_subscription(self._hl_type, self._status_topic,
                                         self._make_cb(None), self._qos, raw=True)
            self._stop = threading.Event()
            self._thread = threading.Thread(target=self._drain, name='mow_recorder',
                                            daemon=True)
            self._thread.start()
            self.get_logger().info('mow_recorder: %d topics -> %s (%s), keep %d mows / %.1f GB'
                                   % (len(self._types), self._root, self._storage,
                                      self._max_mows, self._max_bytes / 1024 ** 3))

        def _make_cb(self, topic):
            from rclpy.serialization import deserialize_message

            def cb(data):
                t = time.time_ns()
                if topic == self._status_topic or topic is None:
                    try:
                        msg = deserialize_message(data, self._hl_type)
                        self._on_state(msg.state_name)
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().warn('status decode: %s' % exc,
                                               throttle_duration_sec=30.0)
                if topic in self._latched:
                    self._latched[topic] = (bytes(data), t)
                if topic is not None and self._session.recording:
                    self._queue.append((topic, bytes(data), t))
            return cb

        def _on_state(self, state_name):
            with self._lock:
                act = self._session.update(state_name, time.monotonic())
                if act == 'start':
                    self._open()
                elif act == 'stop':
                    self._close('mission back to %s' % state_name)

        def _open(self):
            name = session_dir_name(time.time())
            os.makedirs(self._root, exist_ok=True)
            apply_retention(self._root, self._max_mows - 1, self._max_bytes)
            path = os.path.join(self._root, name)
            n = 1
            while os.path.exists(path):
                path = os.path.join(self._root, '%s.%d' % (name, n))
                n += 1
            os.makedirs(path)
            w = rosbag2_py.SequentialWriter()
            w.open(rosbag2_py.StorageOptions(uri=os.path.join(path, 'bag'),
                                             storage_id=self._storage),
                   rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                               output_serialization_format='cdr'))
            for topic, typ in self._types.items():
                w.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typ,
                                                        serialization_format='cdr'))
            self._writer, self._current = w, os.path.basename(path)
            self._queue.clear()
            self._subscribe_streams()
            for topic, last in self._latched.items():
                if last is not None:
                    w.write(topic, last[0], last[1])
            self.get_logger().info('mow_recorder: recording %s' % path)

        def _subscribe_streams(self):
            if self._stream_subs:
                return
            for topic, msg_type in self._stream_specs:
                self._stream_subs.append(self.create_subscription(
                    msg_type, topic, self._make_cb(topic), self._qos, raw=True))

        def _unsubscribe_streams(self):
            subs, self._stream_subs = self._stream_subs, []
            for sub in subs:
                try:
                    self.destroy_subscription(sub)
                except Exception:  # noqa: BLE001
                    pass

        def _close(self, why):
            self._unsubscribe_streams()
            self._flush()
            self._writer = None           # destructor closes the bag + writes metadata
            done, self._current = self._current, None
            self.get_logger().info('mow_recorder: closed %s (%s)' % (done, why))
            gone = apply_retention(self._root, self._max_mows, self._max_bytes)
            if gone:
                self.get_logger().info('mow_recorder: retention removed %s' % gone)

        def _flush(self):
            w = self._writer
            q = self._queue
            while q:
                topic, data, t = q.popleft()
                if w is not None:
                    try:
                        w.write(topic, data, t)
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().error('write %s: %s' % (topic, exc),
                                                throttle_duration_sec=30.0)

        def _drain(self):
            next_ret = time.monotonic() + 60.0
            while not self._stop.wait(0.5 if self._writer is not None else 1.0):
                with self._lock:
                    self._flush()
                    if self._writer is not None and time.monotonic() > next_ret:
                        next_ret = time.monotonic() + 60.0
                        apply_retention(self._root, self._max_mows, self._max_bytes,
                                        keep=self._current)

        def shutdown(self):
            self._stop.set()
            with self._lock:
                if self._writer is not None:
                    self._close('shutdown')

    rclpy.init(args=args)
    node = MowRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
