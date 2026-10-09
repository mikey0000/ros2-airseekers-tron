# SPDX-License-Identifier: GPL-3.0-or-later
"""Always-on rolling rosbag "black box" on disk (2026-10-09, ROS 2 Humble).

Why a third recorder: the supervisor's black box (``blackbox.py``) keeps only 120 s of ~15
rate-capped topics in RAM and dumps on a trigger; ``mow_recorder`` writes only while a mission
runs. Neither has the minutes before an incident that nobody flagged (the 2026-10-08 tuning
bags were recorded by hand with ``ros2 bag record``). This node:

* runs ``ros2 bag record`` (the C++ recorder: no deserialization, no Python per message) as a
  supervised child with 60 s splits into ``<root>/<YYYYmmdd-HHMMSS>/`` (a new session dir per
  (re)start; rosbag2 refuses an existing one), restarting it with backoff when it dies and
  killing it when this node dies (PR_SET_PDEATHSIG), so there is never an orphan recorder;
* prunes the ring every ``prune_period_s``: oldest closed splits first until the ring is under
  ``max_total_gb`` and /userdata keeps ``min_free_gb`` free; if that is impossible (disk filled
  by something else) recording pauses until free space is back above floor + 0.5 GB;
* pins incidents: on a mission incident (/mission/incident kinds ``pin_mission_kinds``), the
  mission entering EMERGENCY, /hardware_bridge/emergency rising, a new supervisor crash dump
  under $MOWER_CRASH_DIR, or ``~/pin`` (std_srvs/Trigger), it hard-links the previous
  ``pin_prev_splits`` closed splits + split 0 (/tf_static) into
  ``<incidents_dir>/<ts>_<reason>/`` at once, and the current + ``pin_post_splits`` more as
  they close. Hard links (same /userdata filesystem) cost no copy and no space until the ring
  prunes its own name. Pins have their own cap (``incident_max_gb``, ``max_incidents``).

Rosbag2 choices (rosbag2 0.15.17 in the image, checked 2026-10-09): storage sqlite3 (mcap is
not installed: ``ros-humble-rosbag2-storage-mcap`` would need an image rebuild); preset
``resilient`` (WAL + synchronous=NORMAL: a power cut loses at most the last cache flush, not
the db); ``--max-cache-size`` 1 MiB (the default 100 MiB double buffer would hold ~10 min of
this topic set in RAM, all lost on a crash, and cost up to 200 MB of the 4 GB); file-mode zstd:
each closed split is compressed once by one rosbag2 thread (zstd level 1, ~10 MB per minute of
data, tens of ms of one A76 core per split on the RK3588), 3-5x smaller on odometry/IMU/tf, so
3 GB holds a day of operation and the eMMC sees a third of the writes. Message-mode compression
is avoided: per-message zstd on 100-byte messages costs far more CPU and compresses poorly.

Reading a pinned/pulled split: ``zstd -d x_3.db3.zstd && ros2 bag info x_3.db3`` (each split is
a complete sqlite3 bag; the image has no zstd CLI, decompress on the dev box; see
``scripts/deploy_to_mower.sh bags``).
"""

import json
import os
import shutil
import signal
import subprocess
import threading
import time

from mower_control import bag_ring as br

# Curated (2026-10-09, measured on the mower): the first ring recorded 73 topics, 554 msg/s,
# and its `ros2 bag record` took 46-53 % of one RK3588 core. rosbag2 Humble's cost is per
# delivered message: Fast DDS UDP receive (no SHM in this container) + one executor wake that
# walks every subscription (~3 us x N per message on x86). Benchmark (dev box, synthetic load
# replaying the mower's measured topic rates/sizes, scripts in docs/crash_recovery.md "Rolling
# bag ring"): all 73 topics 25-29 % x86 (~46 % on the mower: ratio ~1.7), this list 7.4 % x86
# (~12 % mower est.) parked, 9-10 % x86 (~15 % mower) with the cmd_vel chain at 20 Hz while
# driving; the old set measured 31 % x86 while driving. Dropped, with what still covers them:
#   /imu/data_aligned 100 Hz        -> /tilt/status, /heading_aligner/status, /odometry/*
#   /mower_base/status 100 Hz       -> /hardware_bridge/status (5 Hz), /mower_sensor_info
#   /odom 50 Hz, /odometry/gps      -> /odometry/filtered (20 Hz), /mcu/measured_speed, /fix
#   /tf 30 Hz                       -> /odometry/filtered (odom->base_link) and
#                                      /odometry/filtered_map (map frame); /tf_static kept
#   /mower_base/bumper_routing_status 20 Hz, /vision/obstacle_close, /ai/det/obstacles,
#   /stuck, /estop, /fix_gated, /vel, /heading, /gps/status, /cmd_vel_nav_raw, /cmd_vel_raw,
#   /received_global_plan, /map_server_node/boundary_violation (non-lethal), /local_plan,
#   /mcu/sent_speed (one per wire frame while driving: /cmd_vel + /mcu/commanded_speed)
#   (never published) -> derived/duplicate of a kept topic
#   /rosout 12.6 Hz                 -> every node's log file and `docker logs`
# Add any back with the ``extra_topics`` parameter (e.g. ['/imu/data_aligned', '/tf'] for a
# tuning session; each 100 Hz topic costs ~8 % of a core on the mower).
# Also recorded for the per-mow pins (they replaced mow_recorder): boundary_status and
# terrain_summary (latched).
DEFAULT_TOPICS = [
    # command chain (silent when parked): lanes -> twist_mux -> cmd_vel_slew -> /cmd_vel -> MCU
    '/cmd_vel_nav', '/cmd_vel_docking', '/cmd_vel_teleop', '/cmd_vel_bumper',
    '/cmd_vel_emergency', '/cmd_vel', '/cmd_vel_slew/gate_status', '/cmd_vel_slew/shape_status',
    '/estop_request', '/motion_enabled',
    '/mcu/measured_speed', '/mcu/commanded_speed',
    # localization (20 + 10 Hz)
    '/odometry/filtered', '/odometry/filtered_map', '/tf_static', '/heading_aligner/status',
    '/tilt/status', '/localization/status',
    # GPS (10 Hz fix + RTK state)
    '/fix', '/fix_status', '/ntrip/status',
    # mission
    '/behavior_tree_node/high_level_status', '/behavior_tree_node/mow_plan',
    '/behavior_tree_node/mow_progress', '/mission/incident', '/mission/obstacle_decision',
    '/mission/activity', '/stuck_guard/suppress', '/stuck_guard/reset', '/supervisor/status',
    '/coverage/plan_quality',
    # obstacles / perception summaries
    '/obstacle_policy', '/stereo_depth/stats', '/ai/det/detections_ranged',
    # Nav2: plans, local costmap (120x120 at 1 Hz), BT log
    '/plan', '/controller_server/FollowCoveragePathFTC/global_plan', '/local_costmap/costmap',
    '/behavior_tree_log',
    # hardware: bumper/lift (sensor_info), battery, emergency, slip/stuck, docking
    '/battery', '/mower_sensor_info', '/rain',
    '/hardware_bridge/status', '/hardware_bridge/emergency', '/hardware_bridge/power',
    '/mower_base/key_pressed', '/dig_stall', '/stuck_event',
    '/mower_docking/state', '/mower_docking/marker_in_view',
    '/map_server_node/lethal_boundary_violation', '/map_server_node/replan_needed',
    '/map_server_node/boundary_status', '/map_server_node/terrain_summary',
    # diagnostics (9 Hz)
    '/diagnostics',
]

# 'detour' is a routine decision, not an incident (several per mow); the rest are aborts.
DEFAULT_PIN_KINDS = ['subpath_failed', 'transit_abort', 'follow_abort', 'stuck']


def _pdeathsig():
    """preexec_fn: deliver SIGINT (clean bag close) to the recorder if this node dies."""
    try:
        import ctypes
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGINT)  # PR_SET_PDEATHSIG
    except Exception:  # noqa: BLE001 - best effort (non-Linux)
        pass


class RecorderProcess:
    """One supervised ``ros2 bag record`` child (no ROS here: unit-testable with a fake cmd)."""

    def __init__(self, root, log_path, popen=subprocess.Popen):
        self.root = root
        self.log_path = log_path
        self.popen = popen
        self.proc = None
        self.session = None
        self.started = None
        self.restarts = 0
        self.backoff_s = 2.0
        self.next_start = 0.0

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, cmd_for, now, wall):
        name = br.session_name(wall)
        path = os.path.join(self.root, name)
        n = 1
        while os.path.exists(path):
            path = os.path.join(self.root, '%s.%d' % (name, n))
            n += 1
        os.makedirs(self.root, exist_ok=True)
        mode = 'a'
        try:
            if os.path.getsize(self.log_path) > 5 * 1024 * 1024:
                mode = 'w'
        except OSError:
            pass
        log = open(self.log_path, mode, encoding='utf-8')
        log.write('\n=== %s start %s\n' % (time.strftime('%Y-%m-%dT%H:%M:%S'), path))
        log.flush()
        try:
            self.proc = self.popen(cmd_for(path), stdout=log, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL, preexec_fn=_pdeathsig)
        finally:
            log.close()
        self.session = os.path.basename(path)
        self.started = now
        return path

    def poll_exit(self, now):
        """Returns the exit code once when the child has died, else None. Schedules restart."""
        if self.proc is None or self.proc.poll() is None:
            return None
        rc = self.proc.returncode
        self.proc = None
        ran = now - (now if self.started is None else self.started)
        # a child that ran for a while gets a fresh backoff; a crash loop backs off to 60 s
        self.backoff_s = 2.0 if ran > 120.0 else min(60.0, self.backoff_s * 2.0)
        self.next_start = now + self.backoff_s
        self.restarts += 1
        return rc

    def stop(self, timeout_s=15.0):
        """SIGINT (rosbag2 flushes, compresses the last split, writes metadata.yaml)."""
        p, self.proc = self.proc, None
        if p is None or p.poll() is not None:
            return
        try:
            p.send_signal(signal.SIGINT)
            p.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait(timeout=5.0)
        except OSError:
            pass


class RecorderSwitch:
    """Runtime on/off of the ring (``~/enable`` std_srvs/SetBool), persisted in ``state_file``
    so a GUI "off" survives a stack restart. Off = the child is stopped cleanly (SIGINT:
    rosbag2 closes and compresses the last split) and not restarted; pruning, the incident
    pins of already recorded splits and the status keep working. No ROS here (unit-tested)."""

    def __init__(self, rec, state_file, default=True):
        self.rec = rec
        self.state_file = state_file
        self.enabled = br.load_enabled(state_file, default)

    def set(self, enabled, now):
        """Returns (changed, persisted)."""
        enabled = bool(enabled)
        changed = enabled != self.enabled
        self.enabled = enabled
        saved = br.save_enabled(self.state_file, enabled)
        if not enabled:
            self.rec.stop()
        elif changed:
            self.rec.backoff_s = 2.0
            self.rec.next_start = now      # start at the next tick
        return changed, saved

    def due_start(self, paused, now):
        return self.enabled and not paused and not self.rec.running() \
            and now >= self.rec.next_start


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger

    from mower_control.mow_recorder import MowSession
    from mower_control.supervisor import _import_type

    class BagRecorder(Node):
        def __init__(self):
            super().__init__('bag_recorder')
            dp = self.declare_parameter
            dp('root', '/userdata/ros2/bags')
            dp('incidents_dir', '/userdata/ros2/incidents')
            dp('crash_dir', os.environ.get('MOWER_CRASH_DIR', '/userdata/ros2/crashes'))
            dp('topics', DEFAULT_TOPICS)
            dp('extra_topics', [''])            # appended (launch/GUI overrides keep defaults)
            dp('split_s', 60)
            dp('max_total_gb', 3.0)
            dp('min_free_gb', 3.0)
            dp('max_cache_bytes', 1024 * 1024)
            dp('compression', 'zstd')           # '' = none
            dp('storage', 'sqlite3')
            dp('storage_preset', 'resilient')
            dp('prune_period_s', 10.0)
            dp('pin_prev_splits', 2)
            dp('pin_post_splits', 1)
            dp('pin_cooldown_s', 30.0)
            dp('pin_mission_kinds', DEFAULT_PIN_KINDS)
            dp('incident_max_gb', 1.0)
            dp('max_incidents', 50)
            dp('polling_ms', 5000)              # topic discovery poll (1 s cost ~1 % of a core)
            dp('enabled_default', True)         # when state_file does not exist yet
            dp('state_file', '/userdata/ros2/bag_recorder.json')
            # per-mow pins (replace the mow_recorder process): <mows_dir>/<ts>/
            dp('mow_pins', True)
            dp('mows_dir', '/userdata/ros2/mows')
            dp('mow_prev_splits', 1)
            dp('stop_after_idle_s', 5.0)
            dp('max_mows', 10)
            dp('mow_max_gb', 2.0)
            gp = lambda n: self.get_parameter(n).value  # noqa: E731
            self._root = str(gp('root'))
            self._inc_dir = str(gp('incidents_dir'))
            self._crash_dir = str(gp('crash_dir'))
            topics = [t for t in list(gp('topics')) + list(gp('extra_topics')) if t]
            self._topics = list(dict.fromkeys(topics))
            gb = 1024 ** 3
            self._max_bytes = int(float(gp('max_total_gb')) * gb)
            self._min_free = int(float(gp('min_free_gb')) * gb)
            self._resume_free = self._min_free + gb // 2
            self._inc_max_bytes = int(float(gp('incident_max_gb')) * gb)
            self._max_incidents = int(gp('max_incidents'))
            split_s, cache = int(gp('split_s')), int(gp('max_cache_bytes'))
            comp, storage, preset = str(gp('compression')), str(gp('storage')), \
                str(gp('storage_preset'))
            self._compressed = bool(comp)
            polling = int(gp('polling_ms'))
            self._cmd_for = lambda out: br.record_cmd(  # noqa: E731
                out, self._topics, split_s, cache, comp, storage, preset, polling)
            self._rec = RecorderProcess(self._root, os.path.join(self._root, 'recorder.log'))
            self._switch = RecorderSwitch(self._rec, str(gp('state_file')),
                                          bool(gp('enabled_default')))
            self._mow_pins = bool(gp('mow_pins'))
            self._mows_dir = str(gp('mows_dir'))
            self._mow = br.MowPinner(prev=int(gp('mow_prev_splits')))
            self._mow_sess = MowSession(float(gp('stop_after_idle_s')))
            self._mow_files = []
            self._mow_t0 = None
            self._max_mows = int(gp('max_mows'))
            self._mow_max_bytes = int(float(gp('mow_max_gb')) * gb)
            self._ring_bytes = 0
            self._free = None
            self._pins = br.PinPlanner(gp('pin_prev_splits'), gp('pin_post_splits'))
            self._det = br.IncidentDetector(list(gp('pin_mission_kinds')),
                                            cooldown_s=float(gp('pin_cooldown_s')))
            self._paused = False
            self._last_prune = 0.0
            self._prune_period = float(gp('prune_period_s'))
            self._last_crash_poll = 0.0
            self._lock = threading.Lock()

            qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
            self.create_subscription(String, '/mission/incident', self._on_incident, qos)
            try:
                hl = _import_type('mowgli_interfaces/msg/HighLevelStatus')
                em = _import_type('mowgli_interfaces/msg/Emergency')
                self.create_subscription(hl, '/behavior_tree_node/high_level_status',
                                         self._on_hl, qos)
                self.create_subscription(em, '/hardware_bridge/emergency',
                                         self._on_emergency, qos)
            except (ImportError, AttributeError, ValueError) as exc:
                self.get_logger().warn('mission/emergency triggers off: %s' % exc)
            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self._status_pub = self.create_publisher(String, '~/status', latched)
            self.create_service(Trigger, '~/pin', self._srv_pin)
            self.create_service(SetBool, '~/enable', self._srv_enable)
            self.create_service(Trigger, '~/get_state', self._srv_get_state)
            self.create_timer(1.0, self._tick)
            self.get_logger().info(
                'bag_recorder: %s, %d topics -> %s (%d s splits, %s/%s%s), ring %.1f GB, '
                'floor %.1f GB free, pins -> %s, mows -> %s (state file %s)' % (
                    'ENABLED' if self._switch.enabled else 'DISABLED (~/enable to start)',
                    len(self._topics), self._root, split_s, storage, preset,
                    ' + zstd' if comp else '', self._max_bytes / gb, self._min_free / gb,
                    self._inc_dir, self._mows_dir if self._mow_pins else 'off',
                    self._switch.state_file))

        # ------------------------------------------------------------ triggers
        def _on_incident(self, msg):
            self._pin(self._det.on_mission_incident(msg.data, time.monotonic()))

        def _on_hl(self, msg):
            state = getattr(msg, 'state_name', '')
            now = time.monotonic()
            self._pin(self._det.on_state(state, now))
            if self._mow_pins:
                act = self._mow_sess.update(state, now)
                if act == 'start':
                    self._mow_start()
                elif act == 'stop':
                    self._mow_stop(state)

        # ------------------------------------------------------------ per-mow pins
        def _mow_start(self):
            if not self._switch.enabled:
                self.get_logger().info('bag_recorder: mow started, recording disabled: '
                                       'no mow bag')
                return
            if self._mow.active:            # the previous mow's last split never closed
                self._mow.close()
            with self._lock:
                sessions = br.scan(self._root, self._rec.session if self._rec.running()
                                   else None, self._compressed)
                base = os.path.join(self._mows_dir, br.session_name(time.time()))
                mow_dir, n = base, 1
                while os.path.exists(mow_dir):
                    mow_dir, n = '%s.%d' % (base, n), n + 1
                os.makedirs(mow_dir, exist_ok=True)
                self._mow_files = []
                self._mow_t0 = time.strftime('%Y-%m-%dT%H:%M:%S%z')
                for sp in self._mow.start(mow_dir, sessions, self._rec.session
                                          if self._rec.running() else None, time.monotonic()):
                    self._mow_link(sp)
                self._write_mow_info(None)
                self._prune_mows()
            self.get_logger().info('bag_recorder: mow started -> %s' % mow_dir)

        def _mow_stop(self, state):
            if not self._mow.active:
                return
            with self._lock:
                sessions = br.scan(self._root, self._rec.session if self._rec.running()
                                   else None, self._compressed)
                self._mow.stop(sessions, self._rec.session if self._rec.running() else None,
                               time.monotonic())
                self._write_mow_info('mission back to %s' % state)
            self.get_logger().info('bag_recorder: mow ended (%s), linking the last split'
                                   % state)

        def _mow_link(self, sp):
            dst = br.pin_split(sp, self._mow.mow_dir)
            if dst:
                self._mow.mark(sp)
                self._mow_files.append(os.path.basename(dst))
            return dst

        def _write_mow_info(self, end_reason):
            if not self._mow.active:
                return
            try:
                br.write_pin_info(self._mow.mow_dir, {
                    'reason': 'mow', 'start': self._mow_t0,
                    'end': time.strftime('%Y-%m-%dT%H:%M:%S%z') if end_reason else None,
                    'end_reason': end_reason, 'files': sorted(self._mow_files),
                    'read': 'zstd -d *.zstd; ros2 bag info <file>.db3 (each split is a bag)'})
            except OSError as exc:
                self.get_logger().warn('bag_recorder: mow info: %s' % exc)

        def _prune_mows(self):
            try:
                names = sorted(n for n in os.listdir(self._mows_dir)
                               if os.path.isdir(os.path.join(self._mows_dir, n)))
            except OSError:
                return
            entries = [(n, br.dir_bytes(os.path.join(self._mows_dir, n))) for n in names]
            keep = {os.path.basename(self._mow.mow_dir)} if self._mow.active else set()
            for n in br.plan_incident_retention(entries, self._max_mows, self._mow_max_bytes,
                                                keep):
                shutil.rmtree(os.path.join(self._mows_dir, n), ignore_errors=True)

        # ------------------------------------------------------------ on/off + state
        def _status(self):
            return {'enabled': self._switch.enabled, 'recording': self._rec.running(),
                    'session': self._rec.session, 'paused_low_disk': self._paused,
                    'ring_bytes': self._ring_bytes, 'free_bytes': self._free,
                    'restarts': self._rec.restarts, 'pending_pins': len(self._pins.pending),
                    'suppressed_triggers': self._det.suppressed,
                    'mow_pins': self._mow_pins,
                    'mow': os.path.basename(self._mow.mow_dir) if self._mow.active else None,
                    'topics': len(self._topics)}

        def _publish_status(self):
            self._status_pub.publish(String(data=json.dumps(self._status(), sort_keys=True)))

        def _srv_enable(self, req, resp):
            changed, saved = self._switch.set(bool(req.data), time.monotonic())
            if changed:
                self.get_logger().warn('bag_recorder: recording %s via ~/enable%s' % (
                    'ENABLED' if req.data else 'DISABLED',
                    '' if saved else ' (NOT persisted: %s not writable)'
                    % self._switch.state_file))
            if not req.data and self._mow.active:
                self._mow_stop('recording disabled')
            if req.data and changed:
                self._tick()                 # start now, not at the next timer tick
            self._publish_status()
            resp.success = saved
            resp.message = json.dumps(self._status(), sort_keys=True)
            return resp

        def _srv_get_state(self, _req, resp):
            resp.success = True
            resp.message = json.dumps(self._status(), sort_keys=True)
            return resp

        def _on_emergency(self, msg):
            self._pin(self._det.on_emergency(getattr(msg, 'active_emergency', False),
                                             getattr(msg, 'reason', ''), time.monotonic()))

        def _srv_pin(self, _req, resp):
            path = self._pin(self._det.manual('bag_recorder/pin service', time.monotonic()))
            resp.success = path is not None
            resp.message = path or 'no recording yet'
            return resp

        def _pin(self, hit):
            if hit is None:
                return None
            reason, detail = hit
            with self._lock:
                sessions = br.scan(self._root, self._rec.session if self._rec.running()
                                   else None, self._compressed)
                pin_dir = os.path.join(self._inc_dir, br.pin_dir_name(time.time(), reason))
                now_splits = self._pins.trigger(pin_dir, sessions, self._rec.session,
                                                time.monotonic())
                files = [br.pin_split(s, pin_dir) for s in now_splits]
                br.write_pin_info(pin_dir, {
                    'reason': reason, 'detail': detail,
                    'time': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                    'session': self._rec.session, 'files': sorted(f for f in files if f),
                    'pending': sorted('%s_%d' % (e[1], e[2]) for e in self._pins.pending
                                      if e[0] == pin_dir),
                    'read': 'zstd -d *.zstd; ros2 bag info <file>.db3 (each split is a bag)'})
            self.get_logger().warn('bag_recorder: pinned %s (%s) -> %s'
                                   % (reason, detail[:120], pin_dir))
            return pin_dir

        # ------------------------------------------------------------ periodic
        def _tick(self):
            now = time.monotonic()
            rc = self._rec.poll_exit(now)
            if rc is not None:
                self.get_logger().error('bag_recorder: ros2 bag record exited rc=%s, restart in '
                                        '%.0f s (%s)' % (rc, self._rec.backoff_s,
                                                         self._rec.log_path))
            if now - self._last_crash_poll >= 5.0:
                self._last_crash_poll = now
                try:
                    names = [n for n in os.listdir(self._crash_dir)
                             if os.path.isdir(os.path.join(self._crash_dir, n))]
                except OSError:
                    names = []
                self._pin(self._det.on_crash_dirs(names, now))
            due_start = self._switch.due_start(self._paused, now)
            # prune before every (re)start too: never open a session on a disk below the floor
            if due_start or now - self._last_prune >= self._prune_period:
                self._last_prune = now
                self._housekeeping(now)
            if due_start and not self._paused and self._switch.enabled:
                path = self._rec.start(self._cmd_for, now, time.time())
                self.get_logger().info('bag_recorder: recording %s' % path)

        def _housekeeping(self, now):
            with self._lock:
                sessions = br.scan(self._root, self._rec.session if self._rec.running()
                                   else None, self._compressed)
                free = br.free_bytes(self._root if os.path.isdir(self._root)
                                     else os.path.dirname(self._root))
                free = free if free is not None else self._min_free
                # pins first: a split an incident waits for is linked before the ring prunes it
                for pin_dir, sp in self._pins.due(sessions, now):
                    if br.pin_split(sp, pin_dir):
                        self.get_logger().info('bag_recorder: pinned %s_%d -> %s'
                                               % (sp.session, sp.index, pin_dir))
                mow_linked = [sp for _d, sp in self._mow.due(sessions, now)
                              if self._mow_link(sp)]
                if mow_linked:
                    self._write_mow_info(None if self._mow._end is None else 'ended')
                mow_dir = self._mow.mow_dir
                if self._mow.finished(now):
                    self.get_logger().info('bag_recorder: mow bag complete: %s (%d splits)'
                                           % (mow_dir, len(self._mow_files)))
                    self._prune_mows()
                victims, starved = br.plan_prune(sessions, self._rec.session
                                                 if self._rec.running() else None,
                                                 self._max_bytes, free, self._min_free,
                                                 self._pins.pending_splits()
                                                 | self._mow.protect(sessions))
                freed = br.apply_prune(self._root, victims)
                free += freed
                self._prune_incidents()
            if starved and not self._paused:
                self._paused = True
                self._rec.stop()
                self.get_logger().error(
                    'bag_recorder: PAUSED: %.2f GB free on %s < floor %.2f GB and nothing left '
                    'to prune' % (free / 1024 ** 3, self._root, self._min_free / 1024 ** 3))
            elif self._paused and free >= self._resume_free:
                self._paused = False
                self.get_logger().info('bag_recorder: disk space back (%.2f GB), resuming'
                                       % (free / 1024 ** 3))
            self._ring_bytes = br.total_bytes(sessions) - \
                sum(v.size for v in victims if not isinstance(v, str))
            self._free = free
            if victims:
                self.get_logger().info('bag_recorder: pruned %d (%.1f MB)'
                                       % (len(victims), freed / 1e6))
            self._publish_status()

        def _prune_incidents(self):
            try:
                names = sorted(n for n in os.listdir(self._inc_dir)
                               if os.path.isdir(os.path.join(self._inc_dir, n)))
            except OSError:
                return
            entries = [(n, br.dir_bytes(os.path.join(self._inc_dir, n))) for n in names]
            keep = {os.path.basename(d) for d in self._pins.pending_dirs()}
            for n in br.plan_incident_retention(entries, self._max_incidents,
                                                self._inc_max_bytes, keep):
                shutil.rmtree(os.path.join(self._inc_dir, n), ignore_errors=True)

        def shutdown(self):
            self._rec.stop()

    rclpy.init(args=args)
    node = BagRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
