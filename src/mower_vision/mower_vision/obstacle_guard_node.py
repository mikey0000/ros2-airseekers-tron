"""``obstacle_guard`` — det_ros detections -> close-obstacle flag (+ optional stop).

Subscribes ``/ai/det/detections`` (``vision_msgs/Detection2DArray`` from ``det_ros``) and
applies the danger-zone rule in :mod:`mower_vision.guard_logic` to whitelisted classes.

Prefers ``/ai/det/detections_ranged`` (``det_range``: same detections, ranged ones carry
``results[0].pose.pose.position`` in base_link and ``pose.covariance[0] > 0``): a ranged
whitelisted detection is close when its planar distance from (``range_origin_x_m``, 0)
is <= ``obstacle_stop_range_m``; unranged ones keep the image-space rule. While ranged
messages arrive (within ``ranged_timeout_s``) the raw ``/ai/det/detections`` are ignored;
otherwise the guard falls back to them.

Publishes:
* ``/vision/obstacle_close``   ``std_msgs/Bool`` on every detection frame and on hold expiry;
* ``/vision/obstacle_markers``  ``visualization_msgs/ImageMarker`` (LINE_LIST, image pixels,
  frame = camera frame): red = close, yellow = whitelisted, grey = other, blue = danger
  line. Add it as an annotation on the camera image in a Foxglove Image panel.

* ``/obstacle_policy``  ``std_msgs/String`` JSON ``{kind: none|dynamic|static, class,
  distance_m, bearing_deg}`` (latched, re-published at ``policy_rate_hz``): the closest
  detection of ANY class with score >= ``min_score``; ``dynamic_classes`` (person, dog,
  cat) within ``obstacle_stop_range_m`` -> dynamic (mission: stop, blade off, wait),
  every other class RANGED within ``policy_static_range_m`` -> static (mission: detour).
  An unranged static-class box (side cameras, stereo ranging failed) only gives the
  advisory kind ``unranged`` (mission ignores it), unless the owner sets
  ``sensitive_static_unranged: true`` and the area level is ``sensitive``.
  Priority dynamic > static > unranged. Unranged dynamic boxes use the bbox-height proxy.

Motion relevance + level (live parameter ``obstacle_detection``: none|standard|sensitive,
set per area by mower_mission): a camera's detections only count when they matter for the
commanded motion (``/cmd_vel``, unstamped Twist): the front stereo (``vio_camera``) unless
only reversing, a side OA camera only while turning toward that side, the rear camera
(``rear_frames``; not in the detector yet) while reversing. Unranged boxes need a bbox
height >= ``unranged_h_frac`` x image height. ``none`` publishes kind none only;
``sensitive`` uses ``sensitive_stop_range_m`` / ``sensitive_min_score`` and every camera.
``standard`` side cameras (owner: "left and right obstacle camera shouldn't affect direct
path mowing unless it's close and the mower is turning"): applied ``/cmd_vel`` turn toward
that side > ``side_turn_min_radps`` for >= ``side_turn_min_s`` AND ``/odom`` |w| >
``odom_turn_min_radps``, and the box is ranged within ``side_stop_range_m`` or unranged with
bbox height >= ``side_unranged_h_frac``, score >= ``side_unranged_min_score`` in
``persist_frames`` consecutive frames. Unranged front dynamic boxes need score >=
``front_unranged_min_score`` + persistence. The policy JSON also carries ``camera``,
``score``, ``bbox_h_frac`` and ``ranged``; dismissed detections log INFO
``ignored: cat on right camera (...)`` once per episode.

When ``stop_on_close`` is true, on the rising edge of the close state:
* zero ``geometry_msgs/Twist`` on ``/cmd_vel_emergency`` at ``burst_rate_hz`` for
  ``burst_s`` (twist_mux emergency input, priority 100, timeout 0.2 s);
* one ``std_srvs/Trigger`` call to ``/cutter_off`` (``mower_mcu_driver``).
"""

from __future__ import annotations

import json
import math
import struct
import sys
import time
from dataclasses import replace

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from mower_vision.sub_pump import SubscriptionPump, parse_odometry
from sensor_msgs.msg import CameraInfo
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import Bool, ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import ImageMarker

try:
    from vision_msgs.msg import Detection2DArray
    _DEP_ERROR = None
except ImportError as _exc:  # pragma: no cover
    _DEP_ERROR = _exc

from mower_vision.guard_logic import tick_rate_hz
from mower_vision.guard_logic import (CAM_FRONT, CAM_LEFT, CAM_REAR, CAM_RIGHT,
                                      DEFAULT_CAMERA_FRAMES, DEFAULT_CLASSES, DEFAULT_DYNAMIC,
                                      DEFAULT_LEVEL, DEFAULT_WHITELIST, OBSTACLE_LEVELS, Box,
                                      GuardConfig, GuardState, LevelTable, MotionConfig,
                                      MotionState, Persistence, PolicyConfig, PolicyState,
                                      box_outline,
                                      camera_counts, camera_role, class_label, classify,
                                      danger_line, frame_policy, level_policy,
                                      with_image_size)

RED = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
YELLOW = ColorRGBA(r=1.0, g=0.85, b=0.0, a=1.0)
GREY = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)
BLUE = ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0)


def _ranged_xy(det, origin_x):
    if not det.results:
        return None
    pose = det.results[0].pose
    if len(pose.covariance) < 1 or not pose.covariance[0] > 0.0:
        return None
    p = pose.pose.position
    return float(p.x) - origin_x, float(p.y)


def range_of(det, origin_x=0.0):
    """Planar range (m) of a det_range-tagged detection, or None when unranged."""
    xy = _ranged_xy(det, origin_x)
    return None if xy is None else math.hypot(*xy)


def bearing_of(det, origin_x=0.0):
    """Bearing (deg, base_link, left positive) of a ranged detection, or None."""
    xy = _ranged_xy(det, origin_x)
    return None if xy is None else math.degrees(math.atan2(xy[1], xy[0]))


def boxes_from_msg(msg, classes, origin_x=0.0):
    """``Detection2DArray`` -> ``[Box]`` (best hypothesis per detection)."""
    out = []
    for det in msg.detections:
        if not det.results:
            continue
        best = max(det.results, key=lambda r: r.hypothesis.score)
        out.append(Box(label=class_label(best.hypothesis.class_id, classes),
                       score=float(best.hypothesis.score),
                       cx=float(det.bbox.center.position.x),
                       cy=float(det.bbox.center.position.y),
                       w=float(det.bbox.size_x), h=float(det.bbox.size_y),
                       range_m=range_of(det, origin_x),
                       bearing_deg=bearing_of(det, origin_x)))
    return out


class ObstacleGuard(Node):
    def __init__(self):
        super().__init__('obstacle_guard')
        dp = self.declare_parameter
        dp('detections_topic', '/ai/det/detections')
        dp('ranged_topic', '/ai/det/detections_ranged')   # '' = image-space rule only
        dp('ranged_timeout_s', 1.0)
        dp('obstacle_stop_range_m', 1.0)
        dp('range_origin_x_m', 0.466)   # stereo camera x in base_link: range from the lens
        dp('classes', list(DEFAULT_CLASSES))
        dp('whitelist', list(DEFAULT_WHITELIST))
        dp('y_frac', 0.6)
        dp('w_frac', 0.15)
        dp('min_score', 0.4)
        # Fallback only: the real size per frame_id comes from camera_info.
        dp('image_width', 960)
        dp('image_height', 540)
        dp('camera_info_topics', ['/left_oa_camera/camera_info',
                                  '/right_oa_camera/camera_info'])
        dp('hold_s', 0.5)
        dp('stop_on_close', False)
        dp('burst_s', 1.0)
        dp('burst_rate_hz', 20.0)
        dp('emergency_topic', '/cmd_vel_emergency')
        dp('cutter_off_service', '/cutter_off')
        dp('publish_markers', True)
        dp('dynamic_classes', list(DEFAULT_DYNAMIC))
        dp('policy_static_range_m', 1.5)
        dp('policy_topic', '/obstacle_policy')
        dp('policy_rate_hz', 5.0)
        # idle duty cycle: tick rate while /mission/activity is docked_idle|idle and nothing
        # is latched (0 = always burst_rate_hz). Back to burst_rate_hz on the next message.
        dp('activity_topic', '/mission/activity')
        dp('idle_tick_rate_hz', 2.0)
        # per-area level (mower_mission sets it live) + its thresholds
        dp('obstacle_detection', DEFAULT_LEVEL)
        dp('sensitive_stop_range_m', 1.5)
        dp('sensitive_min_score', 0.35)
        dp('unranged_h_frac', 0.45)
        dp('sensitive_static_unranged', False)   # sensitive: unranged boxes may be 'static'
        # motion relevance from the commanded velocity
        dp('cmd_vel_topic', '/cmd_vel')          # '' = every camera always counts
        dp('motion_lin_deadband_mps', 0.03)
        dp('motion_ang_deadband_rps', 0.15)
        dp('motion_hold_s', 0.6)
        # standard level side / front-unranged rules (sensitive keeps the old thresholds)
        dp('odom_topic', '/odom')                # side cameras need odom |w| > odom_turn_min
        dp('odom_turn_min_radps', 0.1)
        dp('side_turn_min_radps', 0.15)
        dp('side_turn_min_s', 0.5)
        dp('side_stop_range_m', 0.8)
        dp('side_unranged_h_frac', 0.60)
        dp('side_unranged_min_score', 0.75)
        dp('front_unranged_min_score', 0.6)
        dp('persist_frames', 2)
        dp('ignored_episode_gap_s', 3.0)         # INFO 'ignored: ...' once per episode
        dp('front_frames', list(DEFAULT_CAMERA_FRAMES[CAM_FRONT]))
        dp('left_frames', list(DEFAULT_CAMERA_FRAMES[CAM_LEFT]))
        dp('right_frames', list(DEFAULT_CAMERA_FRAMES[CAM_RIGHT]))
        dp('rear_frames', list(DEFAULT_CAMERA_FRAMES[CAM_REAR]))

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.classes = [str(c) for c in p('classes')]
        self.cfg = GuardConfig(whitelist=[str(w) for w in p('whitelist')],
                               y_frac=float(p('y_frac')), w_frac=float(p('w_frac')),
                               min_score=float(p('min_score')),
                               image_width=int(p('image_width')),
                               image_height=int(p('image_height')),
                               stop_range_m=float(p('obstacle_stop_range_m')))
        self.range_origin_x = float(p('range_origin_x_m'))
        self.ranged_timeout = float(p('ranged_timeout_s'))
        self._last_ranged = None
        self.state = GuardState(hold_s=float(p('hold_s')), burst_s=float(p('burst_s')))
        self.stop_on_close = bool(p('stop_on_close'))
        self.publish_markers = bool(p('publish_markers'))
        self.policy_cfg = PolicyConfig(dynamic_classes=[str(c) for c in p('dynamic_classes')],
                                       min_score=float(p('min_score')),
                                       dynamic_range_m=float(p('obstacle_stop_range_m')),
                                       static_range_m=float(p('policy_static_range_m')))
        self.level_table = LevelTable(
            standard_stop_range_m=float(p('obstacle_stop_range_m')),
            standard_min_score=float(p('min_score')),
            sensitive_stop_range_m=float(p('sensitive_stop_range_m')),
            sensitive_min_score=float(p('sensitive_min_score')),
            static_range_m=float(p('policy_static_range_m')),
            unranged_h_frac=float(p('unranged_h_frac')),
            sensitive_static_unranged=bool(p('sensitive_static_unranged')),
            side_stop_range_m=float(p('side_stop_range_m')),
            side_unranged_h_frac=float(p('side_unranged_h_frac')),
            side_unranged_min_score=float(p('side_unranged_min_score')),
            front_unranged_min_score=float(p('front_unranged_min_score')),
            persist_frames=int(p('persist_frames')))
        self.camera_frames = {CAM_FRONT: [str(x) for x in p('front_frames')],
                              CAM_LEFT: [str(x) for x in p('left_frames')],
                              CAM_RIGHT: [str(x) for x in p('right_frames')],
                              CAM_REAR: [str(x) for x in p('rear_frames')]}
        self.motion = MotionState(MotionConfig(
            lin_deadband_mps=float(p('motion_lin_deadband_mps')),
            ang_deadband_rps=float(p('motion_ang_deadband_rps')),
            hold_s=float(p('motion_hold_s')),
            side_turn_min_radps=float(p('side_turn_min_radps')),
            side_turn_min_s=float(p('side_turn_min_s')),
            odom_turn_min_radps=float(p('odom_turn_min_radps'))))
        self.persistence = Persistence()
        self._ignored_last = {}     # (camera, class) -> last time it was dismissed
        self._ignored_gap = float(p('ignored_episode_gap_s'))
        self.policy_state = PolicyState(hold_s=float(p('hold_s')))
        level = str(p('obstacle_detection'))
        self._apply_level(level if level in OBSTACLE_LEVELS else DEFAULT_LEVEL)
        self.add_on_set_parameters_callback(self._on_set_params)
        self._policy_period = 1.0 / max(0.5, float(p('policy_rate_hz')))
        self._policy_t = 0.0
        self._policy_last = None
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.policy_pub = self.create_publisher(String, p('policy_topic'), latched)

        self.close_pub = self.create_publisher(Bool, '/vision/obstacle_close', 10)
        self.marker_pub = self.create_publisher(ImageMarker, '/vision/obstacle_markers', 10)
        self.twist_pub = self.create_publisher(Twist, p('emergency_topic'), 10)
        self.cutter_cli = self.create_client(Trigger, p('cutter_off_service'))
        self._sizes = {}  # frame_id -> (w, h) from camera_info
        # camera_info only supplies the image size: unsubscribe once it is known (it was
        # deserialized at the camera rate forever).
        self._info_subs = {}
        for topic in p('camera_info_topics'):
            self._info_subs[str(topic)] = self.create_subscription(
                CameraInfo, str(topic),
                lambda m, _t=str(topic): self._on_info(m, _t), qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, p('detections_topic'), self._on_raw, 10)
        if p('ranged_topic'):
            self.create_subscription(Detection2DArray, p('ranged_topic'), self._on_ranged, 10)
        # /cmd_vel (20 Hz) and /odom (50 Hz) only feed the motion window that is read when a
        # detection frame or a tick is handled: take them in batches there (with their receive
        # times) instead of ~70 executor wakes/s, which were most of this node's idle CPU.
        self._pump = SubscriptionPump(self, 'obstacle_guard_inputs')
        self._motion_subs = []
        self._use_motion = bool(p('cmd_vel_topic'))
        if self._use_motion:
            self._motion_subs.append(self._pump.subscribe(
                Twist, p('cmd_vel_topic'), self._on_cmd_vel,
                QoSProfile(depth=64, reliability=QoSReliabilityPolicy.RELIABLE,
                           history=QoSHistoryPolicy.KEEP_LAST),
                raw=True, sampled=True, deliver_all=True, with_receipt=True))
        if str(p('odom_topic')):
            # only the newest angular rate is kept (MotionTracker.update_odom)
            self._motion_subs.append(self._pump.subscribe(
                Odometry, str(p('odom_topic')), self._on_odom, qos_profile_sensor_data,
                parser=parse_odometry, sampled=True, with_receipt=True))
        self._pump.start()
        self._burst_rate = float(p('burst_rate_hz'))
        self._idle_rate = float(p('idle_tick_rate_hz'))
        self._activity = None
        self._tick_rate = self._burst_rate
        self._timer = self.create_timer(1.0 / self._tick_rate, self._on_tick)
        if str(p('activity_topic')):
            self.create_subscription(String, str(p('activity_topic')), self._on_activity,
                                     latched)
        self._last_published = None

        self.get_logger().info(
            f'obstacle_guard up: whitelist={list(self.cfg.whitelist)} y_frac={self.cfg.y_frac} '
            f'w_frac={self.cfg.w_frac} fallback_image={self.cfg.image_width}x{self.cfg.image_height} '
            f'stop_on_close={self.stop_on_close} ranged={p("ranged_topic") or "off"} '
            f'stop_range={self.cfg.stop_range_m} m level={self.policy_cfg.level}')

    def _apply_level(self, level):
        """Switch the policy / guard thresholds to ``level`` (validated)."""
        self.policy_cfg = level_policy(level, self.policy_cfg, self.level_table)
        self.cfg = replace(self.cfg, stop_range_m=self.policy_cfg.dynamic_range_m,
                           min_score=self.policy_cfg.min_score)
        self.policy_state = PolicyState(hold_s=self.policy_state.hold_s)  # drop held ones

    def _on_set_params(self, params):
        for prm in params:
            if prm.name == 'obstacle_detection':
                lvl = str(prm.value)
                if lvl not in OBSTACLE_LEVELS:
                    return SetParametersResult(
                        successful=False,
                        reason='obstacle_detection: %s' % '|'.join(OBSTACLE_LEVELS))
                if lvl != self.policy_cfg.level:
                    self._apply_level(lvl)
                    self.get_logger().info(
                        'obstacle_detection=%s: stop range %.2f m, min score %.2f, %s' % (
                            lvl, self.policy_cfg.dynamic_range_m, self.policy_cfg.min_score,
                            'vision off (bumper + stereo costmap only)' if lvl == 'none'
                            else 'any camera' if self.policy_cfg.any_camera
                            else 'cameras by motion direction'))
        return SetParametersResult(successful=True)

    def _on_cmd_vel(self, data, receipt):
        if len(data) < 52 or data[0] != 0 or data[1] != 1:   # little-endian CDR Twist
            return
        v = struct.unpack_from('<6d', data, 4)
        self.motion.update(float(v[0]), float(v[5]), receipt)

    def _on_odom(self, msg, receipt):
        self.motion.update_odom(float(msg.twist.twist.angular.z), receipt)

    def _poll_motion(self):
        if self._motion_subs:
            self._pump.poll(self._motion_subs)

    def _log_ignored(self, role, ignored, now):
        """INFO once per episode (per camera + class) for a dismissed detection."""
        for b, reason in ignored:
            key = (role, b.label)
            last = self._ignored_last.get(key)
            self._ignored_last[key] = now
            if last is None or now - last > self._ignored_gap:
                self.get_logger().info('ignored: %s on %s camera (%s)'
                                       % (b.label, role, reason))

    def _on_info(self, msg, topic=None):
        w, h = int(msg.width), int(msg.height)
        if w <= 0 or h <= 0:
            return
        key = msg.header.frame_id
        if self._sizes.get(key) != (w, h):
            self._sizes[key] = (w, h)
            self.get_logger().info(f'image size for frame "{key}": {w}x{h} (camera_info)')
        sub = self._info_subs.pop(topic, None)
        if sub is not None:
            self.destroy_subscription(sub)

    def _on_activity(self, msg):
        self._activity = str(msg.data)
        self._retime()

    def _retime(self):
        now = self._now()
        busy = bool(self._last_published) or self.state.burst_active(now)
        rate = tick_rate_hz(self._activity, busy, self._burst_rate, self._idle_rate)
        if rate != self._tick_rate:
            self._tick_rate = rate
            self.destroy_timer(self._timer)
            self._timer = self.create_timer(1.0 / rate, self._on_tick)

    def _cfg_for(self, frame_id):
        size = self._sizes.get(frame_id)
        return with_image_size(self.cfg, *size) if size else self.cfg

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _publish_close(self, close: bool):
        self.close_pub.publish(Bool(data=bool(close)))
        self._last_published = close

    def _on_raw(self, msg):
        if (self._last_ranged is not None
                and self._now() - self._last_ranged <= self.ranged_timeout):
            return          # det_range is up: its republished copy is used instead
        self._on_dets(msg)

    def _on_ranged(self, msg):
        self._last_ranged = self._now()
        self._on_dets(msg)

    def _on_dets(self, msg):
        self._poll_motion()
        boxes = boxes_from_msg(msg, self.classes, self.range_origin_x)
        cfg = self._cfg_for(msg.header.frame_id)
        verdicts = classify(boxes, cfg)
        now = self._now()
        role = camera_role(msg.header.frame_id, self.camera_frames)
        relevant = self.motion.relevant(now) if self._use_motion else None
        source = msg.header.frame_id or 'camera'
        motion_reason = ''
        if role in (CAM_LEFT, CAM_RIGHT) and self._use_motion:
            motion_reason = self.motion.side_reason(role, now)
        elif relevant is not None and role not in relevant:
            motion_reason = 'not moving toward it'
        ignored = []
        pol = frame_policy(boxes, self.policy_cfg, cfg, role, relevant,
                           persistence=self.persistence, source=source, ignored=ignored,
                           motion_reason=motion_reason)
        self.policy_state.update(source, pol, now)
        if ignored:
            self._log_ignored(role, ignored, now)
        close_now = (any(c for _, _, c in verdicts)
                     and camera_counts(role, relevant, self.policy_cfg))
        close, rising = self.state.update(msg.header.frame_id or 'camera', close_now,
                                          self._now())
        self._publish_close(close)
        if close and self._tick_rate != self._burst_rate:
            self._retime()
        if rising:
            self._on_rising(verdicts)
        if self.publish_markers:
            self.marker_pub.publish(self._markers(msg.header, verdicts, cfg))

    def _publish_policy(self, now):
        pol = self.policy_state.current(now)
        if pol != self._policy_last or now - self._policy_t >= self._policy_period:
            def key(d):
                d = d or {}
                return d.get('kind'), d.get('class'), d.get('camera')
            if key(pol) != key(self._policy_last) and pol['kind'] != 'none':
                self.get_logger().info('obstacle policy: %s' % json.dumps(pol))
            self._policy_last, self._policy_t = pol, now
            self.policy_pub.publish(String(data=json.dumps(pol)))

    def _on_tick(self):
        self._poll_motion()
        now = self._now()
        self._publish_policy(now)
        close, rising = self.state.tick(now)
        if close != self._last_published and self._last_published is not None:
            self._publish_close(close)
        if rising:
            self._on_rising([])
        if self.stop_on_close and self.state.burst_active(now):
            self.twist_pub.publish(Twist())
        self._retime()

    def _on_rising(self, verdicts):
        what = ', '.join(f'{b.label}({b.score:.2f})' for b, _, c in verdicts if c) or 'held'
        if not self.stop_on_close:
            self.get_logger().warning(f'obstacle close: {what} (stop_on_close=false)')
            return
        self.get_logger().warning(
            f'obstacle close: {what} -> {self.state.burst_s:.1f}s zero burst + cutter off')
        self.twist_pub.publish(Twist())
        if self.cutter_cli.service_is_ready():
            self.cutter_cli.call_async(Trigger.Request())
        else:
            self.get_logger().error('cutter_off service not available')

    def _markers(self, header, verdicts, cfg):
        m = ImageMarker()
        m.header = header
        m.ns = 'obstacle_guard'
        m.id = 0
        m.type = ImageMarker.LINE_LIST
        m.action = ImageMarker.ADD
        m.scale = 3.0
        m.outline_color = BLUE
        pts, cols = [], []
        for x, y in danger_line(cfg):
            pts.append(Point(x=x, y=y))
            cols.append(BLUE)
        for box, relevant, close in verdicts:
            colour = RED if close else (YELLOW if relevant else GREY)
            for x, y in box_outline(box):
                pts.append(Point(x=float(x), y=float(y)))
                cols.append(colour)
        m.points = pts
        m.outline_colors = cols
        m.lifetime.nanosec = 500_000_000
        return m


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('obstacle_guard')
    except ImportError:
        pass
    if _DEP_ERROR is not None:
        print(f'[obstacle_guard] FATAL: {_DEP_ERROR}; install ros-humble-vision-msgs.',
              file=sys.stderr)
        sys.exit(2)
    rclpy.init(args=args)
    node = ObstacleGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
