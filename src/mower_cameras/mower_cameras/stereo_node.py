"""``mower_cameras/stereo_cam`` — Metoak front stereo -> synchronised mono8 pair (+ optional colour).

The Metoak module delivers ONE side-by-side frame per exposure on ``/dev/video22``
(``/dev/videoIsp``, rkcif-mipi-lvds3, behind the XC9080 bridge: YUYV/YVYU 1280x480, two
640x480 eyes side by side; the bridge subdev reports 30 fps, the trigger IRQ runs ~25 Hz).
``/dev/video11`` (``/dev/videoSimor``) is the hardware-disparity stream used by
``stereo_depth``; it is a separate video node, so the two nodes never contend.

Output (frame ``vio_camera``, sensor-data QoS):

* ``/vio/left/image_raw`` + ``/vio/right/image_raw``  mono8 640x480, BOTH eyes cut from the
  SAME captured buffer and published with the SAME header stamp (an exact-time pair for
  OpenVINS / message_filters). The stamp is the V4L2 buffer timestamp (kernel
  CLOCK_MONOTONIC) mapped to ROS time, so it does not include our Python/DDS latency and
  shares its clock with ``stereo_imu`` (IIO timestamps, same mapping).
* ``/vio/{left,right}/camera_info``  same stamp as the pair (if a file is configured).
* ``/vio/{left,right}/image_color``  bgr8, only with ``publish_color: true``, at
  ``color_fps`` (default 2 Hz) and only for subscribed eyes. det_ros (colour YOLO on the
  right eye) uses this; the GUI Perception page shows the mono8 ``image_raw`` topics.

Rate: ``fps`` (default 10) is an average-rate decimation of the source driven by the buffer
timestamps (:class:`mower_cameras.stereo_pair.DecimationGate`), so a 25 Hz source gives 10 Hz
(2 of 5 frames), not the 8.3 Hz a "0.9 x period since last" cap would give.

Why the old node managed only 2.6/2.1 Hz unsynchronised: per eye it ran a full YUYV->BGR
``cvtColor`` in the capture thread, then built a Python ``Image`` (921 KB ``array`` copy) and
let rclpy serialise it, in a second thread with a 1-slot "newest wins" hand-off that could
replace a half-published pair; under load (det_ros, foxglove) the eyes drifted apart and
frames were dropped. Now the capture thread only copies the Y plane (2 x 300 KB strided
``np.copyto``) straight into two pre-serialised Image buffers
(:mod:`mower_cameras.image_cdr`) and the publisher thread sends bytes; a pair is handed over
as one unit.

With ``publish_on_demand`` (default) nothing is copied unless one of the topics has a
subscriber. Never run this together with ``stereo_vio_bridge`` (same device, same topics).
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from mower_cameras.activity_gate import ActivityWatch, capture_period
from mower_cameras.image_cdr import ImageCdr
from mower_cameras.stereo_pair import ClockMap, DecimationGate, split_luma_into
from mower_cameras.v4l2 import eye_to_bgr
from mower_cameras.v4l2_node import CaptureLoop, to_image_msg

STEREO_FRAME_ID = 'vio_camera'


def _ns_to_stamp(ns):
    from builtin_interfaces.msg import Time
    sec, nsec = divmod(int(ns), 1_000_000_000)
    return Time(sec=sec, nanosec=nsec)


class StereoCamNode(Node):
    def __init__(self):
        super().__init__('stereo_cam')
        d = self.declare_parameter
        d('video_device', '/dev/video22')
        d('width', 1280)                # full side-by-side width (each eye = width/2)
        d('height', 480)
        d('pixel_format', 'YUYV')
        d('fps', 10.0)                  # average pair rate (0 = every source frame)
        d('frame_id', STEREO_FRAME_ID)
        d('left_topic', '/vio/left/image_raw')
        d('right_topic', '/vio/right/image_raw')
        d('left_camera_info_file', '')
        d('right_camera_info_file', '')
        d('publish_on_demand', True)
        d('close_when_unused_s', 5.0)   # on_demand: close the device after this long unused
        d('publish_color', False)       # bgr8 on <eye>/image_color (det_ros, debugging)
        d('color_fps', 2.0)
        d('stamp_source', 'buffer')     # buffer (V4L2 driver timestamp) | now (host receive)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.frame_id = p('frame_id')
        self.on_demand = bool(p('publish_on_demand'))
        self.publish_color = bool(p('publish_color'))
        self.stamp_from_buffer = p('stamp_source') == 'buffer'
        qos = rclpy.qos.qos_profile_sensor_data
        lt, rt = p('left_topic'), p('right_topic')
        ns = [lt.rsplit('/', 1)[0], rt.rsplit('/', 1)[0]]
        self.pubs = [self.create_publisher(Image, lt, qos),
                     self.create_publisher(Image, rt, qos)]
        self.info_pubs = [self.create_publisher(CameraInfo, n + '/camera_info', qos)
                          for n in ns]
        self.color_pubs = ([self.create_publisher(Image, n + '/image_color', qos) for n in ns]
                           if self.publish_color else [])
        self.infos = [self._load_info(p('left_camera_info_file')),
                      self._load_info(p('right_camera_info_file'))]
        fps = float(p('fps'))
        # tolerance = 25 ms ~ half a 25-30 Hz source interval
        self.gate = DecimationGate(1.0 / fps if fps > 0 else 0.0, 0.025)
        cf = float(p('color_fps'))
        self.color_gate = DecimationGate(1.0 / cf if cf > 0 else 0.0, 0.025)
        self._cdr = None                # (ImageCdr left, ImageCdr right) for the eye size
        self.clock = ClockMap()
        self.fast = True                # Publisher.publish(bytes); falls back on TypeError
        self.pairs = 0
        self.source_frames = 0
        self.replaced = 0               # pairs overwritten before the publisher got them
        self._slot = None
        self._slot_cv = threading.Condition()
        self._stop = False
        self._pub_thread = threading.Thread(target=self._publish_loop, name='pub:stereo',
                                            daemon=True)
        self._pub_thread.start()
        # fps=0: the loop hands over every frame; the gate below decimates on buffer stamps.
        self.loop = CaptureLoop(self.get_logger(), p('video_device'), p('width'), p('height'),
                                p('pixel_format'), 0.0, self._on_frame, 'v4l2',
                                name='cap:stereo',
                                want=self._wanted if self.on_demand else None,
                                close_when_unused_s=p('close_when_unused_s'))
        # idle (docked / parked): 1 pair/s (stereo_depth, det_ros follow); full rate again
        # on the next source frame after /mission/activity leaves idle
        self.activity = ActivityWatch(lambda a: self.get_logger().info(
            f'stereo_cam: activity {a} (idle cap {"on" if self.activity.low_power else "off"})'))
        self.activity.subscribe(self)
        self.loop.period_fn = lambda: capture_period(0.0, self.activity.low_power)
        self.loop.start()
        self._t_stats = time.monotonic()
        self.create_timer(30.0, self._log_stats)
        self.get_logger().info(
            f'stereo_cam {p("video_device")} {p("width")}x{p("height")} {p("pixel_format")} '
            f'-> {lt}, {rt} (mono8 pair, same stamp) @ {fps:g} Hz, '
            f'colour={"on @ %g Hz" % cf if self.publish_color else "off"}, '
            f'stamp={p("stamp_source")}, on_demand={self.on_demand}')

    def _load_info(self, path):
        path = str(path).replace('file://', '', 1)
        if not path:
            return None
        from mower_cameras.camera_node import load_camera_info
        try:
            return load_camera_info(path, self.frame_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'camera_info not loaded ({path}): {exc}')
            return None

    @staticmethod
    def _subscribed(pub):
        return pub.get_subscription_count() > 0

    def _wanted(self):
        return (any(self._subscribed(x) for x in self.pubs)
                or any(self._subscribed(x) for x in self.color_pubs)
                or any(info is not None and self._subscribed(pub)
                       for info, pub in zip(self.infos, self.info_pubs)))

    def _on_frame(self, data, cap):
        """Capture thread: decimate on the buffer stamp, cut both eyes, hand over the pair."""
        self.source_frames += 1
        mono_now = time.monotonic_ns()
        ts = cap.last_timestamp_ns if self.stamp_from_buffer else 0
        if not ts or abs(mono_now - ts) > 5_000_000_000:   # missing / not CLOCK_MONOTONIC
            ts = mono_now
        if not self.gate.keep(ts * 1e-9):
            return
        stamp_ns = self.clock.to_ros(ts)
        half = cap.width // 2
        bpl = cap.bytesperline or cap.width * 2
        if self._cdr is None or self._cdr[0].width != half:
            self._cdr = (ImageCdr(self.frame_id, cap.height, half, 'mono8', 1),
                         ImageCdr(self.frame_id, cap.height, half, 'mono8', 1))
        left, right = self._cdr
        if not split_luma_into(data, cap.width, cap.height, bpl, cap.pixel_format,
                               left.image, right.image):
            return
        mono = (left.serialize(stamp_ns), right.serialize(stamp_ns))   # bytes copies
        colour = None
        if self.color_pubs and self.color_gate.keep(ts * 1e-9):
            colour = [eye_to_bgr(data, cap.width, cap.height, bpl, cap.pixel_format, i)
                      if self._subscribed(pub) else None
                      for i, pub in enumerate(self.color_pubs)]
        with self._slot_cv:
            if self._slot is not None:
                self.replaced += 1
            self._slot = (stamp_ns, mono, colour)   # one unit: both eyes or nothing
            self._slot_cv.notify()

    def _publish_loop(self):
        while True:
            with self._slot_cv:
                while self._slot is None and not self._stop:
                    self._slot_cv.wait(0.5)
                if self._stop:
                    return
                stamp_ns, mono, colour = self._slot
                self._slot = None
            try:
                self._publish_pair(stamp_ns, mono, colour)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'publish failed: {exc}')

    def _publish_pair(self, stamp_ns, mono, colour):
        for payload, pub in zip(mono, self.pubs):
            if self.fast:
                try:
                    pub.publish(payload)
                    continue
                except TypeError:       # rclpy without publish(bytes)
                    self.fast = False
            from rclpy.serialization import deserialize_message
            pub.publish(deserialize_message(payload, Image))
        stamp = _ns_to_stamp(stamp_ns)
        for info, info_pub in zip(self.infos, self.info_pubs):
            if info is not None:
                info.header.stamp = stamp
                info_pub.publish(info)
        if colour:
            for img, pub in zip(colour, self.color_pubs):
                if img is not None:
                    pub.publish(to_image_msg(stamp, self.frame_id, img))
        self.pairs += 1

    def _log_stats(self):
        now = time.monotonic()
        dt, self._t_stats = now - self._t_stats, now
        if self.loop.paused and not self.pairs:
            return      # device closed: nobody subscribes
        self.get_logger().info(
            f'pairs {self.pairs / dt:.1f} Hz (source {self.source_frames / dt:.1f} Hz, '
            f'replaced {self.replaced})')
        self.pairs = self.source_frames = self.replaced = 0

    def destroy_node(self):
        self.loop.stop()
        self.loop.join(timeout=3.0)
        with self._slot_cv:
            self._stop = True
            self._slot_cv.notify()
        self._pub_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
