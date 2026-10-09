"""``mower_cameras/v4l2_cam`` — one V4L2 camera -> ``image_raw`` (bgr8) + ``camera_info``.

Canonical producer for the OA cameras (``/left_oa_camera``, ``/right_oa_camera``; see
``launch/cameras.launch.py``). Run it in the camera's namespace; topics are relative:

* ``image_raw``            sensor_msgs/Image, bgr8 (``publish_raw``)
* ``image_raw/compressed`` sensor_msgs/CompressedImage (``publish_compressed``; MJPG
  sources are passed through without re-encoding)
* ``camera_info``          from ``camera_info_file`` (only if its size matches the capture)

Capture runs in a dedicated thread (never blocks the executor), uses the pure-Python
V4L2 reader in ``mower_cameras.v4l2`` (single- and multi-planar, so it works on the
rkisp ``rkisp_mainpath`` nodes that ``v4l2_camera`` cannot drive), logs every failure and
re-opens the device with back-off. ``fps`` throttles publishing (frames above the rate
are dropped *before* colour conversion); 0 publishes every frame.

CPU (RK3588, 1080p UYVY): the colour conversion is the cost, so

* ``publish_on_demand`` (default true): a frame is only converted/published while
  ``image_raw`` (or ``compressed`` / ``camera_info``) has subscribers (det_ros, seg_ros,
  web_video_server and foxglove subscribe on demand); otherwise the buffer is dequeued and
  given straight back, never copied. After ``close_when_unused_s`` (5 s) without any
  subscriber the device itself is closed (no sensor/ISP stream, no 3A, no dequeue) and it
  is reopened on the next subscriber (polled every 0.2 s); at start-up it is not opened
  until somebody subscribes;
* frames are converted straight from the mmap'ed V4L2 buffer into a pre-serialized Image
  (:mod:`mower_cameras.image_cdr`) and published as bytes (``fast_publish``);
* ``publish_width`` (0 = native) decimates packed 4:2:2 input by an integer factor
  *before* conversion (1920 -> 960: ~1/4 of the work); ``camera_info`` is scaled to match.
"""
from __future__ import annotations

import array
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from mower_cameras import v4l2
from mower_cameras.activity_gate import ActivityWatch, capture_period
from mower_cameras.image_cdr import ImageCdr


def to_image_msg(stamp, frame_id, frame):
    """BGR/mono numpy image -> ``Image`` without the per-element rclpy setter check.

    Assigning ``bytes`` to ``Image.data`` makes the Humble rclpy setter validate every
    element in Python (0.998 s for a 1080p bgr8 frame, measured on the mower); an
    ``array.array('B')`` is accepted as-is (0.005 s).
    """
    import numpy as np
    if frame.ndim == 2 or frame.shape[2] == 1:
        encoding, channels = 'mono8', 1
    else:
        encoding, channels = 'bgr8', 3
    frame = np.ascontiguousarray(frame)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(frame.shape[0])
    msg.width = int(frame.shape[1])
    msg.encoding = encoding
    msg.step = msg.width * channels
    data = array.array('B')
    data.frombytes(frame.data)
    msg.data = data
    return msg


def to_compressed_msg(stamp, frame_id, jpeg_bytes):
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = 'jpeg'
    data = array.array('B')
    data.frombytes(jpeg_bytes)
    msg.data = data
    return msg


class CaptureLoop(threading.Thread):
    """Background capture with reconnect. Calls ``on_frame(data, cap)`` per kept frame.

    ``backend``: ``v4l2`` (``mower_cameras.v4l2``, raw buffers) or ``opencv``
    (``cv2.VideoCapture(CAP_V4L2)``, single-planar only; ``data`` is then a BGR image).
    """

    def __init__(self, logger, device, width, height, pixel_format, fps, on_frame,
                 backend='v4l2', name='capture', want=None, close_when_unused_s=0.0):
        super().__init__(daemon=True, name=name)
        self.log = logger
        self.device, self.width, self.height = device, int(width), int(height)
        self.pixel_format, self.fps, self.backend = pixel_format, float(fps), backend
        self.on_frame = on_frame
        self.want = want            # () -> bool: is anybody interested in this frame?
        self.period_fn = None       # () -> float: overrides the publish period (idle cap)
        # () -> bool: True = device closed, no capture at all (polled every pause_poll_s)
        self.pause_fn = None
        self.pause_poll_s = 0.2
        self.paused = False
        # With ``want``: close the device once nobody has wanted a frame for this long
        # (0 = keep streaming and only skip frames). Reopened on the next poll that sees a
        # subscriber. The device also stays closed at start-up until somebody subscribes.
        self.close_when_unused_s = float(close_when_unused_s or 0.0)
        self._unwanted_since = None
        self.unused = False         # closed because nobody subscribes (vs pause_fn)
        self.stop_evt = threading.Event()
        self.skipped = 0
        self.captured = 0           # frames dequeued (published or skipped)
        self.frames = 0
        self.cap = None

    def stop(self):
        self.stop_evt.set()

    def _open(self):
        if self.backend == 'opencv':
            import cv2
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise OSError(f'cv2 could not open {self.device}')
            # FOURCC before size. Do NOT set CAP_PROP_BUFFERSIZE=1: on the rear UVC cam it
            # halves the rate (13.5 vs 27.9 fps measured).
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format[:4]))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if self.fps:
                cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.log.info(f'{self.device}: opencv {int(cap.get(3))}x{int(cap.get(4))} '
                          f'{self.pixel_format}')
            return cap
        cap = v4l2.V4L2Capture(self.device, self.width, self.height, self.pixel_format,
                               self.fps)
        self.log.info(f'{self.device}: {cap.driver} "{cap.card}" '
                      f'{"mplane" if cap.mplane else "single-plane"} '
                      f'{cap.width}x{cap.height} {cap.pixel_format} bpl={cap.bytesperline}')
        return cap

    def _close(self):
        if self.cap is not None:
            try:
                self.cap.release() if self.backend == 'opencv' else self.cap.close()
            except Exception:  # noqa: BLE001
                pass
            self.cap = None

    def _closed_for_unused(self, now):
        """True while the device should stay closed because nobody wants frames.

        Never-wanted-yet counts as unused from the start (no open at boot); once wanted,
        the device is kept for ``close_when_unused_s`` after the last subscriber left, so
        a viewer that reconnects (or a 1 Hz snapshot poller) does not cycle the device.
        """
        if self.want is None or self.close_when_unused_s <= 0.0:
            return False
        if self.want():
            self._unwanted_since = None
            return False
        if self.cap is None:
            return True             # not open (start-up / closed): stay closed
        if self._unwanted_since is None:
            self._unwanted_since = now
        return now - self._unwanted_since >= self.close_when_unused_s

    def _keep(self, last_pub, period):
        """Decide before touching the pixels: rate cap, then subscriber interest."""
        now = time.monotonic()
        if self.period_fn is not None:
            period = self.period_fn()
        if period and now - last_pub < period * 0.9:
            return None
        if self.want is not None and not self.want():
            self.skipped += 1
            return None
        return now

    def run(self):
        backoff, fails, last_pub = 1.0, 0, 0.0
        period = 1.0 / self.fps if self.fps > 0 else 0.0
        while not self.stop_evt.is_set():
            forced = self.pause_fn is not None and self.pause_fn()
            unused = not forced and self._closed_for_unused(time.monotonic())
            if forced or unused:
                if not self.paused or self.unused != unused:
                    self.paused, self.unused = True, unused
                    was_open = self.cap is not None
                    self._close()
                    if unused:
                        self.log.info(f'{self.device}: no subscribers'
                                      f'{" (device closed)" if was_open else ""}; '
                                      'opening on demand')
                    else:
                        self.log.info(f'{self.device}: paused (device closed)')
                self.stop_evt.wait(self.pause_poll_s)
                continue
            if self.paused:
                self.paused, self.unused = False, False
                self.log.info(f'{self.device}: resuming')
            if self.cap is None:
                try:
                    self.cap = self._open()
                    backoff = 1.0
                except Exception as exc:  # noqa: BLE001
                    self.log.error(f'{self.device}: open failed: {exc}; retry in {backoff:.0f}s')
                    self.stop_evt.wait(backoff)
                    backoff = min(backoff * 2, 30.0)
                    continue
            index = None
            try:
                if self.backend == 'opencv':
                    if not self.cap.grab():          # no decode until retrieve()
                        raise OSError('cv2 grab() returned False')
                else:
                    index, used, _seq = self.cap.dequeue(timeout=2.0)
                fails = 0
                self.captured += 1
            except Exception as exc:  # noqa: BLE001
                fails += 1
                self.log.warning(f'{self.device}: read failed ({fails}): {exc}')
                if fails >= 3:
                    self.log.warning(f'{self.device}: re-opening')
                    self._close()
                    self.stop_evt.wait(backoff)
                    backoff = min(backoff * 2, 30.0)
                continue
            try:
                now = self._keep(last_pub, period)
                if now is None:
                    continue
                if self.backend == 'opencv':
                    ok, data = self.cap.retrieve()
                    if not ok:
                        continue
                else:
                    data = self.cap.view(index, used)   # zero-copy, valid until requeue
                last_pub = now
                try:
                    self.on_frame(data, self.cap)
                    if self.frames == 0:
                        self.log.info(f'{self.device}: streaming')
                    self.frames += 1
                except Exception as exc:  # noqa: BLE001
                    self.log.error(f'{self.device}: frame handling failed: {exc}')
                finally:
                    data = None
            finally:
                if index is not None:
                    try:
                        self.cap.requeue(index)
                    except OSError as exc:
                        self.log.warning(f'{self.device}: requeue failed: {exc}; re-opening')
                        self._close()
        self._close()


class V4L2CamNode(Node):
    def __init__(self):
        super().__init__('v4l2_cam')
        d = self.declare_parameter
        d('video_device', '/dev/left_oa_camera')
        d('width', 1920)
        d('height', 1080)
        d('pixel_format', 'UYVY')       # UYVY | YUYV | NV12 | NV21 | MJPG | GREY
        d('fps', 15.0)                  # publish rate cap; 0 = every frame
        d('frame_id', '')
        d('camera_info_file', '')       # camera_info_manager YAML (path or file:// URL)
        d('publish_raw', True)
        d('publish_compressed', False)
        d('jpeg_quality', 80)
        d('publish_on_demand', True)    # convert/publish only while somebody subscribes
        d('publish_width', 0)           # 0 = native; else width/publish_width must be an int
        d('fast_publish', True)         # pre-serialized Image (bytes) publishing
        # on_demand: close the device (sensor/ISP stream off) after this many seconds
        # without subscribers; reopened on the next subscriber (0 = keep it streaming)
        d('close_when_unused_s', 5.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.frame_id = p('frame_id') or self.get_namespace().strip('/') or 'camera'
        self.publish_raw = bool(p('publish_raw'))
        self.jpeg_quality = int(p('jpeg_quality'))
        self.on_demand = bool(p('publish_on_demand'))
        self.fast_publish = bool(p('fast_publish'))
        pw = int(p('publish_width'))
        self.scale = 1
        if 0 < pw < int(p('width')):
            if int(p('width')) % pw:
                self.get_logger().warning(f'publish_width {pw} does not divide width '
                                          f'{p("width")}: publishing native size')
            else:
                self.scale = int(p('width')) // pw
        self._img = None            # ImageCdr for the current output size
        self._info_out = None       # camera_info scaled to the output size (or None)
        self._info_wh = None
        qos = rclpy.qos.qos_profile_sensor_data
        self.raw_pub = self.create_publisher(Image, 'image_raw', qos) if self.publish_raw \
            else None
        self.comp_pub = self.create_publisher(CompressedImage, 'image_raw/compressed', qos) \
            if p('publish_compressed') else None
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', qos)
        self.info = None
        info_path = str(p('camera_info_file')).replace('file://', '', 1)
        if info_path:
            from mower_cameras.camera_node import load_camera_info
            try:
                self.info = load_camera_info(info_path, self.frame_id)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'camera_info not loaded ({info_path}): {exc}')

        self.loop = CaptureLoop(self.get_logger(), p('video_device'), p('width'), p('height'),
                                p('pixel_format'), p('fps'), self._on_frame, 'v4l2',
                                name=f'cap:{self.frame_id}',
                                want=self._wanted if self.on_demand else None,
                                close_when_unused_s=p('close_when_unused_s'))
        # idle (docked / parked): 1 fps unless a GUI client watches the compressed stream
        self._base_period = 1.0 / float(p('fps')) if float(p('fps')) > 0 else 0.0
        self.activity = ActivityWatch(lambda a: self.get_logger().info(
            f'{self.frame_id}: activity {a} -> period {self._period():.2f} s'))
        self.activity.subscribe(self)
        self.loop.period_fn = self._period
        self.loop.start()
        self.create_timer(10.0, self._report)
        self._last_frames, self._last_t = 0, time.monotonic()
        self.get_logger().info(
            f'v4l2_cam {p("video_device")} {p("width")}x{p("height")} {p("pixel_format")} '
            f'fps<={p("fps")} raw={self.publish_raw} compressed={self.comp_pub is not None} '
            f'on_demand={self.on_demand} scale=1/{self.scale} fast={self.fast_publish}')

    @staticmethod
    def _subscribed(pub):
        return pub is not None and pub.get_subscription_count() > 0

    def _period(self):
        return capture_period(self._base_period, self.activity.low_power,
                              self._subscribed(self.comp_pub))

    def _wanted(self):
        return (self._subscribed(self.raw_pub) or self._subscribed(self.comp_pub)
                or (self.info is not None and self._subscribed(self.info_pub)))

    def _publish_raw(self, stamp_ns, data, cap):
        """Convert ``data`` straight into the pre-serialized Image and publish it."""
        ow, oh = v4l2.output_size(cap.width, cap.height, cap.pixel_format, self.scale)
        img = self._img
        if img is None or (img.width, img.height) != (ow, oh):
            img = self._img = ImageCdr(self.frame_id, oh, ow)
        frame = v4l2.to_bgr(data, cap.width, cap.height, cap.bytesperline, cap.pixel_format,
                            dst=img.image, scale=self.scale)
        if frame is None:
            raise ValueError('undecodable frame')
        if frame.ndim != 3 or frame.shape != img.image.shape:
            return frame, False     # mono etc.: typed path
        if frame.ctypes.data != img.image.ctypes.data:
            img.image[...] = frame
        self.raw_pub.publish(img.serialize(stamp_ns))
        return img.image, True

    def _on_frame(self, data, cap):
        import cv2
        now = self.get_clock().now()
        stamp = now.to_msg()
        fmt = cap.pixel_format
        od = self.on_demand
        want_raw = self.raw_pub is not None and (not od or self._subscribed(self.raw_pub))
        want_comp = self.comp_pub is not None and (not od or self._subscribed(self.comp_pub))
        frame = None
        if want_raw:
            published = False
            if self.fast_publish:
                try:
                    frame, published = self._publish_raw(now.nanoseconds, data, cap)
                except TypeError:   # rclpy without publish(bytes)
                    self.fast_publish = False
            if not published:
                if frame is None:
                    frame = v4l2.to_bgr(data, cap.width, cap.height, cap.bytesperline, fmt,
                                        scale=self.scale)
                    if frame is None:
                        raise ValueError('undecodable frame')
                self.raw_pub.publish(to_image_msg(stamp, self.frame_id, frame))
        if want_comp:
            if fmt == 'MJPG' and self.scale == 1:
                jpeg = data
            else:
                if frame is None:
                    frame = v4l2.to_bgr(data, cap.width, cap.height, cap.bytesperline, fmt,
                                        scale=self.scale)
                    if frame is None:
                        raise ValueError('undecodable frame')
                ok, buf = cv2.imencode('.jpg', frame,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                jpeg = buf.tobytes() if ok else None
            if jpeg is not None:
                self.comp_pub.publish(to_compressed_msg(stamp, self.frame_id, jpeg))
        if self.info is not None and (not od or want_raw or want_comp
                                      or self._subscribed(self.info_pub)):
            wh = v4l2.output_size(cap.width, cap.height, fmt, self.scale)
            if wh != self._info_wh:
                self._info_wh = wh
                from mower_cameras.camera_node import scale_camera_info
                self._info_out = scale_camera_info(self.info, *wh)
                if self._info_out is None:
                    self.get_logger().warning(
                        f'camera_info is {self.info.width}x{self.info.height} but the output '
                        f'is {wh[0]}x{wh[1]} (different aspect): not publishing camera_info')
                elif wh != (self.info.width, self.info.height):
                    self.get_logger().info(f'camera_info scaled to {wh[0]}x{wh[1]}')
            if self._info_out is not None:
                self._info_out.header.stamp = stamp
                self.info_pub.publish(self._info_out)

    def _report(self):
        now, n = time.monotonic(), self.loop.captured
        rate = (n - self._last_frames) / (now - self._last_t)
        self._last_frames, self._last_t = n, now
        if rate < 1.0 and not self.loop.paused:   # closed on purpose: no subscribers
            self.get_logger().warning(f'{self.frame_id}: {rate:.1f} fps captured (stalled?)')

    def destroy_node(self):
        self.loop.stop()
        self.loop.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = V4L2CamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
