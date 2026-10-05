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
                 backend='v4l2', name='capture'):
        super().__init__(daemon=True, name=name)
        self.log = logger
        self.device, self.width, self.height = device, int(width), int(height)
        self.pixel_format, self.fps, self.backend = pixel_format, float(fps), backend
        self.on_frame = on_frame
        self.stop_evt = threading.Event()
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

    def run(self):
        backoff, fails, last_pub = 1.0, 0, 0.0
        period = 1.0 / self.fps if self.fps > 0 else 0.0
        while not self.stop_evt.is_set():
            if self.cap is None:
                try:
                    self.cap = self._open()
                    backoff = 1.0
                except Exception as exc:  # noqa: BLE001
                    self.log.error(f'{self.device}: open failed: {exc}; retry in {backoff:.0f}s')
                    self.stop_evt.wait(backoff)
                    backoff = min(backoff * 2, 30.0)
                    continue
            try:
                if self.backend == 'opencv':
                    ok, data = self.cap.read()
                    if not ok:
                        raise OSError('cv2 read() returned False')
                else:
                    data, _seq = self.cap.read(timeout=2.0)
                fails = 0
            except Exception as exc:  # noqa: BLE001
                fails += 1
                self.log.warning(f'{self.device}: read failed ({fails}): {exc}')
                if fails >= 3:
                    self.log.warning(f'{self.device}: re-opening')
                    self._close()
                    self.stop_evt.wait(backoff)
                    backoff = min(backoff * 2, 30.0)
                continue
            now = time.monotonic()
            if period and now - last_pub < period * 0.9:
                continue
            last_pub = now
            try:
                self.on_frame(data, self.cap)
                if self.frames == 0:
                    self.log.info(f'{self.device}: streaming')
                self.frames += 1
            except Exception as exc:  # noqa: BLE001
                self.log.error(f'{self.device}: frame handling failed: {exc}')
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
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.frame_id = p('frame_id') or self.get_namespace().strip('/') or 'camera'
        self.publish_raw = bool(p('publish_raw'))
        self.jpeg_quality = int(p('jpeg_quality'))
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
                                name=f'cap:{self.frame_id}')
        self.loop.start()
        self.create_timer(10.0, self._report)
        self._last_frames, self._last_t = 0, time.monotonic()
        self.get_logger().info(
            f'v4l2_cam {p("video_device")} {p("width")}x{p("height")} {p("pixel_format")} '
            f'fps<={p("fps")} raw={self.publish_raw} compressed={self.comp_pub is not None}')

    def _on_frame(self, data, cap):
        import cv2
        stamp = self.get_clock().now().to_msg()
        fmt = cap.pixel_format
        frame = None
        if self.raw_pub is not None or (self.comp_pub is not None and fmt != 'MJPG'):
            frame = v4l2.to_bgr(data, cap.width, cap.height, cap.bytesperline, fmt)
            if frame is None:
                raise ValueError('undecodable frame')
        if self.raw_pub is not None:
            self.raw_pub.publish(to_image_msg(stamp, self.frame_id, frame))
        if self.comp_pub is not None:
            if fmt == 'MJPG':
                jpeg = data
            else:
                ok, buf = cv2.imencode('.jpg', frame,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                jpeg = buf.tobytes() if ok else None
            if jpeg is not None:
                self.comp_pub.publish(to_compressed_msg(stamp, self.frame_id, jpeg))
        if self.info is not None:
            if (self.info.width, self.info.height) == (cap.width, cap.height):
                self.info.header.stamp = stamp
                self.info_pub.publish(self.info)
            elif not getattr(self, '_info_warned', False):
                self._info_warned = True
                self.get_logger().warning(
                    f'camera_info is {self.info.width}x{self.info.height} but capture is '
                    f'{cap.width}x{cap.height}: not publishing camera_info')

    def _report(self):
        now, n = time.monotonic(), self.loop.frames
        rate = (n - self._last_frames) / (now - self._last_t)
        self._last_frames, self._last_t = n, now
        if rate < 1.0:
            self.get_logger().warning(f'{self.frame_id}: {rate:.1f} fps published (stalled?)')

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
