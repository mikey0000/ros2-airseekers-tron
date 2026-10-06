"""Minimal pure-Python V4L2 capture (ioctl + mmap), single- and multi-planar.

Why this exists (verified on the mower, 2026-10-06):

* The OA cameras (``/dev/left_oa_camera`` -> video53, ``/dev/right_oa_camera`` -> video44)
  are Rockchip ``rkisp_mainpath`` nodes, which are **Video Capture Multiplanar** only.
  ``v4l2_camera`` 0.6 (Humble) and OpenCV 4.5.4's V4L2 backend only speak single-planar
  ``V4L2_BUF_TYPE_VIDEO_CAPTURE``: v4l2_camera then sees no formats, requests "0x0 UYVY"
  (EINVAL) and fails with "Failed mapping device memory".
* The container has no GStreamer plugins (only coreelements), so ``v4l2src`` is not an option.

This module needs only the stdlib + numpy/cv2 for conversion. It opens the device
non-blocking and waits with ``select`` so a stalled stream raises ``TimeoutError`` instead
of blocking forever.
"""
from __future__ import annotations

import ctypes as C
import errno
import fcntl
import mmap
import os
import select

# --------------------------------------------------------------------------- constants
BUF_TYPE_VIDEO_CAPTURE = 1
BUF_TYPE_VIDEO_CAPTURE_MPLANE = 9
MEMORY_MMAP = 1
FIELD_NONE = 1
CAP_VIDEO_CAPTURE = 0x00000001
CAP_VIDEO_CAPTURE_MPLANE = 0x00001000
CAP_DEVICE_CAPS = 0x80000000
VIDEO_MAX_PLANES = 8


def fourcc(code: str) -> int:
    code = (code + '    ')[:4]
    return ord(code[0]) | ord(code[1]) << 8 | ord(code[2]) << 16 | ord(code[3]) << 24


def fourcc_str(value: int) -> str:
    return ''.join(chr((value >> (8 * i)) & 0xFF) for i in range(4))


# --------------------------------------------------------------------------- structs
class v4l2_capability(C.Structure):
    _fields_ = [('driver', C.c_char * 16), ('card', C.c_char * 32),
                ('bus_info', C.c_char * 32), ('version', C.c_uint32),
                ('capabilities', C.c_uint32), ('device_caps', C.c_uint32),
                ('reserved', C.c_uint32 * 3)]


class v4l2_pix_format(C.Structure):
    _fields_ = [('width', C.c_uint32), ('height', C.c_uint32), ('pixelformat', C.c_uint32),
                ('field', C.c_uint32), ('bytesperline', C.c_uint32), ('sizeimage', C.c_uint32),
                ('colorspace', C.c_uint32), ('priv', C.c_uint32), ('flags', C.c_uint32),
                ('ycbcr_enc', C.c_uint32), ('quantization', C.c_uint32),
                ('xfer_func', C.c_uint32)]


class v4l2_plane_pix_format(C.Structure):
    _pack_ = 1
    _fields_ = [('sizeimage', C.c_uint32), ('bytesperline', C.c_uint32),
                ('reserved', C.c_uint16 * 6)]


class v4l2_pix_format_mplane(C.Structure):
    _pack_ = 1
    _fields_ = [('width', C.c_uint32), ('height', C.c_uint32), ('pixelformat', C.c_uint32),
                ('field', C.c_uint32), ('colorspace', C.c_uint32),
                ('plane_fmt', v4l2_plane_pix_format * VIDEO_MAX_PLANES),
                ('num_planes', C.c_uint8), ('flags', C.c_uint8), ('ycbcr_enc', C.c_uint8),
                ('quantization', C.c_uint8), ('xfer_func', C.c_uint8),
                ('reserved', C.c_uint8 * 7)]


class _fmt_union(C.Union):
    # raw_data[200]; the kernel union also holds struct v4l2_window (has pointers), so it
    # is 8-byte aligned on 64-bit -> sizeof(v4l2_format) == 208.
    _fields_ = [('pix', v4l2_pix_format), ('pix_mp', v4l2_pix_format_mplane),
                ('raw_data', C.c_uint8 * 200), ('_align', C.c_void_p)]


class v4l2_format(C.Structure):
    _fields_ = [('type', C.c_uint32), ('fmt', _fmt_union)]


class v4l2_streamparm_capture(C.Structure):
    _fields_ = [('capability', C.c_uint32), ('capturemode', C.c_uint32),
                ('numerator', C.c_uint32), ('denominator', C.c_uint32),
                ('extendedmode', C.c_uint32), ('readbuffers', C.c_uint32),
                ('reserved', C.c_uint32 * 4)]


class v4l2_streamparm(C.Structure):
    _fields_ = [('type', C.c_uint32), ('capture', v4l2_streamparm_capture),
                ('_pad', C.c_uint8 * (200 - C.sizeof(v4l2_streamparm_capture)))]


class v4l2_requestbuffers(C.Structure):
    _fields_ = [('count', C.c_uint32), ('type', C.c_uint32), ('memory', C.c_uint32),
                ('capabilities', C.c_uint32), ('flags', C.c_uint8), ('reserved', C.c_uint8 * 3)]


class timeval(C.Structure):
    _fields_ = [('tv_sec', C.c_long), ('tv_usec', C.c_long)]


class v4l2_timecode(C.Structure):
    _fields_ = [('type', C.c_uint32), ('flags', C.c_uint32), ('frames', C.c_uint8),
                ('seconds', C.c_uint8), ('minutes', C.c_uint8), ('hours', C.c_uint8),
                ('userbits', C.c_uint8 * 4)]


class _plane_m(C.Union):
    _fields_ = [('mem_offset', C.c_uint32), ('userptr', C.c_ulong), ('fd', C.c_int32)]


class v4l2_plane(C.Structure):
    _fields_ = [('bytesused', C.c_uint32), ('length', C.c_uint32), ('m', _plane_m),
                ('data_offset', C.c_uint32), ('reserved', C.c_uint32 * 11)]


class _buf_m(C.Union):
    _fields_ = [('offset', C.c_uint32), ('userptr', C.c_ulong),
                ('planes', C.POINTER(v4l2_plane)), ('fd', C.c_int32)]


class v4l2_buffer(C.Structure):
    _fields_ = [('index', C.c_uint32), ('type', C.c_uint32), ('bytesused', C.c_uint32),
                ('flags', C.c_uint32), ('field', C.c_uint32), ('timestamp', timeval),
                ('timecode', v4l2_timecode), ('sequence', C.c_uint32), ('memory', C.c_uint32),
                ('m', _buf_m), ('length', C.c_uint32), ('reserved2', C.c_uint32),
                ('request_fd', C.c_int32)]


# --------------------------------------------------------------------------- ioctls
def _ioc(direction, nr, struct):
    return (direction << 30) | (C.sizeof(struct) << 16) | (ord('V') << 8) | nr


_R, _W, _RW = 2, 1, 3
VIDIOC_QUERYCAP = _ioc(_R, 0, v4l2_capability)
VIDIOC_G_FMT = _ioc(_RW, 4, v4l2_format)
VIDIOC_S_FMT = _ioc(_RW, 5, v4l2_format)
VIDIOC_REQBUFS = _ioc(_RW, 8, v4l2_requestbuffers)
VIDIOC_QUERYBUF = _ioc(_RW, 9, v4l2_buffer)
VIDIOC_QBUF = _ioc(_RW, 15, v4l2_buffer)
VIDIOC_DQBUF = _ioc(_RW, 17, v4l2_buffer)
VIDIOC_STREAMON = _ioc(_W, 18, C.c_int)
VIDIOC_STREAMOFF = _ioc(_W, 19, C.c_int)
VIDIOC_S_PARM = _ioc(_RW, 22, v4l2_streamparm)


def _ioctl(fd, req, arg):
    while True:
        try:
            return fcntl.ioctl(fd, req, arg)
        except InterruptedError:
            continue


# --------------------------------------------------------------------------- capture
class V4L2Capture:
    """mmap streaming capture. ``read()`` returns ``(bytes_view, meta)``; see ``convert``."""

    def __init__(self, device, width, height, pixel_format, fps=0.0, n_buffers=4):
        self.device = device
        self.fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        self.maps = []
        self.streaming = False
        try:
            cap = v4l2_capability()
            _ioctl(self.fd, VIDIOC_QUERYCAP, cap)
            caps = cap.device_caps if cap.capabilities & CAP_DEVICE_CAPS else cap.capabilities
            self.card = cap.card.decode(errors='replace')
            self.driver = cap.driver.decode(errors='replace')
            if caps & CAP_VIDEO_CAPTURE_MPLANE:
                self.mplane = True
                self.buf_type = BUF_TYPE_VIDEO_CAPTURE_MPLANE
            elif caps & CAP_VIDEO_CAPTURE:
                self.mplane = False
                self.buf_type = BUF_TYPE_VIDEO_CAPTURE
            else:
                raise OSError(errno.ENODEV, f'{device}: not a video capture device')
            self._set_format(width, height, pixel_format)
            if fps:
                self._set_fps(fps)
            self._alloc(n_buffers)
        except Exception:
            self.close()
            raise

    # ---- setup
    def _set_format(self, width, height, pixel_format):
        f = v4l2_format()
        f.type = self.buf_type
        if self.mplane:
            p = f.fmt.pix_mp
            p.width, p.height, p.pixelformat, p.field = width, height, fourcc(pixel_format), \
                FIELD_NONE
            p.num_planes = 1
        else:
            p = f.fmt.pix
            p.width, p.height, p.pixelformat, p.field = width, height, fourcc(pixel_format), \
                FIELD_NONE
        _ioctl(self.fd, VIDIOC_S_FMT, f)
        _ioctl(self.fd, VIDIOC_G_FMT, f)
        if self.mplane:
            p = f.fmt.pix_mp
            self.num_planes = p.num_planes
            self.bytesperline = p.plane_fmt[0].bytesperline
        else:
            p = f.fmt.pix
            self.num_planes = 1
            self.bytesperline = p.bytesperline
        self.width, self.height = p.width, p.height
        self.pixel_format = fourcc_str(p.pixelformat)
        if self.pixel_format != (pixel_format + '    ')[:4]:
            raise OSError(errno.EINVAL, f'{self.device}: driver chose {self.pixel_format} '
                                        f'instead of {pixel_format}')

    def _set_fps(self, fps):
        parm = v4l2_streamparm()
        parm.type = self.buf_type
        parm.capture.numerator = 1000
        parm.capture.denominator = int(round(fps * 1000))
        try:
            _ioctl(self.fd, VIDIOC_S_PARM, parm)
        except OSError:
            pass  # rkisp has no S_PARM: rate comes from the sensor mode

    def _alloc(self, n):
        req = v4l2_requestbuffers()
        req.count, req.type, req.memory = n, self.buf_type, MEMORY_MMAP
        _ioctl(self.fd, VIDIOC_REQBUFS, req)
        if req.count < 2:
            raise OSError(errno.ENOMEM, f'{self.device}: only {req.count} buffers')
        for i in range(req.count):
            buf, planes = self._new_buf(i)
            _ioctl(self.fd, VIDIOC_QUERYBUF, buf)
            if self.mplane:
                length, offset = planes[0].length, planes[0].m.mem_offset
            else:
                length, offset = buf.length, buf.m.offset
            self.maps.append(mmap.mmap(self.fd, length, mmap.MAP_SHARED,
                                       mmap.PROT_READ | mmap.PROT_WRITE, offset=offset))
        for i in range(len(self.maps)):
            buf, _planes = self._new_buf(i)
            _ioctl(self.fd, VIDIOC_QBUF, buf)
        _ioctl(self.fd, VIDIOC_STREAMON, C.c_int(self.buf_type))
        self.streaming = True

    def _new_buf(self, index):
        buf = v4l2_buffer()
        buf.index, buf.type, buf.memory = index, self.buf_type, MEMORY_MMAP
        planes = None
        if self.mplane:
            planes = (v4l2_plane * VIDEO_MAX_PLANES)()
            buf.m.planes = C.cast(planes, C.POINTER(v4l2_plane))
            buf.length = VIDEO_MAX_PLANES
        return buf, planes

    # ---- streaming
    def dequeue(self, timeout=2.0):
        """Wait for and dequeue one filled buffer; returns ``(index, bytesused, sequence)``.

        The buffer belongs to the caller until :meth:`requeue` (``view`` gives zero-copy
        access to it). Raises ``TimeoutError`` if no frame arrives within ``timeout`` s.
        """
        while True:
            r, _, _ = select.select([self.fd], [], [], timeout)
            if not r:
                raise TimeoutError(f'{self.device}: no frame within {timeout:.1f}s')
            buf, planes = self._new_buf(0)
            try:
                _ioctl(self.fd, VIDIOC_DQBUF, buf)
            except BlockingIOError:
                continue
            used = planes[0].bytesused if self.mplane else buf.bytesused
            return buf.index, used, buf.sequence

    def requeue(self, index):
        buf, _planes = self._new_buf(index)
        _ioctl(self.fd, VIDIOC_QBUF, buf)

    def view(self, index, used):
        """Zero-copy ``memoryview`` of a dequeued buffer (valid until ``requeue``)."""
        return memoryview(self.maps[index])[:used]

    def read(self, timeout=2.0):
        """Dequeue one frame; returns ``(data: bytes, sequence: int)`` (a copy).

        Raises ``TimeoutError`` if no frame arrives within ``timeout`` seconds.
        """
        index, used, seq = self.dequeue(timeout)
        try:
            data = self.maps[index][:used]   # copy out, then give the buffer back
        finally:
            self.requeue(index)
        return data, seq

    def close(self):
        if self.streaming:
            try:
                _ioctl(self.fd, VIDIOC_STREAMOFF, C.c_int(self.buf_type))
            except OSError:
                pass
            self.streaming = False
        for m in self.maps:
            try:
                m.close()
            except Exception:  # noqa: BLE001
                pass
        self.maps = []
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None


def _decimate_422(img, k, pixel_format):
    """Pick every ``k``-th pixel of a packed 4:2:2 image ``(h, w, 2)`` (rows and columns),
    keeping it packed (chroma of the even output pixel's source macropixel). Returns a
    contiguous ``(h // k, w // k, 2)`` array; ``k`` must divide ``h`` and ``2 * k`` ``w``."""
    h, w = img.shape[:2]
    rows = img[::k].reshape(h // k, w // (2 * k), 4 * k)   # groups of k macropixels
    # UYVY: U Y0 V Y1 -> U, Y(px 2mk), V, Y(px (2m+1)k);  YUYV: Y0 U Y1 V
    cols = [0, 1, 2, 2 * k + 1] if pixel_format == 'UYVY' else [0, 1, 2 * k, 3]
    return rows[:, :, cols].reshape(h // k, w // k, 2)


def output_size(width, height, pixel_format, scale):
    """Size :func:`to_bgr` produces for ``scale`` (integer decimation factor, 4:2:2 only)."""
    k = int(scale)
    if k > 1 and pixel_format in ('UYVY', 'YUYV') and height % k == 0 and width % (2 * k) == 0:
        return width // k, height // k
    return width, height


def to_bgr(data, width, height, bytesperline, pixel_format, dst=None, scale=1):
    """Convert one captured buffer (bytes or a zero-copy memoryview) to a BGR ``numpy``
    image (``None`` if undecodable).

    ``dst``: optional preallocated ``(h, w, 3)`` uint8 array the conversion writes into
    (e.g. the data region of a pre-serialized Image). ``scale``: integer decimation for
    packed 4:2:2 input, applied *before* the colour conversion (k**2 less work; see
    :func:`output_size`); other formats ignore it.
    """
    import cv2
    import numpy as np
    buf = np.frombuffer(data, dtype=np.uint8)
    if pixel_format in ('MJPG', 'JPEG'):
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is not None and dst is not None and dst.shape == img.shape:
            np.copyto(dst, img)
            return dst
        return img
    if pixel_format in ('UYVY', 'YUYV'):
        if len(buf) < bytesperline * (height - 1) + 2 * width:
            return None     # short buffer (truncated frame)
        # strided view (no copy even when bytesperline > 2 * width)
        img = np.lib.stride_tricks.as_strided(
            buf, shape=(height, width, 2), strides=(bytesperline, 2, 1), writeable=False)
        if output_size(width, height, pixel_format, scale) != (width, height):
            img = _decimate_422(img, int(scale), pixel_format)
        code = cv2.COLOR_YUV2BGR_UYVY if pixel_format == 'UYVY' else cv2.COLOR_YUV2BGR_YUYV
        if dst is not None and dst.shape == img.shape[:2] + (3,):
            return cv2.cvtColor(img, code, dst=dst)
        return cv2.cvtColor(img, code)
    if pixel_format in ('NV12', 'NV21'):
        img = buf[:bytesperline * height * 3 // 2].reshape(height * 3 // 2, bytesperline)
        img = np.ascontiguousarray(img[:, :width])
        code = cv2.COLOR_YUV2BGR_NV12 if pixel_format == 'NV12' else cv2.COLOR_YUV2BGR_NV21
        return cv2.cvtColor(img, code)
    if pixel_format == 'GREY':
        return buf[:bytesperline * height].reshape(height, bytesperline)[:, :width]
    raise ValueError(f'unsupported pixel format {pixel_format}')


def split_yuyv_luma(data, width, height, bytesperline=0):
    """Side-by-side packed 4:2:2 buffer -> ``(left, right)`` mono8 views of the Y plane.

    Works for YUYV/YVYU (luma on even bytes); ``width`` is the full side-by-side width.
    """
    import numpy as np
    bpl = int(bytesperline) or width * 2
    buf = np.frombuffer(data, dtype=np.uint8, count=bpl * height).reshape(height, bpl)
    luma = buf[:, 0:width * 2:2]
    half = width // 2
    return luma[:, :half], luma[:, half:2 * half]


def split_side_by_side_bgr(data, width, height, bytesperline, pixel_format='YUYV'):
    """Side-by-side packed frame -> ``(left, right)`` bgr8 halves (full colour decode)."""
    frame = to_bgr(data, width, height, bytesperline, pixel_format)
    if frame is None:
        return None
    half = frame.shape[1] // 2
    return frame[:, :half], frame[:, half:2 * half]


def eye_to_bgr(data, width, height, bytesperline, pixel_format, eye):
    """One eye (0 = left, 1 = right) of a side-by-side packed 4:2:2 frame -> bgr8.

    Converts only that half (each eye = ``width // 2`` px = ``width`` bytes per row, a
    YUYV/UYVY macropixel boundary), i.e. half the work of :func:`split_side_by_side_bgr`
    when one eye is wanted. Other formats fall back to the full split.
    """
    import numpy as np
    if pixel_format not in ('YUYV', 'UYVY'):
        eyes = split_side_by_side_bgr(data, width, height, bytesperline, pixel_format)
        return None if eyes is None else np.ascontiguousarray(eyes[eye])
    buf = np.frombuffer(data, dtype=np.uint8)
    half = width // 2
    return to_bgr(buf[eye * 2 * half:], half, height, bytesperline, pixel_format)
