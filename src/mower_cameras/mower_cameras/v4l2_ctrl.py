"""V4L2 user controls (exposure) without ``v4l2-ctl`` (2026-10-21, docking at dusk).

The stock ``mower_charge`` shells out to ``v4l2-ctl --device=/dev/rear_camera
--set-ctrl=exposure_auto=1 --set-ctrl=exposure_absolute=N`` (manual) and
``exposure_auto=3`` (auto, aperture priority). This module does the same with three ioctls
(QUERYCTRL / G_CTRL / S_CTRL) on a separate fd, which V4L2 allows while another fd streams.

Everything returns ``(ok, message)`` instead of raising: a camera without these controls
(EINVAL), a missing device (ENOENT) or a busy one degrades to "no exposure control".
"""
from __future__ import annotations

import ctypes as C
import fcntl
import os
from typing import Callable, Optional, Tuple

CID_EXPOSURE_AUTO = 0x009a0901        # V4L2_CID_EXPOSURE_AUTO (menu; newer name auto_exposure)
CID_EXPOSURE_ABSOLUTE = 0x009a0902    # V4L2_CID_EXPOSURE_ABSOLUTE (100 us units)
EXPOSURE_MANUAL = 1                   # V4L2_EXPOSURE_MANUAL
EXPOSURE_APERTURE_PRIORITY = 3        # V4L2_EXPOSURE_APERTURE_PRIORITY (UVC "auto")
CTRL_FLAG_DISABLED = 0x0001


class v4l2_queryctrl(C.Structure):
    _fields_ = [('id', C.c_uint32), ('type', C.c_uint32), ('name', C.c_char * 32),
                ('minimum', C.c_int32), ('maximum', C.c_int32), ('step', C.c_int32),
                ('default_value', C.c_int32), ('flags', C.c_uint32),
                ('reserved', C.c_uint32 * 2)]


class v4l2_control(C.Structure):
    _fields_ = [('id', C.c_uint32), ('value', C.c_int32)]


def _iowr(nr, struct):
    return (3 << 30) | (C.sizeof(struct) << 16) | (ord('V') << 8) | nr


VIDIOC_G_CTRL = _iowr(27, v4l2_control)
VIDIOC_S_CTRL = _iowr(28, v4l2_control)
VIDIOC_QUERYCTRL = _iowr(36, v4l2_queryctrl)


class V4L2Controls:
    """Get/set integer controls on ``device``. ``opener``/``ioctl`` are injectable (tests)."""

    def __init__(self, device: str, opener: Optional[Callable[[str], int]] = None,
                 ioctl: Optional[Callable] = None, closer: Optional[Callable[[int], None]] = None):
        self.device = device
        self._open = opener or (lambda d: os.open(d, os.O_RDWR | os.O_NONBLOCK))
        self._ioctl = ioctl or fcntl.ioctl
        self._close = closer or os.close

    def _with_fd(self, fn):
        try:
            fd = self._open(self.device)
        except OSError as exc:
            return False, 'open %s: %s' % (self.device, exc.strerror or exc)
        try:
            return fn(fd)
        except OSError as exc:
            return False, '%s: %s' % (self.device, exc.strerror or exc)
        finally:
            try:
                self._close(fd)
            except OSError:
                pass

    def query(self, cid: int) -> Tuple[bool, object]:
        def q(fd):
            qc = v4l2_queryctrl(id=cid)
            self._ioctl(fd, VIDIOC_QUERYCTRL, qc)
            if qc.flags & CTRL_FLAG_DISABLED:
                return False, 'control 0x%x disabled' % cid
            return True, (qc.minimum, qc.maximum, qc.step)
        return self._with_fd(q)

    def get(self, cid: int) -> Tuple[bool, object]:
        def g(fd):
            c = v4l2_control(id=cid)
            self._ioctl(fd, VIDIOC_G_CTRL, c)
            return True, int(c.value)
        return self._with_fd(g)

    def set(self, cid: int, value: int) -> Tuple[bool, str]:
        def s(fd):
            c = v4l2_control(id=cid, value=int(value))
            self._ioctl(fd, VIDIOC_S_CTRL, c)
            return True, 'ok'
        return self._with_fd(s)

    # ---- exposure (the stock mower_charge sequence) --------------------------------
    def set_auto_exposure(self) -> Tuple[bool, str]:
        ok, msg = self.set(CID_EXPOSURE_AUTO, EXPOSURE_APERTURE_PRIORITY)
        return ok, ('auto exposure' if ok else 'auto exposure failed: %s' % msg)

    def set_manual_exposure(self, value: int) -> Tuple[bool, str]:
        ok, rng = self.query(CID_EXPOSURE_ABSOLUTE)
        if not ok:
            return False, 'no exposure_absolute: %s' % rng
        lo, hi, _ = rng
        v = max(lo, min(hi, int(value)))
        ok, msg = self.set(CID_EXPOSURE_AUTO, EXPOSURE_MANUAL)
        if not ok:
            return False, 'manual mode failed: %s' % msg
        ok, msg = self.set(CID_EXPOSURE_ABSOLUTE, v)
        if not ok:
            return False, 'exposure_absolute=%d failed: %s' % (v, msg)
        return True, 'manual exposure %d%s' % (v, '' if v == int(value) else
                                               ' (clamped from %d)' % int(value))

    def apply(self, value: int) -> Tuple[bool, str]:
        """``value`` < 0: auto exposure; otherwise manual exposure_absolute ``value``."""
        return self.set_auto_exposure() if value < 0 else self.set_manual_exposure(value)
