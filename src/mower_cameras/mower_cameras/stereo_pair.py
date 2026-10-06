"""ROS-free helpers for the synchronised Metoak stereo pair (``stereo_cam``).

* :func:`split_luma_into` copies the Y plane of both eyes of ONE side-by-side packed 4:2:2
  buffer into two caller-owned mono8 arrays (typically the data regions of two pre-serialised
  ``sensor_msgs/Image`` buffers, see :mod:`mower_cameras.image_cdr`). No colour conversion.
* :func:`mono_to_ros_ns` maps a kernel ``CLOCK_MONOTONIC`` stamp (V4L2 buffer timestamp, IIO
  sample timestamp) to ROS wall time, so camera and IMU stamps share one clock.
* :class:`DecimationGate` keeps an exact average output rate from a faster source using the
  source's own timestamps (a "period * 0.9 since the last kept frame" cap turns a 25 Hz source
  into 8.3 Hz instead of 10 Hz).
"""
from __future__ import annotations

import numpy as np

# Byte offset of the first luma sample in a 4-byte macropixel.
_LUMA_OFFSET = {'YUYV': 0, 'YVYU': 0, 'UYVY': 1, 'VYUY': 1}


def split_luma_into(data, width, height, bytesperline, pixel_format, dst_left, dst_right):
    """Side-by-side packed 4:2:2 frame -> Y of each eye written into ``dst_left/right``.

    ``width`` is the FULL side-by-side width in pixels (each eye = ``width // 2``);
    ``dst_*`` are writable ``(height, width // 2)`` uint8 arrays. Returns ``False`` (dst
    untouched) for a truncated buffer, raises ``ValueError`` for a non-4:2:2 format.
    """
    try:
        off = _LUMA_OFFSET[pixel_format]
    except KeyError:
        raise ValueError(f'unsupported pixel format {pixel_format!r} (packed 4:2:2 only)')
    bpl = int(bytesperline) or int(width) * 2
    half = int(width) // 2
    buf = np.frombuffer(data, dtype=np.uint8)
    if buf.size < bpl * (height - 1) + 2 * width:
        return False
    rows = np.lib.stride_tricks.as_strided(buf, shape=(height, bpl), strides=(bpl, 1),
                                           writeable=False)
    luma = rows[:, off:off + 2 * width:2]          # (height, width) strided view
    np.copyto(dst_left, luma[:, :half])
    np.copyto(dst_right, luma[:, half:2 * half])
    return True


def mono_to_ros_ns(mono_ns, mono_now_ns, real_now_ns):
    """Kernel monotonic stamp -> ROS (system/wall) time in ns, via the current offset."""
    return int(mono_ns) + (int(real_now_ns) - int(mono_now_ns))


class DecimationGate:
    """Average-rate decimator driven by source timestamps (seconds or ns, any unit).

    ``keep(t)`` is True for the frames that make the output average exactly ``period``
    (e.g. 25 Hz -> 10 Hz keeps 2 of every 5 frames, spaced 80/120 ms). ``tolerance``
    (same unit as ``t``) absorbs source jitter; use about half the source frame interval.
    A gap longer than ``period`` re-anchors the schedule (no burst after a stall).
    ``period`` <= 0 keeps everything.
    """

    def __init__(self, period, tolerance=0.0):
        self.period = float(period)
        self.tolerance = float(tolerance)
        self._due = None

    def keep(self, t):
        if self.period <= 0:
            return True
        if self._due is None or t - self._due > self.period:
            self._due = t + self.period
            return True
        if t >= self._due - self.tolerance:
            self._due += self.period
            return True
        return False


class ClockMap:
    """Smoothed CLOCK_MONOTONIC -> ROS (wall) time offset, shared logic of stereo_cam and
    stereo_imu so both stamp on the same mapping.

    Reading the two clocks back to back jitters by microseconds (preemption between the
    reads), which put non-monotonic steps into a 200 Hz IMU stream when the offset was
    re-read per batch. The offset is re-sampled at most every ``period_s`` (3 tries, the
    tightest bracket wins) and low-pass filtered; a jump > ``step_ns`` (clock set) is
    taken at once.
    """

    def __init__(self, period_s=1.0, alpha=0.05, step_ns=2_000_000, clocks=None):
        import time as _t
        self._mono = clocks[0] if clocks else _t.monotonic_ns
        self._real = clocks[1] if clocks else _t.time_ns
        self.period_ns = int(period_s * 1e9)
        self.alpha, self.step_ns = float(alpha), int(step_ns)
        self.offset = None
        self._next = 0

    def sample(self):
        best = None
        for _ in range(3):
            m0 = self._mono()
            r = self._real()
            m1 = self._mono()
            if best is None or m1 - m0 < best[0]:
                best = (m1 - m0, r - (m0 + m1) // 2)
        return best[1]

    def to_ros(self, mono_ns):
        now = self._mono()
        if self.offset is None or now >= self._next:
            off = self.sample()
            if self.offset is None or abs(off - self.offset) > self.step_ns:
                self.offset = off
            else:
                self.offset += int(round(self.alpha * (off - self.offset)))
            self._next = now + self.period_ns
        return int(mono_ns) + self.offset
