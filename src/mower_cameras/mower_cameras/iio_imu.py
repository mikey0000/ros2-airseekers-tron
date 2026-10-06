"""ROS-free Linux IIO buffer reader pieces for the Metoak ICM-40608 (``stereo_imu``).

The kernel ``inv_icm42600`` driver (Metoak build, loaded with ``Repot_m=0`` = IIO mode)
exposes two IIO devices, ``icm40608-gyro`` and ``icm40608-accel``. Each has its own buffer
(``/dev/iio:deviceN``) fed from the shared chip FIFO; with x, y, z and timestamp enabled a
scan record is ``3 x be:s16`` + pad + ``le:s64`` = 16 bytes (layout parsed from
``scan_elements``, not hard-coded). Gyro and accel samples of one FIFO packet carry the
same timestamp, so :class:`ImuPairer` joins them by stamp.

* :func:`parse_scan_type` / :func:`scan_layout` / :func:`decode_records`: IIO ABI
  (Documentation/ABI/testing/sysfs-bus-iio, "in_*_type": ``[be|le]:[s|u]bits/storagebits
  [Xrepeat]>>shift``; each element aligned to its own storage size, record aligned to the
  largest element).
* :class:`ImuPairer`: gyro + accel streams -> (t, gyro_xyz, accel_xyz) tuples.
* :class:`RestBias`: gyro bias from the first still window (std below a threshold).
"""
from __future__ import annotations

import os
import re
from collections import deque

import numpy as np

_TYPE_RE = re.compile(r'^(be|le):([su])(\d+)/(\d+)(?:X(\d+))?>>(\d+)$')


def parse_scan_type(text):
    """``'be:s16/16>>0'`` -> dict(endian, signed, bits, storage, repeat, shift)."""
    m = _TYPE_RE.match(text.strip())
    if not m:
        raise ValueError(f'bad IIO scan type {text!r}')
    endian, sign, bits, storage, repeat, shift = m.groups()
    storage = int(storage)
    if storage not in (8, 16, 32, 64):
        raise ValueError(f'unsupported storage bits {storage} in {text!r}')
    return dict(endian='>' if endian == 'be' else '<', signed=sign == 's', bits=int(bits),
                storage=storage, repeat=int(repeat or 1), shift=int(shift))


def scan_layout(channels):
    """``[(name, index, type_text), ...]`` (enabled channels) -> ``(np.dtype, types)``.

    Elements are ordered by scan index; padding fields are named ``_padN``.
    """
    types = {}
    fields, offset, max_align = [], 0, 1
    for name, _index, text in sorted(channels, key=lambda c: int(c[1])):
        t = parse_scan_type(text)
        size = t['storage'] // 8
        if offset % size:
            pad = size - offset % size
            fields.append((f'_pad{len(fields)}', f'V{pad}'))
            offset += pad
        code = ('i' if t['signed'] else 'u') + str(size)
        if t['repeat'] > 1:
            fields.append((name, t['endian'] + code, (t['repeat'],)))
        else:
            fields.append((name, t['endian'] + code))
        offset += size * t['repeat']
        max_align = max(max_align, size)
        types[name] = t
    if offset % max_align:
        fields.append(('_padend', f'V{max_align - offset % max_align}'))
    return np.dtype(fields), types


def decode_records(buf, dtype, types):
    """Bytes of whole scan records -> dict name -> int64 array (shift/mask/sign applied).

    A trailing partial record is ignored (the caller keeps it for the next read).
    """
    n = len(buf) // dtype.itemsize
    rec = np.frombuffer(buf, dtype=dtype, count=n)
    out = {}
    for name, t in types.items():
        v = rec[name].astype(np.int64)
        if t['bits'] < t['storage'] or t['shift']:
            v = (v >> t['shift']) & ((1 << t['bits']) - 1) if t['bits'] < 64 else v >> t['shift']
            if t['signed'] and t['bits'] < 64:
                sign = 1 << (t['bits'] - 1)
                v = (v ^ sign) - sign
        out[name] = v
    return out


class ImuPairer:
    """Join gyro and accel samples that share a timestamp (within ``tol_ns``).

    ``add_gyro(t, xyz)`` / ``add_accel(t, xyz)`` queue samples; :meth:`pop` yields
    ``(t, gyro, accel)`` in time order. A sample whose partner can no longer arrive (the
    other stream is already past it by more than ``tol_ns``) is dropped and counted.
    """

    def __init__(self, tol_ns=1_000_000, max_queue=400):
        self.tol = int(tol_ns)
        self.g = deque(maxlen=max_queue)
        self.a = deque(maxlen=max_queue)
        self.dropped = 0

    def add_gyro(self, t, xyz):
        self.g.append((int(t), xyz))

    def add_accel(self, t, xyz):
        self.a.append((int(t), xyz))

    def pop(self):
        out = []
        g, a = self.g, self.a
        while g and a:
            tg, ta = g[0][0], a[0][0]
            if abs(tg - ta) <= self.tol:
                out.append((tg, g.popleft()[1], a.popleft()[1]))
            elif tg < ta:
                g.popleft()
                self.dropped += 1
            else:
                a.popleft()
                self.dropped += 1
        return out


class RestBias:
    """Gyro bias from the first window of ``n`` samples whose per-axis std < ``max_std``.

    Feed every gyro sample (rad/s) to :meth:`add`; :attr:`bias` stays ``None`` until a
    still window was found (a moving window is discarded and a new one started).
    """

    def __init__(self, n=600, max_std=0.004):
        self.n, self.max_std = int(n), float(max_std)
        self._buf = []
        self.bias = None
        self.std = None
        self.windows_rejected = 0

    def discard(self):
        """Drop the window in progress (the robot moved per wheel odometry)."""
        if self._buf:
            self._buf = []
            self.windows_rejected += 1

    def add(self, xyz):
        if self.bias is not None:
            return True
        self._buf.append(tuple(xyz))
        if len(self._buf) < self.n:
            return False
        a = np.asarray(self._buf, dtype=np.float64)
        self._buf = []
        std = a.std(axis=0)
        if (std < self.max_std).all():
            self.bias, self.std = a.mean(axis=0), std
            return True
        self.windows_rejected += 1
        return False


# ---------------------------------------------------------------------- sysfs helpers
IIO_ROOT = '/sys/bus/iio/devices'


def find_devices(name_prefixes, root=IIO_ROOT):
    """-> {'gyro': 'iio:deviceN', 'accel': 'iio:deviceM'} for the first matching chip."""
    found = {}
    try:
        entries = sorted(os.listdir(root))
    except OSError:
        return found
    for dev in entries:
        try:
            with open(os.path.join(root, dev, 'name')) as f:
                name = f.read().strip()
        except OSError:
            continue
        for prefix in name_prefixes:
            for kind in ('gyro', 'accel'):
                if name == f'{prefix}-{kind}' and kind not in found:
                    found[kind] = dev
    return found


def choose_clock(sample_ns, mono_now_ns, real_now_ns):
    """Which kernel clock an IIO timestamp is on: ``'monotonic'`` or ``'realtime'``."""
    return 'monotonic' if abs(sample_ns - mono_now_ns) < abs(sample_ns - real_now_ns) \
        else 'realtime'


class BatchStamper:
    """Per-sample timestamps from the Metoak driver's per-interrupt group stamps.

    Measured 2026-10-07 (200 Hz, watermark 4): the driver stamps every sample of one FIFO
    interrupt group (4-9 samples, ~every 20 ms) with the IRQ time = the time of the group's
    LAST sample; gyro and accel interrupt separately (stamps ~30 us apart, different group
    sizes), and one ``read()`` may end in the middle of a group. So:

    * the trailing (possibly incomplete) group of each read is held back until a later read
      shows a newer stamp (adds <= one interrupt period of latency, ~20 ms);
    * the group stamps carry a variable IRQ latency (ms), so samples are NOT placed at
      ``T - (k-1-i) * period`` directly: a tracking loop predicts the group end from the
      last output stamp + k * period and moves by ``gain`` x the error (even spacing, the
      long-term clock stays locked to the kernel stamps; the mean IRQ latency remains as
      a constant offset that the cam-IMU time-offset calibration absorbs);
    * a gap/jump of more than ``reanchor`` periods (dropped data) re-anchors on the stamp;
    * ``period`` is the ODR from a long baseline (first/last group end in a sliding window
      of ``window`` groups, ~8 s; the chip's "200 Hz" ran at 200.78 Hz): a per-group
      estimate carries the ms IRQ jitter, which the phase loop would turn into ms drifts;
      kept within +-5 % of the nominal value;
    * output stamps are strictly increasing.

    ``feed(ts, values)`` -> ``(ts_out int64 array, values_out list)`` (possibly empty).
    """

    def __init__(self, nominal_period_ns, gain=0.05, reanchor=4.0, window=400):
        self.nominal = float(nominal_period_ns)
        self.period = float(nominal_period_ns)
        self.gain, self.reanchor = float(gain), float(reanchor)
        self._ends = deque(maxlen=int(window))     # (cumulative sample count, group end)
        self._count = 0
        self._pend_t = []
        self._pend_v = []
        self._last_end = None       # stamp of the previous closed group
        self._last_out = None

    def feed(self, ts, values):
        t_all = self._pend_t + [int(x) for x in ts]
        v_all = self._pend_v + list(values)
        if not t_all:
            return np.zeros(0, np.int64), []
        k = len(t_all)                          # hold back the trailing group
        while k > 0 and t_all[k - 1] == t_all[-1]:
            k -= 1
        self._pend_t, self._pend_v = t_all[k:], v_all[k:]
        out_t, out_v = [], []
        i = 0
        while i < k:
            j = i
            while j + 1 < k and t_all[j + 1] == t_all[i]:
                j += 1
            n, end = j - i + 1, t_all[j]
            self._count += n
            if self._ends and abs((end - self._ends[-1][1]) / n - self.nominal) \
                    > self.reanchor * self.nominal:
                self._ends.clear()                 # data gap: restart the baseline
            self._ends.append((self._count, end))
            if len(self._ends) >= 2:
                (c0, e0), (c1, e1) = self._ends[0], self._ends[-1]
                p = (e1 - e0) / (c1 - c0)
                if abs(p - self.nominal) < 0.05 * self.nominal:
                    self.period = p
            self._last_end = end
            if self._last_out is None or \
                    abs(end - (self._last_out + n * self.period)) > self.reanchor * self.period:
                start, step = end - (n - 1) * self.period, self.period       # (re)anchor
            else:
                err = end - (self._last_out + n * self.period)
                step = self.period + self.gain * err / n
                start = self._last_out + step
            for m in range(n):
                t = int(round(start + m * step))
                if self._last_out is not None and t <= self._last_out:
                    t = self._last_out + 1
                self._last_out = t
                out_t.append(t)
                out_v.append(v_all[i + m])
            i = j + 1
        return np.asarray(out_t, np.int64), out_v


class ImuCdr:
    """Pre-serialised ``sensor_msgs/Imu`` (XCDR1 LE) with fixed frame_id and covariances.

    ``serialize(stamp_ns, gyro, accel)`` patches stamp + 6 doubles; orientation is zero with
    ``orientation_covariance[0] = -1`` (no orientation). Same length and content as
    ``rclpy.serialization.serialize_message`` (zeroed padding; test/test_iio_imu.py).
    """

    def __init__(self, frame_id, gyro_cov, accel_cov):
        import struct
        out = bytearray(b'\x00\x01\x00\x00')
        out.extend(b'\0' * 8)                                   # stamp
        raw = frame_id.encode() + b'\0'
        out.extend(struct.pack('<I', len(raw)))
        out.extend(raw)
        out.extend(b'\0' * (-(len(out) - 4) % 8))              # align doubles (payload-relative)
        self._q = len(out)
        out.extend(struct.pack('<4d', 0.0, 0.0, 0.0, 0.0))      # orientation x y z w
        out.extend(struct.pack('<9d', -1.0, *([0.0] * 8)))
        self._g = len(out)
        out.extend(b'\0' * 24)
        out.extend(struct.pack('<9d', *gyro_cov))
        self._a = len(out)
        out.extend(b'\0' * 24)
        out.extend(struct.pack('<9d', *accel_cov))
        self.buf = out
        self._stamp = struct.Struct('<iI')
        self._v3 = struct.Struct('<3d')

    def serialize(self, stamp_ns, gyro, accel):
        sec, nsec = divmod(int(stamp_ns), 1_000_000_000)
        self._stamp.pack_into(self.buf, 4, sec, nsec)
        self._v3.pack_into(self.buf, self._g, *gyro)
        self._v3.pack_into(self.buf, self._a, *accel)
        return bytes(self.buf)
