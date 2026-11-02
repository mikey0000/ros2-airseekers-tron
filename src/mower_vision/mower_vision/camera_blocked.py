# SPDX-License-Identifier: GPL-3.0-or-later
"""Front-camera blocked monitor, pure logic (stock parity 2026-11-03).

Stock raises IotNotice ``CameraBlocked`` (100105, "one or more cameras blocked"). We flag the
front camera blocked when, while MOWING in daylight, its frames stay dark (mean below
``mean_min``), washed out (mean above ``mean_max``) or uniform (std below ``std_max``: grass
clippings / mud on the lens, a leaf, a cover) for ``blocked_s``; it clears once the
statistics are back in range for ``clear_s``. The mission slows down while it is set.
"""

import datetime
from typing import Optional, Sequence, Tuple

import numpy as np

# encoding -> (channels, dtype) for the luminance proxy
_ENCODINGS = {'mono8': (1, np.uint8), '8uc1': (1, np.uint8), 'bgr8': (3, np.uint8),
              'rgb8': (3, np.uint8), 'bgra8': (4, np.uint8), 'rgba8': (4, np.uint8),
              'yuv422': (2, np.uint8), 'uyvy': (2, np.uint8), 'yuyv': (2, np.uint8),
              'yuv422_yuy2': (2, np.uint8), 'mono16': (1, np.uint16)}


def image_stats(data, width: int, height: int, step: int, encoding: str,
                decimate: int = 8) -> Optional[Tuple[float, float]]:
    """(mean, std) of a decimated luminance proxy (8-bit scale), None for an unknown
    encoding or a short buffer. YUV422 uses its luma bytes; colour the channel mean."""
    enc = (encoding or '').lower()
    if enc not in _ENCODINGS or width <= 0 or height <= 0:
        return None
    ch, dt = _ENCODINGS[enc]
    bpp = ch * np.dtype(dt).itemsize
    if step < width * bpp or len(data) < step * height:
        return None
    rows = np.frombuffer(data, dtype=np.uint8, count=step * height).reshape(height, step)
    rows = rows[::max(1, decimate), :width * bpp]
    px = rows.view(dt).reshape(rows.shape[0], width, ch)[:, ::max(1, decimate)]
    if enc in ('yuv422', 'uyvy'):
        y = px[:, :, 1]
    elif enc in ('yuyv', 'yuv422_yuy2'):
        y = px[:, :, 0]
    elif ch >= 3:
        y = px[:, :, :3].mean(axis=2)
    else:
        y = px[:, :, 0]
    y = y.astype(np.float64)
    if dt == np.uint16:
        y = y / 256.0
    return float(y.mean()), float(y.std())


def is_daylight(hours: Sequence[float], now: Optional[datetime.datetime] = None) -> bool:
    """``hours`` = [start, end] local time in hours (e.g. [8.0, 18.0]); empty = always."""
    if not hours or len(hours) < 2:
        return True
    now = now or datetime.datetime.now()
    h = now.hour + now.minute / 60.0
    lo, hi = float(hours[0]), float(hours[1])
    return lo <= h < hi if lo <= hi else (h >= lo or h < hi)


class BlockedMonitor:
    def __init__(self, blocked_s=10.0, clear_s=3.0, std_max=6.0, mean_min=12.0,
                 mean_max=250.0):
        self.blocked_s, self.clear_s = float(blocked_s), float(clear_s)
        self.std_max, self.mean_min, self.mean_max = float(std_max), float(mean_min), \
            float(mean_max)
        self.blocked = False
        self.why = ''
        self._bad_since = None
        self._good_since = None

    def classify(self, mean, std) -> str:
        """'' = a usable image, else why it looks blocked."""
        if mean < self.mean_min:
            return 'dark (mean %.0f)' % mean
        if mean > self.mean_max:
            return 'washed out (mean %.0f)' % mean
        if std < self.std_max:
            return 'uniform (std %.1f)' % std
        return ''

    def update(self, now: float, stats: Optional[Tuple[float, float]], active: bool) -> bool:
        """Feed one sample; returns True when ``blocked`` changed. Inactive (not mowing, not
        daylight) clears at once; no stats (no frame) holds the current verdict."""
        if not active:
            self._bad_since = self._good_since = None
            return self._set(False, '')
        if stats is None:
            return False
        why = self.classify(*stats)
        if why:
            self._good_since = None
            if self._bad_since is None:
                self._bad_since = now
            if not self.blocked and now - self._bad_since >= self.blocked_s:
                return self._set(True, why)
            return False
        self._bad_since = None
        if self._good_since is None:
            self._good_since = now
        if self.blocked and now - self._good_since >= self.clear_s:
            return self._set(False, '')
        return False

    def _set(self, blocked, why):
        changed = blocked != self.blocked
        self.blocked, self.why = blocked, why
        return changed
