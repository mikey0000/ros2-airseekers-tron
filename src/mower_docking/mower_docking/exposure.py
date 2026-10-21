# SPDX-License-Identifier: Apache-2.0
"""Rear-camera exposure stepping for the dock marker (pure logic, no ROS).

Port of the stock ``mower_charge`` structure (decompiled 2026-10-21, node.cpp OnSetParam /
OnDocking): the camera starts on auto exposure; while SEARCHING, once >= ``window_s`` of
frames give a detection ratio <= ``ratio_threshold`` it steps to the next setting of
auto -> manual 100, 90, 80, 30, 20, 10, 5, 2, 1 -> auto ... (stock ``exposures_`` map); while
the marker is lost during the reverse it steps every ``lost_step_s`` (stock: 1 s). A setting
that gives a good ratio is kept. A day/night preset (mean grey level of the first frame,
stock: HSV V) picks the starting setting: a dark scene starts on manual ``dark_start``
(a backlit marker at dusk is blown out by auto exposure, which opens up for the dark lawn).

Settings are ints: ``AUTO`` (-1) or a manual ``exposure_absolute`` value; the node sends
them as std_msgs/Int32 on /rear_camera/exposure_cmd (mower_cameras applies them).
"""
from __future__ import annotations

from typing import List, Optional, Sequence

AUTO = -1
STOCK_SEQUENCE = (100, 90, 80, 30, 20, 10, 5, 2, 1)


class ExposureStepper:

    def __init__(self, sequence: Sequence[int] = STOCK_SEQUENCE, window_s: float = 3.0,
                 min_frames: int = 10, ratio_threshold: float = 0.2, lost_step_s: float = 1.0,
                 settle_s: float = 0.3, dark_brightness: float = 50.0, dark_start: int = 30,
                 enabled: bool = True):
        self.settings: List[int] = [AUTO] + [int(v) for v in sequence if int(v) >= 0]
        self.window_s = float(window_s)
        self.min_frames = max(1, int(min_frames))
        self.ratio_threshold = float(ratio_threshold)
        self.lost_step_s = float(lost_step_s)
        self.settle_s = float(settle_s)
        self.dark_brightness = float(dark_brightness)
        self.dark_start = int(dark_start)
        self.enabled = bool(enabled)
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        self.index: Optional[int] = None      # None = nothing commanded yet this goal
        self.commanded_any_manual = False
        self.steps = 0
        self.last_ratio: Optional[float] = None
        self.brightness: Optional[float] = None
        self._changed_t: Optional[float] = None
        self._win_t0: Optional[float] = None
        self._win_n = 0
        self._win_det = 0
        self._last_det_t: Optional[float] = None

    @property
    def setting(self) -> Optional[int]:
        return None if self.index is None else self.settings[self.index]

    def _set(self, idx: int, t: float) -> int:
        self.index = idx % len(self.settings)
        self._changed_t = t
        self._win_t0, self._win_n, self._win_det = None, 0, 0
        if self.settings[self.index] != AUTO:
            self.commanded_any_manual = True
        return self.settings[self.index]

    def _preset_index(self, brightness: Optional[float]) -> int:
        if brightness is None or brightness >= self.dark_brightness or len(self.settings) < 2:
            return 0
        # nearest manual setting to dark_start
        best = min(range(1, len(self.settings)),
                   key=lambda i: abs(self.settings[i] - self.dark_start))
        return best

    # ------------------------------------------------------------------ API
    def on_frame(self, t: float, detected: bool, brightness: Optional[float] = None,
                 active: bool = True, lost: bool = False) -> Optional[int]:
        """Feed one processed frame. Returns a setting to command, or None.

        ``active``: the FSM expects the marker in view (SEARCHING / DOCKING); ``lost``: it
        was tracking and has lost it (step every ``lost_step_s``)."""
        if not self.enabled:
            return None
        if detected:
            self._last_det_t = t
        if self.index is None:
            self.brightness = brightness
            return self._set(self._preset_index(brightness), t)
        if not active:
            self._win_t0, self._win_n, self._win_det = None, 0, 0
            return None
        if self._changed_t is not None and t - self._changed_t < self.settle_s:
            return None     # the camera needs a few frames to apply a new exposure
        if lost and not detected:
            since = t - max(self._changed_t if self._changed_t is not None else t,
                            self._last_det_t if self._last_det_t is not None else -1e9)
            if since >= self.lost_step_s:
                self.steps += 1
                self.last_ratio = 0.0
                return self._set(self.index + 1, t)
        if self._win_t0 is None:
            self._win_t0 = t
        self._win_n += 1
        self._win_det += 1 if detected else 0
        if t - self._win_t0 >= self.window_s and self._win_n >= self.min_frames:
            ratio = self._win_det / float(self._win_n)
            self.last_ratio = ratio
            if ratio <= self.ratio_threshold:
                self.steps += 1
                return self._set(self.index + 1, t)
            self._win_t0, self._win_n, self._win_det = None, 0, 0   # good: keep it
        return None

    def stop(self) -> Optional[int]:
        """End of the goal: AUTO if anything else was commanded (restore), else None."""
        restore = self.index is not None and (self.commanded_any_manual or
                                              self.setting != AUTO)
        self.reset()
        return AUTO if restore else None

    def state(self) -> dict:
        return {'setting': self.setting, 'steps': self.steps, 'ratio': self.last_ratio,
                'brightness': self.brightness}
