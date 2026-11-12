"""GNSS antenna tilt compensation, pure math (2026-11-12 row straightness review).

The RTK antenna sits about 0.324 m above the ground (URDF gps_link z 0.214 above the drive
axle + the 0.11 m wheel radius). The EKF is 2D and navsat_transform_node removes only the
horizontal lever arm, so body roll / pitch move the reported position: roll swings the
antenna sideways by h sin(roll), and a steady 9 deg side slope shifts it ~5 cm downhill.
The 2026-11-12 field review found the unexplained lateral input of the row loop correlated
with -0.324 sin(roll) (r -0.73, coefficient -1.34, zero lag): the controller steered against
the antenna sway (docs/analysis/2026-11-12_row_straightness.md).

Frames and signs (REP-103): body x forward, y left, z up; roll > 0 = left side up (right
side down), pitch > 0 = nose down; yaw ENU (0 = east, CCW). The antenna, ``h`` above the
ground point under it, is displaced from that ground point by R(yaw) Ry(pitch) Rx(roll)
(0, 0, h) - (0, 0, h). Horizontally, in the body-yaw frame:

    forward = h cos(roll) sin(pitch)      (nose down: antenna ahead)
    left    = -h sin(roll)                (right side down: antenna to the right)

:func:`tilt_offset_enu` rotates that by yaw into ENU / map; the ground point is the fix
minus it. :func:`shift_latlon` applies a metric ENU shift to a WGS84 fix.
"""

from __future__ import annotations

import bisect
import math
from collections import deque
from typing import Optional, Tuple

# WGS84 (as mower_sim.world): radii of curvature for the metric -> degree conversion.
_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3

# Wheel radius (URDF config/urdf/mower.urdf.xacro wheel_radius): base_link (the drive axle)
# height above the ground. Antenna height above ground = base_link -> gps_link z + this.
AXLE_HEIGHT_M = 0.11
# Measured antenna height above the ground (= URDF gps z 0.214 + AXLE_HEIGHT_M), the fallback
# until /tf_static has delivered base_link -> gps_link.
ANTENNA_HEIGHT_M = 0.324


def roll_pitch_yaw(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """(roll, pitch, yaw) rad of a quaternion, ZYX (tf2 getEulerYPR order)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, math.asin(sp), yaw


def tilt_offset_enu(roll: float, pitch: float, yaw: float,
                    height: float) -> Tuple[float, float]:
    """Horizontal (east, north) displacement in m of a point ``height`` above the ground
    contact, caused by body roll / pitch (rad), for a body heading ``yaw`` (rad ENU)."""
    fwd = height * math.cos(roll) * math.sin(pitch)
    left = -height * math.sin(roll)
    c, s = math.cos(yaw), math.sin(yaw)
    return c * fwd - s * left, s * fwd + c * left


def shift_latlon(lat: float, lon: float, east: float, north: float) -> Tuple[float, float]:
    """``(lat, lon)`` deg moved by ``east`` / ``north`` m (local WGS84 radii; exact to well
    under a millimetre for the centimetre shifts used here)."""
    s = math.sin(math.radians(lat))
    w = math.sqrt(1.0 - _WGS84_E2 * s * s)
    meridional = _WGS84_A * (1.0 - _WGS84_E2) / (w ** 3)
    normal = _WGS84_A / w
    return (lat + math.degrees(north / meridional),
            lon + math.degrees(east / (normal * math.cos(math.radians(lat)))))


class AttitudeHistory:
    """Recent (stamp, roll, pitch, yaw) IMU samples; :meth:`at` returns the one nearest a fix
    stamp, or None when none lies within ``max_age_s`` of it."""

    def __init__(self, maxlen: int = 200):
        self._t: deque = deque(maxlen=maxlen)
        self._rpy: deque = deque(maxlen=maxlen)

    def add(self, stamp: float, roll: float, pitch: float, yaw: float) -> None:
        if self._t and stamp < self._t[-1]:     # clock jump back (sim restart): start over
            self._t.clear()
            self._rpy.clear()
        self._t.append(stamp)
        self._rpy.append((roll, pitch, yaw))

    def at(self, stamp: float, max_age_s: float) -> Optional[Tuple[float, float, float]]:
        if not self._t:
            return None
        t = list(self._t)
        i = bisect.bisect_left(t, stamp)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(t) and (best is None or abs(t[j] - stamp) < abs(t[best] - stamp)):
                best = j
        if best is None or abs(t[best] - stamp) > max_age_s:
            return None
        return self._rpy[best]


def antenna_height(gps_z_above_base: Optional[float], override: float,
                   axle_height: float = AXLE_HEIGHT_M) -> float:
    """Antenna height above the ground: ``override`` when > 0, else the base_link ->
    antenna z from TF plus the axle height, else :data:`ANTENNA_HEIGHT_M`."""
    if override > 0.0:
        return float(override)
    if gps_z_above_base is not None:
        return float(gps_z_above_base) + float(axle_height)
    return ANTENNA_HEIGHT_M
