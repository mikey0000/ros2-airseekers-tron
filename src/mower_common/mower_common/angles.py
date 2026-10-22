# SPDX-License-Identifier: GPL-3.0-or-later
"""Angle helpers shared by the nodes (one copy; 2026-10-23)."""

import math

TWO_PI = 2.0 * math.pi


def wrap_angle(a):
    """``a`` wrapped to (-pi, pi] (exact fmod form)."""
    a = math.fmod(a + math.pi, TWO_PI)
    if a <= 0.0:
        a += TWO_PI
    return a - math.pi


def wrap_angle_trig(a):
    """``a`` wrapped to (-pi, pi] as ``atan2(sin a, cos a)`` (the form geometry,
    slip_detector and dock_yaw always used; kept so their results stay bit-identical)."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x, y, z, w):
    """Yaw (rad) of the quaternion (x, y, z, w)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
