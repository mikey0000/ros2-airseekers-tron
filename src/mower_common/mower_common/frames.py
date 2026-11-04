# SPDX-License-Identifier: GPL-3.0-or-later
"""Planar static-transform lookups from /tf_static (no ROS imports).

A static tree is a dict {child: (parent, x, y, yaw)} of the /tf_static transforms seen so far.
"""

import math

from mower_common.angles import yaw_from_quaternion


def static_entry(t):
    """(child, (parent, x, y, yaw)) of one geometry_msgs/TransformStamped, planar."""
    tr = t.transform
    q = tr.rotation
    return t.child_frame_id, (t.header.frame_id, tr.translation.x, tr.translation.y,
                              yaw_from_quaternion(q.x, q.y, q.z, q.w))


def blade_offset_from(static, base='base_link', blade='blade_link'):
    """(x, y) of ``blade`` in ``base`` from static transforms {child: (parent, x, y, yaw)}
    (the /tf_static tree, planar), chained through intermediate frames; None when the chain
    does not reach ``base``."""
    x, y, frame, seen = 0.0, 0.0, blade, set()
    while frame != base:
        if frame in seen or frame not in static:
            return None
        seen.add(frame)
        parent, tx, ty, tyaw = static[frame]
        c, s = math.cos(tyaw), math.sin(tyaw)
        x, y = tx + c * x - s * y, ty + s * x + c * y
        frame = parent
    return x, y
