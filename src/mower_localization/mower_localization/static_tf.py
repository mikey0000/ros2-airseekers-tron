"""Rotation between two frames from ``/tf_static`` alone, without ``tf2_ros``.

``heading_aligner`` needs one static rotation (base_link <- stereo IMU frame) once at
startup. Importing ``tf2_ros`` (Buffer + TransformListener) for that costs ~30 MB per
process and keeps a ``/tf`` subscription alive forever; this module resolves the chain of
static transforms in pure Python instead. Rotation only (translations are not needed).
"""

from __future__ import annotations

import math


def qmul(a, b):
    """Hamilton product ``a * b`` of (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


class StaticRotations:
    """Tree of static rotations fed from ``tf2_msgs/TFMessage`` (``/tf_static``)."""

    def __init__(self):
        self._parent = {}     # child -> (parent, q_parent_child)

    def add(self, parent, child, q):
        self._parent[str(child).lstrip('/')] = (str(parent).lstrip('/'), tuple(q))

    def add_msg(self, msg):
        for t in msg.transforms:
            r = t.transform.rotation
            self.add(t.header.frame_id, t.child_frame_id, (r.x, r.y, r.z, r.w))

    def _to_root(self, frame):
        frame = str(frame).lstrip('/')
        q, seen = (0.0, 0.0, 0.0, 1.0), set()
        while frame in self._parent:
            if frame in seen:   # cycle: malformed tree
                return None, None
            seen.add(frame)
            parent, q_pc = self._parent[frame]
            q = qmul(q_pc, q)
            frame = parent
        return frame, q

    def rotation(self, target, source):
        """``q_target_source`` (as ``tf2 lookup_transform(target, source)``), or None."""
        rt, qt = self._to_root(target)
        rs, qs = self._to_root(source)
        if rt is None or rs is None or rt != rs:
            return None
        q = qmul(qconj(qt), qs)
        n = math.sqrt(sum(c * c for c in q)) or 1.0
        return tuple(c / n for c in q)
