"""Rotation between two frames from ``/tf_static`` alone, without ``tf2_ros``.

``heading_aligner`` needs one static rotation (base_link <- stereo IMU frame) once at
startup. Importing ``tf2_ros`` (Buffer + TransformListener) for that costs ~30 MB per
process and keeps a ``/tf`` subscription alive forever; this module resolves the chain of
static transforms in pure Python instead. 2026-11-12: also the translation between two frames
(``gps_gate`` reads the antenna height, base_link -> gps_link, for its tilt compensation).
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


def qrotate(q, v):
    """Vector ``v`` rotated by the unit quaternion ``q`` (x, y, z, w)."""
    x, y, z, _ = qmul(qmul(q, (v[0], v[1], v[2], 0.0)), qconj(q))
    return (x, y, z)


class StaticRotations:
    """Tree of static rotations fed from ``tf2_msgs/TFMessage`` (``/tf_static``)."""

    def __init__(self):
        self._parent = {}     # child -> (parent, q_parent_child, t_parent_child)

    def add(self, parent, child, q, t=(0.0, 0.0, 0.0)):
        self._parent[str(child).lstrip('/')] = (str(parent).lstrip('/'), tuple(q), tuple(t))

    def add_msg(self, msg):
        for t in msg.transforms:
            r, p = t.transform.rotation, t.transform.translation
            self.add(t.header.frame_id, t.child_frame_id, (r.x, r.y, r.z, r.w),
                     (p.x, p.y, p.z))

    def _to_root(self, frame):
        root, q, _ = self._pose_in_root(frame)
        return root, q

    def _pose_in_root(self, frame):
        """(root, q_root_frame, origin of ``frame`` in root), or (None, None, None)."""
        frame = str(frame).lstrip('/')
        q, p, seen = (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), set()
        while frame in self._parent:
            if frame in seen:   # cycle: malformed tree
                return None, None, None
            seen.add(frame)
            parent, q_pc, t_pc = self._parent[frame]
            r = qrotate(q_pc, p)
            p = (t_pc[0] + r[0], t_pc[1] + r[1], t_pc[2] + r[2])
            q = qmul(q_pc, q)
            frame = parent
        return frame, q, p

    def translation(self, target, source):
        """Origin of ``source`` in ``target`` (the translation of tf2
        ``lookup_transform(target, source)``), or None when the frames are not connected."""
        rt, qt, pt = self._pose_in_root(target)
        rs, _, ps = self._pose_in_root(source)
        if rt is None or rs is None or rt != rs:
            return None
        return qrotate(qconj(qt), (ps[0] - pt[0], ps[1] - pt[1], ps[2] - pt[2]))

    def rotation(self, target, source):
        """``q_target_source`` (as ``tf2 lookup_transform(target, source)``), or None."""
        rt, qt = self._to_root(target)
        rs, qs = self._to_root(source)
        if rt is None or rs is None or rt != rs:
            return None
        q = qmul(qconj(qt), qs)
        n = math.sqrt(sum(c * c for c in q)) or 1.0
        return tuple(c / n for c in q)
