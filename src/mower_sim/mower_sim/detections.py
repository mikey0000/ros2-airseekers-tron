# SPDX-License-Identifier: Apache-2.0
"""Scripted ranged detections: the sim's stand-in for det_ros + det_range (ROS-free).

The sim has no YOLO. A world obstacle with ``detect: {label, score}`` is reported while it
is in the front camera's field of view, in det_range's output encoding (2026-10-23, no-go
zones; docs/analysis/2026-10-23_obstacle_nogo_options.md):

* position = the visible FRONT face (centre minus the radius along the viewing ray), in
  base_link, as det_range's stereo range of the central half of the box;
* width = the visible width (cylinder: its diameter; box: the extent across the ray);
* the box width in pixels = width x fx / range, for the Detection2D bbox.

Visibility: the CENTRE bearing within +-``hfov_deg``/2 of the camera axis and the range
within ``min_range`` .. ``max_range`` (no occlusion model). Like the field trunk, an object
beside the camera (closer than ~0.9 m at the side) is not reported.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from .sensors import STEREO_XYZ


@dataclass
class DetParams:
    hfov_deg: float = 80.0
    min_range: float = 0.3
    max_range: float = 4.0
    fx: float = 372.0          # vio_camera (right colour eye) focal length, px
    range_noise_m: float = 0.0
    score_noise: float = 0.0
    seed: int = 0


@dataclass
class SimDetection:
    label: str
    score: float
    face_base: tuple            # (x, y, z) base_link, front face (det_range's point)
    width: float                # m
    range: float                # m along the camera axis
    bbox_w_px: float
    bearing: float              # rad, camera frame (left +)


class DetectionSource:
    def __init__(self, params: Optional[DetParams] = None, cam_xyz=STEREO_XYZ):
        self.p = params or DetParams()
        self.cam = np.asarray(cam_xyz, dtype=float)
        self.rng = np.random.default_rng(self.p.seed)

    def detect(self, x: float, y: float, yaw: float, obstacles: Sequence, t: float
               ) -> List[SimDetection]:
        p = self.p
        out = []
        c, s = math.cos(yaw), math.sin(yaw)
        for o in obstacles:
            if not getattr(o, 'detect_label', ''):
                continue
            ox, oy = o.position(t)
            # obstacle centre in base_link, then relative to the camera
            bx = c * (ox - x) + s * (oy - y)
            by = -s * (ox - x) + c * (oy - y)
            dx, dy = bx - self.cam[0], by - self.cam[1]
            dist = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx)
            if abs(bearing) > math.radians(p.hfov_deg / 2.0):
                continue
            if o.type == 'cylinder':
                half = o.radius
            else:
                # extent of the box across the viewing ray
                rel = (o.yaw - yaw) - bearing
                half = 0.5 * (abs(o.size_x * math.sin(rel)) + abs(o.size_y * math.cos(rel)))
            face = dist - half
            if face < p.min_range or face > p.max_range:
                continue
            if p.range_noise_m > 0:
                face += float(self.rng.normal(0.0, p.range_noise_m))
            ux, uy = dx / dist, dy / dist
            fx_b = self.cam[0] + ux * face
            fy_b = self.cam[1] + uy * face
            z_axis = max(face * math.cos(bearing), 0.05)
            score = o.detect_score
            if p.score_noise > 0:
                score = float(np.clip(score + self.rng.normal(0.0, p.score_noise), 0.0, 1.0))
            out.append(SimDetection(o.detect_label, score,
                                    (fx_b, fy_b, min(o.height, 0.5) / 2.0), 2.0 * half,
                                    z_axis, 2.0 * half * p.fx / z_axis, bearing))
        return out


def disc_base(d: SimDetection, cam_xy, margin: float = 0.10, spacing: float = 0.05):
    """det_range's static-class cloud for one detection (det_range.geometry.object_disc):
    the edges of the front face across the ray -> a disc of width / 2 + margin."""
    from det_range.geometry import object_disc
    fx, fy, fz = d.face_base
    vx, vy = fx - cam_xy[0], fy - cam_xy[1]
    n = math.hypot(vx, vy) or 1.0
    px, py = -vy / n * d.width / 2.0, vx / n * d.width / 2.0
    return object_disc((fx + px, fy + py, fz), (fx - px, fy - py, fz), cam_xy,
                       margin=margin, spacing=spacing)
