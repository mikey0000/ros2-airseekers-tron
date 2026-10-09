# SPDX-License-Identifier: Apache-2.0
"""ROS-free synthetic sensors: front stereo depth raycast and bumper side.

Stereo (mower_cameras/stereo_depth contract): obstacle-only points in
``stereo_camera_optical`` (z forward, x right, y down), range 0.2-4.0 m, plus a
ground "clear" cloud (ground hits up to 3 m, one per 10 cm cell) that the
costmap uses to raytrace free space. Obstacles are the world's 2.5D shapes;
rays are cast on a regular azimuth x elevation grid inside the camera FOV.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .world import Obstacle, to_local

# URDF stereo_camera_optical_joint: xyz 0.466 0 0.240, rpy -1.5547956 0 -1.5707963
STEREO_XYZ = (0.466, 0.0, 0.240)
STEREO_RPY = (-1.5547956, 0.0, -1.5707963)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF/tf fixed-axis rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


@dataclass
class StereoParams:
    hfov_deg: float = 80.0
    vfov_deg: float = 50.0
    columns: int = 64
    rows: int = 32
    min_range: float = 0.2
    max_range: float = 4.0
    clear_max_range: float = 3.0
    clear_cell_m: float = 0.10
    noise_m: float = 0.0           # gaussian range noise (sigma)


class StereoRaycaster:
    def __init__(self, params: Optional[StereoParams] = None,
                 xyz=STEREO_XYZ, rpy=STEREO_RPY, seed: int = 0):
        self.p = params or StereoParams()
        self.t_cam = np.array(xyz, dtype=float)
        self.r_cam = rpy_matrix(*rpy)          # optical -> base_link
        p = self.p
        az = np.radians(np.linspace(-p.hfov_deg / 2, p.hfov_deg / 2, p.columns))
        el = np.radians(np.linspace(-p.vfov_deg / 2, p.vfov_deg / 2, p.rows))
        A, E = np.meshgrid(az, el)
        # optical frame: z forward, x right, y down
        dirs = np.stack([np.tan(A), np.tan(E), np.ones_like(A)], axis=-1).reshape(-1, 3)
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        self.dirs_opt = dirs
        self.dirs_base = dirs @ self.r_cam.T
        self.rng = np.random.default_rng(seed)

    def to_optical(self, pts_base: np.ndarray) -> np.ndarray:
        return (pts_base - self.t_cam) @ self.r_cam

    def cast(self, x: float, y: float, yaw: float, obstacles: Sequence[Obstacle],
             t: float) -> Tuple[np.ndarray, np.ndarray]:
        """(obstacle points, ground clear points), both Nx3 float32 in the optical frame."""
        p = self.p
        c, s = math.cos(yaw), math.sin(yaw)
        d = self.dirs_base
        # ray directions in map frame
        dx = c * d[:, 0] - s * d[:, 1]
        dy = s * d[:, 0] + c * d[:, 1]
        dz = d[:, 2]
        ox = x + c * self.t_cam[0] - s * self.t_cam[1]
        oy = y + s * self.t_cam[0] + c * self.t_cam[1]
        oz = self.t_cam[2]
        n = len(dz)
        tmax = np.full(n, np.inf)
        # ground plane z = 0
        down = dz < -1e-6
        tg = np.full(n, np.inf)
        tg[down] = oz / -dz[down]
        hit_obstacle = np.zeros(n, dtype=bool)
        tmax = tg.copy()
        for o in obstacles:
            t_in, t_out = _ray_shape(o, t, ox, oy, dx, dy)
            valid = np.isfinite(t_in) & (t_out > 0)
            t_in = np.where(t_in < 0, 0.0, t_in)
            z_in = oz + t_in * dz
            z_out = oz + t_out * dz
            # side hit
            side = valid & (z_in >= 0.0) & (z_in <= o.height)
            th = np.where(side, t_in, np.inf)
            # top hit: ray above the top at entry, descending below it before exit
            top = valid & (z_in > o.height) & (z_out <= o.height) & (dz < 0)
            ttop = np.where(top, (o.height - oz) / np.where(dz < 0, dz, -1.0), np.inf)
            th = np.minimum(th, ttop)
            closer = th < tmax
            tmax = np.where(closer, th, tmax)
            hit_obstacle = np.where(closer, True, hit_obstacle)
        r = tmax
        if p.noise_m > 0:
            r = r + self.rng.normal(0.0, p.noise_m, size=n)
        pts = self.dirs_opt * r[:, None]
        obs_mask = hit_obstacle & (r >= p.min_range) & (r <= p.max_range)
        ground_mask = (~hit_obstacle) & np.isfinite(r) & (r <= p.clear_max_range)
        obstacles_opt = pts[obs_mask]
        ground_opt = pts[ground_mask]
        if len(ground_opt) and p.clear_cell_m > 0:
            # one point per ground cell (in the base frame x/y)
            gb = ground_opt @ self.r_cam.T + self.t_cam
            cells = np.floor(gb[:, :2] / p.clear_cell_m).astype(np.int64)
            _, idx = np.unique(cells, axis=0, return_index=True)
            ground_opt = ground_opt[np.sort(idx)]
        return obstacles_opt.astype(np.float32), ground_opt.astype(np.float32)


def _ray_shape(o: Obstacle, t: float, ox, oy, dx, dy):
    """Ray parameter (3D length along the unit ray) of entry/exit of the 2D shape
    extruded vertically; inf where missed. Works on the xy-projection: the
    parameter is the same along the 3D ray."""
    cx, cy = o.position(t)
    n = len(dx)
    t_in = np.full(n, np.inf)
    t_out = np.full(n, -np.inf)
    if o.type == 'cylinder':
        fx, fy = ox - cx, oy - cy
        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        cc = fx * fx + fy * fy - o.radius ** 2
        disc = b * b - 4 * a * cc
        ok = (disc >= 0) & (a > 1e-12)
        sq = np.sqrt(np.where(ok, disc, 0.0))
        a_safe = np.where(a > 1e-12, a, 1.0)
        t_in = np.where(ok, (-b - sq) / (2 * a_safe), np.inf)
        t_out = np.where(ok, (-b + sq) / (2 * a_safe), -np.inf)
        return t_in, t_out
    # box: slab test in the box frame
    c, s = math.cos(o.yaw), math.sin(o.yaw)
    lx, ly = to_local((ox, oy), cx, cy, o.yaw)
    ldx = c * dx + s * dy
    ldy = -s * dx + c * dy
    lo = np.full(n, -np.inf)
    hi = np.full(n, np.inf)
    for p0, dd, half in ((lx, ldx, o.size_x / 2), (ly, ldy, o.size_y / 2)):
        par = np.abs(dd) < 1e-12
        inside = abs(p0) <= half
        safe = np.where(par, 1.0, dd)
        t1 = (-half - p0) / safe
        t2 = (half - p0) / safe
        tl = np.where(par, -np.inf if inside else np.inf, np.minimum(t1, t2))
        th = np.where(par, np.inf if inside else -np.inf, np.maximum(t1, t2))
        lo = np.maximum(lo, tl)
        hi = np.minimum(hi, th)
    hit = lo <= hi
    return np.where(hit, lo, np.inf), np.where(hit, hi, -np.inf)


def bumper_sides(contacts: Sequence[Obstacle], x: float, y: float, yaw: float,
                 t: float, front_x: float = 0.15) -> Tuple[bool, bool]:
    """(left, right) bumper from the contacted obstacles. The Tron's bumper is the
    front shell: a contact counts when the obstacle centre lies ahead of
    ``front_x`` in base_link; left/right by its lateral side (both when central)."""
    left = right = False
    for o in contacts:
        lx, ly = to_local(o.position(t), x, y, yaw)
        if lx < front_x:
            continue
        if ly > 0.1:
            left = True
        elif ly < -0.1:
            right = True
        else:
            left = right = True
    return left, right
