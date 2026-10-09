# SPDX-License-Identifier: Apache-2.0
"""Synthetic rear camera: renders the dock's ArUco marker with a pinhole model.

mower_docking detects the marker itself (``/rear_camera/image_raw`` +
``camera_info``, DICT_4X4_50, 4 cm), so the sim renders a mono8 image instead of
faking ``~/marker_pose``; the docking node runs unmodified.

Geometry (mower_docking dock_logic.dock_errors): the docked base_link origin lies
``docked_marker_offset`` (0.45 m) from the marker along the marker's outward
normal, and the docked heading equals the normal's heading. So for a dock pose
(x, y, yaw) the marker centre is at dock-local (-offset, 0, marker_z) facing +x.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .sensors import rpy_matrix

REAR_CAMERA_XYZ = (-0.201, 0.0, 0.25)
REAR_CAMERA_RPY = (-1.5707963, 0.0, 1.5707963)


@dataclass
class RearCameraParams:
    # config/cameras/rear_camera_info_640x480.yaml (the live 640x480 mode), no distortion
    width: int = 640
    height: int = 480
    fx: float = 684.67
    fy: float = 684.33
    cx: float = 331.69
    cy: float = 231.39
    marker_size: float = 0.04
    marker_id: int = 0
    marker_offset: float = 0.45     # docking.yaml docked_marker_offset
    marker_z: float = 0.25
    background: int = 110
    quiet_zone_cells: int = 2


def _marker_bitmap(marker_id: int, cells_px: int = 20, quiet: int = 2):
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) \
        if hasattr(cv2.aruco, 'getPredefinedDictionary') else \
        cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    side = 6 * cells_px  # 4x4 bits + 1-cell black border each side
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    else:
        img = cv2.aruco.drawMarker(dictionary, marker_id, side)
    pad = quiet * cells_px
    return np.pad(img, pad, constant_values=255), side, pad


class RearCamera:
    def __init__(self, params: Optional[RearCameraParams] = None,
                 xyz=REAR_CAMERA_XYZ, rpy=REAR_CAMERA_RPY):
        self.p = params or RearCameraParams()
        self.t_cam = np.array(xyz, dtype=float)
        self.r_cam = rpy_matrix(*rpy)      # optical -> base_link
        self._bitmap = None

    @property
    def K(self):
        p = self.p
        return [p.fx, 0.0, p.cx, 0.0, p.fy, p.cy, 0.0, 0.0, 1.0]

    def marker_corners_map(self, dock_x, dock_y, dock_yaw, half):
        """Marker-plane square of half-size ``half`` in the map frame, OpenCV order
        (top-left, top-right, bottom-right, bottom-left seen from the front)."""
        c, s = math.cos(dock_yaw), math.sin(dock_yaw)
        p = self.p
        cx, cy = dock_x - p.marker_offset * c, dock_y - p.marker_offset * s
        xm = np.array([-s, c, 0.0])        # marker +x (image right when facing it)
        ym = np.array([0.0, 0.0, 1.0])     # marker +y (up)
        ctr = np.array([cx, cy, p.marker_z])
        return [ctr + (-half) * xm + half * ym, ctr + half * xm + half * ym,
                ctr + half * xm - half * ym, ctr - half * xm - half * ym]

    def project(self, pts_map, x, y, yaw):
        """Map points -> pixel (u, v) and optical depth z."""
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for pm in pts_map:
            dx, dy = pm[0] - x, pm[1] - y
            pb = np.array([c * dx + s * dy, -s * dx + c * dy, pm[2]])
            po = self.r_cam.T @ (pb - self.t_cam)
            ok = po[2] > 1e-6
            out.append((self.p.fx * po[0] / po[2] + self.p.cx if ok else math.nan,
                        self.p.fy * po[1] / po[2] + self.p.cy if ok else math.nan, po[2]))
        return out

    def render(self, x, y, yaw, dock_x, dock_y, dock_yaw) -> np.ndarray:
        """mono8 image (H x W) of the scene from the robot pose (x, y, yaw)."""
        import cv2
        p = self.p
        img = np.full((p.height, p.width), p.background, dtype=np.uint8)
        if self._bitmap is None:
            self._bitmap = _marker_bitmap(p.marker_id, quiet=p.quiet_zone_cells)
        bmp, side, pad = self._bitmap
        full = bmp.shape[0]
        half_full = p.marker_size / 2.0 * full / side
        proj = self.project(self.marker_corners_map(dock_x, dock_y, dock_yaw, half_full), x, y, yaw)
        if any(not (z > 0.05) for _, _, z in proj):
            return img
        dst = np.array([[u, v] for u, v, _ in proj], dtype=np.float32)
        if not np.all(np.isfinite(dst)):
            return img
        # facing check: the camera must see the marker's front side
        cross = (dst[1, 0] - dst[0, 0]) * (dst[3, 1] - dst[0, 1]) - \
                (dst[1, 1] - dst[0, 1]) * (dst[3, 0] - dst[0, 0])
        if cross <= 0:
            return img
        src = np.array([[0, 0], [full, 0], [full, full], [0, full]], dtype=np.float32)
        H = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(bmp, H, (p.width, p.height), flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        mask = cv2.warpPerspective(np.full_like(bmp, 255), H, (p.width, p.height),
                                   flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT,
                                   borderValue=0)
        img[mask > 0] = warped[mask > 0]
        return img
