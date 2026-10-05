# Copyright 2026 The mower_docking authors
# SPDX-License-Identifier: Apache-2.0
"""ArUco detection + single-marker pose (OpenCV), old and new cv2.aruco APIs.

Vendor ``mower_charge`` (``libmower_charge_core.so``, ``docking::marker``
constructor, disassembled): ``cv::aruco::getPredefinedDictionary(0)`` =
``DICT_4X4_50`` and ``marker_size_ = 0.04f`` (source comment: "方块的宽度4cm",
"square width 4 cm"), pose via ``cv::aruco::estimatePoseSingleMarkers``.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

try:  # cv2 is optional so the pure logic and most tests run without it
    import cv2  # type: ignore
    import numpy as np
    HAVE_CV2 = hasattr(cv2, 'aruco')
except Exception:  # pragma: no cover - depends on the environment
    cv2 = None
    np = None
    HAVE_CV2 = False


def _dictionary(name: str):
    did = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return cv2.aruco.getPredefinedDictionary(did)
    return cv2.aruco.Dictionary_get(did)  # pragma: no cover - very old cv2


def _params():
    if hasattr(cv2.aruco, 'DetectorParameters_create'):  # OpenCV < 4.7
        p = cv2.aruco.DetectorParameters_create()
    else:  # pragma: no cover
        p = cv2.aruco.DetectorParameters()
    try:
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    except Exception:  # pragma: no cover
        pass
    return p


class ArucoDetector:
    """Detect markers of one dictionary and estimate their pose with solvePnP.

    ``detect`` returns a list of ``(marker_id, rvec, tvec, corners)`` with the
    pose of the marker in the camera optical frame.  Object points follow the
    OpenCV ArUco convention (marker X right, Y up, Z out of the marker).
    """

    def __init__(self, dictionary: str = 'DICT_4X4_50', marker_size: float = 0.04):
        if not HAVE_CV2:
            raise RuntimeError('cv2.aruco is not available')
        self.dictionary = _dictionary(dictionary)
        self.params = _params()
        self.marker_size = float(marker_size)
        self._detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):  # OpenCV >= 4.7
            self._detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
        h = self.marker_size / 2.0
        self.obj_pts = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
                                dtype=np.float64)

    def detect_corners(self, gray):
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary,
                                                      parameters=self.params)
        return corners, ids

    def detect(self, gray, K: Sequence[float], D: Sequence[float],
               marker_id: int = -1) -> List[Tuple[int, list, list, object]]:
        corners, ids = self.detect_corners(gray)
        out = []
        if ids is None or len(ids) == 0:
            return out
        kmat = np.asarray(K, dtype=np.float64).reshape(3, 3)
        dvec = np.asarray(D, dtype=np.float64).reshape(-1)
        flag = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
        for c, i in zip(corners, ids.reshape(-1)):
            if marker_id >= 0 and int(i) != marker_id:
                continue
            img_pts = np.asarray(c, dtype=np.float64).reshape(4, 2)
            ok, rvec, tvec = cv2.solvePnP(self.obj_pts, img_pts, kmat, dvec, flags=flag)
            if not ok:
                continue
            out.append((int(i), rvec.reshape(3).tolist(), tvec.reshape(3).tolist(), img_pts))
        # closest first
        out.sort(key=lambda r: r[2][2])
        return out


def generate_marker_image(dictionary: str, marker_id: int, side_px: int):
    """Marker bitmap (black/white), for printing and tests."""
    d = _dictionary(dictionary)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(d, marker_id, side_px)
    return cv2.aruco.drawMarker(d, marker_id, side_px)


def to_gray(img_msg_encoding: str, data, height: int, width: int, step: int) -> Optional[object]:
    """Convert a sensor_msgs/Image buffer to a mono8 numpy image (no cv_bridge)."""
    enc = img_msg_encoding.lower()
    buf = np.frombuffer(bytes(data) if not isinstance(data, (bytes, bytearray)) else data,
                        dtype=np.uint8)
    if enc in ('mono8', '8uc1'):
        return buf.reshape(height, step)[:, :width]
    if enc in ('bgr8', 'rgb8', '8uc3'):
        img = buf.reshape(height, step)[:, :width * 3].reshape(height, width, 3)
        code = cv2.COLOR_RGB2GRAY if enc == 'rgb8' else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(img, code)
    if enc in ('bgra8', 'rgba8'):
        img = buf.reshape(height, step)[:, :width * 4].reshape(height, width, 4)
        code = cv2.COLOR_RGBA2GRAY if enc == 'rgba8' else cv2.COLOR_BGRA2GRAY
        return cv2.cvtColor(img, code)
    if enc in ('yuv422_yuy2', 'yuyv'):
        img = buf.reshape(height, step)[:, :width * 2].reshape(height, width, 2)
        return np.ascontiguousarray(img[:, :, 0])  # Y channel
    if enc in ('uyvy', 'yuv422'):  # ROS 'yuv422' is UYVY
        img = buf.reshape(height, step)[:, :width * 2].reshape(height, width, 2)
        return np.ascontiguousarray(img[:, :, 1])
    return None
