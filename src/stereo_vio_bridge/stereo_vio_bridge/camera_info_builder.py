"""Build ``sensor_msgs/CameraInfo`` + stereo rectification inputs from the recovered
Metoak calibration (``ros2_port_handoff/08_calibration_identity/*.yaml``).

The vendor ``cam0.yaml``/``cam1.yaml`` use a ``PINHOLE_FULL`` model whose distortion
coefficients are ordered ``{k1, k2, k3, p1, p2}``. OpenCV/ROS ``radtan`` (the standard
``plumb_bob`` model used by VINS/OpenVINS and ``sensor_msgs/CameraInfo``) expects
``{k1, k2, p1, p2, k3}`` — we reorder here.

``stereo_params.yaml`` carries the right-camera-relative-to-left rotation vector
(``Rx/Ry/Rz``, radians) and translation (``Tx/Ty/Tz``, **millimetres**), which we expose
for ``cv2.stereoRectify``.
"""
import os

import numpy as np


def parse_pinhole(path):
    """Parse a flat ``key: value`` calibration yaml into a dict."""
    d = {}
    with open(path) as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith('%') or line.startswith('---'):
                continue
            if ':' in line:
                k, v = line.split(':', 1)
                d[k.strip()] = v.strip()
    return d


def load_camera(path):
    """Load one cam*.yaml into a normalised dict with SI doubles.

    The vendor ``cam*.yaml`` nest scalars under ``projection_parameters:``/
    ``distortion_parameters:`` headers, but those headers carry no value so the flat
    parser yields plain ``fx``/``k1``/... keys directly.
    """
    d = parse_pinhole(path)
    return {
        'name': d.get('camera_name', 'camera'),
        'model': d.get('model_type', 'PINHOLE_FULL'),
        'width': int(d['image_width']),
        'height': int(d['image_height']),
        'fx': float(d['fx']), 'fy': float(d['fy']),
        'cx': float(d['cx']), 'cy': float(d['cy']),
        'k1': float(d['k1']), 'k2': float(d['k2']), 'k3': float(d['k3']),
        'p1': float(d['p1']), 'p2': float(d['p2']),
    }


def intrinsics(calib):
    """Return the 3x3 camera matrix ``K`` (numpy)."""
    return np.array([
        [calib['fx'], 0.0, calib['cx']],
        [0.0, calib['fy'], calib['cy']],
        [0.0, 0.0, 1.0],
    ])


def distortion(calib):
    """Return ``plumb_bob`` distortion ``[k1, k2, p1, p2, k3]`` (OpenCV radtan order)."""
    return np.array([calib['k1'], calib['k2'], calib['p1'], calib['p2'], calib['k3']])


def projection(calib):
    """Return ``P`` (3x4) = ``K | 0`` (no rectification)."""
    k = intrinsics(calib)
    p = np.zeros((3, 4))
    p[0:3, 0:3] = k
    return p


def camera_info_dict(calib):
    """Return a plain dict of the ROS ``CameraInfo`` fields (lists, ready to copy into
    a ``sensor_msgs.msg.CameraInfo`` of matching width/height)."""
    k = intrinsics(calib)
    return {
        'height': calib['height'],
        'width': calib['width'],
        'distortion_model': 'plumb_bob',
        'D': distortion(calib).tolist(),
        'K': k.reshape(-1).tolist(),
        'R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'P': projection(calib).reshape(-1).tolist(),
    }


def rodrigues(r):
    """Rotation vector -> 3x3 rotation matrix (Rodrigues formula; numpy)."""
    r = np.asarray(r, dtype=float)
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    k = r / th
    kx, ky, kz = k
    kk = np.array([[0.0, -kz, ky],
                   [kz, 0.0, -kx],
                   [-ky, kx, 0.0]])
    return np.eye(3) + np.sin(th) * kk + (1.0 - np.cos(th)) * (kk @ kk)


def load_stereo(path):
    """Parse stereo_params.yaml -> (R (3x3), T (3, in metres), base_m, bxf)."""
    d = parse_pinhole(path)
    rx, ry, rz = float(d['Rx']), float(d['Ry']), float(d['Rz'])
    tx, ty, tz = float(d['Tx']), float(d['Ty']), float(d['Tz'])
    return (rodrigues([rx, ry, rz]),
            np.array([tx, ty, tz]) / 1000.0,
            float(d['base']) / 1000.0,
            float(d['bxf']))
