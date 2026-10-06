"""Pure conversion OpenVINS ``odomimu`` -> EKF-ready body velocity (no ROS imports).

OpenVINS publishes ``/ov_msckf/odomimu`` with header.frame_id ``global`` (a
gravity-aligned frame with arbitrary yaw, origin where VIO initialised) and the twist
in the IMU body frame (``child_frame_id`` ``imu``; ``ROS2Visualizer::visualize_odometry``
fills ``linear`` from the propagated body velocity and ``angular`` from the gyro).

The EKF only fuses body velocity vx/vy from VIO (config/ekf_vio.yaml): the VIO pose
lives in its own drifting ``global`` frame, so its position/yaw are not published as
measurements (pose covariance is set huge and the EKF ignores pose for odom2 anyway).
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

UNUSED_VARIANCE = 1.0e6


def rotate(r: Sequence[Sequence[float]], v: Sequence[float]) -> Tuple[float, float, float]:
    return tuple(sum(r[i][k] * v[k] for k in range(3)) for i in range(3))  # type: ignore


def body_twist_covariance(ov_twist_cov: Sequence[float], scale: float,
                          floor_var: float) -> List[float]:
    """6x6 row-major twist covariance for the EKF.

    The linear block takes OpenVINS' own velocity variances times ``scale`` (OpenVINS
    is notoriously over-confident), never below ``floor_var``; off-diagonals are dropped
    (robot_localization only uses the diagonal sensibly). Angular rates are marked
    unused: the EKF takes yaw rate from the IMU / wheels, not from VIO.
    """
    cov = [0.0] * 36
    for i in range(3):
        v = ov_twist_cov[i * 6 + i] if len(ov_twist_cov) == 36 else float("nan")
        if not math.isfinite(v) or v < 0.0:
            v = float("nan")  # the gate rejects non-finite covariance
        else:
            v = max(v * scale, floor_var)
        cov[i * 7] = v
    for i in range(3, 6):
        cov[i * 7] = UNUSED_VARIANCE
    return cov


def unused_pose_covariance() -> List[float]:
    cov = [0.0] * 36
    for i in range(6):
        cov[i * 7] = UNUSED_VARIANCE
    return cov
