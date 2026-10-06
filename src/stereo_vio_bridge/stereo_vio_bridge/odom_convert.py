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


def base_twist(r_base_imu: Sequence[Sequence[float]], imu_xyz_in_base: Sequence[float],
               v_imu: Sequence[float], w_imu: Sequence[float]):
    """IMU-frame body twist -> base_link twist: ``w_b = R w_i``, ``v_b = R v_i - w_b x r``.

    ``r`` = IMU position in base_link. The lever arm matters for an IMU off the rotation
    axis: the Metoak IMU sits 0.45 m ahead of base_link, so turning in place at 0.5 rad/s
    moves it sideways at 0.23 m/s, which is not lateral slip of the robot.
    """
    w = rotate(r_base_imu, w_imu)
    v = rotate(r_base_imu, v_imu)
    rx, ry, rz = imu_xyz_in_base
    wxr = (w[1] * rz - w[2] * ry, w[2] * rx - w[0] * rz, w[0] * ry - w[1] * rx)
    return (v[0] - wxr[0], v[1] - wxr[1], v[2] - wxr[2]), w


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
