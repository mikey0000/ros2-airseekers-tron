"""Tests for the OpenVINS odomimu -> /odometry/vio covariance/rotation helpers."""

import math

from stereo_vio_bridge.odom_convert import (UNUSED_VARIANCE, body_twist_covariance,
                                            rotate, unused_pose_covariance)


def _ov_cov(vx, vy, vz):
    cov = [0.0] * 36
    cov[0], cov[7], cov[14] = vx, vy, vz
    cov[1] = cov[6] = 0.5  # off-diagonal from OpenVINS, must be dropped
    return cov


def test_scaled_and_floored():
    cov = body_twist_covariance(_ov_cov(1e-6, 0.01, 0.002), scale=10.0, floor_var=0.0025)
    assert cov[0] == 0.0025           # floored
    assert math.isclose(cov[7], 0.1)  # scaled
    assert math.isclose(cov[14], 0.02)
    assert cov[1] == cov[6] == 0.0
    assert cov[21] == cov[28] == cov[35] == UNUSED_VARIANCE


def test_bad_covariance_becomes_nan_for_the_gate():
    cov = body_twist_covariance(_ov_cov(float("inf"), -1.0, 0.0), 10.0, 0.0025)
    assert math.isnan(cov[0]) and math.isnan(cov[7])
    assert body_twist_covariance([0.0] * 5, 10.0, 0.0025)[0] != 0.0025


def test_rotate_identity_and_yaw():
    assert rotate([[1, 0, 0], [0, 1, 0], [0, 0, 1]], (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)
    x, y, z = rotate([[0, -1, 0], [1, 0, 0], [0, 0, 1]], (1.0, 0.0, 0.0))
    assert (x, y, z) == (0, 1, 0)


def test_pose_unused():
    cov = unused_pose_covariance()
    assert [cov[i * 7] for i in range(6)] == [UNUSED_VARIANCE] * 6
