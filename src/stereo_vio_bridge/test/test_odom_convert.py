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


def test_base_twist_removes_lever_arm_of_metoak_imu():
    import math
    from stereo_vio_bridge.odom_convert import base_twist
    from stereo_vio_bridge.vio_odom_bridge import _rpy_matrix
    r = _rpy_matrix(-1.586797, 0.0, 1.570796)            # stereo_camera_imu -> base_link
    pos = (0.451, -0.0514, 0.2352)
    r_t = [[r[j][i] for j in range(3)] for i in range(3)]  # base -> imu
    # robot turning in place at 0.5 rad/s: base twist v=0, w=(0,0,0.5)
    wb = (0.0, 0.0, 0.5)
    v_imu_base = (wb[1] * pos[2] - wb[2] * pos[1], wb[2] * pos[0] - wb[0] * pos[2], 0.0)
    rot = lambda m, v: tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))  # noqa
    v, w = base_twist(r, pos, rot(r_t, v_imu_base), rot(r_t, wb))
    assert all(abs(a) < 1e-9 for a in v)
    assert math.isclose(w[2], 0.5, abs_tol=1e-9)
    # straight 0.4 m/s forward: IMU sees it along its -z (camera forward = IMU -z)
    v, w = base_twist(r, pos, rot(r_t, (0.4, 0.0, 0.0)), (0.0, 0.0, 0.0))
    assert math.isclose(v[0], 0.4, abs_tol=1e-9) and abs(v[1]) < 1e-9
    assert rot(r_t, (0.4, 0.0, 0.0))[2] < -0.39


def test_base_twist_identity_is_passthrough():
    from stereo_vio_bridge.odom_convert import base_twist
    eye = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    assert base_twist(eye, (0, 0, 0), (1, 2, 3), (0.1, 0.2, 0.3)) == ((1, 2, 3), (0.1, 0.2, 0.3))
