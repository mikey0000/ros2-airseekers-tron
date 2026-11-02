# SPDX-License-Identifier: GPL-3.0-or-later
"""mag_logic: tilt compensation, ellipsoid / ellipse fits on synthetic data, declination,
quality, evaluation helpers; mag_device: IIO discovery + register decoding."""
import math
import os
import struct

import numpy as np
import pytest

from mower_cameras import mag_device, mag_logic as ml

# a southern-hemisphere-like site: 55 uT, dip -65 deg (field points up out of the ground)
F, DIP = 55.0, math.radians(-65.0)
H, V_DOWN = F * math.cos(DIP), F * math.sin(DIP)


def world_field(decl_deg=0.0):
    """ENU world field (uT): horizontal towards magnetic north (declination east of true
    north), vertical component down = V_DOWN."""
    d = math.radians(decl_deg)
    return np.array([H * math.sin(d), H * math.cos(d), -V_DOWN])


def body(yaw, roll=0.0, pitch=0.0):
    """R (base_link -> ENU world) for an ENU yaw and roll / pitch."""
    return ml.rpy_matrix(roll, pitch, yaw)


def sample(yaw, roll=0.0, pitch=0.0, decl_deg=0.0):
    r = body(yaw, roll, pitch)
    m = r.T @ world_field(decl_deg)
    a = r.T @ np.array([0.0, 0.0, ml.G])        # specific force at rest: +g up
    return m, a


def test_bearing_level_cardinal_directions():
    # ENU yaw 90 deg = facing north -> bearing 0; yaw 0 = east -> 90; yaw 180 = west -> 270
    for yaw_deg, bearing_deg in ((90, 0), (0, 90), (180, -90), (-90, 180)):
        m, _a = sample(math.radians(yaw_deg))
        b = ml.level_field(m)['bearing']
        assert math.degrees(ml.wrap(b - math.radians(bearing_deg))) == pytest.approx(0, abs=1e-6)
        assert math.degrees(ml.enu_yaw(b)) == pytest.approx(ml.wrap_deg(yaw_deg), abs=1e-6)


@pytest.mark.parametrize('roll, pitch', [(10, 0), (0, -12), (8, 15), (-20, 5)])
def test_tilt_compensation_recovers_yaw(roll, pitch):
    for yaw_deg in range(-180, 180, 30):
        yaw = math.radians(yaw_deg)
        m, a = sample(yaw, math.radians(roll), math.radians(pitch))
        lf = ml.level_field(m, a)
        assert math.degrees(ml.wrap(ml.enu_yaw(lf['bearing']) - yaw)) == pytest.approx(0, abs=1e-6)
        assert lf['horizontal'] == pytest.approx(H, rel=1e-9)
        assert math.degrees(lf['inclination']) == pytest.approx(math.degrees(DIP), abs=1e-6)
    assert math.degrees(ml.level_field(*sample(0.3, math.radians(roll),
                                               math.radians(pitch)))['tilt']) == \
        pytest.approx(math.degrees(math.acos(math.cos(math.radians(roll)) *
                                             math.cos(math.radians(pitch)))), abs=1e-6)


def test_without_accel_tilt_costs_degrees():
    # dip 65 deg: the vertical field leaks into the horizontal plane by ~tan(dip) x tilt
    worst = max(abs(math.degrees(ml.wrap(ml.enu_yaw(ml.level_field(
        sample(math.radians(y), math.radians(10.0))[0])['bearing']) - math.radians(y))))
        for y in range(0, 360, 15))
    assert worst > 10.0


def test_declination_and_convergence():
    # 20 deg east declination: magnetic north is 20 deg clockwise of true north
    yaw = math.radians(37.0)
    m, a = sample(yaw, decl_deg=20.0)
    b = ml.level_field(m, a)['bearing']
    # uncorrected: the magnetic bearing is 20 deg short, the ENU yaw 20 deg CCW of the truth
    assert math.degrees(ml.wrap(ml.enu_yaw(b) - yaw)) == pytest.approx(20.0, abs=1e-6)
    assert math.degrees(ml.wrap(ml.enu_yaw(b, math.radians(20.0)) - yaw)) == \
        pytest.approx(0.0, abs=1e-6)
    # grid north 2 deg east of true north (convergence +2): grid bearing = true - 2
    assert math.degrees(ml.wrap(ml.enu_yaw(b, math.radians(20.0), math.radians(2.0)) - yaw)) \
        == pytest.approx(2.0, abs=1e-6)


def test_grid_convergence_matches_heading_logic_formula():
    assert ml.utm_grid_convergence(-41.0, 175.0) == pytest.approx(
        math.atan(math.tan(math.radians(175.0 - 177.0)) * math.sin(math.radians(-41.0))))
    assert ml.utm_grid_convergence(10.0, 177.0) == pytest.approx(0.0, abs=1e-12)


def distort(m, b, w_inv):
    return (w_inv @ m.T).T + b


def test_ellipsoid_fit_synthetic_3d():
    rng = np.random.default_rng(1)
    d = rng.normal(size=(2000, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    true = d * F
    a = np.array([[1.2, 0.08, -0.05], [0.08, 0.9, 0.03], [-0.05, 0.03, 1.05]])
    b = np.array([12.0, -30.0, 45.0])
    raw = distort(true, b, a) + rng.normal(scale=0.3, size=true.shape)
    c, w, r = ml.fit_ellipsoid(raw)
    assert np.allclose(c, b, atol=0.3)
    cal = (w @ (raw - c).T).T
    assert np.std(np.linalg.norm(cal, axis=1)) / r < 0.01
    # the map undoes the distortion up to a rotation: w @ a is orthogonal x scale
    m = w @ a
    s = m @ m.T
    assert np.allclose(s / s[0, 0], np.eye(3), atol=0.02)


def test_ellipse_fit_2d_and_planar_calibrate():
    rng = np.random.default_rng(2)
    yaws = rng.uniform(-math.pi, math.pi, 3000)
    a2 = np.array([[1.15, 0.06], [0.06, 0.92]])
    b = np.array([-8.0, 14.0, -25.0])
    m, acc = [], []
    for y in yaws:
        mm, aa = sample(y, math.radians(rng.normal(scale=1.0)), math.radians(rng.normal(scale=1.0)))
        mm = mm.copy()
        mm[:2] = a2 @ mm[:2]
        m.append(mm + b + rng.normal(scale=0.2, size=3))
        acc.append(aa + rng.normal(scale=0.02, size=3))
    m, acc = np.array(m), np.array(acc)
    assert ml.spread_ratio(m) < 0.25           # a mow: planar cloud
    cal = ml.calibrate(m, acc, vertical_ut=V_DOWN)
    assert cal.fit == '2d' and cal.z_observed
    assert np.allclose(cal.hard_iron, b, atol=0.6)
    assert cal.horizontal_ut == pytest.approx(H, rel=0.05)
    errs = []
    for i in range(0, len(m), 10):
        h = ml.heading(m[i], acc[i], cal)
        errs.append(math.degrees(ml.wrap(h['yaw'] - yaws[i])))
        assert h['quality'] > 0.7
    st = ml.error_stats(errs)
    assert st['p95'] < 2.0 and abs(st['bias']) < 1.0
    # no vertical field given: z stays unobserved (0) and the quality is capped
    cal2 = ml.calibrate(m, acc)
    assert cal2.fit == '2d' and not cal2.z_observed and cal2.hard_iron[2] == 0.0
    assert ml.heading(m[0], acc[0], cal2)['quality'] <= 0.8


def test_calibrate_3d_when_tipped():
    rng = np.random.default_rng(3)
    m, acc = [], []
    b = np.array([5.0, 5.0, -10.0])
    for _ in range(1500):
        mm, aa = sample(rng.uniform(-math.pi, math.pi), rng.uniform(-1.2, 1.2),
                        rng.uniform(-1.2, 1.2))
        m.append(mm + b)
        acc.append(aa)
    cal = ml.calibrate(np.array(m), np.array(acc))
    assert cal.fit == '3d' and cal.z_observed
    assert np.allclose(cal.hard_iron, b, atol=0.2)
    assert cal.field_ut == pytest.approx(F, rel=0.01)


def test_quality_drops_with_interference_tilt_and_acceleration():
    cal = ml.MagCalibration(calibrated=True, field_ut=F, horizontal_ut=H,
                            inclination_deg=math.degrees(DIP), z_observed=True, fit='2d')
    m, a = sample(0.5)
    assert ml.heading(m, a, cal)['quality'] == pytest.approx(1.0)
    # a steel object adds 8 uT horizontally: magnitude / dip change -> lower quality
    q = ml.heading(m + np.array([8.0, 0.0, 0.0]), a, cal)['quality']
    assert q < 0.8
    # accelerating at 1.5 m/s^2
    assert ml.heading(m, a * (1.0 + 1.5 / ml.G), cal)['quality'] < 0.3
    # tilted 20 deg (field still clean): tilt term
    m2, a2 = sample(0.5, math.radians(20.0))
    assert ml.heading(m2, a2, cal)['quality'] == pytest.approx(1.0 - 20.0 / 30.0, abs=1e-6)
    # no accelerometer: capped
    assert ml.heading(m, None, cal)['quality'] == pytest.approx(0.5)
    # uncalibrated: 0 but a heading is still reported
    h = ml.heading(m, a, ml.MagCalibration())
    assert h['quality'] == 0.0 and math.isfinite(h['yaw'])


def test_calibration_yaml_roundtrip(tmp_path):
    cal = ml.MagCalibration(hard_iron=(1.5, -2.0, 3.25), soft_iron=np.diag([1.1, 0.9, 1.0]),
                            field_ut=55.0, horizontal_ut=23.2, inclination_deg=-65.0,
                            calibrated=True, fit='2d', z_observed=False, yaw_offset_deg=-3.5,
                            mag_to_base_rpy_deg=[-90.917, 0.0, 90.0], samples=1234,
                            extra={'verdict': 'seed'})
    p = tmp_path / 'mag_calibration.yaml'
    p.write_text(cal.to_yaml('header line\nsecond'))
    got = ml.MagCalibration.load(str(p))
    assert got.calibrated and got.fit == '2d' and not got.z_observed
    assert np.allclose(got.hard_iron, cal.hard_iron) and np.allclose(got.soft_iron, cal.soft_iron)
    assert got.yaw_offset_deg == -3.5 and got.mag_to_base_rpy_deg == [-90.917, 0.0, 90.0]
    assert got.extra['verdict'] == 'seed' and got.samples == 1234
    assert not ml.MagCalibration.load(str(tmp_path / 'missing.yaml')).calibrated


def test_repo_template_is_uncalibrated():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, '..', '..', '..', 'config', 'mag_calibration.yaml')
    if not os.path.exists(path):
        pytest.skip('repo config not next to the package')
    cal = ml.MagCalibration.load(path)
    assert not cal.calibrated and np.allclose(cal.soft_iron, np.eye(3))


def test_error_stats_and_verdict():
    rng = np.random.default_rng(4)
    e = 3.0 + rng.normal(scale=2.0, size=5000)
    st = ml.error_stats(e)
    assert st['bias'] == pytest.approx(3.0, abs=0.1)
    assert st['std'] == pytest.approx(2.0, abs=0.1)
    assert st['p95'] == pytest.approx(1.96 * 2.0, abs=0.2)
    assert st['p95_raw'] > st['p95']
    # wrap-safe around +-180
    assert ml.error_stats([179.0, -179.0])['bias'] in (pytest.approx(180.0), pytest.approx(-180.0))
    assert ml.verdict(st['p95']) == 'seed'
    assert ml.verdict(15.0) == 'watchdog' and ml.verdict(40.0) == 'not usable'
    assert ml.verdict(None) == 'not usable'


def test_mirrored_axis_detected():
    ref = np.linspace(-170, 170, 200)
    assert not ml.mirrored(ref + 5.0, ref)
    assert ml.mirrored(-ref + 30.0, ref)


def test_imu_to_base_default_matches_urdf_composition():
    # base_link <- stereo_camera_optical (rpy -1.5547956 0 -1.5707963) <- imu (Ry(pi))
    r = ml.rpy_matrix(-1.5547956, 0.0, -1.5707963) @ ml.rpy_matrix(0.0, math.pi, 0.0)
    assert np.allclose(ml.rpy_deg_matrix([-90.917, 0.0, 90.0]), r, atol=1e-4)
    # 2026-10-07 rest reading of the chip: gravity comes out (almost) +z in base_link
    g = r @ np.array([-0.03, -9.79, 0.32])
    assert g[2] > 9.7


# ---------------------------------------------------------------------- mag_device
def _iio_dev(root, n, name, files):
    d = root / ('iio:device%d' % n)
    d.mkdir()
    (d / 'name').write_text(name + '\n')
    for k, v in files.items():
        (d / k).write_text(v + '\n')
    return d


def test_find_and_read_iio_mag(tmp_path):
    _iio_dev(tmp_path, 1, 'icm40608-gyro', {'in_anglvel_x_raw': '0'})
    _iio_dev(tmp_path, 3, 'lis2mdl', {
        'in_magn_x_raw': '100', 'in_magn_y_raw': '-200', 'in_magn_z_raw': '300',
        'in_magn_x_scale': '0.001500', 'in_magn_y_scale': '0.001500',
        'in_magn_z_scale': '0.001500', 'sampling_frequency': '10',
        'sampling_frequency_available': '10 20 50 100'})
    assert mag_device.find_iio_mag(str(tmp_path)) == 'iio:device3'
    dev = mag_device.open_mag('iio', root=str(tmp_path))
    try:
        assert dev.odr == 100.0
        assert dev.read() == pytest.approx((15.0, -30.0, 45.0))
    finally:
        dev.close()


def test_iio_shared_scale_and_channel_match(tmp_path):
    _iio_dev(tmp_path, 0, 'magn_vendor', {
        'in_magn_x_raw': '1', 'in_magn_y_raw': '2', 'in_magn_z_raw': '3',
        'in_magn_scale': '0.0015'})
    assert mag_device.find_iio_mag(str(tmp_path)) == 'iio:device0'
    dev = mag_device.IioMag('iio:device0', root=str(tmp_path))
    try:
        assert dev.read() == pytest.approx((0.15, 0.3, 0.45))
    finally:
        dev.close()


def test_open_mag_reports_every_backend(tmp_path):
    with pytest.raises(OSError) as exc:
        mag_device.open_mag('auto', i2c_bus=99, root=str(tmp_path))
    assert 'iio' in str(exc.value) and 'i2c-99' in str(exc.value)


def test_decode_lis2mdl_output_registers():
    raw = struct.pack('<3h', 1000, -1000, 32767)
    assert mag_device.decode_xyz(raw) == pytest.approx((150.0, -150.0, 4915.05))
