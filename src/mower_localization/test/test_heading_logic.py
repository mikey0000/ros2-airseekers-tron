"""Unit tests for the heading alignment estimator (pure Python, no ROS needed)."""

import math

import pytest

from mower_localization import heading_logic as hl

D = math.radians


class Sim:
    """Drives a HeadingEstimator with a simulated robot: true ENU heading ``yaw``,
    IMU yaw = yaw - true_offset (+ drift), 100 Hz IMU, 10 Hz GPS, RTK fixed."""

    def __init__(self, true_offset_deg=-150.0, fix_type=3, **params):
        self.est = hl.HeadingEstimator(hl.AlignParams(**params))
        self.t = 0.0
        self.x = self.y = 0.0
        self.yaw = 0.0
        self.true_offset = D(true_offset_deg)
        self.drift = 0.0
        self.fix_type = fix_type
        self.results = []
        self.gps_noise = None

    def imu_yaw(self):
        return hl.wrap(self.yaw - self.true_offset + self.drift)

    def run(self, v, w, seconds, gps_lateral_wobble=0.0, imu_wobble=0.0):
        steps = int(round(seconds * 100))
        for k in range(steps):
            self.t += 0.01
            self.yaw = hl.wrap(self.yaw + w * 0.01)
            self.x += v * math.cos(self.yaw) * 0.01
            self.y += v * math.sin(self.yaw) * 0.01
            wob = imu_wobble * math.sin(k * 0.05)
            self.est.on_imu(self.t, hl.wrap(self.imu_yaw() + wob), w)
            if k % 10 == 0:
                self.est.on_cmd(self.t, v, w)
                self.est.on_fix_type(self.t, self.fix_type)
                lat = gps_lateral_wobble * math.sin(k * 0.02)
                gx = self.x - math.sin(self.yaw) * lat
                gy = self.y + math.cos(self.yaw) * lat
                r = self.est.on_gps(self.t, gx, gy, 0.2)
                if r is not None:
                    self.results.append(r)

    def ramp_drift(self, deg, steps=10):
        """Change the IMU error in small steps (below the IMU-reset detector)."""
        for _ in range(steps):
            self.drift += D(deg) / steps
            self.t += 0.01
            self.est.on_imu(self.t, self.imu_yaw(), 0.0)

    def heading_error(self):
        return math.degrees(hl.wrap(self.est.apply(self.imu_yaw()) - self.yaw))


def test_wrap_and_rotate():
    assert hl.wrap(D(190)) == pytest.approx(D(-170))
    assert hl.wrap(D(-180)) == pytest.approx(D(180))
    # rotating an attitude with roll/pitch keeps them and adds the offset to yaw
    r, p, y = D(5), D(-3), D(40)
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2),
                              math.sin(p / 2), math.cos(y / 2), math.sin(y / 2))
    q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
         cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
    q2 = hl.rotate_yaw(q, D(100))
    assert math.degrees(hl.yaw_from_quaternion(*q2)) == pytest.approx(140.0, abs=1e-6)
    x, yy, z, w = q2
    roll = math.atan2(2 * (w * x + yy * z), 1 - 2 * (x * x + yy * yy))
    pitch = math.asin(2 * (w * yy - z * x))
    assert math.degrees(roll) == pytest.approx(5.0, abs=1e-6)
    assert math.degrees(pitch) == pytest.approx(-3.0, abs=1e-6)


@pytest.mark.parametrize('heading_deg', [0.0, 75.0, -120.0, 179.0])
def test_cog_forward_aligns(heading_deg):
    s = Sim(true_offset_deg=-150.0)
    s.yaw = D(heading_deg)
    assert not s.est.aligned and s.est.source == hl.SOURCE_NONE
    s.run(0.3, 0.0, 3.0)        # 0.9 m
    assert s.est.aligned and s.est.source == hl.SOURCE_COG
    assert abs(s.heading_error()) < 1.0
    assert math.degrees(hl.wrap(s.est.offset - s.true_offset)) == pytest.approx(0, abs=1.0)


def test_cog_reverse_adds_pi():
    s = Sim(true_offset_deg=33.0)
    s.yaw = D(60.0)
    s.run(-0.2, 0.0, 4.0)       # 0.8 m backwards
    assert s.est.aligned
    assert abs(s.heading_error()) < 1.0


def test_window_needs_min_distance_and_straight_command():
    s = Sim()
    s.run(0.3, 0.0, 1.5)        # 0.45 m < 0.6
    assert not s.est.aligned
    s.run(0.3, 0.6, 5.0)        # real turn (gyro > 0.5 rad/s): window discarded
    assert not s.est.aligned
    s.run(0.04, 0.0, 10.0)      # too slow (< min_speed 0.06)
    assert not s.est.aligned
    s.run(0.3, 0.0, 2.5)
    assert s.est.aligned


def test_requires_rtk_and_float_needs_longer_window():
    s = Sim(fix_type=1)
    s.run(0.3, 0.0, 6.0)
    assert not s.est.aligned
    s = Sim(fix_type=2)
    s.run(0.3, 0.0, 3.0)        # 0.9 m < 1.5 m float distance
    assert not s.est.aligned
    s.run(0.3, 0.0, 3.0)
    assert s.est.aligned


def test_curved_track_rejected():
    s = Sim()
    s.run(0.3, 0.0, 3.0, gps_lateral_wobble=0.2)
    assert not s.est.aligned
    assert any('not straight' in r.get('reason', '') for r in s.results)


def test_low_pass_and_outlier_rejection():
    s = Sim(true_offset_deg=10.0)
    s.run(0.3, 0.0, 2.2)        # first window: seed
    assert s.est.n_cog == 1
    # slow gyro drift of 4 deg: tracked by repeated COG updates
    s.drift = D(4.0)
    s.run(0.3, 0.0, 2.1)
    off1 = s.est.offset
    assert math.degrees(hl.wrap(off1 - s.true_offset)) == pytest.approx(-4.0 * 0.3, abs=0.3)
    for _ in range(20):
        s.run(0.3, 0.0, 2.1)
    assert abs(s.heading_error()) < 0.5
    # one bad window (40 deg off) is rejected
    n = s.est.n_cog
    s.ramp_drift(40.0)          # mid-window: that window fails the IMU-span check
    events = []
    for _ in range(2):
        s.run(0.3, 0.0, 2.1)
        events.append(s.est.last_event)
    assert s.est.n_cog == n and any('outlier' in e for e in events), events
    s.ramp_drift(-40.0)
    for _ in range(2):
        s.run(0.3, 0.0, 2.1)
    assert s.est.n_cog > n and abs(s.heading_error()) < 0.5


def test_consistent_outliers_reseed():
    s = Sim(true_offset_deg=10.0)
    s.run(0.3, 0.0, 2.2)
    s.ramp_drift(90.0)          # e.g. undetected IMU restart
    events = []
    for _ in range(4):
        s.run(0.3, 0.0, 2.1)
        events.append(s.est.last_event)
    assert any('re-seeded' in e for e in events), events
    assert abs(s.heading_error()) < 1.0


def test_dock_seed_sign_convention():
    s = Sim(true_offset_deg=70.0, dock_yaw_trusted=True)
    s.yaw = D(-100.0)            # docked robot heading == dock yaw (faces out of the dock)
    s.est.on_dock_pose(0.0, 0.0, D(-100.0))
    s.run(0.0, 0.0, 0.5)
    s.est.on_docked(s.t, True)
    assert s.est.source == hl.SOURCE_NONE   # settle time
    s.run(0.0, 0.0, 2.5)
    s.est.on_docked(s.t, True)
    assert s.est.source == hl.SOURCE_DOCK and s.est.aligned
    assert abs(s.heading_error()) < 0.1
    # undock forward along the dock yaw: COG refines (agrees), source becomes cog
    s.est.on_docked(s.t, False)
    s.run(0.15, 0.0, 6.0)
    assert s.est.source == hl.SOURCE_COG and abs(s.heading_error()) < 1.0


def test_dock_seed_does_not_override_fresh_cog_and_wrong_dock_yaw_replaced():
    s = Sim(true_offset_deg=70.0, dock_yaw_trusted=True)
    s.est.on_dock_pose(0.0, 0.0, D(0.0))    # unmeasured dock yaw; robot really faces 120
    s.yaw = D(120.0)
    s.run(0.0, 0.0, 0.1)
    s.est.on_docked(s.t, True)
    s.run(0.0, 0.0, 3.0)
    s.est.on_docked(s.t, True)
    assert s.est.source == hl.SOURCE_DOCK
    s.est.on_docked(s.t, False)
    s.run(0.15, 0.0, 6.0)
    assert s.est.source == hl.SOURCE_COG and abs(s.heading_error()) < 1.0
    assert 'replaces dock seed' in s.est.last_event or s.est.n_cog > 1
    s.est.on_docked(s.t, True)
    s.run(0.0, 0.0, 3.0)
    s.est.on_docked(s.t, True)
    assert s.est.source == hl.SOURCE_COG


def test_file_seed_is_not_aligned_and_cog_replaces_it(tmp_path):
    path = str(tmp_path / 'sub' / 'heading_offset.yaml')
    assert hl.load_offset(path) is None
    hl.save_offset(path, D(-123.4), 'cog', 5)
    assert math.degrees(hl.load_offset(path)) == pytest.approx(-123.4, abs=1e-3)
    s = Sim(true_offset_deg=10.0)
    s.est.load_persisted(hl.load_offset(path), 0.0)
    assert s.est.source == hl.SOURCE_FILE and not s.est.aligned
    assert s.est.quality(0.0) == 'low'
    assert math.degrees(s.est.yaw_sigma(0.0)) == pytest.approx(45.0)
    s.run(0.3, 0.0, 2.2)
    assert s.est.source == hl.SOURCE_COG and abs(s.heading_error()) < 1.0
    assert s.est.dirty


def test_bad_file_ignored(tmp_path):
    p = tmp_path / 'h.yaml'
    p.write_text('offset_rad: 1.0\n')
    assert hl.load_offset(str(p)) is None
    p.write_text(hl.FILE_HEADER + '\noffset_rad: nan\n')
    assert hl.load_offset(str(p)) is None


def test_imu_reset_drops_alignment():
    s = Sim(true_offset_deg=10.0)
    s.run(0.3, 0.0, 2.2)
    assert s.est.aligned
    s.est.on_imu(s.t + 0.01, hl.wrap(s.imu_yaw() + D(95.0)), 0.0)
    assert not s.est.aligned and s.est.offset is None
    # a fast but real turn is not a reset
    s = Sim(true_offset_deg=10.0)
    s.run(0.3, 0.0, 2.2)
    s.run(0.0, 1.0, 1.0)
    assert s.est.aligned


def test_yaw_sigma_grows_with_age():
    s = Sim()
    s.run(0.3, 0.0, 2.2)
    s0 = s.est.yaw_sigma(s.t)
    assert math.degrees(s0) == pytest.approx(3.0, abs=0.1)
    assert s.est.yaw_sigma(s.t + 600) > s0 + D(9.0)
    st = s.est.status(s.t)
    assert st['aligned'] and st['source'] == 'cog' and st['quality'] == 'good'
    assert hl.status_json(st).startswith('{')


def test_imu_cdr_rewrite_matches_struct_layout():
    import struct
    frame = b'imu_link\0'
    head = struct.pack('<iI', 5, 6) + struct.pack('<I', len(frame)) + frame
    payload = head
    pad = (-len(payload)) % 8
    body = struct.pack('<4d9d3d9d3d9d', *([0.0, 0.0, 0.0, 1.0] + [0.0] * 8 + [1.0]
                                          + [0.0, 0.0, 0.5] + [0.0] * 9 + [0.0, 0.0, 9.8]
                                          + [0.0] * 9))
    data = b'\x00\x01\x00\x00' + payload + b'\0' * pad + body
    pos = hl.imu_tail_offset(data)
    q, gz = hl.imu_fields(data, pos)
    assert q == (0.0, 0.0, 0.0, 1.0) and gz == 0.5
    out = hl.imu_rewrite(data, pos, (0.0, 0.0, 1.0, 0.0), 0.01)
    v = struct.unpack_from('<4d9d', out, pos)
    assert v[:4] == (0.0, 0.0, 1.0, 0.0) and v[12] == 0.01 and v[4] == 0.0
    assert out[pos + 13 * 8:] == data[pos + 13 * 8:]


def test_imu_cdr_against_rclpy():
    try:
        from rclpy.serialization import serialize_message, deserialize_message
        from sensor_msgs.msg import Imu
    except ImportError:
        pytest.skip('needs rclpy')
    m = Imu()
    m.header.frame_id = 'imu_link'
    m.orientation.w = 1.0
    m.angular_velocity.z = 0.25
    m.orientation_covariance[8] = 1.0
    data = serialize_message(m)
    pos = hl.imu_tail_offset(data)
    q, gz = hl.imu_fields(data, pos)
    assert q == (0.0, 0.0, 0.0, 1.0) and gz == 0.25
    q2 = hl.rotate_yaw(q, D(90.0))
    m2 = deserialize_message(hl.imu_rewrite(data, pos, q2, 0.002), Imu)
    assert math.degrees(hl.yaw_from_quaternion(m2.orientation.x, m2.orientation.y,
                                               m2.orientation.z, m2.orientation.w)) \
        == pytest.approx(90.0)
    assert m2.orientation_covariance[8] == pytest.approx(0.002)
    assert m2.angular_velocity.z == 0.25 and m2.header.frame_id == 'imu_link'


# ---------------------------------------------------------------- persisted continuity
def _restart(tmp_path, *, yaw_shift_deg=0.0, move=(0.0, 0.0), age_s=60.0, fix_type=3):
    """Align on COG, save the record, then 'restart': a new estimator loads it."""
    path = str(tmp_path / 'heading_offset.yaml')
    s = Sim(true_offset_deg=-150.0)
    s.yaw = D(30.0)
    s.run(0.3, 0.0, 2.2)
    assert s.est.aligned
    s.run(0.0, 0.0, 0.5)                    # stop; last GPS position recorded
    off, imu, x, y = s.est.persist_record()
    hl.save_offset(path, off, s.est.source, s.est.n_cog, now_wall=1000.0, imu_yaw=imu,
                   x=x, y=y)
    rec = hl.load_record(path)
    assert rec['imu_yaw'] == pytest.approx(imu, abs=1e-5) and rec['saved_wall'] == 1000.0
    e = hl.HeadingEstimator()
    e.load_persisted(rec['offset'], 0.0, record=rec, now_wall=1000.0 + age_s)
    assert e.source == hl.SOURCE_FILE and not e.aligned
    e.on_imu(0.1, hl.wrap(imu + D(yaw_shift_deg)), 0.0)
    e.on_fix_type(0.1, fix_type)
    for k in range(5):
        e.on_gps(0.1 + 0.1 * k, x + move[0], y + move[1], 0.2)
    return e


def test_persisted_continuity_accepted(tmp_path):
    e = _restart(tmp_path, yaw_shift_deg=2.0, move=(0.1, -0.1))
    assert e.source == hl.SOURCE_FILE_VERIFIED and e.aligned
    assert e.quality(0.1) == 'good'
    assert math.degrees(e.yaw_sigma(0.1)) == pytest.approx(5.0)
    assert 'verified' in e.last_event


def test_persisted_yaw_jump_rejected(tmp_path):
    e = _restart(tmp_path, yaw_shift_deg=40.0)
    assert e.source == hl.SOURCE_FILE and not e.aligned and 'IMU yaw' in e.last_event


def test_persisted_moved_rejected(tmp_path):
    e = _restart(tmp_path, move=(0.6, 0.0))
    assert e.source == hl.SOURCE_FILE and not e.aligned and 'moved' in e.last_event


def test_persisted_stale_rejected(tmp_path):
    e = _restart(tmp_path, age_s=7 * 3600.0)
    assert e.source == hl.SOURCE_FILE and not e.aligned


def test_persisted_needs_rtk(tmp_path):
    e = _restart(tmp_path, fix_type=1)
    assert e.source == hl.SOURCE_FILE and not e.aligned


def test_old_file_without_imu_yaw_stays_file(tmp_path):
    p = str(tmp_path / 'h.yaml')
    hl.save_offset(p, D(10.0), 'cog', 1)
    rec = hl.load_record(p)
    assert rec['imu_yaw'] is None
    e = hl.HeadingEstimator()
    e.load_persisted(rec['offset'], 0.0, record=rec)
    e.on_imu(0.1, 0.0, 0.0)
    e.on_fix_type(0.1, 3)
    for k in range(5):
        e.on_gps(0.1 + 0.1 * k, 0.0, 0.0, 0.2)
    assert e.source == hl.SOURCE_FILE


def test_untrusted_dock_yaw_never_seeds():
    """A 0.0 placeholder dock yaw once poisoned the persisted offset: no seed unless trusted."""
    s = Sim(true_offset_deg=70.0)
    s.est.on_dock_pose(0.0, 0.0, 0.0)
    s.run(0.0, 0.0, 0.1)
    s.est.on_docked(s.t, True)
    s.run(0.0, 0.0, 3.0)
    s.est.on_docked(s.t, True)
    assert s.est.source == hl.SOURCE_NONE and not s.est.aligned


def test_joystick_jitter_does_not_reset_window():
    """Small commanded angular jitter (GUI joystick) must not kill the COG window; the
    straightness check is the GPS chord + gyro."""
    s = Sim(true_offset_deg=-150.0)
    steps = 0
    while steps < 800 and s.est.source != hl.SOURCE_COG:
        # alternate +-0.12 rad/s commanded jitter while the robot actually drives straight
        s.est.on_cmd(s.t, 0.3, 0.12 if steps % 2 else -0.12)
        s.run(0.3, 0.0, 0.1)
        steps += 1
    assert s.est.source == hl.SOURCE_COG and abs(s.heading_error()) < 1.0


def test_real_turn_resets_window_with_reason():
    s = Sim(true_offset_deg=-150.0)
    s.run(0.3, 0.0, 1.0)          # 0.3 m straight, window open
    s.run(0.3, 0.5, 0.5)          # real turn: gyro 0.5 rad/s
    assert s.est.source != hl.SOURCE_COG
    assert 'window reset' in s.est.last_event and 'turning' in s.est.last_event


# ---------------------------------------------------------------------- yaw-rate source
def _q_rpy(r, p, y):
    return _qmul(
        _qmul((0.0, 0.0, math.sin(y / 2), math.cos(y / 2)),
              (0.0, math.sin(p / 2), 0.0, math.cos(p / 2))),
        (math.sin(r / 2), 0.0, 0.0, math.cos(r / 2)))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _matvec(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


# URDF: base_link -> stereo_camera_optical rpy(-1.5547956, 0, -1.5707963),
#       stereo_camera_optical -> stereo_camera_imu rpy(0, pi, 0)
Q_BASE_STEREO_IMU = _qmul(_q_rpy(-1.5547956, 0.0, -1.5707963), _q_rpy(0.0, math.pi, 0.0))


def test_stereo_axis_map_matches_gravity_at_rest():
    row = hl.base_z_row(Q_BASE_STEREO_IMU)
    # measured at rest 2026-10-07 in stereo_camera_imu: specific force (-0.03, -9.79, +0.32)
    az = sum(r * a for r, a in zip(row, (-0.03, -9.79, 0.32)))
    assert az == pytest.approx(9.79, abs=0.1)
    # chip -y is (almost) base up: a CCW base yaw shows as a negative chip-y rate
    assert row[1] == pytest.approx(-1.0, abs=0.01)


def test_stereo_axis_map_round_trip():
    row = hl.base_z_row(Q_BASE_STEREO_IMU)
    m = hl.quat_to_matrix(Q_BASE_STEREO_IMU)
    mt = [[m[j][i] for j in range(3)] for i in range(3)]
    w_imu = _matvec(mt, (0.0, 0.0, 0.3))         # 0.3 rad/s base yaw, seen by the chip
    assert sum(r * w for r, w in zip(row, w_imu)) == pytest.approx(0.3, abs=1e-9)


class SrcSim:
    """WIT at 100 Hz with the JY61P small-rate clamp, stereo at 200 Hz (chip frame)."""

    def __init__(self, mode='auto', stereo_bias=0.0, wit_clamp=0.1, **kw):
        self.row = hl.base_z_row(Q_BASE_STEREO_IMU)
        m = hl.quat_to_matrix(Q_BASE_STEREO_IMU)
        self.mt = [[m[j][i] for j in range(3)] for i in range(3)]
        self.src = hl.YawSource(mode, z_row=self.row, **kw)
        self.t = 0.0
        self.yaw = 0.3             # true base yaw
        self.wit = -1.0            # JY61P yaw (arbitrary zero)
        self.bias = stereo_bias
        self.clamp = wit_clamp
        self.stereo_on = True

    def run(self, w, seconds, rest=False):
        for k in range(int(round(seconds * 200))):
            self.t += 0.005
            self.yaw = hl.wrap(self.yaw + w * 0.005)
            if abs(w) >= self.clamp:
                self.wit = hl.wrap(self.wit + w * 0.005)
            self.src.on_rest(self.t, rest)
            if self.stereo_on:
                self.src.on_stereo(self.t, 1000.0 + self.t,
                                   _matvec(self.mt, (0.0, 0.0, w + self.bias)))
            if k % 2 == 0:
                self.src.on_wit(self.t, self.wit, w if abs(w) >= self.clamp else 0.0)

    def tracked(self):
        """Change of the virtual yaw relative to the true yaw since the start."""
        return self.src.yaw


def test_auto_uses_wit_until_rest_calibrated_then_stereo():
    s = SrcSim('auto')
    s.run(0.0, 0.5)
    assert s.src.select(s.t) == 'wit'
    s.run(0.0, 5.0, rest=True)                  # settle 1 s + one 3 s window
    assert s.src.calibrated()
    assert s.src.select(s.t) == 'stereo'


def test_slow_pivot_invisible_to_wit_is_tracked_by_stereo():
    s = SrcSim('auto')
    s.run(0.0, 5.0, rest=True)
    y0 = s.src.yaw
    s.run(0.05, 20.0)                           # 57 deg at 0.05 rad/s: WIT sees nothing
    assert hl.wrap(s.src.yaw - y0) == pytest.approx(1.0, abs=D(0.5))
    w = SrcSim('wit')
    w.run(0.0, 1.0)
    y0 = w.src.yaw
    w.run(0.05, 20.0)
    assert hl.wrap(w.src.yaw - y0) == pytest.approx(0.0, abs=1e-9)


def test_rest_bias_estimated_and_yaw_held_at_rest():
    s = SrcSim('stereo', stereo_bias=0.004)     # 0.23 deg/s uncorrected
    s.run(0.0, 0.2)
    y0 = s.src.yaw
    s.run(0.0, 60.0, rest=True)
    assert s.src.bias == pytest.approx(0.004, abs=1e-6)
    assert abs(hl.wrap(s.src.yaw - y0)) < D(0.3)
    y1 = s.src.yaw
    s.run(0.0, 60.0)                            # moving gate: no hold, bias removed
    assert abs(hl.wrap(s.src.yaw - y1)) < D(0.1)


def test_bias_window_rejected_when_rotated_by_hand_at_rest():
    s = SrcSim('stereo')
    s.run(0.0, 5.0, rest=True)
    n = s.src.bias_windows
    s.run(0.03, 5.0, rest=True)                 # wheels still, robot turned by hand
    assert s.src.bias_windows == n
    assert s.src.bias == pytest.approx(0.0, abs=1e-6)


def test_wit_reset_does_not_step_virtual_yaw_while_stereo_active():
    s = SrcSim('auto')
    s.run(0.0, 5.0, rest=True)
    y0 = s.src.yaw
    s.wit = hl.wrap(s.wit + D(110.0))           # JY61P yaw jumps (power blip)
    s.run(0.0, 1.0)
    assert abs(hl.wrap(s.src.yaw - y0)) < D(0.1)


def test_stereo_silence_falls_back_to_wit_continuously():
    s = SrcSim('auto', wit_clamp=0.0)
    s.run(0.0, 5.0, rest=True)
    s.run(0.3, 2.0)
    y0, t0 = s.src.yaw, s.yaw
    s.stereo_on = False
    s.run(0.3, 2.0)
    assert s.src.select(s.t) == 'wit'
    # at most max_dt (50 ms) of motion falls between the last stereo and the first WIT step
    assert hl.wrap(s.src.yaw - y0) == pytest.approx(hl.wrap(s.yaw - t0), abs=D(1.0))
    s.stereo_on = True
    s.run(0.3, 2.0)
    assert s.src.select(s.t) == 'stereo'
    assert s.src.switches == 3                   # wit->stereo, stereo->wit, wit->stereo


def test_restore_continues_virtual_yaw_by_wit_delta():
    src = hl.YawSource('wit')
    src.restore(saved_virtual=D(100.0), saved_wit=D(30.0))
    assert src.on_wit(0.0, D(32.0), 0.0) == pytest.approx(D(102.0))
    old = hl.YawSource('auto')                  # pre-YawSource record: imu_yaw was the WIT yaw
    old.restore(saved_virtual=D(30.0), saved_wit=None)
    assert old.on_wit(0.0, D(32.0), 0.0) == pytest.approx(D(32.0))


def test_offset_record_round_trips_wit_yaw(tmp_path):
    p = str(tmp_path / 'h.yaml')
    hl.save_offset(p, 0.5, 'cog', 3, imu_yaw=1.0, x=1.0, y=2.0, wit_yaw=-0.25)
    rec = hl.load_record(p)
    assert rec['wit_yaw'] == pytest.approx(-0.25)
    assert rec['imu_yaw'] == pytest.approx(1.0)
    hl.save_offset(p, 0.5, 'cog', 3, imu_yaw=1.0, x=1.0, y=2.0)
    assert hl.load_record(p)['wit_yaw'] is None


def test_estimator_on_virtual_yaw_end_to_end():
    """Aligner wiring: estimator fed the virtual yaw + active rate keeps its offset through
    a WIT reset while stereo is active (no 'IMU reset, unaligned')."""
    s = SrcSim('auto')
    est = hl.HeadingEstimator()
    est.offset, est.source, est.updated_at = 0.5, hl.SOURCE_COG, 0.0
    s.run(0.0, 5.0, rest=True)
    s.wit = hl.wrap(s.wit + D(90.0))
    for _ in range(100):
        s.run(0.0, 0.01)
        est.on_imu(s.t, s.src.yaw, s.src.rate(s.t))
    assert est.aligned and est.offset == 0.5


def test_stereo_fifo_bursts_are_not_double_counted_with_wit():
    """stereo_imu delivers 4 samples per 20 ms burst; WIT ticks in between must not add."""
    s = SrcSim('auto', wit_clamp=0.0)
    s.run(0.0, 5.0, rest=True)
    src, buf = s.src, []
    y0, true0 = src.yaw, s.yaw
    for k in range(4000):                      # 20 s at 0.3 rad/s, 200 Hz true clock
        s.t += 0.005
        s.yaw = hl.wrap(s.yaw + 0.3 * 0.005)
        s.wit = hl.wrap(s.wit + 0.3 * 0.005)
        buf.append((1000.0 + s.t, _matvec(s.mt, (0.0, 0.0, 0.3))))
        if len(buf) == 4:
            for st, g in buf:
                src.on_stereo(s.t, st, g)
            buf = []
        if k % 2 == 0:
            src.on_wit(s.t, s.wit, 0.3)
    assert src.switches == 1
    assert hl.wrap(src.yaw - y0) == pytest.approx(hl.wrap(s.yaw - true0), abs=D(0.5))


def test_stereo_burst_gaps_do_not_flap_source():
    """2026-10-07: stereo arrives in ~50 ms bursts with occasional longer gaps; the 0.1 s
    window flapped wit<->stereo every 1-8 s. Enter within 0.3 s, leave only after 0.5 s."""
    s = SrcSim('auto', wit_clamp=0.0)
    s.run(0.0, 5.0, rest=True)
    assert s.src.select(s.t) == 'stereo'
    n = s.src.switches
    for gap in (0.12, 0.25, 0.45, 0.2):          # silent stretches shorter than stale_exit
        s.stereo_on = False
        s.run(0.2, gap)
        assert s.src.select(s.t) == 'stereo'
        s.stereo_on = True
        s.run(0.2, 0.05)
    assert s.src.switches == n
    s.stereo_on = False
    s.run(0.2, 0.6)                              # really gone
    assert s.src.select(s.t) == 'wit'
    s.stereo_on = True
    s.run(0.2, 0.05)
    assert s.src.select(s.t) == 'stereo'


def test_stereo_enter_window_is_stale_not_stale_exit():
    src = hl.YawSource('stereo', z_row=(0.0, 0.0, 1.0), stale=0.3, stale_exit=0.5)
    src.st_mono = 0.0
    assert src.select(0.29) == 'stereo'
    src.active = 'wit'
    assert src.select(0.4) == 'wit'              # 0.4 s old: too old to switch TO stereo
    src.active = 'stereo'
    assert src.select(0.4) == 'stereo'           # but not old enough to leave it
    assert src.select(0.51) == 'wit'
