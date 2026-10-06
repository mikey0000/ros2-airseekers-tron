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
    s.run(0.3, 0.2, 5.0)        # turning: window discarded
    assert not s.est.aligned
    s.run(0.1, 0.0, 10.0)       # too slow
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
    s = Sim(true_offset_deg=70.0)
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
    s = Sim(true_offset_deg=70.0)
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
