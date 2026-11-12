"""Antenna tilt compensation (2026-11-12): the pure math in gps_tilt, the /tf_static antenna
height and gps_gate moving the fix to the ground point."""

import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402
import rclpy as _rclpy  # noqa: E402
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus  # noqa: E402

from mower_localization import gps_tilt as gt  # noqa: E402
from mower_localization.gps_gate import GpsGateNode  # noqa: E402
from mower_localization.static_tf import StaticRotations  # noqa: E402

REAL_RCLPY = getattr(_rclpy, "__file__", None) is not None
H = 0.324
LAT, LON = 48.0, 9.0   # a synthetic site, not the owner's datum


def _quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _rotate_up(roll, pitch, yaw, h):
    """R(yaw) Ry(pitch) Rx(roll) (0, 0, h): the antenna's world position (reference)."""
    x, y, z = 0.0, 0.0, h
    y, z = y * math.cos(roll) - z * math.sin(roll), y * math.sin(roll) + z * math.cos(roll)
    x, z = x * math.cos(pitch) + z * math.sin(pitch), -x * math.sin(pitch) + z * math.cos(pitch)
    x, y = x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)
    return x, y


# ---- pure math ---------------------------------------------------------------------------

def test_roll_right_side_down_moves_the_antenna_right():
    # REP-103 roll > 0 = right side down: facing east, the antenna moves south (right)
    e, n = gt.tilt_offset_enu(math.radians(10.0), 0.0, 0.0, H)
    assert e == pytest.approx(0.0, abs=1e-12)
    assert n == pytest.approx(-H * math.sin(math.radians(10.0)))
    assert n == pytest.approx(-0.0563, abs=1e-4)   # the ~5 cm slope offset at 10 deg


def test_pitch_nose_down_moves_the_antenna_forward():
    e, n = gt.tilt_offset_enu(0.0, math.radians(5.0), math.pi / 2, H)   # facing north
    assert e == pytest.approx(0.0, abs=1e-12)
    assert n == pytest.approx(H * math.sin(math.radians(5.0)))


@pytest.mark.parametrize("roll,pitch,yaw", [(0.1, 0.0, 0.3), (-0.07, 0.05, 2.0),
                                            (0.2, -0.1, -2.5), (0.0, 0.0, 1.0)])
def test_matches_the_full_rotation(roll, pitch, yaw):
    assert gt.tilt_offset_enu(roll, pitch, yaw, H) == pytest.approx(
        _rotate_up(roll, pitch, yaw, H), abs=1e-12)


def test_sign_matches_the_field_regression():
    """Field 2026-11-12: the unexplained lateral input N (left of travel +) regressed on
    0.324 sin(roll) with coefficient -1.34. The antenna's lateral displacement (left of
    travel) for any heading is -h sin(roll): the same sign."""
    roll = math.radians(2.0)
    for yaw in (math.radians(-75.0), math.radians(105.0)):
        e, n = gt.tilt_offset_enu(roll, 0.0, yaw, H)
        left = -math.sin(yaw) * e + math.cos(yaw) * n
        assert left == pytest.approx(-H * math.sin(roll))
        assert left < 0.0


def test_roll_pitch_yaw_round_trip():
    for rpy in ((0.1, -0.2, 0.3), (-0.3, 0.1, -2.9), (0.0, 0.0, 0.0)):
        assert gt.roll_pitch_yaw(*_quat(*rpy)) == pytest.approx(rpy, abs=1e-9)


def test_shift_latlon_round_trip_in_metres():
    lat, lon = gt.shift_latlon(LAT, LON, 0.05, -0.03)
    m_per_deg_lat = 111_200.0
    assert (lat - LAT) * m_per_deg_lat == pytest.approx(-0.03, rel=0.01)
    assert (lon - LON) * m_per_deg_lat * math.cos(math.radians(LAT)) == pytest.approx(
        0.05, rel=0.01)
    back = gt.shift_latlon(lat, lon, -0.05, 0.03)
    assert back == pytest.approx((LAT, LON), abs=1e-11)


def test_attitude_history_nearest_and_max_age():
    h = gt.AttitudeHistory()
    assert h.at(1.0, 0.1) is None
    for k in range(10):
        h.add(1.0 + 0.01 * k, 0.001 * k, 0.0, 0.0)
    assert h.at(1.042, 0.1)[0] == pytest.approx(0.004)
    assert h.at(1.046, 0.1)[0] == pytest.approx(0.005)
    assert h.at(0.95, 0.1)[0] == pytest.approx(0.0)
    assert h.at(0.85, 0.1) is None           # older than every sample by > max age
    assert h.at(1.25, 0.1) is None           # newer than the last by > max age
    h.add(0.5, 0.2, 0.0, 0.0)                # clock went back: history restarts
    assert h.at(0.5, 0.01)[0] == pytest.approx(0.2)
    assert h.at(1.0, 0.01) is None


def test_antenna_height_sources():
    assert gt.antenna_height(None, 0.0) == pytest.approx(gt.ANTENNA_HEIGHT_M)
    assert gt.antenna_height(0.214, 0.0) == pytest.approx(0.324)
    assert gt.antenna_height(0.214, 0.40) == pytest.approx(0.40)


def test_static_translation_chain():
    t = StaticRotations()
    t.add("base_link", "gps_link", (0.0, 0.0, 0.0, 1.0), (0.098, 0.0, 0.214))
    assert t.translation("base_link", "gps_link") == pytest.approx((0.098, 0.0, 0.214))
    assert t.translation("gps_link", "base_link") == pytest.approx((-0.098, 0.0, -0.214))
    # through a rotated intermediate frame
    q = _quat(0.0, 0.0, math.pi / 2)
    t.add("base_link", "mast", q, (0.1, 0.0, 0.0))
    t.add("mast", "tip", (0.0, 0.0, 0.0, 1.0), (0.2, 0.0, 0.3))
    assert t.translation("base_link", "tip") == pytest.approx((0.1, 0.2, 0.3), abs=1e-12)
    assert t.translation("base_link", "nowhere") is None


# ---- gps_gate ------------------------------------------------------------------------------

class _Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _set_stamp(stamp, t):
    stamp.sec = int(math.floor(t))
    stamp.nanosec = int(round((t - math.floor(t)) * 1e9))


def _fix(t, frame="gps_link"):
    f = NavSatFix()
    f.latitude, f.longitude = LAT, LON
    f.status.status = NavSatStatus.STATUS_GBAS_FIX
    f.position_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.001]
    f.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    f.header.frame_id = frame
    _set_stamp(f.header.stamp, t)
    return f


def _imu(t, roll, pitch, yaw, yaw_var=0.0003):
    m = Imu()
    _set_stamp(m.header.stamp, t)
    o = m.orientation
    o.x, o.y, o.z, o.w = _quat(roll, pitch, yaw)
    cov = [0.0] * 9
    cov[0] = cov[4] = 0.02
    cov[8] = yaw_var
    m.orientation_covariance = cov
    return m


def _tf_static(z):
    tr = types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id="base_link"), child_frame_id="gps_link",
        transform=types.SimpleNamespace(
            translation=types.SimpleNamespace(x=0.098, y=0.0, z=z),
            rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    return types.SimpleNamespace(transforms=[tr])


def _metres(fix):
    """(east, north) of a fix from the synthetic site, m (inverse of shift_latlon)."""
    s = math.sin(math.radians(LAT))
    w = math.sqrt(1.0 - 6.69437999014e-3 * s * s)
    m = 6378137.0 * (1.0 - 6.69437999014e-3) / w ** 3
    n = 6378137.0 / w
    return (math.radians(fix.longitude - LON) * n * math.cos(math.radians(LAT)),
            math.radians(fix.latitude - LAT) * m)


class TestGateTilt:
    @pytest.fixture(autouse=True)
    def _cleanup(self):
        self._nodes = []
        yield
        for node in self._nodes:
            node.destroy_node()

    def _node(self, **overrides):
        overrides.setdefault("quality_covariance", False)
        if REAL_RCLPY:
            from rclpy.parameter import Parameter
            parameters = [Parameter(n, value=v) for n, v in overrides.items()]
        else:
            parameters = [types.SimpleNamespace(name=n, value=v) for n, v in overrides.items()]
        node = GpsGateNode(parameter_overrides=parameters)
        self._nodes.append(node)
        node.fix_pub = _Recorder()
        return node

    def test_side_slope_fix_moves_uphill_to_the_ground_point(self):
        node = self._node()
        roll = math.radians(10.0)          # facing east, right (south) side down
        for k in range(20):
            node._on_imu(_imu(99.9 + 0.01 * k, roll, 0.0, 0.0))
        node._on_fix(_fix(100.0))
        e, n = _metres(node.fix_pub.msgs[0])
        assert e == pytest.approx(0.0, abs=1e-6)
        assert n == pytest.approx(H * math.sin(roll), abs=1e-5)   # +5.6 cm: north, uphill
        assert node.tilt_applied == 1 and node.tilt_skipped == 0

    def test_uses_the_sample_at_the_fix_epoch(self):
        node = self._node(tilt_fix_latency_s=0.1)
        node._on_imu(_imu(99.90, math.radians(4.0), 0.0, 0.0))
        node._on_imu(_imu(100.00, math.radians(-4.0), 0.0, 0.0))
        node._on_fix(_fix(100.0))
        _, n = _metres(node.fix_pub.msgs[0])
        assert n == pytest.approx(H * math.sin(math.radians(4.0)), abs=1e-5)

    def test_stale_or_unaligned_attitude_passes_the_fix_unmoved(self):
        node = self._node()
        node._on_imu(_imu(99.0, 0.2, 0.0, 0.0))                   # 0.9 s before the epoch
        node._on_imu(_imu(99.9, 0.2, 0.0, 0.0, yaw_var=100.0))    # heading not aligned
        node._on_fix(_fix(100.0))
        assert _metres(node.fix_pub.msgs[0]) == pytest.approx((0.0, 0.0), abs=1e-9)
        assert node.tilt_skipped == 1 and node.passed == 1

    def test_disabled_leaves_the_fix_alone(self):
        node = self._node(tilt_compensation=False)
        node._on_imu(_imu(99.9, 0.2, 0.0, 0.0))
        node._on_fix(_fix(100.0))
        assert _metres(node.fix_pub.msgs[0]) == pytest.approx((0.0, 0.0), abs=1e-9)

    def test_height_from_tf_static_gain_and_override(self):
        node = self._node(tilt_gain=1.3)
        assert node.antenna_height("gps_link") == pytest.approx(gt.ANTENNA_HEIGHT_M)
        node._static_tf.add_msg(_tf_static(0.25))
        assert node.antenna_height("gps_link") == pytest.approx(0.25 + 0.11)
        node._on_imu(_imu(99.9, math.radians(5.0), 0.0, math.pi / 2))   # facing north
        node._on_fix(_fix(100.0))
        e, _ = _metres(node.fix_pub.msgs[0])
        # right side down facing north: antenna east; ground point west of it
        assert e == pytest.approx(-1.3 * 0.36 * math.sin(math.radians(5.0)), abs=1e-5)
        fixed = self._node(tilt_antenna_height=0.40)
        fixed._static_tf.add_msg(_tf_static(0.25))
        assert fixed.antenna_height("gps_link") == pytest.approx(0.40)
