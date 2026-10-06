"""The pre-serialized Imu/Float32 bytes equal rclpy's own serialization of the typed message.

The byte-for-byte comparison needs real rclpy + sensor_msgs (skipped elsewhere); the layout
checks run everywhere.
"""
import math
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from wit_imu_driver import imu_cdr  # noqa: E402
from wit_imu_driver import wit_protocol as wp  # noqa: E402


def _diag3(a, b, c):
    return [a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c]


OCOV, GCOV, ACOV = _diag3(0.02, 0.02, 1.0), _diag3(0.005, 0.005, 0.005), _diag3(.05, .05, .05)
SAMPLES = [
    (1_700_000_123_456_789_012, (0.1, -0.2, 0.3, 0.9), (0.01, -0.02, 1.5), (0.1, 0.2, 9.81)),
    (0, (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    (5_000_000_001, wp.euler_to_quaternion(0.1, -0.05, math.pi), (-3.0, 2.0, 1.0),
     (-156.9, 0.0, 1e-9)),
]


@pytest.mark.parametrize('frame_id', ['imu_link', 'imu', '', 'a_much_longer_imu_frame'])
def test_layout_and_alignment(frame_id):
    ser = imu_cdr.ImuSerializer(frame_id, OCOV, GCOV, ACOV)
    stamp, q, g, a = SAMPLES[0]
    data = ser.serialize(stamp, q, g, a)
    assert data[:4] == b'\x00\x01\x00\x00'
    sec, nsec = struct.unpack_from('<iI', data, 4)
    assert sec * 1_000_000_000 + nsec == stamp
    (n,) = struct.unpack_from('<I', data, 12)
    assert data[16:16 + n] == frame_id.encode() + b'\0'
    tail = len(data) - imu_cdr.IMU_TAIL.size
    assert (tail - 4) % 8 == 0                      # doubles 8-aligned in the payload
    v = imu_cdr.IMU_TAIL.unpack_from(data, tail)
    assert v[0:4] == q and tuple(v[4:13]) == tuple(OCOV)
    assert v[13:16] == g and v[25:28] == a


def test_float32():
    assert imu_cdr.float32_bytes(25.5) == b'\x00\x01\x00\x00' + struct.pack('<f', 25.5)


def test_matches_rclpy_serialization():
    serialization = pytest.importorskip('rclpy.serialization')
    msgs = pytest.importorskip('sensor_msgs.msg')
    std = pytest.importorskip('std_msgs.msg')
    for frame_id in ('imu_link', 'imu', ''):
        ser = imu_cdr.ImuSerializer(frame_id, OCOV, GCOV, ACOV)
        for stamp, q, g, a in SAMPLES:
            m = msgs.Imu()
            m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            m.header.frame_id = frame_id
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
            m.orientation_covariance = OCOV
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = g
            m.angular_velocity_covariance = GCOV
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = a
            m.linear_acceleration_covariance = ACOV
            fast = ser.serialize(stamp, q, g, a)
            ref = serialization.serialize_message(m)
            # Fast-CDR leaves alignment padding uninitialized: compare all other bytes
            assert len(fast) == len(ref)
            assert _mask_pad(fast, frame_id) == _mask_pad(ref, frame_id)
            back = serialization.deserialize_message(fast, msgs.Imu)
            assert back == m
    for t in (25.5, -3.21, 0.0):
        assert imu_cdr.float32_bytes(t) == serialization.serialize_message(std.Float32(data=t))


def _mask_pad(data, frame_id):
    """Zero the padding between the frame id string and the 8-aligned body."""
    end = 16 + len(frame_id) + 1
    body = len(data) - imu_cdr.IMU_TAIL.size
    return data[:end] + b'\0' * (body - end) + data[body:]


def test_node_fast_publish_equals_typed():
    """WitImuNode: fed frames -> fast bytes == typed message (stamps aside)."""
    rclpy = pytest.importorskip('rclpy')
    serialization = pytest.importorskip('rclpy.serialization')
    msgs = pytest.importorskip('sensor_msgs.msg')
    pytest.importorskip('serial')
    pytest.importorskip('tf2_ros')
    import types
    from wit_imu_driver import wit_node

    rclpy.init(args=['--ros-args', '-p', 'port:=/dev/does_not_exist_imu'])
    node = wit_node.WitImuNode()
    try:
        out, temps = [], []
        node.imu_pub = types.SimpleNamespace(publish=out.append)
        node.temp_pub = types.SimpleNamespace(publish=temps.append)
        frames = (wp.build_frame(wp.TYPE_ACC, (100, -200, 2048, 2550)) +
                  wp.build_frame(wp.TYPE_GYRO, (16, -32, 64, 0)) +
                  wp.build_frame(wp.TYPE_ANGLE, (1000, -2000, 3000, 0)))
        node._feed(frames[:7])
        node._feed(frames[7:])
        node.fast_publish = False
        node._feed(frames)
        assert len(out) == 2 and isinstance(out[0], bytes)
        fast = serialization.deserialize_message(out[0], msgs.Imu)
        typed = out[1]
        assert fast.header.stamp.sec != 0
        for m in (fast, typed):
            m.header.stamp.sec = m.header.stamp.nanosec = 0
        assert fast == typed
        assert serialization.serialize_message(fast) == serialization.serialize_message(typed)
        assert isinstance(temps[0], bytes) and abs(
            struct.unpack_from('<f', temps[0], 4)[0] - temps[1].data) < 1e-6
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
