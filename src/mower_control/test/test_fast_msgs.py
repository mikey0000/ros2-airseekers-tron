"""Fast CDR helpers equal real rclpy serialization (byte for byte)."""

import struct

import pytest

from conftest import HAS_RCLPY
from mower_control import fast_msgs as fm

real = pytest.mark.skipif(not HAS_RCLPY, reason='real rclpy serialization required')


def test_twist_layout_host():
    data = fm.twist_bytes(0.25, 1.0, -2.0, 0.5, -0.125, 3.0)
    assert len(data) == 52
    assert data[:4] == b'\x00\x01\x00\x00'
    assert struct.unpack_from('<6d', data, 4) == (0.25, 1.0, -2.0, 0.5, -0.125, 3.0)
    t = fm.parse_twist(data)
    assert (t.linear.x, t.linear.y, t.linear.z) == (0.25, 1.0, -2.0)
    assert (t.angular.x, t.angular.y, t.angular.z) == (0.5, -0.125, 3.0)


def test_string_and_bool_roundtrip_host():
    assert fm.parse_string(fm.string_bytes('quality=RTK_FIXED')).data == 'quality=RTK_FIXED'
    assert fm.parse_string(fm.string_bytes('')).data == ''
    assert fm.bool_bytes(True)[4] == 1 and fm.bool_bytes(False)[4] == 0
    assert fm.parse_twist(b'\x01\x00\x00\x00' + b'\0' * 48) is None  # big endian: fallback
    assert fm.parse_twist(b'\x00\x01\x00\x00' + b'\0' * 8) is None   # truncated


@real
def test_twist_bytes_equal_rclpy():
    from geometry_msgs.msg import Twist
    from rclpy.serialization import deserialize_message, serialize_message
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.linear.z = 0.3, -0.0, 1e-9
    msg.angular.x, msg.angular.y, msg.angular.z = -1.5, 2.25, 0.123456789
    data = fm.twist_bytes(0.3, -0.0, 1e-9, -1.5, 2.25, 0.123456789)
    assert data == serialize_message(msg)
    assert deserialize_message(data, Twist) == msg
    parsed = fm.parse_twist(serialize_message(msg))
    assert (parsed.linear.x, parsed.angular.z) == (0.3, 0.123456789)


@real
@pytest.mark.parametrize('frame', ['', 'b', 'base_link', 'odom123'])
def test_twist_stamped_parse_equals_rclpy(frame):
    from geometry_msgs.msg import TwistStamped
    from rclpy.serialization import serialize_message
    msg = TwistStamped()
    msg.header.stamp.sec, msg.header.stamp.nanosec = 1791245285, 98989902
    msg.header.frame_id = frame
    msg.twist.linear.x, msg.twist.angular.z = 0.42, -0.7
    msg.twist.linear.y = 0.01
    p = fm.parse_twist_stamped(serialize_message(msg))
    assert p.header.frame_id == frame
    assert (p.header.stamp.sec, p.header.stamp.nanosec) == (1791245285, 98989902)
    assert (p.twist.linear.x, p.twist.linear.y, p.twist.angular.z) == (0.42, 0.01, -0.7)


@real
@pytest.mark.parametrize('text', ['', 'a', 'abc', 'abcd', 'quality=RTK_FIXED sats=31'])
def test_string_bytes_equal_rclpy(text):
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    assert fm.string_bytes(text) == serialize_message(String(data=text))
    assert fm.parse_string(serialize_message(String(data=text))).data == text


@real
def test_bool_bytes_equal_rclpy():
    from rclpy.serialization import deserialize_message, serialize_message
    from std_msgs.msg import Bool
    for value in (False, True):
        ref = serialize_message(Bool(data=value))
        fast = fm.bool_bytes(value)
        assert len(fast) == len(ref) and fast[:5] == ref[:5]   # ref padding is uninitialised
        assert deserialize_message(fast, Bool).data is value
