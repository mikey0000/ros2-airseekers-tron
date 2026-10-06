# SPDX-License-Identifier: GPL-3.0-or-later
"""Fast CDR helpers for the small flat messages the mower_control nodes use.

Same idea as the parsers/serializers in ``sub_pump.py`` (kept identical across packages, so
these extra ones live here): read the few fields of a serialized message straight from the
XCDR1 little-endian bytes, and publish pre-serialized bytes (``Publisher.publish(bytes)``)
instead of building a message object. Byte layouts are checked against real rclpy
serialization in ``test/test_fast_msgs.py``.

Parsers return duck-typed objects with the ROS attribute names, or None for anything
unexpected (the pump then falls back to ``deserialize_message``).
"""

import struct
import types

from mower_control.sub_pump import _align, _header, _le_cdr, _string

_NS = types.SimpleNamespace
_ENCAPSULATION = b'\x00\x01\x00\x00'   # CDR little endian, no options
_SIX_DOUBLES = struct.Struct('<6d')


def twist_bytes(lx, ly, lz, ax, ay, az):
    """Serialized geometry_msgs/Twist: encapsulation + linear xyz + angular xyz (52 bytes)."""
    return _ENCAPSULATION + _SIX_DOUBLES.pack(lx, ly, lz, ax, ay, az)


def _twist_ns(v):
    return _NS(linear=_NS(x=v[0], y=v[1], z=v[2]), angular=_NS(x=v[3], y=v[4], z=v[5]))


def parse_twist(data):
    """geometry_msgs/Twist from CDR bytes."""
    if not _le_cdr(data):
        return None
    try:
        return _twist_ns(_SIX_DOUBLES.unpack_from(data, 4))
    except struct.error:
        return None


def parse_twist_stamped(data):
    """geometry_msgs/TwistStamped from CDR bytes."""
    if not _le_cdr(data):
        return None
    try:
        header, pos = _header(data)
        v = _SIX_DOUBLES.unpack_from(data, _align(pos, 8))
    except (ValueError, struct.error, UnicodeDecodeError):
        return None
    return _NS(header=header, twist=_twist_ns(v))


def parse_string(data):
    """std_msgs/String from CDR bytes."""
    if not _le_cdr(data):
        return None
    try:
        text, _pos = _string(data, 4)
    except (ValueError, struct.error, UnicodeDecodeError):
        return None
    return _NS(data=text)


def bool_bytes(value):
    """Serialized std_msgs/Bool (1 byte, end padded to 4 like Fast-CDR)."""
    return _ENCAPSULATION + (b'\x01' if value else b'\x00') + b'\x00\x00\x00'


def string_bytes(text):
    """Serialized std_msgs/String."""
    raw = text.encode('utf-8') + b'\0'
    out = _ENCAPSULATION + struct.pack('<I', len(raw)) + raw
    return out + b'\0' * (-len(out) % 4)
