"""Pre-serialized (CDR) sensor_msgs/Imu and std_msgs/Float32, ROS-free.

``Publisher.publish(bytes)`` (rclpy Humble ``publish_raw``) skips building the Python
message, the Python->C conversion and rosidl serialization (~0.4 ms per publish on the
RK3588). The layout is plain XCDR1 little endian, what rmw_fastrtps/cyclonedds emit; it is
checked byte-for-byte against ``rclpy.serialization.serialize_message`` in
``test/test_imu_cdr.py`` (skipped where rclpy is missing).
"""
import struct

_ENCAPSULATION = b'\x00\x01\x00\x00'
_STAMP = struct.Struct('<iI')
# orientation xyzw, covariance[9], angular_velocity xyz, covariance[9],
# linear_acceleration xyz, covariance[9]
IMU_TAIL = struct.Struct('<4d9d3d9d3d9d')
_FLOAT32 = struct.Struct('<f')


def _align(pos, n):
    # CDR alignment is relative to the payload, which starts after the 4-byte encapsulation.
    return 4 + ((pos - 4 + n - 1) & -n)


def _frame_id_bytes(frame_id):
    """CDR string ``frame_id`` placed right after the stamp (payload offset 12) plus the
    padding up to the first 8-byte field of the message body."""
    raw = frame_id.encode('utf-8') + b'\0'
    pos = 12 + 4 + len(raw)
    pad = _align(pos, 8) - pos
    return struct.pack('<I', len(raw)) + raw + b'\0' * pad


class ImuSerializer:
    """``serialize(stamp_ns, orientation, gyro, acc) -> bytes`` for a fixed frame id and
    fixed covariances (9-element sequences)."""

    def __init__(self, frame_id, orientation_cov, angular_velocity_cov,
                 linear_acceleration_cov):
        self._frame = _frame_id_bytes(frame_id)
        self._ocov = tuple(float(v) for v in orientation_cov)
        self._gcov = tuple(float(v) for v in angular_velocity_cov)
        self._acov = tuple(float(v) for v in linear_acceleration_cov)
        if not (len(self._ocov) == len(self._gcov) == len(self._acov) == 9):
            raise ValueError('covariances must have 9 elements')

    def serialize(self, stamp_ns, orientation, gyro, acc):
        sec, nanosec = divmod(int(stamp_ns), 1_000_000_000)
        return b''.join((_ENCAPSULATION, _STAMP.pack(sec, nanosec), self._frame,
                         IMU_TAIL.pack(*orientation, *self._ocov, *gyro, *self._gcov,
                                       *acc, *self._acov)))


def float32_bytes(value):
    """Serialized std_msgs/Float32."""
    return _ENCAPSULATION + _FLOAT32.pack(value)
