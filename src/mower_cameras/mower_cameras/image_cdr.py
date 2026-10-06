"""Pre-serialized sensor_msgs/Image (XCDR1 little endian), ROS-free.

The converted pixels are written straight into the serialized message buffer (``image``
is a numpy view of its data region, usable as ``cv2.cvtColor(..., dst=image)``), so
publishing a frame is: convert in place, patch the stamp, ``Publisher.publish(bytes)``.
That skips the Python ``Image`` object, the ``array.array`` copy, the Python->C message
conversion and rosidl serialization. Checked byte-for-byte against
``rclpy.serialization.serialize_message`` in ``test/test_image_cdr.py``.
"""
import struct

_ENCAPSULATION = b'\x00\x01\x00\x00'
_STAMP = struct.Struct('<iI')


def _pad4(out):
    # CDR alignment is relative to the payload, which starts after the 4-byte encapsulation
    # (itself 4 bytes long, so 4-alignment of the payload == 4-alignment of the buffer).
    out.extend(b'\0' * (-len(out) % 4))


def _string(out, text):
    _pad4(out)
    raw = text.encode('utf-8') + b'\0'
    out.extend(struct.pack('<I', len(raw)))
    out.extend(raw)


class ImageCdr:
    """Reusable serialized Image of fixed ``frame_id``/size/encoding."""

    def __init__(self, frame_id, height, width, encoding='bgr8', channels=3):
        import numpy as np
        self.height, self.width, self.channels = int(height), int(width), int(channels)
        step = self.width * self.channels
        size = step * self.height
        out = bytearray(_ENCAPSULATION)
        out.extend(b'\0' * 8)                       # stamp, patched per frame
        _string(out, frame_id)
        _pad4(out)
        out.extend(struct.pack('<II', self.height, self.width))
        _string(out, encoding)
        out.append(0)                               # is_bigendian
        _pad4(out)
        out.extend(struct.pack('<II', step, size))  # step, data length
        self.data_offset = len(out)
        out.extend(b'\0' * size)               # no trailing pad (same as rmw_fastrtps)
        self.buf = out
        shape = (self.height, self.width, self.channels) if self.channels > 1 \
            else (self.height, self.width)
        self.image = np.frombuffer(out, dtype=np.uint8, count=size,
                                   offset=self.data_offset).reshape(shape)

    def serialize(self, stamp_ns):
        sec, nanosec = divmod(int(stamp_ns), 1_000_000_000)
        _STAMP.pack_into(self.buf, 4, sec, nanosec)
        return bytes(self.buf)
