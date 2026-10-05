import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from wit_imu_driver import wit_protocol as wp  # noqa: E402


def test_frame_roundtrip_and_checksum():
    f = wp.build_frame(wp.TYPE_ACC, (0, 0, 2048, 2550))
    assert len(f) == 11 and f[0] == 0x55 and f[1] == 0x51
    assert wp.checksum(f) == f[10]


def test_acc_scaling_and_temperature():
    p = wp.WitParser()
    assert p.feed(wp.build_frame(wp.TYPE_ACC, (0, 0, 2048, 2550))) == [wp.TYPE_ACC]
    ax, ay, az = p.sample.acc
    assert abs(az - 9.80665) < 1e-6   # 2048/32768*16 g == 1 g
    assert p.sample.temperature_c == 25.5


def test_gyro_and_angle_scaling():
    p = wp.WitParser()
    p.feed(wp.build_frame(wp.TYPE_GYRO, (16384, 0, 0, 0)))
    assert abs(p.sample.gyro[0] - math.radians(1000.0)) < 1e-9
    p.feed(wp.build_frame(wp.TYPE_ANGLE, (0, 0, 16384, 0)))
    assert abs(p.sample.rpy[2] - math.pi / 2) < 1e-9
    assert not p.sample.complete
    p.feed(wp.build_frame(wp.TYPE_ACC, (0, 0, 2048, 0)))
    assert p.sample.complete


def test_resync_after_garbage_and_split_frames():
    p = wp.WitParser()
    frame = wp.build_frame(wp.TYPE_ANGLE, (100, -200, 300, 0))
    stream = b'\x01\x55\x99' + frame[:4]
    assert p.feed(stream) == []
    assert p.feed(frame[4:]) == [wp.TYPE_ANGLE]
    assert p.sample.rpy[0] > 0 and p.sample.rpy[1] < 0


def test_bad_checksum_is_dropped_and_counted():
    p = wp.WitParser()
    frame = bytearray(wp.build_frame(wp.TYPE_GYRO, (1, 2, 3, 4)))
    frame[10] ^= 0xFF
    assert p.feed(bytes(frame)) == []
    assert p.bad_checksums == 1
    assert p.sample.gyro is None


def test_quaternion_order_is_xyzw():
    p = wp.WitParser()
    p.feed(wp.build_frame(wp.TYPE_QUAT, (32767, 0, 0, 0)))  # q0 = w ~ 1
    x, y, z, w = p.sample.quat
    assert w > 0.99 and x == y == z == 0.0


def test_zero_axis_does_not_block_completion():
    # Regression: the old driver used all(dict.values()) which never fired while
    # any axis read exactly zero (e.g. a level, stationary robot).
    p = wp.WitParser()
    for t, v in ((wp.TYPE_ACC, (0, 0, 2048, 0)), (wp.TYPE_GYRO, (0, 0, 0, 0)),
                 (wp.TYPE_ANGLE, (0, 0, 0, 0))):
        p.feed(wp.build_frame(t, v))
    assert p.sample.complete


def test_config_commands():
    assert wp.cmd_set_rate(100) == bytes([0xFF, 0xAA, 0x03, 0x09, 0x00])
    assert wp.cmd_set_output() == bytes([0xFF, 0xAA, 0x02, 0x0E, 0x00])
