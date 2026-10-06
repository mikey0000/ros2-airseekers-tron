#!/usr/bin/env python3
"""Offline tests for the MCU framing/struct code in ``mower_mcu_driver.mcu_node``.

No ROS installation is required: ``ros_stubs`` installs minimal ``rclpy``/message
placeholders first so that the module can be imported for its codec half.

    python3 test/test_protocol.py          # plain
    python3 -m pytest ros2_stack/src/mower_mcu_driver/test   # with pytest, if available

The headline test replays a real 2.8 MB MCU capture (``mcu_traffic_logs/``) through both this
parser and the reference decoder ``ros2_port_handoff/01_mcu_protocol/mcu_frame_decode.py`` and
requires an identical sub-packet histogram - i.e. byte-for-byte behavioural parity with the
implementation that was validated on 172,850 live frames.
"""

import collections
import importlib.util
import math
import os
import struct
import sys
import unittest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.abspath(os.path.join(PKG_DIR, '..', '..', '..'))
CAPTURE = os.path.join(REPO_DIR, 'ros2_port_handoff', '01_mcu_protocol', 'mcu_traffic_logs',
                       'mcu.dat.current_2026-10-05')
REFERENCE = os.path.join(REPO_DIR, 'ros2_port_handoff', '01_mcu_protocol', 'mcu_frame_decode.py')

if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

import ros_stubs  # noqa: E402  (same directory as this test)

ros_stubs.install()

from mower_mcu_driver import mcu_node as mn  # noqa: E402


def _load_reference():
    spec = importlib.util.spec_from_file_location('mcu_frame_decode_reference', REFERENCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestFraming(unittest.TestCase):

    def test_round_trip_single_subpacket(self):
        payload = bytes(range(64))
        frame = mn.build_frame([(mn.TYPE_ROS_MOWER, mn.MOD_SPEED, payload)])
        self.assertEqual(frame[0], mn.SOF)
        self.assertEqual(frame[-1], mn.EOF)
        parser = mn.FrameParser()
        got = parser.feed(frame)
        self.assertEqual(got, [(mn.TYPE_ROS_MOWER, mn.MOD_SPEED, payload)])
        self.assertEqual(parser.frames_ok, 1)
        self.assertEqual(parser.frames_bad, 0)

    def test_round_trip_needs_escaping(self):
        # Payload stuffed with SOF/EOF/ESC must survive the escape layer.
        payload = b'\xA5\x5A\xF5\xA5\x5A\xF5'
        frame = mn.build_frame([(mn.TYPE_MOWER_ROS, mn.MOD_LOG, payload)])
        self.assertNotIn(b'\xA5', frame[1:-1])  # nothing raw left inside the frame
        got = mn.FrameParser().feed(frame)
        self.assertEqual(got, [(mn.TYPE_MOWER_ROS, mn.MOD_LOG, payload)])

    def test_multiple_subpackets_in_one_frame(self):
        subs = [(mn.TYPE_MOWER_ROS, mn.MOD_BATTERY, b'\x01' * 11),
                (mn.TYPE_MOWER_ROS, mn.MOD_VERSION, b'\x02' * 9),
                (mn.TYPE_MOWER_ROS, mn.MOD_MOTORS, b'\x03' * 32)]
        self.assertEqual(mn.FrameParser().feed(mn.build_frame(subs)), subs)

    def test_chunked_stream_reassembly(self):
        subs = [(mn.TYPE_MOWER_ROS, mn.MOD_SENSOR, os.urandom(10)),
                (mn.TYPE_MOWER_ROS, mn.MOD_SPEED, struct.pack('<ff', -1.25, 0.5))]
        stream = mn.build_frame(subs) + mn.build_frame(subs)
        parser = mn.FrameParser()
        got = []
        for i in range(0, len(stream), 3):  # byte dribble, worst case for a stream parser
            got.extend(parser.feed(stream[i:i + 3]))
        self.assertEqual(got, subs + subs)

    def test_bad_checksum_is_rejected_and_parser_resyncs(self):
        good = mn.build_frame([(mn.TYPE_ROS_MOWER, mn.MOD_SPEED, struct.pack('<ff', 1.0, 0.0))])
        broken = bytearray(good)
        broken[-2] ^= 0xFF  # corrupt the (escaped) checksum byte
        parser = mn.FrameParser()
        self.assertEqual(parser.feed(bytes(broken) + good), [
            (mn.TYPE_ROS_MOWER, mn.MOD_SPEED, struct.pack('<ff', 1.0, 0.0))])
        self.assertGreaterEqual(parser.frames_bad, 1)

    def test_leading_garbage_is_skipped(self):
        frame = mn.build_frame([(mn.TYPE_HEARTBEAT, mn.MOD_ALL, mn.heartbeat_payload())])
        got = mn.FrameParser().feed(b'\x00\x13garbage\xA5\x00' + frame)
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0][0], mn.TYPE_HEARTBEAT)

    def test_heartbeat_payload_layout(self):
        from datetime import datetime
        payload = mn.heartbeat_payload(datetime(2026, 10, 5, 12, 34, 56, 789000))
        self.assertEqual(struct.unpack(mn.HEARTBEAT_FMT, payload),
                         (26, 10, 5, 12, 34, 56, 0x03, 0x15))  # ms = 789 = 0x0315

    def test_speed_payload_layout(self):
        self.assertEqual(mn.speed_payload(1.5, -0.25),
                         struct.pack('<ff', 1.5, -0.25))

    def test_struct_sizes_match_recovered_header(self):
        # Sizes must equal the payload lengths observed on the wire.
        for fmt, expected in ((mn.SPEED_FMT, 8), (mn.BATTERY_FMT, 11), (mn.SENSOR_FMT, 10),
                              (mn.IMU_FMT, 18), (mn.VERSION_FMT, 9), (mn.BMS_FMT, 3),
                              (mn.CALIB_FMT, 3), (mn.HEARTBEAT_FMT, 8), (mn.MOTOR_FMT, 8)):
            self.assertEqual(struct.calcsize(fmt), expected, fmt)

    def test_wit_scaling(self):
        # 32768 raw counts == 180 deg == pi rad (angle), 16 g (acc), 2000 dps (gyro).
        self.assertAlmostEqual(mn.WIT_RAW_ANGLE_TO_RAD * 32768.0, math.pi)
        self.assertAlmostEqual(mn.WIT_RAW_ACC_TO_MS2 * 32768.0, 16.0 * 9.80665)
        self.assertAlmostEqual(mn.WIT_RAW_GYRO_TO_RAD * 32768.0, math.radians(2000.0))


class TestAgainstReferenceDecoder(unittest.TestCase):
    """Bit-exact parity with the reference implementation on a real MCU capture."""

    def setUp(self):
        if not os.path.exists(CAPTURE):
            self.skipTest('capture not present: %s' % CAPTURE)
        if not os.path.exists(REFERENCE):
            self.skipTest('reference decoder not present: %s' % REFERENCE)

    def test_subpacket_histogram_matches_reference(self):
        reference = _load_reference()
        with open(CAPTURE, 'rb') as handle:
            data = handle.read()

        ref_hist = collections.Counter()
        ref_bad = 0
        for _offset, inner, ok in reference.split_frames(data):
            ref_bad += 0 if ok else 1
            for type_id, mod_id, payload in reference.split_subpackets(inner):
                ref_hist[(type_id, mod_id, len(payload))] += 1

        parser = mn.FrameParser()
        our_hist = collections.Counter()
        chunk = 4096  # same granularity as a serial read()
        for i in range(0, len(data), chunk):
            for type_id, mod_id, payload in parser.feed(data[i:i + chunk]):
                our_hist[(type_id, mod_id, len(payload))] += 1

        self.assertTrue(ref_hist, 'reference decoded nothing')
        self.assertEqual(our_hist, ref_hist)
        self.assertEqual(parser.frames_bad, ref_bad)


if __name__ == '__main__':
    unittest.main(verbosity=2)


# --------------------------------------------------------------- MotorInfo / BatteryInfo
# Raw MODULE_MOTORS / MODULE_BATTERY payloads captured on the mower at rest, 2026-10-06.
CAPTURED_MOTORS = (
    '00000000b1071e000000fe0dfe312801000097fdfe312201ef4c60ea00000000',
    '00000000b2071e000000f10df231280100006afdf2312201310060ea00000000',
)
CAPTURED_BATTERY = 'c50007001f002000000000'


def test_decode_captured_motors_si():
    m = mn.decode_motors(bytes.fromhex(CAPTURED_MOTORS[0]))
    cut, left, right = m['cutter'], m['left'], m['right']
    assert cut['raw'] == (0, 0, 1969, 30, 0)
    assert abs(cut['voltage_v'] - 19.69) < 1e-6 and cut['current_a'] == 0.0
    assert cut['temperature_c'] == 30 and cut['status'] == 0
    assert left['raw'] == (0, 3582, 12798, 40, 1)
    assert right['raw'] == (0, -617, 12798, 34, 1)
    for drive in (left, right):
        # drive bus voltage tracks the 19.7 V battery (and the 19.69 V cutter reading)
        assert 19.0 < drive['voltage_v'] < 20.2
        assert abs(drive['current_a']) < 5.0
        assert mn.MOTOR_STATUS_TEXT[drive['status']] == 'running'
    assert abs(left['current_a'] - 3.582) < 1e-9 and abs(right['current_a'] + 0.617) < 1e-9


def test_decode_captured_motors_second_frame_and_msg_units():
    m = mn.decode_motors(bytes.fromhex(CAPTURED_MOTORS[1]))
    f = mn.motor_msg_fields(m['left'])
    # MowerSensorInfo.MotorInfo units: 10 mA, 10 mV
    assert f['current'] == 357 and 1940 < f['voltage'] < 1960 and f['status'] == 1
    assert mn.motor_msg_fields(m['cutter'])['voltage'] == 1970


def test_decode_motors_rejects_bad_length():
    assert mn.decode_motors(b'\x00' * 31) is None


def test_motor_status_is_signed():
    payload = struct.pack(mn.MOTOR_FMT, 0, 0, 0, 0, -2) * 4
    assert mn.MOTOR_STATUS_TEXT[mn.decode_motors(payload)['cutter']['status']] == 'over-current'


def test_captured_battery_layout():
    v, i, pct, dock, temp, err = struct.unpack(mn.BATTERY_FMT, bytes.fromhex(CAPTURED_BATTERY))
    assert (v, i, pct, dock, temp, err) == (197, 7, 31, 0, 32, 0)
