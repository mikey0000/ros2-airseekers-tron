"""Unit tests for the BLE framing, validated against the vendor's worked examples
(mower_docs/reference/proto/ble.md) and the surviving protocol_read/write.cc."""

import unittest

from base_ble.ble_protocol import (
    PROTOCOL_HEAD,
    PROTOCOL_END,
    CmdId,
    build_frame,
    checksum8,
    escape,
    extract_frames,
    parse_frame,
    unescape,
)


class TestEscaping(unittest.TestCase):
    def test_escape_round_trip(self):
        data = bytes([0xA5, 0x5A, 0xF5, 0x00, 0x01, 0xFF])
        self.assertEqual(unescape(escape(data)), data)

    def test_escape_mapping(self):
        self.assertEqual(escape(bytes([0xA5])), bytes([0xF5, 0x01]))
        self.assertEqual(escape(bytes([0x5A])), bytes([0xF5, 0x02]))
        self.assertEqual(escape(bytes([0xF5])), bytes([0xF5, 0x03]))

    def test_unescape_lone_f5(self):
        # checksum byte is never escaped; a lone F5 not followed by a code passes through
        self.assertEqual(unescape(bytes([0xF5, 0x5A])), bytes([0xF5, 0x5A]))
        self.assertEqual(unescape(bytes([0xF5, 0x01])), bytes([0xA5]))


class TestParseDocumentedExamples(unittest.TestCase):
    def test_teleop_start(self):
        # a5 96 02 08 01 a1 5a  -> teleop mode START (TeleopMode{type=START})
        frame = bytes.fromhex('a5 96 02 08 01 a1 5a')
        fields = parse_frame(frame)
        self.assertEqual(fields, [(CmdId.TELEOP, bytes([0x08, 0x01]))])

    def test_teleop_blade_on(self):
        # a5 96 02 10 01 a9 5a  -> blade on (cutter=START)
        fields = parse_frame(bytes.fromhex('a5 96 02 10 01 a9 5a'))
        self.assertEqual(fields, [(CmdId.TELEOP, bytes([0x10, 0x01]))])

    def test_drive(self):
        # a5 84 01 14 85 01 14 33 5a -> linear 20, angular 20
        fields = parse_frame(bytes.fromhex('a5 84 01 14 85 01 14 33 5a'))
        self.assertEqual(fields, [(CmdId.DRIVE_LINEAR, bytes([0x14])),
                                  (CmdId.DRIVE_ANGULAR, bytes([0x14]))])

    def test_get_info(self):
        # a5 89 00 00 89 5a
        fields = parse_frame(bytes.fromhex('a5 89 00 00 89 5a'))
        self.assertEqual(fields, [(CmdId.GET_INFO, b'')])

    def test_heartbeat_protobuf(self):
        # a5 80 09 08 01 10 93 d4 a3 c7 c4 32 69 5a
        frame = bytes.fromhex('a5 80 09 08 01 10 93 d4 a3 c7 c4 32 69 5a')
        fields = parse_frame(frame)
        self.assertEqual(fields, [(CmdId.HEARTBEAT,
                                   bytes.fromhex('08 01 10 93 d4 a3 c7 c4 32'))])

    def test_bad_checksum(self):
        frame = bytes.fromhex('a5 96 02 08 01 a1 5a')
        broken = frame[:-2] + bytes([0x00]) + frame[-1:]
        self.assertIsNone(parse_frame(broken))


class TestBuildFrame(unittest.TestCase):
    def test_build_matches_documentation(self):
        # drive example should round-trip exactly
        fields = [(CmdId.DRIVE_LINEAR, bytes([0x14])), (CmdId.DRIVE_ANGULAR, bytes([0x14]))]
        self.assertEqual(build_frame(fields).hex(), 'a5840114850114335a')

    def test_checksum_over_unescaped(self):
        # checksum is computed over unescaped field bytes
        self.assertEqual(checksum8(bytes([0x84, 0x01, 0x14, 0x85, 0x01, 0x14])), 0x33)

    def test_build_parse_round_trip_escapes_data(self):
        # payload containing A5/5A/F5 survives escape/unescape
        fields = [(CmdId.INIT, bytes([0xA5, 0x5A, 0xF5, 0x00]))]
        frame = build_frame(fields)
        self.assertEqual(frame[0], PROTOCOL_HEAD)
        self.assertEqual(frame[-1], PROTOCOL_END)
        self.assertEqual(parse_frame(frame), fields)


class TestExtractFrames(unittest.TestCase):
    def test_multiple_frames_and_garbage(self):
        f1 = build_frame([(CmdId.GET_INFO, b'')])
        f2 = build_frame([(CmdId.TELEOP, bytes([0x08, 0x01]))])
        buf = b'\x00\x11garbage' + f1 + f2
        frames, leftover = extract_frames(buf)
        self.assertEqual(frames, [f1, f2])
        self.assertEqual(leftover, b'')

    def test_partial_frame_waits(self):
        f1 = build_frame([(CmdId.GET_INFO, b'')])
        frames, leftover = extract_frames(f1[:-2])  # missing checksum + tail
        self.assertEqual(frames, [])
        self.assertEqual(leftover, f1[:-2])


if __name__ == '__main__':
    unittest.main()
