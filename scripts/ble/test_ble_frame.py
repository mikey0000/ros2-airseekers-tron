"""Tests for ble_frame.py. Plain asserts, so ``python3 ble_frame.py selftest`` runs them too."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ble_frame as bf  # noqa: E402


def test_crc_check_value():
    assert bf.crc16_ccitt(b"123456789") == 0x29B1
    assert bf.crc16_ccitt(b"") == 0xFFFF


def test_encode_layout():
    raw = bf.encode(0x01020304, bf.ECHO, b"hi")
    assert raw[:2] == b"\xAA\x55"
    assert raw[2:4] == b"\x02\x00"                 # len LE
    assert raw[4:8] == b"\x04\x03\x02\x01"         # seq LE
    assert raw[8] == bf.ECHO
    assert raw[9:11] == b"hi"
    assert len(raw) == bf.OVERHEAD + 2
    assert int.from_bytes(raw[-2:], "little") == bf.crc16_ccitt(raw[2:-2])


def test_roundtrip_and_empty_payload():
    p = bf.Parser()
    frames = p.feed(bf.encode(7, bf.PING, b"\x00" * 8) + bf.encode(8, bf.STATS_REQ))
    assert frames == [bf.Frame(7, bf.PING, b"\x00" * 8), bf.Frame(8, bf.STATS_REQ, b"")]
    assert p.crc_errors == 0 and p.garbage_bytes == 0


def test_byte_by_byte_feed():
    raw = b"".join(bf.encode(i, bf.SINK, bytes(range(i * 3 % 50))) for i in range(20))
    p = bf.Parser()
    got = []
    for b in raw:
        got += p.feed(bytes([b]))
    assert [f.seq for f in got] == list(range(20))
    assert p.garbage_bytes == 0


def test_garbage_prefix_and_between():
    p = bf.Parser()
    raw = b"CONNECTED\r\n\xAA" + bf.encode(1, bf.ECHO, b"x") + b"\x00\xAA\x55" + bf.encode(2, bf.ECHO)
    frames = p.feed(raw)
    assert [f.seq for f in frames] == [1, 2]
    assert p.garbage_bytes == len(b"CONNECTED\r\n\xAA") + 3
    assert bytes(p.garbage_log).startswith(b"CONNECTED")


def test_crc_error_then_recover():
    bad = bytearray(bf.encode(1, bf.ECHO, b"abcdef"))
    bad[10] ^= 0x40
    p = bf.Parser()
    frames = p.feed(bytes(bad) + bf.encode(2, bf.ECHO, b"ok"))
    assert [f.seq for f in frames] == [2]
    assert p.crc_errors == 1


def test_corrupt_length_does_not_swallow_next_frame():
    bad = bytearray(bf.encode(1, bf.ECHO, b"abc"))
    bad[2] = 0xFF  # len 0x00FF: CRC fails once enough bytes arrive
    good = bf.encode(2, bf.ECHO, b"z" * 300)
    p = bf.Parser()
    frames = p.feed(bytes(bad) + good)
    assert [f.seq for f in frames] == [2]
    assert p.crc_errors == 1


def test_impossible_length_resyncs():
    bad = bytearray(bf.encode(1, bf.ECHO, b"abc"))
    bad[3] = 0x40  # len > MAX_PAYLOAD
    p = bf.Parser()
    frames = p.feed(bytes(bad) + bf.encode(2, bf.ECHO, b"ok"))
    assert [f.seq for f in frames] == [2]
    assert p.bad_lengths == 1


def test_sync_word_inside_payload():
    payload = b"\xAA\x55\xAA\x55" * 10
    p = bf.Parser()
    frames = p.feed(bf.encode(5, bf.ECHO, payload))
    assert frames == [bf.Frame(5, bf.ECHO, payload)]


def test_truncated_frame_waits():
    raw = bf.encode(3, bf.SINK, b"q" * 100)
    p = bf.Parser()
    assert p.feed(raw[:50]) == []
    assert p.feed(raw[50:]) == [bf.Frame(3, bf.SINK, b"q" * 100)]


def test_dropped_bytes_counted_as_crc_error_or_garbage():
    a, b, c = (bf.encode(i, bf.SOURCE_DATA, bf.pattern(i, 200)) for i in range(3))
    p = bf.Parser()
    frames = p.feed(a + b[:80] + b[120:] + c)   # 40 bytes lost in the middle frame
    assert [f.seq for f in frames] == [0, 2]


def test_seq_tracker():
    t = bf.SeqTracker()
    for s in [10, 11, 12, 15, 16, 20]:
        t.update(s)
    assert t.gaps == 2 + 3
    t.update(0)                      # sender restarted
    assert t.resyncs == 1 and t.gaps == 5
    t.update(0xFFFFFFFF)
    t.update(0)                      # wrap is not a gap
    assert t.resyncs == 2            # 0 -> 0xFFFFFFFF is backwards
    t2 = bf.SeqTracker()
    t2.update(0xFFFFFFFF)
    t2.update(0)
    assert t2.gaps == 0 and t2.resyncs == 0


def test_stats_roundtrip():
    vals = {k: i * 1000 + 1 for i, k in enumerate(bf.STATS_FIELDS)}
    assert bf.unpack_stats(bf.pack_stats(vals)) == vals


def test_source_req_and_pattern():
    raw = bf.SOURCE_REQ.pack(0, 60000, 200, 0)
    assert bf.SOURCE_REQ.unpack(raw) == (0, 60000, 200, 0)
    assert bf.pattern(254, 4) == b"\xFE\xFF\x00\x01"


def test_oversize_payload_rejected():
    try:
        bf.encode(0, bf.ECHO, b"x" * (bf.MAX_PAYLOAD + 1))
    except ValueError:
        return
    raise AssertionError("expected ValueError")
