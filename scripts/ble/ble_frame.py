#!/usr/bin/env python3
"""Framing for the phase-0 BLE link measurements (docs/DESIGN_ble_transport_2026-10.md, section 5).

Shared by ``ble_echo_daemon.py`` (mower) and ``ble_link_test.py`` (laptop). Python stdlib only.

Wire format, all integers little-endian::

    0xAA 0x55 | len u16 | seq u32 | type u8 | payload (len bytes) | crc16 u16

``len`` is the payload length. The CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over
``len | seq | type | payload``, i.e. everything between the sync word and the CRC.

Each side numbers the frames it sends with its own ``seq`` (+1 per frame), so the receiver can
count frames that never arrived. Replies have the high bit of the request type set.

    python3 ble_frame.py selftest     # runs test_ble_frame.py without pytest
"""
import struct
import sys
from collections import namedtuple

SYNC = b"\xAA\x55"
HEADER = struct.Struct("<HIB")          # len, seq, type (after the sync word)
HEADER_LEN = 2 + HEADER.size            # 9
OVERHEAD = HEADER_LEN + 2               # 11 bytes around the payload
MAX_PAYLOAD = 1024

# phone -> mower
ECHO = 0x01          # reply ECHO_REPLY with the same payload
SINK = 0x02          # counted and dropped; the daemon sends STATS about once a second
SOURCE = 0x03        # SOURCE_REQ payload; the daemon streams SOURCE_DATA then SOURCE_END
PING = 0x05          # payload = sender's timestamp; reply PONG at once with the same payload
STATS_REQ = 0x06     # reply STATS
RESET = 0x07         # zero the daemon's counters and tx seq, reply STATS
SOURCE_STOP = 0x08   # abort a running SOURCE, reply SOURCE_END
# mower -> phone
ECHO_REPLY = 0x81
SOURCE_DATA = 0x83
SOURCE_END = 0x84
PONG = 0x85
STATS = 0x86

TYPE_NAMES = {ECHO: "ECHO", SINK: "SINK", SOURCE: "SOURCE", PING: "PING", STATS_REQ: "STATS_REQ",
              RESET: "RESET", SOURCE_STOP: "SOURCE_STOP", ECHO_REPLY: "ECHO_REPLY",
              SOURCE_DATA: "SOURCE_DATA", SOURCE_END: "SOURCE_END", PONG: "PONG", STATS: "STATS"}

# SOURCE request: max payload bytes (0 = no limit), duration ms, payload bytes per frame,
# rate limit in wire bytes/s (0 = as fast as the UART takes them)
SOURCE_REQ = struct.Struct("<IIHI")
SOURCE_END_FMT = struct.Struct("<II")   # frames sent, payload bytes sent
PING_FMT = struct.Struct("<Q")          # sender's monotonic clock, ns

STATS_FIELDS = ("rx_bytes", "rx_frames", "crc_errors", "garbage_bytes", "seq_gaps",
                "sink_frames", "sink_bytes", "tx_frames", "tx_bytes", "uptime_ms")
STATS_FMT = struct.Struct("<" + "I" * len(STATS_FIELDS))

Frame = namedtuple("Frame", "seq type payload")


def crc16_ccitt(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE. ``crc16_ccitt(b"123456789") == 0x29B1``."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(seq, ftype, payload=b""):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload %d > %d bytes" % (len(payload), MAX_PAYLOAD))
    body = HEADER.pack(len(payload), seq & 0xFFFFFFFF, ftype) + bytes(payload)
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


def pattern(seq, length):
    """Deterministic SOURCE_DATA payload, so the receiver can check content beyond the CRC."""
    return bytes((seq + i) & 0xFF for i in range(length))


def pack_stats(values):
    return STATS_FMT.pack(*(int(values.get(k, 0)) & 0xFFFFFFFF for k in STATS_FIELDS))


def unpack_stats(payload):
    return dict(zip(STATS_FIELDS, STATS_FMT.unpack(payload[:STATS_FMT.size])))


class Parser:
    """Byte-stream to frames, with resync on garbage.

    A bad CRC or an impossible length drops only the first sync byte, then the search
    restarts, so a corrupt length field cannot swallow the frames after it.
    """

    def __init__(self, max_payload=MAX_PAYLOAD, garbage_keep=256):
        self.buf = bytearray()
        self.max_payload = max_payload
        self.garbage_keep = garbage_keep
        self.garbage_log = bytearray()   # first bytes of unparsed data; the caller clears it
        self.frames = 0
        self.crc_errors = 0
        self.bad_lengths = 0
        self.garbage_bytes = 0

    def _discard(self, n):
        if len(self.garbage_log) < self.garbage_keep:
            self.garbage_log += self.buf[:min(n, self.garbage_keep - len(self.garbage_log))]
        self.garbage_bytes += n
        del self.buf[:n]

    def feed(self, data):
        return list(self.iter_feed(data))

    def iter_feed(self, data):
        """Like feed(), but yields each frame before parsing on, so counters stay in step."""
        self.buf += data
        while True:
            start = self.buf.find(SYNC)
            if start < 0:
                # keep a trailing 0xAA, it may be the first half of a sync word
                keep = 1 if self.buf[-1:] == SYNC[:1] else 0
                if len(self.buf) > keep:
                    self._discard(len(self.buf) - keep)
                return
            if start:
                self._discard(start)
            if len(self.buf) < HEADER_LEN:
                return
            length, seq, ftype = HEADER.unpack_from(self.buf, 2)
            if length > self.max_payload:
                self.bad_lengths += 1
                self._discard(1)
                continue
            total = HEADER_LEN + length + 2
            if len(self.buf) < total:
                return
            body = bytes(self.buf[2:HEADER_LEN + length])
            (crc,) = struct.unpack_from("<H", self.buf, HEADER_LEN + length)
            if crc != crc16_ccitt(body):
                self.crc_errors += 1
                self._discard(1)
                continue
            del self.buf[:total]
            self.frames += 1
            yield Frame(seq, ftype, body[HEADER.size:])


class SeqTracker:
    """Counts frames missing from a sender's +1 sequence. A backwards jump resyncs."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.expected = None
        self.gaps = 0          # frames missing
        self.resyncs = 0       # sequence went backwards or repeated (sender restarted)
        self.received = 0

    def update(self, seq):
        self.received += 1
        if self.expected is not None and seq != self.expected:
            diff = (seq - self.expected) & 0xFFFFFFFF
            if diff < 0x80000000:
                self.gaps += diff
            else:
                self.resyncs += 1
        self.expected = (seq + 1) & 0xFFFFFFFF


def hexdump(data, width=16):
    lines = []
    for off in range(0, len(data), width):
        chunk = bytes(data[off:off + width])
        text = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append("%04x  %-*s  %s" % (off, width * 3 - 1, chunk.hex(" "), text))
    return "\n".join(lines)


def selftest():
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import test_ble_frame
    names = sorted(n for n in dir(test_ble_frame) if n.startswith("test_"))
    failed = 0
    for name in names:
        try:
            getattr(test_ble_frame, name)()
            print("ok    %s" % name)
        except Exception as exc:  # noqa: BLE001 - report every failure
            failed += 1
            print("FAIL  %s: %r" % (name, exc))
    print("%d/%d passed" % (len(names) - failed, len(names)))
    return 1 if failed else 0


if __name__ == "__main__":
    if sys.argv[1:] == ["selftest"]:
        sys.exit(selftest())
    print(__doc__)
    sys.exit(2)
