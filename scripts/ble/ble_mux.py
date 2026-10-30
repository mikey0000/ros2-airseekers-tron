"""BLE tunnel framing and link-level session logic (docs/ble_tunnel_protocol.md, sections 2-5).

Pure and I/O free so it can be tested with an injected clock:

- ``encode_frame`` / ``FrameParser``: the wire format (sync, len, stream, kind, seq, payload,
  CRC-16/CCITT-FALSE), with resync on the sync word and silent drop of bad frames;
- ``Link``: one link session on the mower side. Receive ``seq`` checking (gap -> RESET,
  stale-frame drop after our own RESET), the CREDIT window, the token-bucket pacer and the
  outgoing queues (control first, then ``prio`` streams, then the others round robin, the lossy
  drop-oldest-whole-messages rule);
- ``Reassembler``: DATA* followed by END / TEXT_END into one message.

Python stdlib only.
"""
from __future__ import annotations

import struct
import time
from collections import OrderedDict, deque
from typing import Callable, NamedTuple

SYNC = b"\xAA\x55"
_HEAD = struct.Struct("<HHBB")      # len, stream, kind, seq (after the sync word)
HEADER_SIZE = 8                     # sync + _HEAD
OVERHEAD = 10                       # header + crc16
MAX_PAYLOAD = 480
MAX_FRAME = MAX_PAYLOAD + OVERHEAD  # 490, fits one 497-byte GATT write

OPEN, HEAD, DATA, END, TEXT_END, CLOSE, CREDIT, RESET, PING, PONG, HELLO, WELCOME = range(12)
KIND_NAMES = {OPEN: "OPEN", HEAD: "HEAD", DATA: "DATA", END: "END", TEXT_END: "TEXT_END",
              CLOSE: "CLOSE", CREDIT: "CREDIT", RESET: "RESET", PING: "PING", PONG: "PONG",
              HELLO: "HELLO", WELCOME: "WELCOME"}

DEFAULT_WINDOW = 2048
MAX_WINDOW = 8192
DEFAULT_RATE = 5000
MAX_RATE = 8000
LOSSY_KEEP = 4                      # queued whole messages kept on a lossy stream


def _crc_table():
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        table.append(crc & 0xFFFF)
    return table


_CRC_TABLE = _crc_table()


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: ``crc16(b"123456789") == 0x29B1``."""
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[(crc >> 8) ^ byte]
    return crc


class Frame(NamedTuple):
    stream: int
    kind: int
    seq: int
    payload: bytes


def encode_frame(stream: int, kind: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload %d > %d bytes" % (len(payload), MAX_PAYLOAD))
    body = _HEAD.pack(len(payload), stream & 0xFFFF, kind & 0xFF, seq & 0xFF) + bytes(payload)
    return SYNC + body + crc16(body).to_bytes(2, "little")


def decode_frame(raw: bytes) -> Frame:
    """One whole frame as produced by encode_frame (no CRC check)."""
    length, stream, kind, seq = _HEAD.unpack_from(raw, 2)
    return Frame(stream, kind, seq, bytes(raw[HEADER_SIZE:HEADER_SIZE + length]))


def frame_name(fr: Frame) -> str:
    return "%s stream=%d seq=%d len=%d" % (KIND_NAMES.get(fr.kind, "kind%d" % fr.kind),
                                           fr.stream, fr.seq, len(fr.payload))


class FrameParser:
    """Streaming decoder. Junk, false sync words, oversize lengths and bad CRCs are skipped
    one byte at a time, so the next real sync word is always found."""

    def __init__(self):
        self.buf = bytearray()
        self.frames = self.crc_errors = self.bad_lengths = self.garbage_bytes = 0

    def feed(self, data: bytes) -> list[Frame]:
        self.buf += data
        out = []
        buf = self.buf
        while True:
            i = buf.find(SYNC)
            if i < 0:
                keep = 1 if buf[-1:] == SYNC[:1] else 0     # a sync word split across reads
                self.garbage_bytes += len(buf) - keep
                del buf[:len(buf) - keep]
                break
            if i:
                self.garbage_bytes += i
                del buf[:i]
            if len(buf) < HEADER_SIZE:
                break
            length, stream, kind, seq = _HEAD.unpack_from(buf, 2)
            if length > MAX_PAYLOAD:
                self.bad_lengths += 1
                self.garbage_bytes += 1
                del buf[:1]
                continue
            total = HEADER_SIZE + length + 2
            if len(buf) < total:
                break
            crc = int.from_bytes(buf[total - 2:total], "little")
            if crc16(buf[2:total - 2]) != crc:
                self.crc_errors += 1
                self.garbage_bytes += 1
                del buf[:1]
                continue
            out.append(Frame(stream, kind, seq, bytes(buf[HEADER_SIZE:total - 2])))
            self.frames += 1
            del buf[:total]
        return out


def split_message(data: bytes, last_kind: int = END) -> list[tuple[int, bytes]]:
    """One message as frames: DATA* then END/TEXT_END (the last may be empty)."""
    chunks = [data[i:i + MAX_PAYLOAD] for i in range(0, len(data), MAX_PAYLOAD)] or [b""]
    return [(DATA, c) for c in chunks[:-1]] + [(last_kind, chunks[-1])]


class Reassembler:
    """DATA* + END | TEXT_END -> (is_text, bytes). ``feed`` returns None until a message ends."""

    def __init__(self, max_size: int = 16 << 20):
        self.parts: list[bytes] = []
        self.size = 0
        self.max_size = max_size

    def feed(self, kind: int, payload: bytes):
        self.size += len(payload)
        if self.size > self.max_size:
            raise ValueError("message larger than %d bytes" % self.max_size)
        self.parts.append(payload)
        if kind == DATA:
            return None
        msg = b"".join(self.parts)
        self.parts, self.size = [], 0
        return kind == TEXT_END, msg


def hello_params(hello: dict) -> tuple[int, int]:
    """(window, max_rate) from a HELLO: missing, non-integer or < 1 take the default,
    values above the maximum are clamped."""
    def pick(key, default, cap):
        v = hello.get(key)
        if not isinstance(v, int) or isinstance(v, bool) or v < 1:
            return default
        return min(v, cap)
    return pick("window", DEFAULT_WINDOW, MAX_WINDOW), pick("max_rate", DEFAULT_RATE, MAX_RATE)


class _Unit:
    """Frames that leave together in order: one message, or one HEAD / DATA / CLOSE frame."""
    __slots__ = ("frames", "droppable", "started", "final")

    def __init__(self, frames, droppable=False, final=False):
        self.frames = deque(frames)
        self.droppable = droppable
        self.started = False
        self.final = final          # the stream's last unit (its CLOSE): forget it once sent


class _StreamOut:
    __slots__ = ("prio", "lossy", "units", "nbytes", "dropped")

    def __init__(self, prio, lossy):
        self.prio, self.lossy = prio, lossy
        self.units: deque[_Unit] = deque()
        self.nbytes = 0
        self.dropped = 0


# results of Link.accept
ACCEPT, DROP, GAP = "accept", "drop", "gap"


class Link:
    """Mower-side state of one link session (one HELLO until RESET / timeout / next HELLO)."""

    def __init__(self, clock: Callable[[], float] = time.monotonic):
        self.clock = clock
        self.streams: dict[int, _StreamOut] = {}
        self.order: OrderedDict[int, None] = OrderedDict()   # round-robin order, all streams
        self.control: deque[tuple[int, int, bytes]] = deque()
        self.start(DEFAULT_WINDOW, DEFAULT_RATE, 0)

    # -- session -----------------------------------------------------------------------------
    def start(self, window: int, max_rate: int, hello_seq: int) -> None:
        """New session from a HELLO carrying ``hello_seq``."""
        self.window, self.max_rate = window, max_rate
        self.capacity = float(max(window, MAX_FRAME))
        self.tokens, self.t_tokens = self.capacity, self.clock()
        self._clear_tx()
        self.expected = (hello_seq + 1) & 0xFF
        self.awaiting_zero = False

    def _clear_tx(self):
        self.streams.clear()
        self.order.clear()
        self.control.clear()
        self.tx_seq = 0
        self.bytes_sent = 0
        self.last_credit = 0

    def on_peer_reset(self) -> None:
        """The phone sent RESET: drop every queue, send and expect seq 0, credits from 0."""
        self._clear_tx()
        self.expected = 0
        self.awaiting_zero = False

    def send_reset(self) -> None:
        """We saw a gap: drop every queue and send RESET. It goes out with seq 0, and the
        frame after it has seq 0 again (counters restart after the RESET, see pull())."""
        self._clear_tx()
        self.control.append((0, RESET, b""))
        self.expected = 0
        self.awaiting_zero = True

    def accept(self, fr: Frame) -> str:
        """Sequence check for an in-session frame other than HELLO and RESET."""
        if self.awaiting_zero:
            if fr.seq != 0:
                return DROP                 # stale: sent before the phone saw our RESET
            self.awaiting_zero = False
        elif fr.seq != self.expected:
            return GAP
        self.expected = (fr.seq + 1) & 0xFF
        return ACCEPT

    # -- credit + pacing ---------------------------------------------------------------------
    def on_credit(self, payload: bytes) -> None:
        if len(payload) >= 4:
            self.last_credit = int.from_bytes(payload[:4], "little")

    def in_flight(self) -> int:
        d = (self.bytes_sent - self.last_credit) & 0xFFFFFFFF
        return 0 if d >= 0x80000000 else d       # credit ahead of us (a race): nothing in flight

    def _refill(self, now):
        self.tokens = min(self.capacity, self.tokens + (now - self.t_tokens) * self.max_rate)
        self.t_tokens = now

    # -- outgoing queues ---------------------------------------------------------------------
    def send_control(self, kind: int, payload: bytes = b"", stream: int = 0) -> None:
        """Control frames bypass credit and pacing (but count toward both). Tunnel error
        replies on a stream id the mower does not hold (CLOSE 4404 / 4409) go here too."""
        self.control.append((stream, kind, bytes(payload)))

    def open_stream(self, sid: int, prio: bool = False, lossy: bool = False) -> None:
        self.streams[sid] = _StreamOut(prio, lossy)
        self.order[sid] = None

    def has_stream(self, sid: int) -> bool:
        return sid in self.streams

    def drop_stream(self, sid: int) -> None:
        """Forget a stream and everything queued for it (abort, CLOSE from the phone)."""
        self.streams.pop(sid, None)
        self.order.pop(sid, None)

    def _push(self, sid, unit):
        s = self.streams.get(sid)
        if s is None:
            return False
        s.units.append(unit)
        s.nbytes += sum(len(p) + OVERHEAD for _, p in unit.frames)
        return True

    def send_frame(self, sid: int, kind: int, payload: bytes = b"") -> bool:
        """One frame (HEAD, an HTTP body DATA / END) on a stream, never dropped."""
        return self._push(sid, _Unit([(kind, bytes(payload))]))

    def send_close(self, sid: int, payload: bytes = b"") -> bool:
        """CLOSE after everything already queued; the stream is forgotten once it is sent."""
        return self._push(sid, _Unit([(CLOSE, bytes(payload))], final=True))

    def send_message(self, sid: int, data: bytes, text: bool) -> bool:
        s = self.streams.get(sid)
        if s is None:
            return False
        unit = _Unit(split_message(bytes(data), TEXT_END if text else END), droppable=s.lossy)
        self._push(sid, unit)
        if s.lossy:
            waiting = [u for u in s.units if u.droppable and not u.started]
            for u in waiting[:max(0, len(waiting) - LOSSY_KEEP)]:
                s.units.remove(u)
                s.nbytes -= sum(len(p) + OVERHEAD for _, p in u.frames)
                s.dropped += 1
        return True

    def queued_bytes(self, sid: int) -> int:
        s = self.streams.get(sid)
        return s.nbytes if s else 0

    def pending(self) -> bool:
        return bool(self.control) or any(s.units for s in self.streams.values())

    def _encode(self, stream, kind, payload, now):
        if kind == RESET:
            raw = encode_frame(0, RESET, 0)
            self.tx_seq = 0
            self.bytes_sent = self.last_credit = 0
            self.tokens -= len(raw)
            return raw
        raw = encode_frame(stream, kind, self.tx_seq, payload)
        self.tx_seq = (self.tx_seq + 1) & 0xFF
        self.bytes_sent = (self.bytes_sent + len(raw)) & 0xFFFFFFFF
        self.tokens -= len(raw)
        return raw

    def _candidate(self):
        for want_prio in (True, False):
            for sid in self.order:
                s = self.streams[sid]
                if s.prio == want_prio and s.units:
                    return sid, s
        return None, None

    def pull(self) -> tuple[bytes | None, float | None]:
        """Next frame to write, or (None, wake_at): ``wake_at`` is when pacing allows the next
        stream frame (None when idle or waiting for CREDIT)."""
        now = self.clock()
        self._refill(now)
        if self.control:
            stream, kind, payload = self.control.popleft()
            return self._encode(stream, kind, payload, now), None
        sid, s = self._candidate()
        if s is None or self.in_flight() >= self.window:
            return None, None
        unit = s.units[0]
        kind, payload = unit.frames[0]
        size = len(payload) + OVERHEAD
        if self.tokens < size:
            return None, now + (size - self.tokens) / self.max_rate
        unit.frames.popleft()
        unit.started = True
        s.nbytes -= size
        if not unit.frames:
            s.units.popleft()
            if unit.final:
                self.drop_stream(sid)
        if sid in self.order:
            self.order.move_to_end(sid)
        return self._encode(sid, kind, payload, now), None
