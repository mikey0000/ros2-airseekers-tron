"""Tests for ble_mux.py: golden vectors, parser resync, credit, pacing, ordering, lossy, RESET."""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ble_mux as mx  # noqa: E402

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "testdata",
                       "mux_vectors.json")) as _f:
    VECTORS = json.load(_f)
FRAMES = {f["name"]: f for f in VECTORS["frames"]}


def payload_of(v):
    return v["payload_utf8"].encode() if "payload_utf8" in v else bytes.fromhex(v["payload_hex"])


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def drain(link, limit=1000):
    out = []
    for _ in range(limit):
        raw, _wake = link.pull()
        if raw is None:
            break
        out.append(mx.decode_frame(raw))
    return out


def session(window=2048, rate=5000):
    clock = Clock()
    link = mx.Link(clock)
    link.start(window, rate, 0)
    return link, clock


# -- codec ---------------------------------------------------------------------------------
def test_crc_check_value():
    assert mx.crc16(VECTORS["crc_check"]["input_utf8"].encode()) == VECTORS["crc_check"]["crc"]
    assert mx.crc16(b"123456789") == 0x29B1


@pytest.mark.parametrize("name", sorted(FRAMES))
def test_golden_encode(name):
    v = FRAMES[name]
    assert mx.encode_frame(v["stream"], v["kind"], v["seq"], payload_of(v)).hex() == v["wire_hex"]


@pytest.mark.parametrize("name", sorted(FRAMES))
def test_golden_parse(name):
    v = FRAMES[name]
    frames = mx.FrameParser().feed(bytes.fromhex(v["wire_hex"]))
    assert frames == [mx.Frame(v["stream"], v["kind"], v["seq"], payload_of(v))]


@pytest.mark.parametrize("case", VECTORS["streams"], ids=lambda c: c["name"])
def test_golden_streams(case):
    wire = bytes.fromhex(case["wire_hex"])
    want = [mx.Frame(FRAMES[n]["stream"], FRAMES[n]["kind"], FRAMES[n]["seq"],
                     payload_of(FRAMES[n])) for n in case["frames"]]
    assert mx.FrameParser().feed(wire) == want
    p = mx.FrameParser()                     # byte at a time: same result
    got = []
    for b in wire:
        got += p.feed(bytes([b]))
    assert got == want


def test_parser_counts_bad_crc_and_length():
    p = mx.FrameParser()
    p.feed(bytes.fromhex(VECTORS["streams"][2]["wire_hex"]))
    assert p.crc_errors == 1 and p.frames == 1
    p.feed(b"\xAA\x55\xFF\xFF\x00\x00\x00\x00")
    assert p.bad_lengths == 1


def test_encode_rejects_oversize():
    with pytest.raises(ValueError):
        mx.encode_frame(1, mx.DATA, 0, b"x" * 481)


def test_split_message():
    assert mx.split_message(b"") == [(mx.END, b"")]
    assert mx.split_message(b"a" * 480, mx.TEXT_END) == [(mx.TEXT_END, b"a" * 480)]
    parts = mx.split_message(b"a" * 961)
    assert [k for k, _ in parts] == [mx.DATA, mx.DATA, mx.END]
    assert [len(p) for _, p in parts] == [480, 480, 1]


def test_reassembler():
    r = mx.Reassembler(max_size=10)
    assert r.feed(mx.DATA, b"ab") is None
    assert r.feed(mx.TEXT_END, b"c") == (True, b"abc")
    assert r.feed(mx.END, b"") == (False, b"")
    with pytest.raises(ValueError):
        r.feed(mx.DATA, b"x" * 11)


def test_hello_params():
    assert mx.hello_params({}) == (2048, 5000)
    assert mx.hello_params({"window": 100000, "max_rate": 9000}) == (8192, 8000)
    assert mx.hello_params({"window": 0, "max_rate": "x"}) == (2048, 5000)
    assert mx.hello_params({"window": 1024, "max_rate": 3000}) == (1024, 3000)


# -- seq ------------------------------------------------------------------------------------
def test_seq_follows_hello_and_wraps():
    link, _ = session()
    link.start(2048, 5000, 254)
    assert link.accept(mx.Frame(0, mx.PING, 255, b"")) == mx.ACCEPT
    assert link.accept(mx.Frame(0, mx.PING, 0, b"")) == mx.ACCEPT
    assert link.accept(mx.Frame(0, mx.PING, 2, b"")) == mx.GAP


def test_tx_seq_increments_and_wraps():
    link, _ = session(window=8192, rate=8000)
    link.tx_seq = 254
    for _ in range(3):
        link.send_control(mx.PONG, b"12345678")
    assert [f.seq for f in drain(link)] == [254, 255, 0]


def test_gap_sends_reset_then_seq_restarts_and_stale_dropped():
    link, _ = session()
    link.open_stream(1)
    link.send_message(1, b"x" * 1000, False)
    link.send_control(mx.PONG, b"p" * 8)
    assert link.accept(mx.Frame(0, mx.PING, 5, b"")) == mx.GAP
    link.send_reset()
    assert not link.has_stream(1)
    frames = drain(link)
    assert [(f.kind, f.seq) for f in frames] == [(mx.RESET, 0)]
    assert link.bytes_sent == 0
    # stale frames the phone sent before it saw the RESET are dropped, not a new gap
    assert link.accept(mx.Frame(0, mx.CREDIT, 7, b"")) == mx.DROP
    assert link.accept(mx.Frame(0, mx.PING, 0, b"")) == mx.ACCEPT
    assert link.accept(mx.Frame(0, mx.PING, 1, b"")) == mx.ACCEPT
    link.send_control(mx.PONG, b"p" * 8)
    assert drain(link)[0].seq == 0


def test_peer_reset_restarts_counters():
    link, _ = session()
    link.send_control(mx.PONG, b"p" * 8)
    drain(link)
    link.open_stream(1)
    link.send_message(1, b"abc", True)
    link.on_peer_reset()
    assert not link.pending() and link.tx_seq == 0 and link.bytes_sent == 0
    assert link.accept(mx.Frame(0, mx.PING, 0, b"")) == mx.ACCEPT


# -- credit + pacing ------------------------------------------------------------------------
def test_credit_window_blocks_stream_frames_not_control():
    link, clock = session(window=1000, rate=8000)
    link.open_stream(1)
    link.send_message(1, b"x" * 480 * 4, False)
    sent = drain(link)
    clock.t += 1.0                                   # refill the bucket: credit is the limit
    sent += drain(link)
    # 490 + 490 = 980 < 1000, third frame allowed (in-flight checked before sending)
    assert len(sent) == 3 and link.in_flight() == 1470
    link.send_control(mx.PONG, b"p" * 8)
    assert [f.kind for f in drain(link)] == [mx.PONG]
    clock.t += 10
    assert drain(link) == []                         # still waiting for CREDIT
    link.on_credit((1470).to_bytes(4, "little"))
    assert [f.kind for f in drain(link)] == [mx.END]


def test_credit_wraps_and_ahead_is_zero():
    link, _ = session()
    link.bytes_sent = 10
    link.last_credit = 0xFFFFFFF0
    assert link.in_flight() == 26
    link.last_credit = 20                            # ahead of us (race): treat as caught up
    assert link.in_flight() == 0


def test_token_bucket_paces_at_max_rate():
    link, clock = session(window=8192, rate=1000)
    link.on_credit(b"\x00\x00\x00\x00")
    link.open_stream(1)
    link.send_message(1, b"x" * 480 * 30, False)
    first = drain(link)
    assert len(first) == 8192 // 490 + 0            # burst = window (16 frames, 7840 B)
    raw, wake = link.pull()
    assert raw is None and wake is not None
    assert wake == pytest.approx(clock.t + (490 - link.tokens) / 1000)
    link.last_credit = link.bytes_sent               # plenty of credit from now on
    clock.t += 4.9                                   # 4900 tokens -> ~10 more frames
    got = drain(link)
    assert 9 <= len(got) <= 10
    sent_bytes = sum(len(f.payload) + mx.OVERHEAD for f in got)
    assert sent_bytes <= 4900 + 490


def test_control_frames_bypass_bucket_but_spend_it():
    link, clock = session(window=490, rate=1000)
    for _ in range(5):
        link.send_control(mx.PONG, b"p" * 8)
    assert len(drain(link)) == 5
    assert link.tokens < 490
    link.on_credit(link.bytes_sent.to_bytes(4, "little"))
    link.open_stream(1)
    link.send_message(1, b"y" * 400, False)
    assert drain(link) == []                         # bucket not refilled yet
    clock.t += 1.0
    assert len(drain(link)) == 1


# -- ordering -------------------------------------------------------------------------------
def test_order_control_then_prio_then_round_robin():
    link, _ = session(window=8192, rate=8000)
    link.open_stream(1)
    link.open_stream(3)
    link.open_stream(5, prio=True)
    link.send_message(1, b"a" * 960, False)          # DATA, END
    link.send_message(3, b"b" * 960, False)
    link.send_message(5, b"j" * 10, True)
    link.send_control(mx.PONG, b"p" * 8)
    got = [(f.stream, f.kind) for f in drain(link)]
    assert got == [(0, mx.PONG), (5, mx.TEXT_END), (1, mx.DATA), (3, mx.DATA),
                   (1, mx.END), (3, mx.END)]


def test_close_after_queued_data_then_stream_forgotten():
    link, _ = session(window=8192, rate=8000)
    link.open_stream(1)
    link.send_frame(1, mx.HEAD, b'{"status":200}')
    link.send_frame(1, mx.END, b"body")
    link.send_close(1)
    assert [f.kind for f in drain(link)] == [mx.HEAD, mx.END, mx.CLOSE]
    assert not link.has_stream(1)
    assert link.send_frame(1, mx.DATA, b"late") is False


# -- lossy ----------------------------------------------------------------------------------
def test_lossy_keeps_newest_four_whole_messages():
    link, clock = session(window=490, rate=8000)
    link.open_stream(1, lossy=True)
    link.send_frame(1, mx.HEAD, b'{"status":101}')    # never dropped
    link.send_message(1, b"m0" * 300, False)           # two frames
    assert [f.kind for f in drain(link)] == [mx.HEAD]  # window full now
    link.on_credit(link.bytes_sent.to_bytes(4, "little"))
    clock.t += 1.0
    assert [f.kind for f in drain(link)] == [mx.DATA]  # m0 started: must complete
    for i in range(1, 8):
        link.send_message(1, b"m%d" % i, False)
    link.on_credit(link.bytes_sent.to_bytes(4, "little"))
    link.window = link.capacity = 8192
    clock.t += 10.0
    got = drain(link)
    assert got[0].kind == mx.END and got[0].payload.startswith(b"m0")   # m0 finished
    assert [f.payload for f in got[1:]] == [b"m4", b"m5", b"m6", b"m7"]


def test_non_lossy_queues_everything():
    link, clock = session(window=490, rate=8000)
    link.open_stream(1)
    link.send_frame(1, mx.HEAD, b"{}")
    drain(link)
    for i in range(20):
        link.send_message(1, b"m%d" % i, True)
    link.window = link.capacity = 1 << 20
    clock.t += 10.0
    assert len(drain(link)) == 20


def test_drop_stream_discards_queue():
    link, _ = session(window=8192, rate=8000)
    link.open_stream(1)
    link.send_message(1, b"x" * 5000, False)
    link.drop_stream(1)
    assert drain(link) == [] and not link.pending()
