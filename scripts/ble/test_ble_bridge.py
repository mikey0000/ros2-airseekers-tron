"""End-to-end tests for ble_bridge.py: a PTY pair as the serial port, FakeBackend as the Go
backend, ble_test_fakes.Phone as the web client. No hardware."""
import asyncio
import gzip
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ble_bridge  # noqa: E402
import ble_mux as mx  # noqa: E402
from ble_echo_daemon import open_port  # noqa: E402
from ble_test_fakes import BIG_TEXT, CHUNKED_JSON, FakeBackend, Phone  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def scenario(body, session_timeout=5.0, backend_up=True):
    """Run ``body(phone, backend, bridge)`` against a live bridge on a fresh PTY."""
    async def go():
        backend = await FakeBackend().start()
        if not backend_up:
            await backend.stop()
        master, slave = os.openpty()
        fd = open_port(os.ttyname(slave))
        os.close(slave)
        bridge = ble_bridge.Bridge(fd, ("127.0.0.1", backend.port), token_ttl=30.0,
                                   session_timeout=session_timeout, hostname="testhost")
        stop = asyncio.Event()
        runner = asyncio.ensure_future(bridge.run(stop))
        phone = Phone(master)
        try:
            await body(phone, backend, bridge)
        finally:
            stop.set()
            await asyncio.wait_for(runner, 3)
            phone.close()
            os.close(fd)
            os.close(master)
            if backend_up:
                await backend.stop()
    asyncio.run(asyncio.wait_for(go(), 20))


async def authed(phone, backend):
    welcome = await phone.hello(backend.token)
    assert "error" not in welcome, welcome
    return welcome


def pong_pred(f):
    return f.kind == mx.PONG


# -- session --------------------------------------------------------------------------------
def test_welcome_and_ping_pong():
    async def body(phone, backend, bridge):
        w = await authed(phone, backend)
        assert w["v"] == 1 and w["host"] == "testhost" and w["max_payload"] == 480
        assert len(w["session"]) == 8 and int(w["session"], 16) >= 0
        phone.send(0, mx.PING, b"\x01\x02\x03\x04\x05\x06\x07\x08")
        pong = await phone.next(pong_pred)
        assert pong.payload == b"\x01\x02\x03\x04\x05\x06\x07\x08"
        assert backend.hits("/api/ble/token") == 1
        await phone.hello(backend.token)             # cached token: no second fetch
        assert backend.hits("/api/ble/token") == 1
    scenario(body)


def test_frames_before_hello_ignored_and_unauthenticated():
    async def body(phone, backend, bridge):
        phone.send(0, mx.PING, b"12345678")
        assert await phone.none(pong_pred)
        w = await phone.hello("WRONG")
        assert w == {"v": 1, "error": "unauthenticated"}
        assert backend.hits("/api/ble/token") == 1   # fresh fetch: no re-check
        await phone.hello("WRONG")
        assert backend.hits("/api/ble/token") == 2   # cached miss re-checked once (rotation)
        phone.send(0, mx.PING, b"12345678")
        phone.send_json(1, mx.OPEN, {"t": "http", "method": "GET", "path": "/api/settings"})
        assert await phone.none(lambda f: True)
        assert backend.hits("/api/settings") == 0
    scenario(body)


def test_rotated_token_accepted_despite_cache():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        backend.token = "ROTATEDTOKEN234567ABCDEFGHIJKLM"
        w = await phone.hello(backend.token)
        assert "session" in w and backend.hits("/api/ble/token") == 2
    scenario(body)


def test_bad_hello_and_backend_unavailable():
    async def body(phone, backend, bridge):
        phone.send(0, mx.HELLO, b"{not json")
        fr = await phone.next(lambda f: f.kind == mx.WELCOME)
        assert json.loads(fr.payload) == {"v": 1, "error": "bad_hello"}
        w = await phone.hello("ANY")
        assert w == {"v": 1, "error": "backend_unavailable"}
    scenario(body, backend_up=False)


def test_session_timeout_closes_backend_socket():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        phone.send_json(1, mx.OPEN, {"t": "ws", "path": "/api/ws/echo"})
        assert json.loads((await phone.on(1)).payload) == {"status": 101}
        await asyncio.sleep(0.8)                     # silence > 0.3 s session timeout
        await asyncio.wait_for(backend.ws_closed.wait(), 2)
        assert backend.ws_closes == [(1001, "")]
        phone.send(0, mx.PING, b"12345678")
        assert await phone.none(pong_pred)
    scenario(body, session_timeout=0.3)


def test_seq_gap_sends_reset():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        phone.send(0, mx.PING, b"aaaaaaaa", seq=phone.seq + 5)
        fr = await phone.next(lambda f: f.kind == mx.RESET)
        assert fr.seq == 0
        phone.seq = 0
        phone.send(0, mx.PING, b"bbbbbbbb")
        pong = await phone.next(pong_pred)
        assert pong.payload == b"bbbbbbbb" and pong.seq == 0
    scenario(body)


def test_phone_reset_discards_streams():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        phone.send_json(1, mx.OPEN, {"t": "ws", "path": "/api/ws/echo"})
        await phone.on(1)
        phone.send(0, mx.RESET)
        phone.seq = 0
        await asyncio.wait_for(backend.ws_closed.wait(), 2)
        phone.send(0, mx.PING, b"cccccccc")
        assert (await phone.next(pong_pred)).seq == 0
        phone.send(1, mx.END, b"x")                  # stream 1 is gone
        close = await phone.on(1)
        assert close.kind == mx.CLOSE
        assert json.loads(close.payload) == {"code": 4404, "reason": "no such stream"}
    scenario(body)


# -- HTTP -----------------------------------------------------------------------------------
def test_http_get_json_header_whitelists():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        phone.send_json(1, mx.OPEN, {"t": "http", "method": "GET", "path": "/api/settings",
                                     "headers": {"Accept": "application/json", "cookie": "x",
                                                 "origin": "evil"}})
        phone.send(1, mx.END)
        head, frames, close = await phone.response(1)
        assert head == {"status": 200, "headers": {"content-type": "application/json",
                                                   "etag": "\"e1\""}}
        assert frames == [(mx.END, b'{"a":1}')] and close is None
        _, _, h = backend.requests[-1]
        assert h["x-mower-transport"] == "ble" and h["accept-encoding"] == "identity"
        assert h["accept"] == "application/json" and h["host"] == "127.0.0.1:%d" % backend.port
        assert "cookie" not in h and "origin" not in h
    scenario(body)


def test_http_post_echo_multi_frame_body():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        data = bytes(range(256)) * 5
        phone.send_json(3, mx.OPEN, {"t": "http", "method": "POST", "path": "/api/echo",
                                     "headers": {"content-type": "application/octet-stream"}})
        phone.send_message(3, data)
        head, frames, _ = await phone.response(3)
        assert head["status"] == 200 and "content-encoding" not in head["headers"]
        assert b"".join(p for _, p in frames) == data
        assert [k for k, _ in frames][-1] == mx.END
        assert all(len(p) <= 480 for _, p in frames)
    scenario(body)


def test_http_gzip_streamed_in_frames():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        assert len(BIG_TEXT) >= 2048
        for sid, path, want in ((5, "/api/big", BIG_TEXT), (7, "/api/chunked", CHUNKED_JSON)):
            phone.send_json(sid, mx.OPEN, {"t": "http", "method": "GET", "path": path})
            phone.send(sid, mx.END)
            head, frames, _ = await phone.response(sid)
            assert head["headers"]["content-encoding"] == "gzip"
            assert all(k == mx.DATA for k, _ in frames[:-1]) and frames[-1][0] == mx.END
            assert all(len(p) <= 480 for _, p in frames)
            assert gzip.decompress(b"".join(p for _, p in frames)) == want
    scenario(body)


def test_http_forbidden_paths_never_reach_backend():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        sid = 1
        for path in ("/etc/passwd", "/api/../secret", "/api/x\r\nEvil: 1", "/index.html"):
            phone.send_json(sid, mx.OPEN, {"t": "http", "method": "GET", "path": path})
            phone.send(sid, mx.END)
            head, frames, _ = await phone.response(sid)
            assert head == {"status": 403} and frames == []
            sid += 2
        assert len(backend.requests) == 1            # the token fetch only
        phone.send_json(sid, mx.OPEN, {"t": "http", "method": "GET", "path": "/web-build.json"})
        phone.send(sid, mx.END)
        head, frames, _ = await phone.response(sid)
        assert head["status"] == 200 and frames == [(mx.END, b'{"build":"t"}')]
    scenario(body)


def test_http_backend_down_is_502():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        bridge.backend = ("127.0.0.1", 1)
        phone.send_json(1, mx.OPEN, {"t": "http", "method": "GET", "path": "/api/settings"})
        phone.send(1, mx.END)
        head, frames, _ = await phone.response(1)
        assert head == {"status": 502} and frames == []
    scenario(body)


def test_tunnel_errors():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        phone.send(9, mx.DATA, b"x")
        fr = await phone.on(9)
        assert fr.kind == mx.CLOSE and json.loads(fr.payload)["code"] == 4404
        phone.send(11, mx.CLOSE)                     # CLOSE on an unknown stream: no answer
        assert await phone.none(lambda f: f.stream == 11)
        phone.send(13, mx.OPEN, b"{nope")
        assert json.loads((await phone.on(13)).payload)["code"] == 4400
        phone.send_json(15, mx.OPEN, {"t": "ws", "path": "/api/ws/echo"})
        await phone.on(15)
        phone.send_json(15, mx.OPEN, {"t": "ws", "path": "/api/ws/echo"})
        fr = await phone.next(lambda f: f.stream == 15 and f.kind == mx.CLOSE)
        assert json.loads(fr.payload) == {"code": 4409, "reason": "stream in use"}
    scenario(body)


# -- WebSocket ------------------------------------------------------------------------------
def test_websocket_text_binary_and_close_codes():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        phone.send_json(1, mx.OPEN, {"t": "ws", "path": "/api/ws/echo", "prio": True,
                                     "lossy": False})
        assert json.loads((await phone.on(1)).payload) == {"status": 101}
        _, _, h = backend.requests[-1]
        assert h["x-mower-transport"] == "ble" and "origin" not in h
        phone.send_message(1, b"hi", text=True)
        fr = await phone.on(1)
        assert (fr.kind, fr.payload) == (mx.TEXT_END, b"hi")
        blob = bytes(range(256)) * 4
        phone.send_message(1, blob)
        got = []
        while True:
            fr = await phone.on(1)
            got.append(fr)
            if fr.kind != mx.DATA:
                break
        assert got[-1].kind == mx.END and b"".join(f.payload for f in got) == blob
        phone.send_message(1, b"close:4321", text=True)
        fr = await phone.on(1)
        assert fr.kind == mx.CLOSE and json.loads(fr.payload) == {"code": 4321, "reason": "bye"}

        phone.send_json(3, mx.OPEN, {"t": "ws", "path": "/api/ws/echo"})
        await phone.on(3)
        backend.ws_closed.clear()
        backend.ws_closes.clear()
        phone.send_json(3, mx.CLOSE, {"code": 4000, "reason": "switch"})
        await asyncio.wait_for(backend.ws_closed.wait(), 2)
        assert backend.ws_closes == [(4000, "switch")]
        assert await phone.none(lambda f: f.stream == 3)   # a CLOSE is not answered
    scenario(body)


def test_websocket_refused_and_forbidden():
    async def body(phone, backend, bridge):
        await authed(phone, backend)
        phone.send_json(1, mx.OPEN, {"t": "ws", "path": "/api/ws/missing"})
        head, frames, _ = await phone.response(1)
        assert head == {"status": 404} and frames == []
        phone.send_json(3, mx.OPEN, {"t": "ws", "path": "/web-build.json"})
        head, _, _ = await phone.response(3)
        assert head == {"status": 403}
    scenario(body)


# -- process --------------------------------------------------------------------------------
def test_sigterm_exits_cleanly():
    master, slave = os.openpty()
    path = os.ttyname(slave)
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "ble_bridge.py"), "--port", path,
                             "--force", "--backend", "http://127.0.0.1:1"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 10
        line = ""
        while "ble_bridge on" not in line and time.monotonic() < deadline:
            line = proc.stdout.readline()
        assert "ble_bridge on" in line
        proc.send_signal(signal.SIGTERM)
        assert proc.wait(5) == 0
        assert "stopped" in proc.stdout.read()
    finally:
        if proc.poll() is None:
            proc.kill()
        os.close(master)
        os.close(slave)
