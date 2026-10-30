"""Tests for ble_ws.py against an in-test RFC 6455 server (ble_test_fakes.FakeBackend)."""
import asyncio
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ble_ws  # noqa: E402
from ble_test_fakes import FakeBackend  # noqa: E402


def run(coro):
    return asyncio.run(asyncio.wait_for(coro, 10))


async def with_ws(body, path="/api/ws/echo"):
    be = await FakeBackend().start()
    try:
        ws = await ble_ws.connect("127.0.0.1", be.port, path, {"X-Mower-Transport": "ble"})
        return await body(be, ws)
    finally:
        await be.stop()


def test_mask_roundtrip():
    key = b"\x01\x02\x03\x04"
    assert ble_ws.apply_mask(ble_ws.apply_mask(b"hello world", key), key) == b"hello world"
    assert ble_ws.apply_mask(b"", key) == b""


def test_accept_key_rfc_example():
    assert ble_ws.accept_key(b"dGhlIHNhbXBsZSBub25jZQ==") == b"s3pPLMBiTxaQ9kYGzzhZRbK+xOo="


def test_handshake_headers_and_echo():
    async def body(be, ws):
        _, _, h = be.requests[-1]
        assert h["x-mower-transport"] == "ble"
        assert "origin" not in h
        await ws.send(b"hi", text=True)
        assert await ws.recv() == (True, b"hi")
        blob = bytes(range(256)) * 300                # 76.8 KB: 64-bit length form
        await ws.send(blob)
        assert await ws.recv() == (False, blob)
        await ws.send(b"x" * 300)                     # 16-bit length form
        assert await ws.recv() == (False, b"x" * 300)
        await ws.close(1000, "done")
        await asyncio.wait_for(be.ws_closed.wait(), 2)
        assert be.ws_closes == [(1000, "done")]
    run(with_ws(body))


def test_fragmented_message_and_server_ping():
    async def body(be, ws):
        await ws.send(b"frag", text=True)
        assert await ws.recv() == (True, b"hello")    # ping in the middle of the message
        assert await ws.recv() == (True, b"pong:x")   # ... was answered
        await ws.send(b"ping", text=True)
        assert await ws.recv() == (True, b"pong:pp")  # server got our pong
        await ws.close()
    run(with_ws(body))


def test_server_initiated_close():
    async def body(be, ws):
        await ws.send(b"close:4321", text=True)
        with pytest.raises(ble_ws.ConnectionClosed) as exc:
            await ws.recv()
        assert (exc.value.code, exc.value.reason) == (4321, "bye")
        await asyncio.wait_for(be.ws_closed.wait(), 2)
        assert be.ws_closes == [(4321, "")]           # we echoed the code
    run(with_ws(body))


def test_close_while_recv_pending():
    async def body(be, ws):
        reader = asyncio.ensure_future(ws.recv())
        await asyncio.sleep(0.05)
        await ws.close(4000, "switch")
        with pytest.raises(ble_ws.ConnectionClosed):
            await reader
        assert be.ws_closes == [(4000, "switch")]
    run(with_ws(body))


def test_handshake_refused_carries_status():
    async def go():
        be = await FakeBackend().start()
        try:
            with pytest.raises(ble_ws.HandshakeError) as exc:
                await ble_ws.connect("127.0.0.1", be.port, "/api/nope")
            assert exc.value.status == 404
        finally:
            await be.stop()
    run(go())


def test_connection_refused():
    async def go():
        with pytest.raises(OSError):
            await ble_ws.connect("127.0.0.1", 1, "/api/x", timeout=2)
    run(go())


def test_reserved_close_codes_not_sent():
    assert ble_ws.encode_close(1005) == b""
    assert ble_ws.encode_close(1000, "ok") == b"\x03\xe8ok"
