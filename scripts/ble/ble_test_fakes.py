"""Test helpers for test_ble_ws.py and test_ble_bridge.py (not collected by pytest).

``FakeBackend``: asyncio HTTP/1.1 + RFC 6455 server standing in for the Go backend.
``Phone``: the phone side of the tunnel over the master end of a PTY.
"""
from __future__ import annotations

import asyncio
import json
import os
import struct

import ble_mux as mx
import ble_ws

BIG_TEXT = ("".join("line %04d of a long text body for the gzip check\n" % i
                    for i in range(64))).encode()
CHUNKED_JSON = json.dumps({"items": [{"i": i, "name": "item-%d" % i} for i in range(80)]}).encode()


class FakeBackend:
    """Routes:
    GET /api/ble/token, GET /api/settings, POST /api/echo, GET /api/big (text, Content-Length),
    GET /api/chunked (JSON, chunked), GET /web-build.json, GET /api/ws/echo (WebSocket echo),
    anything else 404.

    WebSocket echo commands (text messages): ``frag`` (fragmented reply with an interleaved
    ping), ``ping`` (server ping; replies ``pong:<payload>`` once the pong arrives),
    ``close:<code>`` (server-initiated close, reason ``bye``); any other message is echoed with
    its type.
    """

    def __init__(self, token: str = "TESTTOKEN234567ABCDEFGHIJKLMNOPQ"):
        self.token = token
        self.requests: list[tuple[str, str, dict]] = []
        self.ws_closes: list[tuple[int, str]] = []
        self.ws_closed = asyncio.Event()
        self.server = None
        self.port = 0

    async def start(self) -> "FakeBackend":
        self.server = await asyncio.start_server(self._serve, "127.0.0.1", 0)
        self.port = self.server.sockets[0].getsockname()[1]
        return self

    async def stop(self) -> None:
        self.server.close()

    def hits(self, path: str) -> int:
        return sum(1 for _, p, _ in self.requests if p == path)

    async def _serve(self, reader, writer):
        try:
            head = await reader.readuntil(b"\r\n\r\n")
            line, *rest = head.decode("latin-1").split("\r\n")
            method, path, _ = line.split(" ", 2)
            headers = {}
            for h in rest:
                if ":" in h:
                    k, v = h.split(":", 1)
                    headers[k.strip().lower()] = v.strip()
            body = await reader.readexactly(int(headers.get("content-length", "0")))
            self.requests.append((method, path, headers))
            if headers.get("upgrade", "").lower() == "websocket" and path == "/api/ws/echo":
                await self._ws(reader, writer, headers)
                return
            await self._http(writer, method, path, headers, body)
        except (asyncio.IncompleteReadError, ConnectionError):
            pass
        finally:
            writer.close()

    async def _http(self, writer, method, path, headers, body):
        def reply(status, ctype, data, extra=""):
            writer.write(("HTTP/1.1 %d X\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
                          "ETag: \"e1\"\r\nX-Secret: no\r\n%s\r\n"
                          % (status, ctype, len(data), extra)).encode() + data)

        if method == "GET" and path == "/api/ble/token":
            reply(200, "application/json", json.dumps({"token": self.token}).encode())
        elif method == "GET" and path == "/api/settings":
            reply(200, "application/json", b'{"a":1}')
        elif method == "GET" and path == "/web-build.json":
            reply(200, "application/json", b'{"build":"t"}')
        elif method == "POST" and path == "/api/echo":
            reply(200, headers.get("content-type", "application/octet-stream"), body)
        elif method == "GET" and path == "/api/big":
            reply(200, "text/plain; charset=utf-8", BIG_TEXT)
        elif method == "GET" and path == "/api/chunked":
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                         b"Transfer-Encoding: chunked\r\n\r\n")
            for i in range(0, len(CHUNKED_JSON), 700):
                part = CHUNKED_JSON[i:i + 700]
                writer.write(b"%x\r\n%s\r\n" % (len(part), part))
                await writer.drain()
                await asyncio.sleep(0.01)
            writer.write(b"0\r\n\r\n")
        else:
            reply(404, "text/plain", b"not found")
        await writer.drain()

    async def _ws(self, reader, writer, headers):
        key = headers["sec-websocket-key"].encode()
        writer.write(b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                     b"Connection: Upgrade\r\nSec-WebSocket-Accept: " + ble_ws.accept_key(key)
                     + b"\r\n\r\n")

        def send(op, payload, fin=True):
            writer.write(ble_ws.build_frame(op, payload, mask=False, fin=fin))

        parts, first = [], None
        while True:
            fin, op, payload = await ble_ws.read_frame(reader)
            if op == ble_ws.OP_CLOSE:
                self.ws_closes.append(ble_ws.parse_close(payload))
                self.ws_closed.set()
                send(ble_ws.OP_CLOSE, payload[:2])
                await writer.drain()
                return
            if op == ble_ws.OP_PONG:
                send(ble_ws.OP_TEXT, b"pong:" + payload)
                await writer.drain()
                continue
            if op in (ble_ws.OP_TEXT, ble_ws.OP_BINARY):
                first = op
            parts.append(payload)
            if not fin:
                continue
            msg, parts = b"".join(parts), []
            if first == ble_ws.OP_TEXT and msg == b"frag":
                send(ble_ws.OP_TEXT, b"hel", fin=False)
                send(ble_ws.OP_PING, b"x")
                send(ble_ws.OP_CONT, b"lo")
            elif first == ble_ws.OP_TEXT and msg == b"ping":
                send(ble_ws.OP_PING, b"pp")
            elif first == ble_ws.OP_TEXT and msg.startswith(b"close:"):
                send(ble_ws.OP_CLOSE, struct.pack("!H", int(msg[6:])) + b"bye")
                await writer.drain()
                try:
                    _, op, payload = await ble_ws.read_frame(reader)
                    self.ws_closes.append(ble_ws.parse_close(payload))
                except asyncio.IncompleteReadError:
                    self.ws_closes.append((1006, ""))
                self.ws_closed.set()
                return
            else:
                send(first, msg)
            await writer.drain()


class Phone:
    """Phone end of the tunnel on a PTY master fd: sends frames with its own seq, parses what
    the bridge sends, and returns CREDIT after every read like the web client would."""

    def __init__(self, fd: int, credit: bool = True):
        self.fd = fd
        self.seq = 0
        self.parser = mx.FrameParser()
        self.frames: list[mx.Frame] = []
        self.event = asyncio.Event()
        self.rx_bytes = 0
        self.credit = credit
        os.set_blocking(fd, False)
        asyncio.get_running_loop().add_reader(fd, self._on_read)

    def close(self):
        asyncio.get_running_loop().remove_reader(self.fd)

    def _on_read(self):
        try:
            data = os.read(self.fd, 65536)
        except OSError:
            return
        new = 0
        for fr in self.parser.feed(data):
            if fr.kind == mx.RESET:
                self.rx_bytes, new = 0, 0
                self.seq = 0
            else:
                self.rx_bytes += len(fr.payload) + mx.OVERHEAD
                new += 1
            self.frames.append(fr)
        if new and self.credit:
            self.send(0, mx.CREDIT, (self.rx_bytes & 0xFFFFFFFF).to_bytes(4, "little"))
        self.event.set()

    def send(self, stream: int, kind: int, payload: bytes = b"", seq: int | None = None):
        raw = mx.encode_frame(stream, kind, self.seq if seq is None else seq, payload)
        self.seq = ((self.seq if seq is None else seq) + 1) & 0xFF
        view = memoryview(raw)
        while view:
            try:
                n = os.write(self.fd, view)
                view = view[n:]
            except BlockingIOError:
                pass

    def send_json(self, stream: int, kind: int, obj):
        self.send(stream, kind, json.dumps(obj).encode())

    def send_message(self, stream: int, data: bytes, text: bool = False):
        for kind, chunk in mx.split_message(data, mx.TEXT_END if text else mx.END):
            self.send(stream, kind, chunk)

    async def next(self, pred=lambda f: True, timeout: float = 3.0) -> mx.Frame:
        async def wait():
            while True:
                for i, f in enumerate(self.frames):
                    if pred(f):
                        return self.frames.pop(i)
                self.event.clear()
                await self.event.wait()
        return await asyncio.wait_for(wait(), timeout)

    async def on(self, stream: int, timeout: float = 3.0) -> mx.Frame:
        return await self.next(lambda f: f.stream == stream, timeout)

    async def none(self, pred, wait: float = 0.3) -> bool:
        try:
            await self.next(pred, wait)
            return False
        except asyncio.TimeoutError:
            return True

    async def hello(self, token: str, **extra) -> dict:
        self.seq = 0
        self.send_json(0, mx.HELLO, dict({"v": 1, "token": token, "client": "test",
                                          "window": 8192, "max_rate": 8000}, **extra))
        fr = await self.next(lambda f: f.kind == mx.WELCOME)
        return json.loads(fr.payload)

    async def response(self, stream: int, timeout: float = 5.0):
        """(head, [(kind, payload)...], close_payload) of one stream until its CLOSE."""
        head, body = None, []
        while True:
            fr = await self.on(stream, timeout)
            if fr.kind == mx.HEAD:
                head = json.loads(fr.payload)
            elif fr.kind == mx.CLOSE:
                return head, body, json.loads(fr.payload) if fr.payload else None
            else:
                body.append((fr.kind, fr.payload))
