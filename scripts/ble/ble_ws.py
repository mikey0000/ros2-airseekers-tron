"""Minimal RFC 6455 WebSocket client on asyncio streams (stdlib only).

Just what the BLE bridge needs: the opening handshake (random key, Accept check, extra
headers, no Origin), masked client frames, text / binary / continuation reassembly,
ping -> pong, and the close handshake with code and reason. No extensions, no subprotocols.
"""
from __future__ import annotations

import asyncio
import base64
import hashlib
import os
import struct

GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
OP_CONT, OP_TEXT, OP_BINARY, OP_CLOSE, OP_PING, OP_PONG = 0x0, 0x1, 0x2, 0x8, 0x9, 0xA
MAX_MESSAGE = 16 << 20
CLOSE_TIMEOUT = 2.0


class HandshakeError(Exception):
    """The server did not upgrade. ``status`` is its HTTP status (0: no valid response)."""

    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status


class ConnectionClosed(Exception):
    def __init__(self, code: int, reason: str = ""):
        super().__init__("closed %d %s" % (code, reason))
        self.code, self.reason = code, reason


def accept_key(key: bytes) -> bytes:
    return base64.b64encode(hashlib.sha1(key + GUID).digest())


def encode_close(code: int, reason: str = "") -> bytes:
    if code in (1005, 1006, 1015):     # reserved: never on the wire
        return b""
    return struct.pack("!H", code) + reason.encode("utf-8")[:123]


def build_frame(opcode: int, payload: bytes, mask: bool = True, fin: bool = True) -> bytes:
    b0 = (0x80 if fin else 0) | opcode
    n = len(payload)
    mbit = 0x80 if mask else 0
    if n < 126:
        head = struct.pack("!BB", b0, mbit | n)
    elif n < 65536:
        head = struct.pack("!BBH", b0, mbit | 126, n)
    else:
        head = struct.pack("!BBQ", b0, mbit | 127, n)
    if not mask:
        return head + payload
    key = os.urandom(4)
    return head + key + apply_mask(payload, key)


def apply_mask(data: bytes, key: bytes) -> bytes:
    if not data:
        return b""
    k = int.from_bytes((key * (len(data) // 4 + 1))[:len(data)], "big")
    return (int.from_bytes(data, "big") ^ k).to_bytes(len(data), "big")


async def read_frame(reader: asyncio.StreamReader) -> tuple[bool, int, bytes]:
    """(fin, opcode, unmasked payload). Raises IncompleteReadError at EOF."""
    b0, b1 = await reader.readexactly(2)
    n = b1 & 0x7F
    if n == 126:
        n = struct.unpack("!H", await reader.readexactly(2))[0]
    elif n == 127:
        n = struct.unpack("!Q", await reader.readexactly(8))[0]
    if n > MAX_MESSAGE:
        raise ConnectionClosed(1009, "frame too large")
    key = await reader.readexactly(4) if b1 & 0x80 else None
    payload = await reader.readexactly(n)
    if key:
        payload = apply_mask(payload, key)
    return bool(b0 & 0x80), b0 & 0x0F, payload


def parse_close(payload: bytes) -> tuple[int, str]:
    if len(payload) < 2:
        return 1005, ""
    return struct.unpack("!H", payload[:2])[0], payload[2:].decode("utf-8", "replace")


class WebSocket:
    """A connected client. ``recv`` returns (is_text, bytes) or raises ConnectionClosed."""

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        self.reader, self.writer = reader, writer
        self.close_sent = False
        self.closed: ConnectionClosed | None = None
        self._lock = asyncio.Lock()
        self._receiving = False
        self._closed_event = asyncio.Event()

    async def _write(self, opcode: int, payload: bytes) -> None:
        async with self._lock:
            self.writer.write(build_frame(opcode, payload))
            await self.writer.drain()

    async def send(self, data: bytes, text: bool = False) -> None:
        if self.closed or self.close_sent:
            raise self.closed or ConnectionClosed(1006, "closing")
        await self._write(OP_TEXT if text else OP_BINARY, bytes(data))

    async def recv(self) -> tuple[bool, bytes]:
        if self.closed:
            raise self.closed
        self._receiving = True
        try:
            return await self._recv()
        finally:
            self._receiving = False

    async def _recv(self) -> tuple[bool, bytes]:
        parts: list[bytes] = []
        first_op = None
        size = 0
        try:
            while True:
                fin, op, payload = await read_frame(self.reader)
                if op == OP_PING:
                    if not self.close_sent:
                        await self._write(OP_PONG, payload)
                    continue
                if op == OP_PONG:
                    continue
                if op == OP_CLOSE:
                    code, reason = parse_close(payload)
                    if not self.close_sent:     # server initiated: echo its code
                        self.close_sent = True
                        try:
                            await self._write(OP_CLOSE, payload[:2])
                        except (ConnectionError, OSError):
                            pass
                    self._finish(code, reason)
                    raise self.closed
                if op in (OP_TEXT, OP_BINARY) and first_op is None:
                    first_op = op
                elif op != OP_CONT or first_op is None:
                    raise ConnectionClosed(1002, "unexpected opcode %d" % op)
                size += len(payload)
                if size > MAX_MESSAGE:
                    raise ConnectionClosed(1009, "message too large")
                parts.append(payload)
                if fin:
                    return first_op == OP_TEXT, b"".join(parts)
        except (asyncio.IncompleteReadError, ConnectionError, OSError):
            self._finish(1006, "connection lost")
            raise self.closed from None
        except ConnectionClosed as exc:
            if not self.closed:             # protocol error on our side of the parse
                await self.close(exc.code, str(exc.reason))
                self._finish(exc.code, exc.reason)
            raise self.closed from None

    async def close(self, code: int = 1000, reason: str = "") -> None:
        """Send CLOSE (once), wait briefly for the server's CLOSE, then drop the socket. Safe
        to call while another task waits in ``recv`` (that task then sees the server's CLOSE)."""
        if not self.close_sent and not self.closed:
            self.close_sent = True
            try:
                await self._write(OP_CLOSE, encode_close(code, reason))
                if self._receiving:
                    await asyncio.wait_for(self._closed_event.wait(), CLOSE_TIMEOUT)
                else:
                    await asyncio.wait_for(self._await_close(), CLOSE_TIMEOUT)
            except (asyncio.TimeoutError, asyncio.IncompleteReadError, ConnectionError, OSError,
                    ConnectionClosed):
                pass
        self._finish(code, reason)

    async def _await_close(self) -> None:
        while True:
            fin, op, payload = await read_frame(self.reader)
            if op == OP_CLOSE:
                self._finish(*parse_close(payload))
                return

    def _finish(self, code: int, reason: str) -> None:
        if self.closed is None:
            self.closed = ConnectionClosed(code, reason)
        self._closed_event.set()
        try:
            self.writer.close()
        except (ConnectionError, OSError, RuntimeError):
            pass


async def connect(host: str, port: int, path: str, headers: dict | None = None,
                  timeout: float = 5.0) -> WebSocket:
    """Open ``ws://host:port/path``. Raises HandshakeError (with the HTTP status) or OSError."""
    return await asyncio.wait_for(_connect(host, port, path, headers or {}), timeout)


async def _connect(host, port, path, headers) -> WebSocket:
    reader, writer = await asyncio.open_connection(host, port)
    try:
        key = base64.b64encode(os.urandom(16))
        lines = ["GET %s HTTP/1.1" % path, "Host: %s:%d" % (host, port),
                 "Upgrade: websocket", "Connection: Upgrade",
                 "Sec-WebSocket-Key: %s" % key.decode(), "Sec-WebSocket-Version: 13"]
        lines += ["%s: %s" % kv for kv in headers.items()]
        writer.write(("\r\n".join(lines) + "\r\n\r\n").encode("latin-1"))
        await writer.drain()
        head = await reader.readuntil(b"\r\n\r\n")
        status_line, *hdr_lines = head.decode("latin-1").split("\r\n")
        parts = status_line.split(" ", 2)
        status = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
        hdrs = {}
        for line in hdr_lines:
            if ":" in line:
                k, v = line.split(":", 1)
                hdrs[k.strip().lower()] = v.strip()
        if status != 101:
            raise HandshakeError(status, "server answered %s" % status_line)
        if (hdrs.get("upgrade", "").lower() != "websocket"
                or hdrs.get("sec-websocket-accept", "").encode() != accept_key(key)):
            raise HandshakeError(0, "bad upgrade response")
    except BaseException:
        writer.close()
        raise
    return WebSocket(reader, writer)
