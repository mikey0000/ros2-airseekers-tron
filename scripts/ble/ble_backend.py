"""Tiny HTTP/1.1 client for the BLE bridge, on asyncio streams (stdlib only).

One connection per request (``Connection: close``). The response body is read incrementally
(Content-Length, chunked, or until EOF), so large files stream through without being held in
memory.
"""
from __future__ import annotations

import asyncio
import json
from typing import AsyncIterator
from urllib.parse import urlsplit

HEAD_LIMIT = 64 << 10


class BackendError(Exception):
    pass


def parse_base(url: str) -> tuple[str, int]:
    u = urlsplit(url)
    if u.scheme != "http" or not u.hostname:
        raise ValueError("backend must be http://host:port, got %r" % url)
    return u.hostname, u.port or 80


class Response:
    def __init__(self, status: int, headers: dict[str, str], reader: asyncio.StreamReader,
                 writer: asyncio.StreamWriter, mode: str, length: int):
        self.status, self.headers = status, headers
        self.reader, self.writer = reader, writer
        self.mode, self.length = mode, length   # mode: "none", "length", "chunked", "eof"

    async def chunks(self, size: int = 4096) -> AsyncIterator[bytes]:
        r = self.reader
        if self.mode == "length":
            left = self.length
            while left > 0:
                data = await r.read(min(size, left))
                if not data:
                    raise BackendError("body cut short")
                left -= len(data)
                yield data
        elif self.mode == "eof":
            while True:
                data = await r.read(size)
                if not data:
                    return
                yield data
        elif self.mode == "chunked":
            while True:
                line = await r.readuntil(b"\r\n")
                try:
                    n = int(line.split(b";", 1)[0].strip(), 16)
                except ValueError:
                    raise BackendError("bad chunk size %r" % line) from None
                if n == 0:
                    while (await r.readuntil(b"\r\n")) != b"\r\n":   # trailers
                        pass
                    return
                while n > 0:
                    data = await r.read(min(size, n))
                    if not data:
                        raise BackendError("chunk cut short")
                    n -= len(data)
                    yield data
                if await r.readexactly(2) != b"\r\n":
                    raise BackendError("missing chunk CRLF")

    def close(self) -> None:
        try:
            self.writer.close()
        except (OSError, RuntimeError):
            pass


async def _read_head(reader) -> tuple[int, dict[str, str]]:
    raw = await reader.readuntil(b"\r\n\r\n")
    if len(raw) > HEAD_LIMIT:
        raise BackendError("response head too large")
    status_line, *lines = raw.decode("latin-1").split("\r\n")
    parts = status_line.split(" ", 2)
    if len(parts) < 2 or not parts[0].startswith("HTTP/1.") or not parts[1].isdigit():
        raise BackendError("bad status line %r" % status_line)
    headers: dict[str, str] = {}
    for line in lines:
        if ":" in line:
            k, v = line.split(":", 1)
            k, v = k.strip().lower(), v.strip()
            headers[k] = headers[k] + ", " + v if k in headers else v
    return int(parts[1]), headers


async def request(host: str, port: int, method: str, path: str, headers: dict[str, str],
                  body: bytes = b"", timeout: float = 30.0) -> Response:
    """Send one request and read the response head. Raises OSError, BackendError, or
    asyncio.TimeoutError before the head is in; the caller closes the Response."""
    reader, writer = await asyncio.wait_for(asyncio.open_connection(host, port), 5.0)
    try:
        lines = ["%s %s HTTP/1.1" % (method, path), "Host: %s:%d" % (host, port),
                 "Connection: close"]
        lines += ["%s: %s" % kv for kv in headers.items()]
        if body or method in ("POST", "PUT", "PATCH"):
            lines.append("Content-Length: %d" % len(body))
        writer.write(("\r\n".join(lines) + "\r\n\r\n").encode("latin-1") + body)
        await writer.drain()
        while True:
            status, rh = await asyncio.wait_for(_read_head(reader), timeout)
            if not 100 <= status < 200:
                break                                   # skip 100 Continue and friends
        if method == "HEAD" or status in (204, 304):
            mode, length = "none", 0
        elif "chunked" in rh.get("transfer-encoding", "").lower():
            mode, length = "chunked", 0
        elif rh.get("content-length", "").strip().isdigit():
            mode, length = "length", int(rh["content-length"])
        else:
            mode, length = "eof", 0
        return Response(status, rh, reader, writer, mode, length)
    except (asyncio.IncompleteReadError, asyncio.LimitOverrunError) as exc:
        writer.close()
        raise BackendError("bad response: %s" % exc) from None
    except BaseException:
        writer.close()
        raise


async def fetch_token(host: str, port: int, timeout: float = 3.0) -> str:
    """``GET /api/ble/token`` -> ``{"token": "..."}``."""
    async def go():
        resp = await request(host, port, "GET", "/api/ble/token",
                             {"Accept": "application/json", "Accept-Encoding": "identity"},
                             timeout=timeout)
        try:
            body = b"".join([c async for c in resp.chunks()])
        finally:
            resp.close()
        if resp.status != 200:
            raise BackendError("token endpoint answered %d" % resp.status)
        token = json.loads(body).get("token")
        if not isinstance(token, str) or not token:
            raise BackendError("no token in reply")
        return token
    return await asyncio.wait_for(go(), timeout)
