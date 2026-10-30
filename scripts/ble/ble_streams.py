"""Tunnel streams of the BLE bridge (docs/ble_tunnel_protocol.md, section 3): one HTTP request
or one WebSocket per stream, forwarded to the local backend.

A stream talks to the bridge through ``tx`` (see ``Bridge`` in ble_bridge.py): ``tx.link`` (the
ble_mux.Link outgoing queues), ``tx.kick()``, ``await tx.writable(sid)``, ``tx.backend``
(host, port), ``tx.spawn(coro)`` and ``tx.stream_done(sid, summary)``.
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
import zlib
from urllib.parse import unquote

import ble_backend
import ble_mux as mx
import ble_ws

log = logging.getLogger("ble_bridge")

METHODS = ("GET", "POST", "PUT", "PATCH", "DELETE")
REQUEST_HEADERS = ("content-type", "accept", "if-none-match", "range")
RESPONSE_HEADERS = ("content-type", "etag", "cache-control", "content-disposition",
                    "last-modified")
HEAD_DROP_ORDER = ("last-modified", "content-disposition", "cache-control", "etag",
                   "content-type")
GZIP_TYPES = ("text/", "application/json", "application/javascript", "application/yaml",
              "application/x-yaml", "image/svg+xml")
GZIP_MIN = 256
BRIDGE_HEADERS = {"X-Mower-Transport": "ble", "Accept-Encoding": "identity"}
ERR_BACKEND = 4499          # backend failed after HEAD was sent (section 6)


def dumps(obj) -> bytes:
    return json.dumps(obj, separators=(",", ":")).encode()


def close_payload(code: int, reason: str = "") -> bytes:
    return dumps({"code": code, "reason": reason})


def path_allowed(path, http: bool) -> bool:
    """``/api/...`` (or ``/web-build.json`` for HTTP); printable ASCII only, no ``..``."""
    if not isinstance(path, str) or not path.startswith("/"):
        return False
    if any(not 0x20 < ord(c) < 0x7F for c in path) or "#" in path:
        return False
    p = path.split("?", 1)[0]
    if ".." in unquote(p).split("/"):
        return False
    return p.startswith("/api/") or (http and p == "/web-build.json")


def head_payload(status: int, headers: dict | None = None) -> bytes:
    """HEAD JSON; drops headers (least useful first) until it fits one frame."""
    if headers is None:
        return dumps({"status": status})
    headers = dict(headers)
    raw = dumps({"status": status, "headers": headers})
    for name in HEAD_DROP_ORDER:
        if len(raw) <= mx.MAX_PAYLOAD:
            break
        headers.pop(name, None)
        raw = dumps({"status": status, "headers": headers})
    return raw if len(raw) <= mx.MAX_PAYLOAD else dumps({"status": status, "headers": {}})


def wants_gzip(content_type: str) -> bool:
    return content_type.lower().startswith(GZIP_TYPES)


class _Stream:
    kind = "?"

    def __init__(self, tx, sid: int, path: str):
        self.tx, self.sid, self.path = tx, sid, path
        self.task: asyncio.Task | None = None
        self.t0 = time.monotonic()
        self.done = False

    def _finish(self, summary: str) -> None:
        if not self.done:
            self.done = True
            self.tx.stream_done(self.sid, "%s %s: %s, %.2f s" % (
                self.kind, self.path, summary, time.monotonic() - self.t0))

    def _reply_close(self, payload: bytes = b"") -> None:
        self.tx.link.send_close(self.sid, payload)
        self.tx.kick()


class HttpStream(_Stream):
    kind = "http"

    def __init__(self, tx, sid, method, path, headers):
        super().__init__(tx, sid, path)
        self.method = method
        self.headers = {k: v for k, v in headers.items()
                        if k in REQUEST_HEADERS and isinstance(v, str)
                        and not any(c in v for c in "\r\n\0")}
        self.body = mx.Reassembler()
        self.status = 0
        self.sent = 0

    def on_frame(self, kind: int, payload: bytes) -> None:
        if kind == mx.CLOSE:
            self.abort("aborted by phone")
        elif kind in (mx.DATA, mx.END, mx.TEXT_END) and self.task is None:
            try:
                msg = self.body.feed(kind, payload)
            except ValueError:
                self.tx.link.send_close(self.sid, close_payload(4400, "body too large"))
                self.tx.kick()
                self._finish("request body too large")
                return
            if msg is not None:
                self.task = self.tx.spawn(self.run(msg[1]))

    def abort(self, why: str) -> None:
        self.tx.link.drop_stream(self.sid)
        if self.task:
            self.task.cancel()
        self._finish(why)

    async def run(self, body: bytes) -> None:
        host, port = self.tx.backend
        headers = dict(BRIDGE_HEADERS, **self.headers)
        try:
            resp = await ble_backend.request(host, port, self.method, self.path, headers, body)
        except (OSError, ble_backend.BackendError, asyncio.TimeoutError) as exc:
            self.tx.link.send_frame(self.sid, mx.HEAD, head_payload(502))
            self._reply_close()
            self._finish("backend failed: %s" % (exc or type(exc).__name__))
            return
        try:
            await self._forward(resp)
            self._reply_close()
            self._finish("%d, %d B on the wire" % (self.status, self.sent))
        except (OSError, ble_backend.BackendError, asyncio.IncompleteReadError,
                asyncio.LimitOverrunError) as exc:
            self._reply_close(close_payload(ERR_BACKEND, "backend failed"))
            self._finish("backend failed mid-body: %s" % exc)
        finally:
            resp.close()

    async def _forward(self, resp) -> None:
        self.status = resp.status
        out = {k: resp.headers[k] for k in RESPONSE_HEADERS if k in resp.headers}
        chunks = resp.chunks()
        first = bytearray()
        backend_enc = resp.headers.get("content-encoding", "").strip().lower()
        gz = None
        if backend_enc and backend_enc != "identity":
            out["content-encoding"] = backend_enc         # already encoded: pass through
        elif wants_gzip(resp.headers.get("content-type", "")):
            if resp.mode == "length":
                big = resp.length >= GZIP_MIN
            else:                                         # unknown length: peek 256 bytes
                async for c in chunks:
                    first += c
                    if len(first) >= GZIP_MIN:
                        break
                big = len(first) >= GZIP_MIN
            if big:
                gz = zlib.compressobj(6, zlib.DEFLATED, 31)
                out["content-encoding"] = "gzip"
        self.tx.link.send_frame(self.sid, mx.HEAD, head_payload(resp.status, out))
        self.tx.kick()
        pending = bytearray()

        async def emit(data: bytes):
            pending.extend(gz.compress(data) + gz.flush(zlib.Z_SYNC_FLUSH) if gz else data)
            while len(pending) > mx.MAX_PAYLOAD:
                await self._send(mx.DATA, bytes(pending[:mx.MAX_PAYLOAD]))
                del pending[:mx.MAX_PAYLOAD]

        if first:
            await emit(bytes(first))
        async for c in chunks:
            await emit(c)
        if gz:
            pending.extend(gz.flush())
        while len(pending) > mx.MAX_PAYLOAD:
            await self._send(mx.DATA, bytes(pending[:mx.MAX_PAYLOAD]))
            del pending[:mx.MAX_PAYLOAD]
        await self._send(mx.END, bytes(pending))

    async def _send(self, kind: int, payload: bytes) -> None:
        await self.tx.writable(self.sid)
        if not self.tx.link.send_frame(self.sid, kind, payload):
            raise asyncio.CancelledError()            # stream dropped (abort / reset)
        self.sent += len(payload) + mx.OVERHEAD
        self.tx.kick()


class WsStream(_Stream):
    kind = "ws"

    def __init__(self, tx, sid, path, prio, lossy):
        super().__init__(tx, sid, path)
        self.prio, self.lossy = prio, lossy
        self.inbox = mx.Reassembler()
        self.to_backend: asyncio.Queue = asyncio.Queue()
        self.ws: ble_ws.WebSocket | None = None
        self.closing = False
        self.rx_msgs = self.tx_msgs = 0
        self.task = tx.spawn(self.run())

    def on_frame(self, kind: int, payload: bytes) -> None:
        if kind == mx.CLOSE:
            code, reason = 1000, ""
            try:
                obj = json.loads(payload) if payload else {}
                code = obj.get("code", 1000) if isinstance(obj.get("code"), int) else 1000
                reason = obj.get("reason", "") if isinstance(obj.get("reason"), str) else ""
            except (ValueError, AttributeError):
                pass
            if not (code in (1000, 1001, 1002, 1003) or 1007 <= code <= 1014
                    or 3000 <= code <= 4999):
                code = 1000
            self.abort("closed by phone %d" % code, code, reason)
        elif kind in (mx.DATA, mx.END, mx.TEXT_END):
            try:
                msg = self.inbox.feed(kind, payload)
            except ValueError:
                self.abort("message too large", 1009, "message too large", reply=True)
                return
            if msg is not None:
                self.to_backend.put_nowait(msg)

    def abort(self, why: str, code: int = 1001, reason: str = "", reply: bool = False) -> None:
        """End the stream now; the backend socket gets ``code``. ``reply``: also tell the
        phone (tunnel-side errors); a CLOSE from the phone or a session discard is not
        answered."""
        if self.closing:
            return
        self.closing = True
        if reply:
            self.tx.link.drop_stream(self.sid)
            self.tx.link.send_control(mx.CLOSE, close_payload(code, reason), self.sid)
            self.tx.kick()
        else:
            self.tx.link.drop_stream(self.sid)
        self._finish(why)
        if self.ws is not None:
            self.tx.spawn(self._close_backend(code, reason))
        elif self.task:
            self.task.cancel()

    async def _close_backend(self, code, reason):
        try:
            await self.ws.close(code, reason)
        finally:
            if self.task:
                self.task.cancel()

    async def run(self) -> None:
        host, port = self.tx.backend
        try:
            ws = await ble_ws.connect(host, port, self.path, {"X-Mower-Transport": "ble"})
        except ble_ws.HandshakeError as exc:
            self._fail(exc.status or 502, "backend refused: %s" % exc)
            return
        except (OSError, asyncio.TimeoutError, asyncio.IncompleteReadError,
                asyncio.LimitOverrunError) as exc:
            self._fail(502, "backend failed: %s" % (exc or type(exc).__name__))
            return
        self.ws = ws
        if self.closing:                    # phone closed while we were connecting
            await ws.close(1001, "")
            return
        self.tx.link.send_frame(self.sid, mx.HEAD, head_payload(101))
        self.tx.kick()
        sender = self.tx.spawn(self._pump_to_backend(ws))
        try:
            while True:
                text, data = await ws.recv()
                self.rx_msgs += 1
                if self.tx.link.send_message(self.sid, data, text):
                    self.tx.kick()
        except ble_ws.ConnectionClosed as exc:
            if not self.closing:
                self.closing = True
                self._reply_close(close_payload(exc.code, exc.reason))
                self._finish("backend closed %d, %d msgs in, %d out"
                             % (exc.code, self.rx_msgs, self.tx_msgs))
        finally:
            sender.cancel()

    async def _pump_to_backend(self, ws) -> None:
        try:
            while True:
                text, data = await self.to_backend.get()
                await ws.send(data, text)
                self.tx_msgs += 1
        except (ble_ws.ConnectionClosed, OSError):
            pass                            # the receive loop reports the close

    def _fail(self, status: int, why: str) -> None:
        if self.closing:
            return
        self.closing = True
        self.tx.link.send_frame(self.sid, mx.HEAD, head_payload(status))
        self._reply_close()
        self._finish(why)


def open_stream(tx, sid: int, payload: bytes):
    """Handle OPEN. Returns the new stream, or None when it was answered and closed at once."""
    def refuse(code, reason):
        tx.link.send_control(mx.CLOSE, close_payload(code, reason), sid)
        tx.kick()
        log.info("stream %d refused: %d %s", sid, code, reason)

    try:
        req = json.loads(payload)
        if not isinstance(req, dict):
            raise ValueError
    except ValueError:
        refuse(4400, "bad open")
        return None
    t, path = req.get("t"), req.get("path")
    if t == "http":
        method, headers = req.get("method"), req.get("headers") or {}
        if method not in METHODS or not isinstance(headers, dict) or not isinstance(path, str):
            refuse(4400, "bad open")
            return None
        if not path_allowed(path, http=True):
            tx.link.open_stream(sid)
            tx.link.send_frame(sid, mx.HEAD, head_payload(403))
            tx.link.send_close(sid)
            tx.kick()
            log.info("stream %d http %s %s: 403", sid, method, path)
            return None
        tx.link.open_stream(sid)
        log.info("stream %d open http %s %s", sid, method, path)
        return HttpStream(tx, sid, method, path, {str(k).lower(): v for k, v in headers.items()})
    if t == "ws":
        if not isinstance(path, str):
            refuse(4400, "bad open")
            return None
        if not path_allowed(path, http=False):
            tx.link.open_stream(sid)
            tx.link.send_frame(sid, mx.HEAD, head_payload(403))
            tx.link.send_close(sid)
            tx.kick()
            log.info("stream %d ws %s: 403", sid, path)
            return None
        prio, lossy = req.get("prio") is True, req.get("lossy") is True
        tx.link.open_stream(sid, prio=prio, lossy=lossy)
        log.info("stream %d open ws %s%s%s", sid, path, " prio" if prio else "",
                 " lossy" if lossy else "")
        return WsStream(tx, sid, path, prio, lossy)
    refuse(4400, "bad open")
    return None
