#!/usr/bin/env python3
"""BLE tunnel bridge: terminates docs/ble_tunnel_protocol.md on /dev/serial_ble and forwards
the phone's HTTP requests and WebSockets to the Go backend on http://127.0.0.1:4006.

Owns the vendor BLE module's transparent UART (/dev/ttyS3, 115200 8N1 raw, no flow control)
exclusively: it refuses to start while another process has the port open, takes TIOCEXCL and
an exclusive flock, so it never runs together with ble_echo_daemon.py. Python stdlib only.

    python3 ble_bridge.py [--port /dev/serial_ble] [--backend http://127.0.0.1:4006]
                          [--log-level INFO] [--token-ttl 30]
"""
from __future__ import annotations

import argparse
import asyncio
import errno
import fcntl
import hmac
import json
import logging
import os
import secrets
import signal
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ble_backend  # noqa: E402
import ble_mux as mx  # noqa: E402
import ble_streams  # noqa: E402
from ble_echo_daemon import close_port, open_port, port_users  # noqa: E402

log = logging.getLogger("ble_bridge")

DEFAULT_PORT = "/dev/serial_ble"
FALLBACK_PORT = "/dev/ttyS3"        # the container maps ttyS3; /dev/serial_ble via the /dev bind
WRITABLE_LIMIT = 2 * mx.MAX_FRAME   # an HTTP body producer waits above this many queued bytes


class FatalSerialError(Exception):
    pass


class Bridge:
    def __init__(self, fd: int, backend: tuple[str, int], token_ttl: float = 30.0,
                 session_timeout: float = 5.0, hostname: str | None = None):
        self.fd = fd
        self.backend = backend
        self.token_ttl = token_ttl
        self.session_timeout = session_timeout
        self.hostname = hostname or socket.gethostname()
        self.loop = asyncio.get_running_loop()
        self.link = mx.Link()
        self.parser = mx.FrameParser()
        self.session: str | None = None
        self.streams: dict[int, object] = {}
        self.closed_ids: set[int] = set()       # streams that ended this session
        self.hello_task: asyncio.Task | None = None
        self.held: list[mx.Frame] = []          # frames that arrived during the token check
        self.token: tuple[str, float] | None = None
        self.last_rx = 0.0
        self.txbuf = bytearray()
        self.writer_armed = False
        self.kick_pending = False
        self.wake: asyncio.TimerHandle | None = None
        self.waiters: list[asyncio.Future] = []
        self.tasks: set[asyncio.Task] = set()
        self.failed: asyncio.Future = self.loop.create_future()

    # -- helpers for ble_streams -------------------------------------------------------------
    def spawn(self, coro) -> asyncio.Task:
        task = self.loop.create_task(coro)
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)
        return task

    def stream_done(self, sid: int, summary: str) -> None:
        if self.streams.pop(sid, None) is not None:
            self.closed_ids.add(sid)
        log.info("stream %d closed %s", sid, summary)

    async def writable(self, sid: int) -> None:
        while self.link.has_stream(sid) and self.link.queued_bytes(sid) >= WRITABLE_LIMIT:
            fut = self.loop.create_future()
            self.waiters.append(fut)
            await fut

    # -- serial output -----------------------------------------------------------------------
    def kick(self) -> None:
        if not self.kick_pending:
            self.kick_pending = True
            self.loop.call_soon(self._pump)

    def _pump(self) -> None:
        self.kick_pending = False
        if self.wake:
            self.wake.cancel()
            self.wake = None
        while len(self.txbuf) < mx.MAX_FRAME:
            raw, wake_at = self.link.pull()
            if raw is None:
                if wake_at is not None:
                    self.wake = self.loop.call_later(max(0.0, wake_at - self.link.clock()),
                                                     self.kick)
                break
            if log.isEnabledFor(logging.DEBUG):
                log.debug("tx %s", mx.frame_name(mx.decode_frame(raw)))
            self.txbuf += raw
        self._flush()
        waiters, self.waiters = self.waiters, []
        for fut in waiters:
            if not fut.done():
                fut.set_result(None)

    def _flush(self) -> None:
        if self.txbuf:
            try:
                n = os.write(self.fd, self.txbuf)
                del self.txbuf[:n]
            except OSError as exc:
                if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                    self._fatal("serial write failed: %s" % exc)
                    return
        if self.txbuf and not self.writer_armed:
            self.loop.add_writer(self.fd, self._on_writable)
            self.writer_armed = True
        elif not self.txbuf and self.writer_armed:
            self.loop.remove_writer(self.fd)
            self.writer_armed = False

    def _on_writable(self) -> None:
        self._flush()
        if not self.txbuf:
            self.kick()

    def _fatal(self, why: str) -> None:
        log.error("%s", why)
        self.discard("serial error")
        if not self.failed.done():
            self.failed.set_exception(FatalSerialError(why))

    # -- serial input ------------------------------------------------------------------------
    def on_readable(self) -> None:
        try:
            data = os.read(self.fd, 4096)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                return
            self.loop.remove_reader(self.fd)
            self._fatal("serial read failed: %s" % exc)
            return
        for fr in self.parser.feed(data):
            self.handle(fr)

    def handle(self, fr: mx.Frame) -> None:
        if log.isEnabledFor(logging.DEBUG):
            log.debug("rx %s", mx.frame_name(fr))
        if fr.kind == mx.HELLO and fr.stream == 0:
            self.discard("new HELLO")
            self.held = []
            if self.hello_task:
                self.hello_task.cancel()
            self.hello_task = self.spawn(self.on_hello(fr))
            return
        if self.hello_task is not None and not self.hello_task.done():
            self.held.append(fr)
            return
        if self.session is None:
            return                                  # nothing until a valid HELLO
        self.last_rx = time.monotonic()
        if fr.kind == mx.RESET:
            self.discard_streams("RESET from phone")
            self.link.on_peer_reset()
            log.info("RESET from phone")
            return
        verdict = self.link.accept(fr)
        if verdict == mx.DROP:
            return
        if verdict == mx.GAP:
            log.warning("seq gap (got %d, expected %d): RESET", fr.seq, self.link.expected)
            self.discard_streams("seq gap")
            self.link.send_reset()
            self.kick()
            return
        self.dispatch(fr)

    def dispatch(self, fr: mx.Frame) -> None:
        k, sid = fr.kind, fr.stream
        if sid == 0:
            if k == mx.CREDIT:
                self.link.on_credit(fr.payload)
                self.kick()
            elif k == mx.PING:
                self.link.send_control(mx.PONG, fr.payload)
                self.kick()
            elif k == mx.OPEN:
                self._refuse(sid, 4400, "bad stream id")
            return
        if k == mx.OPEN:
            if sid % 2 == 0:
                self._refuse(sid, 4400, "bad stream id")
            elif sid in self.streams:
                self.streams[sid].abort("stream id reused")
                self._refuse(sid, 4409, "stream in use")
            else:
                self.closed_ids.discard(sid)
                stream = ble_streams.open_stream(self, sid, fr.payload)
                if stream is None:
                    self.closed_ids.add(sid)
                else:
                    self.streams[sid] = stream
        elif k in (mx.DATA, mx.END, mx.TEXT_END, mx.CLOSE):
            stream = self.streams.get(sid)
            if stream is not None:
                stream.on_frame(k, fr.payload)
            elif k != mx.CLOSE and sid not in self.closed_ids:
                self._refuse(sid, 4404, "no such stream")

    def _refuse(self, sid: int, code: int, reason: str) -> None:
        self.link.send_control(mx.CLOSE, ble_streams.close_payload(code, reason), sid)
        self.kick()

    # -- session -----------------------------------------------------------------------------
    async def get_token(self, force: bool = False) -> tuple[str, bool]:
        """(token, fresh). Cached for token_ttl seconds."""
        now = time.monotonic()
        if not force and self.token and now - self.token[1] < self.token_ttl:
            return self.token[0], False
        token = await ble_backend.fetch_token(*self.backend)
        self.token = (token, now)
        return token, True

    async def on_hello(self, fr: mx.Frame) -> None:
        window, max_rate = mx.DEFAULT_WINDOW, mx.DEFAULT_RATE
        error = None
        try:
            hello = json.loads(fr.payload)
            if not isinstance(hello, dict) or hello.get("v") != 1:
                raise ValueError
        except ValueError:
            hello, error = None, "bad_hello"
        if hello is not None:
            window, max_rate = mx.hello_params(hello)
            given = hello.get("token")
            try:
                token, fresh = await self.get_token()
                ok = isinstance(given, str) and hmac.compare_digest(token.encode(), given.encode())
                if not ok and not fresh and isinstance(given, str):
                    token, _ = await self.get_token(force=True)     # rotated since we cached
                    ok = hmac.compare_digest(token.encode(), given.encode())
                error = None if ok else "unauthenticated"
            except Exception as exc:                # noqa: BLE001 - any failure is "unavailable"
                log.warning("token fetch failed: %s", exc or type(exc).__name__)
                error = "backend_unavailable"
        self.hello_task = None                      # verdict in: stop holding frames
        self.link.start(window, max_rate, fr.seq)
        held, self.held = self.held, []
        if error:
            self.link.send_control(mx.WELCOME, ble_streams.dumps({"v": 1, "error": error}))
            self.kick()
            log.warning("HELLO refused: %s", error)
            return
        self.session = secrets.token_hex(4)
        self.last_rx = time.monotonic()
        self.link.send_control(mx.WELCOME, ble_streams.dumps(
            {"v": 1, "host": self.hostname, "max_payload": mx.MAX_PAYLOAD,
             "session": self.session}))
        self.kick()
        client = hello.get("client") if isinstance(hello.get("client"), str) else "?"
        log.info("session %s: client %s, window %d, max_rate %d", self.session, client[:64],
                 window, max_rate)
        for f in held:
            self.handle(f)

    def discard_streams(self, why: str) -> None:
        streams, self.streams = self.streams, {}
        for stream in streams.values():
            stream.abort(why)
        self.closed_ids.clear()

    def discard(self, why: str) -> None:
        """End the session: close every backend connection, wait for a new HELLO."""
        self.discard_streams(why)
        if self.session is not None:
            log.info("session %s ended: %s", self.session, why)
        self.session = None
        self.link.on_peer_reset()

    async def watchdog(self) -> None:
        period = min(0.5, self.session_timeout / 4)
        while True:
            await asyncio.sleep(period)
            if self.session and time.monotonic() - self.last_rx > self.session_timeout:
                self.discard("no frame for %.1f s" % self.session_timeout)

    async def run(self, stop: asyncio.Event) -> None:
        self.loop.add_reader(self.fd, self.on_readable)
        dog = self.spawn(self.watchdog())
        stopper = self.spawn(stop.wait())
        try:
            await asyncio.wait([stopper, self.failed], return_when=asyncio.FIRST_COMPLETED)
        finally:
            self.loop.remove_reader(self.fd)
            if self.writer_armed:
                self.loop.remove_writer(self.fd)
            self.discard("shutdown")
            dog.cancel()
            pending = [t for t in self.tasks if not t.done()]
            if pending:
                await asyncio.wait(pending, timeout=1.0)
            for t in pending:
                t.cancel()
        if self.failed.done():
            self.failed.result()                    # raises FatalSerialError


def lock_port(fd: int, path: str) -> None:
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        raise SystemExit("%s is locked by another bridge or test daemon" % path) from None


async def amain(args) -> int:
    port = args.port or (DEFAULT_PORT if os.path.exists(DEFAULT_PORT) else FALLBACK_PORT)
    users = port_users(port)
    if users and not args.force:
        log.error("%s is open in: %s. Stop that first (ble_echo_daemon?), or --force.",
                  port, ", ".join(users))
        return 2
    try:
        fd = open_port(port)
    except OSError as exc:
        log.error("cannot open %s: %s", port, exc)
        return 1
    try:
        lock_port(fd, port)
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, stop.set)
        bridge = Bridge(fd, ble_backend.parse_base(args.backend), args.token_ttl,
                        args.session_timeout)
        log.info("ble_bridge on %s -> %s (host %s)", port, args.backend, bridge.hostname)
        try:
            await bridge.run(stop)
        except FatalSerialError:
            return 1
        log.info("stopped")
        return 0
    finally:
        close_port(fd)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default=None,
                    help="serial device (default %s, else %s)" % (DEFAULT_PORT, FALLBACK_PORT))
    ap.add_argument("--backend", default="http://127.0.0.1:4006", help="Go backend base URL")
    ap.add_argument("--log-level", default="INFO",
                    choices=("DEBUG", "INFO", "WARNING", "ERROR"),
                    help="DEBUG logs every frame (default INFO: one line per stream)")
    ap.add_argument("--token-ttl", type=float, default=30.0,
                    help="seconds to cache the BLE token from the backend (default 30)")
    ap.add_argument("--session-timeout", type=float, default=5.0,
                    help="seconds without a frame before the session is discarded (default 5)")
    ap.add_argument("--force", action="store_true",
                    help="start even if another process has the port open")
    args = ap.parse_args(argv)
    logging.basicConfig(level=getattr(logging, args.log_level),
                        format="%(asctime)s %(levelname)s %(message)s", stream=sys.stdout)
    try:
        return asyncio.run(amain(args))
    except SystemExit as exc:
        log.error("%s", exc)
        return 2


if __name__ == "__main__":
    sys.exit(main())
