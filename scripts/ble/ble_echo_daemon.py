#!/usr/bin/env python3
"""Phase-0 BLE link echo daemon for the mower (docs/DESIGN_ble_transport_2026-10.md, section 5).

Owns /dev/serial_ble (the vendor BLE module's transparent UART, /dev/ttyS3, 115200 8N1, no
flow control) and answers the framed test protocol in ble_frame.py:

    ECHO        -> ECHO_REPLY with the same payload
    SINK        -> counted and dropped; STATS about once a second while SINK traffic arrives
    SOURCE      -> SOURCE_DATA frames at the UART's full rate (or a requested rate), SOURCE_END
    PING        -> PONG with the sender's timestamp, at once
    STATS_REQ   -> STATS;  RESET -> zero counters and tx seq, then STATS;  SOURCE_STOP

Python stdlib only, no ROS, sends nothing to the robot. Nothing else may use the port while
it runs: stop the stack (``docker stop mower_jazzy``) and run it on the host, or run it inside
the container with ``docker exec`` if no node there opens the port. It refuses to start when
another process holds the port open (``--force`` skips that check) and takes TIOCEXCL.

    python3 ble_echo_daemon.py [--port /dev/serial_ble] [--stats] [--quiet-garbage]
"""
import argparse
import errno
import fcntl
import os
import select
import signal
import sys
import termios
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ble_frame as bf  # noqa: E402

DEFAULT_PORT = "/dev/serial_ble"
FALLBACK_PORT = "/dev/ttyS3"     # the container maps ttyS3, not the udev symlink
TX_HIGH_WATER = 2048             # refill SOURCE frames below this many queued bytes
SINK_STATS_PERIOD = 1.0


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
    except OSError:
        pass  # PTYs in tests may refuse; not fatal
    attrs = termios.tcgetattr(fd)
    attrs[0] = termios.IGNBRK                                  # iflag: raw, no XON/XOFF
    attrs[1] = 0                                               # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL    # 8N1, no RTS/CTS
    attrs[3] = 0                                               # lflag: raw
    attrs[4] = attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def close_port(fd):
    try:
        fcntl.ioctl(fd, termios.TIOCNXCL)   # TIOCEXCL can outlive us while the tty stays open
    except OSError:
        pass
    os.close(fd)


def port_users(path):
    """PIDs (other than ours) with ``path`` open, as far as /proc lets us see."""
    target = os.path.realpath(path)
    users = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        if int(pid) == os.getpid():
            continue
        try:
            fds = os.listdir("/proc/%s/fd" % pid)
            for fd in fds:
                if os.path.realpath("/proc/%s/fd/%s" % (pid, fd)) == target:
                    with open("/proc/%s/comm" % pid) as f:
                        users.append("%s (%s)" % (pid, f.read().strip()))
                    break
        except OSError:
            continue
    return users


class Daemon:
    def __init__(self, fd, args):
        self.fd = fd
        self.args = args
        self.parser = bf.Parser()
        self.rx_seq = bf.SeqTracker()
        self.txbuf = bytearray()
        self.source = None            # dict while a SOURCE runs
        self.last_sink_stats = 0.0
        self.sink_dirty = False
        self.reset()

    def reset(self):
        self.t0 = time.monotonic()
        self.tx_seq = 0
        self.rx_bytes = self.tx_frames = self.tx_bytes = 0
        self.sink_frames = self.sink_bytes = 0
        self.rx_frames_by_type = {}
        self.parser.crc_errors = self.parser.garbage_bytes = self.parser.frames = 0
        self.parser.bad_lengths = 0
        self.rx_seq.reset()

    def stats(self):
        return {"rx_bytes": self.rx_bytes, "rx_frames": self.parser.frames,
                "crc_errors": self.parser.crc_errors + self.parser.bad_lengths,
                "garbage_bytes": self.parser.garbage_bytes, "seq_gaps": self.rx_seq.gaps,
                "sink_frames": self.sink_frames, "sink_bytes": self.sink_bytes,
                "tx_frames": self.tx_frames, "tx_bytes": self.tx_bytes,
                "uptime_ms": int((time.monotonic() - self.t0) * 1000)}

    def send(self, ftype, payload=b""):
        raw = bf.encode(self.tx_seq, ftype, payload)
        self.tx_seq = (self.tx_seq + 1) & 0xFFFFFFFF
        self.tx_frames += 1
        self.tx_bytes += len(raw)
        self.txbuf += raw

    def log(self, msg):
        print("%s %s" % (time.strftime("%H:%M:%S"), msg), flush=True)

    # -- request handling ------------------------------------------------------------------
    def handle(self, fr):
        self.rx_seq.update(fr.seq)
        name = bf.TYPE_NAMES.get(fr.type, "0x%02x" % fr.type)
        self.rx_frames_by_type[name] = self.rx_frames_by_type.get(name, 0) + 1
        if fr.type == bf.PING:
            self.send(bf.PONG, fr.payload)
        elif fr.type == bf.ECHO:
            self.send(bf.ECHO_REPLY, fr.payload)
        elif fr.type == bf.SINK:
            self.sink_frames += 1
            self.sink_bytes += len(fr.payload)
            self.sink_dirty = True
        elif fr.type == bf.STATS_REQ:
            self.send(bf.STATS, bf.pack_stats(self.stats()))
        elif fr.type == bf.RESET:
            self.source = None
            del self.txbuf[:]
            self.reset()
            self.rx_seq.update(fr.seq)       # the RESET frame itself starts the new count
            self.send(bf.STATS, bf.pack_stats(self.stats()))
            self.log("RESET")
        elif fr.type == bf.SOURCE:
            if len(fr.payload) < bf.SOURCE_REQ.size:
                self.log("SOURCE request too short (%d bytes), ignored" % len(fr.payload))
                return
            max_bytes, dur_ms, chunk, rate = bf.SOURCE_REQ.unpack_from(fr.payload)
            chunk = max(1, min(chunk, bf.MAX_PAYLOAD))
            now = time.monotonic()
            self.source = {"max": max_bytes, "end": now + dur_ms / 1000.0, "chunk": chunk,
                           "rate": rate, "t0": now, "wire": 0, "frames": 0, "bytes": 0}
            self.log("SOURCE %d ms, %d B/frame, rate %s, max %s" %
                     (dur_ms, chunk, rate or "full", max_bytes or "none"))
        elif fr.type == bf.SOURCE_STOP:
            self.finish_source()
        else:
            self.log("unknown frame type 0x%02x (%d bytes), ignored" % (fr.type, len(fr.payload)))

    def finish_source(self):
        s, self.source = self.source, None
        if s is not None:
            self.send(bf.SOURCE_END, bf.SOURCE_END_FMT.pack(s["frames"], s["bytes"]))
            self.log("SOURCE done: %d frames, %d payload bytes" % (s["frames"], s["bytes"]))

    def refill_source(self, now):
        s = self.source
        if s is None:
            return
        if now >= s["end"] or (s["max"] and s["bytes"] >= s["max"]):
            self.finish_source()
            return
        while len(self.txbuf) < TX_HIGH_WATER:
            if s["rate"] and s["wire"] >= s["rate"] * (now - s["t0"]):
                return
            n = s["chunk"] if not s["max"] else min(s["chunk"], s["max"] - s["bytes"])
            if n <= 0:
                return
            before = self.tx_bytes
            self.send(bf.SOURCE_DATA, bf.pattern(self.tx_seq, n))
            s["wire"] += self.tx_bytes - before
            s["frames"] += 1
            s["bytes"] += n

    # -- I/O ---------------------------------------------------------------------------------
    def flush_tx(self):
        try:
            n = os.write(self.fd, self.txbuf[:4096])
            del self.txbuf[:n]
        except OSError as exc:
            if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                raise

    def report_garbage(self):
        if self.parser.garbage_log and not self.args.quiet_garbage:
            self.log("unframed bytes (%d total so far):\n%s" %
                     (self.parser.garbage_bytes, bf.hexdump(self.parser.garbage_log)))
        del self.parser.garbage_log[:]

    def print_stats(self):
        s = self.stats()
        self.log("rx %d B / %d fr  crc_err %d  garbage %d B  seq_gaps %d  sink %d fr %d B  "
                 "tx %d B / %d fr  txq %d B  by type %s%s" %
                 (s["rx_bytes"], s["rx_frames"], s["crc_errors"], s["garbage_bytes"],
                  s["seq_gaps"], s["sink_frames"], s["sink_bytes"], s["tx_bytes"],
                  s["tx_frames"], len(self.txbuf), self.rx_frames_by_type,
                  "  SOURCE running" if self.source else ""))

    def run(self):
        stop = []
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.append(1))
        next_stats = time.monotonic() + self.args.stats_period
        while not stop:
            now = time.monotonic()
            self.refill_source(now)
            if self.sink_dirty and now - self.last_sink_stats >= SINK_STATS_PERIOD:
                self.send(bf.STATS, bf.pack_stats(self.stats()))
                self.last_sink_stats, self.sink_dirty = now, False
            timeout = 0.005 if self.source else 0.2
            try:
                r, w, _ = select.select([self.fd], [self.fd] if self.txbuf else [], [], timeout)
            except InterruptedError:
                continue
            if r:
                data = os.read(self.fd, 4096)
                if data:
                    self.rx_bytes += len(data)
                    for fr in self.parser.iter_feed(data):
                        self.handle(fr)
                    if self.parser.garbage_log:
                        self.report_garbage()
            if w:
                self.flush_tx()
            if self.args.stats and now >= next_stats:
                self.print_stats()
                next_stats = now + self.args.stats_period
        self.print_stats()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("--port", default=None,
                    help="serial device (default %s, else %s)" % (DEFAULT_PORT, FALLBACK_PORT))
    ap.add_argument("--stats", action="store_true", help="print counters periodically")
    ap.add_argument("--stats-period", type=float, default=5.0, help="seconds (default 5)")
    ap.add_argument("--quiet-garbage", action="store_true",
                    help="do not hexdump bytes that are not frames (module chatter, line noise)")
    ap.add_argument("--force", action="store_true",
                    help="start even if another process has the port open")
    args = ap.parse_args()

    port = args.port or (DEFAULT_PORT if os.path.exists(DEFAULT_PORT) else FALLBACK_PORT)
    users = port_users(port)
    if users and not args.force:
        print("%s is open in: %s\nStop that first (docker stop mower_jazzy), or --force."
              % (port, ", ".join(users)), file=sys.stderr)
        return 2
    try:
        fd = open_port(port)
    except OSError as exc:
        print("cannot open %s: %s" % (port, exc), file=sys.stderr)
        return 1
    print("ble_echo_daemon on %s (115200 8N1 raw). Ctrl-C to stop." % port, flush=True)
    try:
        Daemon(fd, args).run()
    finally:
        close_port(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
