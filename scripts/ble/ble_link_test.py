#!/usr/bin/env python3
"""Phase-0 BLE link measurements from a Linux laptop (docs/DESIGN_ble_transport_2026-10.md, sec. 5).

Needs ble_echo_daemon.py running on the mower. Connects with BlueZ (dbus-python + PyGObject),
subscribes to c1db0002 (mower -> phone), writes to c1db0004 (phone -> mower) and runs:

  info       GATT flags, MTU, notify mode, and a read of c1db0003
  sweep      ECHO at write sizes 20/100/180/244/497 B: largest accepted write, bytes lost
  ping       PING at 10 Hz for 30 s: round-trip min / median / p95 / max
  sink       phone->mower SINK for 60 s: bytes/s and loss
  source     mower->phone SOURCE for 60 s: bytes/s, CRC errors, seq gaps
  reconnect  disconnect / reconnect 5 times, timed

    ble_link_test.py [--address <mower-ble-address> | --name-prefix AIRSEEKERS_] --out results.md
    ble_link_test.py --serial /dev/pts/N --tests sweep,ping,sink,source   # bench, no BLE
"""
import argparse
import math
import os
import statistics
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ble_frame as bf  # noqa: E402

ALL_TESTS = "info,sweep,ping,sink,source,reconnect"
NOTIF_BUCKETS = (("<=20", 20), ("21-100", 100), ("101-244", 244), ("245+", 1 << 30))


class SerialTransport:
    """Bench transport: the daemon on a local PTY (socat) instead of the BLE module."""
    notify_mode, mtu, inflight = "serial (bench)", None, 0

    def __init__(self, path, on_rx):
        from ble_echo_daemon import open_port
        import select
        self.select, self.path, self.on_rx = select.select, path, on_rx
        self.fd = open_port(path)
        self.write_errors = []

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while True:
            r, _, _ = self.select([self.fd], [], [], max(0.0, min(0.01, end - time.monotonic())))
            if r:
                data = os.read(self.fd, 65536)
                if data:
                    self.on_rx(data)
            if time.monotonic() >= end:
                return

    def write(self, data, without_response=False):
        view = memoryview(bytes(data))
        while view:
            try:
                view = view[os.write(self.fd, view):]
            except BlockingIOError:
                self.pump(0.001)

    def can_write_without_response(self):
        return False

    def describe(self):
        return {"device": self.path, "mtu": None, "notify": self.notify_mode, "flags": {}}

    def close(self):
        from ble_echo_daemon import close_port
        close_port(self.fd)


class Link:
    def __init__(self, transport_factory):
        self.parser = bf.Parser()
        self.rx_seq = bf.SeqTracker()
        self.queue = deque(maxlen=1000)
        self.tx_seq = 0
        self.pongs = []
        self.src = {}
        self.notifs = [0] * len(NOTIF_BUCKETS)   # notifications (reads, in bench mode) by size
        self.tr = transport_factory(self._rx)

    def _rx(self, data):
        now = time.monotonic_ns()
        self.notifs[next(i for i, (_, hi) in enumerate(NOTIF_BUCKETS) if len(data) <= hi)] += 1
        for fr in self.parser.feed(data):
            self.rx_seq.update(fr.seq)
            if fr.type == bf.SOURCE_DATA:
                s = self.src
                s["frames"] = s.get("frames", 0) + 1
                s["bytes"] = s.get("bytes", 0) + len(fr.payload)
                s.setdefault("t_first", now)
                s["t_last"] = now
                if fr.payload != bf.pattern(fr.seq, len(fr.payload)):
                    s["bad"] = s.get("bad", 0) + 1
            elif fr.type == bf.PONG and len(fr.payload) == 8:
                self.pongs.append((now - bf.PING_FMT.unpack(fr.payload)[0]) / 1e6)
            else:
                self.queue.append(fr)

    def pump(self, seconds=0.0):
        self.tr.pump(seconds)

    def send(self, ftype, payload=b"", wwr=False, wait=True, timeout=10.0):
        raw = bf.encode(self.tx_seq, ftype, payload)
        self.tx_seq = (self.tx_seq + 1) & 0xFFFFFFFF
        self.tr.write(raw, wwr)
        end = time.monotonic() + timeout
        while wait and self.tr.inflight and time.monotonic() < end:
            self.pump(0)
        return len(raw)

    def wait_frame(self, ftype, timeout, pred=lambda fr: True):
        end = time.monotonic() + timeout
        while True:
            for fr in list(self.queue):
                if fr.type == ftype and pred(fr):
                    self.queue.remove(fr)
                    return fr
            if time.monotonic() >= end:
                return None
            self.pump(0.002)

    def counters(self):
        p = self.parser
        return {"crc": p.crc_errors + p.bad_lengths, "garbage": p.garbage_bytes,
                "gaps": self.rx_seq.gaps}

    def exchange(self, ftype, tries=3, timeout=1.0):
        """Send RESET or STATS_REQ and return the STATS reply, draining stale STATS first."""
        for _ in range(tries):
            self.pump(0.2)
            for fr in [f for f in self.queue if f.type == bf.STATS]:
                self.queue.remove(fr)
            self.send(ftype)
            fr = self.wait_frame(bf.STATS, timeout)
            if fr is not None:
                return fr
        return None

    def reset_both(self):
        """Zero the daemon's counters and ours. Returns False if the daemon did not answer."""
        ok = self.exchange(bf.RESET) is not None
        self.pump(0.2)
        self.queue.clear()
        self.parser.crc_errors = self.parser.bad_lengths = self.parser.garbage_bytes = 0
        self.rx_seq.reset()
        self.src = {}
        self.pongs = []
        return ok

    def daemon_stats(self):
        fr = self.exchange(bf.STATS_REQ)
        return bf.unpack_stats(fr.payload) if fr else None


def table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def pct(values, q):
    v = sorted(values)
    return v[max(0, math.ceil(q * len(v)) - 1)] if v else float("nan")


# -- tests -------------------------------------------------------------------------------------
def t_info(link, args):
    d = link.tr.describe()
    dev = str(d["device"] or "")
    if dev.startswith("AIRSEEKERS_"):
        dev = "AIRSEEKERS_xxxxxx"   # the digits are part of the mower serial
    rows = [("device", dev), ("ATT MTU", d["mtu"]), ("notifications via", d["notify"])]
    rows += [(k, v) for k, v in d.items() if k not in ("device", "mtu", "notify", "flags")]
    rows += [("flags %s" % u, f) for u, f in d["flags"].items()]
    text = "### (e) GATT info\n\n" + table(("item", "value"), rows)
    if hasattr(link.tr, "read"):
        from ble_bluez import INFO_UUID
        try:
            data = link.tr.read(INFO_UUID)
            text += "\n\nc1db0003 read, %d bytes:\n\n```\n%s\n```" % (len(data), bf.hexdump(data))
        except Exception as exc:  # noqa: BLE001
            text += "\n\nc1db0003 read failed: %s" % exc
    return text


def t_sweep(link, args):
    rows = []
    for size in args.sizes:
        plen = size - bf.OVERHEAD
        if plen < 0:
            continue
        link.pump(0.3)
        link.queue.clear()
        before, n_before = link.counters(), list(link.notifs)
        ok = lost = 0
        err, rtts = "", []
        for _ in range(args.sweep_count):
            payload = os.urandom(plen)
            del link.tr.write_errors[:]
            t0 = time.monotonic()
            link.send(bf.ECHO, payload)
            if link.tr.write_errors:
                err = link.tr.write_errors[0]
                break
            if link.wait_frame(bf.ECHO_REPLY, 2.0, lambda fr: fr.payload == payload):
                ok += 1
                rtts.append((time.monotonic() - t0) * 1000)
            else:
                lost += 1
        link.pump(0.3)
        after = link.counters()
        hist = [a - b for a, b in zip(link.notifs, n_before)]
        rows.append((size, "no: " + err if err else "yes", "%d/%d" % (ok, ok + lost), lost,
                     after["crc"] - before["crc"], after["garbage"] - before["garbage"],
                     "%.1f" % statistics.median(rtts) if rtts else "-",
                     "%.1f" % (sum(hist) / ok) if ok else "-", " / ".join(map(str, hist))))
    return ("### (a) Write size sweep (ECHO, %d frames per size)\n\n" % args.sweep_count + table(
        ("write B", "accepted", "echo ok", "lost", "CRC err", "garbage B", "median RTT ms",
         "notifs per echo", "notifs by size " + " / ".join(n for n, _ in NOTIF_BUCKETS)), rows)
        + "\n\nEach echo comes back as a frame the same size as the write. If the module "
        "notifies in 20-byte pieces, notifs per echo is about write size / 20.")


def t_sink(link, args):
    if not link.reset_both():
        return "### (b) phone->mower SINK\n\nno answer to RESET from the daemon"
    wwr = link.tr.can_write_without_response() and not args.no_wwr
    window = 8 if wwr else 1      # BlueZ allows one outstanding write request
    plen = args.chunk - bf.OVERHEAD
    payload = bytes(range(256)) * (plen // 256 + 1)
    sent_frames = sent_wire = 0
    del link.tr.write_errors[:]
    t0 = time.monotonic()
    end = t0 + args.sink_seconds
    periodic = []
    while time.monotonic() < end:
        if args.sink_rate and sent_wire >= args.sink_rate * (time.monotonic() - t0):
            link.pump(0.001)
            continue
        while link.tr.inflight >= window:
            link.pump(0)
        sent_wire += link.send(bf.SINK, payload[:plen], wwr=wwr, wait=False)
        sent_frames += 1
        link.pump(0)
        while link.queue and link.queue[0].type == bf.STATS:
            periodic.append(bf.unpack_stats(link.queue.popleft().payload))
    while link.tr.inflight and time.monotonic() < end + 10:
        link.pump(0)
    dur = time.monotonic() - t0
    errors = len(link.tr.write_errors)
    accepted = sent_frames - errors      # writes BlueZ confirmed
    link.pump(2.0)
    for fr in [f for f in link.queue if f.type == bf.STATS]:
        periodic.append(bf.unpack_stats(fr.payload))
        link.queue.remove(fr)
    st = link.daemon_stats() or {}
    mower_rate = "-"
    if len(periodic) >= 2:
        a, b = periodic[0], periodic[-1]
        if b["uptime_ms"] > a["uptime_ms"]:
            mower_rate = "%.0f" % ((b["sink_bytes"] - a["sink_bytes"]) * 1000.0 /
                                   (b["uptime_ms"] - a["uptime_ms"]))
    got = st.get("sink_frames", "-") if st else "no STATS reply"
    lost = ("%d (%.2f %%)" % (accepted - got, 100.0 * (accepted - got) / max(1, accepted))
            if st else "-")
    rows = [("write mode", "without response" if wwr else "with response, 1 in flight"),
            ("rate cap wire B/s", args.sink_rate or "none"),
            ("frame size B (payload)", "%d (%d)" % (args.chunk, plen)),
            ("duration s", "%.1f" % dur), ("writes issued", sent_frames),
            ("writes accepted", accepted),
            ("accepted wire B/s", "%.0f" % (accepted * args.chunk / dur)),
            ("accepted payload B/s", "%.0f" % (accepted * plen / dur)),
            ("frames received by mower", got),
            ("payload B/s received (mower clock)", mower_rate),
            ("frames lost after acceptance", lost),
            ("mower CRC errors", st.get("crc_errors", "-")),
            ("mower garbage B", st.get("garbage_bytes", "-")),
            ("mower seq gaps", st.get("seq_gaps", "-")),
            ("write errors", errors)]
    if link.tr.write_errors:
        rows.append(("first write error", link.tr.write_errors[0]))
    return "### (b) phone->mower throughput (SINK)\n\n" + table(("metric", "value"), rows)


def t_source(link, args):
    if not link.reset_both():
        return "### (c) mower->phone SOURCE\n\nno answer to RESET from the daemon"
    plen = args.chunk - bf.OVERHEAD
    link.send(bf.SOURCE, bf.SOURCE_REQ.pack(0, int(args.source_seconds * 1000), plen,
                                            args.source_rate))
    end_fr = link.wait_frame(bf.SOURCE_END, args.source_seconds + 15)
    if end_fr is None:
        link.send(bf.SOURCE_STOP)
        end_fr = link.wait_frame(bf.SOURCE_END, 3.0)
    link.pump(0.5)
    s, c = link.src, link.counters()
    sent_frames, sent_bytes = bf.SOURCE_END_FMT.unpack(end_fr.payload) if end_fr else (None, None)
    span = (s.get("t_last", 0) - s.get("t_first", 0)) / 1e9
    got = s.get("frames", 0)
    rows = [("frame size B (payload)", "%d (%d)" % (args.chunk, plen)),
            ("requested rate", args.source_rate or "full UART rate"),
            ("frames sent by mower", sent_frames if end_fr else "SOURCE_END lost"),
            ("frames received", got), ("payload bytes received", s.get("bytes", 0)),
            ("receive span s", "%.1f" % span),
            ("payload B/s received", "%.0f" % (s.get("bytes", 0) / span) if span else "-"),
            ("wire B/s received", "%.0f" % (got * args.chunk / span) if span else "-"),
            ("frames lost", "%d (%.2f %%)" % (sent_frames - got, 100.0 * (sent_frames - got) /
                                              max(1, sent_frames)) if end_fr else "-"),
            ("CRC errors (incl. bad length)", c["crc"]), ("garbage B", c["garbage"]),
            ("seq gaps (frames)", c["gaps"]), ("payload pattern mismatches", s.get("bad", 0))]
    return "### (c) mower->phone throughput (SOURCE)\n\n" + table(("metric", "value"), rows)


def t_ping(link, args):
    link.pump(0.3)
    link.pongs = []
    period = 1.0 / args.ping_hz
    n = int(args.ping_seconds * args.ping_hz)
    t_next = time.monotonic()
    for _ in range(n):
        link.send(bf.PING, bf.PING_FMT.pack(time.monotonic_ns()), wait=False)
        t_next += period
        link.pump(max(0.0, t_next - time.monotonic()))
    link.pump(1.0)
    r = link.pongs
    rows = [(n, len(r), n - len(r), "%.1f" % min(r) if r else "-",
             "%.1f" % statistics.median(r) if r else "-", "%.1f" % pct(r, 0.95) if r else "-",
             "%.1f" % max(r) if r else "-")]
    return ("### (d) Round trip, PING at %g Hz for %g s\n\n" % (args.ping_hz, args.ping_seconds)
            + table(("sent", "answered", "lost", "min ms", "median ms", "p95 ms", "max ms"), rows))


def t_reconnect(link, args):
    rows = []
    for i in range(args.reconnects):
        cells = ["-"] * 6
        try:
            cells[0] = "%.2f" % link.tr.disconnect()
            link.pump(args.reconnect_pause)
            t_seen = link.tr.wait_advert(30.0)
            cells[1] = "%.2f" % t_seen if t_seen is not None else "not seen in 30 s"
            t_conn, t_res, attempts = link.tr.connect()
            t0 = time.monotonic() - t_res
            cells[2:5] = ["%.2f" % t_conn, attempts, "%.2f" % t_res]
            link.pongs = []
            link.send(bf.PING, bf.PING_FMT.pack(time.monotonic_ns()))
            end = time.monotonic() + 3.0
            while not link.pongs and time.monotonic() < end:
                link.pump(0.005)
            cells[5] = "%.2f" % (time.monotonic() - t0) if link.pongs else "no reply"
            rows.append([i + 1] + cells + [""])
        except Exception as exc:  # noqa: BLE001 - record and try the next cycle
            rows.append([i + 1] + cells + [str(exc)[:120]])
    return ("### (f) Disconnect / reconnect x%d (pause %.1f s)\n\n" %
            (args.reconnects, args.reconnect_pause) +
            table(("#", "disconnect s", "scan until seen s", "Connect() s", "Connect() calls",
                   "services resolved s", "first PONG s", "error"), rows) +
            "\n\nConnect(), services and first PONG are timed from the start of the "
            "successful Connect(); scanning is stopped before it.")


def redact(argv):
    """Keep the BLE address out of reports that get pasted into docs."""
    return ["<address>" if i and argv[i - 1] == "--address" else
            ("--address=<address>" if a.startswith("--address=") else a)
            for i, a in enumerate(argv)]


TESTS = {"info": t_info, "sweep": t_sweep, "ping": t_ping, "sink": t_sink, "source": t_source,
         "reconnect": t_reconnect}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    who = ap.add_mutually_exclusive_group()
    who.add_argument("--address", help="BLE address of the mower module")
    who.add_argument("--name-prefix", default="AIRSEEKERS_", help="advertised name prefix")
    who.add_argument("--serial", help="bench mode: talk to the daemon over this tty/PTY, no BLE")
    ap.add_argument("--adapter", default="hci0")
    ap.add_argument("--tests", default=ALL_TESTS, help="comma list from: " + ALL_TESTS)
    ap.add_argument("--sizes", default="20,100,180,244,497",
                    type=lambda s: [int(x) for x in s.split(",")], help="sweep write sizes")
    ap.add_argument("--sweep-count", type=int, default=10)
    ap.add_argument("--chunk", type=int, default=244,
                    help="frame (= write) size for sink/source, bytes (default 244)")
    ap.add_argument("--sink-seconds", type=float, default=60)
    ap.add_argument("--sink-rate", type=int, default=8000,
                    help="cap offered wire B/s (default 8000, 0 = none)")
    ap.add_argument("--no-wwr", action="store_true", help="never use write-without-response")
    ap.add_argument("--source-seconds", type=float, default=60)
    ap.add_argument("--source-rate", type=int, default=0, help="mower wire B/s (0 = full UART)")
    ap.add_argument("--ping-seconds", type=float, default=30)
    ap.add_argument("--ping-hz", type=float, default=10)
    ap.add_argument("--reconnects", type=int, default=5)
    ap.add_argument("--reconnect-pause", type=float, default=1.0)
    ap.add_argument("--out", help="also write the markdown report here")
    args = ap.parse_args()
    tests = [t.strip() for t in args.tests.split(",") if t.strip()]
    unknown = [t for t in tests if t not in TESTS]
    if unknown:
        ap.error("unknown tests: %s" % ",".join(unknown))

    if args.serial:
        tests = [t for t in tests if t not in ("reconnect",)]
        link = Link(lambda on_rx: SerialTransport(args.serial, on_rx))
    else:
        from ble_bluez import BluezTransport
        link = Link(lambda on_rx: BluezTransport(args.adapter, args.name_prefix, args.address,
                                                 on_rx))
        name = link.tr.find()
        print("found %s, connecting..." % (name or link.tr.dev_path), file=sys.stderr)
        t_conn, t_res, attempts = link.tr.connect()
        print("connected in %.2f s (%d Connect() calls), services in %.2f s"
              % (t_conn, attempts, t_res), file=sys.stderr)

    report = ["## BLE link measurements, %s" % time.strftime("%Y-%m-%d %H:%M"),
              "", "Command: `%s`" % " ".join(["ble_link_test.py"] + redact(sys.argv[1:]))]
    if link.daemon_stats() is None:
        print("no STATS reply: is ble_echo_daemon.py running on the mower?", file=sys.stderr)
        tests = [t for t in tests if t == "info"]
        report += ["", "**ble_echo_daemon did not answer STATS_REQ; only GATT info was read.**"]
    try:
        for name in tests:
            print("running %s..." % name, file=sys.stderr)
            report += ["", TESTS[name](link, args)]
    except KeyboardInterrupt:
        report += ["", "(interrupted)"]
    finally:
        try:
            link.tr.close()
        except Exception:  # noqa: BLE001
            pass
    text = "\n".join(report) + "\n"
    print(text)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
