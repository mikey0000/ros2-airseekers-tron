#!/usr/bin/env python3
"""Pair the Tron's LoRa RTK rover with a vendor RTK base station, without the Airseekers cloud.

Talks to the AT32 ``rtk_rover`` board on /dev/serial_rtk (115200 8N1) with the exact
sentences the vendor stack uses (``libmower_gps_driver.so``, see docs/rtk_pairing.md and
docs/um960.md "LoRa / vendor base station"). Only the GNSS board is touched; nothing here
commands motion. Python stdlib only, so it runs on the mower host or in the container.

    rtk_pair.py query                      # stored pairing + firmware/hardware ids
    rtk_pair.py watch [--seconds 180]      # are LoRa corrections arriving?
    rtk_pair.py pair --sn <base SN> --addr <A> --channel <C> [--area 00|01|02|03|F]
    rtk_pair.py universal                  # vendor "universal code": addr 38183 ch 40 area 02
    rtk_pair.py selftest                   # checksum / formatting unit check, no serial port

Stop the stack first (``docker stop mower_jazzy``) so nothing else reads the port or sends
NTRIP / ``$GNRTK,ON`` while the board is being configured.
"""
import argparse
import os
import re
import sys
import termios
import time

PORT = "/dev/serial_rtk"
BAUD = termios.B115200

# area/region -> band command, verbatim from the vendor driver (um960_gps_driver/lora.py)
GLIST_BY_AREA = {"00": "$GLIST,0*59", "01": "$GLIST,1*58", "02": "$GLIST,0*59",
                 "03": "$GLIST,2*5B", "F": "$GLIST,F*2F"}
AREA_NAMES = {"00": "Europe 863 MHz", "01": "Americas 902 MHz", "02": "Europe 863 MHz",
              "03": "Australia/NZ 927 MHz", "F": "all bands (test)"}
SN_PREFIX_FORCE_01 = "3001034903"
QUERY = ["$GNCON,0000,FF*4B", "$GNVER,*64", "$GHARD,F*32"]
# The app's cloud-less "universal code" bind (BindRTKController::confirmCustomBind).
UNIVERSAL = dict(sn="3001024503000000", addr=38183, channel=40, area="02")
BOARD_SENTENCES = ("$GNCON", "$GNVER", "$GHARD", "$GHARDM", "$GNSTA", "$GNREF", "$GNRTK", "$GNSNR", "$GLIST")
FIX_QUALITY = {"0": "no fix", "1": "single", "2": "DGPS", "4": "RTK fixed", "5": "RTK float"}


def nmea_wrap(body):
    body = body.lstrip("$").split("*")[0]
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return "$%s*%02X" % (body, cs)


def gncon(addr, channel):
    """``$GNCON,AAAA,CC*XX`` exactly as the vendor formats it (both fields hex)."""
    return nmea_wrap("GNCON,%04X,%02X" % (addr & 0xFFFF, channel & 0xFF))


def band_for_sn(sn):
    if len(sn) < 6:
        return None, "sn %r shorter than 6 characters, band skipped" % sn
    region = sn[4:6]
    if region not in GLIST_BY_AREA:
        return None, "sn %r region %r unknown, band skipped" % (sn, region)
    if sn[:10] == SN_PREFIX_FORCE_01:
        region = "01"
    return GLIST_BY_AREA[region], "region %s (%s) from base SN" % (region, AREA_NAMES[region])


def parse_int(text):
    return int(text, 0) if text.lower().startswith("0x") else int(text, 10)


class Port:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = termios.IGNBRK           # iflag: raw
        attrs[1] = 0                        # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag: 8N1, no flow control
        attrs[3] = 0                        # lflag: raw
        attrs[4] = attrs[5] = BAUD
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buf = b""

    def write_line(self, sentence):
        os.write(self.fd, (sentence + "\r\n").encode("ascii"))
        print("  >> %s" % sentence)

    def lines(self, seconds):
        """Yield complete text lines for `seconds` (binary RTCM/Unicore frames are dropped)."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                chunk = b""
            if not chunk:
                time.sleep(0.02)
                continue
            self.buf += chunk
            while b"\n" in self.buf:
                raw, self.buf = self.buf.split(b"\n", 1)
                line = raw.strip(b"\r").decode("ascii", "ignore").strip()
                if line.startswith("$") or line.startswith("#"):
                    yield line
        if len(self.buf) > 65536:
            self.buf = self.buf[-4096:]


def describe_gga(line):
    f = line.split(",")
    if len(f) < 15:
        return None
    q, sats, age, station = f[6], f[7], f[13], f[14].split("*")[0]
    return "fix=%s sats=%s diff_age=%ss station=%s" % (FIX_QUALITY.get(q, q), sats, age or "-", station or "-")


def collect(port, seconds, want=BOARD_SENTENCES, show_fix=True):
    """Print board sentences for `seconds`; return {sentence head: last line}."""
    seen = {}
    last_fix = None
    for line in port.lines(seconds):
        head = line.split(",")[0]
        if head in want:
            print("  << %s" % line)
            seen[head] = line
        elif show_fix and head[3:] == "GGA":
            fix = describe_gga(line)
            if fix and fix != last_fix:
                print("  fix  %s" % fix)
                last_fix = fix
    return seen


def cmd_query(port, args):
    print("== query rover board")
    for s in QUERY:
        port.write_line(s)
        time.sleep(0.2)
    seen = collect(port, args.seconds, show_fix=False)
    con = seen.get("$GNCON")
    if con:
        f = con.split("*")[0].split(",")
        if len(f) >= 3:
            try:
                print("stored pairing: addr=%d (0x%s) channel=%d (0x%s)"
                      % (int(f[1], 16), f[1], int(f[2], 16), f[2]))
            except ValueError:
                print("stored pairing: %s" % con)
    else:
        print("no $GNCON answer in %.0fs (port busy? wrong device? board asleep?)" % args.seconds)
    return 0 if con else 1


def cmd_watch(port, args):
    print("== watching corrections for %ds (base survey-in alone takes ~120 s)" % args.seconds)
    seen = collect(port, args.seconds)
    sta = seen.get("$GNSTA", "")
    ok = "FINE" in sta
    print("result: %s" % ("LoRa corrections LIVE ($GNSTA FINE)" if ok else
                          "no live corrections ($GNSTA %s)" % (sta.split(",")[-1] if sta else "not seen")))
    return 0 if ok else 1


def do_pair(port, sn, addr, channel, area, watch_s):
    if not 1 <= channel <= 75:
        print("channel %d outside 1-75" % channel)
        return 2
    if not 0 <= addr <= 0xFFFE:
        print("addr %d outside 0000-FFFE" % addr)
        return 2
    if area:
        if area not in GLIST_BY_AREA:
            print("area %r unknown (00 01 02 03 F)" % area)
            return 2
        band, note = GLIST_BY_AREA[area], "area %s (%s) given" % (area, AREA_NAMES[area])
    else:
        band, note = band_for_sn(sn)
    print("== pair: base sn=%s addr=%d (0x%04X) channel=%d (0x%02X); %s"
          % (sn, addr, addr, channel, channel, note))
    if band:
        port.write_line(band)
        time.sleep(0.5)                      # vendor waits 500 ms between band and pairing
        collect(port, 0.3, show_fix=False)
    port.write_line(gncon(addr, channel))
    seen = collect(port, 2.0, show_fix=False)
    echo = seen.get("$GNCON")
    print("board echo: %s" % (echo or "none (vendor does not check one either)"))
    print("== re-query")
    port.write_line(QUERY[0])
    seen = collect(port, 2.0, show_fix=False)
    stored = seen.get("$GNCON", "")
    want = "$GNCON,%04X,%02X" % (addr, channel)
    if stored.split("*")[0].upper() == want:
        print("stored pairing matches")
    elif stored:
        print("WARNING stored pairing %s != requested %s" % (stored, want))
    else:
        print("WARNING no $GNCON answer after pairing")
    if watch_s > 0:
        print("== now power the base within ~2 m, sky view, and wait for its survey-in")
        return cmd_watch(port, argparse.Namespace(seconds=watch_s))
    return 0


def cmd_pair(port, args):
    return do_pair(port, args.sn, parse_int(args.addr), args.channel, args.area, args.watch)


def cmd_universal(port, args):
    u = UNIVERSAL
    return do_pair(port, u["sn"], u["addr"], u["channel"], u["area"], args.watch)


def selftest():
    assert nmea_wrap("GNCON,0000,FF") == "$GNCON,0000,FF*4B"
    assert nmea_wrap("GNRTK,ON") == "$GNRTK,ON*69"
    assert nmea_wrap("GNVER,") == "$GNVER,*64"
    assert nmea_wrap("GHARD,F") == "$GHARD,F*32"
    for cmd in GLIST_BY_AREA.values():
        assert nmea_wrap(cmd) == cmd, cmd
    assert gncon(38183, 40) == "$GNCON,9527,28*48"
    assert band_for_sn("3001024503000000") == ("$GLIST,0*59", "region 02 (Europe 863 MHz) from base SN")
    assert band_for_sn("3001034903XXXX")[0] == "$GLIST,1*58"
    assert band_for_sn("3001")[0] is None
    assert describe_gga("$GNGGA,000000.00,0,N,0,E,4,18,0.8,1.0,M,0,M,0.5,0001*00") == \
        "fix=RTK fixed sats=18 diff_age=0.5s station=0001"
    print("selftest ok")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=PORT)
    sub = ap.add_subparsers(dest="cmd", required=True)
    q = sub.add_parser("query", help="read the stored pairing and board ids")
    q.add_argument("--seconds", type=float, default=3.0)
    w = sub.add_parser("watch", help="watch for live LoRa corrections")
    w.add_argument("--seconds", type=int, default=180)
    p = sub.add_parser("pair", help="write a pairing (base SN + addr + channel)")
    p.add_argument("--sn", required=True, help="base station serial number (label / QR)")
    p.add_argument("--addr", required=True, help="LoRa address, decimal or 0xHEX (0-65534)")
    p.add_argument("--channel", type=int, required=True, help="LoRa channel 1-75")
    p.add_argument("--area", default="", help="band override: 00 01 02 03 F (default: from SN)")
    p.add_argument("--watch", type=int, default=180, help="seconds to watch afterwards (0 = none)")
    u = sub.add_parser("universal", help="vendor universal code: addr 38183 channel 40 area 02")
    u.add_argument("--watch", type=int, default=180)
    sub.add_parser("selftest")
    args = ap.parse_args(argv)
    if args.cmd == "selftest":
        return selftest()
    try:
        port = Port(args.port)
    except OSError as exc:
        print("cannot open %s: %s (run as root on the host, or inside mower_jazzy)" % (args.port, exc))
        return 1
    return {"query": cmd_query, "watch": cmd_watch, "pair": cmd_pair, "universal": cmd_universal}[args.cmd](port, args)


if __name__ == "__main__":
    sys.exit(main())
