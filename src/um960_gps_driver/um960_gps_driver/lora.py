"""Vendor LoRa base-station pairing commands (ROS-free).

Recovered from the vendor driver ``libmower_gps_driver.so``
(``mower_gps_driver::UnicoreGpsRos::setLoRa`` @ 0x0027ff1c and ``setBand`` @ 0x002806a4,
``native_decompile/dec/libmower_gps_driver/libmower_gps_driver.so.c``):

* The host never talks to the LoRa radio directly. It writes NMEA-style ``$G...``
  commands to ``/dev/serial_rtk`` - the *same* UART the UM960 solution stream
  arrives on. That UART is terminated by the AT32 ``rtk_rover`` board, which owns
  the UM960 and the LoRa module host MCU (``rtk_rover_m4``, a GD32/Cortex-M4 driving
  a Semtech SX127x). The rover firmware strings (``mcu_decompile/rtk_rover``) contain
  exactly these commands: ``$GNCON``, ``$GLIST``/``GLIST,%d``, ``$GNRTK``,
  ``RTKRESET``, ``$BRESET``, ``$GHARD``, ``$GNVER``, ``$GLORA``, ``LORARESET``.
* ``set_lora(sn, addr, channel, area)``:
    1. band: if ``area`` is empty, derive the region from the serial number
       (``sn[4:6]``; SN prefix ``3001034903`` is forced to region ``01``) and send the
       matching ``$GLIST`` command; otherwise look ``area`` up directly. Unknown
       area => result false; bad SN => logged, band skipped (vendor still pairs).
    2. wait 0.5 s.
    3. ``$GNCON,<addr %04X>,<channel %02X>*<xor>`` - note the channel is printed
       in **hex** (the vendor's ``std::hex`` manipulator is still active).
    4. result true once written. There is no acknowledgement check; the rover
       echoes ``$GNCON,<addr>,<channel>`` which the vendor only logs.
* On every (re)connect the vendor also polls ``$GNCON,0000,FF*4B`` (query the
  pairing), ``$GNVER,*64`` and ``$GHARD,F*32``.
* Network RTK: ``$GNRTK,ON*69`` switches the rover board from LoRa corrections to
  host-injected RTCM; the vendor only forwards NTRIP RTCM while the rover reports
  ``$GNRTK,ON``, and sends ``RTKRESET`` when NRTK is first enabled or drops.
"""

from typing import Dict, List, Optional, Tuple

from .ntrip_client import nmea_wrap

# area/region code -> $GLIST band command, verbatim from the vendor constructor
# (std::map initialised at UnicoreGpsRos+0x270).
GLIST_BY_AREA = {
    "00": "$GLIST,0*59",
    "01": "$GLIST,1*58",
    "02": "$GLIST,0*59",
    "03": "$GLIST,2*5B",
    "F": "$GLIST,F*2F",
}
# Serial-number prefix the vendor forces onto region "01".
SN_PREFIX_FORCE_01 = "3001034903"

QUERY_PAIRING = "$GNCON,0000,FF*4B"
QUERY_VERSION = "$GNVER,*64"
QUERY_HARDWARE = "$GHARD,F*32"
NRTK_ON = "$GNRTK,ON*69"
NRTK_OFF = "$GNRTK,OFF*27"
RTK_RESET = "RTKRESET"
# Rover firmware string; asks the board to reset (and, per the base firmware, relay a
# reset to the base). Untested on hardware.
BASE_RESET = "$BRESET*17"

CHANNEL_MIN = 1
CHANNEL_MAX = 75
ADDR_MAX = 0xFFFE

# $GLIST band (what the board reports back) -> radio band.
BAND_NAMES: Dict[str, str] = {
    "0": "Europe 863 MHz",
    "1": "Americas 902 MHz",
    "2": "Australia/NZ 927 MHz",
    "F": "all bands",
}


def gncon_command(addr: int, channel: int) -> str:
    """``$GNCON,AAAA,CC*XX`` exactly as the vendor formats it (all hex)."""
    return nmea_wrap("GNCON,%04X,%02X" % (int(addr) & 0xFFFF, int(channel) & 0xFF))


def band_for_sn(sn: str) -> Tuple[Optional[str], str]:
    """Vendor ``setBand``: ``(glist_command | None, explanation)``."""
    if len(sn) < 6:
        return None, "sn error %r (shorter than 6 characters)" % sn
    region = sn[4:6]
    if region not in GLIST_BY_AREA:
        return None, "sn error %r (region %r unknown)" % (sn, region)
    command = GLIST_BY_AREA[region]
    if sn[:10] == SN_PREFIX_FORCE_01:
        command = GLIST_BY_AREA["01"]
    return command, "sn region %s" % region


def _fields(line: str) -> List[str]:
    return line.strip().lstrip("$").split("*")[0].split(",")


def parse_gncon(line: str) -> Optional[Tuple[int, int]]:
    """``$GNCON,AAAA,CC*XX`` (both hex) -> ``(addr, channel)``; None if not one.

    The query sentence itself (``$GNCON,0000,FF``) is not a pairing: channel 0xFF is
    outside 1-75, so it is rejected like any other out-of-range reply."""
    fields = _fields(line)
    if len(fields) < 3 or fields[0] != "GNCON":
        return None
    try:
        addr, channel = int(fields[1], 16), int(fields[2], 16)
    except ValueError:
        return None
    if not (0 <= addr <= 0xFFFF and CHANNEL_MIN <= channel <= CHANNEL_MAX):
        return None
    return addr, channel


def parse_glist(line: str) -> Optional[str]:
    """``$GLIST,<band>*XX`` -> band (``"0"``, ``"1"``, ``"2"``, ``"F"``); None if not one."""
    fields = _fields(line)
    if len(fields) < 2 or fields[0] != "GLIST":
        return None
    band = fields[1].strip().upper()
    return band or None


def validate_pairing(addr: int, channel: int, area: str) -> str:
    """Empty string when ``set_lora`` may write this pairing, else the reason.

    ``area`` "" is allowed: the vendor BLE path derives the band from the base serial."""
    if not 0 <= int(addr) <= ADDR_MAX:
        return "addr %d outside 0-%d (0x0000-0x%04X)" % (addr, ADDR_MAX, ADDR_MAX)
    if not CHANNEL_MIN <= int(channel) <= CHANNEL_MAX:
        return "channel %d outside %d-%d" % (channel, CHANNEL_MIN, CHANNEL_MAX)
    if area and area not in GLIST_BY_AREA:
        return "area %r unknown (expected 00, 01, 02, 03 or F)" % area
    return ""


def pairing_commands(sn: str, addr: int, channel: int,
                     area: str = "") -> Tuple[bool, List[str], str]:
    """Commands for one ``set_lora`` call: ``(ok, [band_cmd?, gncon], message)``.

    ``ok`` is false where the vendor returned false (unknown area), plus an address
    or channel outside the documented ranges (``validate_pairing``).
    """
    error = validate_pairing(addr, channel, area)
    if error:
        return False, [], error
    commands: List[str] = []
    if area:
        commands.append(GLIST_BY_AREA[area])
        note = "area %s" % area
    else:
        band, note = band_for_sn(sn)
        if band is not None:
            commands.append(band)
    commands.append(gncon_command(addr, channel))
    return True, commands, note


__all__ = [
    "BAND_NAMES",
    "BASE_RESET",
    "GLIST_BY_AREA",
    "NRTK_ON",
    "NRTK_OFF",
    "QUERY_PAIRING",
    "RTK_RESET",
    "band_for_sn",
    "gncon_command",
    "pairing_commands",
    "parse_gncon",
    "parse_glist",
    "validate_pairing",
]
