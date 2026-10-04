"""Parser tests for the UM960 driver (no ROS runtime required)."""

import struct

import pytest

from um960_gps_driver.parsers import (
    NmeaParser,
    UnicoreBinaryParser,
    Um960StreamSplitter,
    crc16_xmodem,
    dm_to_degrees,
    nmea_checksum_ok,
    sigma_to_covariance,
)


def _cs(body: str) -> str:
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return "$%s*%02X\r\n" % (body, checksum)


def test_nmea_checksum():
    assert nmea_checksum_ok("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
    assert not nmea_checksum_ok("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00")
    assert not nmea_checksum_ok("garbage")


def test_dm_conversion():
    assert abs(dm_to_degrees("4807.038", "N") - 48.1173) < 1e-4
    assert abs(dm_to_degrees("01131.000", "E") - 11.5166666) < 1e-6
    assert dm_to_degrees("4807.038", "S") < 0.0
    assert dm_to_degrees("01131.000", "W") < 0.0
    assert dm_to_degrees("", "N") is None
    assert dm_to_degrees("notanumber", "N") is None


def test_gga_sets_fix_and_status():
    parser = NmeaParser()
    parser.feed(
        _cs("GNGGA,123519,4807.038,N,01131.000,E,4,12,0.7,545.4,M,46.9,M,1.0,0451")
    )
    fix = parser.fix
    assert parser.has_fix
    assert abs(fix["lat"] - 48.1173) < 1e-4
    assert abs(fix["lon"] - 11.5166666) < 1e-6
    assert abs(fix["altitude"] - 545.4) < 1e-6
    assert fix["quality"] == 4
    assert fix["quality_label"] == "RTK_FIXED"
    assert fix["status"] == "STATUS_FIX"
    assert fix["num_sats"] == 12
    assert abs(fix["hdop"] - 0.7) < 1e-9
    assert abs(fix["fix_age"] - 1.0) < 1e-9
    assert fix["station_id"] == "0451"


def test_gga_quality_zero_has_no_fix():
    parser = NmeaParser()
    parser.feed(_cs("GNGGA,123519,4807.038,N,01131.000,E,0,05,1.2,10.0,M,0.0,M,,,"))
    assert not parser.has_fix
    assert parser.fix["status"] == "STATUS_NO_FIX"


def test_gga_quality_zero_clears_a_previous_fix():
    parser = NmeaParser()
    parser.feed(_cs("GNGGA,123519,4807.038,N,01131.000,E,4,12,0.7,545.4,M,46.9,M,,"))
    assert parser.has_fix
    parser.feed(_cs("GNGGA,123520,,,,,0,00,99.9,,,,,,"))
    assert not parser.has_fix


def test_insane_coordinates_are_rejected():
    parser = NmeaParser()
    parser.feed(_cs("GNGGA,123519,9130.000,N,18131.000,E,4,12,0.7,545.4,M,46.9,M,,"))
    assert not parser.has_fix


def test_rmc_void_clears_a_previous_fix():
    parser = NmeaParser()
    parser.feed(_cs("GNRMC,123520,A,4807.038,N,01131.000,E,22.4,84.4,230394,,,A"))
    assert parser.has_fix
    parser.feed(_cs("GNRMC,123521,V,,,,,,230394,,,N"))
    assert not parser.has_fix


def test_rmc_and_vtg():
    parser = NmeaParser()
    parser.feed(_cs("GNRMC,123520,A,4807.038,N,01131.000,E,22.4,84.4,230394,,,A"))
    assert abs(parser.fix["speed_mps"] - 22.4 * 0.514444) < 1e-4
    assert abs(parser.fix["course_deg"] - 84.4) < 1e-9
    parser.feed(_cs("GNVTG,84.4,T,87.6,M,22.4,N,41.5,K,A"))
    assert abs(parser.fix["speed_mps"] - 41.5 / 3.6) < 1e-4


def test_gsa_and_gsv():
    parser = NmeaParser()
    # 12 satellite slots (5 populated), then PDOP/HDOP/VDOP.
    parser.feed(_cs("GNGSA,A,3,01,03,05,11,17,,,,,,,,2.10,1.05,2.33"))
    assert parser.fix["fix_type"] == 3
    assert parser.fix["gsa_mode"] == "A"
    assert parser.fix["num_sats_used"] == 5  # 01, 03, 05, 11, 17
    assert abs(parser.fix["hdop"] - 1.05) < 1e-9
    assert abs(parser.fix["pdop"] - 2.10) < 1e-9
    parser.feed(_cs("GPGSV,3,1,11,01,40,083,44,03,17,313,40,06,07,344,39,11,77,084,45"))
    assert parser.fix["satellites_in_view"] == 11


def test_gphpr_heading():
    parser = NmeaParser()
    parser.feed(_cs("GPHPR,275.34,1,1.2,-0.8"))
    assert parser.fix["heading_deg"] == pytest.approx(275.34)


def test_unicore_ascii_logs():
    parser = NmeaParser()
    parser.feed(_cs("GNVER,UM960,1.2.3,2.1.0"))
    assert parser.fix["version"] == "UM960,1.2.3,2.1.0"
    parser.feed(_cs("GNSTA,3,FINE"))
    assert parser.fix["reference_station_status"] == "FINE"
    parser.feed(_cs("GNREF,BASE1,BASE ONE,48.1,11.5"))


def _bestnav_ascii(sol_status="0", pos_type="12", lat="48.1173", lon="11.5166666"):
    return _cs(
        ",".join(
            [
                "BESTNAV",
                "101530.00",
                "2318",
                "468000.000",
                sol_status,
                pos_type,
                lat,
                lon,
                "545.4000",
                "0.0050",
                "0.0070",
                "0.0120",
                "0.500",
                "0.100",
                "21",
                "18",
                "0",
                "0",
                "0",
                "45.500",
                "61",
                "0",
                "0",
                "0",
                "0",
                "3",
                "0.500",
                "1.234",
                "0.020",
                "-0.015",
                "87.600",
            ]
        )
    )


def test_bestnav_ascii_rejects_insane_coordinates():
    parser = NmeaParser()
    parser.feed(_bestnav_ascii(lat="913.0", lon="1815.0"))
    assert not parser.has_fix


def test_unicore_ascii_bestnav():
    parser = NmeaParser()
    body = ",".join(
        [
            "BESTNAV",
            "101530.00",      # time
            "2318",           # week
            "468000.000",     # ms of week
            "0",              # solStatus: computed
            "12",             # posType: narrow-lane integer
            "48.117300000",   # lat
            "11.516666600",   # lon
            "545.4000",       # hgt
            "0.0050",         # lat sigma
            "0.0070",         # lon sigma
            "0.0120",         # hgt sigma
            "0.500",          # diff age
            "0.100",          # sol age
            "21",             # #SV
            "18",             # #solnSV
            "0", "0", "0",    # reserved
            "45.500",         # undulation
            "61",             # datum
            "0", "0", "0",    # ext sol status / masks
            "0",              # velStatus
            "3",              # velType: doppler velocity
            "0.500",          # age
            "1.234",          # horizontal speed
            "0.020",          # latency
            "-0.015",         # vertical speed
            "87.600",         # track
        ]
    )
    parser.feed(_cs(body))
    fix = parser.fix
    assert parser.has_fix
    assert abs(fix["lat"] - 48.1173) < 1e-6
    assert abs(fix["lon"] - 11.5166666) < 1e-6
    assert fix["position_type"] == "NARROW_INT"
    assert fix["solution_status"] == "SOL_COMPUTED"
    assert fix["velocity_type"] == "DOPPLER_VELOCITY"
    assert abs(fix["horizontal_speed"] - 1.234) < 1e-6
    assert abs(fix["track_ground"] - 87.6) < 1e-6


def _bestnav_binary(payload: bytes) -> bytes:
    body = bytes([0x0A]) + struct.pack("<H", len(payload)) + payload
    return b"\xaa\x44\xb5" + body + struct.pack("<H", crc16_xmodem(body))


def test_bestnav_binary():
    payload = bytearray(124)
    struct.pack_into("<II", payload, 0, 0, 12)  # solStatus computed, posType narrow-int
    struct.pack_into("<q", payload, 8, int(48.1173 * 1e8))
    struct.pack_into("<q", payload, 16, int(11.5166666 * 1e8))
    struct.pack_into("<i", payload, 24, int(545.4 * 1e4))
    struct.pack_into("<fff", payload, 28, 0.005, 0.007, 0.012)
    payload[40:44] = b"BAS1"
    struct.pack_into("<ff", payload, 44, 0.5, 0.1)
    struct.pack_into("<f", payload, 56, 45.5)  # undulation
    struct.pack_into("<I", payload, 60, 61)  # datum id: WGS84
    payload[52] = 21
    payload[53] = 18
    struct.pack_into("<II", payload, 68, 0, 4)  # velStatus, velType doppler
    struct.pack_into("<f", payload, 76, 0.5)
    struct.pack_into("<f", payload, 80, 0.02)
    struct.pack_into("<d", payload, 88, 1.234)
    struct.pack_into("<d", payload, 96, 87.6)
    struct.pack_into("<d", payload, 104, -0.015)

    parser = UnicoreBinaryParser()
    results = parser.feed(_bestnav_binary(bytes(payload)))
    assert len(results) == 1
    frame = results[0]
    assert abs(frame["latitude"] - 48.1173) < 1e-7
    assert abs(frame["longitude"] - 11.5166666) < 1e-7
    assert abs(frame["height"] - 545.4) < 1e-3
    assert frame["station_id"] == "BAS1"
    assert frame["num_sats"] == 21
    assert abs(frame["undulation"] - 45.5) < 1e-6
    assert frame["datum_id"] == 61
    assert abs(frame["horizontal_speed"] - 1.234) < 1e-9
    assert abs(frame["track_ground"] - 87.6) < 1e-9
    assert parser.message_counts[0x0A] == 1


def test_binary_stream_is_reassembled_across_chunks():
    payload = bytearray(124)
    struct.pack_into("<II", payload, 0, 0, 4)
    struct.pack_into("<q", payload, 8, int(48.0 * 1e8))
    struct.pack_into("<q", payload, 16, int(11.0 * 1e8))
    frame = _bestnav_binary(bytes(payload))
    parser = UnicoreBinaryParser()
    out = []
    for i in range(0, len(frame), 7):
        out.extend(parser.feed(frame[i : i + 7]))
    assert len(out) == 1


def test_binary_resyncs_on_garbage():
    payload = bytearray(124)
    struct.pack_into("<q", payload, 8, int(48.0 * 1e8))
    struct.pack_into("<q", payload, 16, int(11.0 * 1e8))
    parser = UnicoreBinaryParser()
    out = parser.feed(b"\x11\x22\x33" + _bestnav_binary(bytes(payload)) + b"trailing")
    assert len(out) == 1


def test_binary_bad_crc_is_dropped():
    parser = UnicoreBinaryParser()
    frame = bytearray(_bestnav_binary(bytes(bytearray(124))))
    frame[-1] ^= 0xFF
    assert parser.feed(bytes(frame)) == []


def test_splitter_keeps_ascii_and_binary_in_order():
    payload = bytearray(124)
    struct.pack_into("<II", payload, 0, 0, 4)
    struct.pack_into("<q", payload, 8, int(48.0 * 1e8))
    struct.pack_into("<q", payload, 16, int(11.0 * 1e8))
    frame = _bestnav_binary(bytes(payload))
    splitter = Um960StreamSplitter()
    events = splitter.feed(
        _cs("GNGGA,101530.00,4807.038,N,01131.000,E,4,21,0.6,545.4,M,46.9,M,,").encode()
        + frame
        + _cs("GNVTG,87.6,T,90.0,M,2.4,N,4.4,K,A").encode()
    )
    kinds = [kind for kind, _ in events]
    assert kinds == ["line", "frame", "line"]
    assert events[0][1].startswith("$GNGGA")
    assert events[1][1]["kind"] == "bestnav"
    assert events[2][1].startswith("$GNVTG")


def test_splitter_survives_arbitrary_chunking():
    payload = bytearray(124)
    struct.pack_into("<II", payload, 0, 0, 4)
    struct.pack_into("<q", payload, 8, int(48.0 * 1e8))
    struct.pack_into("<q", payload, 16, int(11.0 * 1e8))
    stream = _cs("GNGGA,101530.00,4807.038,N,01131.000,E,4,21,0.6,545.4,M,46.9,M,,").encode()
    stream += _bestnav_binary(bytes(payload))
    splitter = Um960StreamSplitter()
    events = []
    for i in range(0, len(stream), 3):
        events.extend(splitter.feed(stream[i : i + 3]))
    assert [kind for kind, _ in events] == ["line", "frame"]
    assert splitter.buffer == bytearray()


def test_splitter_keeps_a_split_sync_triplet():
    payload = bytearray(124)
    struct.pack_into("<q", payload, 8, int(48.0 * 1e8))
    struct.pack_into("<q", payload, 16, int(11.0 * 1e8))
    frame = _bestnav_binary(bytes(payload))
    splitter = Um960StreamSplitter()
    events = splitter.feed(frame[:2])
    assert events == []
    events = splitter.feed(frame[2:])
    assert [kind for kind, _ in events] == ["frame"]


def test_covariance_helpers():
    cov = sigma_to_covariance(0.005, 0.007, 0.012)
    assert abs(cov[0] - 0.005 ** 2) < 1e-12
    assert abs(cov[4] - 0.007 ** 2) < 1e-12
    assert abs(cov[8] - 0.012 ** 2) < 1e-12
    assert sigma_to_covariance(0.0, 0.0, 0.0) == [0.0] * 9
