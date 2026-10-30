"""lora.py: the vendor pairing bytes, reply parsing and validation (no ROS, no port)."""

from um960_gps_driver import lora
from um960_gps_driver.ntrip_client import nmea_wrap


def test_command_bytes_match_scripts_rtk_pair():
    # Same literals scripts/rtk_pair.py writes (verified on the rover board).
    assert lora.GLIST_BY_AREA["03"] == "$GLIST,2*5B"
    assert lora.GLIST_BY_AREA["02"] == "$GLIST,0*59"
    assert lora.gncon_command(38183, 40) == "$GNCON,9527,28*48"   # vendor universal code
    assert lora.QUERY_PAIRING == "$GNCON,0000,FF*4B"
    assert lora.BASE_RESET == "$BRESET*17" == nmea_wrap("BRESET")


def test_universal_code_pairing_commands():
    ok, commands, _ = lora.pairing_commands("", 38183, 40, area="02")
    assert ok and commands == ["$GLIST,0*59", "$GNCON,9527,28*48"]
    ok, commands, _ = lora.pairing_commands("", 38183, 40, area="03")
    assert ok and commands == ["$GLIST,2*5B", "$GNCON,9527,28*48"]


def test_parse_gncon():
    assert lora.parse_gncon("$GNCON,9527,28*48") == (0x9527, 40)
    assert lora.parse_gncon("$GNCON,9527,18*4B\r\n") == (0x9527, 24)
    assert lora.parse_gncon("GNCON,0001,4B") == (1, 75)
    assert lora.parse_gncon("$GNCON,0000,FF*4B") is None      # the query, not a pairing
    assert lora.parse_gncon("$GNCON,ZZZZ,28*00") is None
    assert lora.parse_gncon("$GNCON,9527*00") is None
    assert lora.parse_gncon("$GLIST,0*59") is None


def test_parse_glist():
    assert lora.parse_glist("$GLIST,0*59") == "0"
    assert lora.parse_glist("$GLIST,2*5B\r\n") == "2"
    assert lora.parse_glist("$GLIST,f*0F") == "F"
    assert lora.parse_glist("$GLIST,*00") is None
    assert lora.parse_glist("$GNCON,9527,28*48") is None
    assert set(lora.BAND_NAMES) == {"0", "1", "2", "F"}


def test_validate_pairing():
    assert lora.validate_pairing(0, 1, "02") == ""
    assert lora.validate_pairing(0xFFFE, 75, "F") == ""
    assert lora.validate_pairing(38183, 40, "") == ""            # vendor BLE: band from SN
    assert "addr" in lora.validate_pairing(0xFFFF, 40, "02")
    assert "addr" in lora.validate_pairing(-1, 40, "02")
    assert "channel" in lora.validate_pairing(1, 0, "02")
    assert "channel" in lora.validate_pairing(1, 76, "02")
    assert "area" in lora.validate_pairing(1, 40, "04")
    assert lora.pairing_commands("", 0xFFFF, 40, area="02")[0] is False
