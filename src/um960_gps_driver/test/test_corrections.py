"""Correction-source integration: pty in place of the UM960, fake NTRIP caster.

Checks that RTCM from the caster lands on the receiver port only in ntrip mode,
that the receiver GGA reaches the caster, the /fix_status tokens, the
/gps/corrections JSON, the vendor ntrip.yaml fallback and the LoRa pairing services
(set_lora / get_lora / reset_base, /mower_gps_node/lora_status) against a fake rover board.
"""

import json
import os
import select
import threading
import time
import types

import pytest

pytest.importorskip("rclpy")

from test_um960_node import GGA_RTK, Harness, _bestnav_frame, _cs, _Recorder  # noqa: E402

from um960_gps_driver import lora  # noqa: E402
from um960_gps_driver.fake_caster import FakeCaster, sample_frame  # noqa: E402


def _wait(predicate, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    raise AssertionError("condition not met within %.1fs" % timeout)


class PtyReader:
    """Collects what the node writes to the receiver end of the pty."""

    def __init__(self, master):
        self.master = master
        self.data = bytearray()

    def poll(self, wait=0.0):
        deadline = time.time() + wait
        while True:
            ready, _, _ = select.select([self.master], [], [], 0.02)
            if ready:
                try:
                    chunk = os.read(self.master, 65536)
                except OSError:
                    chunk = b""
                self.data.extend(chunk)
                continue
            if time.time() >= deadline:
                return bytes(self.data)


def _ntrip_params(caster, **extra):
    params = {
        "correction_source": "ntrip",
        "ntrip_host": caster.host,
        "ntrip_port": caster.port,
        "ntrip_mountpoint": caster.mountpoint,
        "ntrip_user": caster.user,
        "ntrip_password": caster.password,
        "ntrip_gga_interval_s": 0.2,
        "ntrip_version": 2,
    }
    params.update(extra)
    return params


def _status(h):
    h.tick()
    return h.recorder("/fix_status").msgs[-1].data


def _corrections(h):
    rec = _Recorder()
    h.node.corrections_pub = rec
    h.node.publish_corrections()
    return json.loads(rec.msgs[-1].data)


def _request(**kwargs):
    fields = {"sn": "", "addr": 1, "channel": 10, "area": ""}
    fields.update(kwargs)
    return types.SimpleNamespace(**fields)


@pytest.fixture
def caster():
    c = FakeCaster(frame_period_s=0.02).start()
    yield c
    c.stop()


def test_lora_default_only_queries_the_pairing_and_reports_receiver_diff_age():
    h = Harness()
    try:
        reader = PtyReader(h.master)
        assert h.node.correction_source == "lora"
        status = _status(h)
        assert "corr_src=lora corr=lora corr_flow=waiting" in status
        h.send(GGA_RTK)  # diff age 0.5 s
        status = _status(h)
        assert "corr_flow=active" in status and "corr_age=0.5s" in status
        assert _corrections(h)["source"] == "lora"
        # The vendor's on-connect pairing query is the only write in lora mode.
        assert reader.poll(0.2) == lora.QUERY_PAIRING.encode() + b"\r\n"
    finally:
        h.close()


def test_none_reports_off():
    h = Harness(extra_params={"correction_source": "none"})
    try:
        assert "corr_src=none corr=off corr_flow=idle" in _status(h)
        assert PtyReader(h.master).poll(0.2) == b""
    finally:
        h.close()


def test_ntrip_without_host_falls_back_to_none():
    h = Harness(extra_params={"correction_source": "ntrip"})
    try:
        assert h.node.correction_source == "none" and h.node.ntrip is None
    finally:
        h.close()


RESET_ON = b"RTKRESET\r\n" + lora.NRTK_ON.encode() + b"\r\n"
ON = lora.NRTK_ON.encode() + b"\r\n"


def test_ntrip_handshake_holds_rtcm_until_the_board_acks(caster):
    h = Harness(extra_params=_ntrip_params(caster))
    try:
        reader = PtyReader(h.master)
        h.send(GGA_RTK)
        # Off->on transition at node start: the 12-char ON command, no RTKRESET (the
        # running receiver keeps its RTK solution through a stack restart).
        _wait(lambda: reader.poll().startswith(ON))
        # RTCM arrives from the caster but the board has not said ON: nothing passes.
        _wait(lambda: h.node.correction_summary()["rtcm_held_bytes"] > 0)
        assert b"\xd3" not in reader.poll(0.1)
        status = _status(h)
        assert "corr=streaming corr_flow=held" in status
        assert _corrections(h)["rtcm_gate"] == "held"
        # Board acknowledges network RTK: RTCM flows unchanged.
        h.send(_cs("GNRTK,ON"))
        _wait(lambda: b"\xd3\x00\x13" in reader.poll())
        _wait(lambda: any(u.startswith("$GNGGA,101530.00,4807.038") for u in caster.uploads))
        status = _status(h)
        assert "corr_src=ntrip corr=streaming corr_flow=active" in status
        assert "corr_rate=" in status and "corr_age=" in status
        diag = _corrections(h)
        assert diag["rtcm_gate"] == "open" and diag["frames"] >= 2
        assert diag["mountpoint"] == caster.mountpoint and "password" not in json.dumps(diag)
        assert diag["serial_bytes_written"] > len(ON)
        assert reader.poll().count(b"RTKRESET") == 0
    finally:
        h.close()


def test_ntrip_keepalive_repeats_on_every_period(caster):
    h = Harness(extra_params=_ntrip_params(caster, ntrip_keepalive_s=0.5))
    try:
        reader = PtyReader(h.master)
        _wait(lambda: reader.poll().startswith(ON))
        t0 = time.monotonic()
        # Unconditional: the board saying ON does not stop the keep-alive.
        h.send(_cs("GNRTK,ON"))
        _wait(lambda: reader.poll().count(ON) >= 4, timeout=4.0)
        elapsed = time.monotonic() - t0
        assert 1.2 < elapsed < 2.5, elapsed
        assert reader.poll().count(b"RTKRESET") == 0
    finally:
        h.close()


def test_board_on_to_off_triggers_reset_and_on(caster):
    h = Harness(extra_params=_ntrip_params(caster))
    try:
        reader = PtyReader(h.master)
        _wait(lambda: reader.poll().startswith(ON))
        h.send(_cs("GNRTK,OFF"))   # unknown -> OFF: just the keep-alive, no reset
        assert reader.poll(0.2).count(b"RTKRESET") == 0
        h.send(_cs("GNRTK,ON"))
        reader.data.clear()
        h.send(_cs("GNRTK,OFF"))   # ON -> OFF: RTKRESET + ON again
        _wait(lambda: RESET_ON in reader.poll())
        assert h.node.correction_summary()["rover_nrtk"] == "OFF"
    finally:
        h.close()


def test_rtk_reset_on_start_restores_the_vendor_reset(caster):
    h = Harness(extra_params=_ntrip_params(caster, rtk_reset_on_start=True))
    try:
        reader = PtyReader(h.master)
        _wait(lambda: reader.poll().startswith(RESET_ON))
        assert reader.poll(0.2).count(b"RTKRESET") == 1
    finally:
        h.close()


def test_forward_without_board_ack_for_bench_tests(caster):
    h = Harness(extra_params=_ntrip_params(caster, forward_rtcm_without_board_ack=True))
    try:
        reader = PtyReader(h.master)
        _wait(lambda: sample_frame(1005, 0) in reader.poll())
        assert _corrections(h)["rtcm_gate"] == "forced"
    finally:
        h.close()


def test_ntrip_synthesizes_gga_from_bestnav(caster):
    h = Harness(extra_params=_ntrip_params(caster))
    try:
        h.send(_bestnav_frame(lat=48.2, lon=11.6))
        _wait(lambda: any(u.startswith("$GPGGA") for u in caster.uploads))
        gga = [u for u in caster.uploads if u.startswith("$GPGGA")][-1]
        assert ",4812.0000000,N,01136.0000000,E," in gga
    finally:
        h.close()


def test_netmode_maps_to_bind_device(caster):
    h = Harness(extra_params=_ntrip_params(caster, ntrip_netmode="wifi"))
    try:
        assert h.node.ntrip_config.bind_device == "wlan0"
        assert _corrections(h)["netmode"] == "wifi"
    finally:
        h.close()
    h = Harness(extra_params=_ntrip_params(caster, ntrip_netmode="4g"))
    try:
        assert h.node.ntrip_config.bind_device == "usb0"
    finally:
        h.close()
    h = Harness(extra_params=_ntrip_params(caster))
    try:
        assert h.node.ntrip_config.bind_device == ""
    finally:
        h.close()


def test_vendor_ntrip_yaml_enables_ntrip_when_source_unset(caster, tmp_path):
    path = tmp_path / "ntrip.yaml"
    path.write_text(
        "ntrip_enable: true\nntrip_ip: \"%s\"\nntrip_port: %d\nntrip_user: \"%s\"\n"
        "ntrip_passwd: \"%s\"\nntrip_mountpoint: \"%s\"\n"
        % (caster.host, caster.port, caster.user, caster.password, caster.mountpoint))
    h = Harness(extra_params={"ntrip_config_file": str(path),
                              "forward_rtcm_without_board_ack": True})
    try:
        # rtcm_source auto: NTRIP takes over once the caster delivers valid RTCM.
        _wait(lambda: h.node.correction_source == "ntrip")
        assert h.node.ntrip_config_source == str(path)
        reader = PtyReader(h.master)
        _wait(lambda: sample_frame(1005, 10) in reader.poll())
    finally:
        h.close()


def test_vendor_ntrip_yaml_disabled_keeps_lora(tmp_path):
    path = tmp_path / "ntrip.yaml"
    path.write_text("ntrip_enable: false\nntrip_ip: \"h\"\nntrip_port: 2101\n"
                    "ntrip_user: \"\"\nntrip_passwd: \"\"\nntrip_mountpoint: \"M\"\n")
    h = Harness(extra_params={"ntrip_config_file": str(path)})
    try:
        assert h.node.correction_source == "lora" and h.node.ntrip is None
    finally:
        h.close()


class FakeRoverBoard:
    """The rtk_rover board's pairing replies on the receiver end of the pty.

    Stores ``$GLIST`` / ``$GNCON`` writes and answers the ``$GNCON,0000,FF`` query with
    ``$GNCON,<addr>,<chan>`` then ``$GLIST,<band>``, as the real board does.
    ``store=False`` models a board that ignores the pairing write."""

    def __init__(self, master, addr="9527", channel="18", band="0", store=True):
        self.master = master
        self.addr, self.channel, self.band, self.store = addr, channel, band, store
        self.written = bytearray()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        buf = b""
        while not self._stop.is_set():
            ready, _, _ = select.select([self.master], [], [], 0.02)
            if not ready:
                continue
            try:
                chunk = os.read(self.master, 4096)
            except OSError:
                return
            self.written.extend(chunk)
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._handle(raw.strip().decode("ascii", "ignore"))

    def _handle(self, line):
        body = line.lstrip("$").split("*")[0]
        fields = body.split(",")
        if body == "GNCON,0000,FF":
            os.write(self.master, _cs("GNCON,%s,%s" % (self.addr, self.channel))
                     + _cs("GLIST,%s" % self.band))
        elif fields[0] == "GNCON" and len(fields) == 3 and self.store:
            self.addr, self.channel = fields[1], fields[2]
            os.write(self.master, _cs(body))  # the board echoes the pairing
        elif fields[0] == "GLIST" and self.store:
            self.band = fields[1]

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)


def _lora_harness(**extra):
    h = Harness(extra_params=extra)
    board = FakeRoverBoard(h.master)
    _wait(lambda: h.node.lora_pairing() is not None)  # answer to the on-connect query
    return h, board


def _trigger():
    return types.SimpleNamespace(success=None, message="")


def _lora_status(h):
    rec = _Recorder()
    h.node.lora_status_pub = rec
    h.node.publish_lora_status()
    return json.loads(rec.msgs[-1].data)


def test_set_lora_is_disabled_by_parameter():
    h = Harness(extra_params={"lora_pairing_enabled": False})
    try:
        response = h.node._on_set_lora(_request(sn="AB12030000"), types.SimpleNamespace())
        assert response.result is False
        assert h.node._on_get_lora(None, types.SimpleNamespace()).ok is False
        assert h.node._on_reset_base(None, _trigger()).success is False
        assert PtyReader(h.master).poll(0.2) == b""
    finally:
        h.close()


def test_get_lora_reads_the_stored_pairing_and_band():
    h, board = _lora_harness()
    try:
        response = h.node._on_get_lora(None, types.SimpleNamespace())
        assert response.ok is True
        assert (response.addr, response.channel, response.band) == (0x9527, 24, "0")
        assert response.raw.splitlines() == ["$GNCON,9527,18*4B", "$GLIST,0*59"]
        assert h.node.lora_pairing() == {"addr": 0x9527, "addr_hex": "9527", "channel": 24,
                                          "band": "0", "band_name": "Europe 863 MHz"}
    finally:
        board.stop()
        h.close()


def test_get_lora_without_a_board_reply_is_not_ok():
    h = Harness()
    try:
        response = h.node._on_get_lora(None, types.SimpleNamespace())
        assert response.ok is False and "no $GNCON reply" in response.message
    finally:
        h.close()


def test_set_lora_writes_vendor_bytes_and_verifies_the_read_back():
    h, board = _lora_harness()
    try:
        board.written.clear()
        response = h.node._on_set_lora(_request(addr=38183, channel=40, area="03"),
                                       types.SimpleNamespace())
        assert response.result is True
        assert bytes(board.written) == (b"$GLIST,2*5B\r\n$GNCON,9527,28*48\r\n"
                                        + lora.QUERY_PAIRING.encode() + b"\r\n")
        pairing = h.node.lora_pairing()
        assert (pairing["addr"], pairing["channel"], pairing["band"]) == (38183, 40, "2")
        assert h.node.correction_summary()["lora_pairing"] == "9527,28"
    finally:
        board.stop()
        h.close()


def test_set_lora_vendor_ble_path_derives_the_band_from_the_serial():
    h, board = _lora_harness()
    try:
        board.written.clear()
        response = h.node._on_set_lora(_request(sn="AB12030000", addr=0x1234, channel=75),
                                       types.SimpleNamespace())
        assert response.result is True
        assert bytes(board.written).startswith(
            b"$GLIST,2*5B\r\n" + lora.gncon_command(0x1234, 75).encode() + b"\r\n")
    finally:
        board.stop()
        h.close()


def test_set_lora_is_false_when_the_board_keeps_its_old_pairing():
    h, board = _lora_harness()
    try:
        board.store = False
        response = h.node._on_set_lora(_request(addr=38183, channel=40, area="02"),
                                       types.SimpleNamespace())
        assert response.result is False
        assert h.node.lora_pairing()["channel"] == 24
    finally:
        board.stop()
        h.close()


def test_set_lora_validates_before_writing():
    h, board = _lora_harness()
    try:
        board.written.clear()
        for request in (_request(area="ZZ"), _request(addr=0xFFFF, area="02"),
                        _request(channel=0, area="02"), _request(channel=76, area="02")):
            assert h.node._on_set_lora(request, types.SimpleNamespace()).result is False
        time.sleep(0.1)
        assert bytes(board.written) == b""
    finally:
        board.stop()
        h.close()


def test_reset_base_writes_breset_in_lora_mode():
    h, board = _lora_harness()
    try:
        board.written.clear()
        response = h.node._on_reset_base(None, _trigger())
        assert response.success is True
        _wait(lambda: bytes(board.written) == b"$BRESET*17\r\n")
    finally:
        board.stop()
        h.close()


def test_lora_status_json():
    h, board = _lora_harness()
    try:
        h.send(_cs("GLIST,2"))
        h.send(_cs("GNSTA,3,FINE"))
        h.send(GGA_RTK)
        status = _lora_status(h)
        assert status["pairing"] == {"addr": 0x9527, "addr_hex": "9527", "channel": 24,
                                     "band": "2", "band_name": "Australia/NZ 927 MHz"}
        assert status["ref_station"] == "FINE"
        assert status["nrtk_on"] is False
        assert status["station_id"] == "0451"
        assert status["diff_age_s"] == 0.5
        assert status["fix_type"] == "RTK_FIXED"
        assert status["correction_source"] == "lora"
        assert status["pairing_enabled"] is True
    finally:
        board.stop()
        h.close()


def test_lora_status_before_any_reply():
    h = Harness(extra_params={"lora_pairing_enabled": False})
    try:
        status = _lora_status(h)
        assert status["pairing"] is None and status["ref_station"] is None
        assert status["pairing_enabled"] is False
    finally:
        h.close()


def test_set_lora_and_reset_base_refused_in_ntrip_mode(caster):
    h = Harness(extra_params=_ntrip_params(caster))
    try:
        assert h.node._on_set_lora(_request(area="02"), types.SimpleNamespace()).result is False
        response = h.node._on_reset_base(None, _trigger())
        assert response.success is False and "ntrip" in response.message
    finally:
        h.close()
