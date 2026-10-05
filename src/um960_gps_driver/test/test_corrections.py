"""Correction-source integration: pty in place of the UM960, fake NTRIP caster.

Checks that RTCM from the caster lands on the receiver port only in ntrip mode,
that the receiver GGA reaches the caster, the /fix_status tokens, the
/gps/corrections JSON, the vendor ntrip.yaml fallback and the set_lora stub.
"""

import json
import os
import select
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


def test_lora_default_is_read_only_and_reports_receiver_diff_age():
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
        assert reader.poll(0.2) == b"", "lora mode must never write to the receiver"
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


def test_ntrip_end_to_end_rtcm_reaches_the_port_and_gga_reaches_the_caster(caster):
    h = Harness(extra_params=_ntrip_params(caster))
    try:
        reader = PtyReader(h.master)
        h.send(GGA_RTK)
        _wait(lambda: reader.poll() and sample_frame(1077, 1) in bytes(reader.data))
        written = bytes(reader.data)
        # The rover board is switched to network RTK before any RTCM goes out.
        assert written.startswith(lora.NRTK_ON.encode() + b"\r\n")
        assert sample_frame(1005, 0) in written
        _wait(lambda: any(u.startswith("$GNGGA,101530.00,4807.038") for u in caster.uploads))
        status = _status(h)
        assert "corr_src=ntrip corr=streaming corr_flow=active" in status
        assert "corr_rate=" in status and "corr_age=" in status
        diag = _corrections(h)
        assert diag["state"] == "streaming" and diag["frames"] >= 2
        assert diag["mountpoint"] == caster.mountpoint and "password" not in json.dumps(diag)
        assert diag["serial_bytes_written"] > 0
    finally:
        h.close()


def test_ntrip_synthesizes_gga_from_bestnav_and_reasserts_nrtk(caster):
    h = Harness(extra_params=_ntrip_params(caster))
    try:
        reader = PtyReader(h.master)
        h.send(_bestnav_frame(lat=48.2, lon=11.6))
        _wait(lambda: any(u.startswith("$GPGGA") for u in caster.uploads))
        gga = [u for u in caster.uploads if u.startswith("$GPGGA")][-1]
        assert ",4812.0000000,N,01136.0000000,E," in gga
        # Rover board says it fell back to LoRa: the node asks again.
        h.node._rover_nrtk_sent = 0.0
        reader.poll(0.1)
        reader.data.clear()
        h.send(_cs("GNRTK,OFF"))
        _wait(lambda: lora.NRTK_ON.encode() in reader.poll())
        assert h.node.correction_summary()["rover_nrtk"] == "OFF"
    finally:
        h.close()


def test_vendor_ntrip_yaml_enables_ntrip_when_source_unset(caster, tmp_path):
    path = tmp_path / "ntrip.yaml"
    path.write_text(
        "ntrip_enable: true\nntrip_ip: \"%s\"\nntrip_port: %d\nntrip_user: \"%s\"\n"
        "ntrip_passwd: \"%s\"\nntrip_mountpoint: \"%s\"\n"
        % (caster.host, caster.port, caster.user, caster.password, caster.mountpoint))
    h = Harness(extra_params={"ntrip_config_file": str(path)})
    try:
        assert h.node.correction_source == "ntrip"
        assert h.node.ntrip_config_source == str(path)
        reader = PtyReader(h.master)
        _wait(lambda: sample_frame(1005, 0) in reader.poll())
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


def test_set_lora_is_a_stub_by_default():
    h = Harness()
    try:
        response = h.node._on_set_lora(_request(sn="AB12030000"), types.SimpleNamespace())
        assert response.result is False
        assert PtyReader(h.master).poll(0.2) == b""
    finally:
        h.close()


def test_set_lora_writes_vendor_commands_when_enabled():
    h = Harness(extra_params={"lora_pairing_enabled": True})
    try:
        reader = PtyReader(h.master)
        _wait(lambda: lora.QUERY_PAIRING.encode() in reader.poll())  # on-connect query
        reader.data.clear()
        response = h.node._on_set_lora(_request(sn="AB12030000", addr=0x1234, channel=75),
                                       types.SimpleNamespace())
        assert response.result is True
        assert reader.poll(0.2) == b"$GLIST,2*5B\r\n" + lora.gncon_command(0x1234, 75).encode() \
            + b"\r\n"
        h.send(_cs("GNCON,1234,4B"))
        assert h.node.correction_summary()["lora_pairing"] == "1234,4B"
        bad = h.node._on_set_lora(_request(area="ZZ"), types.SimpleNamespace())
        assert bad.result is False
    finally:
        h.close()


def test_set_lora_refused_in_ntrip_mode(caster):
    h = Harness(extra_params=_ntrip_params(caster, lora_pairing_enabled=True))
    try:
        assert h.node._on_set_lora(_request(), types.SimpleNamespace()).result is False
    finally:
        h.close()
