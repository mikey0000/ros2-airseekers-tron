"""External rain verdict: payload parsing and expiry (mower_mission/external_rain.py)."""

import pytest

from mower_mission.external_rain import ExternalRain, parse_rain_payload


@pytest.mark.parametrize('payload', ['yes', 'YES', ' on\n', 'true', 'rain', 'Wet', '1', b'yes'])
def test_wet_words(payload):
    assert parse_rain_payload(payload) is True


@pytest.mark.parametrize('payload', ['no', 'Off', 'false', 'dry', '0', b' no '])
def test_dry_words(payload):
    assert parse_rain_payload(payload) is False


@pytest.mark.parametrize('payload', ['', '   ', 'unknown', 'unavailable', 'maybe', '2',
                                     b'\xff', None, 1])
def test_anything_else_is_ignored(payload):
    assert parse_rain_payload(payload) is None


def test_no_data_is_not_wet():
    r = ExternalRain()
    assert r.state(0.0) == 'no_data' and not r.wet(0.0)


def test_wet_until_it_expires():
    r = ExternalRain(max_age_s=120.0)
    r.update(True, 100.0)
    assert r.wet(100.0) and r.wet(220.0)
    assert r.state(220.1) == 'expired' and not r.wet(220.1)


def test_republish_refreshes_the_expiry():
    r = ExternalRain(max_age_s=60.0)
    r.update(True, 0.0)
    r.update(True, 50.0)
    assert r.wet(100.0)


def test_zero_max_age_never_expires():
    r = ExternalRain(max_age_s=0.0)
    r.update(True, 0.0)
    assert r.wet(1e9)


def test_dry_verdict_clears_wet():
    r = ExternalRain()
    r.update(True, 0.0)
    r.update(False, 1.0)
    assert r.state(1.0) == 'dry' and not r.wet(1.0)


def test_disabled_ignores_a_wet_verdict():
    r = ExternalRain(enabled=False)
    r.update(True, 0.0)
    assert r.state(0.0) == 'disabled' and not r.wet(0.0)


# --- mission_node wiring (needs rclpy + the message packages; skipped elsewhere) ---------------

class _Clock:
    def __init__(self):
        self.t = 1000.0

    def now(self):
        return self.t


class _Log:
    def info(self, *_a):
        pass


def _stub(enabled=True, max_age_s=600.0):
    pytest.importorskip('rclpy')
    pytest.importorskip('mower_interfaces.msg')
    pytest.importorskip('mowgli_interfaces.action')
    import threading
    from types import SimpleNamespace

    from mower_mission.mission_node import MissionNode

    class Stub:
        _refresh_rain = MissionNode._refresh_rain
        _set_external_rain = MissionNode._set_external_rain
        _on_external_rain = MissionNode._on_external_rain
        _srv_external_rain = MissionNode._srv_external_rain

        def get_logger(self):
            return _Log()

    s = Stub()
    s._lock = threading.RLock()
    s._clk = _Clock()
    s._hw_rain = s._base_rain = False
    s._ext_rain = ExternalRain(enabled, max_age_s)
    s._ext_rain_state = None
    s.fsm = SimpleNamespace(inputs=SimpleNamespace(rain=False))
    return s


def test_node_service_feeds_the_fsm_rain_input_and_expires():
    from std_srvs.srv import SetBool
    s = _stub(max_age_s=600.0)
    res = s._srv_external_rain(SetBool.Request(data=True), SetBool.Response())
    assert res.success and s.fsm.inputs.rain
    s._clk.t += 601.0
    s._refresh_rain()                      # what the 10 Hz tick does
    assert not s.fsm.inputs.rain


def test_node_topic_dry_never_hides_the_mowers_own_sensor():
    from std_msgs.msg import Bool
    s = _stub()
    s._hw_rain = True
    s._on_external_rain(Bool(data=False))
    assert s.fsm.inputs.rain


def test_node_disabled_reports_failure_and_ignores_wet():
    from std_srvs.srv import SetBool
    s = _stub(enabled=False)
    res = s._srv_external_rain(SetBool.Request(data=True), SetBool.Response())
    assert not res.success and 'use_external_rain' in res.message
    assert not s.fsm.inputs.rain
