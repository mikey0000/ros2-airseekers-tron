"""wifi_monitor: parsers (fixtures with made-up SSID/BSSIDs), roam tracking, sampler."""

import json
import math
import os
import sys
import types

from mower_control import wifi_logic as wl

FIX = os.path.join(os.path.dirname(__file__), 'fixtures', 'wifi')


def fx(name):
    with open(os.path.join(FIX, name), encoding='utf-8') as f:
        return f.read()


def _import_node():
    try:
        import mower_interfaces.msg  # noqa: F401
        has = hasattr(sys.modules['mower_interfaces.msg'], 'WifiStatus')
    except ImportError:
        has = False
    if not has:
        class WifiStatus:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=None, frame_id='')
        pkg = sys.modules.get('mower_interfaces') or types.ModuleType('mower_interfaces')
        msg = sys.modules.get('mower_interfaces.msg') or types.ModuleType('mower_interfaces.msg')
        msg.WifiStatus = WifiStatus
        pkg.msg = msg
        sys.modules['mower_interfaces'] = pkg
        sys.modules['mower_interfaces.msg'] = msg
    from mower_control import wifi_monitor
    return wifi_monitor


def test_proc_wireless():
    assert wl.parse_proc_wireless(fx('proc_net_wireless.txt'), 'wlan0') == {
        'link_quality': 48.0, 'signal_dbm': -62.0}
    assert wl.parse_proc_wireless(fx('proc_net_wireless.txt'), 'wlan1') is None
    assert wl.parse_proc_wireless('', 'wlan0') is None
    # unsigned 8-bit level from old drivers
    assert wl.parse_proc_wireless(' wlan0: 0000 40. 200. 0 0 0 0 0 0 0', 'wlan0')[
        'signal_dbm'] == -56.0


def test_iw_link_connected():
    d = wl.parse_iw_link(fx('iw_link_connected.txt'))
    assert d['connected'] and d['bssid'] == '02:00:5e:10:20:30'
    assert d['ssid'] == 'example-net' and d['freq_mhz'] == 5180.0
    assert d['signal_dbm'] == -63.0
    assert d['tx_bitrate_mbps'] == 433.3 and d['rx_bitrate_mbps'] == 390.0


def test_iw_link_partial_and_disconnected():
    d = wl.parse_iw_link(fx('iw_link_roamed.txt'))
    assert d['freq_mhz'] == 2437.0 and d['tx_bitrate_mbps'] == 72.2
    assert d['rx_bitrate_mbps'] == 0.0
    assert not wl.parse_iw_link(fx('iw_link_disconnected.txt'))['connected']
    assert not wl.parse_iw_link('')['connected']


def test_station_dump():
    assert wl.parse_station_dump(fx('iw_station_dump.txt')) == {
        'tx_retries': 1873, 'tx_failed': 42}
    assert wl.parse_station_dump('') is None


def test_channel_and_state():
    assert wl.freq_to_channel(2437) == 6 and wl.freq_to_channel(5180) == 36
    assert wl.freq_to_channel(2484) == 14 and wl.freq_to_channel(0) == 0
    assert wl.link_state(True, 'up', -55) == 'good'
    assert wl.link_state(True, 'up', -60) == 'ok'
    assert wl.link_state(True, 'up', -70) == 'ok'
    assert wl.link_state(True, 'up', -75) == 'weak'
    assert wl.link_state(True, 'up', -80) == 'bad'
    assert wl.link_state(False, 'up', None) == 'disconnected'
    assert wl.link_state(False, 'down', None) == 'down'


def test_build_sample_fallbacks():
    s = wl.build_sample('wlan0', fx('proc_net_wireless.txt'),
                        'Connected to 02:00:5e:10:20:30 (on wlan0)\n\tSSID: x\n', 'up\n')
    assert s['connected'] and s['signal_dbm'] == -62.0  # from /proc
    s = wl.build_sample('wlan0', '', '', '')
    assert not s['connected'] and s['signal_dbm'] is None
    assert s['link_state'] == 'disconnected' and s['bssid'] == ''
    s = wl.build_sample('wlan0', '', fx('iw_link_connected.txt'), 'down')
    assert not s['connected'] and s['link_state'] == 'down'


def _s(link, station=None):
    return wl.build_sample('wlan0', fx('proc_net_wireless.txt'), fx(link), 'up', station)


def test_roam_tracker_sequence():
    t = wl.RoamTracker()
    ev = t.update(_s('iw_link_connected.txt'))
    assert [e['event'] for e in ev] == ['connect'] and t.roam_count == 0
    assert t.update(_s('iw_link_connected.txt')) == []
    ev = t.update(_s('iw_link_roamed.txt'))
    assert [e['event'] for e in ev] == ['roam'] and t.roam_count == 1
    assert ev[0]['old']['bssid'] == '02:00:5e:10:20:30'
    assert ev[0]['new'] == {'bssid': '02:00:5e:10:20:41', 'freq_mhz': 2437.0,
                            'signal_dbm': -71.0}
    ev = t.update(_s('iw_link_disconnected.txt'))
    assert [e['event'] for e in ev] == ['disconnect']
    assert ev[0]['old']['bssid'] == '02:00:5e:10:20:41' and ev[0]['new']['bssid'] == ''
    assert ev[0]['ssid'] == 'example-net'
    assert t.update(_s('iw_link_disconnected.txt')) == []
    ev = t.update(_s('iw_link_connected.txt'))
    # reconnect to a different AP is a reconnect, not a roam
    assert [e['event'] for e in ev] == ['reconnect'] and t.roam_count == 1
    json.dumps(ev)


def test_tracker_starts_disconnected():
    t = wl.RoamTracker()
    assert t.update(_s('iw_link_disconnected.txt')) == []
    assert [e['event'] for e in t.update(_s('iw_link_connected.txt'))] == ['connect']


def test_sampler_station_period_and_missing_iw():
    wm = _import_node()
    calls = []
    now = [0.0]

    def run(argv, _timeout):
        calls.append(argv[-1])
        return fx('iw_station_dump.txt') if argv[-1] == 'dump' else fx('iw_link_connected.txt')

    def read(path):
        return 'up\n' if path.endswith('operstate') else fx('proc_net_wireless.txt')

    s = wm.WifiSampler('wlan0', 10.0, run=run, read=read, clock=lambda: now[0])
    for k in range(12):
        now[0] = float(k)
        sample = s.sample()
    assert calls.count('dump') == 2 and calls.count('link') == 12
    assert sample['tx_retries'] == 1873 and sample['connected']

    gone = wm.WifiSampler('wlan9', run=lambda *_: '', read=lambda _p: '')
    sample = gone.sample()
    assert not sample['connected'] and sample['link_state'] == 'disconnected'


def test_run_cmd_missing_binary():
    wm = _import_node()
    assert wm.run_cmd(['/nonexistent/iw-binary', 'dev'], 0.5) == ''
    assert wm.read_file('/nonexistent/operstate') == ''


def test_fill_status_and_describe():
    wm = _import_node()
    t = wl.RoamTracker()
    t.update(_s('iw_link_connected.txt'))
    ev = t.update(_s('iw_link_roamed.txt', {'tx_retries': 5, 'tx_failed': 1}))
    msg = wm.fill_status(wm.WifiStatus(), _s('iw_link_roamed.txt', {'tx_retries': 5,
                                                                    'tx_failed': 1}),
                         t.roam_count)
    assert msg.connected and msg.bssid == '02:00:5e:10:20:41' and msg.roam_count == 1
    assert msg.tx_retries == 5 and msg.link_state == 'weak'
    off = wm.fill_status(wm.WifiStatus(), _s('iw_link_disconnected.txt'), 1)
    assert not off.connected and math.isnan(off.signal_dbm)
    line = wm.describe_event(ev[0])
    assert 'roam' in line and '02:00:5e:10:20:41 2437 MHz -71 dBm' in line


def test_bcmdhd_shapes():
    """Shapes captured from the mower's bcmdhd on 2026-11-01 (values made up)."""
    proc = fx('bcmdhd_proc_net_wireless.txt')
    assert wl.parse_proc_wireless(proc, 'wlan0') == {'link_quality': 3.0, 'signal_dbm': -73.0}
    assert wl.parse_proc_wireless(proc, 'wlan1') == {'link_quality': 0.0, 'signal_dbm': 0.0}
    d = wl.parse_iw_link(fx('bcmdhd_iw_link.txt'))
    assert (d['bssid'], d['freq_mhz'], d['signal_dbm']) == ('02:00:5e:33:44:55', 5500.0, -72.0)
    assert (d['tx_bitrate_mbps'], d['rx_bitrate_mbps']) == (39.0, 54.0)
    assert wl.parse_station_dump(fx('bcmdhd_iw_station_dump.txt')) == {
        'tx_retries': 4321, 'tx_failed': 7654}
    s = wl.build_sample('wlan0', proc, fx('bcmdhd_iw_link.txt'), 'up')
    assert s['link_state'] == 'weak' and wl.freq_to_channel(5500) == 100


def test_signal_chain_form():
    d = wl.parse_iw_link('Connected to 02:00:5e:00:00:01 (on wlan0)\n'
                         '\tsignal: -69 [-69, -71] dBm\n'
                         '\ttx bitrate: 97.5 MBit/s VHT-MCS 4 80MHz VHT-NSS 1\n')
    assert d['signal_dbm'] == -69.0 and d['tx_bitrate_mbps'] == 97.5
