"""mow_recorder start/stop + retention logic (pure)."""

import os

from mower_control import mow_recorder as mr


def test_starts_on_leaving_idle_and_stops_after_idle_debounce():
    s = mr.MowSession(stop_after_idle_s=5.0)
    assert s.update('IDLE_DOCKED', 0.0) is None
    assert s.update('UNDOCKING', 1.0) == 'start'
    assert s.update('MOWING', 2.0) is None
    assert s.update('IDLE', 3.0) is None                 # brief idle blip: keep going
    assert s.update('TRANSIT', 4.0) is None
    assert s.update('CHARGING', 10.0) is None
    assert s.update('CHARGING', 14.9) is None
    assert s.update('CHARGING', 15.0) == 'stop'
    assert s.update('CHARGING', 20.0) is None
    assert s.update('EMERGENCY', 21.0) == 'start'        # any non-idle state records


def test_retention_by_count_keeps_newest():
    entries = [('20261001-100000', 10), ('20261003-100000', 10), ('20261002-100000', 10)]
    assert mr.retention_victims(entries, 2, 1e9) == ['20261001-100000']
    assert mr.retention_victims(entries, 3, 1e9) == []


def test_retention_by_size_and_never_the_active_one():
    gb = 1024 ** 3
    entries = [('a1', gb), ('a2', gb), ('a3', gb)]
    assert mr.retention_victims(entries, 10, 2 * gb) == ['a1']
    assert mr.retention_victims(entries, 10, 0.5 * gb, keep='a3') == ['a1', 'a2']
    assert mr.retention_victims([('a1', 3 * gb)], 10, gb, keep='a1') == []


def test_apply_retention_on_disk(tmp_path):
    for i in range(12):
        d = tmp_path / ('20261007-1000%02d' % i)
        d.mkdir()
        (d / 'x').write_bytes(b'0' * 100)
    gone = mr.apply_retention(str(tmp_path), 10, 10 ** 9)
    assert gone == ['20261007-100000', '20261007-100001']
    assert len(os.listdir(tmp_path)) == 10


def test_storage_pick():
    assert mr.pick_storage({'sqlite3'}) == 'sqlite3'
    assert mr.pick_storage({'sqlite3', 'mcap'}) == 'mcap'
