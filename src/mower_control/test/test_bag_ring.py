"""bag_ring: rolling rosbag ring logic (scan, prune, pin, incident detection). Pure, no ROS."""

import json
import os

import pytest

from mower_control import bag_recorder
from mower_control import bag_ring as br

S1 = '20261009-101500'
S2 = '20261009-111500'


def mk(root, session, names, size=10):
    d = os.path.join(str(root), session)
    os.makedirs(d, exist_ok=True)
    for n in names:
        with open(os.path.join(d, n), 'wb') as f:
            f.write(b'x' * size)
    return d


def states(splits):
    return {s.index: s.state for s in splits}


# --------------------------------------------------------------------------- scan_session
def test_active_newest_plain_split_is_active(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_0.db3.zstd', S1 + '_1.db3.zstd', S1 + '_2.db3'])
    splits, other = br.scan_session(d, active=True)
    assert states(splits) == {0: br.CLOSED, 1: br.CLOSED, 2: br.ACTIVE}
    assert other == 0


def test_plain_with_zstd_sibling_is_compressing(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_0.db3.zstd', S1 + '_1.db3', S1 + '_1.db3.zstd', S1 + '_2.db3'])
    splits, _ = br.scan_session(d, active=True)
    assert states(splits) == {0: br.CLOSED, 1: br.COMPRESSING, 2: br.ACTIVE}


def test_compressing_sibling_also_in_inactive_session(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_1.db3', S1 + '_1.db3.zstd'])
    splits, _ = br.scan_session(d, active=False)
    assert states(splits) == {1: br.COMPRESSING}


def test_older_plain_split_of_active_session_is_compressing(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_0.db3.zstd', S1 + '_1.db3', S1 + '_2.db3'])
    splits, _ = br.scan_session(d, active=True, compressed=True)
    assert states(splits) == {0: br.CLOSED, 1: br.COMPRESSING, 2: br.ACTIVE}


def test_older_plain_split_without_compression_is_closed(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_0.db3', S1 + '_1.db3', S1 + '_2.db3'])
    splits, _ = br.scan_session(d, active=True, compressed=False)
    assert states(splits) == {0: br.CLOSED, 1: br.CLOSED, 2: br.ACTIVE}


def test_inactive_session_plain_splits_are_closed(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_0.db3', S1 + '_1.db3'])
    for compressed in (True, False):
        splits, _ = br.scan_session(d, active=False, compressed=compressed)
        assert states(splits) == {0: br.CLOSED, 1: br.CLOSED}


def test_sidecars_counted_in_size_and_paths(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_0.db3', S1 + '_0.db3-wal', S1 + '_0.db3-shm'], size=7)
    splits, other = br.scan_session(d, active=True)
    assert len(splits) == 1
    sp = splits[0]
    assert sp.size == 21 and other == 0
    assert sorted(os.path.basename(p) for p in sp.paths) == [
        S1 + '_0.db3', S1 + '_0.db3-shm', S1 + '_0.db3-wal']
    assert sp.state == br.ACTIVE
    assert os.path.basename(sp.data_path()) == S1 + '_0.db3'


def test_data_path_prefers_zstd(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_1.db3', S1 + '_1.db3.zstd'])
    splits, _ = br.scan_session(d, active=False)
    assert splits[0].data_path().endswith('.db3.zstd')


def test_other_files_counted_as_other(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_0.db3.zstd', 'metadata.yaml'], size=5)
    splits, other = br.scan_session(d, active=False)
    assert len(splits) == 1 and splits[0].size == 5
    assert other == 5


def test_scan_session_missing_dir(tmp_path):
    assert br.scan_session(str(tmp_path / 'nope'), True) == ([], 0)


def test_splits_sorted_numerically(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_%d.db3.zstd' % i for i in (10, 2, 1, 0)])
    splits, _ = br.scan_session(d, active=False)
    assert [s.index for s in splits] == [0, 1, 2, 10]


def test_list_sessions_ignores_non_sessions_and_sorts(tmp_path):
    for n in (S2, S1, '20261009-101500.1', 'incidents', '2026'):
        os.makedirs(str(tmp_path / n))
    (tmp_path / '20261008-000000').write_text('a file, not a dir')
    (tmp_path / 'recorder.log').write_text('x')
    assert br.list_sessions(str(tmp_path)) == [S1, S1 + '.1', S2]
    assert br.list_sessions(str(tmp_path / 'missing')) == []


def test_scan_marks_only_active_session(tmp_path):
    mk(tmp_path, S1, [S1 + '_0.db3', S1 + '_1.db3'])
    mk(tmp_path, S2, [S2 + '_0.db3', S2 + '_1.db3'])
    out = br.scan(str(tmp_path), active_session=S2)
    assert [n for n, _s, _o in out] == [S1, S2]
    assert states(out[0][1]) == {0: br.CLOSED, 1: br.CLOSED}
    assert states(out[1][1]) == {0: br.COMPRESSING, 1: br.ACTIVE}
    assert br.total_bytes(out) == 40


# --------------------------------------------------------------------------- plan_prune
def sp(session, index, size, state=br.CLOSED):
    s = br.Split(session, index)
    s.size = size
    s.state = state
    s.paths = ['/nowhere/%s_%d.db3.zstd' % (session, index)]
    return s


def ring_two_sessions():
    old = (S1, [sp(S1, i, 10) for i in range(3)], 0)
    new = (S2, [sp(S2, 0, 10), sp(S2, 1, 10), sp(S2, 2, 10, br.COMPRESSING),
                sp(S2, 3, 10, br.ACTIVE)], 0)
    return [old, new]


def test_prune_oldest_session_lowest_index_first_and_stops_early():
    sessions = ring_two_sessions()           # 70 bytes
    victims, starved = br.plan_prune(sessions, S2, 50, 10 ** 9, 0)
    assert [(v.session, v.index) for v in victims] == [(S1, 0), (S1, 1)]
    assert not starved


def test_prune_nothing_within_limits():
    victims, starved = br.plan_prune(ring_two_sessions(), S2, 1000, 10 ** 9, 100)
    assert victims == [] and not starved


def test_prune_free_floor_alone_triggers():
    victims, starved = br.plan_prune(ring_two_sessions(), S2, 10 ** 9, 100, 125)
    assert [(v.session, v.index) for v in victims if not isinstance(v, str)] == [
        (S1, 0), (S1, 1), (S1, 2)]
    assert not starved                       # 100 + 30 >= 125


def test_prune_never_deletes_active_compressing_or_active_split0():
    sessions = ring_two_sessions()
    victims, starved = br.plan_prune(sessions, S2, 0, 0, 10 ** 9)
    names = [v for v in victims if isinstance(v, str)]
    splits = [v for v in victims if not isinstance(v, str)]
    assert names == [S1]
    got = {(v.session, v.index) for v in splits}
    assert got == {(S1, 0), (S1, 1), (S1, 2), (S2, 1)}
    assert (S2, 0) not in got and (S2, 2) not in got and (S2, 3) not in got
    assert starved                           # free floor unreachable


def test_prune_deletes_split0_of_old_sessions_and_returns_dir_name():
    sessions = [(S1, [sp(S1, 0, 10), sp(S1, 1, 10)], 4)]
    victims, starved = br.plan_prune(sessions, None, 0, 0, 0)
    assert [type(v) for v in victims] == [br.Split, br.Split, str]
    assert victims[-1] == S1
    assert not starved                       # total 24 -> 0


def test_prune_dir_not_dropped_when_session_partially_deleted():
    sessions = [(S1, [sp(S1, 0, 10), sp(S1, 1, 10)], 4)]
    victims, _ = br.plan_prune(sessions, None, 14, 10 ** 9, 0)
    assert len(victims) == 1 and not isinstance(victims[0], str)


def test_prune_dir_not_dropped_when_session_has_compressing_split():
    sessions = [(S1, [sp(S1, 0, 10), sp(S1, 1, 10, br.COMPRESSING)], 0)]
    victims, starved = br.plan_prune(sessions, None, 0, 0, 0)
    assert [v.index for v in victims] == [0] and not starved   # over cap only


def test_prune_starved_when_free_floor_unreachable():
    sessions = [(S2, [sp(S2, 0, 10), sp(S2, 1, 10, br.ACTIVE)], 0)]
    victims, starved = br.plan_prune(sessions, S2, 10 ** 9, 5, 100)
    assert victims == [] and starved


def test_prune_over_cap_with_enough_free_is_not_starved():
    sessions = [(S2, [sp(S2, 0, 100), sp(S2, 1, 10, br.ACTIVE)], 0)]
    victims, starved = br.plan_prune(sessions, S2, 10, 10 ** 9, 100)
    assert victims == [] and not starved


def test_prune_protect_respected():
    sessions = ring_two_sessions()
    victims, _ = br.plan_prune(sessions, S2, 0, 10 ** 9, 0, protect={(S1, 0), (S2, 1)})
    got = {(v.session, v.index) for v in victims if not isinstance(v, str)}
    assert got == {(S1, 1), (S1, 2)}
    assert S1 not in victims                 # dir kept: split 0 still protected


def test_prune_protect_with_floor_unreachable_is_starved():
    sessions = [(S1, [sp(S1, 0, 10)], 0)]
    victims, starved = br.plan_prune(sessions, None, 10 ** 9, 0, 100, protect={(S1, 0)})
    assert victims == [] and starved


def test_pending_splits(tmp_path):
    root = live_ring(tmp_path)
    pl = br.PinPlanner(prev=2, post=1)
    assert pl.pending_splits() == set()
    pl.trigger('/pins/a', br.scan(root, S1), S1, 0.0)
    assert pl.pending_splits() == {(S1, 5), (S1, 6)}
    pl.due(br.scan(root, S1), 10 ** 6)
    assert pl.pending_splits() == set()


# --------------------------------------------------------------------------- apply_prune
def test_apply_prune_on_disk(tmp_path):
    old = mk(tmp_path, S1, [S1 + '_0.db3.zstd', S1 + '_1.db3.zstd', 'metadata.yaml'])
    new = mk(tmp_path, S2, [S2 + '_0.db3.zstd', S2 + '_1.db3', S2 + '_1.db3-wal'])
    sessions = br.scan(str(tmp_path), active_session=S2)
    victims, _ = br.plan_prune(sessions, S2, 0, 10 ** 9, 0)
    freed = br.apply_prune(str(tmp_path), victims)
    assert freed == 20
    assert not os.path.exists(old)           # emptied session dir incl. metadata.yaml
    assert sorted(os.listdir(new)) == [S2 + '_0.db3.zstd', S2 + '_1.db3', S2 + '_1.db3-wal']


def test_apply_prune_partial_session_keeps_dir(tmp_path):
    d = mk(tmp_path, S1, [S1 + '_0.db3.zstd', S1 + '_1.db3.zstd', 'metadata.yaml'])
    sessions = br.scan(str(tmp_path))
    victims, _ = br.plan_prune(sessions, None, 25, 10 ** 9, 0)
    br.apply_prune(str(tmp_path), victims)
    assert sorted(os.listdir(d)) == [S1 + '_1.db3.zstd', 'metadata.yaml']


def test_apply_prune_tolerates_missing_files(tmp_path):
    s = sp(S1, 0, 10)
    assert br.apply_prune(str(tmp_path), [s, 'gone']) == 10


# --------------------------------------------------------------------------- PinPlanner
def live_ring(tmp_path, active_idx=5):
    names = [S1 + '_%d.db3.zstd' % i for i in range(active_idx)] + [S1 + '_%d.db3' % active_idx]
    mk(tmp_path, S1, names)
    return str(tmp_path)


def test_trigger_while_split5_active(tmp_path):
    root = live_ring(tmp_path)
    pl = br.PinPlanner(prev=2, post=1)
    now_list = pl.trigger('/pins/a', br.scan(root, S1), S1, 100.0)
    assert [s.index for s in now_list] == [0, 3, 4]
    assert sorted(e[2] for e in pl.pending) == [5, 6]
    assert pl.pending_dirs() == {'/pins/a'}


def test_due_only_once_closed(tmp_path):
    root = live_ring(tmp_path)
    pl = br.PinPlanner(prev=2, post=1)
    pl.trigger('/pins/a', br.scan(root, S1), S1, 100.0)
    assert pl.due(br.scan(root, S1), 110.0) == []
    d = os.path.join(root, S1)
    # split 5 rolled: compressing (both files)
    open(os.path.join(d, S1 + '_5.db3.zstd'), 'wb').write(b'z')
    open(os.path.join(d, S1 + '_6.db3'), 'wb').write(b'y')
    assert pl.due(br.scan(root, S1), 120.0) == []
    os.remove(os.path.join(d, S1 + '_5.db3'))
    due = pl.due(br.scan(root, S1), 130.0)
    assert [(p, s.index) for p, s in due] == [('/pins/a', 5)]
    assert [e[2] for e in pl.pending] == [6]
    # 6 closes after 7 appears and is compressed
    open(os.path.join(d, S1 + '_6.db3.zstd'), 'wb').write(b'z')
    os.remove(os.path.join(d, S1 + '_6.db3'))
    open(os.path.join(d, S1 + '_7.db3'), 'wb').write(b'y')
    due = pl.due(br.scan(root, S1), 140.0)
    assert [s.index for _p, s in due] == [6]
    assert pl.pending == [] and pl.pending_dirs() == set()


def test_pending_expires_after_ttl(tmp_path):
    root = live_ring(tmp_path)
    pl = br.PinPlanner(prev=2, post=1, pending_ttl_s=900.0)
    pl.trigger('/pins/a', br.scan(root, S1), S1, 100.0)
    assert pl.due(br.scan(root, S1), 999.0) == []
    assert len(pl.pending) == 2
    assert pl.due(br.scan(root, S1), 1000.1) == []
    assert pl.pending == []


def test_trigger_recorder_not_running_pins_last_prev_and_split0(tmp_path):
    # session exists but nothing is being written: every split closed
    mk(tmp_path, S1, [S1 + '_%d.db3.zstd' % i for i in range(5)])
    pl = br.PinPlanner(prev=2, post=1)
    now_list = pl.trigger('/pins/a', br.scan(str(tmp_path), None), S1, 50.0)
    assert [s.index for s in now_list] == [0, 3, 4]
    # nothing live: the 'current/post' splits do not exist, they wait for the TTL
    assert sorted(e[2] for e in pl.pending) == [5, 6]


def test_trigger_with_no_session_data_pins_nothing():
    pl = br.PinPlanner()
    assert pl.trigger('/pins/a', [], None, 0.0) == []
    assert pl.due([], 1.0) == []
    assert pl.due([], 10 ** 6) == []
    assert pl.pending == []


def test_trigger_early_split_dedups_split0(tmp_path):
    root = live_ring(tmp_path, active_idx=1)
    now_list = br.PinPlanner(prev=2, post=1).trigger('/p', br.scan(root, S1), S1, 0.0)
    assert [s.index for s in now_list] == [0]


# --------------------------------------------------------------------------- pin_split
def closed_split(tmp_path, name=S1 + '_3.db3.zstd', data=b'payload'):
    d = mk(tmp_path, S1, [])
    with open(os.path.join(d, name), 'wb') as f:
        f.write(data)
    splits, _ = br.scan_session(d, active=False)
    return splits[0]


def test_pin_split_hardlinks(tmp_path):
    split = closed_split(tmp_path)
    pin = str(tmp_path / 'inc' / 'x')
    dst = br.pin_split(split, pin)
    assert os.path.basename(dst) == '%s__%s_3.db3.zstd' % (S1, S1)
    src = split.data_path()
    assert os.path.samefile(src, dst)
    assert os.stat(dst).st_nlink == 2
    with open(dst, 'rb') as f:
        assert f.read() == b'payload'


def test_pin_split_falls_back_to_copy(tmp_path):
    split = closed_split(tmp_path)

    def bad_link(_s, _d):
        raise OSError('cross-device')

    dst = br.pin_split(split, str(tmp_path / 'pin'), link=bad_link)
    assert os.stat(dst).st_nlink == 1
    assert not os.path.samefile(split.data_path(), dst)
    with open(dst, 'rb') as f:
        assert f.read() == b'payload'


def test_pin_split_none_when_copy_also_fails(tmp_path):
    split = closed_split(tmp_path)

    def bad(_s, _d):
        raise OSError('nope')

    assert br.pin_split(split, str(tmp_path / 'pin'), link=bad, copy=bad) is None


def test_pin_split_idempotent(tmp_path):
    split = closed_split(tmp_path)
    pin = str(tmp_path / 'pin')
    first = br.pin_split(split, pin)
    calls = []
    second = br.pin_split(split, pin, link=lambda *a: calls.append(a),
                          copy=lambda *a: calls.append(a))
    assert first == second and calls == []
    assert os.stat(first).st_nlink == 2


def test_pin_split_none_without_data_file(tmp_path):
    s = br.Split(S1, 3)
    assert br.pin_split(s, str(tmp_path / 'pin')) is None
    s.paths = [str(tmp_path / 'x.db3-wal')]
    assert br.pin_split(s, str(tmp_path / 'pin')) is None
    assert not (tmp_path / 'pin').exists()


def test_write_pin_info_atomic_json(tmp_path):
    pin = str(tmp_path / 'a' / 'b')
    br.write_pin_info(pin, {'reason': 'x', 'n': 3})
    assert sorted(os.listdir(pin)) == ['incident.json']      # no .tmp left behind
    with open(os.path.join(pin, 'incident.json'), encoding='utf-8') as f:
        assert json.load(f) == {'reason': 'x', 'n': 3}
    br.write_pin_info(pin, {'reason': 'y'})
    with open(os.path.join(pin, 'incident.json'), encoding='utf-8') as f:
        assert json.load(f) == {'reason': 'y'}


# --------------------------------------------------------------------------- retention
def test_incident_retention_by_count_oldest_first():
    e = [('c', 1), ('a', 1), ('b', 1), ('d', 1)]
    assert br.plan_incident_retention(e, 2, 10 ** 9) == ['a', 'b']
    assert br.plan_incident_retention(e, 4, 10 ** 9) == []


def test_incident_retention_by_bytes():
    e = [('a', 100), ('b', 100), ('c', 100)]
    assert br.plan_incident_retention(e, 10, 250) == ['a']
    assert br.plan_incident_retention(e, 10, 100) == ['a', 'b']
    assert br.plan_incident_retention(e, 10, 0) == ['a', 'b', 'c']


def test_incident_retention_respects_keep():
    e = [('a', 100), ('b', 100), ('c', 100)]
    assert br.plan_incident_retention(e, 1, 10 ** 9, keep={'a'}) == ['b', 'c'][:2]
    assert br.plan_incident_retention(e, 1, 10 ** 9, keep=('a', 'b', 'c')) == []


# --------------------------------------------------------------------------- detector
def det(**kw):
    return br.IncidentDetector(bag_recorder.DEFAULT_PIN_KINDS, **kw)


def test_mission_kind_filter():
    d = det()
    assert d.on_mission_incident(json.dumps({'kind': 'detour'}), 0.0) is None
    r = d.on_mission_incident(json.dumps({'kind': 'stuck', 'x': 1}), 1.0)
    assert r[0] == 'mission_stuck' and '"stuck"' in r[1]
    assert d.suppressed == 0


@pytest.mark.parametrize('text', ['not json', '', '[1, 2]', 'null', '"s"', '{"nokind": 1}'])
def test_mission_invalid_json_ignored(text):
    d = det()
    assert d.on_mission_incident(text, 0.0) is None
    assert d.suppressed == 0


def test_detour_does_not_start_cooldown():
    d = det()
    assert d.on_mission_incident('{"kind": "detour"}', 0.0) is None
    assert d.on_mission_incident('{"kind": "stuck"}', 0.1) is not None


def test_state_edge_into_emergency_fires_once_not_on_first_message():
    d = det()
    assert d.on_state('EMERGENCY', 0.0) is None          # first message: no edge
    assert d.on_state('EMERGENCY', 100.0) is None
    assert d.on_state('IDLE', 101.0) is None
    r = d.on_state('EMERGENCY', 200.0)
    assert r == ('state_emergency', 'from IDLE')
    assert d.on_state('EMERGENCY', 300.0) is None


def test_emergency_rising_edge_only():
    d = det()
    assert d.on_emergency(False, '', 0.0) is None
    r = d.on_emergency(True, 'bumper hit', 100.0)
    assert r == ('emergency', 'bumper hit')
    assert d.on_emergency(True, 'bumper hit', 200.0) is None
    assert d.on_emergency(False, '', 300.0) is None
    assert d.on_emergency(True, 'again', 400.0)[0] == 'emergency'


def test_crash_dirs_baseline_then_new_dir():
    d = det()
    assert d.on_crash_dirs(['old_1'], 0.0) is None       # baseline
    assert d.on_crash_dirs(['old_1'], 100.0) is None
    r = d.on_crash_dirs(['old_1', '20261009-120000_node_down'], 200.0)
    assert r[0] == 'crash_node_down'
    assert r[1] == '20261009-120000_node_down'
    assert d.on_crash_dirs(['old_1', '20261009-120000_node_down'], 300.0) is None


def test_crash_reason_is_suffix_after_first_underscore():
    d = det()
    d.on_crash_dirs([], 0.0)
    r = d.on_crash_dirs(['black_box_lethal'], 100.0)
    assert r[0] == 'crash_box_lethal'
    r = d.on_crash_dirs(['black_box_lethal', 'plain'], 200.0)
    assert r[0] == 'crash_plain'                         # no underscore: whole name


def test_cooldown_suppresses_and_counts():
    d = det(cooldown_s=30.0)
    assert d.on_emergency(True, 'a', 100.0) is not None
    assert d.on_mission_incident('{"kind": "stuck"}', 110.0) is None
    assert d.on_mission_incident('{"kind": "stuck"}', 129.9) is None
    assert d.suppressed == 2
    assert d.on_mission_incident('{"kind": "stuck"}', 130.0) is not None
    assert d.suppressed == 2


def test_suppressed_edge_is_consumed():
    d = det(cooldown_s=30.0)
    d.on_mission_incident('{"kind": "stuck"}', 0.0)
    assert d.on_emergency(True, 'x', 5.0) is None        # suppressed
    assert d.on_emergency(True, 'x', 50.0) is None       # no new edge afterwards
    assert d.suppressed == 1


def test_manual_bypasses_cooldown():
    d = det(cooldown_s=30.0)
    assert d.on_emergency(True, 'a', 100.0) is not None
    assert d.manual('button', 101.0) == ('manual', 'button')
    assert d.suppressed == 0
    # manual restarts the cooldown window
    assert d.on_mission_incident('{"kind": "stuck"}', 102.0) is None


def test_reasons_sanitized_and_detail_truncated():
    d = br.IncidentDetector(['we ird/kind!!'])
    r = d.on_mission_incident(json.dumps({'kind': 'we ird/kind!!', 'p': 'x' * 900}), 0.0)
    assert r[0] == 'mission_we_ird_kind'
    assert len(r[1]) == 500


def test_safe_reason():
    assert br.safe_reason('a b/c') == 'a_b_c'
    assert br.safe_reason('../../etc') == '.._.._etc'.strip('_')
    assert '/' not in br.safe_reason('../../etc')
    assert br.safe_reason('') == 'incident'
    assert br.safe_reason('///') == 'incident'
    assert len(br.safe_reason('x' * 200)) == 48
    assert br.safe_reason(None) == 'None'


def test_pin_dir_name_and_session_name():
    import time
    t = time.mktime((2026, 10, 9, 10, 15, 0, 0, 0, -1))
    assert br.session_name(t) == '20261009-101500'
    assert br.pin_dir_name(t, 'mission stuck') == '20261009-101500_mission_stuck'
    assert br.SESSION_RE.match(br.session_name(t))


# --------------------------------------------------------------------------- record_cmd
def test_record_cmd_defaults():
    cmd = br.record_cmd('/b/s', ['/tf', '/odom'], 60, 1048576)
    assert cmd[:3] == ['ros2', 'bag', 'record']
    j = ' '.join(cmd)
    assert '-o /b/s' in j
    assert '--max-bag-duration 60' in j
    assert '--max-cache-size 1048576' in j
    assert '--storage-preset-profile resilient' in j
    assert '--compression-mode file --compression-format zstd' in j
    assert cmd[-2:] == ['/tf', '/odom']


def test_record_cmd_no_compression_no_preset():
    cmd = br.record_cmd('/b/s', ['/tf'], 60, 1, compression='', preset='')
    assert '--compression-mode' not in cmd and '--compression-format' not in cmd
    assert '--storage-preset-profile' not in cmd
    assert cmd[-1] == '/tf'


# --------------------------------------------------------------------------- topics
def test_default_topics_sane():
    t = bag_recorder.DEFAULT_TOPICS
    assert len(t) == len(set(t))
    for bad in ('image', 'points', 'cloud', '/global_costmap'):
        assert not [x for x in t if bad in x], bad
    for need in ('/tf_static', '/cmd_vel', '/odometry/filtered', '/odometry/filtered_map',
                 '/mission/incident', '/hardware_bridge/emergency', '/diagnostics',
                 '/map_server_node/terrain_summary', '/dig_stall'):
        assert need in t
    assert all(x.startswith('/') for x in t)


def test_default_topics_leave_out_the_high_rate_streams():
    # 2026-10-09 CPU budget: the 50-100 Hz streams dominated the recorder's cost
    t = bag_recorder.DEFAULT_TOPICS
    for hot in ('/imu/data_aligned', '/mower_base/status', '/odom', '/tf', '/rosout',
                '/mower_base/bumper_routing_status', '/local_plan'):
        assert hot not in t, hot
    assert len(t) <= 60


def test_record_cmd_polling_default_5s():
    cmd = br.record_cmd('/b/s', ['/tf'], 60, 1)
    assert cmd[cmd.index('--polling-interval') + 1] == '5000'


def test_default_pin_kinds_exclude_detour():
    assert 'detour' not in bag_recorder.DEFAULT_PIN_KINDS


# --------------------------------------------------------------------------- per-mow pins
def _roll(root, session, idx):
    """Split ``idx`` closes (compressed) and ``idx + 1`` starts being written."""
    d = os.path.join(root, session)
    open(os.path.join(d, session + '_%d.db3.zstd' % idx), 'wb').write(b'z')
    try:
        os.remove(os.path.join(d, session + '_%d.db3' % idx))
    except OSError:
        pass
    open(os.path.join(d, session + '_%d.db3' % (idx + 1)), 'wb').write(b'y')


def _link_due(mp, root, active, now):
    got = []
    for _d, sp in mp.due(br.scan(root, active), now):
        mp.mark(sp)
        got.append((sp.session, sp.index))
    return got


def test_mow_start_links_split0_and_prev_then_follows_until_stop(tmp_path):
    root = live_ring(tmp_path, active_idx=5)
    mp = br.MowPinner(prev=1)
    first = mp.start('/mows/a', br.scan(root, S1), S1, 0.0)
    assert sorted(s.index for s in first) == [0, 4]
    for s in first:
        mp.mark(s)
    assert mp.active and _link_due(mp, root, S1, 1.0) == []
    # protected until linked: the split being written
    assert (S1, 5) in mp.protect(br.scan(root, S1))
    _roll(root, S1, 5)
    _roll(root, S1, 6)
    assert _link_due(mp, root, S1, 2.0) == [(S1, 5), (S1, 6)]
    mp.stop(br.scan(root, S1), S1, 3.0)        # split 7 being written: the last one
    assert not mp.finished(3.0)
    _roll(root, S1, 7)
    _roll(root, S1, 8)
    assert _link_due(mp, root, S1, 4.0) == [(S1, 7)]      # 8 is after the end
    assert mp.finished(4.0) and not mp.active
    assert mp.due(br.scan(root, S1), 5.0) == [] and mp.protect(br.scan(root, S1)) == set()


def test_mow_follows_recorder_restart_into_new_session(tmp_path):
    root = live_ring(tmp_path, active_idx=2)
    mp = br.MowPinner(prev=1)
    for s in mp.start('/mows/a', br.scan(root, S1), S1, 0.0):
        mp.mark(s)
    # recorder restarted: S1 ends (split 2 closed), S2 begins
    _roll(root, S1, 2)
    os.remove(os.path.join(root, S1, S1 + '_3.db3'))
    mk(tmp_path, S2, [S2 + '_0.db3.zstd', S2 + '_1.db3'])
    got = _link_due(mp, root, S2, 1.0)
    assert got == [(S1, 2), (S2, 0)]
    mp.stop(br.scan(root, S2), S2, 2.0)
    _roll(root, S2, 1)
    assert _link_due(mp, root, S2, 3.0) == [(S2, 1)]
    assert mp.finished(3.0)


def test_mow_stop_with_recorder_off_ends_at_last_split_or_ttl(tmp_path):
    mk(tmp_path, S1, [S1 + '_%d.db3.zstd' % i for i in range(3)])
    root = str(tmp_path)
    mp = br.MowPinner(prev=1, ttl_s=60.0)
    first = mp.start('/mows/a', br.scan(root, None), None, 0.0)
    # nothing live: next split would be 3, so prev=1 -> split 2, plus split 0
    assert sorted(s.index for s in first) == [0, 2]
    for s in first:
        mp.mark(s)
    mp.stop(br.scan(root, None), None, 10.0)
    assert mp.finished(10.0)                     # end split (2) already linked


def test_mow_ttl_finishes_when_end_split_never_closes(tmp_path):
    root = live_ring(tmp_path, active_idx=3)
    mp = br.MowPinner(prev=1, ttl_s=60.0)
    for s in mp.start('/mows/a', br.scan(root, S1), S1, 0.0):
        mp.mark(s)
    mp.stop(br.scan(root, S1), S1, 10.0)
    assert not mp.finished(69.0)
    assert mp.finished(70.0) and not mp.active


def test_mow_close_drops_window(tmp_path):
    root = live_ring(tmp_path, active_idx=3)
    mp = br.MowPinner()
    mp.start('/mows/a', br.scan(root, S1), S1, 0.0)
    mp.close()
    assert not mp.active and mp.due(br.scan(root, S1), 1.0) == []
    assert mp.protect(br.scan(root, S1)) == set()


def test_mow_window_excludes_older_sessions(tmp_path):
    mk(tmp_path, S1, [S1 + '_%d.db3.zstd' % i for i in range(3)])
    mk(tmp_path, S2, [S2 + '_0.db3.zstd', S2 + '_1.db3'])
    root = str(tmp_path)
    mp = br.MowPinner(prev=1)
    first = mp.start('/mows/a', br.scan(root, S2), S2, 0.0)
    assert [(s.session, s.index) for s in first] == [(S2, 0)]


# --------------------------------------------------------------------------- on/off state
def test_enabled_state_roundtrip_and_defaults(tmp_path):
    path = str(tmp_path / 'sub' / 'state.json')
    assert br.load_enabled(path, True) is True
    assert br.load_enabled(path, False) is False
    assert br.save_enabled(path, False)
    assert br.load_enabled(path, True) is False
    assert br.save_enabled(path, True)
    assert br.load_enabled(path, False) is True
    assert not os.path.exists(path + '.tmp')


@pytest.mark.parametrize('text', ['', 'not json', '[]', '{"enabled": "yes"}', '{}'])
def test_enabled_state_corrupt_falls_back_to_default(tmp_path, text):
    path = tmp_path / 'state.json'
    path.write_text(text)
    assert br.load_enabled(str(path), True) is True
    assert br.load_enabled(str(path), False) is False


def test_save_enabled_unwritable_returns_false(tmp_path):
    blocker = tmp_path / 'file'
    blocker.write_text('x')
    assert br.save_enabled(str(blocker / 'state.json'), True) is False
