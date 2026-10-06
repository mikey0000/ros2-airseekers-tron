"""Black-box ring buffer (mower_control.blackbox): window, rate cap, size cap, dump."""

import os
import sys
import types

from mower_control.blackbox import SnapshotBuffer, dump_bag, make_dump_dir

S = 1_000_000_000


def buf(**kw):
    b = SnapshotBuffer(**kw)
    b.add_topic('/a', 'std_msgs/msg/String')
    b.add_topic('/b', 'std_msgs/msg/Bool', max_rate_hz=2.0)
    return b


def test_time_window_drops_old_messages():
    b = buf(window_s=10.0)
    for t in range(0, 30):
        b.push('/a', t * S, b'x' * 10)
    snap = b.snapshot()
    assert [m[2] for m in snap] == [t * S for t in range(19, 30)]
    assert b.total_bytes == 11 * 10


def test_snapshot_expires_relative_to_now():
    b = buf(window_s=10.0)
    b.push('/a', 0, b'x')
    assert len(b.snapshot(now_ns=5 * S)) == 1
    assert b.snapshot(now_ns=11 * S) == [] and b.total_bytes == 0


def test_rate_cap_per_topic():
    b = buf()
    stored = [b.push('/b', int(t * 0.1 * S), b'1') for t in range(20)]   # 10 Hz in, 2 Hz cap
    assert sum(stored) == 4 and b.dropped_rate == 16


def test_unknown_topic_ignored():
    b = buf()
    assert not b.push('/nope', 0, b'x')


def test_size_cap_evicts_oldest_across_topics():
    b = buf(max_bytes=100)
    b.push('/a', 1 * S, b'a' * 40)
    b.push('/b', 2 * S, b'b' * 40)
    b.push('/a', 3 * S, b'c' * 40)          # 120 > 100: the t=1 message goes
    snap = b.snapshot()
    assert [(m[0], m[2]) for m in snap] == [('/b', 2 * S), ('/a', 3 * S)]
    assert b.total_bytes == 80 and b.evicted == 1


def test_snapshot_is_time_ordered_with_types():
    b = buf()
    b.push('/a', 3 * S, b'3')
    b.push('/b', 1 * S, b'1')
    b.push('/a', 5 * S, b'5')
    b.push('/b', 4 * S, b'4')
    snap = b.snapshot()
    assert [m[2] for m in snap] == [1 * S, 3 * S, 4 * S, 5 * S]
    assert snap[0][1] == 'std_msgs/msg/Bool' and snap[1][1] == 'std_msgs/msg/String'


def test_dump_writes_every_message_once(monkeypatch, tmp_path):
    calls = {'open': [], 'topics': [], 'writes': []}

    class Writer:
        def open(self, storage, conv):
            calls['open'].append(storage.uri)

        def create_topic(self, meta):
            calls['topics'].append((meta.name, meta.type))

        def write(self, topic, data, t):
            calls['writes'].append((topic, data, t))

    fake = types.ModuleType('rosbag2_py')
    fake.SequentialWriter = Writer
    fake.StorageOptions = lambda uri, storage_id: types.SimpleNamespace(uri=uri)
    fake.ConverterOptions = lambda **k: None
    fake.TopicMetadata = lambda name, type, serialization_format: types.SimpleNamespace(
        name=name, type=type)
    monkeypatch.setitem(sys.modules, 'rosbag2_py', fake)

    b = buf()
    b.push('/a', 1 * S, b'one')
    b.push('/b', 2 * S, b'two')
    b.push('/a', 3 * S, b'three')
    n = dump_bag(b.snapshot(), str(tmp_path / 'bag'))
    assert n == 3
    assert calls['topics'] == [('/a', 'std_msgs/msg/String'), ('/b', 'std_msgs/msg/Bool')]
    assert [w[1] for w in calls['writes']] == [b'one', b'two', b'three']
    # the snapshot does not consume the buffer (a later dump still has the history)
    assert len(b.snapshot()) == 3


def test_make_dump_dir_unique_and_sanitised(tmp_path):
    a = make_dump_dir(str(tmp_path), '20261007-120000', 'node_down /x y')
    b = make_dump_dir(str(tmp_path), '20261007-120000', 'node_down /x y')
    assert a != b and os.path.isdir(a) and os.path.isdir(b)
    assert os.path.basename(a) == '20261007-120000_node_down__x_y'
