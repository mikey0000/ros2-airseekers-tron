"""bag_compress: own zstd compression of closed splits (fake session dirs, real sqlite files)."""

import os
import sqlite3
import subprocess
import threading

import pytest

from mower_control import bag_compress as bc
from mower_control import bag_ring as br

S1 = '20261103-101500'
S2 = '20261103-111500'

BACKEND = bc.find_backend()
needs_zstd = pytest.mark.skipif(BACKEND is None, reason='no zstandard module or zstd binary')


def mkdb(root, session, idx, rows=200, wal=False):
    d = os.path.join(str(root), session)
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, '%s_%d.db3' % (session, idx))
    con = sqlite3.connect(path)
    if wal:
        con.execute('PRAGMA journal_mode=WAL')
        con.execute('PRAGMA wal_autocheckpoint=0')
    con.execute('CREATE TABLE messages (id INTEGER PRIMARY KEY, data BLOB)')
    con.executemany('INSERT INTO messages (data) VALUES (?)',
                    [(b'odom %d ' % i * 20,) for i in range(rows)])
    con.commit()
    if wal:
        return path, con             # keep it open: the WAL stays (a "crashed" writer)
    con.close()
    return path


def rows(path):
    con = sqlite3.connect(path)
    try:
        return con.execute('SELECT COUNT(*) FROM messages').fetchone()[0]
    finally:
        con.close()


def decompress(zst, out):
    if isinstance(BACKEND, bc.ZstdModule):
        with open(zst, 'rb') as fi, open(out, 'wb') as fo:
            BACKEND.z.ZstdDecompressor().copy_stream(fi, fo)
    else:
        subprocess.run([BACKEND.exe, '-q', '-d', '-f', '-o', out, zst], check=True)
    return out


class Garbage:
    """A backend whose output never verifies."""
    name = 'garbage'

    def compress(self, src, dst):
        with open(dst, 'wb') as f:
            f.write(b'\x28\xb5\x2f\xfd broken')

    def verify(self, src, dst):
        return False


def worker(root, open_session=None, backend=None):
    return bc.CompressWorker(str(root), lambda: open_session, threading.Lock(),
                             backend or BACKEND)


# --------------------------------------------------------------------------- compress
@needs_zstd
def test_compress_verify_replace(tmp_path):
    db = mkdb(tmp_path, S1, 0)
    with open(db, 'rb') as f:
        original = f.read()
    assert bc.compress_split(db, BACKEND) == 'ok'
    assert not os.path.exists(db)
    assert not os.path.exists(db + '.zstd.tmp')
    out = decompress(db + '.zstd', str(tmp_path / 'x.db3'))
    with open(out, 'rb') as f:
        assert f.read() == original
    assert os.path.getsize(db + '.zstd') < len(original)


@needs_zstd
def test_worker_skips_open_split_and_compresses_rolled_ones(tmp_path):
    for i in range(3):
        mkdb(tmp_path, S2, i)
    w = worker(tmp_path, open_session=S2)
    assert w.run_once() == 2
    names = sorted(os.listdir(tmp_path / S2))
    assert names == [S2 + '_0.db3.zstd', S2 + '_1.db3.zstd', S2 + '_2.db3']
    w.open_session = lambda: None           # recorder stopped: the last split is closed now
    assert w.run_once() == 1
    assert sorted(os.listdir(tmp_path / S2)) == [S2 + '_%d.db3.zstd' % i for i in range(3)]


@needs_zstd
def test_open_split_with_live_wal_is_left_alone(tmp_path):
    _p, con = mkdb(tmp_path, S2, 0, wal=True)
    mkdb(tmp_path, S2, 1)
    try:
        assert bc.candidates(str(tmp_path), S2) == []
    finally:
        con.close()


@needs_zstd
def test_leftovers_resumed_on_start(tmp_path):
    old = [mkdb(tmp_path, S1, i) for i in range(2)]
    stale = old[0] + '.zstd.tmp'
    with open(stale, 'wb') as f:
        f.write(b'half written')
    # crash between os.replace and unlink: verified .zstd + .db3 both present
    BACKEND.compress(old[1], old[1] + '.zstd')
    w = worker(tmp_path)
    assert w.start()
    try:
        for _ in range(200):
            if sorted(os.listdir(tmp_path / S1)) == [os.path.basename(p) + '.zstd'
                                                     for p in old]:
                break
            threading.Event().wait(0.05)
    finally:
        w.stop(10.0)
    assert sorted(os.listdir(tmp_path / S1)) == [os.path.basename(p) + '.zstd' for p in old]


@needs_zstd
def test_crashed_session_wal_is_checkpointed_before_compressing(tmp_path):
    src, con = mkdb(tmp_path / 'live', S1, 0, wal=True)
    d = tmp_path / 'ring' / S1
    d.mkdir(parents=True)
    p = str(d / os.path.basename(src))
    for sc in ('', '-wal'):                  # a power cut: db + unfolded WAL on disk
        with open(src + sc, 'rb') as fi, open(p + sc, 'wb') as fo:
            fo.write(fi.read())
    con.close()
    assert os.path.getsize(p + '-wal') > 0
    assert worker(tmp_path / 'ring').run_once() == 1
    assert not os.path.exists(p + '-wal')
    out = decompress(p + '.zstd', str(tmp_path / 'x.db3'))
    assert rows(out) == 200


def test_corrupt_output_not_promoted_then_gives_up(tmp_path):
    db = mkdb(tmp_path, S1, 0)
    w = worker(tmp_path, backend=Garbage())
    for k in range(bc.MAX_ATTEMPTS):
        assert w.run_once() == 0
        assert os.path.exists(db)
        assert not os.path.exists(db + '.zstd')
        assert not os.path.exists(db + '.zstd.tmp')
    assert os.path.exists(db + br.NOZSTD_SUFFIX)
    assert w.failed == 1
    assert bc.candidates(str(tmp_path), None) == []
    # the ring sees it as a closed plain split (pinnable, prunable)
    splits, _ = br.scan_session(str(tmp_path / S1), active=False)
    assert splits[0].state == br.CLOSED and splits[0].data_path() == db


@needs_zstd
def test_split_pruned_while_compressing_is_gone(tmp_path):
    db = mkdb(tmp_path, S1, 0)

    class Pruned:
        name = 'pruned'

        def compress(self, src, dst):
            BACKEND.compress(src, dst)
            os.remove(src)                   # the ring pruned it meanwhile

        def verify(self, src, dst):
            return BACKEND.verify(src, dst)

    assert bc.compress_split(db, Pruned()) == 'gone'
    assert os.listdir(tmp_path / S1) == []


@needs_zstd
def test_pin_hardlink_survives_compression(tmp_path):
    db = mkdb(tmp_path, S1, 0)
    splits, _ = br.scan_session(str(tmp_path / S1), active=False)
    pinned = br.pin_split(splits[0], str(tmp_path / 'pins' / 'p1'))
    assert worker(tmp_path).run_once() == 1
    assert not os.path.exists(db)
    assert rows(pinned) == 200               # the pin's own name of the old inode
    # a later pin of the same split links the compressed file
    splits, _ = br.scan_session(str(tmp_path / S1), active=False)
    p2 = br.pin_split(splits[0], str(tmp_path / 'pins' / 'p2'))
    assert p2.endswith('.db3.zstd') and os.stat(p2).st_ino == os.stat(db + '.zstd').st_ino


@needs_zstd
def test_ring_waits_for_compression_before_pinning_active_session(tmp_path):
    for i in range(3):
        mkdb(tmp_path, S2, i)
    sessions = br.scan(str(tmp_path), S2, compressed=True)
    assert [s.state for s in sessions[0][1]] == [br.COMPRESSING, br.COMPRESSING, br.ACTIVE]
    worker(tmp_path, open_session=S2).run_once()
    sessions = br.scan(str(tmp_path), S2, compressed=True)
    assert [s.state for s in sessions[0][1]] == [br.CLOSED, br.CLOSED, br.ACTIVE]
    assert sessions[0][1][0].data_path().endswith('.db3.zstd')


@pytest.mark.skipif(not isinstance(BACKEND, bc.ZstdModule), reason='zstandard not installed')
def test_module_backend_rejects_truncated_frame(tmp_path):
    db = mkdb(tmp_path, S1, 0)
    BACKEND.compress(db, db + '.zstd')
    with open(db + '.zstd', 'r+b') as f:
        f.truncate(os.path.getsize(db + '.zstd') // 2)
    assert not BACKEND.verify(db, db + '.zstd')


@pytest.mark.skipif(bc.shutil.which('zstd') is None, reason='no zstd binary')
def test_binary_backend_rejects_truncated_frame(tmp_path):
    b = bc.ZstdBinary(bc.shutil.which('zstd'))
    db = mkdb(tmp_path, S1, 0)
    b.compress(db, db + '.zstd')
    assert b.verify(db, db + '.zstd')
    with open(db + '.zstd', 'r+b') as f:
        f.truncate(os.path.getsize(db + '.zstd') // 2)
    assert not b.verify(db, db + '.zstd')


def test_find_backend_none_without_module_or_binary(monkeypatch):
    import builtins
    real = builtins.__import__

    def no_zstandard(name, *a, **k):
        if name == 'zstandard':
            raise ImportError(name)
        return real(name, *a, **k)

    monkeypatch.setattr(builtins, '__import__', no_zstandard)
    assert bc.find_backend(which=lambda _n: None) is None
    assert isinstance(bc.find_backend(which=lambda _n: '/usr/bin/zstd'), bc.ZstdBinary)


# --------------------------------------------------------------------------- metadata
META = """rosbag2_bagfile_information:
  version: 9
  storage_identifier: sqlite3
  duration:
    nanoseconds: 120000000000
  relative_file_paths:
    - {s}_0.db3
    - {s}_1.db3
  files:
    - path: {s}_0.db3
      starting_time:
        nanoseconds_since_epoch: 1
      message_count: 10
    - path: {s}_1.db3
      starting_time:
        nanoseconds_since_epoch: 2
      message_count: 10
  compression_format: ""
  compression_mode: ""
  custom_data: ~
"""


def test_metadata_rewritten_per_split_and_mode_set_when_all_compressed(tmp_path):
    d = tmp_path / S1
    d.mkdir()
    (d / 'metadata.yaml').write_text(META.format(s=S1))
    (d / (S1 + '_0.db3.zstd')).write_bytes(b'z')
    (d / (S1 + '_1.db3')).write_bytes(b'p')
    assert bc.fix_metadata(str(d))
    text = (d / 'metadata.yaml').read_text()
    assert '- %s_0.db3.zstd' % S1 in text and 'path: %s_0.db3.zstd' % S1 in text
    assert '- %s_1.db3\n' % S1 in text
    assert 'compression_mode: ""' in text      # mixed: rosbag2 cannot read it as FILE yet
    assert not bc.fix_metadata(str(d))         # idempotent
    os.remove(d / (S1 + '_1.db3'))
    (d / (S1 + '_1.db3.zstd')).write_bytes(b'z')
    assert bc.fix_metadata(str(d))
    text = (d / 'metadata.yaml').read_text()
    assert 'compression_format: zstd' in text and 'compression_mode: FILE' in text
    assert '.db3\n' not in text
    assert 'custom_data: ~' in text and 'version: 9' in text
    assert not os.path.exists(d / 'metadata.yaml.tmp')


def test_metadata_absent_is_fine(tmp_path):
    (tmp_path / S1).mkdir()
    assert not bc.fix_metadata(str(tmp_path / S1))


@needs_zstd
def test_metadata_readable_by_yaml_after_worker(tmp_path):
    yaml = pytest.importorskip('yaml')
    for i in range(2):
        mkdb(tmp_path, S1, i)
    (tmp_path / S1 / 'metadata.yaml').write_text(META.format(s=S1))
    worker(tmp_path).run_once()
    info = yaml.safe_load((tmp_path / S1 / 'metadata.yaml').read_text())
    bi = info['rosbag2_bagfile_information']
    assert bi['relative_file_paths'] == ['%s_%d.db3.zstd' % (S1, i) for i in range(2)]
    assert [f['path'] for f in bi['files']] == bi['relative_file_paths']
    assert bi['compression_format'] == 'zstd' and bi['compression_mode'] == 'FILE'
