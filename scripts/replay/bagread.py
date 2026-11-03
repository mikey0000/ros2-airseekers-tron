"""Read rosbag2 sqlite splits (.db3, or .db3.zstd via the zstd CLI) without metadata.yaml.

Incident directories hold a subset of a session's splits and no metadata, so this reads the
sqlite tables directly and deserializes with rclpy (needs a sourced ROS install whose message
packages cover the recorded types: run it in the dev image after ``dev_build.sh build``).
"""
import glob
import os
import shutil
import sqlite3
import subprocess
import tempfile


def split_files(paths):
    """Every .db3 / .db3.zstd under the given files or directories, sorted by split index."""
    out = []
    for p in paths:
        if os.path.isdir(p):
            out += glob.glob(os.path.join(p, '*.db3')) + glob.glob(os.path.join(p, '*.db3.zstd'))
        else:
            out.append(p)

    def key(f):
        stem = os.path.basename(f).split('.db3')[0]
        tail = stem.rsplit('_', 1)[-1]
        return (stem.rsplit('_', 1)[0], int(tail) if tail.isdigit() else -1)
    return sorted(set(out), key=key)


def _open(path, tmp):
    if not path.endswith('.zstd'):
        return path
    dst = os.path.join(tmp, os.path.basename(path)[:-5])
    if shutil.which('zstd'):
        subprocess.check_call(['zstd', '-dqf', path, '-o', dst])
    else:
        import zstandard  # noqa: F401  (pip install zstandard) when the CLI is missing
        with open(path, 'rb') as src, open(dst, 'wb') as out:
            zstandard.ZstdDecompressor().copy_stream(src, out)
    return dst


def topics(paths):
    """{topic: type} over the splits."""
    seen = {}
    with tempfile.TemporaryDirectory() as tmp:
        for f in split_files(paths):
            con = sqlite3.connect(_open(f, tmp))
            for name, typ in con.execute('SELECT name, type FROM topics'):
                seen[name] = typ
            con.close()
    return seen


def messages(paths, wanted, t_from=None, t_until=None):
    """Yield (stamp_s, topic, msg) for topics in ``wanted`` within [t_from, t_until] (epoch s),
    in receive order, deserialized with rclpy."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    lo = -1 if t_from is None else int(t_from * 1e9)
    hi = 1 << 62 if t_until is None else int(t_until * 1e9)
    types = {}
    with tempfile.TemporaryDirectory() as tmp:
        for f in split_files(paths):
            db = _open(f, tmp)
            con = sqlite3.connect(db)
            tmap = {i: (n, t) for i, n, t in con.execute('SELECT id, name, type FROM topics')
                    if n in wanted}
            if tmap:
                q = ('SELECT topic_id, timestamp, data FROM messages WHERE timestamp BETWEEN ? AND ?'
                     ' AND topic_id IN (%s) ORDER BY timestamp' % ','.join(map(str, tmap)))
                for tid, ts, data in con.execute(q, (lo, hi)):
                    name, typ = tmap[tid]
                    if typ not in types:
                        types[typ] = get_message(typ)
                    yield ts / 1e9, name, deserialize_message(data, types[typ])
            con.close()
            if db != f:
                os.remove(db)
