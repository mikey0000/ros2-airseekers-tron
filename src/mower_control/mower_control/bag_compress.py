# SPDX-License-Identifier: GPL-3.0-or-later
"""Compresses closed rosbag splits in the background (2026-11-03, used by ``bag_recorder``).

rosbag2's file compression (``--compression-mode file``) lost a split whenever the recorder
was stopped for a ring-profile switch while it was still compressing the last closed split:
SIGINT does not finish an in-flight compression (field 2026-11-03: split _21 "premature end",
compression started 48 s before "Pausing recording"). The recorder now writes plain ``.db3``
splits and this module compresses each closed one:

    x_3.db3 -> x_3.db3.zstd.tmp   (zstd level 1, one thread, nice 10)
            -> verified           (full decompress; size + sha256 against the source)
            -> os.replace to x_3.db3.zstd, then x_3.db3 is deleted
            -> metadata.yaml: relative_file_paths / files[].path renamed to .zstd, and
               compression_format zstd / compression_mode FILE once every listed file is .zstd

The split being written is never touched (the newest plain split of the session the recorder
has open or is still closing), nor a split whose sqlite WAL still exists in that session.
A crashed session's leftover WAL is checkpointed first. Leftovers (plain splits, stale
``.tmp``) are picked up on start. A split that fails ``MAX_ATTEMPTS`` times gets a ``.nozstd``
marker and stays plain (``bag_ring.scan_session`` then treats it as closed, so it can be
pinned and pruned instead of blocking the ring). Hard-linked pins keep their own inode: a pin
of the plain ``.db3`` stays readable after the ring's name is replaced.

Backends: the ``zstandard`` module (python3-zstandard), else the ``zstd`` binary.
No ROS here (unit-tested with a fake session dir).
"""

import hashlib
import os
import re
import shutil
import sqlite3
import subprocess
import threading

from mower_control import bag_ring as br

ZSTD = '.zstd'
MAX_ATTEMPTS = 3
CHUNK = 1 << 20


def _nice():
    try:
        os.nice(10)
    except OSError:
        pass


class ZstdModule:
    name = 'zstandard'

    def __init__(self, zstd):
        self.z = zstd

    def compress(self, src, dst):
        cctx = self.z.ZstdCompressor(level=1, write_checksum=True, write_content_size=True)
        with open(src, 'rb') as fi, open(dst, 'wb') as fo:
            cctx.copy_stream(fi, fo, size=os.fstat(fi.fileno()).st_size)
            fo.flush()
            os.fsync(fo.fileno())

    def verify(self, src, dst):
        """Full decompress of ``dst``; True when it equals ``src`` byte for byte (hash)."""
        h_src, n_src = _digest_file(src)
        h = hashlib.sha256()
        n = 0
        try:
            with open(dst, 'rb') as f:
                reader = self.z.ZstdDecompressor().stream_reader(f, read_across_frames=True)
                while True:
                    b = reader.read(CHUNK)
                    if not b:
                        break
                    h.update(b)
                    n += len(b)
        except (self.z.ZstdError, OSError):
            return False
        return n == n_src and h.digest() == h_src


class ZstdBinary:
    name = 'zstd'

    def __init__(self, exe):
        self.exe = exe

    def compress(self, src, dst):
        subprocess.run([self.exe, '-q', '-f', '-1', '-T1', '--check', '-o', dst, src],
                       check=True, stdin=subprocess.DEVNULL, preexec_fn=_nice,
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        with open(dst, 'rb') as f:
            os.fsync(f.fileno())

    def verify(self, src, dst):
        h_src, n_src = _digest_file(src)
        h = hashlib.sha256()
        n = 0
        try:
            p = subprocess.Popen([self.exe, '-q', '-d', '-c', dst], stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                                 preexec_fn=_nice)
        except OSError:
            return False
        with p.stdout:
            for b in iter(lambda: p.stdout.read(CHUNK), b''):
                h.update(b)
                n += len(b)
        return p.wait() == 0 and n == n_src and h.digest() == h_src


def _digest_file(path):
    h = hashlib.sha256()
    n = 0
    with open(path, 'rb') as f:
        for b in iter(lambda: f.read(CHUNK), b''):
            h.update(b)
            n += len(b)
    return h.digest(), n


def find_backend(which=shutil.which):
    """The zstandard module if importable, else the zstd binary, else None."""
    try:
        import zstandard
        return ZstdModule(zstandard)
    except ImportError:
        pass
    exe = which('zstd')
    return ZstdBinary(exe) if exe else None


# --------------------------------------------------------------------------- candidates
def candidates(root, open_session):
    """[(session_dir, plain_path)] closed plain splits, oldest first. ``open_session``: the
    session the recorder is writing or still closing (its newest plain split, and any split
    with a live -wal, are left alone)."""
    out = []
    for name in br.list_sessions(root):
        d = os.path.join(root, name)
        is_open = name == open_session
        splits, _ = br.scan_session(d, active=is_open, compressed=False)
        for sp in splits:
            if sp.state == br.ACTIVE:
                continue
            plain = [p for p in sp.paths if p.endswith('.db3')]
            if not plain or any(p.endswith(br.NOZSTD_SUFFIX) for p in sp.paths):
                continue
            if is_open and any(p.endswith(('-wal', '-journal')) for p in sp.paths):
                continue                 # rosbag2 still closing it
            out.append((d, plain[0]))
    return out


def stale_tmps(root):
    """Every ``*.zstd.tmp`` under the ring (left by a crash or a kill)."""
    out = []
    for name in br.list_sessions(root):
        d = os.path.join(root, name)
        try:
            out += [os.path.join(d, n) for n in os.listdir(d)
                    if n.endswith(ZSTD + br.TMP_SUFFIX)]
        except OSError:
            pass
    return out


def _checkpoint_wal(db):
    """A crashed session's split with a leftover WAL: fold it into the db before compressing."""
    if not (os.path.exists(db + '-wal') or os.path.exists(db + '-journal')):
        return
    con = sqlite3.connect(db)
    try:
        con.execute('PRAGMA wal_checkpoint(TRUNCATE)')
        con.execute('PRAGMA journal_mode=DELETE')
    finally:
        con.close()
    for sc in ('-wal', '-shm'):
        try:
            if os.path.getsize(db + sc) == 0:
                os.remove(db + sc)
        except OSError:
            pass


def _remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


def compress_split(plain, backend, lock=None):
    """Compress -> verify -> promote one split. Returns 'ok', 'bad' (verify failed, nothing
    promoted) or 'gone' (the ring pruned it meanwhile). Raises OSError/CalledProcessError on
    I/O or tool errors (the caller counts attempts)."""
    final = plain + ZSTD
    tmp = final + br.TMP_SUFFIX
    if os.path.exists(final):
        # crash between os.replace and unlink, or an old rosbag2 compression that died
        if backend.verify(plain, final):
            with lock or _NoLock():
                _remove(plain)
            return 'ok'
        _remove(final)
    try:
        backend.compress(plain, tmp)
        good = backend.verify(plain, tmp)
    except FileNotFoundError:
        _remove(tmp)
        return 'gone'
    except BaseException:
        _remove(tmp)
        raise
    if not good:
        _remove(tmp)
        return 'bad'
    with lock or _NoLock():
        if not os.path.exists(plain):        # pruned while compressing
            _remove(tmp)
            return 'gone'
        os.replace(tmp, final)
        _remove(plain)
    return 'ok'


class _NoLock:
    def __enter__(self):
        return self

    def __exit__(self, *_a):
        return False


# --------------------------------------------------------------------------- metadata.yaml
_PATH_LINE = re.compile(r'^(?P<pre>\s*-\s*(?:path:\s*)?)(?P<q>["\']?)(?P<name>[^\s"\']+\.db3)'
                        r'(?P=q)(?P<post>\s*)$')
_REL_KEY = re.compile(r'^\s*relative_file_paths:\s*$')
_KEY = re.compile(r'^(?P<pre>\s*)(?P<key>compression_format|compression_mode):.*$')


def fix_metadata(session_dir):
    """Renames each listed ``x.db3`` whose file is now only ``x.db3.zstd`` and sets the
    compression fields once every relative_file_path is compressed (rosbag2's reader handles
    one mode per bag). Text edit (keeps rosbag2's layout); atomic. Returns True if rewritten.
    rosbag2 rewrites the file while recording, so the worker calls this again each pass."""
    path = os.path.join(session_dir, 'metadata.yaml')
    try:
        with open(path, encoding='utf-8') as f:
            lines = f.read().split('\n')
    except OSError:
        return False
    out = []
    rel = []
    in_rel = False
    rel_indent = None
    for line in lines:
        if _REL_KEY.match(line):
            in_rel, rel_indent = True, len(line) - len(line.lstrip())
            out.append(line)
            continue
        m = _PATH_LINE.match(line)
        if m:
            name = m.group('name')
            if os.path.exists(os.path.join(session_dir, name + ZSTD)) and \
                    not os.path.exists(os.path.join(session_dir, name)):
                line = '%s%s%s%s%s%s' % (m.group('pre'), m.group('q'), name, ZSTD,
                                         m.group('q'), m.group('post'))
        elif in_rel and line.strip() and not line.lstrip().startswith('-') and \
                len(line) - len(line.lstrip()) <= rel_indent:
            in_rel = False
        if in_rel and line.lstrip().startswith('-'):
            rel.append(line.strip().lstrip('-').strip().strip('"\''))
        out.append(line)
    if bool(rel) and all(r.endswith(ZSTD) for r in rel):
        fmt, mode = 'zstd', 'FILE'
    else:                 # mixed (recording, or a .nozstd split): leave rosbag2's fields
        fmt = mode = None
    for i, line in enumerate(out):
        m = _KEY.match(line)
        if m and fmt is not None:
            out[i] = '%s%s: %s' % (m.group('pre'), m.group('key'),
                                   fmt if m.group('key') == 'compression_format' else mode)
    if out == lines:
        return False
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write('\n'.join(out))
    os.replace(tmp, path)
    return True


# --------------------------------------------------------------------------- worker
class CompressWorker:
    """Background thread: compresses one closed split at a time, then fixes metadata.

    ``open_session()`` returns the session the recorder is writing or closing (None = none);
    ``lock`` is the node's ring lock (held only for the rename + unlink so a prune never
    races the promote). ``log(level, text)`` gets one line per event."""

    def __init__(self, root, open_session, lock=None, backend=None, log=None, period_s=5.0):
        self.root = root
        self.open_session = open_session
        self.lock = lock
        self.backend = backend if backend is not None else find_backend()
        self.log = log or (lambda _lvl, _txt: None)
        self.period_s = float(period_s)
        self.attempts = {}            # plain path -> failed attempts
        self.compressed = 0
        self.failed = 0
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self.backend is None or self._thread is not None:
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='bag_compress', daemon=True)
        self._thread.start()
        return True

    def stop(self, timeout_s=None):
        """Lets the split in progress finish (an interrupted one is redone on next start)."""
        self._stop.set()
        self._wake.set()
        t, self._thread = self._thread, None
        if t is not None:
            t.join(timeout_s)

    def wake(self):
        self._wake.set()

    def _run(self):
        _nice()                         # per-thread on Linux
        for tmp in stale_tmps(self.root):
            _remove(tmp)
        while not self._stop.is_set():
            try:
                self.run_once()
            except Exception as exc:  # noqa: BLE001 - never kill the thread
                self.log('error', 'bag_compress: %s' % exc)
            self._wake.wait(self.period_s)
            self._wake.clear()

    def run_once(self):
        """One pass: every candidate, then metadata of every session. Returns #compressed."""
        n = 0
        for _d, plain in candidates(self.root, self.open_session()):
            if self._stop.is_set():
                break
            n += self._one(plain)
        for name in br.list_sessions(self.root):
            d = os.path.join(self.root, name)
            try:
                fix_metadata(d)
            except OSError as exc:
                self.log('warn', 'bag_compress: metadata %s: %s' % (d, exc))
        return n

    def _one(self, plain):
        try:
            if self.open_session() != os.path.basename(os.path.dirname(plain)):
                _checkpoint_wal(plain)
            res = compress_split(plain, self.backend, self.lock)
        except (OSError, subprocess.CalledProcessError, sqlite3.Error) as exc:
            res, why = 'bad', str(exc)
        else:
            why = 'verify failed'
        if res == 'ok':
            self.attempts.pop(plain, None)
            self.compressed += 1
            return 1
        if res == 'bad':
            k = self.attempts[plain] = self.attempts.get(plain, 0) + 1
            if k >= MAX_ATTEMPTS:
                self.failed += 1
                self.attempts.pop(plain, None)
                try:
                    open(plain + br.NOZSTD_SUFFIX, 'w').close()
                except OSError:
                    pass
                self.log('error', 'bag_compress: giving up on %s after %d tries (%s); kept '
                         'uncompressed' % (plain, k, why))
            else:
                self.log('warn', 'bag_compress: %s: %s (try %d/%d)'
                         % (plain, why, k, MAX_ATTEMPTS))
        return 0
