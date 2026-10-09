# SPDX-License-Identifier: GPL-3.0-or-later
"""Always-on rolling rosbag ring: pure logic (no ROS), used by ``bag_recorder``.

Layout written by ``ros2 bag record`` (rosbag2 0.15.17, verified in the dev image 2026-10-09):

    <root>/<YYYYmmdd-HHMMSS>/<YYYYmmdd-HHMMSS>_<i>.db3        split being written (+ -wal/-shm
                                                             with the 'resilient' preset)
    <root>/<YYYYmmdd-HHMMSS>/<YYYYmmdd-HHMMSS>_<i>.db3.zstd   closed split (file compression)
    <root>/<YYYYmmdd-HHMMSS>/metadata.yaml                    only after a clean stop

With file compression a closed split is compressed by a rosbag2 thread and then the .db3 is
deleted, so for a moment both exist: that split is "compressing" (never deleted, never copied).
Without compression a split is closed once a higher index exists or the session has ended.

Each decompressed split is a complete sqlite3 bag on its own (``ros2 bag info x_3.db3`` works
without metadata.yaml), so pruning single splits and pinning a handful of them is safe.
"""

import json
import os
import re
import shutil
import time

SPLIT_RE = re.compile(r'^(?P<stem>.+)_(?P<idx>\d+)\.(?P<ext>db3|mcap)(?P<zst>\.zstd)?$')
SIDECARS = ('-wal', '-shm', '-journal')
SESSION_RE = re.compile(r'^\d{8}-\d{6}(\.\d+)?$')

ACTIVE, COMPRESSING, CLOSED = 'active', 'compressing', 'closed'


class Split:
    __slots__ = ('session', 'index', 'paths', 'size', 'state', 'mtime')

    def __init__(self, session, index):
        self.session = session
        self.index = index
        self.paths = []          # every file of this split (data, .zstd, sidecars)
        self.size = 0
        self.state = CLOSED
        self.mtime = 0.0

    def data_path(self):
        """The file to pin: the compressed one when it exists, else the .db3/.mcap."""
        zst = [p for p in self.paths if p.endswith('.zstd')]
        if zst:
            return zst[0]
        plain = [p for p in self.paths if not p.endswith(SIDECARS)]
        return plain[0] if plain else None

    def __repr__(self):
        return 'Split(%s,%d,%s,%d)' % (self.session, self.index, self.state, self.size)


def _file_size(path):
    try:
        st = os.stat(path)
        return st.st_size, st.st_mtime
    except OSError:
        return 0, 0.0


def scan_session(path, active, compressed=True):
    """Splits of one session dir, sorted by index; plus the bytes of non-split files.
    ``active``: the recorder is still writing this session (its newest uncompressed split is
    the one being written). ``compressed``: the recorder runs with file compression, so an
    older uncompressed split of the active session is queued for / under compression (rosbag2
    starts the .zstd a moment after the split rolls)."""
    session = os.path.basename(path)
    try:
        names = os.listdir(path)
    except OSError:
        return [], 0
    splits = {}
    plain = {}                       # index -> has an uncompressed data file
    zst = {}                         # index -> has a .zstd file
    other = 0
    for name in names:
        full = os.path.join(path, name)
        base = name
        for sc in SIDECARS:
            if name.endswith(sc):
                base = name[:-len(sc)]
                break
        m = SPLIT_RE.match(base)
        size, mtime = _file_size(full)
        if not m:
            other += size
            continue
        idx = int(m.group('idx'))
        sp = splits.get(idx)
        if sp is None:
            sp = splits[idx] = Split(session, idx)
        sp.paths.append(full)
        sp.size += size
        sp.mtime = max(sp.mtime, mtime)
        if base == name:
            if m.group('zst'):
                zst[idx] = True
            else:
                plain[idx] = True
    ordered = [splits[i] for i in sorted(splits)]
    newest = ordered[-1].index if ordered else None
    for sp in ordered:
        if plain.get(sp.index) and zst.get(sp.index):
            sp.state = COMPRESSING
        elif plain.get(sp.index) and active and sp.index == newest:
            sp.state = ACTIVE
        elif plain.get(sp.index) and active and compressed:
            sp.state = COMPRESSING   # rolled, .zstd not started yet: leave it to rosbag2
        else:                        # compressed, or uncompressed with a later split / ended
            sp.state = CLOSED
    return ordered, other


def list_sessions(root):
    """Session dir names under ``root``, oldest first (names sort chronologically)."""
    try:
        return sorted(n for n in os.listdir(root)
                      if SESSION_RE.match(n) and os.path.isdir(os.path.join(root, n)))
    except OSError:
        return []


def scan(root, active_session=None, compressed=True):
    """[(session_name, [Split], other_bytes)] oldest first."""
    out = []
    for name in list_sessions(root):
        splits, other = scan_session(os.path.join(root, name), name == active_session,
                                     compressed)
        out.append((name, splits, other))
    return out


def total_bytes(sessions):
    return sum(sum(s.size for s in splits) + other for _n, splits, other in sessions)


# --------------------------------------------------------------------------- pruning
def plan_prune(sessions, active_session, max_bytes, free_bytes, min_free_bytes, protect=()):
    """Oldest-first deletions so that the ring is <= ``max_bytes`` and the filesystem keeps
    >= ``min_free_bytes`` free. Returns (victims, starved): ``victims`` is a list of Splits and
    session names (str: the whole dir goes, used for sessions left without splits);
    ``starved`` = True when the free-space floor cannot be met with what is deletable (the
    caller pauses recording). A ring over ``max_bytes`` with only protected splits left is not
    starved: it shrinks at the next rotation (smoke run 2026-10-09 paused on that wrongly).

    Never deleted: the split being written, a split being compressed, ``protect``
    ((session, index): splits an incident pin still waits for), and split 0 of the active
    session (it holds the one-shot transient-local messages: /tf_static, latched
    gate/terrain topics; rosbag2 Humble has no --repeat-transient-local).
    """
    total = total_bytes(sessions)
    free = free_bytes
    victims = []

    def done():
        return total <= max_bytes and free >= min_free_bytes

    for name, splits, other in sessions:
        if done():
            break
        is_active = name == active_session
        removable = [s for s in splits
                     if s.state == CLOSED and not (is_active and s.index == 0)
                     and (name, s.index) not in protect]
        for sp in removable:
            if done():
                break
            victims.append(sp)
            total -= sp.size
            free += sp.size
        if not is_active and len(removable) == len(splits) and \
                all(v in victims for v in splits):
            victims.append(name)    # its metadata.yaml now points at nothing: drop the dir
            total -= other
            free += other
    return victims, free < min_free_bytes


def apply_prune(root, victims, rmtree=shutil.rmtree, remove=os.remove):
    """Deletes what plan_prune returned. Returns the number of bytes released (approx)."""
    freed = 0
    for v in victims:
        if isinstance(v, str):
            rmtree(os.path.join(root, v), ignore_errors=True)
            continue
        for p in v.paths:
            try:
                remove(p)
            except OSError:
                pass
        freed += v.size
    return freed


def free_bytes(path):
    try:
        st = os.statvfs(path)
    except OSError:
        return None
    return st.f_bavail * st.f_frsize


# --------------------------------------------------------------------------- incident pins
def safe_reason(text):
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', str(text)).strip('_')[:48] or 'incident'


class PinPlanner:
    """Which splits an incident pin copies, and when.

    On a trigger the closed splits ``prev`` back from the one being written are pinned at once
    (they could be pruned any minute), plus split 0 of the session (/tf_static). The split
    being written and ``post`` further ones are pinned as each one closes, so the pin also
    holds what happened after the trigger. A pending entry expires after ``pending_ttl_s``
    (recorder died, session ended without the split ever closing)."""

    def __init__(self, prev=2, post=1, pending_ttl_s=900.0):
        self.prev = int(prev)
        self.post = int(post)
        self.pending_ttl_s = float(pending_ttl_s)
        self.pending = []        # [pin_dir, session, index, deadline]

    def trigger(self, pin_dir, sessions, active_session, now):
        """Returns the Splits to pin immediately; registers the pending ones."""
        cur = None
        splits = []
        for name, sp, _o in sessions:
            if name == active_session:
                splits = sp
        live = [s for s in splits if s.state in (ACTIVE, COMPRESSING)]
        if live:
            cur = max(s.index for s in live)
        elif splits:
            cur = splits[-1].index + 1   # between splits / paused: everything so far is closed
        else:
            cur = 0
        now_list = []
        wanted = set(range(max(0, cur - self.prev), cur)) | {0}
        pend = set(range(cur, cur + self.post + 1))
        for s in splits:
            if s.index in wanted or s.index in pend:
                if s.state == CLOSED:
                    now_list.append(s)
                    pend.discard(s.index)
                else:
                    pend.add(s.index)
        for idx in sorted(pend):
            self.pending.append([pin_dir, active_session, idx, now + self.pending_ttl_s])
        return now_list

    def due(self, sessions, now):
        """[(pin_dir, Split)] pending splits that have closed since; drops expired entries."""
        by_name = {name: {s.index: s for s in sp} for name, sp, _o in sessions}
        out = []
        keep = []
        for entry in self.pending:
            pin_dir, session, idx, deadline = entry
            sp = by_name.get(session, {}).get(idx)
            if sp is not None and sp.state == CLOSED:
                out.append((pin_dir, sp))
            elif now < deadline:
                keep.append(entry)
        self.pending = keep
        return out

    def pending_dirs(self):
        return {e[0] for e in self.pending}

    def pending_splits(self):
        """{(session, index)} still to be pinned: plan_prune must not delete them."""
        return {(e[1], e[2]) for e in self.pending}


def _live_index(splits):
    """Index of the split being written (or the next one when nothing is live)."""
    live = [s.index for s in splits if s.state in (ACTIVE, COMPRESSING)]
    if live:
        return max(live)
    return splits[-1].index + 1 if splits else 0


class MowPinner:
    """Mission-scoped pin (2026-10-09: replaces the separate ``mow_recorder`` process, which
    cost a second ~35 % of a core while mowing): every split recorded from ``prev`` splits
    before the mission started to the split being written when it ended is hard-linked into
    ``<mows_dir>/<ts>/`` as it closes, plus split 0 of every session touched (/tf_static and
    latched topics). Sessions are compared by name (they sort chronologically), so a recorder
    restart mid-mow is followed into the new session. One mow at a time.

    After ``stop`` the mow is finished once the end split is linked, or ``ttl_s`` later
    (recorder stopped/disabled: that split never closes)."""

    def __init__(self, prev=1, ttl_s=900.0):
        self.prev = int(prev)
        self.ttl_s = float(ttl_s)
        self.mow_dir = None
        self._start = None       # (session, first index)
        self._end = None         # (session, last index) once stopped
        self._deadline = None
        self.linked = set()      # {(session, index)}

    @property
    def active(self):
        return self.mow_dir is not None

    def _in_window(self, session, index):
        s0, i0 = self._start
        if session < s0:
            return False
        if session == s0 and index < i0 and index != 0:
            return False
        if self._end is not None:
            s1, i1 = self._end
            if session > s1 or (session == s1 and index > i1):
                return False
        return True

    def start(self, mow_dir, sessions, active_session, now):
        """Opens a mow; returns the closed Splits to link at once."""
        if active_session is None:      # recorder not running: the newest session
            active_session = sessions[-1][0] if sessions else ''
        splits = []
        for name, sp, _o in sessions:
            if name == active_session:
                splits = sp
        cur = _live_index(splits)
        self.mow_dir = mow_dir
        self._start = (active_session, max(0, cur - self.prev))
        self._end = None
        self._deadline = None
        self.linked = set()
        return [s for s, _d in self._closed_unlinked(sessions)]

    def stop(self, sessions, active_session, now):
        """The mission ended: the window closes at the split being written now."""
        if not self.active or self._end is not None:
            return
        splits = []
        for name, sp, _o in sessions:
            if name == active_session:
                splits = sp
        if active_session is None or not splits:
            # nothing live: the last split that exists ends the mow
            last = sessions[-1] if sessions else None
            if last is None or not last[1]:
                self._end = self._start
            else:
                self._end = (last[0], last[1][-1].index)
        else:
            live = [s.index for s in splits if s.state in (ACTIVE, COMPRESSING)]
            self._end = (active_session, max(live) if live else splits[-1].index)
        self._deadline = now + self.ttl_s

    def _closed_unlinked(self, sessions):
        out = []
        for name, sp, _o in sessions:
            for s in sp:
                if s.state == CLOSED and (name, s.index) not in self.linked and \
                        self._in_window(name, s.index):
                    out.append((s, self.mow_dir))
        return out

    def due(self, sessions, now):
        """[(mow_dir, Split)] closed window splits not linked yet; the caller links them and
        calls ``mark``, then ``finished``."""
        if not self.active:
            return []
        out = [(d, s) for s, d in self._closed_unlinked(sessions)]
        return out

    def mark(self, split):
        self.linked.add((split.session, split.index))

    def finished(self, now):
        """True (and resets) when a stopped mow has everything it will get."""
        if not self.active or self._end is None:
            return False
        if self._end in self.linked or (self._deadline is not None and now >= self._deadline):
            self.close()
            return True
        return False

    def close(self):
        """Drops the current mow (what is linked stays)."""
        self.mow_dir = None
        self._start = self._end = self._deadline = None

    def protect(self, sessions):
        """{(session, index)} window splits not linked yet: plan_prune must keep them."""
        if not self.active:
            return set()
        return {(name, s.index) for name, sp, _o in sessions for s in sp
                if (name, s.index) not in self.linked and self._in_window(name, s.index)}


# --------------------------------------------------------------------------- on/off state
def load_enabled(path, default=True):
    """The persisted on/off switch (``{"enabled": bool}``); ``default`` when unreadable."""
    try:
        with open(path, encoding='utf-8') as f:
            v = json.load(f).get('enabled')
    except (OSError, ValueError, AttributeError):
        return bool(default)
    return v if isinstance(v, bool) else bool(default)


def save_enabled(path, enabled):
    """Atomic write; returns False when the directory is not writable."""
    try:
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump({'enabled': bool(enabled),
                       'changed': time.strftime('%Y-%m-%dT%H:%M:%S%z')}, f)
        os.replace(tmp, path)
        return True
    except OSError:
        return False


def pin_split(split, pin_dir, link=os.link, copy=shutil.copy2):
    """Hard-link (same filesystem: no copy, no extra space until the ring prunes its name) or
    copy one closed split into ``pin_dir``. Returns the destination path or None."""
    src = split.data_path()
    if src is None:
        return None
    os.makedirs(pin_dir, exist_ok=True)
    dst = os.path.join(pin_dir, '%s__%s' % (split.session, os.path.basename(src)))
    if os.path.exists(dst):
        return dst
    try:
        link(src, dst)
    except OSError:
        try:
            copy(src, dst)
        except OSError:
            return None
    return dst


def write_pin_info(pin_dir, info):
    os.makedirs(pin_dir, exist_ok=True)
    path = os.path.join(pin_dir, 'incident.json')
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(info, f, indent=2, sort_keys=True)
    os.replace(tmp, path)


def dir_bytes(path):
    total = 0
    for base, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(base, f))
            except OSError:
                pass
    return total


def plan_incident_retention(entries, max_count, max_bytes, keep=()):
    """``entries`` = [(dir_name, bytes)] (names sort chronologically). Oldest-first victims so
    at most ``max_count`` remain and their total is <= ``max_bytes``; ``keep`` (pins still
    receiving splits) are never returned."""
    ordered = sorted(entries)
    count = len(ordered)
    total = sum(b for _n, b in ordered)
    victims = []
    for name, size in ordered:
        if count <= max_count and total <= max_bytes:
            break
        if name in keep:
            continue
        victims.append(name)
        count -= 1
        total -= size
    return victims


# --------------------------------------------------------------------------- triggers
class IncidentDetector:
    """Turns mission/emergency/crash signals into pin reasons (edge-triggered, rate limited).

    Inputs (all optional): /mission/incident JSON ``kind`` in ``mission_kinds``; the mission
    state entering one of ``alarm_states``; /hardware_bridge/emergency ``active_emergency``
    rising; a new dir under the supervisor's crash dir (a black-box dump: crash, node down,
    emergency, lethal boundary)."""

    def __init__(self, mission_kinds, alarm_states=('EMERGENCY',), cooldown_s=30.0):
        self.mission_kinds = set(mission_kinds)
        self.alarm_states = set(alarm_states)
        self.cooldown_s = float(cooldown_s)
        self._last = None
        self._state = None
        self._emergency = False
        self._crash_seen = None
        self.suppressed = 0

    def _fire(self, reason, detail, now, force=False):
        if not force and self._last is not None and now - self._last < self.cooldown_s:
            self.suppressed += 1
            return None
        self._last = now
        return (safe_reason(reason), str(detail)[:500])

    def on_mission_incident(self, text, now):
        try:
            d = json.loads(text)
            kind = str(d.get('kind', ''))
        except (ValueError, AttributeError):
            return None
        if kind not in self.mission_kinds:
            return None
        return self._fire('mission_%s' % kind, text, now)

    def on_state(self, state_name, now):
        prev, self._state = self._state, str(state_name)
        if self._state in self.alarm_states and prev != self._state and prev is not None:
            return self._fire('state_%s' % self._state.lower(), 'from %s' % prev, now)
        return None

    def on_emergency(self, active, reason, now):
        prev, self._emergency = self._emergency, bool(active)
        if self._emergency and not prev:
            return self._fire('emergency', reason, now)
        return None

    def on_crash_dirs(self, names, now):
        """``names``: current dir names in the crash dir. The first call is the baseline."""
        names = set(names)
        if self._crash_seen is None:
            self._crash_seen = names
            return None
        new = sorted(names - self._crash_seen)
        self._crash_seen = names
        if not new:
            return None
        return self._fire('crash_%s' % new[-1].split('_', 1)[-1], ','.join(new), now)

    def manual(self, detail, now):
        return self._fire('manual', detail, now, force=True)


def pin_dir_name(wall_time, reason):
    return '%s_%s' % (time.strftime('%Y%m%d-%H%M%S', time.localtime(wall_time)),
                      safe_reason(reason))


def session_name(wall_time):
    return time.strftime('%Y%m%d-%H%M%S', time.localtime(wall_time))


def record_cmd(output, topics, split_s, max_cache_bytes, compression='zstd',
               storage='sqlite3', preset='resilient', polling_ms=5000):
    """``ros2 bag record`` argv (C++ recorder: no deserialization, no Python per message)."""
    cmd = ['ros2', 'bag', 'record', '-o', output,
           '--storage', storage,
           '--max-bag-duration', str(int(split_s)),
           '--max-cache-size', str(int(max_cache_bytes)),
           '--polling-interval', str(int(polling_ms))]
    if preset:
        cmd += ['--storage-preset-profile', preset]
    if compression:
        cmd += ['--compression-mode', 'file', '--compression-format', compression]
    return cmd + list(topics)
