# SPDX-License-Identifier: GPL-3.0-or-later
"""In-memory "black box" for ROS 2 Jazzy (pure Python; rosbag2_py only inside ``dump``).

The supervisor keeps the last ``window_s`` seconds of a few key topics in RAM as *serialized*
CDR bytes (no deserialization cost) and writes them out as a regular rosbag2 (sqlite3)
when something goes wrong. The result plays back with ``ros2 bag play <dir>/bag``. This is
independent of ``ros2 bag record --snapshot-mode`` (available since Iron/Jazzy): the buffer is
scoped to the supervisor and only the handful of topics that matter for a crash.

:class:`SnapshotBuffer`
    per-topic ring buffer with a time window, a per-topic minimum period (rate cap) and a
    global byte cap (oldest messages of the whole buffer are evicted first).
:func:`dump_bag`
    writes a snapshot (list of ``(topic, type, t_ns, bytes)``) with rosbag2_py.
"""

import collections
import heapq
import os


class SnapshotBuffer:
    def __init__(self, window_s=120.0, max_bytes=32 * 1024 * 1024):
        self.window_ns = int(window_s * 1e9)
        self.max_bytes = int(max_bytes)
        self._topics = {}          # topic -> dict(type, min_period_ns, last_ns, q)
        self.total_bytes = 0
        self.dropped_rate = 0
        self.evicted = 0

    def add_topic(self, topic, type_name, max_rate_hz=0.0):
        period = int(1e9 / max_rate_hz) if max_rate_hz and max_rate_hz > 0 else 0
        self._topics[topic] = {'type': type_name, 'min_period_ns': period, 'last_ns': None,
                               'q': collections.deque()}

    def topics(self):
        return {t: v['type'] for t, v in self._topics.items()}

    def push(self, topic, t_ns, data):
        """Store one serialized message received at ``t_ns``. Returns False if rate-capped."""
        entry = self._topics.get(topic)
        if entry is None:
            return False
        last = entry['last_ns']
        if last is not None and entry['min_period_ns'] and t_ns - last < entry['min_period_ns']:
            self.dropped_rate += 1
            return False
        data = bytes(data)
        entry['last_ns'] = t_ns
        entry['q'].append((t_ns, data))
        self.total_bytes += len(data)
        self._expire(t_ns)
        self._enforce_cap()
        return True

    def _expire(self, now_ns):
        horizon = now_ns - self.window_ns
        for entry in self._topics.values():
            q = entry['q']
            while q and q[0][0] < horizon:
                self.total_bytes -= len(q.popleft()[1])

    def _enforce_cap(self):
        while self.total_bytes > self.max_bytes:
            oldest = None
            for entry in self._topics.values():
                q = entry['q']
                if q and (oldest is None or q[0][0] < oldest['q'][0][0]):
                    oldest = entry
            if oldest is None:
                break
            self.total_bytes -= len(oldest['q'].popleft()[1])
            self.evicted += 1

    def snapshot(self, now_ns=None):
        """All buffered messages, time ordered: ``[(topic, type, t_ns, bytes)]``."""
        if now_ns is not None:
            self._expire(now_ns)
        streams = [[(t, topic, e['type'], d) for t, d in e['q']]
                   for topic, e in self._topics.items()]
        return [(topic, typ, t, d) for t, topic, typ, d in heapq.merge(*streams,
                                                                       key=lambda m: m[0])]

    def stats(self):
        return {'messages': sum(len(e['q']) for e in self._topics.values()),
                'bytes': self.total_bytes, 'rate_dropped': self.dropped_rate,
                'evicted': self.evicted}


def dump_bag(messages, bag_dir, writer_factory=None):
    """Write ``messages`` (from :meth:`SnapshotBuffer.snapshot`) to a rosbag2 at ``bag_dir``.

    ``writer_factory()`` -> object with ``open(storage, converter)``, ``create_topic(meta)``,
    ``write(topic, data, t_ns)``; defaults to ``rosbag2_py.SequentialWriter``. Returns the
    number of messages written.
    """
    import rosbag2_py  # noqa: deferred: only the dump needs it
    writer = writer_factory() if writer_factory else rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    created = set()
    for topic, typ, _t, _d in messages:
        if topic not in created:
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typ,
                                                         serialization_format='cdr'))
            created.add(topic)
    for topic, _typ, t_ns, data in messages:
        writer.write(topic, data, t_ns)
    del writer                     # closes the bag (metadata.yaml is written on destruction)
    return len(messages)


def make_dump_dir(root, stamp, reason):
    safe = ''.join(c if c.isalnum() or c in '-_.' else '_' for c in reason)[:48]
    path = os.path.join(root, '%s_%s' % (stamp, safe))
    n = 1
    while os.path.exists(path):
        path = os.path.join(root, '%s_%s.%d' % (stamp, safe, n))
        n += 1
    os.makedirs(path)
    return path
