# SPDX-License-Identifier: GPL-3.0-or-later
"""Low-overhead subscription handling for rclpy (ROS 2 Humble).

Why: on the RK3588 every wake of an rclpy Humble executor costs ~1.5-4 ms of pure Python
(the wait set is rebuilt from every entity of the node, each callback is wrapped in a Task,
MultiThreadedExecutor additionally hops through a thread pool and re-wakes on its guard
condition after every callback). Nodes that merely *cache* 100 Hz inputs burned 40-80 % of a
core doing that. Even outside the executor each received message still costs ~0.35 ms to take
and as much again to convert to a Python message, so the second lever is to take and convert
fewer messages: sample latest-value inputs at the rate they are consumed, pause inputs that
are only needed while an action runs, and parse the few fields of hot flat messages straight
from the CDR bytes.

How: subscriptions created through :meth:`SubscriptionPump.subscribe` live on the same node
(same topics and graph entry as before) but in a callback group whose ``can_execute()`` is
always False, so the node's normal executor never waits on or runs them. They are always
taken as serialized bytes and converted here (``deserialize_message``, a fast ``parser`` or
not at all for ``raw=True``). Two kinds of entries:

* event (default): a daemon thread waits on the active ones with one reused rcl wait set,
  drains every queued message and calls the callback right away (optionally under ``lock``).
  :meth:`set_active` pauses/resumes them; :meth:`flush` drops what queued while paused.
* sampled (``sampled=True``, use a depth-1 QoS): never waited on; :meth:`poll` takes what is
  queued and delivers only the newest message, optionally dropping it when it was received
  more than ``max_age`` seconds ago (``deliver_all=True``: every queued message, in order,
  for inputs like /tf that are only needed on demand). Call ``poll`` holding the same lock the callbacks need
  (or none), never while holding a lock a pump callback might wait for.

Callbacks of event entries run on the pump thread, concurrently with the node's executor:
they must be thread-safe (take the node's lock) or a ``lock`` must be passed to ``subscribe``.

Identical copies of this file live in the Python node packages that use it (there is no
shared Python library package in this workspace); keep them in sync.
"""

import contextlib
import struct
import threading
import time
import traceback
import types

from rclpy.callback_groups import CallbackGroup
from rclpy.serialization import deserialize_message

try:  # private in Humble, but stable for the whole distro
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
except ImportError:  # pragma: no cover - very old/new rclpy
    _rclpy = None


class _PumpOnlyGroup(CallbackGroup):
    """Callback group that rclpy executors never schedule (the pump owns it)."""

    def can_execute(self, entity):
        return False

    def beginning_execution(self, entity):
        return False

    def ending_execution(self, entity):
        pass


class _Entry:
    __slots__ = ('sub', 'callback', 'lock', 'convert', 'sampled', 'active', 'max_age_ns',
                 'deliver_all', 'with_receipt')

    def __init__(self, sub, callback, lock, convert, sampled, active, max_age, deliver_all,
                 with_receipt):
        self.sub = sub
        self.callback = callback
        self.lock = lock
        self.convert = convert
        self.sampled = sampled
        self.active = active
        self.max_age_ns = None if max_age is None else int(max_age * 1e9)
        self.deliver_all = bool(deliver_all)
        self.with_receipt = bool(with_receipt)


class SubscriptionPump:
    """Serve selected subscriptions of ``node`` without the rclpy executor.

    Usage::

        self._pump = SubscriptionPump(self)
        self._pump.subscribe(Odometry, '/odom', self._on_odom, 10, parser=parse_odometry)
        self._pump.subscribe(Battery, '/battery', self._on_batt, 1, sampled=True)
        self._pump.start()        # after all subscribe() calls
        self._pump.poll()         # wherever the sampled inputs are consumed
        self._pump.stop()         # before destroy_node()
    """

    WAIT_TIMEOUT_NS = 500_000_000   # re-check shutdown at least every 0.5 s
    MAX_TAKE_PER_WAKE = 256         # per subscription, keeps one topic from starving others

    @staticmethod
    def available():
        return _rclpy is not None

    def __init__(self, node, name='sub_pump'):
        self._node = node
        self._name = name
        self.group = _PumpOnlyGroup()
        self._entries = []
        self._by_sub = {}
        self._thread = None
        self._stopping = False
        self._generation = 0       # bumped by set_active() so the thread rebuilds its list
        self._wake = node.create_guard_condition(lambda: None, callback_group=self.group)
        self._errors_logged = set()

    def subscribe(self, msg_type, topic, callback, qos, *, raw=False, parser=None, lock=None,
                  sampled=False, active=True, max_age=None, deliver_all=False,
                  with_receipt=False):
        """``node.create_subscription`` served by the pump (see module docstring).

        ``parser(bytes)`` may return a duck-typed message or None (then the bytes are
        deserialized normally). ``max_age`` (s), ``deliver_all`` and ``with_receipt`` only
        apply to sampled entries; ``with_receipt`` calls ``callback(msg, receipt)`` with the
        time.monotonic() at which the message was received (not when it was polled).
        """
        if self._thread is not None:
            raise RuntimeError('SubscriptionPump.subscribe() after start()')
        if with_receipt and not sampled:
            # Event entries call callback(msg): a receipt callback would raise TypeError
            # on every message (this silently starved the cmd_vel_slew motion gate).
            raise ValueError('with_receipt=True requires sampled=True (%s)' % topic)
        sub = self._node.create_subscription(msg_type, topic, callback, qos,
                                             callback_group=self.group, raw=True)
        if raw:
            convert = None
        elif parser is not None:
            def convert(data, _parse=parser, _type=msg_type):
                msg = _parse(data)
                return msg if msg is not None else deserialize_message(data, _type)
        else:
            def convert(data, _type=msg_type):
                return deserialize_message(data, _type)
        entry = _Entry(sub, callback, lock, convert, bool(sampled), bool(active), max_age,
                       deliver_all, with_receipt)
        self._entries.append(entry)
        self._by_sub[sub] = entry
        return sub

    def start(self):
        if self._thread is None and any(not e.sampled for e in self._entries):
            self._thread = threading.Thread(target=self._run, name=self._name, daemon=True)
            self._thread.start()

    def stop(self, timeout=2.0):
        self._stopping = True
        self._kick()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout)

    def close(self, timeout=2.0):
        """Stop and destroy every subscription of this pump (it cannot be restarted)."""
        self.stop(timeout)
        for e in self._entries:
            self._node.destroy_subscription(e.sub)
        self._node.destroy_guard_condition(self._wake)
        self._entries = []
        self._by_sub = {}

    def set_active(self, subs, active):
        """Pause (``False``) or resume event subscriptions; paused ones are not taken."""
        for sub in subs:
            self._by_sub[sub].active = bool(active)
        self._generation += 1
        self._kick()

    def flush(self, sub):
        """Drop everything queued on a paused event or a sampled subscription."""
        h = sub.handle
        with h:
            while h.take_message(sub.msg_type, True) is not None:
                pass

    def poll(self, subs=None):
        """Deliver the newest queued message of each sampled subscription (all by default)."""
        entries = self._entries if subs is None else [self._by_sub[s] for s in subs]
        for e in entries:
            if not e.sampled:
                continue
            if e.deliver_all:
                self._drain(e)
                continue
            sub = e.sub
            last = None
            try:
                with sub.handle:
                    while True:
                        taken = sub.handle.take_message(sub.msg_type, True)
                        if taken is None:
                            break
                        last = taken
                if last is None:
                    continue
                age_ns = time.time_ns() - last[1]['received_timestamp']
                if e.max_age_ns is not None and age_ns > e.max_age_ns:
                    continue
                if e.with_receipt:
                    msg = last[0] if e.convert is None else e.convert(last[0])
                    receipt = time.monotonic() - max(0, age_ns) * 1e-9
                    if e.lock is None:
                        e.callback(msg, receipt)
                    else:
                        with e.lock:
                            e.callback(msg, receipt)
                else:
                    self._deliver(e, last[0])
            except Exception as exc:  # noqa: BLE001
                self._report(sub, exc)

    # ------------------------------------------------------------------
    def _kick(self):
        try:
            self._wake.trigger()
        except Exception:  # noqa: BLE001 - already destroyed
            pass

    def _report(self, sub, exc):
        key = (sub.topic_name, type(exc).__name__)
        if key not in self._errors_logged:
            self._errors_logged.add(key)
            self._node.get_logger().error(
                'callback for %s raised (logged once per error type, inputs keep flowing):\n%s'
                % (sub.topic_name, traceback.format_exc()))

    @staticmethod
    def _deliver(e, data):
        msg = data if e.convert is None else e.convert(data)
        if e.lock is None:
            e.callback(msg)
        else:
            with e.lock:
                e.callback(msg)

    def _drain(self, e):
        sub = e.sub
        handle = sub.handle
        with handle:
            for _ in range(self.MAX_TAKE_PER_WAKE):
                taken = handle.take_message(sub.msg_type, True)
                if taken is None:
                    return
                try:
                    self._deliver(e, taken[0])
                except Exception as exc:  # noqa: BLE001 - one bad message must not stop inputs
                    self._report(sub, exc)

    def _run(self):
        ctx = self._node.context
        events = [e for e in self._entries if not e.sampled]
        try:
            with contextlib.ExitStack() as stack:
                # Hold the handles for the lifetime of the loop: rclpy defers destroying an
                # entity that is in use, so destroy_node() can never free one under the wait.
                for e in events:
                    stack.enter_context(e.sub.handle)
                stack.enter_context(self._wake.handle)
                wait_set = _rclpy.WaitSet(len(events), 1, 0, 0, 0, 0, ctx.handle)
                wake_handle = self._wake.handle
                generation = None
                active = []
                while not self._stopping and ctx.ok():
                    if generation != self._generation:
                        generation = self._generation
                        active = [(e.sub.handle, e.sub.handle.pointer, e)
                                  for e in events if e.active]
                    wait_set.clear_entities()
                    for h, _p, _e in active:
                        wait_set.add_subscription(h)
                    wait_set.add_guard_condition(wake_handle)
                    wait_set.wait(self.WAIT_TIMEOUT_NS)
                    if self._stopping:
                        break
                    ready = wait_set.get_ready_entities('subscription')
                    if not ready:
                        continue
                    for _h, ptr, e in active:
                        if ptr in ready and e.active:
                            self._drain(e)
        except Exception:  # noqa: BLE001 - shutdown races (InvalidHandle, RCLError)
            if not self._stopping and ctx.ok():
                self._node.get_logger().error(
                    'subscription pump stopped unexpectedly:\n%s' % traceback.format_exc())


class PeriodicRunner:
    """Run ``[(period_s, fn), ...]`` on one plain thread instead of rclpy timers.

    Each rclpy timer firing costs a full executor wake (wait-set rebuild, Task, thread-pool
    hop on a MultiThreadedExecutor), ~1.5-3 ms of Python on this CPU; a sleeping thread costs
    nothing between deadlines. The tasks run sequentially (like timers sharing one mutually
    exclusive callback group), each on its own fixed-rate schedule that skips instead of
    bursting after a stall. Exceptions are logged once per (task, type) and the schedule
    continues. Stops by itself when the node's context shuts down.
    """

    def __init__(self, node, tasks, name='periodic'):
        self._node = node
        self._tasks = [(float(p), fn) for p, fn in tasks]
        self._name = name
        self._stop = threading.Event()
        self._thread = None
        self._errors_logged = set()

    def start(self):
        if self._thread is None and self._tasks:
            self._thread = threading.Thread(target=self._run, name=self._name, daemon=True)
            self._thread.start()

    def stop(self, timeout=2.0):
        self._stop.set()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout)

    def _run(self):
        ctx = self._node.context
        start = time.monotonic()
        due = [start + p for p, _fn in self._tasks]
        while not self._stop.is_set():
            wait = min(due) - time.monotonic()
            if wait > 0 and self._stop.wait(wait):
                break
            if not ctx.ok():
                break
            now = time.monotonic()
            for idx, (period, fn) in enumerate(self._tasks):
                if now < due[idx]:
                    continue
                due[idx] += period
                if due[idx] <= now:
                    due[idx] = now + period
                try:
                    fn()
                except Exception as exc:  # noqa: BLE001 - keep the other tasks running
                    key = (getattr(fn, '__name__', repr(fn)), type(exc).__name__)
                    if key not in self._errors_logged and ctx.ok():
                        self._errors_logged.add(key)
                        self._node.get_logger().error(
                            'periodic task %s raised (logged once per error type):\n%s'
                            % (key[0], traceback.format_exc()))


# ----------------------------------------------------------------------------
# Fast CDR parsers (XCDR1 little endian, what rmw_fastrtps/cyclonedds emit here). They return
# duck-typed objects with the same attribute names as the ROS message, or None for anything
# unexpected (the pump then falls back to deserialize_message).
# ----------------------------------------------------------------------------
_NS = types.SimpleNamespace
_PRIMITIVES = {
    'boolean': '?', 'octet': 'B', 'uint8': 'B', 'int8': 'b', 'char': 'B', 'byte': 'B',
    'uint16': 'H', 'int16': 'h', 'uint32': 'I', 'int32': 'i', 'uint64': 'Q', 'int64': 'q',
    'float': 'f', 'double': 'd',
}


def _align(pos, n):
    # CDR alignment is relative to the payload, which starts after the 4-byte encapsulation.
    return 4 + ((pos - 4 + n - 1) & -n)


def _string(data, pos):
    pos = _align(pos, 4)
    (n,) = struct.unpack_from('<I', data, pos)
    if n == 0 or pos + 4 + n > len(data):
        raise ValueError('bad CDR string')
    return bytes(data[pos + 4:pos + 3 + n]).decode('utf-8'), pos + 4 + n


def _header(data):
    sec, nanosec = struct.unpack_from('<iI', data, 4)
    frame_id, pos = _string(data, 12)
    return _NS(stamp=_NS(sec=sec, nanosec=nanosec), frame_id=frame_id), pos


def _le_cdr(data):
    return len(data) >= 4 and data[0] == 0 and data[1] == 1


def flat_parser(msg_type):
    """Parser for a message made of an optional leading std_msgs/Header plus primitives.

    Returns None when ``msg_type`` has any other kind of field.
    """
    fields = list(msg_type.get_fields_and_field_types().items())
    has_header = bool(fields) and fields[0] == ('header', 'std_msgs/Header')
    if has_header:
        fields = fields[1:]
    if not fields or any(t not in _PRIMITIVES for _n, t in fields):
        return None
    names = tuple(n for n, _t in fields)
    layout = [(_PRIMITIVES[t], struct.calcsize(_PRIMITIVES[t])) for _n, t in fields]
    single = all(size == 1 for _f, size in layout)
    fmt1 = '<' + ''.join(f for f, _s in layout)
    size1 = struct.calcsize(fmt1)

    def parse(data):
        if not _le_cdr(data):
            return None
        try:
            if has_header:
                header, pos = _header(data)
            else:
                header, pos = None, 4
            if single:
                if pos + size1 > len(data):
                    return None
                values = struct.unpack_from(fmt1, data, pos)
            else:
                values = []
                for fmt, size in layout:
                    pos = _align(pos, size)
                    values.append(struct.unpack_from('<' + fmt, data, pos)[0])
                    pos += size
        except (ValueError, struct.error, UnicodeDecodeError):
            return None
        msg = _NS(**dict(zip(names, values)))
        if has_header:
            msg.header = header
        return msg
    return parse


_ODOM_TAIL = struct.Struct('<7d36d6d36d')


def parse_odometry(data):
    """nav_msgs/Odometry from CDR bytes (all fields, covariances as tuples)."""
    if not _le_cdr(data):
        return None
    try:
        header, pos = _header(data)
        child, pos = _string(data, pos)
        pos = _align(pos, 8)
        v = _ODOM_TAIL.unpack_from(data, pos)
    except (ValueError, struct.error, UnicodeDecodeError):
        return None
    pose = _NS(position=_NS(x=v[0], y=v[1], z=v[2]),
               orientation=_NS(x=v[3], y=v[4], z=v[5], w=v[6]))
    twist = _NS(linear=_NS(x=v[43], y=v[44], z=v[45]), angular=_NS(x=v[46], y=v[47], z=v[48]))
    return _NS(header=header, child_frame_id=child,
               pose=_NS(pose=pose, covariance=v[7:43]),
               twist=_NS(twist=twist, covariance=v[49:85]))


def odometry_frames(data):
    """``(frame_id, child_frame_id)`` of serialized nav_msgs/Odometry, or None."""
    if not _le_cdr(data):
        return None
    try:
        frame, pos = _string(data, 12)
        child, _ = _string(data, pos)
    except (ValueError, struct.error, UnicodeDecodeError):
        return None
    return frame, child


def _cdr_string_bytes(text, pos):
    """(bytes, new_pos) encoding ``text`` as a CDR string at payload offset ``pos``."""
    raw = text.encode('utf-8') + b'\0'
    pad = _align(pos, 4) - pos
    return b'\0' * pad + struct.pack('<I', len(raw)) + raw, pos + pad + 4 + len(raw)


def odometry_with_frames(data, frame_id, child_frame_id):
    """Serialized nav_msgs/Odometry ``data`` with both frame ids replaced, or None.

    Byte-identical to deserialize -> set frame ids -> serialize (unit tested).
    """
    if not _le_cdr(data):
        return None
    try:
        _frame, pos = _string(data, 12)
        _child, pos = _string(data, pos)
    except (ValueError, struct.error, UnicodeDecodeError):
        return None
    tail = _align(pos, 8)                    # pose: first 8-byte field after the strings
    out_frame, p = _cdr_string_bytes(frame_id, 12)
    out_child, p = _cdr_string_bytes(child_frame_id, p)
    pad = _align(p, 8) - p
    return b''.join((bytes(data[:12]), out_frame, out_child, b'\0' * pad, bytes(data[tail:])))


_IMU_TAIL = struct.Struct('<4d9d3d9d3d9d')


def parse_imu(data):
    """sensor_msgs/Imu from CDR bytes (all fields, covariances as tuples)."""
    if not _le_cdr(data):
        return None
    try:
        header, pos = _header(data)
        v = _IMU_TAIL.unpack_from(data, _align(pos, 8))
    except (ValueError, struct.error, UnicodeDecodeError):
        return None
    return _NS(header=header,
               orientation=_NS(x=v[0], y=v[1], z=v[2], w=v[3]),
               orientation_covariance=v[4:13],
               angular_velocity=_NS(x=v[13], y=v[14], z=v[15]),
               angular_velocity_covariance=v[16:25],
               linear_acceleration=_NS(x=v[25], y=v[26], z=v[27]),
               linear_acceleration_covariance=v[28:37])


# ----------------------------------------------------------------------------
# Fast CDR serializers for hot publishers: Publisher.publish(bytes) skips the message object,
# the Python->C conversion and rosidl serialization (~0.4 ms per publish on this CPU).
# ----------------------------------------------------------------------------
_ENCAPSULATION = b'\x00\x01\x00\x00'


def _header_bytes(stamp_ns, frame_id):
    sec, nanosec = divmod(int(stamp_ns), 1_000_000_000)
    frame, pos = _cdr_string_bytes(frame_id, 12)
    return struct.pack('<iI', sec, nanosec) + frame, pos


def flat_serializer(msg_type):
    """``serialize(values: dict, stamp_ns=0, frame_id='') -> bytes`` for the message kinds
    :func:`flat_parser` handles; fields missing from ``values`` keep the message defaults.
    Returns None for other message types.
    """
    fields = list(msg_type.get_fields_and_field_types().items())
    has_header = bool(fields) and fields[0] == ('header', 'std_msgs/Header')
    if has_header:
        fields = fields[1:]
    if not fields or any(t not in _PRIMITIVES for _n, t in fields):
        return None
    defaults = msg_type()
    layout = [(n, _PRIMITIVES[t], struct.calcsize(_PRIMITIVES[t]), getattr(defaults, n))
              for n, t in fields]
    single = all(size == 1 for _n, _f, size, _d in layout)
    packer = struct.Struct('<' + ''.join(f for _n, f, _s, _d in layout))

    def serialize(values, stamp_ns=0, frame_id=''):
        if has_header:
            head, pos = _header_bytes(stamp_ns, frame_id)
        else:
            head, pos = b'', 4
        if single:
            body = packer.pack(*[values.get(n, d) for n, _f, _s, d in layout])
        else:
            parts = []
            for n, fmt, size, d in layout:
                pad = _align(pos, size) - pos
                parts.append(b'\0' * pad + struct.pack('<' + fmt, values.get(n, d)))
                pos += pad + size
            body = b''.join(parts)
        out = _ENCAPSULATION + head + body
        return out + b'\0' * (-len(out) % 4)      # Fast-CDR also pads the end to 4 bytes
    return serialize


_ZERO36 = (0.0,) * 36


def odometry_bytes(stamp_ns, frame_id, child_frame_id, position, orientation,
                   linear, angular, pose_covariance=_ZERO36, twist_covariance=_ZERO36):
    """Serialized nav_msgs/Odometry (position/linear/angular xyz, orientation xyzw)."""
    head, pos = _header_bytes(stamp_ns, frame_id)
    child, pos = _cdr_string_bytes(child_frame_id, pos)
    pad = _align(pos, 8) - pos
    return b''.join((_ENCAPSULATION, head, child, b'\0' * pad,
                     _ODOM_TAIL.pack(*position, *orientation, *pose_covariance,
                                     *linear, *angular, *twist_covariance)))
