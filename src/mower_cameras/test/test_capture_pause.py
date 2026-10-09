"""CaptureLoop.pause_fn: no device open / no frames while paused, resumes after."""
import time

from mower_cameras.v4l2_node import CaptureLoop


class _Log:
    def info(self, *_a): pass
    warning = error = info


def test_pause_never_opens_then_resumes():
    opened = []
    loop = CaptureLoop(_Log(), '/dev/null', 4, 4, 'RGB3', 0, lambda d, c: None)
    loop.pause_poll_s = 0.01
    state = {'pause': True}
    loop.pause_fn = lambda: state['pause']

    def fake_open():
        opened.append(time.monotonic())
        raise OSError('test')
    loop._open = fake_open
    loop.start()
    time.sleep(0.1)
    assert opened == [] and loop.paused
    state['pause'] = False
    time.sleep(0.1)
    loop.stop()
    loop.join(2.0)
    assert opened and not loop.paused


class _FakeCap:
    """Minimal V4L2Capture stand-in: one buffer, ~200 fps."""

    def __init__(self, log):
        self.log = log
        self.closed = False

    def dequeue(self, timeout=2.0):
        time.sleep(0.005)
        return 0, 1, 0

    def view(self, index, used):
        return b'x'

    def requeue(self, index):
        pass

    def close(self):
        self.closed = True
        self.log.append('close')


def _unused_loop(want, close_after):
    events, frames = [], []
    loop = CaptureLoop(_Log(), '/dev/null', 4, 4, 'GREY', 0, lambda d, c: frames.append(1),
                       want=want, close_when_unused_s=close_after)
    loop.pause_poll_s = 0.01

    def fake_open():
        events.append('open')
        return _FakeCap(events)
    loop._open = fake_open
    return loop, events, frames


def _wait(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False


def test_unused_not_opened_at_start_then_opens_on_subscriber():
    state = {'subs': 0}
    loop, events, frames = _unused_loop(lambda: state['subs'] > 0, 0.2)
    loop.start()
    try:
        time.sleep(0.15)
        assert events == [] and loop.paused and loop.unused   # nobody listens: never opened
        state['subs'] = 1
        assert _wait(lambda: frames)                           # opens within a poll
        assert events == ['open'] and not loop.paused
    finally:
        loop.stop()
        loop.join(2.0)


def test_unused_closes_after_linger_and_reopens():
    state = {'subs': 1}
    loop, events, frames = _unused_loop(lambda: state['subs'] > 0, 0.15)
    loop.start()
    try:
        assert _wait(lambda: frames)
        state['subs'] = 0
        t0 = time.monotonic()
        assert _wait(lambda: 'close' in events)
        assert time.monotonic() - t0 >= 0.12                   # kept open for the linger
        assert loop.paused and loop.unused and loop.cap is None
        n = len(frames)
        time.sleep(0.05)
        assert len(frames) == n                                # closed: no frames at all
        state['subs'] = 1
        assert _wait(lambda: len(frames) > n)
        assert events == ['open', 'close', 'open'] and not loop.paused
    finally:
        loop.stop()
        loop.join(2.0)


def test_short_gap_does_not_cycle_device():
    """A viewer that reconnects (or a snapshot poller) inside the linger keeps it open."""
    state = {'subs': 1}
    loop, events, frames = _unused_loop(lambda: state['subs'] > 0, 0.5)
    loop.start()
    try:
        assert _wait(lambda: frames)
        for _ in range(3):
            state['subs'] = 0
            time.sleep(0.1)
            state['subs'] = 1
            time.sleep(0.05)
        assert events == ['open']
    finally:
        loop.stop()
        loop.join(2.0)


def test_close_when_unused_disabled_keeps_streaming():
    state = {'subs': 0}
    loop, events, frames = _unused_loop(lambda: state['subs'] > 0, 0.0)
    loop.start()
    try:
        assert _wait(lambda: events == ['open'])
        time.sleep(0.1)
        assert events == ['open'] and not loop.paused and frames == []   # skip, not close
    finally:
        loop.stop()
        loop.join(2.0)


def test_pause_fn_wins_over_want():
    state = {'subs': 1, 'pause': True}
    loop, events, frames = _unused_loop(lambda: state['subs'] > 0, 0.1)
    loop.pause_fn = lambda: state['pause']
    loop.start()
    try:
        time.sleep(0.1)
        assert events == [] and loop.paused and not loop.unused
        state['pause'] = False
        assert _wait(lambda: frames)
    finally:
        loop.stop()
        loop.join(2.0)
