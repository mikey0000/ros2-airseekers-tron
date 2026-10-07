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
