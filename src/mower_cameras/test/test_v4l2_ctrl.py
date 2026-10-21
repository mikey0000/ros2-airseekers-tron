"""v4l2_ctrl: exposure set via ioctls, graceful degradation (no camera needed)."""
import errno

from mower_cameras import v4l2_ctrl as vc


class FakeCam:
    def __init__(self, controls=None, fail_open=False):
        # cid -> [value, min, max]
        self.c = controls if controls is not None else {
            vc.CID_EXPOSURE_AUTO: [3, 0, 3], vc.CID_EXPOSURE_ABSOLUTE: [156, 3, 2047]}
        self.fail_open = fail_open
        self.opened = 0
        self.closed = 0

    def open(self, _dev):
        if self.fail_open:
            raise OSError(errno.ENOENT, 'No such file or directory')
        self.opened += 1
        return 7

    def close(self, _fd):
        self.closed += 1

    def ioctl(self, _fd, req, arg):
        if arg.id not in self.c:
            raise OSError(errno.EINVAL, 'Invalid argument')
        ent = self.c[arg.id]
        if req == vc.VIDIOC_QUERYCTRL:
            arg.minimum, arg.maximum, arg.step = ent[1], ent[2], 1
        elif req == vc.VIDIOC_G_CTRL:
            arg.value = ent[0]
        elif req == vc.VIDIOC_S_CTRL:
            ent[0] = arg.value
        return 0

    def ctl(self):
        return vc.V4L2Controls('/dev/rear_camera', self.open, self.ioctl, self.close)


def test_ioctl_numbers_match_kernel():
    assert vc.VIDIOC_G_CTRL == 0xc008561b
    assert vc.VIDIOC_S_CTRL == 0xc008561c
    assert vc.VIDIOC_QUERYCTRL == 0xc0445624


def test_manual_then_auto():
    cam = FakeCam()
    ok, msg = cam.ctl().apply(100)
    assert ok, msg
    assert cam.c[vc.CID_EXPOSURE_AUTO][0] == vc.EXPOSURE_MANUAL
    assert cam.c[vc.CID_EXPOSURE_ABSOLUTE][0] == 100
    ok, _ = cam.ctl().apply(-1)
    assert ok and cam.c[vc.CID_EXPOSURE_AUTO][0] == vc.EXPOSURE_APERTURE_PRIORITY
    assert cam.opened == cam.closed


def test_manual_clamped_to_range():
    cam = FakeCam()
    ok, msg = cam.ctl().apply(1)
    assert ok and cam.c[vc.CID_EXPOSURE_ABSOLUTE][0] == 3 and 'clamped' in msg


def test_missing_control_degrades():
    cam = FakeCam(controls={})
    ok, msg = cam.ctl().apply(50)
    assert not ok and 'exposure_absolute' in msg
    ok, msg = cam.ctl().apply(-1)
    assert not ok


def test_missing_device_degrades():
    ok, msg = FakeCam(fail_open=True).ctl().apply(-1)
    assert not ok and 'open' in msg
