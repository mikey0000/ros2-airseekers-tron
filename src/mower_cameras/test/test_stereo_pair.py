"""Pair splitter, stamp mapping and decimation gate of stereo_cam (ROS-free)."""
import numpy as np
import pytest

from mower_cameras.image_cdr import ImageCdr
from mower_cameras.stereo_pair import DecimationGate, mono_to_ros_ns, split_luma_into


def _frame(w, h, bpl, left_y, right_y, luma_off=0):
    buf = np.full((h, bpl), 77, np.uint8)            # chroma / padding
    half = w // 2
    buf[:, luma_off:half * 2:2] = left_y
    buf[:, half * 2 + luma_off:w * 2:2] = right_y
    return buf.tobytes()


@pytest.mark.parametrize('fmt,off', [('YUYV', 0), ('YVYU', 0), ('UYVY', 1)])
def test_split_luma_into_both_eyes_same_buffer(fmt, off):
    w, h = 16, 4
    data = _frame(w, h, w * 2, 10, 200, off)
    left = np.zeros((h, w // 2), np.uint8)
    right = np.zeros_like(left)
    assert split_luma_into(data, w, h, 0, fmt, left, right)
    assert (left == 10).all() and (right == 200).all()


def test_split_respects_padding_and_gradient():
    w, h, bpl = 8, 3, 24
    buf = np.zeros((h, bpl), np.uint8)
    buf[:, 0:w * 2:2] = np.arange(w, dtype=np.uint8) + 1      # luma 1..8
    left = np.zeros((h, 4), np.uint8)
    right = np.zeros_like(left)
    assert split_luma_into(memoryview(buf.tobytes()), w, h, bpl, 'YUYV', left, right)
    assert left.tolist() == [[1, 2, 3, 4]] * h and right.tolist() == [[5, 6, 7, 8]] * h


def test_split_into_preserialised_images_real_size():
    rng = np.random.default_rng(1)
    data = rng.integers(0, 255, 480 * 2560, dtype=np.uint8).tobytes()
    lc, rc = (ImageCdr('vio_camera', 480, 640, 'mono8', 1) for _ in range(2))
    assert split_luma_into(data, 1280, 480, 2560, 'YUYV', lc.image, rc.image)
    y = np.frombuffer(data, np.uint8).reshape(480, 2560)[:, 0::2]
    assert np.array_equal(lc.image, y[:, :640]) and np.array_equal(rc.image, y[:, 640:])
    # same stamp in both serialised messages
    a, b = lc.serialize(1_700_000_000_123_456_789), rc.serialize(1_700_000_000_123_456_789)
    assert a[4:12] == b[4:12]


def test_split_truncated_and_bad_format():
    left = np.zeros((4, 4), np.uint8)
    right = np.zeros_like(left)
    assert not split_luma_into(b'\0' * 10, 8, 4, 16, 'YUYV', left, right)
    with pytest.raises(ValueError):
        split_luma_into(b'\0' * 64, 8, 4, 16, 'NV12', left, right)


def test_mono_to_ros_ns():
    assert mono_to_ros_ns(5_000, 10_000, 1_000_010_000) == 1_000_005_000


def _kept(src_hz, out_hz, seconds=10.0, jitter=0.0, seed=0):
    rng = np.random.default_rng(seed)
    gate = DecimationGate(1.0 / out_hz, 0.5 / src_hz)
    ts = np.arange(0, seconds, 1.0 / src_hz) + rng.uniform(-jitter, jitter, int(seconds * src_hz))
    return [t for t in ts if gate.keep(t)]


@pytest.mark.parametrize('src', [25.0, 30.0, 20.0])
def test_gate_average_rate(src):
    kept = _kept(src, 10.0, jitter=0.002)
    rate = (len(kept) - 1) / (kept[-1] - kept[0])
    assert abs(rate - 10.0) < 0.3
    assert max(np.diff(kept)) <= np.ceil(src / 10.0) / src + 0.01   # minimal skipping


def test_gate_slow_source_and_gap():
    kept = _kept(5.0, 10.0)
    assert len(kept) == 50                              # slower source: keep all
    g = DecimationGate(0.1, 0.02)
    assert g.keep(0.0) and not g.keep(0.05) and g.keep(0.1)
    assert g.keep(5.0)                                  # after a stall: keep, re-anchor
    assert not g.keep(5.04) and g.keep(5.1)


def test_gate_disabled():
    g = DecimationGate(0.0)
    assert all(g.keep(t) for t in (0, 0, 0.001))


def test_clock_map_smooths_jitter_and_takes_steps():
    from mower_cameras.stereo_pair import ClockMap
    state = {'m': 0, 'r': 1_000_000_000_000, 'jit': 0}

    def mono():
        state['m'] += 1_000
        return state['m']

    def real():
        state['jit'] = -state['jit'] or 5_000         # +-5 us read jitter
        return state['m'] + 1_000_000_000_000 + state['jit']
    cm = ClockMap(period_s=0.0, alpha=0.05, clocks=(mono, real))
    offs = [cm.to_ros(0) for _ in range(200)]
    assert max(offs) - min(offs) <= 10_000            # bounded by the jitter, no drift
    assert np.std(offs[100:]) < 1_000                 # low-passed
    state['r'] = 0
    real2 = lambda: state['m'] + 2_000_000_000_000   # noqa: E731  clock set: 1000 s jump
    cm._real = real2
    assert abs(cm.to_ros(0) - 2_000_000_000_000) < 10_000
