import numpy as np
import pytest

from mower_cameras.v4l2 import split_yuyv_luma


def test_split_yuyv_luma_side_by_side():
    w, h = 8, 2
    buf = np.zeros((h, w * 2), np.uint8)
    buf[:, 1::2] = 128                       # chroma
    buf[:, 0:w:2] = 10                       # left eye luma (first 4 px)
    buf[:, w::2] = 200                       # right eye luma
    left, right = split_yuyv_luma(buf.tobytes(), w, h)
    assert left.shape == right.shape == (h, w // 2)
    assert (left == 10).all() and (right == 200).all()


def test_split_respects_bytesperline_padding():
    w, h, bpl = 4, 3, 12
    buf = np.full((h, bpl), 7, np.uint8)
    buf[:, 0:w * 2:2] = np.arange(w, dtype=np.uint8)
    left, right = split_yuyv_luma(memoryview(buf.tobytes()), w, h, bpl)
    assert left.tolist() == [[0, 1]] * h and right.tolist() == [[2, 3]] * h


def test_split_side_by_side_bgr_keeps_colour():
    import pytest
    pytest.importorskip('cv2')
    from mower_cameras.v4l2 import split_side_by_side_bgr
    w, h = 8, 2
    buf = np.zeros((h, w * 2), np.uint8)
    buf[:, 0::2] = 120                     # Y
    buf[:, 1::4] = 128                     # U neutral
    buf[:, 3::4] = 220                     # V high -> red
    left, right = split_side_by_side_bgr(buf.tobytes(), w, h, w * 2, 'YUYV')
    assert left.shape == right.shape == (h, w // 2, 3)
    b, g, r = left[0, 0].astype(int)
    assert r > g + 40 and r > b + 40


def test_eye_to_bgr_matches_full_split():
    import numpy as np
    from mower_cameras.v4l2 import eye_to_bgr, split_side_by_side_bgr
    pytest.importorskip('cv2')
    rng = np.random.default_rng(0)
    data = rng.integers(0, 255, 480 * 2560, dtype=np.uint8).tobytes()
    left, right = split_side_by_side_bgr(data, 1280, 480, 2560, 'YUYV')
    assert np.array_equal(eye_to_bgr(data, 1280, 480, 2560, 'YUYV', 0), left)
    assert np.array_equal(eye_to_bgr(data, 1280, 480, 2560, 'YUYV', 1), right)


def test_colour_format_only_swaps_y_first_422():
    from mower_cameras.v4l2 import colour_format
    assert colour_format('YUYV', 'YVYU') == 'YVYU'
    assert colour_format('YVYU', 'yuyv') == 'YUYV'
    assert colour_format('YUYV', '') == 'YUYV'
    assert colour_format('UYVY', 'YVYU') == 'UYVY'     # different luma positions: untouched
    assert colour_format('MJPG', 'YVYU') == 'MJPG'


def test_yvyu_decodes_byte1_as_v():
    """XC9080 order Y0 V Y1 U: a high byte 1 is red (V), not blue (as YUYV would read it)."""
    pytest.importorskip('cv2')
    from mower_cameras.v4l2 import eye_to_bgr
    w, h = 8, 2
    buf = np.zeros((h, w * 2), np.uint8)
    buf[:, 0::2] = 120
    buf[:, 1::4] = 220                     # byte 1 = V in YVYU
    buf[:, 3::4] = 128                     # byte 3 = U neutral
    data = buf.tobytes()
    b, g, r = eye_to_bgr(data, w, h, w * 2, 'YVYU', 0)[0, 0].astype(int)
    assert r > g + 40 and r > b + 40
    b, g, r = eye_to_bgr(data, w, h, w * 2, 'YUYV', 1)[0, 0].astype(int)
    assert b > g + 40 and b > r + 40       # the old (swapped) reading


def test_yvyu_equals_yuyv_of_chroma_swapped_buffer():
    pytest.importorskip('cv2')
    from mower_cameras.v4l2 import eye_to_bgr, to_bgr
    rng = np.random.default_rng(1)
    raw = rng.integers(0, 255, (48, 64 * 2), dtype=np.uint8)
    swapped = raw.copy()
    swapped[:, 1::4], swapped[:, 3::4] = raw[:, 3::4], raw[:, 1::4]
    for eye in (0, 1):
        assert np.array_equal(eye_to_bgr(raw.tobytes(), 64, 48, 128, 'YVYU', eye),
                              eye_to_bgr(swapped.tobytes(), 64, 48, 128, 'YUYV', eye))
    assert np.array_equal(to_bgr(raw.tobytes(), 64, 48, 128, 'YVYU', scale=2),
                          to_bgr(swapped.tobytes(), 64, 48, 128, 'YUYV', scale=2))
