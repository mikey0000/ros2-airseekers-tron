import numpy as np

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
