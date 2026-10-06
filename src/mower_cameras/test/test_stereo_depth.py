import numpy as np
import pytest

sd = pytest.importorskip('mower_cameras.stereo_depth_node')


def _frame(raw12, grey):
    h, w = raw12.shape
    px = np.zeros((h, w, 3), np.uint8)
    px[:, :, 0] = raw12 & 0xFF
    px[:, :, 1] = ((raw12 >> 8) & 0x0F) | 0xA0     # high nibble = noise, must be ignored
    px[:, :, 2] = grey
    return px.tobytes()


def test_decode_and_depth_scale():
    raw = np.zeros((360, 640), np.uint16)
    raw[300, 320] = 1317                       # 41.16 px -> 0.551 m
    raw[10, 10] = 4095
    grey = np.full((360, 640), 77, np.uint8)
    r, g = sd.decode_simor(_frame(raw, grey))
    assert r[300, 320] == 1317 and r[10, 10] == 4095 and g[0, 0] == 77
    z = sd.disparity_to_depth(r, 22698.4, 32.0)
    assert z[300, 320] == pytest.approx(22.6984 / (1317 / 32.0), rel=1e-4)
    assert z[0, 0] == 0.0


def test_points_optical_axes_and_range():
    z = np.zeros((360, 640), np.float32)
    z[180, 480] = 2.0                          # right of centre -> +x
    z[340, 320] = 1.0                          # below centre -> +y
    z[0, 0] = 9.0                              # beyond max range -> dropped
    pts = sd.depth_to_points(z, 378.0, 378.0, 319.5, 179.5, step=1, min_z=0.2, max_z=4.0)
    assert len(pts) == 2
    by_z = {round(float(p[2]), 3): p for p in pts}
    assert by_z[2.0][0] > 0 and by_z[1.0][1] > 0
