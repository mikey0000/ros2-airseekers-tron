import numpy as np
import pytest

from det_ros.yolo_postprocess import _dfl, _box_process, post_process

IMG = (480, 640)  # (W, H) of the shipped model


def test_dfl_soft_argmax():
    # one anchor, bin k has overwhelming mass in every coord group -> argmax == k
    reg_max = 16
    d = np.full((1, 4 * reg_max, 1, 1), 1.0, dtype=np.float32)
    k = 7
    d[:, k, 0, 0] = 100.0          # coord 0, bin k
    d[:, 16 + 3, 0, 0] = 100.0     # coord 1, bin 3
    d[:, 32 + 9, 0, 0] = 100.0     # coord 2, bin 9
    d[:, 48 + 12, 0, 0] = 100.0    # coord 3, bin 12
    out = _dfl(d)  # [1, 4, 1, 1]
    assert out.shape == (1, 4, 1, 1)
    expect = np.array([k, 3, 9, 12], dtype=np.float32)
    assert np.allclose(out.reshape(4), expect, atol=1e-3)


def test_box_process_stride_non_square():
    grid_h, grid_w = 80, 60
    # peak bin 0 of all four coords -> DFL offset ~0, so x1 == (grid_x+0.5)*stride
    box = np.zeros((1, 64, grid_h, grid_w), dtype=np.float32)
    box[:, 0::16, :, :] = 100.0  # bin 0 of each coord group
    out = _box_process(box, IMG)
    assert out.shape == (1, 4, grid_h, grid_w)

    # stride must be 8 uniformly for 640x480 at stride-8 (80x60 grid)
    assert abs(IMG[0] / grid_w - 8.0) < 1e-6  # x stride
    assert abs(IMG[1] / grid_h - 8.0) < 1e-6  # y stride
    x1 = out[0, 0, 0, 0]
    assert abs(x1 - (0 + 0.5) * 8.0) < 1e-3


def _outputs(grid_h, grid_w, cls=22, score=0.9):
    """Build one scale's three tensors (box, class, score_sum) with a known object."""
    box = np.zeros((1, 64, grid_h, grid_w), dtype=np.float32)
    cls_t = np.full((1, cls, grid_h, grid_w), 0.01, dtype=np.float32)
    cls_t[:, 3, grid_h // 2, grid_w // 2] = score
    ss = np.ones((1, 1, grid_h, grid_w), dtype=np.float32)
    return box, cls_t, ss


def test_post_process_detects_one():
    scales = [(80, 60), (40, 30), (20, 15)]
    tensors = []
    for gh, gw in scales:
        box, cls_t, ss = _outputs(gh, gw, cls=22, score=0.9)
        tensors.extend([box, cls_t, ss])
    boxes, classes, scores = post_process(tensors, img_size=IMG)
    assert boxes is not None
    assert boxes.shape[1] == 4  # xyxy
    assert len(classes) >= 1
    assert classes[0] == 3  # the class we boosted
    assert scores[0] > 0.8


def test_post_process_none_when_empty():
    tensors = []
    for gh, gw in [(80, 60), (40, 30), (20, 15)]:
        box, cls_t, ss = _outputs(gh, gw, cls=22, score=0.0)
        tensors.extend([box, cls_t, ss])
    boxes, classes, scores = post_process(tensors, img_size=IMG)
    assert boxes is None and classes is None and scores is None


def test_fast_post_process_matches_reference():
    import numpy as np
    from det_ros.yolo_postprocess import post_process, post_process_reference
    rng = np.random.default_rng(3)
    outs = []
    for gh, gw in ((80, 60), (40, 30), (20, 15)):
        outs.append(rng.normal(0, 2, (1, 64, gh, gw)).astype(np.float32))
        cls = rng.random((1, 22, gh, gw)).astype(np.float32) * 0.3
        cls[0, rng.integers(0, 22, 40), rng.integers(0, gh, 40), rng.integers(0, gw, 40)] = 0.9
        outs.append(cls)
        outs.append(np.ones((1, 1, gh, gw), np.float32))
    a = post_process(outs, (480, 640), 0.25, 0.45)
    b = post_process_reference(outs, (480, 640), 0.25, 0.45)
    oa, ob = np.lexsort((a[2], a[1])), np.lexsort((b[2], b[1]))
    assert np.allclose(a[0][oa], b[0][ob], atol=1e-3)
    assert (a[1][oa] == b[1][ob]).all()
    assert np.allclose(a[2][oa], b[2][ob])
