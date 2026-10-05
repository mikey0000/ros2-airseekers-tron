import os

from mower_rknn.model_paths import (DEVICE_MODELS_DIR, candidate_model_paths,
                                    resolve_model_path)
from mower_rknn.startup import startup_problem


def test_candidates_order_and_dedup():
    c = candidate_model_paths('/userdata/ros2/models/a.rknn', '/tmp/m')
    assert c == ['/userdata/ros2/models/a.rknn', '/tmp/m/a.rknn']
    c = candidate_model_paths('model/a.rknn', '')
    assert c == ['model/a.rknn', os.path.join(DEVICE_MODELS_DIR, 'a.rknn')]


def test_resolve_falls_back_to_models_dir(tmp_path):
    (tmp_path / 'best_large_0208.rknn').write_bytes(b'x')
    got = resolve_model_path('/nonexistent/best_large_0208.rknn', str(tmp_path))
    assert got == str(tmp_path / 'best_large_0208.rknn')
    assert resolve_model_path('/nonexistent/other.rknn', str(tmp_path)) is None


def test_startup_problem_mentions_fix(tmp_path):
    # On a dev host rknnlite is absent -> runtime problem; on the NPU -> model problem.
    _, problem = startup_problem('/nonexistent/x.rknn', str(tmp_path))
    assert problem is not None
    assert 'rknn-toolkit-lite2' in problem or 'install_models.sh' in problem
