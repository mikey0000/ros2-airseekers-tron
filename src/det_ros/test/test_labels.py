import os

import pytest

from det_ros.labels import BEST_LARGE_CLASSES, default_classes, label_for

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_builtin_list_matches_yaml():
    yaml = pytest.importorskip('yaml')
    with open(os.path.join(PKG, 'config', 'det.yaml')) as fh:
        params = yaml.safe_load(fh)['det_ros']['ros__parameters']
    assert params['classes'] == BEST_LARGE_CLASSES
    assert params['model_path'].startswith('/userdata/ros2/models/')
    assert len(BEST_LARGE_CLASSES) == 22


def test_default_classes_and_label_for():
    assert default_classes('/userdata/ros2/models/best_large_0208.rknn')[0] == 'person'
    assert default_classes('best_small_0208.rknn') == []
    assert label_for(1, BEST_LARGE_CLASSES) == 'dog'
    assert label_for(99, BEST_LARGE_CLASSES) == '99'
