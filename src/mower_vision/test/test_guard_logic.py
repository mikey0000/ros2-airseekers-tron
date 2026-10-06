import os

import pytest

from mower_vision.guard_logic import (DEFAULT_CLASSES, DEFAULT_WHITELIST, Box, GuardConfig,
                                      GuardState, any_close, box_outline, class_label,
                                      classify, danger_line, in_danger_zone, is_whitelisted,
                                      with_image_size)

CFG = GuardConfig(y_frac=0.6, w_frac=0.15, min_score=0.4, image_width=1920, image_height=1080)


def box(label='person', score=0.9, x1=800, y1=500, x2=1200, y2=1000):
    return Box(label, score, (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1)


def test_box_edges():
    b = box(x1=10, y1=20, x2=110, y2=220)
    assert (b.x1, b.y1, b.x2, b.y2) == (10, 20, 110, 220)


def test_close_when_low_and_wide():
    assert in_danger_zone(box(), CFG)          # bottom 1000 > 648, width 400 > 288


def test_not_close_when_high_in_image():
    assert not in_danger_zone(box(y1=100, y2=600), CFG)   # bottom 600 < 648


def test_not_close_when_narrow():
    assert not in_danger_zone(box(x1=900, x2=1100), CFG)  # width 200 < 288


def test_boundary_is_exclusive():
    b = box(x1=0, x2=288, y1=0, y2=648)                  # exactly on both thresholds
    assert not in_danger_zone(b, CFG)


def test_whitelist_filters_classes():
    assert any_close([box('dog')], CFG)
    assert not any_close([box('stone')], CFG)
    assert is_whitelisted('Person ', DEFAULT_WHITELIST)
    assert is_whitelisted('stone', ['*'])


def test_min_score():
    verdicts = classify([box(score=0.39), box(score=0.41)], CFG)
    assert [(r, c) for _, r, c in verdicts] == [(False, False), (True, True)]


def test_class_label_index_and_name():
    assert class_label('1', DEFAULT_CLASSES) == 'dog'
    assert class_label('person', DEFAULT_CLASSES) == 'person'
    assert class_label('99', DEFAULT_CLASSES) == '99'


def test_rising_edge_once_and_burst_window():
    s = GuardState(hold_s=0.5, burst_s=1.0)
    assert s.update('left', True, 10.0) == (True, True)
    assert s.update('left', True, 10.1) == (True, False)   # no second edge
    assert s.burst_active(10.9) and not s.burst_active(11.0)


def test_hold_debounces_then_clears():
    s = GuardState(hold_s=0.5)
    s.update('left', True, 0.0)
    assert s.update('left', False, 0.4) == (True, False)  # within hold
    assert s.tick(0.6) == (False, False)                   # expired
    assert s.update('left', True, 0.7) == (True, True)     # new edge


def test_other_camera_does_not_clear():
    s = GuardState(hold_s=0.5)
    s.update('left', True, 0.0)
    assert s.update('right', False, 0.1)[0] is True


def test_marker_geometry():
    pts = box_outline(box(x1=0, y1=0, x2=10, y2=20))
    assert len(pts) == 8 and pts[0] == pts[-1] == (0, 0)
    assert danger_line(CFG) == [(0.0, 648.0), (1920.0, 648.0)]


def test_classes_match_det_ros():
    yaml = pytest.importorskip('yaml')
    det_yaml = os.path.join(os.path.dirname(__file__), '..', '..', 'det_ros', 'config',
                            'det.yaml')
    if not os.path.isfile(det_yaml):
        pytest.skip('det_ros source not next to mower_vision')
    with open(det_yaml) as fh:
        assert yaml.safe_load(fh)['det_ros']['ros__parameters']['classes'] == DEFAULT_CLASSES


def test_config_yaml_whitelist_known():
    yaml = pytest.importorskip('yaml')
    with open(os.path.join(os.path.dirname(__file__), '..', 'config',
                           'obstacle_guard.yaml')) as fh:
        p = yaml.safe_load(fh)['obstacle_guard']['ros__parameters']
    assert set(p['whitelist']) <= set(DEFAULT_CLASSES)
    assert p['stop_on_close'] is False


def _scaled(b, k):
    return Box(b.label, b.score, b.cx * k, b.cy * k, b.w * k, b.h * k)


@pytest.mark.parametrize('x1,y1,x2,y2', [
    (800, 500, 1200, 1000), (800, 500, 1000, 1000), (800, 500, 1200, 640),
    (800, 500, 1200, 650), (0, 0, 288, 700), (0, 0, 290, 700), (100, 100, 400, 300)])
def test_same_decision_at_1920_and_960(x1, y1, x2, y2):
    full = with_image_size(GuardConfig(), 1920, 1080)
    half = with_image_size(GuardConfig(), 960, 540)
    b = box(x1=x1, y1=y1, x2=x2, y2=y2)
    assert in_danger_zone(b, full) == in_danger_zone(_scaled(b, 0.5), half)


def test_with_image_size_fallback_and_marker_line():
    base = GuardConfig(image_width=960, image_height=540)
    assert with_image_size(base, 0, 0) is base
    assert with_image_size(base, 1920, 1080).image_height == 1080
    assert danger_line(with_image_size(base, 960, 540)) == [(0.0, 324.0), (960.0, 324.0)]
