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


def test_measured_range_overrides_image_rule():
    from dataclasses import replace
    far_but_big = replace(box(), range_m=2.5)          # image rule says close
    near_but_small = replace(box(y1=100, y2=300), range_m=0.8)   # image rule says far
    assert in_danger_zone(far_but_big, CFG) and not any_close([far_but_big], CFG)
    assert not in_danger_zone(near_but_small, CFG) and any_close([near_but_small], CFG)
    assert any_close([replace(box(), range_m=1.0)], CFG)        # boundary inclusive
    assert not any_close([replace(box(), range_m=1.2)], CFG)
    assert any_close([replace(box(), range_m=1.2)], replace(CFG, stop_range_m=1.5))
    assert not any_close([replace(box('stone'), range_m=0.3)], CFG)   # still whitelisted only
    assert any_close([box()], CFG)                      # no range -> image rule fallback


def test_range_of_ranged_detection():
    pytest.importorskip('rclpy')
    pytest.importorskip('vision_msgs')
    from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
    from mower_vision.obstacle_guard_node import range_of
    det = Detection2D()
    assert range_of(det) is None
    det.results.append(ObjectHypothesisWithPose())
    assert range_of(det) is None                        # covariance 0 = unranged
    det.results[0].pose.pose.position.x = 0.466 + 0.6
    det.results[0].pose.pose.position.y = 0.8
    cov = [0.0] * 36
    cov[0] = 0.01
    det.results[0].pose.covariance = cov
    assert range_of(det, 0.466) == pytest.approx(1.0)


def test_frame_policy_dynamic_wins_and_static_by_range():
    from mower_vision.guard_logic import Box, GuardConfig, PolicyConfig, frame_policy
    g, pc = GuardConfig(), PolicyConfig()
    chair = Box('chair', 0.8, 100, 100, 10, 10, range_m=1.2, bearing_deg=5.0)
    dog = Box('dog', 0.8, 100, 100, 10, 10, range_m=0.9)
    far_dog = Box('dog', 0.8, 100, 100, 10, 10, range_m=1.2)
    assert frame_policy([chair], pc, g)['kind'] == 'static'
    assert frame_policy([chair], pc, g)['bearing_deg'] == 5.0
    assert frame_policy([chair, dog], pc, g) == {'kind': 'dynamic', 'class': 'dog',
                                                 'distance_m': 0.9, 'bearing_deg': None}
    assert frame_policy([far_dog], pc, g)['kind'] == 'none'
    assert frame_policy([Box('chair', 0.1, 1, 1, 1, 1, range_m=0.5)], pc, g)['kind'] == 'none'


def test_policy_state_holds_then_clears():
    from mower_vision.guard_logic import PolicyState
    st = PolicyState(hold_s=0.5)
    st.update('l', {'kind': 'static', 'class': 'chair', 'distance_m': 1.0,
                    'bearing_deg': None}, 0.0)
    st.update('r', {'kind': 'none', 'class': '', 'distance_m': None, 'bearing_deg': None}, 0.1)
    assert st.current(0.4)['class'] == 'chair'
    assert st.current(0.6)['kind'] == 'none'


# ---------------------------------------------------------------------------
# motion relevance + obstacle_detection levels
# ---------------------------------------------------------------------------
from mower_vision.guard_logic import (CAM_FRONT, CAM_LEFT, CAM_REAR, CAM_RIGHT,  # noqa: E402
                                      DEFAULT_CAMERA_FRAMES, LevelTable, MotionState,
                                      PolicyConfig, camera_role, frame_policy, level_policy)

G540 = GuardConfig(image_width=960, image_height=540)
TABLE = LevelTable()


def pol(level):
    return level_policy(level, PolicyConfig(), TABLE)


def unranged_person(h=300, score=0.8):
    return Box('person', score, 480, 300, 120, h)


def test_camera_roles():
    f = DEFAULT_CAMERA_FRAMES
    assert camera_role('vio_camera', f) == CAM_FRONT
    assert camera_role('left_oa_camera', f) == CAM_LEFT
    assert camera_role('right_oa_camera', f) == CAM_RIGHT
    assert camera_role('rear_camera', f) == CAM_REAR
    assert camera_role('mystery', f) == CAM_FRONT


def test_motion_relevance():
    m = MotionState()
    assert m.relevant(0.0) == {CAM_FRONT}                 # no command: front only
    m.update(0.3, 0.0, 1.0)
    assert m.relevant(1.1) == {CAM_FRONT}
    m.update(0.3, -0.5, 2.0)                              # turning right
    assert m.relevant(2.1) == {CAM_FRONT, CAM_RIGHT}
    m.update(0.3, 0.05, 2.2)                              # inside the deadband
    assert CAM_RIGHT in m.relevant(2.5)                   # held
    assert CAM_RIGHT not in m.relevant(2.7)               # hold (0.6 s) expired
    m.update(-0.2, 0.0, 5.0)                              # reversing only
    assert m.relevant(5.1) == {CAM_REAR}


def test_live_bug_right_camera_unranged_person_ignored_while_driving_straight():
    m = MotionState()
    m.update(0.3, 0.0, 0.0)
    p = frame_policy([unranged_person(h=500)], pol('standard'), G540,
                     CAM_RIGHT, m.relevant(0.1))
    assert p['kind'] == 'none'


def test_standard_side_needs_turn_and_tall_box():
    m = MotionState()
    m.update(0.2, -0.4, 0.0)                              # turning toward the right camera
    rel = m.relevant(0.1)
    assert frame_policy([unranged_person(h=300)], pol('standard'), G540,
                        CAM_RIGHT, rel)['kind'] == 'dynamic'          # 300 >= 0.45*540
    assert frame_policy([unranged_person(h=200)], pol('standard'), G540,
                        CAM_RIGHT, rel)['kind'] == 'none'             # small = far
    assert frame_policy([unranged_person(h=300)], pol('standard'), G540,
                        CAM_LEFT, rel)['kind'] == 'none'              # other side


def test_standard_front_ranged_stop_range_and_reverse():
    near = Box('person', 0.8, 100, 100, 10, 10, range_m=0.9)
    mid = Box('person', 0.8, 100, 100, 10, 10, range_m=1.3)
    m = MotionState()
    m.update(0.3, 0.0, 0.0)
    assert frame_policy([near], pol('standard'), G540, CAM_FRONT, m.relevant(0.1))['kind'] \
        == 'dynamic'
    assert frame_policy([mid], pol('standard'), G540, CAM_FRONT, m.relevant(0.1))['kind'] \
        == 'none'
    m.update(-0.2, 0.0, 1.0)
    assert frame_policy([near], pol('standard'), G540, CAM_FRONT, m.relevant(1.1))['kind'] \
        == 'none'                                         # reversing: front irrelevant
    rock = Box('stone', 0.8, 100, 100, 10, 10, range_m=1.3)
    assert frame_policy([rock], pol('standard'), G540, CAM_FRONT, {CAM_FRONT})['kind'] \
        == 'static'


def test_level_none_never_reports():
    near = Box('person', 0.99, 100, 100, 10, 10, range_m=0.3)
    assert frame_policy([near], pol('none'), G540, CAM_FRONT, {CAM_FRONT})['kind'] == 'none'
    assert frame_policy([near], pol('none'), G540, CAM_FRONT, None)['kind'] == 'none'


def test_level_sensitive_range_score_and_any_camera():
    s = pol('sensitive')
    assert (s.dynamic_range_m, s.min_score, s.any_camera) == (1.5, 0.35, True)
    mid = Box('person', 0.37, 100, 100, 10, 10, range_m=1.3)
    assert frame_policy([mid], s, G540, CAM_FRONT, {CAM_FRONT})['kind'] == 'dynamic'
    assert frame_policy([mid], pol('standard'), G540, CAM_FRONT, {CAM_FRONT})['kind'] == 'none'
    # unranged from a side camera counts even driving straight
    assert frame_policy([unranged_person(h=300)], s, G540, CAM_RIGHT, {CAM_FRONT})['kind'] \
        == 'dynamic'


def test_unknown_level_is_standard():
    assert pol('bogus').level == 'standard'
    assert pol('standard').dynamic_range_m == 1.0 and pol('standard').min_score == 0.4
