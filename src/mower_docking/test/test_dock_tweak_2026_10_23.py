"""2026-10-23 dock tweak (owner: less forward driving while getting ready to align).
Field mow 2026-10-23 midday, attempt 1 (docs/wheel_smoothness_field_tests.md): ALIGNING cut
short 42 deg off by a marker sighting, LATERAL_ALIGN drove 0.43 m forward to the 1.05 m cap
for nothing (lateral -0.21 -> -0.23 m) and lost the marker, SEARCHING 0/96 frames, then a
forced Nav2 re-approach (1.72 m forward, 0.46 m back)."""
import math
import os

import pytest
import yaml

from mower_docking import dock_logic as dl
from test_dock_stock_port import World, both_wheels_ok, params

S = dl.DockState
CAM_HALF_FOV = math.radians(35.0)


class FovWorld(World):
    """Marker only visible within the rear camera's field of view."""

    def marker(self):
        mk = super().marker()
        if mk is None or abs(math.atan2(mk.y, -mk.x)) > CAM_HALF_FOV:
            return None
        return mk


def forward_travel(outs, dt=0.05):
    return sum(o.linear * dt for o in outs if o.linear > 0)


def test_new_defaults_and_yaml_agree():
    p = dl.DockParams()
    assert p.lateral_align_tolerance == 0.25 and p.lateral_align_max_heading_deg == 15.0
    assert p.lateral_align_skip_if_marker and p.inplace_search_retry
    assert p.forward_cap_margin == 0.10 and p.realign_forward_distance == 0.40
    assert p.approach_distance == 0.8 and p.approach_skip_radius_m == 0.6   # kept
    assert p.gains.wheel_floor == 0.06                                       # kept
    cfg = os.path.join(os.path.dirname(__file__), '..', 'config', 'docking.yaml')
    with open(cfg) as f:
        y = next(iter(yaml.safe_load(f).values()))['ros__parameters']
    for n in ('lateral_align_tolerance', 'lateral_align_max_heading_deg', 'forward_cap_margin',
              'realign_forward_distance', 'approach_distance', 'approach_skip_radius_m',
              'realign_forward_per_lateral', 'realign_forward_max'):
        assert y[n] == pytest.approx(getattr(p, n)), n
    for n in ('lateral_align_skip_if_marker', 'inplace_search_retry', 'realign_steer',
              'align_to_marker_bearing'):
        assert y[n] is getattr(p, n), n


def _states(x, y, yaw, visible, seconds=1.0):
    m = dl.DockStateMachine(params(), dl.Pose2D(), dock_pose_measured=True)
    w = World(m, x, y, yaw)
    w.visible = visible
    w.run(seconds)
    return [o.state for o in w.outs], m, w


def test_lateral_align_only_without_marker_and_near_the_dock_heading():
    # 0.30 m off, heading on the dock yaw, no marker: pre-align (inside the approach
    # distance, so there is forward room under the 0.90 m cap)
    assert S.LATERAL_ALIGN in _states(0.6, 0.30, 0.0, visible=False)[0]
    # marker in view: straight to SEARCHING/DOCKING, the KF + controller fix the lateral
    st, m, _ = _states(0.6, 0.30, 0.0, visible=True)
    assert S.LATERAL_ALIGN not in st and S.DOCKING in st
    assert any('marker is in view' in n for o in _states(0.6, 0.3, 0.0, True)[2].outs
               for n in o.notes)
    # 20 deg off the dock yaw (> 15, < the 25 deg ALIGNING threshold): no pre-alignment
    assert S.LATERAL_ALIGN not in _states(0.6, 0.30, math.radians(20), visible=False)[0]
    assert S.LATERAL_ALIGN in _states(0.6, 0.30, math.radians(12), visible=False)[0]
    # within the new 0.25 m tolerance (was 0.15): no pre-alignment
    assert S.LATERAL_ALIGN not in _states(0.6, 0.22, 0.0, visible=False)[0]


def test_forward_cap_is_090m():
    m = dl.DockStateMachine(params(), dl.Pose2D(), dock_pose_measured=True)
    assert m._forward_cap() == pytest.approx(0.90)
    w = World(m, 0.6, 0.30, 0.0)
    w.visible = False
    xs = []
    w.run(60, until=lambda o: (xs.append(w.x), o.state == S.SEARCHING)[1])
    assert S.LATERAL_ALIGN in [o.state for o in w.outs]
    assert max(xs) <= 0.90 + 0.02
    assert both_wheels_ok([o for o in w.outs if o.state == S.LATERAL_ALIGN])


def test_realign_distance_040_longer_only_when_badly_off():
    m = dl.DockStateMachine(params(), dl.Pose2D())
    snap = dl.Snapshot(t=1.0, odom=dl.Pose2D(0.2, 0.1, 0.0))
    m._enter_realign(snap, dl.DockErrors(0.2, 0.10, 0.0))
    assert m._retry_distance == pytest.approx(0.40)
    m._enter_realign(snap, dl.DockErrors(0.2, 0.36, 0.0))
    assert m._retry_distance == pytest.approx(0.54)
    m._enter_realign(snap, dl.DockErrors(0.6, 0.36, 0.0))     # corridor cap: 0.90 - 0.60
    assert m._retry_distance == pytest.approx(0.30)
    m._enter_realign(snap, dl.DockErrors(0.1, 0.60, 0.0))     # at most realign_forward_max
    assert m._retry_distance == pytest.approx(0.65)


def test_realign_forward_arcs_back_toward_the_axis():
    m = dl.DockStateMachine(params(), dl.Pose2D())
    m.start(dl.Snapshot(t=0.0, odom=dl.Pose2D(0.2, 0.2, 0.0)))
    m._enter_realign(dl.Snapshot(t=0.0, odom=dl.Pose2D(0.2, 0.2, 0.0)),
                     dl.DockErrors(0.2, 0.2, 0.0))
    out = m.step(dl.Snapshot(t=0.05, odom=dl.Pose2D(0.2, 0.2, 0.0)))
    assert out.state == S.RETRY and out.linear > 0 and out.angular < 0   # toward y = 0
    assert both_wheels_ok([out])
    off = dl.DockStateMachine(params(realign_steer=False), dl.Pose2D())
    off.start(dl.Snapshot(t=0.0, odom=dl.Pose2D(0.2, 0.2, 0.0)))
    off._enter_realign(dl.Snapshot(t=0.0, odom=dl.Pose2D(0.2, 0.2, 0.0)),
                       dl.DockErrors(0.2, 0.2, 0.0))
    assert off.step(dl.Snapshot(t=0.05, odom=dl.Pose2D(0.2, 0.2, 0.0))).angular == 0.0


def test_search_timeout_near_the_approach_pose_aligns_in_place_then_offset_renav():
    m = dl.DockStateMachine(params(skip_nav_to_approach=False), dl.Pose2D(),
                            dock_pose_measured=True)
    w = World(m, 0.75, 0.05, math.radians(40))      # arrives 40 deg off, never sees the marker
    w.visible = False
    navs = []
    w.run(200, until=lambda o: (navs.extend(r for r in o.requests if r[0] == 'start_nav'),
                                bool(navs))[1])
    notes = [n for o in w.outs for n in o.notes]
    st = [o.state for o in w.outs]
    # first failed search: in-place align + search again, no Nav2 leg, no retry used
    i = next(k for k, n in enumerate(notes) if 'aligning in place' in n)
    assert m.retries == 1 and navs, 'second failed search: offset re-approach with Nav2'
    assert any('retry 1/' in n and 'offset +0.10' in n for n in notes[i:])
    assert st.count(S.ALIGNING) > 0
    assert forward_travel(w.outs) < 0.05


def test_field_2026_10_23_attempt1_replay():
    """At the approach pose facing the dock (Nav2 arrival), 0.21 m off the axis; the rear
    marker comes into view ~40 deg into the about-turn. Must not LATERAL_ALIGN, must finish
    the about-turn, and dock without the 0.43 m forward drive."""
    # the pre-DOCKING forward driving (near-field realigns: test_dock_lateral_2026_10_23)
    m = dl.DockStateMachine(params(skip_nav_to_approach=False, near_check_remaining=0.0),
                            dl.Pose2D(), dock_pose_measured=True, goal_timeout_s=400.0)
    w = FovWorld(m, 0.62, -0.21, math.pi - 0.05)
    out = w.run(300)
    st = [o.state for o in w.outs]
    notes = [n for o in w.outs for n in o.notes]
    assert out.success, (out.message, notes)
    assert S.LATERAL_ALIGN not in st
    assert not any(r[0] == 'start_nav' for o in w.outs for r in o.requests)
    # the about-turn ran to the tolerance, not cut short by the first sighting
    last_align = max(k for k, s in enumerate(st) if s == S.ALIGNING)
    assert st[last_align + 1] == S.SEARCHING
    seen = [k for k, o in enumerate(w.outs) if o.marker_in_view]
    assert seen and seen[0] <= last_align
    first_dock = st.index(S.DOCKING)
    assert forward_travel(w.outs[:first_dock]) == 0.0      # was 0.43 m (LATERAL_ALIGN)
    # at most one 0.40 m pose-check realign afterwards (was up to 0.65 m)
    assert forward_travel(w.outs) <= 0.40 + 0.01 and m.retries == 0
    assert both_wheels_ok(w.outs)


def test_aligning_aims_the_rear_camera_at_the_marker():
    """Sim 2026-10-23 off25: 0.25 m off the axis at the approach pose, aligned to the dock
    yaw, the marker sat outside the +-25 deg rear camera: 0 detections in 15 s, twice."""
    m = dl.DockStateMachine(params(skip_nav_to_approach=False), dl.Pose2D(),
                            dock_pose_measured=True, goal_timeout_s=400.0)
    snap = dl.Snapshot(t=0.0, map_pose=dl.Pose2D(0.8, 0.25, math.pi))
    want = math.atan2(-0.25, -0.8 - 0.45) + math.pi
    assert m._align_yaw(snap) == pytest.approx(dl.wrap_angle(want))      # ~ +11 deg
    assert dl.DockStateMachine(params(align_to_marker_bearing=False), dl.Pose2D(),
                               dock_pose_measured=True)._align_yaw(snap) == 0.0
    global CAM_HALF_FOV
    old, CAM_HALF_FOV = CAM_HALF_FOV, math.radians(25.0)
    try:
        w = FovWorld(m, 0.8, 0.25, math.pi)
        out = w.run(300)
    finally:
        CAM_HALF_FOV = old
    notes = [n for o in w.outs for n in o.notes]
    assert out.success and m.retries == 0, notes
    assert not any('no usable marker' in n for n in notes)
