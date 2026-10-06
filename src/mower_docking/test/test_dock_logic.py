# Copyright 2026 The mower_docking authors
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for mower_docking.dock_logic (no ROS, no cv2)."""

import math

import pytest

from mower_docking import dock_logic as dl
from mower_docking.dock_logic import DockState as S

CAM_XYZ = (-0.201, 0.0, 0.25)
CAM_RPY = (-math.pi / 2, 0.0, math.pi / 2)


# ----------------------------------------------------------------- geometry
def test_wrap_angle():
    assert dl.wrap_angle(0.0) == pytest.approx(0.0)
    assert dl.wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert dl.wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_yaw_quaternion_roundtrip():
    for yaw in (-2.5, -0.3, 0.0, 1.0, 3.0):
        assert dl.yaw_from_quaternion(*dl.quaternion_from_yaw(yaw)) == pytest.approx(yaw)


def test_approach_pose_vendor_default():
    a = dl.approach_pose(dl.Pose2D(0, 0, 0), 0.8)
    assert (a.x, a.y, a.yaw) == pytest.approx((0.8, 0.0, 0.0))


def test_approach_pose_rotated_dock():
    a = dl.approach_pose(dl.Pose2D(1.0, 2.0, math.pi / 2), 0.8)
    assert (a.x, a.y, a.yaw) == pytest.approx((1.0, 2.8, math.pi / 2))


def test_rpy_matrix_rear_camera_optical_axis_points_backwards():
    r = dl.rpy_to_matrix(*CAM_RPY)
    z = [r[0][2], r[1][2], r[2][2]]  # optical +Z in base_link
    assert z == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9)
    x = [r[0][0], r[1][0], r[2][0]]  # image right -> robot +Y (left) when looking back
    assert x == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)


def test_rodrigues_identity_and_quarter_turn():
    assert dl.rodrigues([0, 0, 0]) == [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    r = dl.rodrigues([0, 0, math.pi / 2])
    assert r[0][1] == pytest.approx(-1.0) and r[1][0] == pytest.approx(1.0)


def _marker_facing_camera_rvec():
    # marker +Z points back at the camera: rotate pi about the optical X axis
    return [math.pi, 0.0, 0.0]


def test_marker_in_base_straight_behind():
    obs = dl.marker_in_base(_marker_facing_camera_rvec(), [0.0, 0.0, 0.5], CAM_XYZ, CAM_RPY)
    assert obs.x == pytest.approx(-0.701)
    assert obs.y == pytest.approx(0.0, abs=1e-9)
    assert obs.yaw == pytest.approx(0.0, abs=1e-9)


def test_marker_in_base_lateral_offset_sign():
    # optical +X (image right) is robot +Y (left) for the rear camera
    obs = dl.marker_in_base(_marker_facing_camera_rvec(), [0.1, 0.0, 0.5], CAM_XYZ, CAM_RPY)
    assert obs.y == pytest.approx(0.1)


def test_marker_in_base_yaw():
    # marker rotated about the (vertical) optical Y axis by +0.2 rad
    r = dl._matmul(dl.rodrigues([0.0, 0.2, 0.0]), dl.rodrigues([math.pi, 0, 0]))
    # back to rvec via axis-angle of r (small helper)
    ang = math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2)))
    ax = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]]
    n = math.sqrt(sum(a * a for a in ax))
    if n < 1e-9:  # angle == pi, recover axis from the diagonal
        ax = [math.sqrt(max(0.0, (r[i][i] + 1) / 2)) for i in range(3)]
        if r[0][2] < 0:
            ax[2] = -ax[2]
        rvec = [a * ang for a in ax]
    else:
        rvec = [a / n * ang for a in ax]
    obs = dl.marker_in_base(rvec, [0.0, 0.0, 0.5], CAM_XYZ, CAM_RPY)
    assert abs(obs.yaw) == pytest.approx(0.2, abs=1e-6)


def test_dock_errors_aligned():
    e = dl.dock_errors(dl.MarkerObs(-1.2, 0.0, 0.0), 0.45)
    assert e.remaining == pytest.approx(0.75)
    assert e.lateral == pytest.approx(0.0)
    assert e.heading == pytest.approx(0.0)


def test_dock_errors_lateral_and_heading():
    e = dl.dock_errors(dl.MarkerObs(-1.0, -0.2, 0.0), 0.45)
    assert e.lateral == pytest.approx(0.2)  # robot is LEFT of the dock axis
    e2 = dl.dock_errors(dl.MarkerObs(-1.0, 0.0, 0.1), 0.45)
    assert e2.heading == pytest.approx(-0.1)


def test_marker_gate():
    lim = math.radians(25)
    assert dl.marker_gate_ok(dl.DockErrors(1.0, 0.4, 0.3), 0.5, lim)
    assert not dl.marker_gate_ok(dl.DockErrors(1.0, 0.6, 0.0), 0.5, lim)
    assert not dl.marker_gate_ok(dl.DockErrors(1.0, 0.0, math.radians(30)), 0.5, lim)


# --------------------------------------------------------------- controller
def test_reverse_speed_profile():
    g = dl.ControllerGains()
    assert dl.reverse_speed(2.0, g) == pytest.approx(0.15)
    assert dl.reverse_speed(0.3, g) == pytest.approx(0.08)    # close zone, steering on
    assert dl.reverse_speed(0.3, g, near=0.05) == pytest.approx(0.05)   # blind straight
    assert 0.08 < dl.reverse_speed(0.45, g) < 0.15


def test_reverse_control_is_reverse_and_bounded():
    g = dl.ControllerGains()
    v, w, _ = dl.reverse_control(dl.DockErrors(1.0, 0.4, -0.4), g)
    assert v < 0 and abs(v) <= 0.15
    assert abs(w) <= g.max_angular


def test_reverse_control_converges_in_simulation():
    """Closed loop: unicycle reversing with the controller -> lateral & heading -> 0."""
    g = dl.ControllerGains()
    # robot pose in dock frame (dock at origin, docked heading 0, robot ahead at x>0)
    x, y, th = 1.5, 0.25, 0.15
    dt = 0.05
    prev = None
    for _ in range(2000):
        err = dl.DockErrors(remaining=x, lateral=y, heading=th)
        if err.remaining <= g.final_zone:
            break
        v, w, prev = dl.reverse_control(err, g, prev, dt)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
    assert x <= g.final_zone + 0.01
    assert abs(y) < 0.03
    assert abs(th) < 0.06


def test_heading_hold():
    assert dl.heading_hold(0.0, 0.1, 1.0, 0.3) == pytest.approx(-0.1)
    assert dl.heading_hold(0.0, None, 1.0, 0.3) == 0.0
    assert dl.heading_hold(0.0, -2.0, 1.0, 0.3) == pytest.approx(0.3)


# ------------------------------------------------------ debounce / distance
def test_contact_debounce_three_samples():
    d = dl.ContactDebouncer(3)
    assert not d.update(True)
    assert not d.update(True)
    assert d.update(True)


def test_contact_debounce_resets_on_false():
    d = dl.ContactDebouncer(3)
    d.update(True)
    d.update(True)
    assert not d.update(False)
    d.update(True)
    assert not d.value


def test_distance_tracker_odom():
    t = dl.DistanceTracker()
    t.start(0.0, dl.Pose2D(0, 0, 0))
    t.update(0.1, dl.Pose2D(0.03, 0.04, 0), 0.1)
    assert t.distance == pytest.approx(0.05)
    assert not t.used_fallback


def test_distance_tracker_time_fallback():
    t = dl.DistanceTracker()
    t.start(0.0, None)
    t.update(1.0, None, -0.15)
    t.update(2.0, None, -0.15)
    assert t.distance == pytest.approx(0.3)
    assert t.used_fallback


# ---------------------------------------------------------- dock FSM helpers
class Sim:
    """Tiny 1-D/2-D world: integrates the commanded twist into odom and
    raises contact once the robot has reversed `contact_after` metres."""

    def __init__(self, machine, contact_after=None, x0=0.8, odom=True):
        self.m = machine
        self.t = 0.0
        self.x, self.y, self.yaw = x0, 0.0, 0.0
        self.contact_after = contact_after
        self.reversed = 0.0
        self.deb = dl.ContactDebouncer(3)
        self.v = self.w = 0.0
        self.odom = odom
        self.nav_status = None
        self.charging = None
        self.marker = None
        self.stop = False
        self.status_fresh = True
        self.states = []
        self.requests = []

    def snap(self):
        return dl.Snapshot(t=self.t,
                           odom=dl.Pose2D(self.x, self.y, self.yaw) if self.odom else None,
                           marker=self.marker, contact=self.deb.value,
                           status_fresh=self.status_fresh, stop_triggered=self.stop,
                           nav_status=self.nav_status, charging_result=self.charging)

    def _apply(self, out):
        self.v, self.w = out.linear, out.angular
        self.states.append(out.state)
        self.requests.extend(out.requests)
        for r in out.requests:
            if r[0] == 'enable_charging':
                self.charging = True
        return out

    def start(self):
        return self._apply(self.m.start(self.snap()))

    def tick(self, dt=0.05):
        self.t += dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt
        if self.v < 0:
            self.reversed += -self.v * dt
        elif self.v > 0:
            self.reversed = max(0.0, self.reversed - self.v * dt)
        self.deb.update(self.contact_after is not None and self.reversed >= self.contact_after)
        return self._apply(self.m.step(self.snap()))

    def run(self, n=5000):
        out = None
        for _ in range(n):
            out = self.tick()
            if out.done:
                break
        return out


def blind_params(**kw):
    p = dl.DockParams(use_vision=False, skip_nav_to_approach=True, allow_blind_docking=True)
    for k, v in kw.items():
        setattr(p, k, v)
    return p


# ---------------------------------------------------------------- dock FSM
def test_dock_starts_with_nav_to_approach_request():
    m = dl.DockStateMachine(dl.DockParams(), dl.Pose2D(0, 0, 0))
    out = m.start(dl.Snapshot(t=0.0))
    assert out.state == S.NAV_TO_APPROACH
    assert out.requests[0][0] == 'start_nav'
    assert out.requests[0][1] == dl.Pose2D(0.8, 0.0, math.pi)   # facing the dock
    assert out.linear == 0.0


def test_dock_nav_failure():
    m = dl.DockStateMachine(dl.DockParams(), dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    out = m.step(dl.Snapshot(t=1.0, nav_status='failed'))
    assert out.done and not out.success and out.message == 'NAV_TO_DOCK_FAILED'


def test_dock_nav_timeout_cancels_nav():
    p = dl.DockParams(nav_timeout_s=5.0)
    m = dl.DockStateMachine(p, dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    out = m.step(dl.Snapshot(t=6.0, nav_status='running'))
    assert out.message == 'NAV_TO_DOCK_FAILED'
    assert ('cancel_nav',) in out.requests


def test_dock_blind_success_reverses_and_enables_charging():
    sim = Sim(dl.DockStateMachine(blind_params(), dl.Pose2D()), contact_after=0.8)
    sim.start()
    assert sim.states[-1] == S.FINAL_DOCKING
    out = sim.run()
    assert out.success and out.message == 'DOCKED'
    assert ('enable_charging',) in sim.requests
    assert S.CHARGING in sim.states
    assert sim.reversed == pytest.approx(0.8, abs=0.03)
    assert all(v == 0.0 for v in (out.linear, out.angular))


def test_dock_blind_never_exceeds_speed_limits():
    sim = Sim(dl.DockStateMachine(blind_params(), dl.Pose2D()), contact_after=0.8)
    sim.start()
    trace = []
    for _ in range(400):
        before = sim.reversed
        out = sim.tick()
        trace.append((before, out.linear))
        if out.done:
            break
    assert min(v for _, v in trace) >= -0.15 - 1e-9
    assert max(v for _, v in trace) <= 1e-9  # no forward motion when contact comes on time
    assert min(v for _, v in trace) == pytest.approx(-0.15)
    # final 0.3 m before the expected contact at FinalDock speed (vendor 0.05 m/s)
    assert all(abs(v) <= 0.05 + 1e-9 for r, v in trace if r >= 0.5 + 0.01)


def test_dock_vision_off_after_nav_goes_blind():
    m = dl.DockStateMachine(dl.DockParams(use_vision=False, allow_blind_docking=True),
                            dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    out = m.step(dl.Snapshot(t=1.0, nav_status='succeeded', odom=dl.Pose2D(0.8, 0, 0)))
    assert out.state == S.FINAL_DOCKING


def test_dock_search_timeout_falls_back_to_blind():
    p = dl.DockParams(skip_nav_to_approach=True, search_timeout_s=2.0, allow_blind_docking=True)
    sim = Sim(dl.DockStateMachine(p, dl.Pose2D()))
    sim.start()
    assert sim.states[-1] == S.SEARCHING
    for _ in range(30):
        sim.tick()
    assert sim.states[-1] == S.SEARCHING and sim.v == 0.0
    for _ in range(20):
        sim.tick()
    assert sim.states[-1] == S.FINAL_DOCKING


def test_dock_search_rejects_bad_marker():
    p = dl.DockParams(skip_nav_to_approach=True)
    m = dl.DockStateMachine(p, dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    bad = dl.MarkerObs(-1.2, 0.0, math.radians(40), stamp=0.1)
    out = m.step(dl.Snapshot(t=0.1, marker=bad))
    assert out.state == S.SEARCHING
    good = dl.MarkerObs(-1.2, 0.05, math.radians(5), stamp=0.2)
    out = m.step(dl.Snapshot(t=0.2, marker=good))
    assert out.state == S.DOCKING


def test_dock_vision_docking_commands_reverse_then_final():
    p = dl.DockParams(skip_nav_to_approach=True)
    m = dl.DockStateMachine(p, dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    m.step(dl.Snapshot(t=0.1, marker=dl.MarkerObs(-1.3, 0.1, 0.05, stamp=0.1)))
    out = m.step(dl.Snapshot(t=0.15, marker=dl.MarkerObs(-1.3, 0.1, 0.05, stamp=0.15)))
    assert out.state == S.DOCKING and out.linear < 0
    assert out.remaining == pytest.approx(0.85, abs=0.02)
    for i in range(3):   # median of the last 3 observations
        out = m.step(dl.Snapshot(t=0.2 + 0.05 * i,
                                 marker=dl.MarkerObs(-0.72, 0.0, 0.0, stamp=0.2 + 0.05 * i)))
    assert out.state == S.DOCKING and out.linear == pytest.approx(-0.08)  # close zone max
    # inside final_zone the controller keeps steering (no straight freeze) ...
    for i in range(3):
        out = m.step(dl.Snapshot(t=0.4 + 0.05 * i,
                                 marker=dl.MarkerObs(-0.6, 0.03, 0.03, stamp=0.4 + 0.05 * i)))
    assert out.state == S.DOCKING and out.linear < 0 and out.angular != 0.0
    assert out.detail.startswith('docking: 0.1')
    # ... until the marker is lost: straight on the last docked heading
    out = m.step(dl.Snapshot(t=0.6))
    assert out.state == S.FINAL_DOCKING and out.linear == pytest.approx(-0.05)
    assert 'marker lost' in out.detail


def test_dock_lost_frames_trigger_retry():
    p = dl.DockParams(skip_nav_to_approach=True, max_lost_frames=5)
    m = dl.DockStateMachine(p, dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    m.step(dl.Snapshot(t=0.1, marker=dl.MarkerObs(-1.3, 0.0, 0.0, stamp=0.1)))
    states = []
    for i in range(30):
        out = m.step(dl.Snapshot(t=1.0 + 0.05 * i, odom=dl.Pose2D()))  # stale marker
        states.append((out.state, out.linear))
    assert all(v >= 0.0 for _, v in states)  # never reverses blindly while lost
    assert S.RETRY in [s for s, _ in states]
    assert m.retries == 1


def test_dock_final_timeout_retries_then_maxout():
    # contact never arrives -> FINAL_DOCKING overruns -> RETRY x3 -> DOCK_MAXOUT
    sim = Sim(dl.DockStateMachine(blind_params(max_retries=3), dl.Pose2D()), contact_after=None)
    sim.start()
    out = sim.run(n=20000)
    assert out.done and not out.success
    assert out.message == 'DOCK_MAXOUT'
    assert out.retries == 3
    assert sim.states.count(S.RETRY) > 0
    assert any(s == S.RETRY for s in sim.states)


def test_dock_retry_drives_forward():
    sim = Sim(dl.DockStateMachine(blind_params(max_retries=1), dl.Pose2D()), contact_after=None)
    sim.start()
    fwd = False
    for _ in range(5000):
        out = sim.tick()
        if out.state == S.RETRY and out.linear > 0:
            fwd = True
            assert out.linear == pytest.approx(0.1)
        if out.done:
            break
    assert fwd


def test_dock_retry_with_nav_reissues_approach():
    p = dl.DockParams(use_vision=False, final_timeout_s=1.0, blind_timeout_margin_s=0.0,
                      approach_distance=0.1, blind_extra_distance=0.05,
                      allow_blind_docking=True)
    m = dl.DockStateMachine(p, dl.Pose2D())
    sim = Sim(m, contact_after=None)
    sim.start()
    sim.nav_status = 'succeeded'
    navs = 0
    for _ in range(3000):
        out = sim.tick()
        navs += sum(1 for r in out.requests if r[0] == 'start_nav')
        if out.done:
            break
    assert navs >= 1  # re-approach via Nav2 after the first RETRY


def test_dock_emergency_stop_aborts_with_zero_cmd():
    sim = Sim(dl.DockStateMachine(blind_params(), dl.Pose2D()), contact_after=0.8)
    sim.start()
    for _ in range(10):
        sim.tick()
    assert sim.v < 0
    sim.stop = True
    out = sim.tick()
    assert out.done and not out.success and out.message == 'EMERGENCY_STOP'
    assert out.linear == 0.0 and out.angular == 0.0


def test_dock_stale_status_aborts_motion():
    sim = Sim(dl.DockStateMachine(blind_params(), dl.Pose2D()), contact_after=0.8)
    sim.start()
    sim.tick()
    sim.status_fresh = False
    out = sim.tick()
    assert out.message == 'BASE_STATUS_STALE' and out.linear == 0.0


def test_dock_already_docked():
    m = dl.DockStateMachine(dl.DockParams(), dl.Pose2D())
    out = m.start(dl.Snapshot(t=0.0, contact=True))
    assert out.state == S.CHARGING and ('enable_charging',) in out.requests
    out = m.step(dl.Snapshot(t=0.1, contact=True, charging_result=True))
    assert out.success


def test_dock_charging_failure():
    m = dl.DockStateMachine(dl.DockParams(), dl.Pose2D())
    m.start(dl.Snapshot(t=0.0, contact=True))
    out = m.step(dl.Snapshot(t=0.1, contact=True, charging_result=False))
    assert out.done and out.message == 'CHARGER_ENABLE_FAILED'


def test_dock_goal_timeout():
    m = dl.DockStateMachine(dl.DockParams(skip_nav_to_approach=True), dl.Pose2D(), 10.0)
    m.start(dl.Snapshot(t=0.0))
    out = m.step(dl.Snapshot(t=11.0))
    assert out.message == 'DOCK_TIMEOUT'


def test_dock_cancel_during_nav():
    m = dl.DockStateMachine(dl.DockParams(), dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    out = m.cancel()
    assert out.done and out.message == 'CANCELED' and ('cancel_nav',) in out.requests


def test_dock_blind_without_odom_uses_time_fallback():
    sim = Sim(dl.DockStateMachine(blind_params(), dl.Pose2D()), contact_after=None, odom=False)
    sim.start()
    for _ in range(4000):
        out = sim.tick()
        if out.state == S.RETRY:
            break
    # the fallback integrates |cmd| * dt and stops at approach + 0.1 m, then (after the
    # contact settle) creeps final_extra_creep_m further before the retry
    assert sim.reversed == pytest.approx(0.9 + 0.15, abs=0.02)


# ----------------------------------------------------------------- undock
def _undock(dist=0.8, speed=0.15, rtk=False, rtk_timeout=5.0):
    return dl.UndockStateMachine(dl.UndockParams(), dist, speed, rtk, rtk_timeout)


def test_undock_full_sequence_forward():
    m = _undock()
    t, x = 0.0, 0.0
    out = m.start(dl.Snapshot(t=t, odom=dl.Pose2D(x, 0, 0)))
    assert out.state == 'CHARGER_OFF' and ('disable_charging',) in out.requests
    seen = set()
    v = 0.0
    for _ in range(1000):
        t += 0.05
        x += v * 0.05
        out = m.step(dl.Snapshot(t=t, odom=dl.Pose2D(x, 0, 0), charging_result=True))
        v = out.linear
        seen.add(out.state)
        assert v >= 0.0 and v <= 0.15 + 1e-9
        if out.done:
            break
    assert out.success and out.message == 'UNDOCKED'
    assert 'BACKING_UP' in seen
    assert x == pytest.approx(0.8, abs=0.02)


def test_undock_charger_off_failure():
    m = _undock()
    m.start(dl.Snapshot(t=0.0))
    out = m.step(dl.Snapshot(t=0.1, charging_result=False))
    assert out.message == 'CHARGER_OFF_FAILED' and out.linear == 0.0


def test_undock_lift_aborts():
    m = _undock()
    m.start(dl.Snapshot(t=0.0, odom=dl.Pose2D()))
    m.step(dl.Snapshot(t=0.1, odom=dl.Pose2D(), charging_result=True))
    out = m.step(dl.Snapshot(t=0.2, odom=dl.Pose2D(), lift_triggered=True))
    assert out.done and out.message == 'EMERGENCY_STOP' and out.linear == 0.0


def test_undock_waits_for_rtk_and_times_out():
    m = _undock(dist=0.1, rtk=True, rtk_timeout=2.0)
    t, x, v = 0.0, 0.0, 0.0
    m.start(dl.Snapshot(t=t, odom=dl.Pose2D()))
    out = None
    for _ in range(200):
        t += 0.05
        x += v * 0.05
        out = m.step(dl.Snapshot(t=t, odom=dl.Pose2D(x, 0, 0), charging_result=True))
        v = out.linear
        if out.state == 'WAITING_FOR_RTK':
            break
    assert out.state == 'WAITING_FOR_RTK' and out.linear == 0.0
    out = m.step(dl.Snapshot(t=t + 3.0, odom=dl.Pose2D(x, 0, 0)))
    assert out.message == 'RTK_TIMEOUT' and not out.success


def test_undock_rtk_fixed_succeeds():
    m = _undock(dist=0.0, rtk=True)
    m.start(dl.Snapshot(t=0.0, odom=dl.Pose2D()))
    m.step(dl.Snapshot(t=0.1, odom=dl.Pose2D(), charging_result=True))
    m.step(dl.Snapshot(t=0.2, odom=dl.Pose2D()))
    out = m.step(dl.Snapshot(t=0.3, odom=dl.Pose2D(), rtk_fixed=True))
    assert out.success


def test_undock_speed_clamped():
    assert _undock(speed=1.0).speed == pytest.approx(0.3)


# ------------------------------------------------- blind gating (incident 2026-10-06)
def _search_until_timeout(m, marker=None, t_end=16.0, odom=dl.Pose2D(0.8, 0, 0)):
    out = None
    t = 0.0
    while t < t_end:
        t += 0.05
        mk = marker(t) if marker else None
        out = m.step(dl.Snapshot(t=t, marker=mk, odom=odom))
        if out.done or out.state != S.SEARCHING:
            break
    return out


def test_no_blind_without_marker_even_when_measured():
    m = dl.DockStateMachine(dl.DockParams(skip_nav_to_approach=True), dl.Pose2D(),
                            dock_pose_measured=True)
    m.start(dl.Snapshot(t=0.0))
    out = _search_until_timeout(m)
    assert out.done and not out.success
    assert out.message.startswith('DOCK_NOT_FOUND') and 'marker not visible' in out.message
    assert out.linear == 0.0 and out.angular == 0.0
    assert m.step(dl.Snapshot(t=20.0)).linear == 0.0


def _bad_marker(t):
    # seen, but outside the 25 deg gate -> stays in SEARCHING
    return dl.MarkerObs(-1.2, 0.0, math.radians(40), stamp=t) if t < 1.0 else None


def test_no_blind_with_marker_but_unmeasured_dock():
    m = dl.DockStateMachine(dl.DockParams(skip_nav_to_approach=True), dl.Pose2D(),
                            dock_pose_measured=False)
    m.start(dl.Snapshot(t=0.0))
    out = _search_until_timeout(m, _bad_marker)
    assert out.done and out.message.startswith('DOCK_NOT_FOUND')
    assert 'not measured' in out.message and out.linear == 0.0


def test_blind_allowed_with_measured_dock_and_recent_marker():
    m = dl.DockStateMachine(dl.DockParams(skip_nav_to_approach=True), dl.Pose2D(),
                            dock_pose_measured=True)
    m.start(dl.Snapshot(t=0.0))
    out = _search_until_timeout(m, _bad_marker)
    assert not out.done and out.state == S.FINAL_DOCKING


def test_no_blind_when_marker_too_old():
    p = dl.DockParams(skip_nav_to_approach=True, blind_marker_max_age_s=5.0)
    m = dl.DockStateMachine(p, dl.Pose2D(), dock_pose_measured=True)
    m.start(dl.Snapshot(t=0.0))
    out = _search_until_timeout(m, _bad_marker)
    assert out.done and out.message.startswith('DOCK_NOT_FOUND')


def test_marker_from_before_the_attempt_does_not_count():
    m = dl.DockStateMachine(dl.DockParams(skip_nav_to_approach=True), dl.Pose2D(),
                            dock_pose_measured=True)
    m.start(dl.Snapshot(t=100.0))
    old = dl.MarkerObs(-1.2, 0.0, math.radians(40), stamp=99.9)
    assert not m.blind_allowed(100.0)[0]
    m.step(dl.Snapshot(t=100.05, marker=old))
    assert not m.blind_allowed(100.05)[0]


def test_vision_off_without_allow_blind_fails_not_found():
    m = dl.DockStateMachine(dl.DockParams(use_vision=False, skip_nav_to_approach=True),
                            dl.Pose2D(), dock_pose_measured=True)
    out = m.start(dl.Snapshot(t=0.0))
    assert m.state == S.FAILED and out.linear == 0.0
    assert m.message.startswith('DOCK_NOT_FOUND')


# ------------------------------------------------------------ stall guard
def _docking_machine(**kw):
    m = dl.DockStateMachine(dl.DockParams(skip_nav_to_approach=True, **kw), dl.Pose2D(),
                            dock_pose_measured=True)
    m.start(dl.Snapshot(t=0.0))
    mk = dl.MarkerObs(-1.3, 0.0, 0.0, stamp=0.05)
    out = m.step(dl.Snapshot(t=0.05, marker=mk, progress_pose=dl.Pose2D(1.0, 0, 0)))
    assert out.state == S.DOCKING
    return m


def test_stall_aborts_when_ekf_does_not_move():
    m = _docking_machine()
    t, out = 0.05, None
    for _ in range(200):
        t += 0.05
        mk = dl.MarkerObs(-1.3, 0.0, 0.0, stamp=t)
        out = m.step(dl.Snapshot(t=t, marker=mk, progress_pose=dl.Pose2D(1.0, 0, 0)))
        if out.done:
            break
    assert out.done and not out.success and out.message.startswith('DOCK_STALLED')
    assert 3.9 < t < 4.6    # ~1 s stall window + 3 s contact grace
    assert out.linear == 0.0 and out.angular == 0.0
    # no retry / no further reversing afterwards
    assert m.step(dl.Snapshot(t=t + 0.05)).linear == 0.0 and m.state == S.FAILED


def test_no_stall_when_ekf_follows_the_command():
    m = _docking_machine()
    t, x, out = 0.05, 1.0, None
    for _ in range(60):
        x -= abs(out.linear) * 0.05 if out else 0.15 * 0.05
        t += 0.05
        mk = dl.MarkerObs(-1.3 + (1.0 - x), 0.0, 0.0, stamp=t)
        out = m.step(dl.Snapshot(t=t, marker=mk, progress_pose=dl.Pose2D(x, 0, 0)))
        if out.done:
            break
    assert not (out.done and out.message.startswith('DOCK_STALLED'))


def test_stall_guard_unit():
    g = dl.StallGuard(0.03, 0.2, 1.0)
    p = dl.Pose2D(0, 0, 0)
    assert not any(g.update(0.05 * i, p, -0.1) for i in range(20))
    assert g.update(1.05, p, -0.1)
    g.reset()
    assert not any(g.update(0.05 * i, p, -0.02) for i in range(60))  # below min_cmd
    assert not any(g.update(0.05 * i, None, -0.1) for i in range(60))  # no fused pose


def test_dig_stall_rising_edge_aborts_but_old_latch_is_ignored():
    m = _docking_machine()
    out = m.step(dl.Snapshot(t=0.1, marker=dl.MarkerObs(-1.3, 0, 0, stamp=0.1),
                             dig_stall=True))
    assert not out.done and out.linear == 0.0 and out.angular == 0.0   # contact grace hold
    out = m.step(dl.Snapshot(t=3.2, dig_stall=True))
    assert out.done and 'DOCK_STALLED' in out.message and out.linear == 0.0
    m = dl.DockStateMachine(dl.DockParams(skip_nav_to_approach=True), dl.Pose2D(),
                            dock_pose_measured=True)
    m.start(dl.Snapshot(t=0.0, dig_stall=True))
    out = m.step(dl.Snapshot(t=0.05, marker=dl.MarkerObs(-1.3, 0, 0, stamp=0.05),
                             dig_stall=True))
    out = m.step(dl.Snapshot(t=0.1, marker=dl.MarkerObs(-1.3, 0, 0, stamp=0.1),
                             dig_stall=True))
    assert not out.done and out.state == S.DOCKING


def test_cancel_mid_reverse_outputs_zero():
    m = _docking_machine()
    out = m.step(dl.Snapshot(t=0.1, marker=dl.MarkerObs(-1.3, 0, 0, stamp=0.1)))
    assert out.linear < 0
    out = m.cancel()
    assert out.done and out.linear == 0.0 and out.angular == 0.0 and out.message == 'CANCELED'


# ------------------------------------- approach skip / align / nav fallback (2026-10-07)
# Live: dock pose yaw 110.7 deg, approach (0.07, -0.52), robot 0.25 m away at another
# heading -> Nav2 "Failed to make progress" (pivot on turf) -> cancel/re-send loop.
DOCK_YAW = math.radians(110.7)
DOCK = dl.Pose2D(0.07 - 0.8 * math.cos(DOCK_YAW), -0.52 - 0.8 * math.sin(DOCK_YAW), DOCK_YAW)


def _snap(t, map_pose=None, odom=None, **kw):
    return dl.Snapshot(t=t, map_pose=map_pose,
                       odom=odom if odom is not None else map_pose, **kw)


def test_goal_near_approach_with_good_heading_skips_nav():
    m = dl.DockStateMachine(dl.DockParams(), DOCK)
    out = m.start(_snap(0.0, dl.Pose2D(-0.16 + 0.1, -0.42, DOCK_YAW + math.radians(10))))
    assert out.state == S.SEARCHING
    assert not any(r[0] == 'start_nav' for r in out.requests)
    assert any('skipping Nav2' in n for n in out.notes)


def test_goal_near_approach_with_bad_heading_aligns_then_searches():
    yaw0 = DOCK_YAW - math.radians(70)
    m = dl.DockStateMachine(dl.DockParams(), DOCK)
    out = m.start(_snap(0.0, dl.Pose2D(-0.16, -0.42, yaw0)))
    assert out.state == S.ALIGNING and not out.requests
    out = m.step(_snap(0.05, dl.Pose2D(-0.16, -0.42, yaw0)))
    assert out.state == S.ALIGNING
    assert out.angular == pytest.approx(0.25)          # toward the dock yaw (left)
    assert out.linear == 0.0                           # true pivot (legal now)
    assert out.detail.startswith('aligning to dock heading: 70')
    assert 'marker not visible' in out.detail
    # heading reaches the tolerance -> SEARCHING
    out = m.step(_snap(1.0, dl.Pose2D(-0.16, -0.42, DOCK_YAW - math.radians(5))))
    assert out.state == S.SEARCHING and out.linear == 0.0


def test_align_gives_up_on_travel_or_timeout_and_searches():
    yaw0 = DOCK_YAW + math.radians(90)
    m = dl.DockStateMachine(dl.DockParams(), DOCK)
    m.start(_snap(0.0, dl.Pose2D(0.0, -0.5, yaw0)))
    out = m.step(_snap(0.05, dl.Pose2D(0.0, -0.5, yaw0)))
    assert out.state == S.ALIGNING and out.angular == pytest.approx(-0.25)   # right
    out = m.step(_snap(5.0, dl.Pose2D(0.0, -0.05, yaw0)))   # 0.45 m crept, still off
    assert out.state == S.SEARCHING and any('alignment stopped' in n for n in out.notes)
    m = dl.DockStateMachine(dl.DockParams(), DOCK)
    m.start(_snap(0.0, dl.Pose2D(0.0, -0.5, yaw0)))
    out = m.step(_snap(20.5, dl.Pose2D(0.0, -0.5, yaw0)))
    assert out.state == S.ALIGNING                          # timeout is 25 s now
    out = m.step(_snap(25.5, dl.Pose2D(0.0, -0.5, yaw0)))
    assert out.state == S.SEARCHING


def test_align_stops_when_marker_enters_gate():
    m = dl.DockStateMachine(dl.DockParams(), DOCK)
    yaw0 = DOCK_YAW - math.radians(40)
    m.start(_snap(0.0, dl.Pose2D(0.07, -0.52, yaw0)))
    good = dl.MarkerObs(-1.2, 0.05, math.radians(5), stamp=0.1)
    out = m.step(_snap(0.1, dl.Pose2D(0.07, -0.52, yaw0), marker=good))
    assert out.state == S.SEARCHING
    out = m.step(_snap(0.15, dl.Pose2D(0.07, -0.52, yaw0), marker=good))
    assert out.state == S.DOCKING


def test_nav_goal_faces_the_dock_then_about_turn_pivot():
    """Owner rule: go in front of the dock facing it, then about-turn in place."""
    m = dl.DockStateMachine(dl.DockParams(), DOCK)
    out = m.start(_snap(0.0, dl.Pose2D(-2.5, 3.5, 0.0)))
    goal = [r for r in out.requests if r[0] == 'start_nav'][0][1]
    assert (goal.x, goal.y) == pytest.approx((m.approach.x, m.approach.y))
    assert dl.wrap_angle(goal.yaw - (DOCK_YAW + math.pi)) == pytest.approx(0.0, abs=1e-9)
    # arrives facing the dock, 10 deg CCW past exact: 170 deg left is the shorter way
    arr = dl.Pose2D(m.approach.x, m.approach.y, DOCK_YAW + math.pi + math.radians(10))
    out = m.step(_snap(30.0, arr, nav_status='succeeded'))
    assert out.state == S.ALIGNING
    out = m.step(_snap(30.05, arr))
    assert out.linear == 0.0 and out.angular == pytest.approx(0.25)
    arr = dl.Pose2D(m.approach.x, m.approach.y, DOCK_YAW + math.pi - math.radians(10))
    m2 = dl.DockStateMachine(dl.DockParams(), DOCK)
    m2.start(_snap(0.0, dl.Pose2D(-2.5, 3.5, 0.0)))
    m2.step(_snap(30.0, arr, nav_status='succeeded'))
    out = m2.step(_snap(30.05, arr))
    assert out.linear == 0.0 and out.angular == pytest.approx(-0.25)   # 170 deg right
    # about-turn done -> SEARCHING
    out = m2.step(_snap(45.0, dl.Pose2D(m.approach.x, m.approach.y, DOCK_YAW + 0.05)))
    assert out.state == S.SEARCHING


def test_approach_facing_dock_off_keeps_dock_yaw_goal():
    m = dl.DockStateMachine(dl.DockParams(approach_facing_dock=False), DOCK)
    goal = m.start(_snap(0.0, dl.Pose2D(-2.5, 3.5, 0.0))).requests[0][1]
    assert goal.yaw == pytest.approx(m.approach.yaw)


def test_nav_failure_within_1m_is_treated_as_arrived():
    m = dl.DockStateMachine(dl.DockParams(), DOCK)
    far = dl.Pose2D(3.0, 3.0, 0.0)
    assert m.start(_snap(0.0, far)).requests[0][0] == 'start_nav'
    near = dl.Pose2D(-0.16 - 0.5, -0.42, DOCK_YAW)          # ~0.73 m, beyond skip radius
    out = m.step(_snap(20.0, near, nav_status='failed'))
    assert not out.done and out.state == S.SEARCHING
    assert any('treating as arrived' in n for n in out.notes)
    # with the heading off: align instead
    m = dl.DockStateMachine(dl.DockParams(), DOCK)
    m.start(_snap(0.0, far))
    out = m.step(_snap(20.0, dl.Pose2D(-0.16, -0.42, 0.0), nav_status='failed'))
    assert out.state == S.ALIGNING
    # nav timeout within 1 m: cancel Nav2, continue
    m = dl.DockStateMachine(dl.DockParams(nav_timeout_s=5.0), DOCK)
    m.start(_snap(0.0, far))
    out = m.step(_snap(6.0, dl.Pose2D(-0.16, -0.42, DOCK_YAW), nav_status='running'))
    assert out.state == S.SEARCHING and ('cancel_nav',) in out.requests


def test_nav_failure_far_retries_once_with_current_yaw_then_fails():
    m = dl.DockStateMachine(dl.DockParams(), DOCK)
    far = dl.Pose2D(2.0, 1.0, 0.4)
    m.start(_snap(0.0, far))
    out = m.step(_snap(20.0, far, nav_status='failed'))
    assert out.state == S.NAV_TO_APPROACH and not out.done
    req = [r for r in out.requests if r[0] == 'start_nav'][0][1]
    assert (req.x, req.y) == pytest.approx((m.approach.x, m.approach.y))
    assert req.yaw == pytest.approx(0.4)                    # current yaw: no pivot
    assert 'current yaw' in out.detail
    out = m.step(_snap(20.05, far, nav_status='running'))
    assert out.state == S.NAV_TO_APPROACH
    out = m.step(_snap(40.0, far, nav_status='failed'))
    assert out.done and out.message == 'NAV_TO_DOCK_FAILED'


def test_relaxed_nav_success_then_aligns():
    m = dl.DockStateMachine(dl.DockParams(), DOCK)
    far = dl.Pose2D(2.0, 1.0, 0.4)
    m.start(_snap(0.0, far))
    m.step(_snap(20.0, far, nav_status='failed'))
    m.step(_snap(20.05, far, nav_status='running'))
    out = m.step(_snap(30.0, dl.Pose2D(m.approach.x, m.approach.y, 0.4),
                       nav_status='succeeded'))
    assert out.state == S.ALIGNING


def test_detail_reports_marker_visibility():
    m = dl.DockStateMachine(dl.DockParams(skip_nav_to_approach=True), dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    out = m.step(dl.Snapshot(t=0.05))
    assert out.detail == 'searching marker: not visible'
    # marker 0.86 m behind, robot heading 3.5 deg off the dock axis
    mk = dl.MarkerObs(-0.86, 0.0, math.radians(-3.5), stamp=0.1)
    m2 = dl.DockStateMachine(dl.DockParams(skip_nav_to_approach=True), dl.Pose2D())
    m2.start(dl.Snapshot(t=0.0))
    out = m2.step(dl.Snapshot(t=0.1, marker=mk))
    assert out.state == S.DOCKING
    assert 'marker seen 0.86 m, 3.5\N{DEGREE SIGN}' in out.detail
    bad = dl.MarkerObs(-1.2, 0.0, math.radians(40), stamp=0.1)
    out = m.step(dl.Snapshot(t=0.1, marker=bad))
    assert out.detail.startswith('searching marker: seen 1.20 m, -40.0\N{DEGREE SIGN}')
    assert 'outside gate' in out.detail and out.state == S.SEARCHING


def _marker_from_pose(x, y, th, off, stamp):
    """Synthetic marker for a robot at (x, y, th) in the dock frame (docked pose at the
    origin, heading 0; marker plane `off` behind it, normal +X)."""
    mx, my = -off - x, -y
    c, s = math.cos(-th), math.sin(-th)
    return dl.MarkerObs(c * mx - s * my, s * mx + c * my, -th, stamp=stamp)


def test_marker_from_pose_matches_dock_errors():
    for x, y, th in ((0.75, 0.0, 0.0), (1.0, 0.3, 0.2), (0.6, -0.4, -0.3)):
        e = dl.dock_errors(_marker_from_pose(x, y, th, 0.45, 0.0), 0.45)
        assert (e.remaining, e.lateral, e.heading) == pytest.approx((x, y, th), abs=1e-9)


@pytest.mark.parametrize('x0,y0,th0', [
    (1.0, 0.5, math.radians(24)), (1.0, -0.5, math.radians(24)),
    (0.8, 0.45, -math.radians(24)), (1.4, -0.5, -math.radians(24)), (0.8, 0.0, 0.0)])
def test_vision_docking_tolerates_gate_edge_start(x0, y0, th0):
    """Enter SEARCHING at the gate edge (25 deg / 0.5 m): the FSM must end on the contacts
    (within 3 cm of the axis) using realigns, without consuming retries or failing."""
    p = dl.DockParams(skip_nav_to_approach=True)
    m = dl.DockStateMachine(p, dl.Pose2D(), goal_timeout_s=400.0)
    x, y, th, t, dt = x0, y0, th0, 0.0, 0.05
    m.start(dl.Snapshot(t=0.0))
    deb = dl.ContactDebouncer(3)
    out = None
    for _ in range(8000):
        t += dt
        deb.update(x <= 0.0 and abs(y) < 0.03)
        mk = _marker_from_pose(x, y, th, p.docked_marker_offset, t) if x > 0.05 else None
        out = m.step(dl.Snapshot(t=t, odom=dl.Pose2D(x, y, th), marker=mk,
                                 contact=deb.value, charging_result=True if
                                 m.state == S.CHARGING else None,
                                 progress_pose=dl.Pose2D(x, y, th)))
        if out.done:
            break
        x += out.linear * math.cos(th) * dt
        y += out.linear * math.sin(th) * dt
        th += out.angular * dt
        x = max(x, 0.0)
    assert out.success, (out.message, m.state, x, y, th)
    assert m.retries == 0


def test_reverse_control_turn_first_slows_on_large_heading_error():
    g = dl.ControllerGains()
    v_big, _, _ = dl.reverse_control(dl.DockErrors(1.0, 0.0, math.radians(25)), g)
    v_small, _, _ = dl.reverse_control(dl.DockErrors(1.0, 0.0, math.radians(2)), g)
    assert -g.max_speed <= v_small < v_big < 0
    assert abs(v_big) >= g.min_turn_speed


# ------------------------- FINAL budget / contact handling / calibration (2026-10-07 Home)
# Owner saw: reversed onto the pins, then the robot drove forward off them ("undock on
# touching the pins"); contacts reported 7 s later while the server was in RETRY.
def _final_machine(**kw):
    p = dl.DockParams(skip_nav_to_approach=True, **kw)
    m = dl.DockStateMachine(p, dl.Pose2D(), goal_timeout_s=400.0)
    m.start(dl.Snapshot(t=0.0, odom=dl.Pose2D(0.65, 0, 0)))
    mk = dl.MarkerObs(-1.1, 0.0, 0.0, stamp=0.05)          # remaining 0.65
    assert m.step(dl.Snapshot(t=0.05, marker=mk, odom=dl.Pose2D(0.65, 0, 0))).state == S.DOCKING
    mk = dl.MarkerObs(-0.6, 0.0, 0.0, stamp=0.1)           # remaining 0.15: still DOCKING
    out = m.step(dl.Snapshot(t=0.1, marker=mk, odom=dl.Pose2D(0.15, 0, 0)))
    assert out.state == S.DOCKING
    out = m.step(dl.Snapshot(t=0.1, odom=dl.Pose2D(0.15, 0, 0)))   # marker lost -> final
    assert out.state == S.FINAL_DOCKING
    return m


def test_final_budget_settles_then_creeps_before_retry():
    m = _final_machine()
    t, x, out, phases = 0.1, 0.15, None, []
    while t < 40:
        t += 0.05
        out = m.step(dl.Snapshot(t=t, odom=dl.Pose2D(x, 0, 0), progress_pose=dl.Pose2D(x, 0, 0)))
        x += out.linear * 0.05
        phases.append((round(t, 2), out.state, m._final_phase, out.linear))
        if out.state == S.RETRY:
            break
    settle = [p for p in phases if p[2] == 'settle' and p[1] == S.FINAL_DOCKING]
    assert settle and all(p[3] == 0.0 for p in settle)
    assert settle[-1][0] - settle[0][0] >= 3.0 - 0.06
    creep = [p for p in phases if p[2] == 'creep' and p[1] == S.FINAL_DOCKING and p[3] < 0]
    assert creep and all(p[3] == pytest.approx(-0.05) for p in creep)
    assert out.state == S.RETRY
    # remaining 0.15 + 0.10 overrun + 0.15 creep reversed in total from final entry
    assert 0.15 - x == pytest.approx(0.40, abs=0.03)


def test_contact_during_settle_docks_without_retry():
    m = _final_machine()
    t, x = 0.1, 0.15
    while m._final_phase != 'settle':
        t += 0.05
        out = m.step(dl.Snapshot(t=t, odom=dl.Pose2D(x, 0, 0)))
        x += out.linear * 0.05
    out = m.step(dl.Snapshot(t=t + 1.0, odom=dl.Pose2D(x, 0, 0), contact=True))
    assert out.state == S.CHARGING and ('enable_charging',) in out.requests
    out = m.step(dl.Snapshot(t=t + 1.1, contact=True, charging_result=True))
    assert out.done and out.success


@pytest.mark.parametrize('kind', ['debounced', 'charging'])
def test_contact_in_retry_stops_forward_and_confirms(kind):
    sim = Sim(dl.DockStateMachine(blind_params(max_retries=2), dl.Pose2D()), contact_after=None)
    sim.start()
    for _ in range(5000):
        out = sim.tick()
        if out.state == S.RETRY and out.linear > 0:
            break
    assert out.state == S.RETRY and out.linear > 0
    snap = sim.snap()
    snap = dl.Snapshot(t=snap.t + 0.05, odom=snap.odom, contact=(kind == 'debounced'),
                       is_charging=(kind == 'charging'), raw_contact=True)
    out = sim.m.step(snap)
    assert out.state == S.CHARGING and out.linear == 0.0 and out.angular == 0.0
    assert ('enable_charging',) in out.requests


def test_raw_contact_in_retry_holds_still():
    sim = Sim(dl.DockStateMachine(blind_params(max_retries=2), dl.Pose2D()), contact_after=None)
    sim.start()
    for _ in range(5000):
        out = sim.tick()
        if out.state == S.RETRY and out.linear > 0:
            break
    s = sim.snap()
    out = sim.m.step(dl.Snapshot(t=s.t + 0.05, odom=s.odom, raw_contact=True))
    assert out.state == S.RETRY and out.linear == 0.0 and out.angular == 0.0


def test_contact_during_nav_cancels_nav():
    m = dl.DockStateMachine(dl.DockParams(), dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    out = m.step(dl.Snapshot(t=1.0, nav_status='running', contact=True))
    assert out.state == S.CHARGING and ('cancel_nav',) in out.requests


def test_success_reports_calibrated_marker_offset():
    m = _final_machine()
    # last marker at stamp 0.1: rem0 = 0.6 (offset 0); robot then reverses 0.10 m more
    t = 0.1
    out = None
    for i in range(1, 41):
        t += 0.05
        x = 0.15 - 0.0025 * i                                # 0.10 m in 40 ticks
        out = m.step(dl.Snapshot(t=t, odom=dl.Pose2D(x, 0, 0)))
    out = m.step(dl.Snapshot(t=t + 0.05, odom=dl.Pose2D(0.05, 0, 0), contact=True))
    assert out.state == S.CHARGING and out.calibrated_offset is None
    out = m.step(dl.Snapshot(t=t + 0.1, contact=True, charging_result=True))
    assert out.success and out.calibrated_offset == pytest.approx(0.6 - 0.10, abs=1e-6)


def test_calibration_rejects_stale_or_absurd():
    m = _final_machine(calib_max_marker_age_s=1.0)
    out = m.step(dl.Snapshot(t=5.0, odom=dl.Pose2D(0.1, 0, 0), contact=True))
    m.step(dl.Snapshot(t=5.05, contact=True, charging_result=True))
    assert m.calibrated_offset is None


# ------------- continuous correction to contact (2026-10-07 first live dock: arrived
# 8 cm / 8 deg, two forward realign loops, then a straight FINAL with no steering)
def _closed_loop(x0, y0, th0, noise_frame=None, **kw):
    """Full FSM in the dock frame; contact at x <= 0 (x clamped: the dock is a wall).
    Returns (out, machine, (lateral, heading) at contact, time, details)."""
    p = dl.DockParams(skip_nav_to_approach=True, **kw)
    m = dl.DockStateMachine(p, dl.Pose2D(), goal_timeout_s=400.0)
    x, y, th, t, dt = x0, y0, th0, 0.0, 0.05
    m.start(dl.Snapshot(t=0.0))
    deb = dl.ContactDebouncer(3)
    out, at_contact, details, k, injected = None, None, [], 0, []
    for _ in range(10000):
        t += dt
        k += 1
        if x <= 0.0 and at_contact is None:
            at_contact = (y, th)
        deb.update(x <= 0.0)
        mk = _marker_from_pose(x, y, th, p.docked_marker_offset, t) if x > 0.05 else None
        if mk is not None and noise_frame is not None and noise_frame(m):
            mk = dl.MarkerObs(mk.x, mk.y + 0.25, mk.yaw + 0.3, mk.stamp)   # one wild frame
            injected.append(t)
        out = m.step(dl.Snapshot(t=t, odom=dl.Pose2D(x, y, th), marker=mk, contact=deb.value,
                                 charging_result=True if m.state == S.CHARGING else None,
                                 progress_pose=dl.Pose2D(x, y, th)))
        details.append(out.detail)
        if out.done:
            break
        x += out.linear * math.cos(th) * dt
        y += out.linear * math.sin(th) * dt
        th += out.angular * dt
        x = max(x, 0.0)
    _closed_loop.injected = injected
    return out, m, at_contact, t, details


def test_converges_from_half_metre_8cm_8deg_without_realign():
    out, m, (lat, hdg), t, details = _closed_loop(0.5, 0.08, math.radians(8))
    assert out.success and m._realigns == 0 and m.retries == 0
    assert abs(lat) < 0.02 and abs(math.degrees(hdg)) < 2.0
    assert t < 15.0
    assert any(d.startswith('docking: 0.') and 'lateral' in d and '\N{DEGREE SIGN}' in d
               for d in details)


@pytest.mark.parametrize('x0', [0.6, 1.0, 1.4])
@pytest.mark.parametrize('y0', [-0.3, 0.0, 0.15, 0.3])
@pytest.mark.parametrize('thd', [-20, 0, 20])
def test_closed_loop_sweep_converges_at_contact(x0, y0, thd):
    out, m, at, t, _ = _closed_loop(x0, y0, math.radians(thd))
    assert out.success and at is not None and m.retries == 0
    assert abs(at[0]) < 0.02 and abs(math.degrees(at[1])) < 2.0
    assert m._realigns <= 2 and t < 60.0


def test_speed_floor_keeps_stall_guard_armed():
    g = dl.ControllerGains()
    p = dl.DockParams()
    v, _, _ = dl.reverse_control(dl.DockErrors(0.1, 0.3, 0.4), g)
    assert abs(v) == pytest.approx(g.min_turn_speed) and abs(v) > p.stall_min_cmd
    v, _, _ = dl.reverse_control(dl.DockErrors(0.25, 0.0, 0.0), g)
    assert abs(v) == pytest.approx(0.08)


def test_moderate_error_at_final_zone_keeps_correcting_no_realign():
    p = dl.DockParams(skip_nav_to_approach=True)
    m = dl.DockStateMachine(p, dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    for i in range(4):   # 0.15 m out, 10 cm / 10 deg off: corrected while reversing
        out = m.step(dl.Snapshot(t=0.1 * (i + 1), odom=dl.Pose2D(),
                                 marker=_marker_from_pose(0.15, 0.10, math.radians(10), 0.45,
                                                          0.1 * (i + 1))))
    assert out.state == S.DOCKING and out.linear < 0 and m._realigns == 0


def test_bad_error_at_final_zone_realigns_at_most_twice():
    p = dl.DockParams(skip_nav_to_approach=True)
    m = dl.DockStateMachine(p, dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    t = 0.0
    for _ in range(4):
        m.state, m._t_state = S.DOCKING, t   # force back to DOCKING at the final zone
        out = None
        for _ in range(3):
            t += 0.1
            out = m.step(dl.Snapshot(t=t, odom=dl.Pose2D(),
                                     marker=_marker_from_pose(0.15, 0.2, 0.0, 0.45, t)))
            if out.state != S.DOCKING:
                break
    assert m._realigns == 2


def test_single_noisy_frame_does_not_trigger_realign():
    # robot nearly on axis; one wild frame (25 cm / 17 deg) right in the final zone
    done = []

    def once(mm):
        if done or not (mm.state == S.DOCKING and 0 < mm._remaining < 0.18):
            return False
        done.append(1)
        return True
    out, m, at, _, _ = _closed_loop(0.5, 0.02, 0.0, noise_frame=once)
    assert len(_closed_loop.injected) == 1
    assert out.success and m._realigns == 0
    # without the median the same frame would have crossed the realign gate
    assert 0.25 > dl.DockParams().final_max_lateral


def test_overrun_with_marker_in_view_holds_for_contacts():
    p = dl.DockParams(skip_nav_to_approach=True)
    m = dl.DockStateMachine(p, dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    for i in range(3):
        out = m.step(dl.Snapshot(t=0.1 * (i + 1), odom=dl.Pose2D(),
                                 marker=_marker_from_pose(-0.12, 0.0, 0.0, 0.45, 0.1 * (i + 1))))
    assert out.state == S.FINAL_DOCKING and m._final_phase == 'settle' and out.linear == 0.0


def test_marker_in_view_flag():
    p = dl.DockParams(skip_nav_to_approach=True)
    m = dl.DockStateMachine(p, dl.Pose2D())
    out = m.start(dl.Snapshot(t=0.0))
    assert not out.marker_in_view
    out = m.step(dl.Snapshot(t=0.1))
    assert not out.marker_in_view                        # searching, nothing seen
    bad = dl.MarkerObs(-1.2, 0.0, math.radians(60), stamp=0.2)
    assert not m.step(dl.Snapshot(t=0.2, marker=bad)).marker_in_view   # outside the gate
    good = _marker_from_pose(0.8, 0.0, 0.0, 0.45, 0.3)
    assert m.step(dl.Snapshot(t=0.3, marker=good)).marker_in_view
    assert m.step(dl.Snapshot(t=0.6, marker=good)).marker_in_view       # 0.3 s old
    assert not m.step(dl.Snapshot(t=0.9, marker=good)).marker_in_view   # stale (> 0.5 s)
    assert not m.cancel().marker_in_view
    # vision FINAL (marker lost in the last 0.2 m, settle/creep): flag stays true
    fm = _final_machine()
    assert fm.state == S.FINAL_DOCKING
    assert fm.step(dl.Snapshot(t=5.0, odom=dl.Pose2D(0.1, 0, 0))).marker_in_view


# ---- 2026-10-07: stall against the dock before the contact debounce confirmed ----------

def _stall_into_hold(m):
    t, out = 0.05, None
    for _ in range(60):
        t += 0.05
        out = m.step(dl.Snapshot(t=t, marker=dl.MarkerObs(-1.3, 0, 0, stamp=t),
                                 progress_pose=dl.Pose2D(1.0, 0, 0)))
        if m._stall_hold is not None:
            break
    assert m._stall_hold is not None and not out.done
    assert out.linear == 0.0 and out.angular == 0.0
    return t


@pytest.mark.parametrize('contact_kw', [dict(raw_contact=True), dict(contact=True),
                                        dict(is_charging=True)])
def test_stall_then_contact_within_grace_docks(contact_kw):
    m = _docking_machine()
    t = _stall_into_hold(m)
    out = m.step(dl.Snapshot(t=t + 0.5, progress_pose=dl.Pose2D(1.0, 0, 0)))
    assert not out.done and out.linear == 0.0 and out.state == S.DOCKING
    out = m.step(dl.Snapshot(t=t + 2.0, progress_pose=dl.Pose2D(1.0, 0, 0), **contact_kw))
    assert out.state == S.CHARGING and ('enable_charging',) in out.requests
    assert out.linear == 0.0
    out = m.step(dl.Snapshot(t=t + 2.2, charging_result=True, **contact_kw))
    assert out.done and out.success and out.state == S.SUCCEEDED


def test_stall_without_contact_fails_after_grace():
    m = _docking_machine(stall_contact_grace_s=3.0)
    t = _stall_into_hold(m)
    out = m.step(dl.Snapshot(t=t + 2.9, progress_pose=dl.Pose2D(1.0, 0, 0)))
    assert not out.done and out.linear == 0.0
    out = m.step(dl.Snapshot(t=t + 3.05, progress_pose=dl.Pose2D(1.0, 0, 0)))
    assert out.done and not out.success and out.message.startswith('DOCK_STALLED')
    assert 'DOCKING' in out.message


def test_stall_grace_zero_fails_immediately():
    m = _docking_machine(stall_contact_grace_s=0.0)
    t, out = 0.05, None
    for _ in range(60):
        t += 0.05
        out = m.step(dl.Snapshot(t=t, marker=dl.MarkerObs(-1.3, 0, 0, stamp=t),
                                 progress_pose=dl.Pose2D(1.0, 0, 0)))
        if out.done:
            break
    assert out.done and out.message.startswith('DOCK_STALLED') and t < 1.5


def test_stall_guard_counts_pivot_as_progress():
    g = dl.StallGuard(0.03, 0.2, 1.0, turn_radius=0.2)
    # commanded 0.07 m/s, the shaper pivots at 0.3 rad/s in place: 0.06 m/s wheel travel
    trips = [g.update(0.05 * i, dl.Pose2D(0, 0, 0.3 * 0.05 * i), -0.07) for i in range(60)]
    assert not any(trips)
    g = dl.StallGuard(0.03, 0.2, 1.0, turn_radius=0.2)
    trips = [g.update(0.05 * i, dl.Pose2D(0, 0, 0.0), -0.07) for i in range(30)]
    assert any(trips)


def test_post_fail_watch():
    w = dl.PostFailWatch(0.0, 10.0)
    assert w.step(1.0, False, False, False) is None
    assert w.step(8.0, True, False, False) == 'enable'
    assert dl.PostFailWatch(0.0, 10.0).step(1.0, True, True, False) == 'charging'
    assert dl.PostFailWatch(0.0, 10.0).step(1.0, True, False, True) == 'abort'
    w = dl.PostFailWatch(0.0, 10.0)
    assert w.step(10.5, False, False, False) == 'abort'
