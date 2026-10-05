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
    assert dl.reverse_speed(0.3, g) == pytest.approx(0.05)
    assert 0.05 < dl.reverse_speed(0.45, g) < 0.15


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
    p = dl.DockParams(use_vision=False, skip_nav_to_approach=True)
    for k, v in kw.items():
        setattr(p, k, v)
    return p


# ---------------------------------------------------------------- dock FSM
def test_dock_starts_with_nav_to_approach_request():
    m = dl.DockStateMachine(dl.DockParams(), dl.Pose2D(0, 0, 0))
    out = m.start(dl.Snapshot(t=0.0))
    assert out.state == S.NAV_TO_APPROACH
    assert out.requests[0][0] == 'start_nav'
    assert out.requests[0][1] == dl.Pose2D(0.8, 0.0, 0.0)
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
    m = dl.DockStateMachine(dl.DockParams(use_vision=False), dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    out = m.step(dl.Snapshot(t=1.0, nav_status='succeeded', odom=dl.Pose2D(0.8, 0, 0)))
    assert out.state == S.FINAL_DOCKING


def test_dock_search_timeout_falls_back_to_blind():
    p = dl.DockParams(skip_nav_to_approach=True, search_timeout_s=2.0)
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
    out = m.step(dl.Snapshot(t=0.2, marker=dl.MarkerObs(-0.72, 0.0, 0.0, stamp=0.2)))
    assert out.state == S.DOCKING and out.linear == pytest.approx(-0.05)  # slow zone
    out = m.step(dl.Snapshot(t=0.25, marker=dl.MarkerObs(-0.6, 0.0, 0.0, stamp=0.25)))
    assert out.state == S.FINAL_DOCKING
    assert out.linear == pytest.approx(-0.05)


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
                      approach_distance=0.1, blind_extra_distance=0.05)
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
    # the fallback integrates |cmd| * dt and stops at approach + 0.1 m
    assert sim.reversed == pytest.approx(0.9, abs=0.02)


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
