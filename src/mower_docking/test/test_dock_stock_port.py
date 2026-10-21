"""2026-10-21 stock-structure port: marker filter, no search gate, offset retries, RTK
lateral pre-alignment, reverse on the prediction while the marker is lost, pose check."""
import math

from mower_docking import dock_logic as dl

S = dl.DockState
OFF = 0.45
FLOOR = 0.06
HALF = 0.20


class World:
    """Dock at the map origin (yaw 0); the robot pose is map = odom; marker rendered from the
    true pose (optionally hidden / biased); contact at x <= 0."""

    def __init__(self, m, x, y, yaw, rtk=True):
        self.m, self.t = m, 0.0
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0
        self.hidden = lambda t: False
        self.visible = True
        self.rtk = rtk
        self.outs = []
        self.charging = None
        self.deb = dl.ContactDebouncer(3)

    def marker(self):
        if not self.visible or self.hidden(self.t):
            return None
        dx, dy = -OFF - self.x, -self.y
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        return dl.MarkerObs(c * dx - s * dy, s * dx + c * dy, dl.wrap_angle(-self.yaw),
                            stamp=self.t)

    def snap(self):
        p = dl.Pose2D(self.x, self.y, self.yaw)
        return dl.Snapshot(t=self.t, odom=p, map_pose=p, progress_pose=p, marker=self.marker(),
                           contact=self.deb.value, rtk_fixed=self.rtk,
                           charging_result=self.charging, nav_status='succeeded')

    def apply(self, out):
        self.v, self.w = out.linear, out.angular
        self.outs.append(out)
        if any(r[0] == 'enable_charging' for r in out.requests):
            self.charging = True
        return out

    def run(self, seconds, dt=0.05, until=None):
        out = self.apply(self.m.start(self.snap())) if not self.outs else self.outs[-1]
        for _ in range(int(seconds / dt)):
            self.t += dt
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.yaw += self.w * dt
            self.deb.update(self.x <= 0.0)
            out = self.apply(self.m.step(self.snap()))
            if out.done or (until is not None and until(out)):
                break
        return out


def params(**kw):
    kw.setdefault('skip_nav_to_approach', True)
    return dl.DockParams(**kw)


def both_wheels_ok(outs):
    """No wheel turning against the body motion below the floor (a sub-floor pivot / one-wheel
    turn). reverse_control's existing [0, floor) same-direction inner-wheel band is kept."""
    for o in outs:
        if abs(o.linear) < 1e-9:
            continue
        left, right = o.linear - o.angular * HALF, o.linear + o.angular * HALF
        for wv in (left, right):
            if wv * o.linear < 0 and abs(wv) < FLOOR - 1e-6:
                return False
    return True


# ---------------------------------------------------------------- filter
def test_filter_predicts_reverse_motion():
    f = dl.MarkerFilter()
    f.predict(dl.Pose2D(0.0, 0.0, 0.0))
    f.init(dl.DockErrors(0.8, 0.0, 0.0))
    f.predict(dl.Pose2D(-0.1, 0.0, 0.0))        # reversed 0.1 m
    assert abs(f.x.remaining - 0.7) < 1e-9 and abs(f.x.lateral) < 1e-9


def test_filter_prediction_in_rotated_odom_frame():
    f = dl.MarkerFilter()
    f.predict(dl.Pose2D(5.0, 5.0, math.pi / 2))
    f.init(dl.DockErrors(0.8, 0.1, math.radians(10)))
    # reverse 0.1 m along the robot axis (odom yaw 90 deg)
    f.predict(dl.Pose2D(5.0, 4.9, math.pi / 2))
    assert abs(f.x.remaining - (0.8 - 0.1 * math.cos(math.radians(10)))) < 1e-9
    assert abs(f.x.lateral - (0.1 - 0.1 * math.sin(math.radians(10)))) < 1e-9


def test_filter_clamps_innovation():
    f = dl.MarkerFilter(max_innov_m=0.5, max_innov_rad=math.radians(20))
    f.init(dl.DockErrors(0.8, 0.0, 0.0))
    f.correct(dl.DockErrors(0.8, 3.0, math.radians(90)))   # flipped / garbage
    assert f.clamped == 1
    assert abs(f.x.lateral) <= 0.5 and abs(f.x.heading) <= math.radians(20)


def test_filter_converges_to_measurement():
    f = dl.MarkerFilter()
    f.init(dl.DockErrors(0.8, 0.2, 0.0))
    for _ in range(30):
        f.predict(None)
        f.correct(dl.DockErrors(0.8, 0.0, 0.0))
    assert abs(f.x.lateral) < 0.01


# ---------------------------------------------------------------- gate dropped
def test_search_accepts_marker_outside_old_gate():
    m = dl.DockStateMachine(params(), dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    bad = dl.MarkerObs(-1.2, 0.0, math.radians(40), stamp=0.1)
    out = m.step(dl.Snapshot(t=0.1, marker=bad))
    assert out.state == S.DOCKING
    assert any('accepted' in n for n in out.notes)


def test_old_gate_still_available():
    m = dl.DockStateMachine(params(use_marker_gate=True), dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    bad = dl.MarkerObs(-1.2, 0.0, math.radians(40), stamp=0.1)
    assert m.step(dl.Snapshot(t=0.1, marker=bad)).state == S.SEARCHING


# ---------------------------------------------------------------- full approach
def test_docks_from_offset_start_both_wheels():
    m = dl.DockStateMachine(params(lateral_align=False), dl.Pose2D(), dock_pose_measured=True)
    w = World(m, 0.8, 0.08, math.radians(-6))
    out = w.run(120)
    assert out.success, [o.notes for o in w.outs if o.notes][-5:]
    assert both_wheels_ok(w.outs)
    assert abs(w.y) < 0.03


def test_marker_lost_briefly_keeps_reversing_on_prediction():
    m = dl.DockStateMachine(params(lateral_align=False), dl.Pose2D(), dock_pose_measured=True)
    w = World(m, 0.9, 0.0, 0.0)
    w.hidden = lambda t: 2.0 <= t < 3.5
    out = w.run(120)
    assert out.success
    # while hidden (after marker_timeout_s), the robot still reverses, slower, >= floor
    i0, i1 = int(2.6 / 0.05), int(3.4 / 0.05)
    seg = w.outs[i0:i1]
    assert all(o.state == S.DOCKING for o in seg)
    assert all(-0.08 - 1e-9 <= o.linear <= -FLOOR + 1e-9 for o in seg), [o.linear for o in seg]
    assert S.RETRY not in [o.state for o in w.outs]


def test_marker_lost_long_holds_after_prediction_window():
    m = dl.DockStateMachine(params(lateral_align=False, lost_predict_s=1.0,
                                   marker_lost_hold_s=100.0, max_lost_frames=10**6),
                            dl.Pose2D(), dock_pose_measured=True)
    w = World(m, 1.0, 0.0, 0.0)
    w.hidden = lambda t: t >= 1.0
    w.run(5.0)
    assert w.outs[-1].state == S.DOCKING and w.outs[-1].linear == 0.0


def test_pose_check_realigns_when_off():
    p = params(lateral_align=False, final_max_lateral=0.05, final_max_heading_deg=8.0,
               realign_forward_distance=0.65)
    p.gains.k_lateral = 0.0          # controller does not correct: the check must catch it
    p.gains.kp_heading = 0.0
    m = dl.DockStateMachine(p, dl.Pose2D(), dock_pose_measured=True)
    w = World(m, 0.8, 0.10, 0.0)
    w.run(30, until=lambda o: o.state == S.RETRY)
    assert w.outs[-1].state == S.RETRY and m._realign
    # the check fired around a marker distance of 0.65 m
    assert abs((w.x + OFF) - 0.65) < 0.05
    assert m._retry_distance == 0.65


def test_pose_check_passes_when_on_axis():
    m = dl.DockStateMachine(params(lateral_align=False, final_max_lateral=0.05,
                                   final_max_heading_deg=8.0), dl.Pose2D(),
                            dock_pose_measured=True)
    w = World(m, 0.8, 0.01, 0.0)
    out = w.run(60)
    assert out.success and m._realigns == 0


# ---------------------------------------------------------------- retries
def test_retry_offsets_alternate_and_grow_on_the_seen_side():
    m = dl.DockStateMachine(params(), dl.Pose2D())
    m._side = -1.0
    assert [m._offset_for(k) for k in range(1, 9)] == \
        [-0.1, 0.1, -0.2, 0.2, -0.3, 0.3, 0.0, -0.1]


def test_retry_moves_the_approach_goal_sideways():
    d = dl.Pose2D(1.0, 2.0, math.pi / 2)
    m = dl.DockStateMachine(dl.DockParams(max_retries=8), d)
    m.start(dl.Snapshot(t=0.0, map_pose=dl.Pose2D(5, 5, 0)))
    m._side = 1.0
    m._enter_retry(dl.Snapshot(t=1.0), 0.0, renav=True)
    # dock yaw 90 deg: the axis points +y, left of it is -x
    assert abs(m.approach.x - (1.0 - 0.1)) < 1e-9 and abs(m.approach.y - 2.8) < 1e-9
    nav = [r for r in m._out().requests if r[0] == 'start_nav']
    assert nav and abs(nav[0][1].x - 0.9) < 1e-9


def test_max_retries_eight_then_maxout():
    m = dl.DockStateMachine(params(max_retries=8, search_timeout_s=1.0), dl.Pose2D())
    w = World(m, 0.8, 0.0, 0.0)
    w.visible = False
    out = w.run(400)
    assert out.done and not out.success and m.retries == 8
    assert 'DOCK_NOT_FOUND' in out.message


# ---------------------------------------------------------------- lateral pre-alignment
def test_lateral_prealign_gets_back_on_axis_with_both_wheels():
    m = dl.DockStateMachine(params(), dl.Pose2D(), dock_pose_measured=True)
    w = World(m, 0.8, 0.30, 0.0)
    w.visible = False
    w.run(60, until=lambda o: o.state == S.SEARCHING)
    states = [o.state for o in w.outs]
    assert S.LATERAL_ALIGN in states and w.outs[-1].state == S.SEARCHING
    assert abs(w.y) < 0.06 and abs(w.yaw) < math.radians(8)
    assert abs(w.x - 0.8) < 0.1
    moving = [o for o in w.outs if o.state == S.LATERAL_ALIGN]
    assert both_wheels_ok(moving)
    assert all(abs(o.angular) <= 0.3 + 1e-9 for o in moving)
    assert any(o.linear > 0 for o in moving) and any(o.linear < 0 for o in moving)
    for o in moving:      # forward leg: both wheels at or above the floor
        if o.linear > 0:
            assert min(o.linear - abs(o.angular) * HALF, o.linear + abs(o.angular) * HALF) \
                >= FLOOR - 1e-9


def test_lateral_prealign_targets_the_retry_offset_line():
    m = dl.DockStateMachine(params(), dl.Pose2D(), dock_pose_measured=True)
    m._set_offset(0.2)
    w = World(m, 0.8, 0.2, 0.0)
    w.visible = False
    w.run(5, until=lambda o: o.state == S.SEARCHING)
    assert S.LATERAL_ALIGN not in [o.state for o in w.outs]


def test_lateral_prealign_needs_rtk_and_measured_dock():
    for measured, rtk in ((False, True), (True, False)):
        m = dl.DockStateMachine(params(), dl.Pose2D(), dock_pose_measured=measured)
        w = World(m, 0.8, 0.3, 0.0, rtk=rtk)
        w.visible = False
        w.run(1.0)
        assert S.LATERAL_ALIGN not in [o.state for o in w.outs]


def test_full_dock_from_lateral_offset():
    m = dl.DockStateMachine(params(), dl.Pose2D(), dock_pose_measured=True)
    w = World(m, 0.8, -0.25, math.radians(5))
    out = w.run(200)
    assert out.success and S.LATERAL_ALIGN in [o.state for o in w.outs]
    assert both_wheels_ok(w.outs)
