"""2026-10-21 night field docking regressions (findings doc, "Field docking 2026-10-21 night").

Synthetic marker stream modelled on the night's 329 detections: ~4 Hz, 71 % inliers (heading
noise 3.9 deg, position noise 2 cm), 29 % IPPE pose flips (the marker normal 22 deg off), all
seen through a rear camera mounted 7.5 deg off its nominal yaw. The detections go through the
real chain (camera-frame rvec/tvec -> marker_in_base with the configured yaw correction).
"""
import math
import random

import pytest

from mower_docking import dock_logic as dl

S = dl.DockState
OFF = 0.45
CAM_XYZ = (-0.201, 0.0, 0.25)
CAM_RPY = (-1.5707963, 0.0, 1.5707963)
MOUNT_ERR = math.radians(-7.5)        # true camera yaw minus nominal
HALF = 0.20
FLOOR = 0.06


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mt(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _mv(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _rvec(r):
    """Rotation matrix -> Rodrigues vector (angles well below pi here)."""
    tr = r[0][0] + r[1][1] + r[2][2]
    th = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    if th < 1e-9:
        return [0.0, 0.0, 0.0]
    k = th / (2.0 * math.sin(th))
    return [k * (r[2][1] - r[1][2]), k * (r[0][2] - r[2][0]), k * (r[1][0] - r[0][1])]


# marker frame in the dock frame: Z (normal) = +x, Y = up, X = Y x Z = +y
R_DM = [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]


def detection(x, y, th, stamp, flip=0.0, hdg_noise=0.0, pos_noise=(0.0, 0.0),
              mount=MOUNT_ERR, correction=MOUNT_ERR):
    """What the node would publish for a robot at (x, y, th) in the dock frame: the marker
    pose solved in the TRUE (mis-mounted) camera, converted with the nominal extrinsic plus
    ``correction``. ``flip`` / ``hdg_noise`` rotate the solved marker normal (IPPE ambiguity)."""
    p_m = [-OFF, 0.0, 0.25]
    c, s = math.cos(th), math.sin(th)
    dx, dy = p_m[0] - x + pos_noise[0], p_m[1] - y + pos_noise[1]
    p_base = [c * dx + s * dy, -s * dx + c * dy, 0.25]
    r_bm = _mm(_rz(-th), R_DM)
    r_bc_true = _mm(_rz(mount), dl.rpy_to_matrix(*CAM_RPY))
    r_cm = _mm(_mt(r_bc_true), r_bm)
    # ambiguity: the normal swings about the marker's vertical axis (marker-frame Y)
    r_cm = _mm(r_cm, _ry(-(flip + hdg_noise)))
    t_cm = _mv(_mt(r_bc_true), [p_base[i] - CAM_XYZ[i] for i in range(3)])
    return dl.marker_in_base(_rvec(r_cm), t_cm, CAM_XYZ, CAM_RPY, stamp=stamp,
                             yaw_correction=correction)


def test_yaw_correction_removes_the_mount_error():
    for x, y, th in ((1.0, 0.1, 0.1), (0.6, -0.2, -0.3)):
        m = detection(x, y, th, 0.0)
        e = dl.dock_errors(m, OFF)
        assert (e.remaining, e.lateral, e.heading) == pytest.approx((x, y, th), abs=1e-3)
        biased = dl.dock_errors(detection(x, y, th, 0.0, correction=0.0), OFF)
        assert abs(math.degrees(dl.wrap_angle(biased.heading - th)) + 7.5) < 0.5


def test_position_only_errors_ignore_the_marker_yaw():
    m = detection(1.0, 0.15, 0.2, 0.0, flip=math.radians(22))
    raw = dl.dock_errors(m, OFF)
    assert abs(raw.lateral - 0.15) > 0.1             # the flip moves the yaw-derived lateral
    pos = dl.dock_errors_with_heading(m, 0.2, OFF)    # true heading from the filter
    assert (pos.remaining, pos.lateral) == pytest.approx((1.0, 0.15), abs=1e-3)


class FieldStream:
    """Detections at ~4 Hz (every 5th 20 Hz tick, some dropped), 29 % flips of +-22 deg."""

    def __init__(self, seed=1, flip_rate=0.29, correction=MOUNT_ERR):
        self.rng = random.Random(seed)
        self.flip_rate, self.correction = flip_rate, correction
        self.k = 0
        self.last = None
        self.flips = 0

    def __call__(self, x, y, th, t):
        self.k += 1
        if self.k % 5 or x < 0.05:
            return self.last if self.last is not None and t - self.last.stamp < 0.3 else None
        r = self.rng
        flip = 0.0
        if r.random() < self.flip_rate:
            flip = math.radians(22.0) * (1 if r.random() < 0.5 else -1)
            self.flips += 1
        self.last = detection(x, y, th, t, flip=flip,
                              hdg_noise=math.radians(r.gauss(0.0, 3.9)),
                              pos_noise=(r.gauss(0.0, 0.02), r.gauss(0.0, 0.02)),
                              correction=self.correction)
        return self.last


def _run(x0, y0, th0, stream, seconds=80.0, **kw):
    kw.setdefault('skip_nav_to_approach', True)
    kw.setdefault('search_min_detections', 3)
    p = dl.DockParams(**kw)
    m = dl.DockStateMachine(p, dl.Pose2D(), goal_timeout_s=400.0)
    x, y, th, t, dt = x0, y0, th0, 0.0, 0.05
    m.start(dl.Snapshot(t=0.0))
    deb = dl.ContactDebouncer(3)
    log = []
    out = None
    for _ in range(int(seconds / dt)):
        t += dt
        deb.update(x <= 0.0 and abs(y) < 0.04)
        mk = stream(x, y, th, t)
        out = m.step(dl.Snapshot(t=t, odom=dl.Pose2D(x, y, th), marker=mk, contact=deb.value,
                                 charging_result=True if m.state == S.CHARGING else None,
                                 progress_pose=dl.Pose2D(x, y, th)))
        est = m._kf.x
        log.append((t, m.state, x, y, th, out.linear, out.angular,
                    est.heading if est is not None else None))
        if out.done:
            break
        x += out.linear * math.cos(th) * dt
        y += out.linear * math.sin(th) * dt
        th += out.angular * dt
        x = max(x, 0.0)
    return out, m, log


def _sign_flips_per_s(log):
    rows = [r for r in log if r[1] == S.DOCKING]
    flips, prev = 0, 0
    for r in rows:
        w = r[6]
        sg = 0 if abs(w) < 0.05 else (1 if w > 0 else -1)
        if sg and prev and sg != prev:
            flips += 1
        if sg:
            prev = sg
    dur = max(1e-6, len(rows) * 0.05)
    return flips / dur, dur


@pytest.mark.parametrize('seed', [1, 2, 3, 4, 5])
@pytest.mark.parametrize('x0,y0,thd', [(1.1, 0.10, 0.0), (1.0, -0.15, 8.0), (0.9, 0.05, -10.0)])
def test_flips_and_bias_heading_estimate_stays_within_5_deg(seed, x0, y0, thd):
    stream = FieldStream(seed)
    # the filter heading on one approach (a near-field realign re-acquires with a fresh filter)
    out, m, log = _run(x0, y0, math.radians(thd), stream, near_check_remaining=0.0)
    assert stream.flips > 3
    errs = [abs(math.degrees(dl.wrap_angle(r[7] - r[4]))) for r in log
            if r[1] == S.DOCKING and r[7] is not None]
    assert errs and max(errs) < 5.0, max(errs)
    assert out.success, (out.message, m.state)


@pytest.mark.parametrize('seed', [1, 2, 3, 4, 5])
def test_no_sign_flip_oscillation_or_heading_swing(seed):
    """Night: w changed sign 0.9-1.8 times per second and the heading swung 0 -> 40 deg."""
    out, m, log = _run(1.1, 0.12, 0.0, FieldStream(seed))
    rate, dur = _sign_flips_per_s(log)
    assert dur > 3.0
    assert rate < 0.4, rate
    assert max(abs(math.degrees(r[4])) for r in log if r[1] == S.DOCKING) < 16.0
    assert out.success and m._realigns == 0, (out.message, m._realigns)


def test_old_settings_reproduce_the_over_steer():
    """Control experiment: k 7 / 28.6 deg, clamped KF per tick, raw marker lateral, no camera
    correction -> the oscillation the field showed."""
    g = dl.ControllerGains(k_lateral=7.0, max_approach_angle=0.5, max_angular_accel=0.0)
    out, m, log = _run(1.1, 0.12, 0.0, FieldStream(1, correction=0.0), gains=g,
                       marker_position_only=False, kf_gate_chi2=0.0, kf_q=0.01, kf_r=0.1,
                       search_min_detections=1, pose_check_distance=0.0)
    rate, _ = _sign_flips_per_s(log)
    swing = max(abs(math.degrees(r[4])) for r in log if r[1] == S.DOCKING)
    assert rate > 0.5 or swing > 20.0, (rate, swing)


def test_wheels_stay_at_or_above_the_floor_in_docking():
    out, m, log = _run(1.1, 0.12, 0.0, FieldStream(2))
    for r in log:
        v, w = r[5], r[6]
        if r[1] != S.DOCKING or abs(v) < 1e-9:
            continue
        for wheel in (v - w * HALF, v + w * HALF):
            assert not (wheel * v < 0 and abs(wheel) < FLOOR - 1e-6), (v, w)


def test_first_detection_disagreeing_with_rtk_is_rejected():
    """The night's first solve was -167 deg while RTK said 11 deg."""
    p = dl.DockParams(skip_nav_to_approach=True, search_min_detections=2, lateral_align=False)
    m = dl.DockStateMachine(p, dl.Pose2D(), dock_pose_measured=True)
    pose = dl.Pose2D(0.8, 0.0, math.radians(11))
    m.start(dl.Snapshot(t=0.0, map_pose=pose, rtk_fixed=True))
    bad = dl.MarkerObs(-2.0, 0.4, math.radians(167), stamp=0.0)
    t = 0.0
    for i in range(10):
        t += 0.25
        bad = dl.MarkerObs(bad.x, bad.y, bad.yaw, stamp=t)
        out = m.step(dl.Snapshot(t=t, odom=pose, map_pose=pose, rtk_fixed=True, marker=bad))
        assert out.state == S.SEARCHING
    good = detection(0.8, 0.0, math.radians(11), t)
    for i in range(3):
        t += 0.25
        good = dl.MarkerObs(good.x, good.y, good.yaw, stamp=t)
        out = m.step(dl.Snapshot(t=t, odom=pose, map_pose=pose, rtk_fixed=True, marker=good))
    assert out.state == S.DOCKING
    assert m.retries == 0


def test_a_single_detection_and_an_inconsistent_pair_are_not_accepted():
    p = dl.DockParams(skip_nav_to_approach=True, search_min_detections=2)
    m = dl.DockStateMachine(p, dl.Pose2D())
    pose = dl.Pose2D(0.8, 0.05, 0.0)
    m.start(dl.Snapshot(t=0.0))
    a = detection(0.8, 0.05, 0.0, 0.25)
    out = m.step(dl.Snapshot(t=0.25, odom=pose, marker=a))
    assert out.state == S.SEARCHING                   # one detection is not enough
    b = detection(0.8, 0.05, 0.0, 0.5, flip=math.radians(22))
    out = m.step(dl.Snapshot(t=0.5, odom=pose, marker=b))
    assert out.state == S.SEARCHING                   # a flip pair disagrees
    c = detection(0.8, 0.05, 0.0, 0.75)
    out = m.step(dl.Snapshot(t=0.75, odom=pose, marker=c))
    d = detection(0.8, 0.05, 0.0, 1.0)
    out = m.step(dl.Snapshot(t=1.0, odom=pose, marker=d))
    assert out.state == S.DOCKING


def test_filter_gates_a_flip_instead_of_passing_half_of_it():
    f = dl.MarkerFilter(q=0.01, r=0.01, gate_chi2=9.21, q_heading=0.0005, r_heading=0.006)
    f.init(dl.DockErrors(1.0, 0.1, 0.0))
    for k in range(20):
        f.predict(dl.Pose2D(0.0, 0.0, 0.0), 0.05 * k)
    f.correct(dl.DockErrors(1.0, 0.1, math.radians(22)))
    assert abs(math.degrees(f.x.heading)) < 0.5 and f.rejected_heading == 1
    # per-second process noise: 20 ticks of 0.05 s add q * 1 s, not 20 q
    assert f.P[0] < 0.01


def test_filter_reinitialises_on_consistent_rejections():
    f = dl.MarkerFilter(q=0.01, r=0.01, gate_chi2=9.21, q_heading=0.0005, r_heading=0.006,
                        reinit_after=8)
    f.init(dl.DockErrors(1.0, 0.1, math.radians(25)))     # a wrong (flipped) start
    for _ in range(6):
        f.correct(dl.DockErrors(1.0, 0.1, 0.0))
    assert abs(math.degrees(f.x.heading)) < 1.0


# ------------------------------------------------------------- forward moves stay in corridor
from test_dock_stock_port import World, params  # noqa: E402


def test_realign_and_lateral_align_stay_inside_the_corridor():
    """08:30:42: realign 2/2 forward (0.58 -> 1.21 m) then LATERAL_ALIGN (-> 1.40 m) tripped
    the Dock path polygon. Force the pose check to fail (marker heading +20 deg) twice."""
    m = dl.DockStateMachine(params(final_max_heading_deg=12.0, marker_position_only=False,
                                   rtk_check_max_deg=0.0, rtk_check_max_m=0.0),
                            dl.Pose2D(), dock_pose_measured=True)
    w = World(m, 0.8, 0.25, 0.0)
    real_marker = w.marker
    w.marker = lambda: (lambda mk: None if mk is None else dl.MarkerObs(
        mk.x, mk.y, mk.yaw - math.radians(20), mk.stamp))(real_marker())
    xs = []
    w.run(120, until=lambda o: (xs.append(w.x), m._realigns >= 2 and o.state == S.DOCKING)[1])
    assert m._realigns == 2
    assert max(xs) <= 0.8 + 0.25 + 0.03, max(xs)
    states = [o.state for o in w.outs]
    # no LATERAL_ALIGN after a realign
    first_realign = next(i for i, o in enumerate(w.outs) if o.state == S.RETRY)
    assert S.LATERAL_ALIGN not in states[first_realign:]


def test_lateral_align_bumper_hit_ends_the_prealignment():
    m = dl.DockStateMachine(params(), dl.Pose2D(), dock_pose_measured=True)
    # 2026-10-23: start inside the approach distance (the forward cap is 0.90 m now)
    w = World(m, 0.6, 0.30, 0.0)
    w.visible = False
    w.run(1.0)
    assert m.state == S.LATERAL_ALIGN
    bump = {'on': True}
    snap0 = w.snap
    import dataclasses
    w.snap = lambda: dataclasses.replace(snap0(), bumper=bump['on'])
    w.run(0.5)
    bump['on'] = False
    w.run(4.0)
    assert m._lat_done and m.state != S.LATERAL_ALIGN
    n0 = len(w.outs)
    w.run(5.0)
    assert S.LATERAL_ALIGN not in [o.state for o in w.outs[n0:]]


def test_teleop_pauses_docking_and_a_joystick_dock_still_succeeds():
    import dataclasses
    m = dl.DockStateMachine(params(lateral_align=False), dl.Pose2D())
    w = World(m, 1.0, 0.0, 0.0)
    w.run(3.0, until=lambda o: o.state == S.DOCKING)
    joy = {'on': True}
    snap0 = w.snap
    w.snap = lambda: dataclasses.replace(snap0(), teleop=joy['on'])
    out = w.run(0.2)
    assert out.linear == 0.0 and out.angular == 0.0
    assert m._teleop is not None
    # the operator reverses onto the dock himself: the FSM output stays zero, contact wins
    for _ in range(400):
        w.t += 0.05
        w.x = max(0.0, w.x - 0.1 * 0.05)
        w.deb.update(w.x <= 0.0)
        out = w.apply(m.step(w.snap()))
        assert out.linear == 0.0 or out.done
        if out.done:
            break
    assert out.success


def test_teleop_release_resumes_from_the_current_pose():
    import dataclasses
    m = dl.DockStateMachine(params(lateral_align=False), dl.Pose2D())
    w = World(m, 1.0, 0.0, 0.0)
    w.run(3.0, until=lambda o: o.state == S.DOCKING)
    joy = {'on': True}
    snap0 = w.snap
    w.snap = lambda: dataclasses.replace(snap0(), teleop=joy['on'])
    w.run(1.0)
    joy['on'] = False
    out = w.run(120.0)
    assert out.success and m.retries == 0
