"""2026-10-31 night dock: the SEARCH filter heading is seeded from RTK, not the marker yaw.

Field (bag 20261031-015604, chunks 304-307): the scene was black, only the dock's lit marker
was visible and its yaw swung +-20 deg per frame. Each SEARCH seeded the filter from the
median marker yaw (-13.5, -7.5, -13.3, -21.2 deg against RTK -3..+9 deg), so every approach
started 0.1-0.2 m off in lateral: 2 pose realigns, 2 near realigns, a missed contact and a
retry, 77 s for a dock that took 27 s in daylight.

With the dock pose measured, RTK fixed and a trusted heading-aligner yaw, the filter now
starts on the RTK heading and the marker centres (search_seed_rtk_heading). Otherwise it
falls back to the marker median.
"""
import math
import random

import pytest

from mower_docking import dock_logic as dl
from test_dock_field_2026_10_21 import OFF, detection

S = dl.DockState
FLIP = math.radians(20.0)


def _det(x, y, th, t, flip=0.0, noise=(0.0, 0.0, 0.0)):
    """Detection by the ideal camera (the shipped rear_camera_yaw_correction_deg is 0)."""
    return detection(x, y, th, t, flip=flip, hdg_noise=noise[0], pos_noise=noise[1:],
                     mount=0.0, correction=0.0)


def test_rehead_errors_recovers_the_pose_from_a_flipped_detection():
    for x, y, th, f in ((1.0, 0.1, 0.05, FLIP), (0.8, -0.15, -0.1, -FLIP), (0.6, 0.0, 0.0, 0.0)):
        raw = dl.dock_errors(_det(x, y, th, 0.0, flip=f), OFF)
        if f:
            assert abs(raw.lateral - y) > 0.1          # the flip moved the yaw-derived lateral
        e = dl.rehead_errors(raw, th, OFF)
        assert (e.remaining, e.lateral, e.heading) == pytest.approx((x, y, th), abs=1e-6)


def test_rehead_errors_onto_its_own_heading_is_identity():
    e = dl.DockErrors(0.9, 0.12, 0.2)
    r = dl.rehead_errors(e, 0.2, OFF)
    assert (r.remaining, r.lateral, r.heading) == pytest.approx((0.9, 0.12, 0.2), abs=1e-9)


@pytest.mark.parametrize('text,trusted', [
    ('{"aligned": true, "source": "cog", "quality": "good"}', True),
    ('{"aligned": true, "source": "cog", "quality": "fair"}', False),
    ('{"aligned": false, "source": "file", "quality": "low"}', False),
    ('{"aligned": true}', False),
    ('not json', False),
    ('[1, 2]', False),
])
def test_heading_status_trusted(text, trusted):
    assert dl.heading_status_trusted(text, ['good']) is trusted


class DuskStream:
    """~4 Hz detections of a lit marker in a black scene: on ``p_flip`` of the frames the solve
    reports the IPPE mirror ``flip_deg`` off (on the minus side with ``p_minus_side``, as the
    night's seeds were). Heading noise 2 deg, position noise 1 cm."""

    def __init__(self, seed=1, p_flip=0.7, p_minus_side=0.8, flip_deg=14.0):
        self.rng = random.Random(seed)
        self.flip = math.radians(flip_deg)
        self.p_flip, self.p_minus_side = p_flip, p_minus_side
        self.k = 0
        self.last = None

    def __call__(self, x, y, th, t):
        self.k += 1
        if self.k % 5 or x < 0.05:
            return self.last if self.last is not None and t - self.last.stamp < 0.3 else None
        r = self.rng
        mirror = -self.flip if r.random() < self.p_minus_side else self.flip
        flip = mirror if r.random() < self.p_flip else 0.0
        self.last = _det(x, y, th, t, flip=flip,
                         noise=(math.radians(r.gauss(0.0, 2.0)), r.gauss(0.0, 0.01),
                                r.gauss(0.0, 0.01)))
        return self.last


def _machine(measured=True, **kw):
    kw.setdefault('skip_nav_to_approach', True)
    kw.setdefault('search_min_detections', 3)
    return dl.DockStateMachine(dl.DockParams(**kw), dl.Pose2D(), goal_timeout_s=400.0,
                               dock_pose_measured=measured)


def _search(m, stream, x, y, th, rtk_fixed=True, trusted=True, seconds=20.0):
    """Hold the robot still at (x, y, th) (dock frame = map frame) until SEARCH accepts.
    Returns the machine and its log notes."""
    t = 0.0
    notes = list(m.start(dl.Snapshot(t=0.0)).notes)
    pose = dl.Pose2D(x, y, th)
    while m.state == S.SEARCHING and t < seconds:
        t += 0.05
        notes += m.step(dl.Snapshot(t=t, odom=pose, map_pose=pose, progress_pose=pose,
                           marker=stream(x, y, th, t), rtk_fixed=rtk_fixed,
                           heading_trusted=trusted)).notes
    return m, notes


# the night's seeds: a 14 deg mirror on 70 % of the frames, 80 % of them on the minus side:
# SEARCH accepts a flipped cluster that still passes the RTK cross-check (30 deg / 0.3 m)
NIGHT = dict(p_flip=0.7, p_minus_side=0.8, flip_deg=14.0)
X, Y, TH = 0.6, 0.05, math.radians(4.0)


@pytest.mark.parametrize('seed', range(1, 9))
def test_flipping_marker_yaw_seeds_the_filter_on_rtk(seed):
    m, notes = _search(_machine(), DuskStream(seed, **NIGHT), X, Y, TH)
    assert m.state == S.DOCKING
    h = m._kf.x
    assert abs(math.degrees(dl.wrap_angle(h.heading - TH))) < 1.0
    assert abs(h.lateral - Y) < 0.02
    assert any('seeded from RTK' in n for n in notes)


def test_marker_seed_control_reproduces_the_night():
    """search_seed_rtk_heading false (master): the flipped cluster becomes the filter start,
    12-16 deg and 0.2-0.3 m lateral off (field: 7.5-21 deg, 0.1-0.2 m)."""
    bad = 0
    for seed in range(1, 9):
        m, _ = _search(_machine(search_seed_rtk_heading=False), DuskStream(seed, **NIGHT),
                       X, Y, TH)
        assert m.state == S.DOCKING
        h = m._kf.x
        bad += abs(math.degrees(h.heading - TH)) > 10.0 and abs(h.lateral - Y) > 0.15
    assert bad >= 4


def test_alternating_flips_with_rtk_steady_seed_on_rtk():
    """The marker yaw alternates +-20 deg about the truth while RTK stays steady. The shipped
    SEARCH acceptance never takes such a stream (consecutive detections disagree by 40 deg and
    0.4 m lateral); with the acceptance widened, the seed still lands on RTK and the marker
    centres, not between the two flips."""
    class Alternating:
        k = 0

        def __call__(self, x, y, th, t):
            self.k += 1
            return _det(x, y, th, t, flip=FLIP if self.k % 2 else -FLIP)
    m, _ = _search(_machine(), Alternating(), X, Y, TH, seconds=10.0)
    assert m.state == S.SEARCHING
    m, notes = _search(_machine(search_consistency_deg=45.0, search_consistency_m=1.0,
                                rtk_check_max_m=0.6), Alternating(), X, Y, TH)
    assert m.state == S.DOCKING
    assert abs(math.degrees(dl.wrap_angle(m._kf.x.heading - TH))) < 1.0
    assert abs(m._kf.x.lateral - Y) < 0.01
    assert any('seeded from RTK' in n for n in notes)


@pytest.mark.parametrize('measured,rtk_fixed,trusted', [
    (True, False, True), (False, True, True), (True, True, False)])
def test_falls_back_to_the_marker_seed(measured, rtk_fixed, trusted):
    """No measured dock pose, no RTK fix or no trusted heading: exactly the master seed."""
    a, notes = _search(_machine(measured), DuskStream(3, **NIGHT), X, Y, TH, rtk_fixed, trusted)
    b, _ = _search(_machine(measured, search_seed_rtk_heading=False), DuskStream(3, **NIGHT),
                   X, Y, TH, rtk_fixed, trusted)
    assert a.state == b.state == S.DOCKING
    assert (a._kf.x.remaining, a._kf.x.lateral, a._kf.x.heading) == \
        pytest.approx((b._kf.x.remaining, b._kf.x.lateral, b._kf.x.heading), abs=1e-12)
    assert not any('seeded from RTK' in n for n in notes)


def _dock(stream, x0, y0, th0, seconds=150.0, **kw):
    """Closed loop from (x0, y0, th0) with RTK = truth; returns (output, machine, seconds)."""
    m = _machine(**kw)
    x, y, th, t, dt = x0, y0, th0, 0.0, 0.05
    m.start(dl.Snapshot(t=0.0))
    deb = dl.ContactDebouncer(3)
    out = None
    while t < seconds:
        t += dt
        deb.update(x <= 0.0 and abs(y) < 0.045)
        pose = dl.Pose2D(x, y, th)
        out = m.step(dl.Snapshot(t=t, odom=pose, map_pose=pose, progress_pose=pose,
                                 marker=stream(x, y, th, t), contact=deb.value,
                                 charging_result=True if m.state == S.CHARGING else None,
                                 rtk_fixed=True, heading_trusted=True))
        if out.done:
            break
        x += out.linear * math.cos(th) * dt
        y += out.linear * math.sin(th) * dt
        th += out.angular * dt
        x = max(x, 0.0)
    return out, m, t


def test_night_closed_loop_rtk_seed_spends_fewer_approaches():
    """Closed loop on the night stream, seeds 1-10 from 0.8 m / 6 cm / 3 deg (2026-11-04): master
    46 s mean, 4 pose + 20 near realigns; RTK seed 35 s, 1 + 16. The near check's heading is the
    raw marker median, which the 70 % flips still trip; the seed fixes the start, not that."""
    def spent(**kw):
        total = 0
        for seed in range(1, 11):
            out, m, _ = _dock(DuskStream(seed, **NIGHT), 0.8, 0.06, math.radians(3.0), **kw)
            assert out.success, (out.message, seed)
            total += m.retries + m._realigns + m._near_realigns
        return total
    assert spent() + 5 <= spent(search_seed_rtk_heading=False)
