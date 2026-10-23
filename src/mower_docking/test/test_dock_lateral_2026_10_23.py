"""2026-10-23 dock lateral offset (findings doc, "Lateral offset at the contacts").

All four approaches of the 00:45-00:47 UTC dock goal (df07940) ended 4-14 cm left of the dock
axis by RTK, and the marker filter agreed (+4.5..+9.6 cm when it went blind): the robot knew
it was off and did not fix it. The filter heading read +4.8 deg against RTK near the dock,
because ``rear_camera_yaw_correction_deg`` -6.5 rotated every detection. The reverse
controller drives the filter heading to h_d = k_lateral * lateral, so a heading bias b holds
the lateral at b / k_lateral (0.084 rad / 2.5 = 3.4 cm, 0.113 / 2.5 = 4.5 cm).

``data/field_2026_10_23_dock_markers.csv``: the day's 441 published detections (correction
-6.5 applied) with the RTK pose in the stored dock frame.
"""
import csv
import math
import os
import random
import statistics

import pytest
import yaml

from mower_docking import dock_logic as dl
from test_dock_field_2026_10_21 import detection

S = dl.DockState
CAM = (-0.201, 0.0)
OFF = 0.2168                       # the mower's calibrated docked_marker_offset
OLD = math.radians(-6.5)
HERE = os.path.dirname(__file__)


def _rows():
    with open(os.path.join(HERE, 'data', 'field_2026_10_23_dock_markers.csv')) as f:
        lines = [ln for ln in f if not ln.startswith('#')]
    out = []
    for r in csv.DictReader(lines):
        if r['state'] not in ('SEARCHING', 'DOCKING', 'FINAL_DOCKING'):
            continue
        out.append({k: (r[k] if k in ('time', 'state') else float(r[k])) for k in r})
    return out


def _recorrect(r, corr):
    """The detection as the node would publish it with ``corr`` instead of -6.5 deg."""
    d = corr - OLD
    c, s = math.cos(d), math.sin(d)
    dx, dy = r['mx'] - CAM[0], r['my'] - CAM[1]
    return dl.MarkerObs(CAM[0] + c * dx - s * dy, CAM[1] + s * dx + c * dy,
                        math.radians(r['myaw_deg']) + d)


def _inliers(corr, lo, hi):
    out = []
    for r in _rows():
        if not lo <= r['rtk_rem'] < hi:
            continue
        m = _recorrect(r, corr)
        h = math.radians(r['rtk_hdg_deg'])
        e = dl.dock_errors(m, OFF)
        if abs(math.degrees(dl.wrap_angle(e.heading - h))) > 15.0:      # IPPE flips
            continue
        pos = dl.dock_errors_with_heading(m, h, OFF)
        out.append((math.degrees(dl.wrap_angle(e.heading - h)), pos.lateral - r['rtk_lat']))
    return out


def test_yaml_correction_is_zero():
    with open(os.path.join(HERE, '..', 'config', 'docking.yaml')) as f:
        y = next(iter(yaml.safe_load(f).values()))['ros__parameters']
    assert y['rear_camera_yaw_correction_deg'] == 0.0


@pytest.mark.parametrize('lo,hi', [(0.0, 0.3), (0.3, 0.6), (0.6, 1.0), (1.0, 2.0)])
def test_replay_marker_centre_matches_rtk_without_the_correction(lo, hi):
    """Marker centre + RTK heading -> lateral: within 1.5 cm of RTK at every range with 0, and
    3-14 cm off with -6.5 (the rotation about the camera moves the centre by range x 6.5 deg)."""
    new = _inliers(0.0, lo, hi)
    old = _inliers(OLD, lo, hi)
    assert len(new) >= 10
    assert abs(statistics.median(d for _, d in new)) < 0.015
    assert statistics.median(d for _, d in old) < -0.03


def test_replay_heading_bias_near_the_dock():
    """Within 0.3 m of the contacts the corrected heading read +4.8 deg against RTK; without
    the correction it is -1.7 deg (the marker normal's own bias)."""
    old = statistics.median(h for h, _ in _inliers(OLD, 0.0, 0.3))
    new = statistics.median(h for h, _ in _inliers(0.0, 0.0, 0.3))
    assert 3.5 < old < 6.5
    assert abs(new) < 2.5


# --- closed loop with the day's measured error model ---------------------------------------

def normal_bias_deg(rem):
    """Marker-normal heading bias against RTK by range (2026-10-23, no camera correction)."""
    for top, b in ((0.15, -1.3), (0.30, -2.0), (0.60, -1.6), (1.00, -3.7)):
        if rem < top:
            return b
    return -7.1


class DayStream:
    """~5 Hz detections from a camera with NO yaw error, converted with ``correction``; the
    normal carries the day's range-dependent bias + 3.9 deg noise, the centre 1 cm noise,
    10 % IPPE flips; the marker leaves the frame 0.08 m before the contacts (field: 0.07-0.10)."""

    def __init__(self, correction, seed=1):
        self.corr, self.rng = correction, random.Random(seed)
        self.k, self.last = 0, None

    def __call__(self, x, y, th, t):
        self.k += 1
        if x < 0.08:
            return None
        if self.k % 4:
            return self.last if self.last is not None and t - self.last.stamp < 0.3 else None
        r = self.rng
        flip = math.radians(22.0) * r.choice((-1, 1)) if r.random() < 0.1 else 0.0
        self.last = detection(x, y, th, t, flip=flip, mount=0.0, correction=self.corr,
                              hdg_noise=math.radians(normal_bias_deg(x) + r.gauss(0.0, 3.9)),
                              pos_noise=(r.gauss(0.0, 0.01), r.gauss(0.0, 0.01)))
        return self.last


def _final_lateral(x0, y0, thd, correction, seed=1, seconds=60.0):
    """Lateral (m) when the robot reaches the contact line (x <= 0), or None."""
    import test_dock_field_2026_10_21 as f
    old_off, f.OFF = f.OFF, OFF
    try:
        p = dl.DockParams(skip_nav_to_approach=True, docked_marker_offset=OFF)
        m = dl.DockStateMachine(p, dl.Pose2D(), goal_timeout_s=400.0)
        stream = DayStream(correction, seed)
        x, y, th, t, dt = x0, y0, math.radians(thd), 0.0, 0.05
        m.start(dl.Snapshot(t=0.0))
        for _ in range(int(seconds / dt)):
            t += dt
            out = m.step(dl.Snapshot(t=t, odom=dl.Pose2D(x, y, th), marker=stream(x, y, th, t),
                                     progress_pose=dl.Pose2D(x, y, th)))
            if out.done:
                return None
            x += out.linear * math.cos(th) * dt
            y += out.linear * math.sin(th) * dt
            th += out.angular * dt
            if x <= 0.0:
                return y
        return None
    finally:
        f.OFF = old_off


# the day's four DOCKING starts by RTK (remaining, lateral, heading deg)
STARTS = [(0.73, -0.03, -24.0), (1.04, 0.10, -3.0), (0.31, 0.04, -1.0), (0.76, 0.18, -3.0)]


SEEDS = (1, 2, 3, 4)


def _all(correction):
    return [_final_lateral(x0, y0, thd, correction, s) for x0, y0, thd in STARTS for s in SEEDS]


def test_closed_loop_lands_on_the_axis():
    """No correction: every run reaches the contact line, median |lateral| <= 2 cm, none over
    4 cm. The residual is the marker normal's own range bias (-1.3..-7 deg) over k_lateral 2.5;
    without that bias the same runs end within 1.5 cm (k_lateral convergence)."""
    lats = _all(0.0)
    assert None not in lats
    assert statistics.median(abs(v) for v in lats) <= 0.02, lats
    assert max(abs(v) for v in lats) < 0.04, lats


def test_closed_loop_old_correction_reproduces_the_offset():
    """Control experiment: -6.5 deg on a camera without that error -> ~5 cm to the left, as
    the field (RTK +4..+6 cm at the contact line on all four approaches)."""
    lats = [v for v in _all(OLD) if v is not None]
    assert statistics.median(lats) > 0.04, lats


def test_closed_loop_without_the_normal_bias_is_exact(monkeypatch):
    import test_dock_lateral_2026_10_23 as me
    monkeypatch.setattr(me, 'normal_bias_deg', lambda rem: 0.0)
    lats = [_final_lateral(x0, y0, thd, 0.0, 1) for x0, y0, thd in STARTS]
    assert max(abs(v) for v in lats) < 0.015, lats
