"""2026-11-03 slope sim: coverage pivots on a side slope slid 0.42-0.72 m downhill. A pivot
with |roll| > pivot_creep_roll_deg first faces upslope and drives up by the expected creep."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_mission import mission_fsm as f                       # noqa: E402
from test_mission_fsm import plan, line, square, start_until_planning  # noqa: E402
from test_path_conditioning import ring                          # noqa: E402
from keep_harness import KeepHarness                             # noqa: E402


def _h(roll, yaw_off=140.0, **params):
    params.setdefault('path_conditioning', True)
    h = KeepHarness(**params)       # the upslope drive keeps the blade on inside the area
    h.areas = [square(-8, -4, 12)]
    r = ring()
    start_until_planning(h)
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h.fsm.inputs.pose = (r[0][0], r[0][1], y0 + math.radians(yaw_off))
    i = h.fsm.inputs
    i.tilt_band, i.tilt_roll_deg, i.tilt_pitch_deg, i.tilt_stamp = 'ok', roll, 0.0, h.t
    sp = [(q[0], q[1], math.atan2(r[min(k + 1, len(r) - 1)][1] - q[1],
                                  r[min(k + 1, len(r) - 1)][0] - q[0]))
          for k, q in enumerate(r)]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp, line(-4.0, 3.0, -1.0, 3.0)]))
    h.fsm.inputs.is_cutting = True
    i.tilt_stamp = h.t
    h.tick()
    return h, y0


def test_flat_pivot_is_unchanged():
    h, _ = _h(0.0)
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'


def test_side_slope_pivot_faces_upslope_drives_up_then_pivots():
    h, y0 = _h(-10.0)
    yaw = h.fsm.inputs.pose[2]
    assert h.pending_action(f.ACT_SPIN).purpose == 'nogo_step'
    # roll -10: left side down -> upslope = yaw - 90 deg, inside the -140 deg sweep
    assert abs(h.goal(f.ACT_SPIN)['angle'] + math.pi / 2) < 1e-6
    h.fsm.inputs.pose = (h.fsm.inputs.pose[0], h.fsm.inputs.pose[1], yaw - math.pi / 2)
    h.fsm.inputs.tilt_stamp = h.t
    h.finish(f.ACT_SPIN)
    h.pending_action(f.ACT_DRIVE)
    # the three blade gates agree on the slope pre-drive (val5: _blade_keep_ok listed it late)
    assert h.fsm.mission.step == 'slope_pivot'
    assert h.fsm._blade_keep_ok() and f.blade_allowed(h.fsm)
    assert h.fsm._keep_blade_inside('slope pre-drive') == h.fsm.blade_on
    # half the creep of the 140 deg pivot at the 0.6 rad/s mean Spin rate (2026-11-06; was
    # 0.3), 0.045 m/s, x1.3, capped at 0.2 m: 0.12 m
    t = math.radians(140) / 0.6
    assert abs(h.goal(f.ACT_DRIVE)['distance_m'] - min(0.2, 0.5 * 0.045 * t * 1.3)) < 0.02
    h.fsm.inputs.tilt_stamp = h.t
    h.finish(f.ACT_DRIVE)
    h.fsm.inputs.tilt_stamp = h.t
    h.tick(n=3)
    s = h.pending_action(f.ACT_SPIN)
    assert s is not None and s.purpose == 'cov_pivot'     # no second pre-drive
    assert h.fsm.mission.slope_pivot_key is None


def test_upslope_outside_the_sweep_adds_no_rotation():
    h, _ = _h(10.0)                 # upslope = yaw + 90, the pivot turns -140: not in it
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'


def test_below_threshold_or_off_pivots_as_planned():
    h, _ = _h(5.0)
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    h, _ = _h(10.0, pivot_creep_roll_deg=0.0)
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
