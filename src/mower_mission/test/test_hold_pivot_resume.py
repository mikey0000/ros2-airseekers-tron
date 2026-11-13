"""A hold that interrupts a coverage pivot redoes the rest of the pivot on resume.

Field 2026-11-04 (area 3, 23:42:29 and 23:53:12): pivot 135 deg in place -> localization
degraded hold -> ok again -> /follow_path sent with no spin; the robot, still facing out
past the row end, drove forward and tripped BOUNDARY OUT. ``_cov_pivot`` marks the pivot
done when it STARTS (pivot_done = key); the hold's resume re-dispatched into
``_cov_pivot``, saw the key and skipped the pivot."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_mission import mission_fsm as f             # noqa: E402
from test_blocked_pivot import ring_h                  # noqa: E402
from test_loc_hold import run, set_loc                 # noqa: E402
from test_mission_fsm import tick_seen                 # noqa: E402
from test_path_conditioning import ring                # noqa: E402


def _pivoting():
    """Robot at the ring start facing 140 deg off the first leg: a coverage pivot runs."""
    r = ring()
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h = ring_h(yaw=y0 + math.radians(140))
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    assert h.fsm.mission.step == 'corner_pivot'
    return h, y0


def _after(h, mk):
    """Names of the actions started since ``mk``, in order."""
    return [e.name for e in h.since(mk, f.StartAction)]


def _first_motion(h, mk):
    """(name, purpose) of the first Spin / follow started since ``mk``."""
    names = [n for n in _after(h, mk) if n in (f.ACT_SPIN, f.ACT_FOLLOW)]
    if not names:
        return None
    a = h.fsm._action
    return names[0], (a.purpose if a is not None and a.name == names[0] else None)


def test_loc_hold_mid_pivot_redoes_the_pivot_on_resume():
    h, y0 = _pivoting()
    x, y, _ = h.fsm.inputs.pose
    h.fsm.inputs.pose = (x, y, y0 + math.radians(80))   # 60 of 140 deg turned when it hit
    set_loc(h, 'degraded')
    h.tick()
    assert h.fsm.mission.step == 'loc_hold' and h.fsm._action is None
    mk = h.mark()
    run(h, 'ok', 5.5)                    # RTK fixed settled (loc_fixed_settle_s 5 s)
    h.tick(n=3)                          # blade spin-up (is_cutting) -> _follow
    assert _first_motion(h, mk) == (f.ACT_SPIN, 'cov_pivot'), _after(h, mk)
    spin = h.goal(f.ACT_SPIN)
    assert abs(abs(spin['angle']) - math.radians(80)) < math.radians(15), spin
    assert h.fsm.mission.step == 'corner_pivot'
    h.fsm.inputs.pose = (x, y, y0)       # the pivot completes: then the leg is followed
    h.finish(f.ACT_SPIN)
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW) is not None
    assert h.fsm.mission.step == 'follow'


def test_dynamic_hold_mid_pivot_redoes_the_pivot_on_resume():
    h, y0 = _pivoting()
    tick_seen(h, 'dynamic', 'person', 1.0)
    assert h.fsm._dyn is not None and h.fsm.mission.step == 'dyn_wait'
    mk = h.mark()
    h.tick(dt=0.5, n=12)                 # policy stale: clear hold over, resumes
    assert h.fsm._dyn is None
    assert _first_motion(h, mk) == (f.ACT_SPIN, 'cov_pivot'), _after(h, mk)


def test_hold_resume_after_a_finished_pivot_does_not_pivot_again():
    """The pivot ran to its end before the hold (robot on the leg heading): nothing left,
    the resume follows the leg."""
    h, y0 = _pivoting()
    x, y, _ = h.fsm.inputs.pose
    h.fsm.inputs.pose = (x, y, y0 + math.radians(2))
    set_loc(h, 'degraded')
    h.tick()
    mk = h.mark()
    run(h, 'ok', 5.5)
    h.tick(n=3)
    assert _first_motion(h, mk)[0] == f.ACT_FOLLOW, _after(h, mk)
