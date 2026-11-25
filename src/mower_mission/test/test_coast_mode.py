"""Coast mode (2026-11-25, docs/analysis/2026-11-25_coast_mode.md): degraded during a mow follow
or a transit keeps going on dead reckoning while the monitor's coast budget lasts and the path
ahead clears the boundary by est_drift + half width + coast_edge_margin_m; then the hold."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_body_boundary import feed, mowing_h   # noqa: E402
from test_loc_hold import held, transit   # noqa: E402
from test_loc_policy import big_mow, speed_calls   # noqa: E402
from test_mission_fsm import Harness   # noqa: E402
from test_rtk_dropout import LEG, logged, step, strip_mow   # noqa: E402

SINGLE = 'ekf_map sigma 0.50 m > 0.15 m (rtk single)'


def coast_loc(h, state='degraded', rtk='single', est=0.09, left=3.0, reason=SINGLE):
    i = h.fsm.inputs
    i.loc_state, i.loc_reason, i.loc_stamp, i.loc_rtk = state, reason, h.t, rtk
    i.loc_est_drift, i.loc_coast_left = est, left


def run(h, secs, dt=0.2, **kw):
    for _ in range(int(round(secs / dt))):
        coast_loc(h, **kw)
        h.tick(dt=dt)


def test_flat_dropout_coasts_through_without_stopping():
    h = big_mow(coast_enabled=True)
    m = h.mark()
    run(h, 15.0, left=3.0)                      # 15 s single, budget left
    assert h.fsm.mission.step == 'follow' and h.fsm._loc is None and h.blade
    assert h.fsm._loc_coast is not None
    assert not h.since(m, f.BladeOff) and not h.since(m, f.ZeroBurst)
    caps = speed_calls(h, m)
    assert caps and all(v == 0.15 for v in caps[-1].values())
    assert logged(h, m, 'coasting on dead reckoning')
    run(h, 3.0, state='ok', rtk='fixed', est=0.05, left=5.0, reason='')
    assert h.fsm._loc_coast is None and h.fsm._loc is None and h.blade
    assert logged(h, m, 'coast ended') and not h.fsm._loc_fix_pending()


def test_budget_spent_on_a_slope_holds_and_waits_for_fixed():
    h = big_mow(coast_enabled=True)
    run(h, 2.0, est=0.2, left=0.5)              # 1.5 m slope budget nearly used
    assert not held(h)
    m = h.mark()
    run(h, 0.4, est=0.25, left=0.0)
    assert held(h) and not h.blade and h.since(m, f.BladeOff)
    assert logged(h, m, 'coast ended') and logged(h, m, 'coast budget used up')
    run(h, 20.0, state='ok', rtk='float', est=0.25, left=0.0, reason='rtk float')
    assert held(h)                              # float does not resume after a stop
    run(h, 6.0, state='ok', rtk='fixed', est=0.05, left=5.0, reason='')
    h.tick()
    assert not held(h) and h.blade


def test_float_that_disagrees_holds_until_fixed_settles():
    h = big_mow(coast_enabled=True)
    run(h, 5.0)
    m = h.mark()
    run(h, 0.4, state='lost', rtk='float', est=0.1, left=0.0,
        reason='RTK position 0.31 m from dead reckoning (gate 0.25 m) (rtk float)')
    assert held(h) and not h.blade and h.fsm._loc_coast is None
    assert logged(h, m, 'from dead reckoning')
    run(h, 10.0, state='ok', rtk='float', est=0.1, left=0.0, reason='rtk float')
    assert held(h)
    run(h, 6.0, state='ok', rtk='fixed', est=0.05, left=5.0, reason='')
    assert not held(h)


def test_float_that_agrees_carries_on():
    h = big_mow(coast_enabled=True)
    run(h, 5.0)
    run(h, 10.0, state='ok', rtk='float', est=0.1, left=2.0, reason='rtk float')
    assert not held(h) and h.blade and h.fsm.mission.step == 'follow'


def test_the_nose_counts_at_a_row_end():
    """The strip leg ends 0.1 m from the east edge, heading at it, and the 5 m look-ahead
    reaches it: the nose (0.6 m ahead) is 0.5 m out there, 0.1 m past its 0.40 m soft limit,
    so even a small drift may not coast (a half-width disc round base_link saw 0.215 m of
    room: review 2026-11-25)."""
    h = strip_mow(coast_enabled=True)
    m = h.mark()
    coast_loc(h, left=5.0, est=0.02)
    h.tick()
    assert held(h) and not h.blade and logged(h, m, 'edge clearance -0.10 m')


def test_clearance_follows_the_drift_and_the_budget_horizon():
    """Same leg, a 1 m budget: the edge is beyond the horizon, so it coasts; a large drift
    estimate ends it."""
    h = strip_mow(coast_enabled=True)
    coast_loc(h, left=1.0)
    h.tick()
    assert not held(h) and h.fsm._loc_coast is not None
    m = h.mark()
    coast_loc(h, left=1.0, est=1.8)
    h.tick()
    assert held(h) and logged(h, m, 'edge clearance')


def test_transit_coasts():
    h = Harness(coast_enabled=True)
    transit(h)
    m = h.mark()
    run(h, 5.0, left=4.0)
    assert h.fsm.mission.step == 'transit' and h.fsm._loc is None
    assert logged(h, m, 'coasting on')


def test_coast_off_restores_the_swath_rule():
    h = big_mow(coast_enabled=False)
    m = h.mark()
    run(h, 1.0, est=0.4, left=3.0)
    assert h.fsm._loc_coast is None and logged(h, m, 'finishing the current swath')


def test_no_budget_field_holds():
    """An older monitor without coast_left_m: no coast."""
    h = big_mow(coast_enabled=True)
    run(h, 0.4, left=None)
    assert held(h)


def test_pivot_coasts():
    h = big_mow(coast_enabled=True)
    h.fsm.mission.step = 'corner_pivot'
    run(h, 2.0, left=3.0)
    assert h.fsm._loc is None and h.fsm._loc_coast is not None


# ---- the boundary guard while coasting ----------------------------------------------------------
def coast_strip(**params):
    """Coasting on the strip leg at (3, 2), wheels with the pose."""
    h = mowing_h(LEG, (1.0, 2.0, 0.0), **{"coast_enabled": True, **params})
    x = 1.0
    while x < 3.0 - 1e-9:
        x += 0.05
        step(h, (x, 2.0, 0.0), wheel=(x, 2.0))
    step(h, (3.0, 2.0, 0.0), wheel=(3.0, 2.0), n=70)
    for _ in range(60):                         # single for 6 s: past the class-change window
        feed(h, (3.0, 2.0, 0.0))
        coast_loc(h, left=1.0)
        h.fsm.inputs.wheel_xy = (3.0, 2.0)
        h.tick(dt=0.1)
    assert h.fsm._loc_coast is not None and not held(h)
    return h


def test_real_drive_out_while_coasting_trips():
    """The pose moves with the wheels: a real excursion (margin off so the coast's own
    clearance check does not stop it first)."""
    h = coast_strip(coast_edge_margin_m=-1.0)
    y = 2.0
    while h.fsm._hb is None and y > -1.0:
        y -= 0.1
        feed(h, (3.0, y, 0.0))
        coast_loc(h, left=1.0)
        h.fsm.inputs.wheel_xy = (3.0, y)
        h.tick(dt=0.1)
    assert h.fsm._hb is not None and h.fsm._hb_count == 1 and not h.blade


def test_drifting_toward_the_edge_holds_before_the_body_is_out():
    h = coast_strip()
    m = h.mark()
    y = 2.0
    while not held(h) and y > -1.0:
        y -= 0.05
        feed(h, (3.0, y, 0.0))
        coast_loc(h, left=1.0)
        h.fsm.inputs.wheel_xy = (3.0, y)
        h.tick(dt=0.1)
    # base_link 0.05 m inside: the body side 0.185 m out, under the 0.35 m soft limit
    assert held(h) and y > 0.0 and h.fsm._hb is None and logged(h, m, 'edge clearance')


def test_pose_jump_while_coasting_is_not_trusted():
    h = coast_strip()
    m = h.mark()
    for _ in range(3):                          # map pose 2.3 m south, wheels still
        feed(h, (3.0, -0.3, 0.0))
        coast_loc(h, left=1.0)
        h.fsm.inputs.wheel_xy = (3.0, 2.0)
        h.tick(dt=0.1)
    assert h.fsm._hb is None and h.fsm._hb_count == 0 and held(h) and not h.blade
    assert logged(h, m, 'pose not trusted')
