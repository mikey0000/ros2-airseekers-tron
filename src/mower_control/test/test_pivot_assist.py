"""cmd_vel_slew pivot assist: unit logic + a small kinematic sim of an RPP-style
rotate-to-heading on the Tron's turf (in-place pivots barely turn; arcs do)."""

import math

import pytest

from mower_control.cmd_vel_slew import PivotAssist

DT = 0.05


def test_only_after_delay_in_motion_phase_and_capped():
    pa = PivotAssist()
    outs = [pa.apply(0.0, 0.3, 'TRANSIT', DT) for _ in range(int(20 / DT))]
    assert all(o == 0.0 for o in outs[:int(1.5 / DT) - 1])
    assert outs[int(1.6 / DT)] == pytest.approx(0.05)
    assert pa.dist_m == pytest.approx(0.3, abs=0.01)
    assert sum(o * DT for o in outs) == pytest.approx(0.3, abs=0.01)
    assert outs[-1] == 0.0


@pytest.mark.parametrize('lin,ang,phase', [
    (0.0, 0.3, 'IDLE'), (0.0, 0.3, 'RETURNING_HOME'), (0.0, 0.3, None),
    (0.0, 0.15, 'MOWING'), (0.1, 0.3, 'MOWING'), (-0.1, 0.3, 'TRANSIT')])
def test_never_outside_a_pure_pivot_in_transit_or_mowing(lin, ang, phase):
    pa = PivotAssist()
    assert all(pa.apply(lin, ang, phase, DT) == lin for _ in range(100))


def test_new_pivot_resets_budget_and_disabled_is_neutral():
    pa = PivotAssist()
    for _ in range(200):
        pa.apply(0.0, 0.3, 'MOWING', DT)
    pa.apply(0.2, 0.0, 'MOWING', DT)                      # pivot over
    outs = [pa.apply(0.0, -0.3, 'MOWING', DT) for _ in range(60)]
    assert outs[-1] == pytest.approx(0.05)
    off = PivotAssist(enabled=False)
    assert all(off.apply(0.0, 0.3, 'MOWING', DT) == 0.0 for _ in range(100))


# ---------------------------------------------------------------- kinematic sim
def turf_yaw_rate(v, w):
    """Measured on turf (2026-10-06): in place ~0.05 rad/s of 0.3; arcs ~0.2 of 0.3."""
    return w * (0.17 if abs(v) < 0.02 else 0.7)


def rpp_rotate_cmd(heading_err):
    """Humble RPP rotate_to_heading: linear 0, angular +-0.3 while |err| > min angle."""
    if abs(heading_err) > 0.785:
        return 0.0, math.copysign(0.3, heading_err)
    return 0.3, 0.0           # handed back to pure pursuit (end of the pivot)


def simulate(assist, target=math.radians(150), t_max=120.0):
    x = y = yaw = 0.0
    t = 0.0
    while t < t_max:
        err = target - yaw
        v, w = rpp_rotate_cmd(err)
        if v > 0.0:
            return t, math.hypot(x, y)
        if assist is not None:
            v = assist.apply(v, w, 'TRANSIT', DT)
        yaw += turf_yaw_rate(v, w) * DT
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        t += DT
    return None, math.hypot(x, y)


def test_sim_pivot_assist_finishes_the_turn_far_sooner_within_cap():
    t_plain, d_plain = simulate(None)
    t_assist, d_assist = simulate(PivotAssist())
    # 150 deg -> 45 deg remaining = 1.83 rad at 0.051 rad/s in place: ~36 s (live: ~2 min
    # with stalls). With the creep the arc yaw rate (0.21) does most of it.
    assert t_plain is not None and t_plain > 30.0 and d_plain == 0.0
    assert t_assist is not None and t_assist < 0.5 * t_plain
    assert d_assist <= 0.3 + 1e-6        # displacement bounded by the per-pivot cap
    # A Nav2 PoseProgressChecker (0.25 m or 0.5 rad within 20 s) would not abort the
    # assisted pivot: in any 20 s window it turns > 0.5 rad.
    assert 0.21 * 6.0 + 0.051 * 14.0 > 0.5
