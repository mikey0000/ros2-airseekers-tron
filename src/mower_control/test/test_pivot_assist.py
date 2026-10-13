"""cmd_vel_slew pivot assist: unit logic + a small kinematic sim of an RPP-style
rotate-to-heading on the Tron's turf (in-place pivots barely turn; arcs do).

Values updated 2026-10-10 after the live side-to-side stuck incident: delay 1.5 -> 0.8 s,
linear 0.05 -> 0.14 m/s. 0.05 never reached the wheels -- shape_for_both_wheels snapped it
back to a pure pivot (inner wheel -0.022 m/s at |w| = 0.3); 0.14 gives inner 0.068 and
passes as a legal arc, but the 0.3 m per-pivot cap is then spent in ~2.2 s, so the assisted
turn in the sim below is only ~19% faster than plain (the live win is making progress at
all, instead of rocking in place until the slip detector trips).
"""

import math

import pytest

from mower_control.cmd_vel_slew import PivotAssist, shape_for_both_wheels

DT = 0.05


def test_only_after_delay_in_motion_phase_and_capped():
    pa = PivotAssist()
    outs = [pa.apply(0.0, 0.3, 'TRANSIT', DT) for _ in range(int(20 / DT))]
    # >= 15 samples (0.75 s) are always silent; the drift-safe fire check samples 0.9 s.
    assert all(o == 0.0 for o in outs[:int(0.8 / DT) - 1])
    assert outs[int(0.9 / DT)] == pytest.approx(0.14)
    assert pa.dist_m == pytest.approx(0.3, abs=0.02)
    assert sum(o * DT for o in outs) == pytest.approx(0.3, abs=0.02)
    assert outs[-1] == 0.0               # capped long before 20 s


@pytest.mark.parametrize('lin,ang,phase', [
    (0.0, 0.3, 'IDLE'), (0.0, 0.3, 'RETURNING_HOME'), (0.0, 0.3, None),
    (0.0, 0.05, 'MOWING'), (0.1, 0.3, 'MOWING'), (-0.1, 0.3, 'TRANSIT')])
def test_never_outside_a_pure_pivot_in_transit_or_mowing(lin, ang, phase):
    pa = PivotAssist()
    assert all(pa.apply(lin, ang, phase, DT) == lin for _ in range(100))


def test_new_pivot_resets_budget_and_disabled_is_neutral():
    pa = PivotAssist()
    for _ in range(200):
        pa.apply(0.0, 0.3, 'MOWING', DT)          # exhaust the 0.3 m cap
    pa.apply(0.2, 0.0, 'MOWING', DT)              # pivot over -> budget reset
    outs = [pa.apply(0.0, -0.3, 'MOWING', DT) for _ in range(100)]
    assert outs[int(1.6 / DT)] == pytest.approx(0.14)   # re-armed and firing
    assert outs[-1] == 0.0                       # cap reached again
    off = PivotAssist(enabled=False)
    assert all(off.apply(0.0, 0.3, 'MOWING', DT) == 0.0 for _ in range(100))


# ------------------------------------------------------- turn shaper interplay
def test_creep_survives_the_turn_shaper_as_a_legal_arc():
    """Regression 2026-10-10: the 0.05 m/s creep (inner wheel -0.022 at |w| = 0.3) was
    snapped back to a pure pivot by shape_for_both_wheels, so the creep never reached the
    wheels and the swath-end turn stalled/rocked in place -> STUCK at (-1.35, 2.90).
    0.14 m/s gives inner 0.068 >= min_inner and passes through unchanged as a tight arc."""
    pa = PivotAssist()
    lin = 0.0
    for _ in range(int(1.2 / DT)):
        lin = pa.apply(0.0, 0.3, 'TRANSIT', DT)
        v, w, info = shape_for_both_wheels(lin, 0.3)
        assert v == pytest.approx(lin)     # creep not stripped (0.0 before the delay)
        assert info is None                # proper arc/pivot pass-through, no snap
    assert lin == pytest.approx(0.14)
    assert abs(lin) - 0.3 * 0.24 >= 0.06 - 1e-9   # inner wheel clears the deadband


def test_sub_floor_creep_is_snapped_back_to_a_pure_pivot():
    """Why 0.05 failed live: it lands in the one-wheel zone and the shaper zeroes v."""
    v, w, info = shape_for_both_wheels(0.05, 0.3)
    assert v == 0.0
    assert info is not None and info['kind'] == 'pivot'


# ---------------------------------------------------------------- kinematic sim
def turf_yaw_rate(v, w):
    """Measured on turf (2026-10-06): in place ~0.05 rad/s of 0.3; arcs ~0.2 of 0.3."""
    return w * (0.17 if abs(v) < 0.02 else 0.7)


def rpp_rotate_cmd(heading_err):
    """Jazzy RPP rotate_to_heading: linear 0, angular +-0.3 while |err| > min angle."""
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


def test_sim_pivot_assist_finishes_the_turn_sooner_within_cap():
    t_plain, d_plain = simulate(None)
    t_assist, d_assist = simulate(PivotAssist())
    # 150 deg -> 45 deg remaining = 1.83 rad at 0.051 rad/s in place: ~36 s (live: ~2 min
    # with stalls). The 0.3 m cap at 0.14 m/s is spent in only ~2.2 s of arc (0.21 rad/s),
    # giving ~29 s -- 19% sooner, the rest still in place. The real win is progress at all
    # (the 0.05 creep was snapped back by the shaper and never helped).
    assert t_plain is not None and t_plain > 30.0 and d_plain == 0.0
    assert t_assist is not None and t_assist < 0.9 * t_plain
    assert d_assist <= 0.31 + 1e-6        # displacement bounded near the per-pivot cap
    # A Nav2 PoseProgressChecker (0.25 m or 0.5 rad within 20 s) would not abort the
    # assisted pivot: in any 20 s window it turns > 0.5 rad.
    assert 0.21 * 2.2 + 0.051 * 17.8 > 0.5


def test_slow_rotate_to_heading_0_1_radps_triggers_by_default():
    """Live 2026-10-07: RPP ramped only to 0.16 rad/s; the 0.2 threshold never fired."""
    pa = PivotAssist()
    assert pa.min_angular == pytest.approx(0.08)
    assert pa.delay_s == pytest.approx(0.8) and pa.linear == pytest.approx(0.14)
    outs = [pa.apply(0.0, 0.1, 'TRANSIT', DT) for _ in range(int(3.0 / DT))]
    assert outs[int(1.25 / DT)] == pytest.approx(0.14)   # firing mid-rotate