"""Tests for the VIO gate decision logic (pure, no ROS needed)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

from mower_localization.vio_gate_logic import (  # noqa: E402
    VioGateLogic,
    VioGateParams,
    rtk_is_fixed,
)

FIXED = ("um960=connected quality=RTK_FIXED sats=25/25 hdop=0.77 diff_age=1.0s "
         "corr_src=ntrip corr=streaming age=0.05s")
FLOAT = FIXED.replace("RTK_FIXED", "RTK_FLOAT")
GOOD = dict(vx=0.3, vy=0.0, var_vx=0.01, var_vy=0.01)


def _warm(gate, t0=0.0, n=None, dt=0.1, **kw):
    """Feed n healthy VIO messages at 10 Hz; return (last verdict, next time)."""
    n = gate.p.warmup_msgs if n is None else n
    msg = dict(GOOD, **kw)
    t = t0
    verdict = None
    for _ in range(n):
        verdict = gate.on_vio(t, **msg)
        t += dt
    return verdict, t


# -- rtk_is_fixed ------------------------------------------------------------------
@pytest.mark.parametrize("text, fixed", [
    (FIXED, True),
    (FLOAT, False),
    ("um960=connected solution=NARROW_INT sats=30/31", True),
    ("um960=connected solution=WIDE_INT", True),
    ("um960=connected solution=NARROW_FLOAT", False),
    ("um960=connected quality=GPS", False),
    ("STALE " + FIXED, False),
    ("", False),
    ("quality=RTK_FIXEDISH", False),  # token match, not substring
])
def test_rtk_is_fixed(text, fixed):
    assert rtk_is_fixed(text) is fixed


# -- RTK gating -------------------------------------------------------------------
def test_blocks_while_rtk_fixed():
    g = VioGateLogic()
    g.on_fix_status(FIXED, 0.0)
    verdict, t = _warm(g)
    g.on_fix_status(FIXED, t)
    assert g.on_vio(t, **GOOD) == "RTK fixed"


def test_passes_after_rtk_drops_and_holdoff():
    g = VioGateLogic(VioGateParams(rtk_holdoff_s=1.0))
    g.on_fix_status(FIXED, 0.0)
    _, t = _warm(g)
    g.on_fix_status(FLOAT, t)
    lost = t
    verdict, t = _warm(g, t0=t, n=5)          # 0.4 s after the drop
    assert "holdoff" in verdict
    g.on_fix_status(FLOAT, lost + 1.0)
    verdict, t = _warm(g, t0=t, n=7)          # now > 1.0 s after the drop
    assert verdict is None


def test_float_flicker_does_not_restart_holdoff():
    g = VioGateLogic(VioGateParams(rtk_holdoff_s=1.0))
    g.on_fix_status(FLOAT, 0.0)
    g.on_fix_status(FLOAT, 0.9)       # still lost: since stays at 0.0
    _, t = _warm(g, t0=0.0)
    assert t >= 1.0
    assert g.on_vio(t, **GOOD) is None


def test_rtk_recovery_blocks_immediately():
    g = VioGateLogic()
    g.on_fix_status(FLOAT, 0.0)
    _, t = _warm(g, t0=1.0)
    assert g.on_vio(t, **GOOD) is None
    g.on_fix_status(FIXED, t)
    assert g.on_vio(t + 0.1, **GOOD) == "RTK fixed"


def test_stale_fix_status_counts_as_rtk_lost():
    g = VioGateLogic(VioGateParams(fix_status_timeout_s=2.0))
    g.on_fix_status(FIXED, 0.0)
    _, t = _warm(g, t0=3.0)
    assert g.on_vio(t, **GOOD) is None  # receiver silent for > 2 s


def test_no_gps_at_all_passes_after_warmup():
    g = VioGateLogic()
    _, t = _warm(g, t0=5.0)
    assert g.on_vio(t, **GOOD) is None


# -- VIO health -------------------------------------------------------------------
def test_warmup_required():
    g = VioGateLogic(VioGateParams(warmup_msgs=5))
    g.on_fix_status(FLOAT, 0.0)
    verdict, t = _warm(g, t0=2.0, n=4)
    assert "warming up (4/5)" in verdict
    assert g.on_vio(t, **GOOD) is None


def test_gap_resets_warmup():
    g = VioGateLogic(VioGateParams(warmup_msgs=5, max_gap_s=0.5))
    g.on_fix_status(FLOAT, 0.0)
    _, t = _warm(g, t0=2.0)
    assert g.on_vio(t, **GOOD) is None
    assert "warming up (1/5)" in g.on_vio(t + 1.0, **GOOD)


@pytest.mark.parametrize("bad, fragment", [
    (dict(vx=float("nan")), "non-finite"),
    (dict(var_vx=math.inf), "non-finite"),
    (dict(var_vy=-1.0), "negative"),
    (dict(vx=2.0), "max_speed"),
    (dict(var_vx=1.0), "max_twist_var"),
])
def test_unhealthy_message_blocked_and_resets_streak(bad, fragment):
    g = VioGateLogic(VioGateParams(warmup_msgs=3))
    g.on_fix_status(FLOAT, 0.0)
    _, t = _warm(g, t0=2.0)
    assert fragment in g.on_vio(t, **dict(GOOD, **bad))
    assert g.healthy_streak == 0
    assert "warming up" in g.on_vio(t + 0.1, **GOOD)


def test_wheel_disagreement_latches_divergence_and_recovers():
    p = VioGateParams(warmup_msgs=3, disagreement_hold_s=1.0, recover_s=3.0)
    g = VioGateLogic(p)
    g.on_fix_status(FLOAT, 0.0)
    t = 2.0
    # wheels stopped, VIO claims 0.6 m/s (drift / divergence)
    for _ in range(12):
        g.on_wheel(0.0, t)
        verdict = g.on_vio(t, **dict(GOOD, vx=0.6))
        t += 0.1
    assert g.diverged and verdict == "VIO diverged from wheel odometry"
    # agreement for < recover_s keeps it blocked
    for _ in range(20):
        g.on_wheel(0.3, t)
        verdict = g.on_vio(t, **GOOD)
        t += 0.1
    assert g.diverged
    for _ in range(15):
        g.on_wheel(0.3, t)
        verdict = g.on_vio(t, **GOOD)
        t += 0.1
    assert not g.diverged and verdict is None


def test_short_disagreement_tolerated():
    g = VioGateLogic(VioGateParams(warmup_msgs=3, disagreement_hold_s=1.0))
    g.on_fix_status(FLOAT, 0.0)
    _, t = _warm(g, t0=2.0)
    for _ in range(5):  # 0.5 s of wheel slip
        g.on_wheel(0.0, t)
        g.on_vio(t, **GOOD)
        t += 0.1
    assert not g.diverged


def test_stale_wheel_odom_not_compared():
    g = VioGateLogic(VioGateParams(warmup_msgs=3, wheel_timeout_s=0.5))
    g.on_fix_status(FLOAT, 0.0)
    g.on_wheel(0.0, 0.0)
    _, t = _warm(g, t0=5.0, n=30, vx=0.6)
    assert not g.diverged


def test_reset_clears_state():
    g = VioGateLogic(VioGateParams(warmup_msgs=3))
    _warm(g, t0=2.0)
    g.diverged = True
    g.reset()
    assert g.healthy_streak == 0 and not g.diverged
