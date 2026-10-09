"""Tests for localization_status_logic (drift budget behind /localization/status).

One monotonic clock is fed by the ``_Sim`` helper; driving = x steps of 0.05 m every 0.1 s
with fix_status and map odom refreshed each step, like the live node.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

from mower_localization.localization_status_logic import (  # noqa: E402
    LocalizationStatus, StatusParams)

FIXED = 'um960=connected solution=NARROW_INT sats=20/30'
FLOAT = 'um960=connected solution=NARROW_FLOAT sats=20/30'
STEP, DT = 0.05, 0.1


class _Sim:
    def __init__(self, **params):
        self.s = LocalizationStatus(StatusParams(**params))
        self.t = 100.0
        self.x = 0.0
        self.s.on_odom_pose(self.t, self.x, 0.0)

    def tick(self, text=None, dx=0.0, vio=False, dt=DT, map_odom=True):
        self.t += dt
        self.x += dx
        if text is not None:
            self.s.on_fix_status(self.t, text)
        if map_odom:
            self.s.on_map_odom(self.t, 0.01, 0.0025)
        if vio:
            self.s.on_vio(self.t)
        self.s.on_odom_pose(self.t, self.x, 0.0)

    def hold(self, text, seconds):
        for _ in range(int(round(seconds / DT))):
            self.tick(text)

    def drive(self, text, metres, **kw):
        for _ in range(int(round(metres / STEP))):
            self.tick(text, dx=STEP, **kw)

    def fixed(self):
        self.hold(FIXED, 1.2)

    def verdict(self):
        return self.s.verdict(self.t)

    def est(self):
        return self.s.estimate(self.t)


class TestStartup:
    def test_no_map_odom_is_lost(self):
        sim = _Sim()
        sim.hold(FIXED, 2.0)
        sim.s.map_odom_t = None
        state, reason = sim.verdict()
        assert state == 'lost' and 'filtered_map' in reason

    def test_never_fixed_is_lost(self):
        sim = _Sim()
        sim.hold(FLOAT, 2.0)
        state, reason = sim.verdict()
        assert state == 'lost' and 'no RTK FIXED' in reason
        assert sim.est() is None

    def test_single_fixed_message_does_not_reset(self):
        sim = _Sim()
        sim.tick(FIXED)
        assert sim.est() is None
        assert sim.verdict()[0] == 'lost'

    def test_fixed_held_confirms(self):
        sim = _Sim()
        sim.fixed()
        assert sim.est() == pytest.approx(0.03)
        assert sim.verdict()[0] == 'ok'

    def test_unclassifiable_status_line_is_ignored(self):
        sim = _Sim()
        sim.fixed()
        sim.s.on_fix_status(sim.t + 0.01, 'um960=connected')
        assert sim.s.rtk == 'fixed'


class TestDrift:
    def test_five_metres_float_ok(self):
        sim = _Sim()
        sim.fixed()
        sim.drive(FLOAT, 5.0)
        assert sim.est() == pytest.approx(0.03 + 5 * 0.03, abs=1e-6)
        assert sim.verdict()[0] == 'ok'

    def test_ten_metres_degraded(self):
        sim = _Sim()
        sim.fixed()
        sim.drive(FLOAT, 10.0)
        # 0.03 + 0.3 = 0.33 > 0.3 budget (float cap 0.5 is above it)
        assert sim.est() == pytest.approx(0.33, abs=1e-6)
        assert sim.verdict()[0] == 'degraded'

    def test_float_cap_while_fresh(self):
        sim = _Sim()
        sim.fixed()
        sim.drive(FLOAT, 30.0)
        assert sim.s.drift_dr > 0.5          # uncapped value keeps growing
        assert sim.est() == pytest.approx(0.5)
        assert sim.verdict()[0] == 'degraded'

    def test_stale_fix_status_uncaps_and_goes_lost(self):
        sim = _Sim()
        sim.fixed()
        sim.drive(FLOAT, 40.0)
        assert sim.est() == pytest.approx(0.5)
        # /fix_status goes silent; keep driving 3 s (> the 2 s timeout)
        sim.drive(None, 1.5)
        assert sim.s.rtk_class(sim.t) == 'stale'
        assert sim.est() == pytest.approx(0.03 + 41.5 * 0.03, abs=1e-6)
        assert sim.est() > 1.0
        assert sim.verdict()[0] == 'lost'

    def test_vio_uses_lower_rate(self):
        sim = _Sim()
        sim.fixed()
        sim.drive(FLOAT, 5.0, vio=True)
        assert sim.s.vio_active(sim.t)
        assert sim.est() == pytest.approx(0.03 + 5 * 0.015, abs=1e-6)

    def test_vio_inactive_after_timeout(self):
        sim = _Sim()
        sim.fixed()
        sim.s.on_vio(sim.t)
        sim.tick(FLOAT, dt=1.5)
        assert not sim.s.vio_active(sim.t)

    def test_jump_is_ignored(self):
        sim = _Sim()
        sim.fixed()
        sim.s.on_odom_pose(sim.t + 0.1, sim.x + 5.0, 0.0)   # > max_step_m
        assert sim.s.dist_since_fixed == pytest.approx(0.0)
        assert sim.est() == pytest.approx(0.03)

    def test_non_finite_pose_is_ignored(self):
        sim = _Sim()
        sim.fixed()
        sim.s.on_odom_pose(sim.t + 0.1, float('nan'), 0.0)
        assert sim.est() == pytest.approx(0.03)

    def test_pivot_in_place_costs_nothing(self):
        sim = _Sim()
        sim.fixed()
        for _ in range(50):
            sim.tick(FLOAT, dx=0.0)
        assert sim.est() == pytest.approx(0.03)
        assert sim.s.dist_since_fixed == pytest.approx(0.0)

    def test_return_to_fixed_resets(self):
        sim = _Sim()
        sim.fixed()
        sim.drive(FLOAT, 10.0)
        assert sim.verdict()[0] == 'degraded'
        sim.hold(FIXED, 1.2)
        assert sim.est() == pytest.approx(0.03)
        assert sim.verdict()[0] == 'ok'
        assert sim.s.dist_since_fixed == pytest.approx(0.0)

    def test_lone_fixed_in_float_does_not_reset(self):
        sim = _Sim()
        sim.fixed()
        sim.drive(FLOAT, 10.0)
        sim.tick(FIXED)
        sim.tick(FLOAT)
        assert sim.s.drift_dr == pytest.approx(0.33, abs=1e-6)

    def test_map_odom_stale_is_lost(self):
        sim = _Sim()
        sim.fixed()
        sim.tick(FIXED, dt=1.5, map_odom=False)
        state, reason = sim.verdict()
        assert state == 'lost' and 'filtered_map' in reason


class TestStatusDict:
    def test_keys_and_sigma(self):
        sim = _Sim()
        sim.fixed()
        d = sim.s.status(sim.t)
        for key in ('state', 'reason', 'rtk', 'since_fixed_s', 'dist_since_fixed_m',
                    'est_drift_m', 'drift_budget_m', 'lost_drift_m', 'ekf_xy_sigma_m',
                    'vio_active', 'heading_aligned'):
            assert key in d
        assert d['state'] == 'ok' and d['rtk'] == 'fixed'
        assert d['ekf_xy_sigma_m'] == pytest.approx(math.sqrt(0.01), abs=1e-3)
        assert d['drift_budget_m'] == 0.3 and d['lost_drift_m'] == 1.0
        assert d['vio_active'] is False

    def test_heading_status_reflected(self):
        sim = _Sim()
        sim.fixed()
        assert sim.s.status(sim.t)['heading_aligned'] is False
        sim.s.on_heading_status({'aligned': True, 'source': 'cog', 'yaw_sigma_deg': 3.0})
        d = sim.s.status(sim.t)
        assert d['heading_aligned'] is True
        assert d['heading_source'] == 'cog' and d['heading_sigma_deg'] == 3.0

    def test_non_dict_heading_resets(self):
        s = LocalizationStatus()
        s.on_heading_status({'aligned': True})
        s.on_heading_status(None)
        assert s.status(1.0)['heading_aligned'] is False

    def test_status_before_any_input(self):
        d = LocalizationStatus().status(5.0)
        assert d['state'] == 'lost' and d['est_drift_m'] is None
        assert d['ekf_xy_sigma_m'] is None and d['since_fixed_s'] is None
