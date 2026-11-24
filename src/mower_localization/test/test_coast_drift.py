"""Coast mode (2026-11-25): the tilt-aware drift model, the coast budget, slip -> lost and the
RTK-versus-dead-reckoning gate (docs/analysis/2026-11-25_coast_mode.md)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

from mower_localization import drift_model as dm  # noqa: E402
from mower_localization.localization_status_logic import (  # noqa: E402
    LocalizationStatus, StatusParams)

FIXED = 'um960=connected solution=NARROW_INT sats=20/30'
FLOAT = 'um960=connected solution=NARROW_FLOAT sats=20/30'
SINGLE = 'um960=connected solution=SINGLE sats=20/30'
DP, CP = dm.DriftParams(), dm.CoastParams()


# ---- the pure model ---------------------------------------------------------------------------
class TestModel:
    def test_flat_five_metres(self):
        assert dm.drift_estimate(5.0, 2.0, 0, DP) == pytest.approx(0.05 + 0.02 * 5)

    def test_slope_at_eleven_degrees(self):
        assert dm.drift_estimate(1.5, 11.0, 0, DP) == pytest.approx(0.10 + 0.10 * 1.5)
        assert dm.drift_estimate(1.5, 15.0, 0, DP) == pytest.approx(0.10 + 0.10 * 1.5)

    def test_tilt_is_linear_between_flat_and_slope(self):
        assert dm.drift_estimate(2.0, 7.5, 0, DP) == pytest.approx(0.075 + 0.06 * 2.0)

    def test_pivots_cost_one_centimetre_each(self):
        assert dm.drift_estimate(0.0, 0.0, 3, DP) == pytest.approx(0.05 + 0.03)

    def test_vio_aid_halves_the_rate(self):
        assert dm.drift_rate(0.0, DP, 'vio') == pytest.approx(0.01)
        assert dm.drift_rate(0.0, DP, 'unknown source') == pytest.approx(0.02)

    def test_legacy_parameters_give_the_old_model(self):
        old = dm.DriftParams(0.03, 0.03, 0.03, 0.03, pivot_m=0.0)
        for tilt in (0.0, 6.0, 12.0):
            assert dm.drift_estimate(10.0, tilt, 4, old) == pytest.approx(0.33)

    def test_coast_budget_by_tilt(self):
        assert dm.coast_budget_m(None, DP, CP) == 5.0
        assert dm.coast_budget_m(3.0, DP, CP) == 5.0
        assert dm.coast_budget_m(6.0, DP, CP) == pytest.approx(3.25)
        assert dm.coast_budget_m(8.0, DP, CP) == pytest.approx(1.5)
        assert dm.coast_budget_m(11.0, DP, CP) == pytest.approx(1.5)

    def test_float_gate(self):
        assert not dm.float_disagrees(0.20, 0.10, 0.15)
        assert dm.float_disagrees(0.30, 0.10, 0.15)       # the 11-13 float after 1-2 m
        assert not dm.float_disagrees(5.0, 0.1, 0.0)      # gate off

    def test_pivot_counter(self):
        pc = dm.PivotCounter()
        for k in range(10):                               # 90 deg in place
            pc.update(math.radians(10 * k), 0.0)
        pc.update(math.radians(90), 0.2)                  # drives off
        for k in range(4):                                # 30 deg: not a pivot
            pc.update(math.radians(90 + 10 * k), 0.0)
        assert pc.count == 1

    def test_turning_while_driving_is_not_a_pivot(self):
        pc = dm.PivotCounter()
        for k in range(20):
            pc.update(math.radians(10 * k), 0.2)
        assert pc.count == 0

    def test_slip_detector(self):
        sd = dm.SlipDetector()
        for k in range(10):                               # wheels 0.7 rad/s, gyro 0.2: 0.45 rad
            sd.update(0.1 * k, 0.7, 0.2)
        assert not sd.slipped()
        for k in range(10, 14):                           # 1.3 s: 0.65 rad
            sd.update(0.1 * k, 0.7, 0.2)
        assert sd.slipped()

    def test_slow_mismatch_is_not_slip(self):
        sd = dm.SlipDetector()
        for k in range(300):                              # 0.1 rad/s for 30 s: 0.2 rad per 2 s
            sd.update(0.1 * k, 0.3, 0.2)
        assert not sd.slipped()


# ---- the status logic -------------------------------------------------------------------------
class _Sim:
    """5 Hz ticks: /fix_status, ekf_map (pose = the dead-reckoned one unless told), odom."""
    DT = 0.2

    def __init__(self, **params):
        self.s = LocalizationStatus(StatusParams(**params))
        self.t, self.x, self.y, self.yaw = 100.0, 0.0, 0.0, 0.0
        self.tick(FIXED)

    def tick(self, text, dx=0.0, dyaw=0.0, vx=None, gyro=0.0, wheel=None, gps=None, tilt=None):
        self.t += self.DT
        self.x += dx * math.cos(self.yaw)
        self.y += dx * math.sin(self.yaw)
        self.yaw += dyaw
        if text is not None:
            self.s.on_fix_status(self.t, text)
        self.s.on_map_odom(self.t, 0.01, 0.01, (self.x, self.y, self.yaw))
        if tilt is not None:
            self.s.on_tilt(self.t, 0.0, tilt)
        self.s.on_odom_pose(self.t, self.x, self.y, self.yaw,
                            dx / self.DT if vx is None else vx, gyro)
        if wheel is not None:
            self.s.on_wheel_twist(self.t, wheel)
        if gps is not None:
            self.s.on_gps_odom(self.t, self.x + gps[0], self.y + gps[1])

    def fixed(self, secs=2.0):
        for _ in range(int(round(secs / self.DT))):
            self.tick(FIXED)

    def drive(self, text, metres, speed=0.25, **kw):
        for _ in range(int(round(metres / (speed * self.DT)))):
            self.tick(text, dx=speed * self.DT, **kw)

    def st(self):
        return self.s.status(self.t)


def fixed_sim(**params):
    sim = _Sim(**params)
    sim.fixed()
    return sim


class TestCoastBudget:
    def test_flat_dropout_coasts_five_metres(self):
        sim = fixed_sim()
        sim.drive(SINGLE, 2.0)
        st = sim.st()
        assert st['state'] != 'lost' and st['coast_left_m'] == pytest.approx(3.0, abs=0.06)
        assert st['est_drift_m'] == pytest.approx(0.05 + 0.02 * 2.0, abs=0.003)
        sim.drive(SINGLE, 3.1)
        assert sim.st()['coast_left_m'] == 0.0

    def test_no_rtk_rule_waits_while_the_coast_lasts(self):
        sim = fixed_sim()
        sim.drive(SINGLE, 2.4, speed=0.15)                # 16 s > lost_after_s 10 s
        assert sim.st()['state'] == 'degraded' or sim.st()['state'] == 'ok'
        assert sim.st()['no_rtk_s'] > 10.0 and sim.st()['coast_left_m'] > 2.0
        sim.drive(SINGLE, 2.7, speed=0.15)                # budget spent
        st = sim.st()
        assert st['state'] == 'lost' and 'no RTK' in st['reason']

    def test_time_cap(self):
        sim = fixed_sim()
        sim.drive(SINGLE, 1.0, speed=0.05)                # 20 s
        assert sim.st()['coast_left_m'] > 0.0
        for _ in range(int(41 / _Sim.DT)):                # standing, 61 s since the fix
            sim.tick(SINGLE)
        assert sim.st()['coast_left_m'] == 0.0 and sim.st()['state'] == 'lost'

    def test_slope_budget_and_drift(self):
        sim = fixed_sim()
        sim.drive(SINGLE, 1.0, tilt=11.0)
        st = sim.st()
        assert st['tilt_mean_deg'] == pytest.approx(11.0)
        assert st['est_drift_m'] == pytest.approx(0.10 + 0.10 * 1.0, abs=0.006)
        assert st['coast_left_m'] == pytest.approx(0.5, abs=0.06)
        sim.drive(SINGLE, 0.6, tilt=11.0)
        assert sim.st()['coast_left_m'] == 0.0

    def test_pivots_spend_no_distance(self):
        sim = fixed_sim()
        for _ in range(4):                                # four 90 deg pivots in place
            for _ in range(10):
                sim.tick(SINGLE, dyaw=math.radians(9), vx=0.0)
            sim.tick(SINGLE, dx=0.01, vx=0.2)
        st = sim.st()
        assert st['pivots'] == 4 and st['dist_since_fixed_m'] < 0.05
        assert st['coast_left_m'] > 4.9
        assert st['est_drift_m'] == pytest.approx(0.05 + 0.04, abs=0.003)

    def test_fixed_resets_the_budget(self):
        sim = fixed_sim()
        sim.drive(SINGLE, 5.5)
        assert sim.st()['coast_left_m'] == 0.0
        sim.fixed()
        st = sim.st()
        assert st['coast_left_m'] == 5.0 and st['state'] == 'ok' and st['pivots'] == 0

    def test_coast_off_restores_lost_after_s(self):
        sim = fixed_sim(coast_max_s=0.0)
        sim.drive(SINGLE, 2.0, speed=0.15)                # 13 s
        st = sim.st()
        assert st['coast_left_m'] == 0.0 and st['state'] == 'lost' and 'no RTK' in st['reason']


class TestSlip:
    def test_slip_is_lost_until_the_next_fix(self):
        sim = fixed_sim()
        sim.drive(SINGLE, 1.0)
        for _ in range(10):                               # 2 s: wheels 0.7 rad/s, gyro 0.2
            sim.tick(SINGLE, dyaw=0.04, vx=0.0, gyro=0.2, wheel=0.7)
        st = sim.st()
        assert st['state'] == 'lost' and 'wheel slip' in st['reason']
        assert st['coast_left_m'] == 0.0 and st['dr_lost']
        sim.fixed()
        assert sim.st()['state'] == 'ok' and sim.st()['coast_left_m'] == 5.0

    def test_matching_wheels_and_gyro_are_not_slip(self):
        sim = fixed_sim()
        for _ in range(50):
            sim.tick(SINGLE, dyaw=0.1, vx=0.0, gyro=0.5, wheel=0.52)
        assert sim.st()['state'] != 'lost'

    def test_slip_while_fixed_is_ignored(self):
        sim = fixed_sim()
        for _ in range(10):
            sim.tick(FIXED, vx=0.0, gyro=0.2, wheel=0.7)
        assert sim.st()['state'] == 'ok'


class TestRtkGate:
    def _coast(self):
        sim = fixed_sim()
        sim.drive(SINGLE, 2.0)                            # est 0.09: gate 0.24
        return sim

    def test_float_that_disagrees_is_lost(self):
        sim = self._coast()
        sim.tick(FLOAT, gps=(0.0, -0.3))
        assert sim.st()['state'] != 'lost'                # 1 sample < float_gate_n
        sim.tick(FLOAT, gps=(0.0, -0.3))
        st = sim.st()
        assert st['state'] == 'lost' and 'from dead reckoning' in st['reason']
        assert st['rtk_dr_m'] == pytest.approx(0.3, abs=0.01)
        assert st['rtk_dr_gate_m'] == pytest.approx(0.15 + 0.09, abs=0.01)
        for _ in range(20):                               # stays lost while the float agrees again
            sim.tick(FLOAT, gps=(0.0, 0.0))
        assert sim.st()['state'] == 'lost'
        sim.fixed()
        assert sim.st()['state'] == 'ok' and sim.st()['dr_lost'] is None

    def test_float_that_agrees_carries_on(self):
        sim = self._coast()
        for _ in range(20):
            sim.tick(FLOAT, gps=(0.1, -0.1))
        st = sim.st()
        assert st['state'] == 'ok' and st['rtk'] == 'float'
        assert st['rtk_dr_m'] == pytest.approx(0.141, abs=0.01)

    def test_one_outlier_is_not_enough(self):
        sim = self._coast()
        for k in range(12):
            sim.tick(FLOAT, gps=(0.0, -0.4) if k % 3 == 0 else (0.0, 0.05))
        assert sim.st()['state'] != 'lost'

    def test_unconfirmed_fixed_is_checked_too(self):
        sim = self._coast()
        for _ in range(3):
            sim.tick(FIXED, gps=(0.4, 0.0))               # within fixed_confirm_s
        assert sim.st()['state'] == 'lost'

    def test_field_1121_float_onset_jump_without_a_dropout(self):
        """11-21 07:13:41: RTK fixed straight to float 3.02 m off (receiver sigma 8 cm) for
        0.6 s while mowing; the gate ignores the fix's own sigma."""
        sim = fixed_sim()
        sim.drive(FIXED, 1.0)
        for _ in range(3):
            sim.tick(FLOAT, dx=0.05, gps=(-2.67, -1.41))
        st = sim.st()
        assert st['state'] == 'lost' and st['rtk_dr_m'] == pytest.approx(3.02, abs=0.01)

    def test_dead_reckoned_pose_ignores_the_map_pose_jump(self):
        """ekf_map pulled onto the float does not move the reference."""
        sim = self._coast()
        x, y = sim.x, sim.y
        sim.s.on_map_odom(sim.t, 0.01, 0.01, (x, y - 0.3, 0.0))
        dr = sim.s.dead_reckoned_pose()
        assert dr[0] == pytest.approx(x) and dr[1] == pytest.approx(y)

    def test_gate_off(self):
        sim = fixed_sim(float_gate_m=0.0)
        sim.drive(SINGLE, 2.0)
        for _ in range(5):
            sim.tick(FLOAT, gps=(0.0, -1.0))
        assert sim.st()['state'] != 'lost'
