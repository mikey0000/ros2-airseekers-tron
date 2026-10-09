"""Tests for mower_localization.rtk_quality (pure logic, no ROS).

The class strings come from um960_gps_driver's /fix_status (BESTNAV ``solution=`` or GGA
``quality=``); the covariance policy decides what the EKF is told per class.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

from mower_localization import rtk_quality as rq  # noqa: E402
from mower_localization.rtk_quality import (  # noqa: E402
    CovariancePolicy, class_from_navsat, classify_fix_status, fused_variance)

FIX2 = 0.03 ** 2


class TestClassifyFixStatus:
    def test_real_driver_line(self):
        # the exact shape um960_gps_driver publishes
        assert classify_fix_status(
            'um960=connected solution=NARROW_INT sats=20/30 hdop=0.7') == 'fixed'

    @pytest.mark.parametrize('sol', ['WIDE_INT', 'L1_INT', 'INS_RTKFIXED', 'FIXEDPOS',
                                     'NARROW_INT'])
    def test_fixed_solutions(self, sol):
        assert classify_fix_status('um960=connected solution=%s' % sol) == 'fixed'

    @pytest.mark.parametrize('sol', ['NARROW_FLOAT', 'L1_FLOAT', 'IONOFREE_FLOAT',
                                     'INS_RTKFLOAT'])
    def test_float_solutions(self, sol):
        assert classify_fix_status('solution=%s sats=20' % sol) == 'float'

    @pytest.mark.parametrize('text,expected', [
        ('quality=RTK_FLOAT', 'float'), ('quality=RTK_FIXED', 'fixed'),
        ('quality=DGPS', 'dgps'), ('quality=GPS', 'single'),
        ('solution=PSRDIFF', 'dgps'), ('solution=SBAS', 'dgps'),
        ('solution=SINGLE', 'single'),
        ('solution=NONE', 'none'), ('solution=INVALID', 'none'),
    ])
    def test_other_classes(self, text, expected):
        assert classify_fix_status(text) == expected

    def test_stale_prefix(self):
        assert classify_fix_status('STALE no fix for 5 s') == 'stale'

    @pytest.mark.parametrize('text', ['', None, 'um960=connected sats=5/9',
                                      'hello world'])
    def test_no_solution_is_none(self, text):
        # None = "line says nothing about the solution": callers keep the previous class
        assert classify_fix_status(text) is None

    def test_solution_wins_over_quality(self):
        assert classify_fix_status('quality=RTK_FIXED solution=NARROW_FLOAT') == 'float'
        assert classify_fix_status('solution=SINGLE quality=RTK_FIXED') == 'single'

    def test_case_insensitive(self):
        assert classify_fix_status('solution=narrow_int') == 'fixed'

    @pytest.mark.parametrize('text,expected', [
        ('solution=FOO_INT', 'fixed'),
        ('solution=WEIRD_FLOAT_X', 'float'),
        ('solution=SOMETHING_ELSE', 'single'),
    ])
    def test_unknown_labels_fall_back(self, text, expected):
        assert classify_fix_status(text) == expected


class TestClassFromNavsat:
    @pytest.mark.parametrize('status,expected', [
        (None, 'none'), (-1, 'none'), (2, 'float'), (1, 'dgps'), (0, 'single')])
    def test_mapping(self, status, expected):
        assert class_from_navsat(status) == expected


class TestFusedVariance:
    def test_fixed_floors_at_3cm(self):
        assert fused_variance('fixed', 0.0001) == pytest.approx(FIX2)

    def test_fixed_keeps_larger_receiver_variance(self):
        assert fused_variance('fixed', 0.01) == pytest.approx(0.01)

    def test_float_is_4x_with_floor(self):
        assert fused_variance('float', 0.01) == pytest.approx(0.16)   # floor 0.4^2
        assert fused_variance('float', 0.1) == pytest.approx(0.4)     # 4x wins

    @pytest.mark.parametrize('cls', ['dgps', 'single'])
    def test_dgps_single_rejected_by_default(self, cls):
        assert fused_variance(cls, 0.01) is None

    @pytest.mark.parametrize('cls', ['dgps', 'single'])
    def test_loose_policy_accepts(self, cls):
        p = CovariancePolicy(single_policy='loose')
        assert fused_variance(cls, 0.01, p) == pytest.approx(9.0)
        assert fused_variance(cls, 5.0, p) == pytest.approx(20.0)     # 4x

    def test_unknown_receiver_variance(self):
        assert fused_variance('fixed', None) == pytest.approx(FIX2)
        assert fused_variance('float', None) == pytest.approx(0.16)
        assert fused_variance('single', None, CovariancePolicy(single_policy='loose')) \
            == pytest.approx(25.0)

    @pytest.mark.parametrize('cls', ['none', 'stale', None])
    def test_unusable_classes_drop(self, cls):
        assert fused_variance(cls, 0.01) is None
        assert fused_variance(cls, None) is None

    def test_loose_policy_still_drops_none_stale(self):
        p = CovariancePolicy(single_policy='loose')
        assert fused_variance('none', 0.01, p) is None
        assert fused_variance('stale', 0.01, p) is None

    @pytest.mark.parametrize('cls', ['fixed', 'float'])
    @pytest.mark.parametrize('rx', [1e-6, 1e-4, 0.01, 0.16, 1.0, 50.0])
    def test_never_below_receiver_variance(self, cls, rx):
        # the filter must never be told something tighter than the receiver claims
        assert fused_variance(cls, rx) >= rx
        assert math.isfinite(fused_variance(cls, rx))
