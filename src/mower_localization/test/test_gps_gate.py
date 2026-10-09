"""Tests for the mower_localization GPS quality gate.

Covers the pure decision function (which is where the gating logic
lives) and the node's pass-through behaviour: a fix with
status >= STATUS_FIX and sane covariance is forwarded unchanged, a
no-fix / bad-covariance / non-used fix is withheld, and the thresholds
are configurable. No hardware and no serial port involved.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensor_msgs.msg import NavSatFix  # noqa: E402  (after the sys.path fix)
from sensor_msgs.msg import NavSatStatus  # noqa: E402

import types as _types

import pytest

import rclpy as _rclpy

# The conftest shim module has no __file__; real rclpy does.
REAL_RCLPY = getattr(_rclpy, "__file__", None) is not None


class _Recorder:
    """Stands in for the node's fix publisher (works under stub and real rclpy)."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


from mower_localization.gps_gate import (  # noqa: E402
    DEFAULT_USED_FIXES,
    GpsGateNode,
    _max_position_covariance,
    _reject_reason,
)


def _fix(status=NavSatStatus.STATUS_FIX, covariance=None,
         covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN):
    """Build a NavSatFix carrying just the fields the gate reads.

    Covariance type defaults to DIAGONAL_KNOWN so the covariance
    gate is exercised; the UNKNOWN case is tested explicitly.
    """
    msg = NavSatFix(latitude=47.5, longitude=8.3)
    msg.status.status = status
    msg.position_covariance = covariance or [0.0] * 9
    msg.position_covariance_type = covariance_type
    return msg


def _diag_covariance(xx):
    return [xx, 0.0, 0.0, 0.0, xx, 0.0, 0.0, 0.0, xx]


class TestRejectReason:
    def test_plain_gps_fix_passes(self):
        assert _reject_reason(_fix(), 0, 100.0, frozenset()) is None

    def test_no_fix_is_rejected(self):
        reason = _reject_reason(
            _fix(status=NavSatStatus.STATUS_NO_FIX), 0, 100.0, frozenset())
        assert reason is not None
        assert "min_fix_status" in reason

    def test_status_threshold_is_inclusive(self):
        # Humble: STATUS_FIX (0) passes against min_fix_status 0, STATUS_NO_FIX
        # (-1) does not.
        assert _reject_reason(
            _fix(status=0), 0, 100.0, frozenset()) is None
        assert _reject_reason(
            _fix(status=-1), 0, 100.0, frozenset()) is not None

    def test_min_fix_status_is_configurable(self):
        # A deployment that demands at least a differential (SBAS) fix.
        sbas = NavSatStatus.STATUS_SBAS_FIX
        assert _reject_reason(
            _fix(status=NavSatStatus.STATUS_FIX), sbas, 100.0,
            frozenset()) is not None
        assert _reject_reason(
            _fix(status=NavSatStatus.STATUS_SBAS_FIX), sbas, 100.0,
            frozenset()) is None

    def test_unknown_status_values_are_rejected_by_the_whitelist(self):
        # A status value Humble does not define (e.g. a driver bug) is
        # numerically >= STATUS_FIX but must not be fused.
        used = frozenset(DEFAULT_USED_FIXES)
        for status in (3, 7, 8):
            assert _reject_reason(
                _fix(status=status), 0, 100.0, used) is not None

    def test_large_covariance_is_rejected(self):
        reason = _reject_reason(
            _fix(covariance=_diag_covariance(500.0)), 0, 100.0, frozenset())
        assert reason is not None
        assert "covariance" in reason

    def test_covariance_threshold_is_inclusive(self):
        assert _reject_reason(
            _fix(covariance=_diag_covariance(100.0)), 0, 100.0,
            frozenset()) is None

    def test_covariance_check_is_disabled_at_zero(self):
        assert _reject_reason(
            _fix(covariance=_diag_covariance(1e9)), 0, 0.0, frozenset()) \
            is None

    def test_unknown_covariance_type_passes(self):
        # The UM960 driver reports UNKNOWN when the receiver gave no
        # accuracy numbers; the gate cannot check what it does not have.
        msg = _fix(covariance=_diag_covariance(1e9),
                   covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN)
        assert _reject_reason(msg, 0, 100.0, frozenset()) is None
    def test_used_fixes_whitelist_filters(self):
        used = frozenset([NavSatStatus.STATUS_GBAS_FIX])
        # Single-point fix is >= STATUS_FIX but not an RTK (GBAS) fix.
        assert _reject_reason(
            _fix(status=NavSatStatus.STATUS_FIX), 0, 100.0, used) is not None
        assert _reject_reason(
            _fix(status=NavSatStatus.STATUS_GBAS_FIX), 0, 100.0,
            used) is None

    def test_empty_used_fixes_disables_the_whitelist(self):
        assert _reject_reason(
            _fix(status=7), 0, 100.0,
            frozenset()) is None

    def test_default_used_fixes_cover_every_humble_fix(self):
        assert DEFAULT_USED_FIXES == (
            NavSatStatus.STATUS_FIX,
            NavSatStatus.STATUS_SBAS_FIX,
            NavSatStatus.STATUS_GBAS_FIX,
        )

    def test_reason_names_the_first_failure_only(self):
        # Failure-first precedence: status is checked before covariance.
        msg = _fix(status=NavSatStatus.STATUS_NO_FIX,
                   covariance=_diag_covariance(1e9))
        reason = _reject_reason(msg, 0, 100.0, frozenset())
        assert reason is not None
        assert "min_fix_status" in reason
        assert "covariance" not in reason


class TestMaxPositionCovariance:
    def test_returns_the_largest_diagonal(self):
        msg = _fix(covariance=[1.0, 9.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 2.0])
        assert _max_position_covariance(msg) == 4.0

    def test_off_diagonal_correlations_are_ignored(self):
        msg = _fix(covariance=[1.0, 99.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0])
        assert _max_position_covariance(msg) == 3.0

    def test_unknown_type_returns_none(self):
        msg = _fix(covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN)
        assert _max_position_covariance(msg) is None


class TestGpsGateNode:
    @pytest.fixture(autouse=True)
    def _cleanup(self):
        self._nodes = []
        yield
        for node in self._nodes:
            node.destroy_node()

    def _node(self, **overrides):
        # The pass-through gate tests below predate the 2026-10-09 fix-quality covariance
        # (TestQualityCovariance); they run with it off unless a test asks for it.
        overrides.setdefault("quality_covariance", False)
        if REAL_RCLPY:
            from rclpy.parameter import Parameter
            parameters = [Parameter(n, value=v) for n, v in overrides.items()]
        else:
            parameters = [_types.SimpleNamespace(name=n, value=v)
                          for n, v in overrides.items()]
        node = GpsGateNode(parameter_overrides=parameters)
        self._nodes.append(node)
        # Capture what would be published (replaces the real publisher).
        node.fix_pub = _Recorder()
        return node

    def test_fix_passes_through_unchanged(self):
        node = self._node()
        fix = _fix()
        node._on_fix(fix)
        assert node.fix_pub.msgs == [fix]
        assert node.received == 1
        assert node.passed == 1
        assert node.rejected == 0

    def test_no_fix_is_withheld(self):
        node = self._node()
        node._on_fix(_fix(status=NavSatStatus.STATUS_NO_FIX))
        assert node.fix_pub.msgs == []
        assert node.rejected == 1

    def test_large_covariance_is_withheld(self):
        node = self._node()
        node._on_fix(_fix(covariance=_diag_covariance(1000.0)))
        assert node.fix_pub.msgs == []

    def test_rtk_passes_by_default(self):
        node = self._node()
        node._on_fix(_fix(status=NavSatStatus.STATUS_GBAS_FIX))
        assert len(node.fix_pub.msgs) == 1

    def test_used_fixes_param_tightens_the_gate(self):
        # RTK-only deployment: autonomous fixes are dropped.
        node = self._node(used_fixes=[NavSatStatus.STATUS_GBAS_FIX])
        node._on_fix(_fix(status=NavSatStatus.STATUS_FIX))
        assert node.fix_pub.msgs == []
        node._on_fix(_fix(status=NavSatStatus.STATUS_GBAS_FIX))
        assert len(node.fix_pub.msgs) == 1

    def test_covariance_param_tightens_the_gate(self):
        node = self._node(max_position_covariance=1.0)
        node._on_fix(_fix(covariance=_diag_covariance(4.0)))
        assert node.fix_pub.msgs == []
        node._on_fix(_fix(covariance=_diag_covariance(0.5)))
        assert len(node.fix_pub.msgs) == 1

    def test_topic_names_are_remappable(self):
        node = self._node(
            input_topic="/fix_src", output_topic="/fix_gated_tmcoffset")
        assert node.input_topic == "/fix_src"
        assert node.output_topic == "/fix_gated_tmcoffset"

    def test_subscription_matches_driver_qos(self):
        # um960_gps_driver publishes /fix reliable, depth 10: the gate
        # must subscribe with a compatible profile or it sees nothing.
        node = self._node()
        if REAL_RCLPY:
            from rclpy.qos import ReliabilityPolicy
            info = node.get_subscriptions_info_by_topic("/fix")
            assert len(info) == 1
            assert info[0].qos_profile.reliability == \
                ReliabilityPolicy.RELIABLE
        else:
            subscription = node.subs["/fix"]
            assert subscription.qos["reliability"] == 1  # RELIABLE
            assert subscription.qos["depth"] == 10


class TestQualityCovariance:
    """quality_covariance=True: /fix_status class sets the covariance the EKF sees."""

    @pytest.fixture(autouse=True)
    def _cleanup(self):
        self._nodes = []
        yield
        for node in self._nodes:
            node.destroy_node()

    def _node(self, **overrides):
        return TestGpsGateNode._node(self, quality_covariance=True, **overrides)

    @staticmethod
    def _policy(**kw):
        from mower_localization.rtk_quality import CovariancePolicy
        return CovariancePolicy(**kw)

    # (a) pure _apply_quality
    def test_fixed_floors_at_3cm_and_zeroes_cross_terms(self):
        from mower_localization.gps_gate import _apply_quality
        msg = _fix(covariance=[0.0001, 0.5, 0.0, 0.5, 0.0001, 0.0, 0.0, 0.0, 0.7])
        assert _apply_quality(msg, "fixed", self._policy()) is None
        assert msg.position_covariance[0] == msg.position_covariance[4] == \
            pytest.approx(0.0009)
        assert msg.position_covariance[1] == msg.position_covariance[3] == 0.0
        assert msg.position_covariance[8] == 0.7     # vertical kept
        assert msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    def test_float_is_inflated_to_floor(self):
        from mower_localization.gps_gate import _apply_quality
        msg = _fix(covariance=_diag_covariance(0.01))
        assert _apply_quality(msg, "float", self._policy()) is None
        assert msg.position_covariance[0] == msg.position_covariance[4] == \
            pytest.approx(0.16)

    def test_single_is_rejected_with_reason(self):
        from mower_localization.gps_gate import _apply_quality
        msg = _fix(covariance=_diag_covariance(0.01))
        reason = _apply_quality(msg, "single", self._policy())
        assert reason is not None and "single" in reason

    def test_unknown_covariance_type_gets_conservative_values(self):
        from mower_localization.gps_gate import _apply_quality
        msg = _fix(covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN)
        assert _apply_quality(msg, "float", self._policy()) is None
        assert msg.position_covariance[0] == msg.position_covariance[4] == \
            pytest.approx(0.16)
        assert msg.position_covariance[8] == pytest.approx(0.64)    # 4x
        assert msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    # (b) node behaviour
    @staticmethod
    def _status(text):
        from std_msgs.msg import String
        return String(data=text)

    def test_float_status_inflates_fix(self):
        node = self._node()
        node._on_fix_status(self._status('um960=connected solution=NARROW_FLOAT'))
        node._on_fix(_fix(covariance=_diag_covariance(0.01)))
        assert len(node.fix_pub.msgs) == 1
        assert node.fix_pub.msgs[0].position_covariance[0] == pytest.approx(0.16)

    def test_fixed_status_keeps_receiver_sigma(self):
        node = self._node()
        node._on_fix_status(self._status('solution=NARROW_INT'))
        node._on_fix(_fix(status=NavSatStatus.STATUS_GBAS_FIX,
                          covariance=_diag_covariance(0.01)))
        assert node.fix_pub.msgs[0].position_covariance[0] == pytest.approx(0.01)

    def test_single_status_drops_next_fix(self):
        node = self._node()
        node._on_fix_status(self._status('solution=SINGLE'))
        node._on_fix(_fix(covariance=_diag_covariance(0.01)))
        assert node.fix_pub.msgs == []
        assert node.rejected == 1

    def test_no_status_gbas_passes_as_float(self):
        node = self._node()
        node._on_fix(_fix(status=NavSatStatus.STATUS_GBAS_FIX,
                          covariance=_diag_covariance(0.01)))
        assert node.fix_pub.msgs[0].position_covariance[0] == pytest.approx(0.16)

    def test_no_status_plain_fix_is_dropped(self):
        node = self._node()
        node._on_fix(_fix(status=NavSatStatus.STATUS_FIX,
                          covariance=_diag_covariance(0.01)))
        assert node.fix_pub.msgs == []
        assert node.rejected == 1

    def test_stale_status_falls_back_to_navsat_class(self):
        import time
        node = self._node()
        node._on_fix_status(self._status('solution=NARROW_INT'))
        node._rtk_t = time.monotonic() - 100.0
        # stale 'fixed' must not be trusted: GBAS -> float floor
        node._on_fix(_fix(status=NavSatStatus.STATUS_GBAS_FIX,
                          covariance=_diag_covariance(0.01)))
        assert node.fix_pub.msgs[0].position_covariance[0] == pytest.approx(0.16)
        # and a plain fix falls back to single -> dropped
        node._on_fix(_fix(status=NavSatStatus.STATUS_FIX,
                          covariance=_diag_covariance(0.01)))
        assert len(node.fix_pub.msgs) == 1

    def test_stale_text_status_falls_back(self):
        node = self._node()
        node._on_fix_status(self._status('STALE no fix'))
        node._on_fix(_fix(status=NavSatStatus.STATUS_GBAS_FIX,
                          covariance=_diag_covariance(0.01)))
        assert node.fix_pub.msgs[0].position_covariance[0] == pytest.approx(0.16)

    def test_loose_single_policy_lets_single_through(self):
        node = self._node(single_policy="loose")
        node._on_fix_status(self._status('solution=SINGLE'))
        node._on_fix(_fix(covariance=_diag_covariance(0.01)))
        assert node.fix_pub.msgs[0].position_covariance[0] == pytest.approx(9.0)


class TestMain:
    def test_main_is_importable(self):
        from mower_localization.gps_gate import main

        assert callable(main)
