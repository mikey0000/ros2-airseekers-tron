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
        assert _reject_reason(_fix(), 1, 100.0, frozenset()) is None

    def test_no_fix_is_rejected(self):
        reason = _reject_reason(
            _fix(status=NavSatStatus.STATUS_NO_FIX), 1, 100.0, frozenset())
        assert reason is not None
        assert "min_fix_status" in reason

    def test_status_threshold_is_inclusive(self):
        # STATUS_FIX (1) passes against min_fix_status 1, STATUS_NO_FIX
        # (0) does not.
        assert _reject_reason(
            _fix(status=1), 1, 100.0, frozenset()) is None
        assert _reject_reason(
            _fix(status=0), 1, 100.0, frozenset()) is not None

    def test_min_fix_status_is_configurable(self):
        # A deployment that demands at least DGPS.
        assert _reject_reason(
            _fix(status=NavSatStatus.STATUS_FIX), 2, 100.0, frozenset()) \
            is not None
        assert _reject_reason(
            _fix(status=NavSatStatus.STATUS_DGPS_FIX), 2, 100.0,
            frozenset()) is None

    def test_estimated_and_simulated_fixes_are_rejected(self):
        # ... by the default used_fixes whitelist: numerically they are
        # >= STATUS_FIX, but the stack does not localise from an
        # estimated, manual or simulated solution.
        used = frozenset(DEFAULT_USED_FIXES)
        for status in (NavSatStatus.STATUS_ESTIMATED,
                       NavSatStatus.STATUS_MANUAL,
                       NavSatStatus.STATUS_SIMULATION):
            assert _reject_reason(
                _fix(status=status), 1, 100.0, used) is not None

    def test_large_covariance_is_rejected(self):
        reason = _reject_reason(
            _fix(covariance=_diag_covariance(500.0)), 1, 100.0, frozenset())
        assert reason is not None
        assert "covariance" in reason

    def test_covariance_threshold_is_inclusive(self):
        assert _reject_reason(
            _fix(covariance=_diag_covariance(100.0)), 1, 100.0,
            frozenset()) is None

    def test_covariance_check_is_disabled_at_zero(self):
        assert _reject_reason(
            _fix(covariance=_diag_covariance(1e9)), 1, 0.0, frozenset()) \
            is None

    def test_unknown_covariance_type_passes(self):
        # The UM960 driver reports UNKNOWN when the receiver gave no
        # accuracy numbers; the gate cannot check what it does not have.
        msg = _fix(covariance=_diag_covariance(1e9),
                   covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN)
        assert _reject_reason(msg, 1, 100.0, frozenset()) is None
    def test_used_fixes_whitelist_filters(self):
        used = frozenset([NavSatStatus.STATUS_RTK_FIX])
        # Single-point fix is >= STATUS_FIX but not an RTK fix.
        assert _reject_reason(
            _fix(status=NavSatStatus.STATUS_FIX), 1, 100.0, used) is not None
        assert _reject_reason(
            _fix(status=NavSatStatus.STATUS_RTK_FIX), 1, 100.0,
            used) is None

    def test_empty_used_fixes_disables_the_whitelist(self):
        assert _reject_reason(
            _fix(status=NavSatStatus.STATUS_PPS_FIX), 1, 100.0,
            frozenset()) is None

    def test_default_used_fixes_cover_the_rtk_family(self):
        assert DEFAULT_USED_FIXES == (
            NavSatStatus.STATUS_FIX,
            NavSatStatus.STATUS_DGPS_FIX,
            NavSatStatus.STATUS_RTK_FIX,
            NavSatStatus.STATUS_RTK_FLOAT,
        )

    def test_reason_names_the_first_failure_only(self):
        # Failure-first precedence: status is checked before covariance.
        msg = _fix(status=NavSatStatus.STATUS_NO_FIX,
                   covariance=_diag_covariance(1e9))
        reason = _reject_reason(msg, 1, 100.0, frozenset())
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
    def _node(self, **overrides):
        parameters = [
            type("P", (), {"name": name, "value": value})()
            for name, value in overrides.items()
        ]
        return GpsGateNode(parameter_overrides=parameters)

    def test_fix_passes_through_unchanged(self):
        node = self._node()
        fix = _fix()
        node._on_fix(fix)
        assert node.pubs["/fix_gated"].msgs == [fix]
        assert node.received == 1
        assert node.passed == 1
        assert node.rejected == 0

    def test_no_fix_is_withheld(self):
        node = self._node()
        node._on_fix(_fix(status=NavSatStatus.STATUS_NO_FIX))
        assert node.pubs["/fix_gated"].msgs == []
        assert node.rejected == 1

    def test_large_covariance_is_withheld(self):
        node = self._node()
        node._on_fix(_fix(covariance=_diag_covariance(1000.0)))
        assert node.pubs["/fix_gated"].msgs == []

    def test_rtk_float_passes_by_default(self):
        node = self._node()
        node._on_fix(_fix(status=NavSatStatus.STATUS_RTK_FLOAT))
        assert len(node.pubs["/fix_gated"].msgs) == 1

    def test_used_fixes_param_tightens_the_gate(self):
        # RTK-fixed-only deployment.
        node = self._node(used_fixes=[NavSatStatus.STATUS_RTK_FIX])
        node._on_fix(_fix(status=NavSatStatus.STATUS_RTK_FLOAT))
        assert node.pubs["/fix_gated"].msgs == []
        node._on_fix(_fix(status=NavSatStatus.STATUS_RTK_FIX))
        assert len(node.pubs["/fix_gated"].msgs) == 1

    def test_covariance_param_tightens_the_gate(self):
        node = self._node(max_position_covariance=1.0)
        node._on_fix(_fix(covariance=_diag_covariance(4.0)))
        assert node.pubs["/fix_gated"].msgs == []
        node._on_fix(_fix(covariance=_diag_covariance(0.5)))
        assert len(node.pubs["/fix_gated"].msgs) == 1

    def test_topic_names_are_remappable(self):
        node = self._node(
            input_topic="/fix_src", output_topic="/fix_gated_tmcoffset")
        assert "/fix_src" in node.subs
        assert "/fix_gated_tmcoffset" in node.pubs

    def test_subscription_matches_driver_qos(self):
        # um960_gps_driver publishes /fix reliable, depth 10: the gate
        # must subscribe with a compatible profile or it sees nothing.
        node = self._node()
        subscription = node.subs["/fix"]
        assert subscription.qos["reliability"] == 1  # RELIABLE
        assert subscription.qos["depth"] == 10


class TestMain:
    def test_main_is_importable(self):
        from mower_localization.gps_gate import main

        assert callable(main)
