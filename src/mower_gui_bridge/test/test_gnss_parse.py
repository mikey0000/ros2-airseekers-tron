"""Unit tests for /fix_status parsing and GnssStatus derivation (no ROS)."""

import math
import os
import sys

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from mower_gui_bridge import state_machine as sm  # noqa: E402

FIXED_STR = 'um960=connected quality=RTK_FIXED sats=21/28 hdop=0.70 diff_age=1.0s age=0.10s'
FLOAT_STR = 'um960=connected quality=RTK_FLOAT sats=18/25 hdop=0.95 age=0.10s'
BESTNAV_STR = ('um960=connected solution=NARROW_INT sol_status=SOL_COMPUTED '
               'vel_type=DOPPLER_VELOCITY sats=30 hdop=0.60 age=0.05s')
DIAG_COV = [0.0004, 0.0, 0.0, 0.0, 0.0009, 0.0, 0.0, 0.0, 0.0016]


def test_parse_quality_sats_hdop():
    p = sm.parse_fix_status(FIXED_STR)
    assert p['connected'] is True
    assert p['solution'] == 'RTK_FIXED'
    assert p['sats_used'] == 21 and p['sats_total'] == 28
    assert math.isclose(p['hdop'], 0.70)


def test_parse_bestnav_solution_and_single_sats():
    p = sm.parse_fix_status(BESTNAV_STR)
    assert p['solution'] == 'NARROW_INT'
    assert p['sats_used'] == 30 and p['sats_total'] == 30


def test_parse_empty_and_no_fix_yet():
    assert sm.parse_fix_status('')['solution'] is None
    p = sm.parse_fix_status('um960=disconnected no fix yet')
    assert p['connected'] is False and p['solution'] is None and p['hdop'] is None


def test_classify_tokens():
    assert sm.classify_solution('RTK_FIXED') == sm.FIX_TYPE_RTK_FIXED
    assert sm.classify_solution('NARROW_INT') == sm.FIX_TYPE_RTK_FIXED
    assert sm.classify_solution('RTK_FLOAT') == sm.FIX_TYPE_RTK_FLOAT
    assert sm.classify_solution('NARROW_FLOAT') == sm.FIX_TYPE_RTK_FLOAT
    assert sm.classify_solution('SINGLE') == sm.FIX_TYPE_GPS_FIX
    assert sm.classify_solution('DGPS') == sm.FIX_TYPE_GPS_FIX
    assert sm.classify_solution('INVALID') == sm.FIX_TYPE_NO_FIX
    assert sm.classify_solution('DR') == sm.FIX_TYPE_DEAD_RECKONING
    assert sm.classify_solution('WEIRD') is None
    assert sm.classify_solution(None) is None


def test_rtk_fixed_from_string():
    g = sm.derive_gnss_status(sm.NAVSAT_GBAS_FIX, DIAG_COV, 2, sm.parse_fix_status(FIXED_STR))
    assert g['fix_type'] == sm.FIX_TYPE_RTK_FIXED
    assert g['fix_valid'] and g['rtk_mode'] == sm.RTK_MODE_FIXED
    assert g['quality_percent'] == 100.0
    assert g['satellites_used'] == 21 and g['satellites_visible'] == 28
    assert math.isclose(g['hdop'], 0.70)
    assert math.isclose(g['horizontal_accuracy_m'], 0.03)
    assert math.isclose(g['vertical_accuracy_m'], 0.04)
    for cap in (sm.CAP_RTK_MODE, sm.CAP_HDOP, sm.CAP_SATELLITES_USED,
                sm.CAP_HORIZONTAL_ACCURACY):
        assert g['capability_flags'] & cap
        assert g['value_flags'] & cap


def test_rtk_float_distinguished_from_fixed():
    g = sm.derive_gnss_status(sm.NAVSAT_GBAS_FIX, DIAG_COV, 2, sm.parse_fix_status(FLOAT_STR))
    assert g['fix_type'] == sm.FIX_TYPE_RTK_FLOAT
    assert g['rtk_mode'] == sm.RTK_MODE_FLOAT
    assert g['quality_percent'] == 80.0
    assert g['differential_corrections']


def test_gbas_without_string_is_conservative_float():
    g = sm.derive_gnss_status(sm.NAVSAT_GBAS_FIX, DIAG_COV, 2, None)
    assert g['fix_type'] == sm.FIX_TYPE_RTK_FLOAT
    # values that only the string provides are flagged as unknown
    assert not g['value_flags'] & sm.CAP_HDOP
    assert not g['value_flags'] & sm.CAP_SATELLITES_USED
    assert g['capability_flags'] & sm.CAP_HDOP


def test_plain_gps_fix():
    g = sm.derive_gnss_status(sm.NAVSAT_FIX, DIAG_COV, 2,
                              sm.parse_fix_status('um960=connected quality=GPS sats=9/12'))
    assert g['fix_type'] == sm.FIX_TYPE_GPS_FIX
    assert g['quality_percent'] == 40.0
    assert g['rtk_mode'] == sm.RTK_MODE_NONE
    assert not g['differential_corrections']


def test_navsat_no_fix_overrides_stale_rtk_label():
    g = sm.derive_gnss_status(sm.NAVSAT_NO_FIX, DIAG_COV, 2, sm.parse_fix_status(FIXED_STR))
    assert g['fix_type'] == sm.FIX_TYPE_NO_FIX
    assert not g['fix_valid'] and g['quality_percent'] == 0.0


def test_unknown_covariance_leaves_accuracy_unset():
    g = sm.derive_gnss_status(sm.NAVSAT_GBAS_FIX, [0.0] * 9, sm.COVARIANCE_TYPE_UNKNOWN,
                              sm.parse_fix_status(FIXED_STR))
    assert g['horizontal_accuracy_m'] == 0.0
    assert not g['value_flags'] & sm.CAP_HORIZONTAL_ACCURACY
    assert g['capability_flags'] & sm.CAP_HORIZONTAL_ACCURACY


def test_value_flags_subset_of_capability_flags():
    for status in (sm.NAVSAT_NO_FIX, sm.NAVSAT_FIX, sm.NAVSAT_GBAS_FIX):
        for text in ('', FIXED_STR, FLOAT_STR, BESTNAV_STR):
            g = sm.derive_gnss_status(status, DIAG_COV, 2, sm.parse_fix_status(text))
            assert g['value_flags'] & ~g['capability_flags'] == 0
