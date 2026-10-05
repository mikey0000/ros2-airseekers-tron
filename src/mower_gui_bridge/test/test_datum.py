"""Unit tests for the set_datum helpers (no ROS)."""

import math
import os
import re
import sys

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from mower_gui_bridge import datum as dt  # noqa: E402

ROBOT_YAML = """# header comment
mowgli:
  ros__parameters:
    # Map origin comment
    datum_lat: 0.000000000
    datum_lon: 0.000000000   # trailing comment

    # Dock pose
    dock_pose_x: 1.5
"""


def gui_parse(message):
    """Port of web/src/utils/datumGps.ts:25-35 (split on ',', parseFloat each)."""
    parts = message.split(',')
    if len(parts) != 2:
        return None

    def parse_float(s):
        m = re.match(r'\s*([-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?)', s)
        return float(m.group(1)) if m else float('nan')
    lat, lon = parse_float(parts[0]), parse_float(parts[1])
    if math.isfinite(lat) and math.isfinite(lon):
        return lat, lon
    return None


def test_reply_matches_gui_contract():
    msg = dt.format_reply(47.123456789, 8.987654321)
    assert msg == '47.123456789,8.987654321'
    assert gui_parse(msg) == (47.123456789, 8.987654321)


def test_reply_negative_coordinates():
    msg = dt.format_reply(-33.5, -70.25)
    assert gui_parse(msg) == (-33.5, -70.25)


def test_reply_warns_on_shift_but_stays_parseable():
    prev = (47.0, 8.0)
    new = (47.0001, 8.0)          # ~11 m north
    msg = dt.format_reply(new[0], new[1], prev)
    assert 'WARNING' in msg and '11.1 m' in msg
    assert msg.count(',') == 1
    assert gui_parse(msg) == new


def test_reply_no_warning_for_small_shift_or_unset_previous():
    assert 'WARNING' not in dt.format_reply(47.000001, 8.0, (47.0, 8.0))   # 0.11 m
    assert 'WARNING' not in dt.format_reply(47.1, 8.0, (0.0, 0.0))
    assert 'WARNING' not in dt.format_reply(47.1, 8.0, None)


def test_fix_usable():
    assert dt.fix_usable(0, 47.0, 8.0, 0.2, 2.0) == (True, '')
    assert dt.fix_usable(2, 47.0, 8.0, 0.2, 2.0)[0]
    assert not dt.fix_usable(None, None, None, None, 2.0)[0]
    assert 'no fix' in dt.fix_usable(-1, 47.0, 8.0, 0.2, 2.0)[1]
    assert 'stale' in dt.fix_usable(0, 47.0, 8.0, 5.0, 2.0)[1]
    assert not dt.fix_usable(0, 0.0, 0.0, 0.2, 2.0)[0]
    assert not dt.fix_usable(0, float('nan'), 8.0, 0.2, 2.0)[0]
    for ok, reason in (dt.fix_usable(-1, 1, 1, 0, 1), dt.fix_usable(0, 1, 1, 9, 1)):
        assert ',' not in reason


def test_distance():
    assert math.isclose(dt.distance_m(47.0, 8.0, 47.0001, 8.0), 11.12, rel_tol=1e-3)
    assert dt.distance_m(47.0, 8.0, 47.0, 8.0) == 0.0


def test_yaml_splice_keeps_comments_and_layout():
    out, ok = dt.splice_datum_yaml(ROBOT_YAML, 47.123456789, -8.5)
    assert ok
    assert '    datum_lat: 47.123456789\n' in out
    assert '    datum_lon: -8.500000000   # trailing comment\n' in out
    assert out.replace('47.123456789', '0.000000000').replace(
        '-8.500000000', '0.000000000') == ROBOT_YAML
    assert dt.read_yaml_datum(out) == (47.123456789, -8.5)


def test_yaml_splice_adds_missing_keys():
    text = 'mowgli:\n  ros__parameters:\n    dock_pose_x: 0.0\n'
    out, ok = dt.splice_datum_yaml(text, 1.25, 2.5)
    assert ok
    assert dt.read_yaml_datum(out) == (1.25, 2.5)
    assert out.endswith('    dock_pose_x: 0.0\n')
    _, ok = dt.splice_datum_yaml('foo: 1\n', 1.0, 2.0)
    assert not ok


def test_env_roundtrip_and_parse_variants():
    text = dt.format_datum_env(47.123456789, 8.5, '2026-10-06T12:00:00')
    assert 'DATUM_LAT=47.123456789\n' in text and 'DATUM_LON=8.500000000\n' in text
    assert dt.parse_datum_env(text) == (47.123456789, 8.5)
    assert dt.parse_datum_env('export DATUM_LAT="1.5"\nDATUM_LON = \'-2\'\n') == (1.5, -2.0)
    assert dt.parse_datum_env('DATUM_LAT=1.5\n') is None
    assert dt.parse_datum_env('DATUM_LAT=x\nDATUM_LON=1\n') is None


def test_write_text_atomic_follows_symlink(tmp_path):
    real = tmp_path / 'real.yaml'
    real.write_text(ROBOT_YAML)
    link = tmp_path / 'link.yaml'
    link.symlink_to(real)
    out, _ = dt.splice_datum_yaml(dt.read_text(str(link)), 1.0, 2.0)
    dt.write_text_atomic(str(link), out)
    assert link.is_symlink()
    assert dt.read_yaml_datum(real.read_text()) == (1.0, 2.0)
    env = tmp_path / 'sub' / 'datum.env'
    dt.write_text_atomic(str(env), dt.format_datum_env(1.0, 2.0))
    assert dt.parse_datum_env(env.read_text()) == (1.0, 2.0)
    assert dt.read_text(str(tmp_path / 'missing')) is None
