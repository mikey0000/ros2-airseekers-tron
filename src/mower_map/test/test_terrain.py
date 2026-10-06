# SPDX-License-Identifier: GPL-3.0-or-later
import math

import numpy as np
import pytest

from mower_map import terrain as T

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
DAY = 86400.0


def test_tilt_to_gradient_rep103():
    # nose down (pitch > 0) heading +x: ground falls forward -> dz/dx < 0
    gx, gy = T.tilt_to_gradient(0.0, math.radians(10), 0.0)
    assert gx == pytest.approx(-math.tan(math.radians(10)))
    assert gy == pytest.approx(0.0, abs=1e-12)
    # same slope seen while heading +y with the left side (=-x) up: dz/dx < 0 again
    gx2, gy2 = T.tilt_to_gradient(math.radians(10), 0.0, math.pi / 2)
    assert gx2 == pytest.approx(gx)
    assert gy2 == pytest.approx(0.0, abs=1e-12)


def test_rpy_roundtrip():
    r, p, y = 0.1, -0.2, 1.0
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2),
                              math.sin(p / 2), math.cos(y / 2), math.sin(y / 2))
    q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
         cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
    assert T.rpy_from_quaternion(*q) == pytest.approx((r, p, y))


def test_raster_snapped_and_index():
    r = T.TerrainRaster.for_polygon(SQUARE, 0.1, 1.0)
    assert (r.origin_x, r.origin_y) == (pytest.approx(-1.0), pytest.approx(-1.0))
    assert (r.width, r.height) == (120, 120)
    assert r.index(0.05, 0.05) == (10, 10)
    assert not r.contains(20, 20)


def test_splat_and_dismiss_restores():
    a = T.AreaTerrain('A', SQUARE, t=0)
    inc = a.add_incident('dig_stall', 5, 5, 0)
    s = a.raster.traction_score()
    r, c = a.raster.index(5, 5)
    assert s[r, c] > 60 and s[0, 0] == 0
    assert a.set_status([inc['id']], 'dismissed') == 1
    assert a.raster.bad.max() < 1e-5
    assert a.clusters() == []


def test_marker_kind_has_no_traction():
    a = T.AreaTerrain('A', SQUARE, t=0)
    a.add_incident('detour', 5, 5, 0)
    assert a.raster.bad.max() == 0
    assert len(a.clusters()) == 1


def test_decay_half_life_and_prune():
    a = T.AreaTerrain('A', SQUARE, {'bad_half_life_days': 10}, t=0)
    a.add_incident('dig_stall', 5, 5, 0)
    peak = a.raster.bad.max()
    a.decay(10 * DAY)
    assert a.raster.bad.max() == pytest.approx(peak / 2, rel=1e-4)
    assert a.incidents[0]['w'] == pytest.approx(0.5)
    a.decay(60 * DAY)           # 0.5 * 2^-5 < prune 0.05
    assert a.incidents == []


def test_confirmed_incident_not_pruned():
    a = T.AreaTerrain('A', SQUARE, {'bad_half_life_days': 1}, t=0)
    inc = a.add_incident('dig_stall', 5, 5, 0)
    a.set_status([inc['id']], 'confirmed')
    a.decay(30 * DAY)
    assert len(a.incidents) == 1


def test_clusters_and_hull():
    a = T.AreaTerrain('A', SQUARE, t=0)
    a.add_incident('dig_stall', 2, 2, 0)
    a.add_incident('follow_abort', 2.5, 2.2, 1)
    a.add_incident('dig_stall', 8, 8, 2)
    cl = a.clusters()
    assert [c['n'] for c in cl] == [2, 1]
    c = cl[0]
    assert c['id'] == 1 and c['ids'] == [1, 2]
    xs = [p[0] for p in c['hull']]
    assert min(xs) <= 2 - 0.4 + 1e-6 and max(xs) >= 2.5 + 0.4 - 1e-6
    assert len(c['hull']) >= 3


def _tilted(a, axis_deg, slope_deg, headings=(0, 90, 180, 270), bias=(0.0, 0.0)):
    """Drive the whole square at several headings over a plane falling along
    axis_deg (downhill direction) by slope_deg; bias = IMU (roll, pitch) offset."""
    g = math.tan(math.radians(slope_deg))
    ax = math.radians(axis_deg)
    gx, gy = -g * math.cos(ax), -g * math.sin(ax)     # gradient points uphill
    for h in headings:
        yaw = math.radians(h)
        c, s = math.cos(yaw), math.sin(yaw)
        dzdx_b = c * gx + s * gy
        dzdy_b = -s * gx + c * gy
        pitch, roll = -math.atan(dzdx_b) + bias[1], math.atan(dzdy_b) + bias[0]
        for x in np.arange(0.05, 10, 0.1):
            for y in np.arange(0.05, 10, 0.1):
                a.raster.add_tilt(x, y, *T.tilt_to_gradient(roll, pitch, yaw))


def test_slope_axis_and_bias_cancels():
    a = T.AreaTerrain('A', SQUARE, t=0)
    _tilted(a, 30, 12, bias=(math.radians(2), math.radians(-1.5)))
    ax = T.slope_axis(a.raster)
    assert ax['axis_deg'] == pytest.approx(30, abs=1.0)
    assert ax['slope_deg'] == pytest.approx(12, abs=0.6)
    assert ax['anisotropy'] > 0.9
    sl = a.raster.slope_deg()
    assert np.nanmax(sl) == pytest.approx(12, abs=0.6)


def test_choose_mow_angle_modes():
    ax = {'axis_deg': 30.0, 'slope_deg': 12.0, 'anisotropy': 0.9}
    assert T.choose_mow_angle(ax, 'updown')[0] == 30.0
    assert T.choose_mow_angle(ax, 'contour')[0] == 120.0
    assert T.choose_mow_angle(ax, 'auto', contour_above_deg=10)[0] == 120.0
    assert T.choose_mow_angle(ax, 'auto', contour_above_deg=15)[0] == 30.0
    assert T.choose_mow_angle(ax, 'off')[0] is None
    assert T.choose_mow_angle(None, 'auto')[0] is None
    assert T.choose_mow_angle(dict(ax, slope_deg=1.0), 'auto')[0] is None
    assert T.choose_mow_angle(dict(ax, anisotropy=0.1), 'auto')[0] is None
    # wrap
    assert T.choose_mow_angle(dict(ax, axis_deg=170.0), 'contour')[0] == 80.0


def test_slope_axis_needs_coverage():
    a = T.AreaTerrain('A', SQUARE, t=0)
    for k in range(10):
        a.raster.add_tilt(1 + 0.1 * k, 1, 0.1, 0.0)
    assert T.slope_axis(a.raster) is None


def test_persistence_roundtrip_and_regrid(tmp_path):
    a = T.AreaTerrain('Front lawn', SQUARE, t=0)
    a.add_incident('dig_stall', 3, 3, 5, 'x')
    a.raster.add_tilt(4, 4, 0.1, 0.2)
    a.save(str(tmp_path))
    assert (tmp_path / 'terrain_Front_lawn.npz').exists()
    assert (tmp_path / 'terrain_Front_lawn.json').exists()
    b = T.AreaTerrain.load(str(tmp_path), 'Front lawn', SQUARE)
    assert b.raster.same_grid(a.raster)
    assert np.allclose(b.raster.bad, a.raster.bad)
    assert b.incidents[0]['kind'] == 'dig_stall' and b.next_id == 2
    # area grown by 2 m to the left: cells keep their map position
    big = [(-2.0, 0.0), (10.0, 0.0), (10.0, 10.0), (-2.0, 10.0)]
    c = T.AreaTerrain.load(str(tmp_path), 'Front lawn', big)
    r, col = c.raster.index(4, 4)
    assert c.raster.gw[r, col] == 1.0 and c.raster.gy[r, col] == pytest.approx(0.2)


def test_load_corrupt_is_fresh(tmp_path):
    (tmp_path / 'terrain_A.json').write_text('{not json')
    a = T.AreaTerrain.load(str(tmp_path), 'A', SQUARE)
    assert a.incidents == [] and getattr(a, 'load_error', '')


def test_stamp_and_traction_cost():
    a = T.AreaTerrain('A', SQUARE, t=0)
    a.add_incident('dig_stall', 5, 5, 0, weight=5.0)
    grid = np.zeros((200, 200), np.int16)
    cost = T.traction_cost(a.raster.traction_score(), 20, 60)
    assert cost.max() == 60 - 0 or cost.max() >= 59
    T.stamp_into(grid, (-5.0, -5.0), 0.1, a.raster, cost)
    r = int(round((5 + 5) / 0.1))
    assert grid[r, r] >= 59 and grid[0, 0] == 0


def test_summary_fields():
    a = T.AreaTerrain('A', SQUARE, t=0)
    _tilted(a, 0, 5)
    s = a.summary('updown')
    assert s['slope_mow_angle_deg'] == pytest.approx(0.0, abs=1.0) or \
        s['slope_mow_angle_deg'] == pytest.approx(180.0, abs=1.0)
    assert s['slope']['slope_deg'] == pytest.approx(5, abs=0.5)


def test_stuck_incident_is_a_traction_kind():
    assert T.KINDS['stuck'][0] >= 0.8
    a = T.AreaTerrain('A', SQUARE, t=0)
    inc = a.add_incident('stuck', 5, 5, 0, 'stuck guard')
    assert inc['kind'] == 'stuck' and inc['w'] == T.KINDS['stuck'][0]
