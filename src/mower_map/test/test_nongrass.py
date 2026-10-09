# SPDX-License-Identifier: GPL-3.0-or-later
"""Non-grass memory (mower_map.nongrass, 2026-10-09): temporal/viewpoint confirmation, soft
halo cost, clusters, dismiss, persistence and PointCloud2 decoding. Pure, no ROS."""
import struct
import types

import numpy as np
import pytest

from mower_map import areas as core
from mower_map import nongrass as ng

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
DAY = 86400.0
X, Y = 5.05, 5.05          # centre of one cell


def _area(polygon=SQUARE, **params):
    return ng.AreaNonGrass('Lawn', polygon, params, t=1000.0)


def _vote(a, xs=(X,), ys=(Y,), nongrass=True, vx=0.0, vy=0.0, samples=None):
    n = len(xs)
    ngv = np.full(n, nongrass) if np.isscalar(nongrass) else nongrass
    return a.observe(np.array(xs), np.array(ys), ngv, samples, vx, vy)


def _confirm(a, n=4, **kw):
    """n non-grass frames from viewpoints 0.2 m apart (spread 0.6 for n=4)."""
    for k in range(n):
        _vote(a, vx=0.2 * k, **kw)
    return a.update_confirmed()


def _cell(a, x=X, y=Y):
    r, c, ok = a.raster.cells([x], [y])
    assert ok[0]
    return int(r[0]), int(c[0])


def test_single_frame_never_confirms():
    """One frame (even many cells) is never enough evidence."""
    a = _area()
    _vote(a, nongrass=True, vx=1.0, vy=1.0)
    assert not a.update_confirmed()
    assert a.cost().max() == 0 and a.confirmed_mask().sum() == 0


def test_same_viewpoint_never_confirms():
    """A parked robot / fixed shadow sees it from one spot: spread stays 0."""
    a = _area()
    for _ in range(30):
        _vote(a, vx=2.0, vy=2.0)
    r, c = _cell(a)
    assert a.raster.hit[r, c] == pytest.approx(30.0) and a.raster.spread[r, c] == 0.0
    assert not a.update_confirmed() and a.cost().max() == 0


def test_spread_viewpoints_confirm_with_cost_max_not_lethal():
    """Hits from >= 0.5 m apart confirm; cost is the soft cost_max, never lethal."""
    a = _area()
    assert _confirm(a, 4)
    r, c = _cell(a)
    cost = a.cost()
    assert cost[r, c] == 75 == a.p['cost_max'] and cost.max() < 100
    assert a.confirmed_mask()[r, c]


def test_spread_just_below_threshold_does_not_confirm():
    """Viewpoints 0.4 m apart (< 0.5) with plenty of hits stay unconfirmed."""
    a = _area()
    for k in range(10):
        _vote(a, vx=0.4 * (k % 2))
    assert not a.update_confirmed()


def test_too_few_hits_do_not_confirm():
    """Wide spread but only 2 frames (< confirm_hits 3) is not confirmed."""
    a = _area()
    _vote(a, vx=0.0)
    _vote(a, vx=1.0)
    assert not a.update_confirmed()


def test_margin_cells_cost_zero_even_if_confirmed():
    """Votes in the 0.5 m raster margin outside the polygon are kept but never costed."""
    a = _area()
    assert _confirm(a, 4, xs=(-0.25,), ys=(5.05,))
    r, c = _cell(a, -0.25, 5.05)
    assert a.raster.confirmed[r, c] == 1.0
    assert not a.confirmed_mask()[r, c] and a.cost().max() == 0


def test_alternating_grass_prevents_confirmation():
    """A shadow that comes and goes (ratio 0.5 < 0.75) is never confirmed."""
    a = _area()
    for k in range(20):
        _vote(a, vx=0.3 * k, nongrass=(k % 2 == 0))
    assert a.raster.spread.max() >= 0.5
    assert not a.update_confirmed() and a.cost().max() == 0


def test_hysteresis_keeps_then_releases():
    """Ratio between release (0.5) and confirm (0.75) keeps it; below 0.5 releases it."""
    a = _area()
    _confirm(a, 4)                                # hit 4, miss 0
    r, c = _cell(a)
    _vote(a, nongrass=False)
    _vote(a, nongrass=False)                      # 4/6 = 0.67
    assert not a.update_confirmed() and a.raster.confirmed[r, c] == 1.0
    assert a.cost()[r, c] == 75
    for _ in range(3):
        _vote(a, nongrass=False)                  # 4/9 = 0.44 < 0.5
    assert a.update_confirmed() and a.raster.confirmed[r, c] == 0.0
    assert a.cost().max() == 0


def test_release_on_low_hits():
    """Decay below release_hits (1.5) releases even with a perfect ratio."""
    a = _area()
    _confirm(a, 4)
    a.decay(1000.0 + 14 * DAY)                    # hit 4 -> 2: still held
    assert not a.update_confirmed()
    a.decay(1000.0 + 28 * DAY)                    # hit -> 1 < 1.5
    assert a.update_confirmed() and a.cost().max() == 0


def test_low_sample_votes_weigh_half():
    """samples=1 of full_weight_samples=2 counts 0.5, so 4 frames only give hit 2."""
    a = _area()
    for k in range(4):
        _vote(a, vx=0.2 * k, samples=np.array([1.0]))
    r, c = _cell(a)
    assert a.raster.hit[r, c] == pytest.approx(2.0)
    assert not a.update_confirmed()
    for k in range(2):
        _vote(a, vx=0.2 * k, samples=np.array([1.0]))
    assert a.update_confirmed()                   # hit 3.0 reached
    _vote(a, samples=np.array([5.0]))             # capped at weight 1
    assert a.raster.hit[r, c] == pytest.approx(4.0)


def test_decay_halves_and_long_decay_resets_viewpoints():
    """14 d half-life; once evidence is gone the viewpoint memory restarts."""
    a = _area()
    _confirm(a, 4)
    _vote(a, nongrass=False)
    r, c = _cell(a)
    a.decay(1000.0 + 14 * DAY)
    assert a.raster.hit[r, c] == pytest.approx(2.0) and a.raster.miss[r, c] == pytest.approx(0.5)
    assert a.raster.spread[r, c] > 0
    a.decay(1000.0 + 2000 * DAY)
    assert np.isnan(a.raster.vx0[r, c]) and np.isnan(a.raster.vy0[r, c])
    assert a.raster.spread[r, c] == 0.0
    assert a.update_confirmed() and a.raster.confirmed.sum() == 0
    a.decay(500.0)                                # going back in time is a no-op
    assert a.raster.decayed_at == pytest.approx(1000.0 + 2000 * DAY)


def test_cost_disabled_is_zero():
    """An area with avoid_non_grass off costs nothing even when confirmed."""
    a = _area()
    _confirm(a, 4)
    z = a.cost(enabled=False)
    assert z.shape == a.raster.hit.shape and z.max() == 0 and a.cost().max() == 75


def test_halo_around_single_confirmed_cell():
    """Neighbours: 50 at 0.1 m, 25 at 0.2 m (round(75*(1-d/0.3))), nothing further."""
    a = _area()
    _confirm(a, 4)
    r, c = _cell(a)
    cost = a.cost()
    assert cost[r, c] == 75
    for dr, dc in ((0, 1), (1, 0), (0, -1), (-1, 0)):
        assert cost[r + dr, c + dc] == 50
    for dr, dc in ((0, 2), (2, 0), (0, -2), (-2, 0)):
        assert cost[r + dr, c + dc] == 25
    assert cost[r + 1, c + 1] == round(75 * (1 - 0.1 * 2 ** 0.5 / 0.3))   # 40
    assert cost[r + 2, c + 1] == 0 and cost[r, c + 3] == 0
    nz = np.argwhere(cost > 0)
    assert np.abs(nz - [r, c]).max() == 2


def test_halo_cost_direct_edges_and_overlap():
    """Edge cells do not crash or wrap; overlapping halos take the max; empty -> zeros."""
    m = np.zeros((5, 5), bool)
    m[0, 0] = m[4, 4] = True
    out = ng.halo_cost(m, 75, 0.2, 0.1)
    assert out.dtype == np.int16 and out[0, 0] == 75 and out[4, 4] == 75
    assert out[0, 1] == 50 and out[0, 2] == 25 and out[4, 3] == 50 and out[3, 4] == 50
    assert out[0, 4] == 0 and out[4, 0] == 0                   # no wrap-around
    m2 = np.zeros((1, 5), bool)
    m2[0, 0] = m2[0, 2] = True
    assert ng.halo_cost(m2, 75, 0.2, 0.1)[0, 1] == 50
    assert ng.halo_cost(np.zeros((3, 3), bool), 75, 0.2, 0.1).max() == 0
    flat = ng.halo_cost(m, 75, 0.0, 0.1)
    assert flat.sum() == 150


def test_grid_unseen_probability_and_confirmed_floor():
    """-1 never seen, 0..100 elsewhere, confirmed cells >= 50."""
    a = _area()
    _vote(a, xs=(1.05,), ys=(1.05,), nongrass=False)             # grass only
    _confirm(a, 4)
    for _ in range(3):
        _vote(a, nongrass=False)                                 # 4/7 = 57 %, still held
    a.update_confirmed()
    g = a.grid()
    assert g[0, 0] == -1 and g[_cell(a, 1.05, 1.05)] == 0
    r, c = _cell(a)
    assert g[r, c] == 57 and a.raster.confirmed[r, c] == 1.0
    # ratio below 50 % but still confirmed (hysteresis not yet evaluated) is lifted to 50
    a.raster.miss[r, c] = 10.0
    assert a.grid()[r, c] == 50
    assert g.min() == -1 and g.max() <= 100


def _blob(a, cells_xy, n=4):
    xs = [p[0] for p in cells_xy]
    ys = [p[1] for p in cells_xy]
    for k in range(n):
        a.observe(np.array(xs), np.array(ys), np.ones(len(xs), bool), None, 0.2 * k, 0.0)
    a.update_confirmed()


def test_clusters_two_blobs_ids_area_keepout_hull():
    """Stable id = smallest flat index, area = cells*0.01, suggest only >= 0.25 m2."""
    a = _area()
    big = [(2.05 + 0.1 * i, 2.05 + 0.1 * j) for i in range(5) for j in range(5)]   # 25 cells
    small = [(7.05, 7.05), (7.15, 7.05)]
    _blob(a, big + small)
    cl = sorted(a.clusters(), key=lambda c: c['id'])
    assert len(cl) == 2
    b, s = cl
    r0, c0 = _cell(a, 2.05, 2.05)
    assert b['id'] == r0 * a.raster.width + c0
    assert b['cells'] == 25 and b['area_m2'] == pytest.approx(0.25) and b['suggest_keepout']
    assert s['cells'] == 2 and s['area_m2'] == pytest.approx(0.02) and not s['suggest_keepout']
    assert b['id'] < s['id']
    for c in cl:
        assert len(c['hull']) >= 3
        assert core.point_in_polygon(c['x'], c['y'], [tuple(p) for p in c['hull']])
    assert b['x'] == pytest.approx(2.25, abs=1e-3) and b['y'] == pytest.approx(2.25, abs=1e-3)
    assert ng.AreaNonGrass('E', SQUARE).clusters() == []


def test_single_cell_cluster_has_real_hull():
    """A lone cell still yields a polygon containing it (cell corners + margin)."""
    a = _area()
    _blob(a, [(X, Y)])
    (cl,) = a.clusters()
    assert len(cl['hull']) >= 4 and cl['cells'] == 1
    assert core.point_in_polygon(X, Y, [tuple(p) for p in cl['hull']])


def test_diagonal_cells_form_one_cluster():
    """Clusters are 8-connected."""
    a = _area()
    _blob(a, [(3.05, 3.05), (3.15, 3.15), (3.25, 3.25)])
    assert len(a.clusters()) == 1 and a.clusters()[0]['cells'] == 3
    assert len(ng.components(np.eye(4, dtype=bool))) == 1
    assert len(ng.components(np.array([[1, 0, 1]], bool))) == 2


def test_dismiss_clears_and_needs_many_new_frames():
    """Dismiss wipes hit, adds 20 grass, so re-confirming takes a lot of new votes."""
    a = _area()
    _confirm(a, 4)
    r, c = _cell(a)
    (cl,) = a.clusters()
    assert a.dismiss(cl['id'])
    rs = a.raster
    assert rs.confirmed[r, c] == 0 and rs.hit[r, c] == 0 and rs.miss[r, c] == 20.0
    assert np.isnan(rs.vx0[r, c]) and rs.spread[r, c] == 0
    assert a.cost().max() == 0 and a.dirty
    _confirm(a, 10)
    assert a.raster.confirmed[r, c] == 0
    for k in range(70):
        _vote(a, vx=0.1 * k)
    assert a.update_confirmed() and a.raster.confirmed[r, c] == 1.0


def test_dismiss_unknown_id():
    """Unknown id (and an unconfirmed cell's flat index) is refused, nothing changes."""
    a = _area()
    _confirm(a, 4)
    assert not a.dismiss(123456789)
    assert a.raster.confirmed.sum() == 1


def test_save_load_round_trip_keeps_nan(tmp_path):
    """npz keeps every layer including the NaN viewpoints."""
    a = _area()
    _confirm(a, 4)
    a.save(str(tmp_path / 'sub'))
    assert not a.dirty and (tmp_path / 'sub' / 'nongrass_Lawn.npz').exists()
    b = ng.AreaNonGrass.load(str(tmp_path / 'sub'), 'Lawn', SQUARE, None, 1000.0)
    assert getattr(b, 'load_error', None) is None
    for name in ng.NonGrassRaster.LAYERS:
        np.testing.assert_array_equal(getattr(b.raster, name), getattr(a.raster, name))
    assert np.isnan(b.raster.vx0).any() and (b.cost() == a.cost()).all()


def test_load_grown_polygon_adopts_overlap(tmp_path):
    """A resized area keeps the evidence on the overlapping cells."""
    a = _area()
    _confirm(a, 4)
    a.save(str(tmp_path))
    grown = [(-2.0, 0.0), (10.0, 0.0), (10.0, 10.0), (-2.0, 10.0)]
    b = ng.AreaNonGrass.load(str(tmp_path), 'Lawn', grown, None, 1000.0)
    assert b.raster.width > a.raster.width
    r, c = _cell(b)
    assert b.raster.hit[r, c] == pytest.approx(4.0) and b.raster.confirmed[r, c] == 1.0
    assert b.raster.hit.sum() == pytest.approx(4.0)


def test_load_missing_is_fresh_corrupt_sets_error(tmp_path):
    """Missing file -> fresh; unreadable file -> fresh memory with load_error."""
    a = ng.AreaNonGrass.load(str(tmp_path), 'Lawn', SQUARE, None, 1.0)
    assert a.raster.hit.sum() == 0 and not hasattr(a, 'load_error')
    (tmp_path / 'nongrass_Lawn.npz').write_bytes(b'this is not an npz')
    b = ng.AreaNonGrass.load(str(tmp_path), 'Lawn', SQUARE, None, 1.0)
    assert b.load_error and b.raster.hit.sum() == 0 and np.isnan(b.raster.vx0).all()


def test_clear_wipes_evidence_keeps_geometry():
    """Clear resets every layer (NaN viewpoints too) on the same grid."""
    a = _area()
    _confirm(a, 4)
    g = (a.raster.origin_x, a.raster.origin_y, a.raster.width, a.raster.height)
    a.clear()
    r = a.raster
    assert (r.origin_x, r.origin_y, r.width, r.height) == g
    assert r.hit.sum() == r.miss.sum() == r.confirmed.sum() == 0 and np.isnan(r.vx0).all()
    assert r.decayed_at == pytest.approx(1000.0) and a.dirty and a.cost().max() == 0


def test_observe_outside_raster_and_missing_viewpoint():
    """Off-raster points count 0; NaN viewpoints add hits but never spread."""
    a = _area()
    assert _vote(a, xs=(50.0,), ys=(50.0,)) == 0 and not a.dirty
    for _ in range(10):
        a.observe([X], [Y], [True], None, None, None)
    r, c = _cell(a)
    assert a.raster.hit[r, c] == 10 and a.raster.spread[r, c] == 0
    assert not a.update_confirmed()
    s = a.summary()
    assert s['area'] == 'Lawn' and s['enabled'] and s['confirmed_cells'] == 0
    assert s['observed_cells'] == 1


# ---- cloud_fields
def _fields(names=('x', 'y', 'z', 'nongrass', 'weight', 'vx', 'vy'), datatype=7, step=4):
    return [types.SimpleNamespace(name=n, offset=step * i, datatype=datatype)
            for i, n in enumerate(names)]


def _buf(rows, nf=7):
    return b''.join(struct.pack('<%df' % nf, *r) for r in rows)


def test_cloud_fields_decodes_little_endian_float32():
    """Fields are picked by offset, extra z ignored, values round-trip."""
    rows = [(1.0, 2.0, 9.0, 1.0, 2.0, 0.5, 0.25), (3.0, 4.0, 9.0, 0.0, 1.0, 0.75, 0.125)]
    f = ng.cloud_fields(_fields(), _buf(rows), 28, 2)
    assert set(f) == {'x', 'y', 'nongrass', 'weight', 'vx', 'vy'}
    assert f['x'].tolist() == [1.0, 3.0] and f['y'].tolist() == [2.0, 4.0]
    assert f['nongrass'].tolist() == [1.0, 0.0] and f['weight'].tolist() == [2.0, 1.0]
    assert f['vx'].tolist() == [0.5, 0.75] and f['vy'].tolist() == [0.25, 0.125]
    assert f['x'].dtype == np.float32
    # trailing padding (point_step > fields) and extra bytes
    padded = b''.join(struct.pack('<7f', *r) + b'\0' * 4 for r in rows) + b'\0' * 3
    assert ng.cloud_fields(_fields(), padded, 32, 2)['vy'].tolist() == [0.25, 0.125]


def test_cloud_fields_rejects_bad_input():
    """Missing field / non-float32 / short buffer -> None; width 0 -> empty arrays."""
    rows = [(1.0,) * 7]
    assert ng.cloud_fields(_fields(names=('x', 'y', 'z', 'nongrass', 'weight', 'vx', 'wrong')),
                           _buf(rows), 28, 1) is None
    assert ng.cloud_fields(_fields(datatype=8), _buf(rows), 28, 1) is None
    assert ng.cloud_fields(_fields(), _buf(rows), 28, 2) is None
    e = ng.cloud_fields(_fields(), b'', 28, 0)
    assert all(len(v) == 0 for v in e.values()) and len(e) == 6
