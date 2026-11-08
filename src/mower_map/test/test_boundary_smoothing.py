# SPDX-License-Identifier: GPL-3.0-or-later
"""Boundary smoothing (inner polygon hull with a distance tolerance): corners kept, the
tolerance bound, validity, inward-only. Synthetic shapes only."""

import math
import random

import pytest

from mower_map import areas as core
from mower_map import boundary_smoothing as bs


def bumpy_edge(a, b, bump, period=1.0, step=0.1, outward=True):
    """Points from a (inclusive) to b (exclusive) with a tent zig every ``period`` m, outward
    (right of a counter-clockwise edge) or inward."""
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
    nx, ny = (uy, -ux) if outward else (-uy, ux)
    out = []
    for j in range(int(round(L / step))):
        s = j * step
        off = bump * (1.0 - abs(((s % period) - period / 2) / (period / 2))) \
            if period / 2 < s < L - period / 2 else 0.0
        out.append((a[0] + ux * s + nx * off, a[1] + uy * s + ny * off))
    return out


def bumpy_ring(corners, bump, **kw):
    ring = []
    for k in range(len(corners)):
        ring += bumpy_edge(corners[k], corners[(k + 1) % len(corners)], bump, **kw)
    return ring


def densify(ring, step=0.01):
    out = []
    for i in range(len(ring)):
        a, b = ring[i], ring[(i + 1) % len(ring)]
        k = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
        out += [(a[0] + (b[0] - a[0]) * j / k, a[1] + (b[1] - a[1]) * j / k) for j in range(k)]
    return out


def dist_to_ring(p, ring):
    return min(bs.segment_distance(p, ring[i], ring[(i + 1) % len(ring)])
               for i in range(len(ring)))


def inside_or_on(p, ring, eps=1e-9):
    return core.point_in_polygon(p[0], p[1], ring) or dist_to_ring(p, ring) <= eps


def assert_guarantees(ring, res, tol):
    out = res.ring
    assert set(out) <= set(ring)                       # vertex subset: nothing moved
    assert bs.ring_is_simple(out)                      # valid
    assert (bs.signed_area(out) > 0) == (bs.signed_area(ring) > 0)   # same winding
    assert res.max_deviation_m <= tol + 1e-12
    for p in ring:                                     # recorded line within tol of result
        assert dist_to_ring(p, out) <= tol + 1e-9
    for p in densify(out):                             # result within tol of recorded line
        assert dist_to_ring(p, ring) <= tol + 1e-9
        assert inside_or_on(p, ring, 1e-7)             # inward-or-on
    assert res.area_after_m2 <= res.area_before_m2 + 1e-9


SQUARE = [(0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)]


def test_outward_zigs_within_tolerance_are_removed_and_corners_kept():
    ring = bumpy_ring(SQUARE, 0.06)
    res = bs.smooth_ring(ring, 0.08)
    assert sorted(res.ring) == sorted(SQUARE)
    assert res.max_deviation_m == pytest.approx(0.06, abs=1e-9)
    assert res.removed == len(ring) - 4
    assert_guarantees(ring, res, 0.08)


def test_zigs_larger_than_the_tolerance_are_only_blunted_by_the_tolerance():
    """A 12 cm zig at 8 cm tolerance: its tip may be cut by up to 8 cm, the zig stays."""
    ring = bumpy_ring(SQUARE, 0.12)
    res = bs.smooth_ring(ring, 0.08)
    peaks = [p for p in ring if dist_to_ring(p, SQUARE) > 0.11]
    assert peaks and all(dist_to_ring(p, res.ring) <= 0.08 + 1e-9 for p in peaks)
    assert max(dist_to_ring(p, SQUARE) for p in res.ring) >= 0.12 - 0.08 - 1e-9
    assert_guarantees(ring, res, 0.08)


def test_inward_zigs_pull_the_line_in_to_their_bottoms_never_out():
    """Inward dips: the line moves in to run through the dip bottoms (the inner envelope),
    never out across the recorded line."""
    ring = bumpy_ring(SQUARE, 0.05, outward=False)
    res = bs.smooth_ring(ring, 0.08)
    bottoms = [p for p in ring if dist_to_ring(p, SQUARE) > 0.0499]
    assert bottoms and all(dist_to_ring(p, res.ring) < 1e-9 for p in bottoms)
    assert res.max_deviation_m == pytest.approx(0.05, abs=1e-6)
    assert_guarantees(ring, res, 0.08)


def test_right_angle_corner_stays_a_sharp_corner():
    """An L-shape: the convex corners and the reflex corner stay exact vertices."""
    L = [(0.0, 0.0), (6.0, 0.0), (6.0, 2.0), (2.0, 2.0), (2.0, 6.0), (0.0, 6.0)]
    ring = bumpy_ring(L, 0.04)
    res = bs.smooth_ring(ring, 0.08)
    for c in L:
        assert c in res.ring
    assert len(res.ring) == len(L)
    assert_guarantees(ring, res, 0.08)


def test_small_bevelled_corner_is_cut_by_at_most_the_tolerance():
    """A corner apex closer than tol to the chord of its neighbours may go (a 5 cm bevel),
    never more: the corner stays a straight bevel, not an arc."""
    ring = [(0.0, 0.0), (4.0, 0.0), (4.0, 2.95), (3.95, 3.0), (0.0, 3.0)]
    res = bs.smooth_ring(ring, 0.08)
    assert_guarantees(ring, res, 0.08)
    assert len(res.ring) >= 4


@pytest.mark.parametrize('clockwise', [False, True])
def test_winding_is_kept(clockwise):
    ring = bumpy_ring(SQUARE, 0.06)
    if clockwise:
        ring = ring[::-1]
    res = bs.smooth_ring(ring, 0.08)
    assert (bs.signed_area(res.ring) < 0) == clockwise
    assert_guarantees(ring, res, 0.08)


def test_ear_rule_keeps_a_narrow_neck_valid():
    """Two lobes joined by a 6 cm neck: cutting the neck corners within the tolerance would
    cross the other side; the ear test refuses it."""
    ring = [(0.0, 0.0), (3.0, 0.0), (3.0, 1.0), (1.53, 1.0), (1.53, 1.5), (3.0, 1.5),
            (3.0, 2.5), (0.0, 2.5), (0.0, 1.5), (1.47, 1.5), (1.47, 1.0), (0.0, 1.0)]
    res = bs.smooth_ring(ring, 0.08)
    assert_guarantees(ring, res, 0.08)


@pytest.mark.parametrize('seed', range(25))
def test_random_noisy_rings_keep_every_guarantee(seed):
    rnd = random.Random(seed)
    n = rnd.randint(20, 120)
    ring = []
    for k in range(n):
        a = 2 * math.pi * k / n
        r = 5.0 + 1.5 * math.sin(3 * a + seed) + rnd.uniform(-0.12, 0.12)
        ring.append((r * math.cos(a), r * math.sin(a)))
    tol = rnd.choice([0.03, 0.05, 0.08, 0.15])
    res = bs.smooth_ring(ring, tol)
    assert_guarantees(ring, res, tol)


def test_self_crossing_ring_is_returned_unchanged():
    bow = [(0.0, 0.0), (2.0, 2.0), (2.0, 0.0), (0.0, 2.0)]
    res = bs.smooth_ring(bow, 0.08)
    assert res.ring == bow and res.removed == 0 and 'crosses itself' in res.reason


def test_tolerance_zero_is_off_and_input_is_not_changed():
    ring = bumpy_ring(SQUARE, 0.06)
    copy = list(ring)
    assert bs.smooth_ring(ring, 0.0).ring == ring
    bs.smooth_ring(ring, 0.08)
    assert ring == copy


def test_repeated_closing_vertex_and_duplicates_are_dropped():
    ring = [(0.0, 0.0), (0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)]
    res = bs.smooth_ring(ring, 0.08)
    assert res.ring == [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]


def test_wiggle_metric_is_scale_fixed():
    """wiggle_m resamples, so a straight edge cut into many vertices reads 0 either way."""
    def median(v):
        return sorted(v)[len(v) // 2]
    assert median(bs.wiggle_m(bumpy_ring(SQUARE, 0.0))) < 1e-9
    assert median(bs.wiggle_m(SQUARE)) < 1e-9
    zig = bumpy_ring(SQUARE, 0.06)
    assert median(bs.wiggle_m(zig)) > 0.02
    assert bs.excess_turning_deg(SQUARE) == pytest.approx(0.0, abs=1e-9)
    assert bs.excess_turning_deg(zig) > 100.0
