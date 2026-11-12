# Copyright 2026 michael
# SPDX-License-Identifier: Apache-2.0
"""Narrow-strip detector and shuttle planner (strip.py) on the real map's polygons."""

import math
import os

import numpy as np
import pytest

from mower_coverage_bridge import splitter, strip

WORLDS = os.path.join(os.path.dirname(__file__), '..', '..', 'mower_sim', 'worlds')


def load_areas(name):
    """{area name: polygon} of the mowing (not navigation) areas of a sim areas file."""
    d = {}
    with open(os.path.join(WORLDS, name)) as fh:
        for line in fh:
            if ':' in line:
                k, v = line.split(':', 1)
                d[k.strip()] = v.strip()
    out = {}
    for i in range(int(d['area_count'])):
        if d.get('area_%d_is_navigation' % i) == '1':
            continue
        out[d['area_%d_name' % i]] = [tuple(map(float, q.split(',')))
                                      for q in d['area_%d_polygon' % i].split(';')]
    return out


# owner_far_areas.dat "Area 4" = the live map's Area 4 (map area 1), field 2026-11-13
STRIP = load_areas('owner_far_areas.dat')['Area 4']
LAWNS = [('real_run5_areas.dat', 'Area 1'), ('real_run5_smooth_areas.dat', 'Area 1'),
         ('owner_far_areas.dat', 'heart'), ('owner_far_areas.dat', 'Area 5'),
         ('owner_far_areas.dat', 'Area 6')]


def test_detector_fires_on_the_field_strip():
    shape = strip.measure(STRIP)
    narrow, why = strip.detect(shape)
    assert narrow, why
    assert 0.33 < shape.inscribed_r < 0.37            # 0.72 m at its widest
    assert 6.1 < shape.length < 6.4
    assert abs(((shape.axis_deg - 17.7) + 90.0) % 180.0 - 90.0) < 1.5


@pytest.mark.parametrize('dat, name', LAWNS)
def test_detector_stays_off_on_the_lawns(dat, name):
    narrow, why = strip.detect(strip.measure(load_areas(dat)[name]))
    assert not narrow, why


def test_detector_needs_elongation_and_length():
    p = strip.StripParams()
    short = [(0, 0), (1.2, 0), (1.2, 0.5), (0, 0.5)]           # narrow but 1.2 m long
    assert not strip.detect(strip.measure(short), p)[0]
    blob = [(0, 0), (1.5, 0), (1.5, 0.7), (0, 0.7)]            # narrow but stubby
    assert 'not elongated' in strip.detect(strip.measure(blob), p)[1]


def body_ok(ring, pose, p):
    """Independent check of the soft body limits at one pose (point-outside distances)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [('nose', p.front_m, 0.0), ('tail', -p.rear_m, 0.0),
           ('front', p.front_m, p.half_width_m), ('front', p.front_m, -p.half_width_m),
           ('side', -p.rear_m, p.half_width_m), ('side', -p.rear_m, -p.half_width_m)]
    worst = 0.0
    for kind, bx, by in pts:
        px, py = x + c * bx - s * by, y + s * bx + c * by
        out = 0.0 if splitter.point_in_polygon((px, py), ring) else min(
            _seg(px, py, ring[i - 1], ring[i]) for i in range(len(ring)))
        worst = max(worst, out - p.out_limits[kind])
    return worst


def _seg(px, py, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / L2))
    return math.hypot(a[0] + t * dx - px, a[1] + t * dy - py)


@pytest.fixture(scope='module')
def strip_plan():
    return strip.plan_strip(STRIP, extra_axes_deg=[17.74])


def test_strip_plan_covers_the_strip(strip_plan):
    pl = strip_plan
    assert pl.ok, pl.message
    assert pl.message.startswith(strip.STRIP_TAG)
    assert pl.cut_fraction >= 0.90, pl.message      # Fields2Cover: 0.315
    assert 3 <= len(pl.passes) <= 6


def test_strip_plan_is_a_shuttle_without_pivots_or_transits(strip_plan):
    res = strip_plan.split()
    subs = res.subpaths
    assert subs[0].reverse is False
    # forward pass, reverse retrace, forward, reverse ...; each starts where the last ended
    assert [s.reverse for s in subs] == [k % 2 == 1 for k in range(len(subs))]
    assert splitter.transit_gaps(subs) == pytest.approx([0.0] * (len(subs) - 1), abs=1e-6)
    # a reverse leg runs straight back along its pass's line (the last one: along the first
    # pass's line, after the return lane change ended within a degree of it)
    for fwd, rev in zip(subs[0:-2:2], subs[1:-2:2]):
        assert len(rev.points) == 2
        a, b = rev.points
        h = math.atan2(a[1] - b[1], a[0] - b[0])
        last = math.atan2(fwd.points[-1][1] - fwd.points[-2][1],
                          fwd.points[-1][0] - fwd.points[-2][0])
        assert abs(math.sin(h - last)) < 1e-6 and math.cos(h - last) > 0
    # no pivot anywhere: consecutive forward headings differ by little (lane changes)
    for s in subs[0::2]:
        hs = [math.atan2(q[1] - p[1], q[0] - p[0]) for p, q in zip(s.points, s.points[1:])
              if math.hypot(q[0] - p[0], q[1] - p[1]) > 1e-6]
        for h0, h1 in zip(hs, hs[1:]):
            assert abs(math.degrees(math.atan2(math.sin(h1 - h0), math.cos(h1 - h0)))) < 15.0
    # the plan ends where it began: the pose the inbound transit reached (on the route
    # graph), so leaving the strip needs no in-area transit (sim 2026-11-13)
    entry = subs[0].points[0]
    assert math.hypot(subs[-1].points[-1][0] - entry[0], subs[-1].points[-1][1] - entry[1]) \
        < 1e-6


def test_strip_plan_keeps_the_body_within_the_soft_limits(strip_plan):
    p = strip.StripParams()
    ring = [tuple(q) for q in STRIP]
    for s in strip_plan.split().subpaths:
        pts = splitter.densify(s.points, 0.05)
        # body heading (yaws faces against the travel on a reverse leg); base_link inside
        for (x, y), yaw in zip(pts, splitter.yaws(pts, s.reverse)):
            assert body_ok(ring, (x, y, yaw), p) <= 0.01, (x, y)
            assert splitter.point_in_polygon((x, y), ring) or min(
                _seg(x, y, ring[i - 1], ring[i]) for i in range(len(ring))) < 0.01, (x, y)


def test_too_narrow_strip_is_refused_with_a_reason():
    # 0.25 m wide: the nose / tail / side limits leave no pass of 0.7 m
    thin = [(0, 0), (0.6, 0), (0.6, 0.25), (0, 0.25)]
    pl = strip.plan_strip(thin)
    assert not pl.ok and 'no pass' in pl.message


def test_split_reverse_shares_cusps_for_the_mission():
    pl = strip.plan_strip(STRIP)
    res = pl.split()
    assert res.mode == strip.MODE_STRIP
    assert all(seg.kind == splitter.SEGMENT_SWATH for seg in res.segments)
    for a, b in zip(res.subpaths, res.subpaths[1:]):
        assert np.allclose(a.points[-1], b.points[0])


def test_drop_fragments_keeps_joined_pieces_and_reverse_legs():
    S = splitter.SubPath
    subs = [S([(0, 0), (5, 0)]), S([(6, 0), (6.2, 0)]),                    # isolated 0.2
            S([(8, 0), (9, 0)]), S([(9, 0), (8.8, 0)], reverse=True),     # joined pair
            S([(8.8, 0), (8.9, 0)])]                                       # joined 0.1
    kept, dropped = splitter.drop_fragments(subs, 0.7)
    assert dropped == pytest.approx([0.2])
    assert len(kept) == 4
    assert splitter.drop_fragments(subs, 0.0)[1] == []
