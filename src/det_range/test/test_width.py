# SPDX-License-Identifier: GPL-3.0-or-later
"""Option B (2026-10-23 trunk incident): ranged boxes become a disc of their width."""
import math

import numpy as np
import pytest

from det_range.geometry import (DEFAULT_STATIC_CLASSES, StereoModel, best_label, box_edges_ref,
                                box_width_m, disc_points, object_disc)

M = StereoModel()


def test_box_width_pinhole():
    # the trunk: ~0.25 m wide at 1.5 m with fx ~ 370 px -> ~62 px
    assert box_width_m(62.0, 1.5, 372.0) == pytest.approx(0.25, abs=1e-3)
    assert box_width_m(-5.0, 1.0, 400.0) == 0.0


@pytest.mark.parametrize('z', [0.8, 1.5, 3.0])
def test_box_edges_span_the_box_width(z):
    fx = M.src_k[0]
    w_px = 80.0
    a, b = box_edges_ref(M, M.src_k[2], M.src_k[3], w_px, z)
    # both edges at depth z; their spacing = w_px * z / fx (small stereo rotation aside)
    assert a[2] == pytest.approx(z) and b[2] == pytest.approx(z)
    assert abs(b[0] - a[0]) == pytest.approx(box_width_m(w_px, z, fx), rel=0.03)


def test_disc_points_grid_and_centre():
    pts = disc_points(1.0, 2.0, 0.3, 0.2, 0.05)
    d = np.hypot(pts[:, 0] - 1.0, pts[:, 1] - 2.0)
    assert d.max() <= 0.2 + 1e-9
    assert np.any(d < 1e-9)                  # centre included
    assert np.allclose(pts[:, 2], 0.3)
    assert len(pts) >= int(math.pi * 0.2 ** 2 / 0.05 ** 2 * 0.8)
    assert len(disc_points(0.0, 0.0, 0.0, 0.0)) == 1


def test_object_disc_pushes_centre_behind_the_front_face():
    # camera at the origin looking along +x, front face at x = 2, 0.3 m wide
    disc = object_disc((2.0, 0.15, 0.2), (2.0, -0.15, 0.2), (0.0, 0.0), margin=0.10)
    assert disc.width == pytest.approx(0.3)
    assert disc.centre[0] == pytest.approx(2.15)      # half a width further away
    assert disc.centre[1] == pytest.approx(0.0)
    assert disc.radius == pytest.approx(0.25)         # width / 2 + margin
    # the disc covers the whole front face (its edges are inside)
    d = np.hypot(disc.points[:, 0] - 2.0, disc.points[:, 1] - 0.15)
    assert d.min() < 0.05
    # far wider than the single centroid cell it replaces (0.05 m costmap)
    assert np.ptp(disc.points[:, 1]) >= 0.45


def test_object_disc_width_capped():
    disc = object_disc((2.0, 3.0, 0.0), (2.0, -3.0, 0.0), (0.0, 0.0), max_width=1.5)
    assert disc.width == 1.5
    assert disc.radius == pytest.approx(0.85)


def test_static_classes_never_living():
    living = {'person', 'dog', 'cat', 'hedgehog', 'rabbit'}
    assert not living & set(DEFAULT_STATIC_CLASSES)
    assert 'trunk' in DEFAULT_STATIC_CLASSES
    assert best_label([('Trunk', 0.7), ('wood', 0.3)]) == 'trunk'
    assert best_label([]) == ''
