"""Tests for seg_ros.ground_projection (pure numpy; 2026-10-09)."""
import math

import numpy as np
import pytest

from seg_ros import ground_projection as gp
from seg_ros.ground_projection import (
    CAM1_D, CAM1_K, MotionGate, ProjectionParams, bin_cells, compose, depth_elevated,
    flat_ground_base, intersect_plane, invert, label_pixels, make_ray_grid, plane_agrees,
    plane_from_fit, project_frame, scale_intrinsics, src_in_ref, to_map, transform_plane,
    FrameVotes)

K = (100.0, 100.0, 50.0, 40.0)
SIZE = (100, 80)
H = 0.25
R_BC = np.array([[0.0, 0, 1], [-1, 0, 0], [0, -1, 0]])
T_BC = np.array([0.5, 0.0, H])
CAM = (R_BC, T_BC)
GROUND = flat_ground_base()
PARAMS = ProjectionParams()


def _grid():
    return make_ray_grid(SIZE, K, (), stride=8)


def _mask(cls):
    return np.full((SIZE[1], SIZE[0]), cls, np.uint8)


def _project(cls, conf=None, **kw):
    c = None if conf is None else np.full((SIZE[1], SIZE[0]), conf, np.uint8)
    return project_frame(_mask(cls), c, _grid(), CAM, GROUND, PARAMS, **kw)


# ------------------------------------------------------------------ intersect_plane
def test_intersect_plane_matches_pinhole_formula():
    """Ground distance from row v is the core of the whole projection; a sign/scale slip shifts every mark."""
    vs = np.array([45, 50, 60, 79])
    rays = np.stack([np.zeros(4), (vs - 40) / 100.0, np.ones(4)], axis=1)
    pts, ok = intersect_plane(CAM[1], rays @ R_BC.T, *GROUND)
    assert ok.all()
    assert np.allclose(pts[:, 0], 0.5 + H * 100.0 / (vs - 40), atol=1e-9)
    assert np.allclose(pts[:, 2], 0.0, atol=1e-12)


def test_intersect_plane_horizon_and_upward_rays_invalid():
    """Rays at/above the horizon never reach the ground; a hit there would be a far phantom mark."""
    vs = np.array([40, 30, 0])
    rays = np.stack([np.zeros(3), (vs - 40) / 100.0, np.ones(3)], axis=1)
    _, ok = intersect_plane(CAM[1], rays @ R_BC.T, *GROUND)
    assert not ok.any()


def test_intersect_plane_rays_pointing_away_invalid():
    """A ray looking backwards-and-up (s<0) must not be mirrored onto the ground behind the robot."""
    _, ok = intersect_plane(CAM[1], np.array([[-1.0, 0, 0.5], [0, 0, 1.0]]), *GROUND)
    assert not ok.any()


# ------------------------------------------------------------------ project_frame
def test_project_frame_nongrass_all_hits_in_range():
    """Confident non-grass pixels become hits only inside the range/lateral window."""
    v = _project(5)
    assert v.x.size > 0 and (v.vote == 1).all()
    rng = np.hypot(v.x - 0.5, v.y)
    assert ((rng >= PARAMS.min_range_m) & (rng <= PARAMS.max_range_m)).all()
    assert (np.abs(v.y) <= PARAMS.max_lateral_m).all()
    assert v.n_rays == _grid().u.size and v.n_ground >= v.x.size


@pytest.mark.parametrize('cls', [1, 3])
def test_project_frame_grass_and_soil_are_misses(cls):
    """Soil counts as grass: dry/worn patches must clear, never mark, non-grass."""
    v = _project(cls)
    assert v.x.size > 0 and (v.vote == 0).all()


def test_project_frame_confidence_gates():
    """Unsure pixels must be ignored: that is what keeps shadows from becoming marks."""
    assert _project(5, 150).x.size == 0            # 0.588 < 0.8
    assert (_project(5, 230).vote == 1).all()      # 0.90 >= 0.8
    assert (_project(1, 160).vote == 0).all() and _project(1, 160).x.size > 0   # 0.627 >= 0.6
    assert _project(1, 140).x.size == 0            # 0.549 < 0.6


def test_project_frame_rejects_mismatched_grid():
    """A mask of another size would index the wrong pixels silently; it must raise."""
    with pytest.raises(ValueError):
        project_frame(np.zeros((40, 50), np.uint8), None, _grid(), CAM, GROUND, PARAMS)


# ------------------------------------------------------------------ label_pixels
@pytest.mark.parametrize('cls,conf,want', [
    (0, 255, 1), (2, 255, 1), (4, 255, 1), (5, 255, 1),
    (1, 255, 0), (3, 255, 0),
    (9, 255, -1), (9, None, -1),
    (5, 203, -1), (5, 204, 1),
    (1, 153, 0), (1, 152, -1),
    (5, None, 1), (1, None, 0),
])
def test_label_pixels_table(cls, conf, want):
    """Vote semantics (hit/miss/ignore) are the contract with the non-grass memory."""
    c = None if conf is None else np.array([conf], np.uint8)
    assert label_pixels(np.array([cls]), c)[0] == want


def test_label_pixels_float_confidence():
    """Float 0..1 confidence must use the same thresholds as uint8."""
    out = label_pixels(np.array([5, 5, 1, 1, 9]), np.array([0.85, 0.7, 0.65, 0.5, 1.0]))
    assert out.tolist() == [1, -1, 0, -1, -1]


# ------------------------------------------------------------------ real calibration
def _rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _real_cam():
    return compose((_rpy(-1.5547956, 0, -1.5707963), np.array([0.466, 0, 0.240])), src_in_ref())


def test_real_calibration_right_eye_offset():
    """The colour camera is the RIGHT eye (y about -0.060 m); the stereo extrinsic sign matters."""
    assert _real_cam()[1][1] == pytest.approx(-0.060, abs=2e-3)


def test_real_calibration_bottom_centre_hits_near_ground():
    """The lowest image row must hit the ground just ahead of the mower (0.38..0.45 m)."""
    r, o = _real_cam()
    n = gp.undistort_points([[318, 479]], CAM1_K, CAM1_D)
    p, ok1 = intersect_plane(o, np.hstack([n, [[1.0]]]) @ r.T, *GROUND)
    assert ok1[0] and 0.38 <= p[0, 0] - o[0] <= 0.45


def test_real_calibration_centre_row_near_horizon():
    """The image centre row is near the horizon: it must not give a close-by ground hit."""
    r, o = _real_cam()
    n = gp.undistort_points([[318, 240]], CAM1_K, CAM1_D)
    p, ok = intersect_plane(o, np.hstack([n, [[1.0]]]) @ r.T, *GROUND)
    assert (not ok[0]) or p[0, 0] - o[0] > 5.0


# ------------------------------------------------------------------ live plane
def test_plane_from_fit_valid_and_invalid():
    """A failed or NaN RANSAC fit must fall back to the prior (None), never feed NaN downstream."""
    n, d = plane_from_fit([0, 1, 0, 0.25, 100, 1])
    assert np.allclose(n, [0, 1, 0]) and d == pytest.approx(0.25)
    assert plane_from_fit([0, 1, 0, 0.25, 100, 0]) is None
    assert plane_from_fit([0, 1, 0, float('nan'), 100, 1]) is None
    assert plane_from_fit([float('nan'), 1, 0, 0.25, 100, 1]) is None
    assert plane_from_fit([0, 0, 0, 0.25, 100, 1]) is None
    assert plane_from_fit(None) is None and plane_from_fit([0, 1]) is None


def test_transform_plane_optical_to_base_is_ground():
    """The live camera-height plane must land on z=0 in base_link, else live and URDF ground disagree."""
    n, d = plane_from_fit([0, 1, 0, 0.25, 100, 1])
    nb, db = transform_plane(n, d, CAM)
    if nb[2] < 0:
        nb, db = -nb, -db
    assert np.allclose(nb, [0, 0, 1], atol=1e-12) and db == pytest.approx(0.0, abs=1e-12)
    assert plane_agrees((nb, db), GROUND)


def test_plane_agrees_cases():
    """A tilted/offset live plane (wall, hedge) must be rejected; the same plane flipped accepted."""
    prior = (np.array([0.0, 0, 1]), 0.0)
    t = math.radians(10)
    assert plane_agrees(prior, prior)
    assert not plane_agrees((np.array([math.sin(t), 0, math.cos(t)]), 0.0), prior)
    assert not plane_agrees((np.array([0.0, 0, 1]), 0.1), prior)
    assert plane_agrees((np.array([0.0, 0, -1]), -0.0), prior)
    prior2 = (np.array([0.0, 0, 1]), 0.02)
    assert plane_agrees((np.array([0.0, 0, -1]), -0.02), prior2)
    assert not plane_agrees((np.array([0.0, 0, -1]), 0.1), prior2)   # flipped, 0.12 m apart


# ------------------------------------------------------------------ depth check
def _depth(val):
    return np.full((360, 640), val, np.float32)


P_AHEAD = np.array([[0.0, 0.1, 2.0]])


def test_depth_elevated_cases():
    """Only a clearly shorter valid depth marks 'something stands here'; invalid depth must not."""
    e, c = depth_elevated(P_AHEAD, _depth(1.0))
    assert e[0] and c[0]
    e, c = depth_elevated(P_AHEAD, _depth(2.02))
    assert not e[0] and c[0]
    e, c = depth_elevated(P_AHEAD, _depth(0.0))
    assert not e[0] and not c[0]
    e, c = depth_elevated(np.array([[5.0, 0.1, 1.0]]), _depth(0.5))
    assert not e[0] and not c[0]
    e, c = depth_elevated(P_AHEAD, None)
    assert not e.any() and not c.any()


def test_project_frame_drops_points_in_front_of_a_wall():
    """Points whose depth pixel sees a near wall are standing objects, not ground; others must stay."""
    base = _project(5)
    depth = np.zeros((SIZE[1], SIZE[0]), np.float32)
    depth[:, :50] = 0.3
    v = _project(5, depth=depth, base_to_ref=invert(CAM), ref_k=K)
    assert v.n_elevated > 0 and v.x.size == base.x.size - v.n_elevated and v.x.size > 0
    assert (v.y < 0).all()        # left image half (y>0 in base) is the wall, removed
    assert (base.y > 0).any()


# ------------------------------------------------------------------ map / bin / gate
def test_to_map_yaw90_translation():
    """base->map transform order (rotate then translate) decides where marks land in the map."""
    r = np.array([[0.0, -1, 0], [1, 0, 0], [0, 0, 1]])
    mx, my = to_map(FrameVotes(np.array([1.0]), np.array([0.0]), np.array([1], np.int8)),
                    (r, np.array([1.0, 2.0, 0.0])))
    assert mx[0] == pytest.approx(1.0) and my[0] == pytest.approx(3.0)


def test_bin_cells_majority_and_ties():
    """One vote per cell per frame; ties count as grass so a lone hit never marks."""
    cx, cy, ng, n = bin_cells([0.31, 0.32, 0.33], [0.01, 0.02, 0.03], [1, 1, 0])
    assert len(cx) == 1 and ng[0] and n[0] == 3
    assert cx[0] == pytest.approx(0.35) and cy[0] == pytest.approx(0.05)
    _, _, ng, n = bin_cells([0.31, 0.32], [0.01, 0.02], [1, 0])
    assert not ng[0] and n[0] == 2


def test_bin_cells_negative_and_empty():
    """floor (not int truncation) keeps cells around the origin from merging; empty input is safe."""
    cx, cy, _, _ = bin_cells([-0.05, 0.05], [-0.05, 0.05], [1, 1])
    assert sorted(np.round(cx, 6).tolist()) == [-0.05, 0.05]
    assert sorted(np.round(cy, 6).tolist()) == [-0.05, 0.05]
    out = bin_cells([], [], [])
    assert all(len(a) == 0 for a in out)


def test_motion_gate():
    """A parked robot must not pile up votes; yaw wrap must not look like a big turn."""
    g = MotionGate()
    assert g.accept(0, 0, 0.0)
    assert not g.accept(0, 0, 0.0)
    assert g.accept(0.2, 0, 0.0)
    assert g.accept(0.2, 0, math.radians(15))
    g2 = MotionGate()
    assert g2.accept(0, 0, math.radians(179))
    assert not g2.accept(0, 0, math.radians(-179))


def test_scale_intrinsics():
    """Half-resolution K must follow the pixel-centre convention (cx+0.5)*s-0.5."""
    fx, fy, cx, cy = scale_intrinsics((448, 448, 318, 238), (640, 480), (320, 240))
    assert fx == 224 and fy == 224
    assert cx == pytest.approx(318.5 * 0.5 - 0.5) and cy == pytest.approx(238.5 * 0.5 - 0.5)
