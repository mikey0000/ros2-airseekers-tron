"""Tests for camera_info_builder against the recovered Metoak calibration."""

import os
import unittest

import numpy as np

from stereo_vio_bridge.camera_info_builder import (
    camera_info_dict,
    distortion,
    intrinsics,
    load_camera,
    load_stereo,
    rodrigues,
)

CAL = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..',
    'ros2_port_handoff', '08_calibration_identity'))


class TestLoadCamera(unittest.TestCase):
    def test_cam0_values(self):
        c = load_camera(os.path.join(CAL, 'cam0.yaml'))
        self.assertEqual((c['width'], c['height']), (640, 480))
        self.assertAlmostEqual(c['fx'], 449.6371154785156, places=4)
        self.assertAlmostEqual(c['fy'], 449.66879272460938, places=4)
        self.assertAlmostEqual(c['cx'], 319.90609741210938, places=4)
        self.assertAlmostEqual(c['cy'], 240.66145324707031, places=4)
        self.assertAlmostEqual(c['k1'], -0.408391416, places=6)
        self.assertAlmostEqual(c['k2'], 0.1972861439, places=6)
        self.assertAlmostEqual(c['k3'], -0.0504755191, places=6)
        self.assertAlmostEqual(c['p1'], -7.9372643e-05, places=9)
        self.assertAlmostEqual(c['p2'], 2.7776312e-05, places=9)

    def test_distortion_reorder_to_radtan(self):
        c = load_camera(os.path.join(CAL, 'cam0.yaml'))
        d = distortion(c)
        # OpenCV radtan order: [k1, k2, p1, p2, k3]
        self.assertEqual(d.tolist(), [c['k1'], c['k2'], c['p1'], c['p2'], c['k3']])

    def test_intrinsics_matrix(self):
        c = load_camera(os.path.join(CAL, 'cam0.yaml'))
        k = intrinsics(c)
        self.assertEqual(k.shape, (3, 3))
        self.assertAlmostEqual(k[0, 0], c['fx'])
        self.assertAlmostEqual(k[1, 1], c['fy'])
        self.assertAlmostEqual(k[0, 2], c['cx'])
        self.assertAlmostEqual(k[1, 2], c['cy'])
        self.assertAlmostEqual(k[2, 2], 1.0)

    def test_camera_info_shapes(self):
        c = load_camera(os.path.join(CAL, 'cam0.yaml'))
        info = camera_info_dict(c)
        self.assertEqual(len(info['D']), 5)
        self.assertEqual(len(info['K']), 9)
        self.assertEqual(len(info['R']), 9)
        self.assertEqual(len(info['P']), 12)
        self.assertEqual(info['distortion_model'], 'plumb_bob')
        self.assertEqual(info['K'][0], c['fx'])


class TestStereo(unittest.TestCase):
    def test_stereo_rt(self):
        r, t, base_m, bxf = load_stereo(os.path.join(CAL, 'stereo_params.yaml'))
        self.assertAlmostEqual(base_m, 0.060047, places=5)
        # |Tx| ~ baseline
        self.assertAlmostEqual(abs(float(t[0])), 0.060048, places=5)
        # R is a valid rotation matrix (orthonormal)
        np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-9)
        self.assertAlmostEqual(abs(np.linalg.det(r)), 1.0, places=6)

    def test_rodrigues_identity_and_norm(self):
        np.testing.assert_allclose(rodrigues([0, 0, 0]), np.eye(3))
        r = rodrigues([0.1, -0.2, 0.05])
        np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-12)


if __name__ == '__main__':
    unittest.main()
