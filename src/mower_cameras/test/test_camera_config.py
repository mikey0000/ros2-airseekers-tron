"""Sanity tests for mower_cameras: default parameters and (when available) frame conversion."""

import os

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_default_params_are_consistent():
    yaml = pytest.importorskip('yaml')
    with open(os.path.join(PKG_DIR, 'config', 'cameras.yaml')) as handle:
        params = yaml.safe_load(handle)['mower_cameras']['ros__parameters']
    # Side-by-side stereo: each eye is half the total capture width.
    assert params['width'] % 2 == 0
    assert params['height'] > 0 and params['fps'] > 0
    assert 1 <= params['jpeg_quality'] <= 100
    assert params['rear_width'] > 0 and params['rear_height'] > 0


def test_to_image_encodings():
    pytest.importorskip('rclpy')
    np = pytest.importorskip('numpy')
    pytest.importorskip('cv2')
    sensor_msgs = pytest.importorskip('sensor_msgs.msg')  # noqa: F841
    from builtin_interfaces.msg import Time
    from mower_cameras.camera_node import _to_image

    stamp = Time()

    colour = _to_image(stamp, 'f', np.zeros((4, 6, 3), dtype=np.uint8))
    assert (colour.encoding, colour.width, colour.height, colour.step) == ('bgr8', 6, 4, 18)
    mono = _to_image(stamp, 'f', np.zeros((4, 6), dtype=np.uint8))
    assert (mono.encoding, mono.step) == ('mono8', 6)
