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


def test_producers_off_by_default():
    yaml = pytest.importorskip('yaml')
    with open(os.path.join(PKG_DIR, 'config', 'cameras.yaml')) as handle:
        params = yaml.safe_load(handle)['mower_cameras']['ros__parameters']
    # v4l2_camera (cameras.launch.py) and stereo_vio_bridge are the canonical producers.
    assert params['enable_stereo'] is False
    assert params['enable_rear'] is False
    assert params['rear_device'] == '/dev/rear_camera'
    assert params['rear_fourcc'] == 'MJPG'


def test_split_and_camera_info():
    pytest.importorskip('rclpy')
    np = pytest.importorskip('numpy')
    pytest.importorskip('yaml')
    from mower_cameras.camera_node import load_camera_info, split_side_by_side

    left, right = split_side_by_side(np.zeros((4, 10, 3), dtype=np.uint8))
    assert left.shape == right.shape == (4, 5, 3)
    stack = os.path.dirname(os.path.dirname(PKG_DIR))
    info = load_camera_info(os.path.join(stack, 'config', 'cameras', 'rear_camera_info.yaml'),
                            'rear_camera')
    assert (info.width, info.height, info.header.frame_id) == (1920, 1080, 'rear_camera')
    assert len(info.k) == 9 and len(info.p) == 12 and len(info.d) == 5
    assert load_camera_info('') is None


def test_v4l2_abi_and_conversion():
    import ctypes
    import struct
    from mower_cameras import v4l2
    if struct.calcsize('P') == 8:  # 64-bit kernel ABI (the mower is aarch64)
        assert ctypes.sizeof(v4l2.v4l2_format) == 208
        assert ctypes.sizeof(v4l2.v4l2_buffer) == 88
        assert ctypes.sizeof(v4l2.v4l2_plane) == 64
        assert v4l2.VIDIOC_DQBUF == 0xc0585611
        assert v4l2.VIDIOC_S_FMT == 0xc0d05605
    assert v4l2.fourcc_str(v4l2.fourcc('UYVY')) == 'UYVY'
    np = pytest.importorskip('numpy')
    pytest.importorskip('cv2')
    # 4x2 UYVY with padded stride (bytesperline 12 > 8) -> 4x2x3 BGR
    raw = np.full((2, 12), 128, np.uint8).tobytes()
    img = v4l2.to_bgr(raw, 4, 2, 12, 'UYVY')
    assert img.shape == (2, 4, 3)
