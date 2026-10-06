"""Zero-copy conversion, 4:2:2 decimation and the pre-serialized Image."""

import pytest

np = pytest.importorskip('numpy')


def _uyvy(h, w, bpl, seed=1):
    rnd = np.random.default_rng(seed)
    raw = rnd.integers(0, 256, size=(h, bpl), dtype=np.uint8)
    return raw


def test_strided_view_conversion_equals_copy():
    cv2 = pytest.importorskip('cv2')
    from mower_cameras import v4l2
    h, w, bpl = 6, 8, 20                    # padded stride
    raw = _uyvy(h, w, bpl)
    ref = cv2.cvtColor(np.ascontiguousarray(raw[:, :2 * w]).reshape(h, w, 2),
                       cv2.COLOR_YUV2BGR_UYVY)
    out = v4l2.to_bgr(memoryview(raw.tobytes()), w, h, bpl, 'UYVY')
    assert np.array_equal(out, ref)
    dst = np.zeros((h, w, 3), np.uint8)
    out2 = v4l2.to_bgr(raw.tobytes(), w, h, bpl, 'UYVY', dst=dst)
    assert np.array_equal(dst, ref) and out2.ctypes.data == dst.ctypes.data
    assert v4l2.to_bgr(raw.tobytes()[:10], w, h, bpl, 'UYVY') is None


@pytest.mark.parametrize('fmt', ['UYVY', 'YUYV'])
@pytest.mark.parametrize('k', [2, 3])
def test_decimation_picks_every_kth_pixel(fmt, k):
    from mower_cameras import v4l2
    h, w = 6, 12
    raw = _uyvy(h, w, 2 * w, seed=k).reshape(h, w, 2)
    dec = v4l2._decimate_422(raw, k, fmt)
    assert dec.shape == (h // k, w // k, 2) and dec.flags['C_CONTIGUOUS']
    flat = raw.reshape(h, 2 * w)
    yoff, uoff, voff = (1, 0, 2) if fmt == 'UYVY' else (0, 1, 3)
    for r in range(h // k):
        src = flat[r * k]
        for j in range(w // k):
            p = j * k                                   # source pixel
            # luma of output pixel j == luma of source pixel p
            assert dec[r, j, 1 if fmt == 'UYVY' else 0] == src[2 * p + yoff]
            if j % 2 == 0:                              # chroma from the even pixel's macro
                m = (p // 2) * 4
                if fmt == 'UYVY':
                    assert (dec[r, j, 0], dec[r, j + 1, 0]) == (src[m + uoff], src[m + voff])
                else:
                    assert (dec[r, j, 1], dec[r, j + 1, 1]) == (src[m + uoff], src[m + voff])
    assert v4l2.output_size(1920, 1080, fmt, 2) == (960, 540)
    assert v4l2.output_size(1920, 1080, 'MJPG', 2) == (1920, 1080)


def test_to_bgr_scale_matches_decimated_input():
    cv2 = pytest.importorskip('cv2')
    from mower_cameras import v4l2
    h, w = 8, 16
    raw = _uyvy(h, w, 2 * w)
    out = v4l2.to_bgr(raw.tobytes(), w, h, 2 * w, 'UYVY', scale=2)
    ref = cv2.cvtColor(v4l2._decimate_422(raw.reshape(h, w, 2), 2, 'UYVY'),
                       cv2.COLOR_YUV2BGR_UYVY)
    assert out.shape == (4, 8, 3) and np.array_equal(out, ref)


@pytest.mark.parametrize('frame_id,h,w', [('left_oa_camera', 3, 5), ('rear_camera', 4, 6),
                                          ('x', 2, 2), ('', 1, 3)])
def test_image_cdr_matches_rclpy(frame_id, h, w):
    from mower_cameras.image_cdr import ImageCdr
    img = ImageCdr(frame_id, h, w)
    pix = np.arange(h * w * 3, dtype=np.uint8).reshape(h, w, 3)
    img.image[...] = pix
    data = img.serialize(1_700_000_000_123_456_789)
    serialization = pytest.importorskip('rclpy.serialization')
    msgs = pytest.importorskip('sensor_msgs.msg')
    m = msgs.Image()
    m.header.stamp.sec, m.header.stamp.nanosec = 1_700_000_000, 123_456_789
    m.header.frame_id = frame_id
    m.height, m.width, m.encoding, m.step = h, w, 'bgr8', 3 * w
    m.data = pix.tobytes()
    ref = serialization.serialize_message(m)
    # Fast-CDR leaves alignment padding uninitialized: compare everything else
    assert len(data) == len(ref)
    pads = set()
    s_end = 16 + len(frame_id) + 1                      # end of frame_id string
    pads.update(range(s_end, s_end + (-s_end % 4)))
    e_end = s_end + (-s_end % 4) + 8 + 4 + len('bgr8') + 1 + 1   # after is_bigendian
    pads.update(range(e_end, e_end + (-e_end % 4)))
    assert bytes(b for i, b in enumerate(data) if i not in pads) == \
        bytes(b for i, b in enumerate(ref) if i not in pads)
    assert serialization.deserialize_message(data, msgs.Image) == m


def test_camera_info_scaling():
    pytest.importorskip('rclpy')
    pytest.importorskip('yaml')
    import os
    from mower_cameras.camera_node import load_camera_info, scale_camera_info
    stack = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))
    info = load_camera_info(os.path.join(stack, 'config', 'cameras',
                                         'left_oa_camera_info.yaml'), 'left_oa_camera')
    assert scale_camera_info(info, info.width, info.height) is info
    half = scale_camera_info(info, info.width // 2, info.height // 2)
    assert (half.width, half.height) == (info.width // 2, info.height // 2)
    assert half.k[0] == pytest.approx(info.k[0] / 2) and half.k[5] == pytest.approx(info.k[5] / 2)
    assert list(half.d) == list(info.d)
    assert scale_camera_info(info, 100, 100) is None


def test_camera_info_mode_file(tmp_path):
    """A 4:3 mode of the 16:9-calibrated rear webcam uses <stem>_<w>x<h>.yaml if present."""
    pytest.importorskip('rclpy')
    pytest.importorskip('yaml')
    from mower_cameras.camera_node import (camera_info_for_mode, load_camera_info,
                                           mode_info_path)
    base = tmp_path / 'rear_camera_info.yaml'
    tmpl = ('image_width: {w}\nimage_height: {h}\ncamera_name: rear_camera\n'
            'camera_matrix: {{rows: 3, cols: 3, data: [{f}, 0, {cx}, 0, {f}, {cy}, 0, 0, 1]}}\n'
            'distortion_model: plumb_bob\n'
            'distortion_coefficients: {{rows: 1, cols: 5, data: [0, 0, 0, 0, 0]}}\n'
            'rectification_matrix: {{rows: 3, cols: 3, data: [1, 0, 0, 0, 1, 0, 0, 0, 1]}}\n'
            'projection_matrix: {{rows: 3, cols: 4, data: [{f}, 0, {cx}, 0, 0, {f}, {cy}, 0, '
            '0, 0, 1, 0]}}\n')
    base.write_text(tmpl.format(w=1920, h=1080, f=1500.0, cx=960.0, cy=540.0))
    info = load_camera_info(str(base), 'rear_camera')
    assert mode_info_path(str(base), 640, 480) == str(tmp_path / 'rear_camera_info_640x480.yaml')
    assert mode_info_path('', 640, 480) is None
    # no mode file: 16:9 rescales, 4:3 has no calibration
    assert camera_info_for_mode(info, str(base), 1280, 720).k[0] == pytest.approx(1000.0)
    assert camera_info_for_mode(info, str(base), 640, 480) is None
    (tmp_path / 'rear_camera_info_640x480.yaml').write_text(
        tmpl.format(w=640, h=480, f=666.0, cx=320.0, cy=240.0))
    mode = camera_info_for_mode(info, str(base), 640, 480, 'rear_camera')
    assert (mode.width, mode.height, mode.k[0]) == (640, 480, pytest.approx(666.0))
    assert mode.header.frame_id == 'rear_camera'
