"""IIO scan parsing, gyro/accel pairing and rest-bias logic (ROS-free)."""
import struct

import numpy as np
import pytest

from mower_cameras import iio_imu

ICM_CHANS = [('in_anglvel_x', '0', 'be:s16/16>>0'), ('in_anglvel_y', '1', 'be:s16/16>>0'),
             ('in_anglvel_z', '2', 'be:s16/16>>0'), ('in_timestamp', '4', 'le:s64/64>>0')]


def test_parse_scan_type():
    t = iio_imu.parse_scan_type('be:s16/16>>0')
    assert t == dict(endian='>', signed=True, bits=16, storage=16, repeat=1, shift=0)
    t = iio_imu.parse_scan_type('le:u12/16X2>>4\n')
    assert t['endian'] == '<' and not t['signed'] and t['bits'] == 12 and t['repeat'] == 2
    with pytest.raises(ValueError):
        iio_imu.parse_scan_type('xx:s16/16')


def test_layout_matches_icm40608_16_byte_record():
    dt, _ = iio_imu.scan_layout(ICM_CHANS)
    assert dt.itemsize == 16
    assert dt.fields['in_timestamp'][1] == 8           # 6 bytes data + 2 pad


def test_decode_icm_records_like_the_kernel_writes_them():
    dt, types = iio_imu.scan_layout(ICM_CHANS)
    recs = [(-7, 1, 32767, 1_000_000_000), (-32768, 0, 5, 1_005_000_000)]
    raw = b''.join(struct.pack('>hhh', x, y, z) + b'\0\0' + struct.pack('<q', t)
                   for x, y, z, t in recs)
    out = iio_imu.decode_records(raw + b'\1\2\3', dt, types)    # trailing partial ignored
    assert out['in_anglvel_x'].tolist() == [-7, -32768]
    assert out['in_anglvel_z'].tolist() == [32767, 5]
    assert out['in_timestamp'].tolist() == [1_000_000_000, 1_005_000_000]


def test_decode_shift_and_sign_extension():
    dt, types = iio_imu.scan_layout([('v', '0', 'le:s12/16>>4')])
    raw = struct.pack('<H', (0xFFF << 4) | 0x5)          # -1 in 12 bits, low nibble junk
    assert iio_imu.decode_records(raw, dt, types)['v'].tolist() == [-1]


def test_pairer_joins_equal_stamps_and_drops_orphans():
    p = iio_imu.ImuPairer(tol_ns=1_000_000)
    p.add_gyro(0, 'g0')
    p.add_gyro(5_000_000, 'g1')
    p.add_gyro(10_000_000, 'g2')
    p.add_accel(5_000_000, 'a1')
    p.add_accel(10_000_200, 'a2')
    out = p.pop()
    assert out == [(5_000_000, 'g1', 'a1'), (10_000_000, 'g2', 'a2')]
    assert p.dropped == 1
    p.add_accel(15_000_000, 'a3')
    assert p.pop() == [] and len(p.a) == 1               # waits for its gyro partner


def test_rest_bias_rejects_motion_then_accepts_still():
    rng = np.random.default_rng(0)
    rb = iio_imu.RestBias(n=200, max_std=0.004)
    for _ in range(200):
        rb.add(rng.normal(0, 0.05, 3))                   # moving
    assert rb.bias is None and rb.windows_rejected == 1
    for _ in range(200):
        rb.add(np.array([-0.008, -0.0095, -0.002]) + rng.normal(0, 0.0008, 3))
    assert rb.bias is not None
    assert np.allclose(rb.bias, [-0.008, -0.0095, -0.002], atol=2e-4)


def test_find_devices_and_clock(tmp_path):
    for dev, name in [('iio:device0', 'fec10000.saradc'), ('iio:device1', 'icm40608-gyro'),
                      ('iio:device2', 'icm40608-accel')]:
        (tmp_path / dev).mkdir()
        (tmp_path / dev / 'name').write_text(name + '\n')
    assert iio_imu.find_devices(['icm40608'], str(tmp_path)) == \
        {'gyro': 'iio:device1', 'accel': 'iio:device2'}
    assert iio_imu.find_devices(['icm42600'], str(tmp_path)) == {}
    assert iio_imu.choose_clock(1_000, 1_100, 10**18) == 'monotonic'
    assert iio_imu.choose_clock(10**18, 1_100, 10**18 + 5) == 'realtime'


def test_batch_stamper_groups_split_reads_and_period():
    p = 4_980_000                                    # true ODR 200.8 Hz, nominal 5 ms
    st = iio_imu.BatchStamper(5_000_000)
    true_t = [1_000_000_000 + k * p for k in range(40)]
    # IRQ groups of 4,5,6,4,5,... samples stamped with their last sample's time
    groups, k = [], 0
    for size in [4, 5, 6, 4, 5, 6, 4, 6]:
        groups.append([true_t[k + size - 1]] * size)
        k += size
    flat = [t for g in groups for t in g]
    # reads split groups arbitrarily
    cuts = [0, 3, 11, 12, 20, 27, 33, len(flat)]
    out_t, out_v = [], []
    for a, b in zip(cuts, cuts[1:]):
        t, v = st.feed(flat[a:b], list(range(a, b)))
        out_t += t.tolist()
        out_v += v
    assert out_v == list(range(len(out_v)))           # order kept, nothing duplicated
    assert len(out_v) == len(flat) - 6                # last group still held back
    err = np.abs(np.array(out_t) - np.array(true_t[:len(out_t)]))
    assert err.max() < 0.1 * p                        # (no IRQ jitter in this case)
    assert (np.diff(out_t) > 0).all()


def test_batch_stamper_smooths_irq_latency_and_reanchors():
    rng = np.random.default_rng(3)
    p = 4_980_000
    st = iio_imu.BatchStamper(5_000_000)
    k, t_out = 0, []
    for g in range(400):                              # 8 s of 200 Hz in groups of 4..6
        size = int(rng.integers(4, 7))
        k += size
        stamp = 10**9 + (k - 1) * p + int(rng.uniform(0.2e6, 3e6))   # IRQ latency 0.2-3 ms
        t, _ = st.feed([stamp] * size, [0] * size)
        t_out += t.tolist()
    dt = np.diff(t_out[200:])
    assert abs(np.mean(dt) - p) < 0.002 * p           # ODR tracked
    assert np.std(dt) < 0.05 * p                      # evenly spaced despite ms IRQ jitter
    lag = np.array(t_out[200:]) - (10**9 + np.arange(200, len(t_out)) * p)
    assert 0 < lag.mean() < 3e6 and lag.std() < 0.6e6    # = mean IRQ latency, steady
    t, _ = st.feed([t_out[-1] + 10**9] * 4, [0] * 4)   # 1 s gap: re-anchor, no catch-up
    t, _ = st.feed([t_out[-1] + 10**9 + 4 * p] * 4, [0] * 4)
    assert abs(t[-1] - (t_out[-1] + 10**9)) < 2 * p


def test_batch_stamper_empty_and_single():
    st = iio_imu.BatchStamper(5_000_000)
    t, v = st.feed([], [])
    assert len(t) == 0 and v == []
    t, v = st.feed([10], ['a'])
    assert len(t) == 0                                # held back
    t, v = st.feed([20_000_000], ['b'])
    assert t.tolist() == [10] and v == ['a']


def test_imu_cdr_matches_rclpy_serialisation():
    pytest.importorskip('rclpy')
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import Imu
    for frame in ('stereo_camera_imu', 'abc', 'abcd', 'a'):
        gc, ac = [1e-6 * (i + 1) for i in range(9)], [4e-4] * 9
        cdr = iio_imu.ImuCdr(frame, gc, ac)
        stamp = 1_791_309_125_293_565_497
        m = Imu()
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10**9)
        m.header.frame_id = frame
        m.orientation.w = 0.0
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = 0.1, -0.2, 0.3
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = 0.0, -9.8, 0.3
        m.angular_velocity_covariance = gc
        m.linear_acceleration_covariance = ac
        ours = cdr.serialize(stamp, (0.1, -0.2, 0.3), (0.0, -9.8, 0.3))
        ref = serialize_message(m)
        assert len(ours) == len(ref)            # padding bytes differ (rclpy leaves garbage)
        assert deserialize_message(ours, Imu) == m


def test_rest_bias_discard_restarts_the_window():
    from mower_cameras.iio_imu import RestBias
    b = RestBias(n=10, max_std=0.01)
    for _ in range(9):
        b.add((0.5, 0.5, 0.5))                  # a moving start, would pass the std gate
    b.discard()
    assert b.windows_rejected == 1
    for _ in range(10):
        b.add((0.001, 0.002, 0.003))
    assert b.bias is not None and abs(b.bias[2] - 0.003) < 1e-12
