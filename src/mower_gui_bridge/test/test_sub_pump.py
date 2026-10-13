"""sub_pump: CDR fast parsers vs rclpy deserialization, and pump delivery semantics.

Needs a real ROS 2 Jazzy environment (rclpy + message packages); skipped elsewhere.
"""

import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nav_msgs.msg')
pytest.importorskip('mower_interfaces.msg')

from rclpy.serialization import serialize_message  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from mower_interfaces.msg import MowerBaseDevStatus  # noqa: E402

from mower_gui_bridge import sub_pump  # noqa: E402


def _odom(frame, child, seed):
    m = Odometry()
    m.header.stamp.sec, m.header.stamp.nanosec = 1700000000 + seed, 123456789
    m.header.frame_id, m.child_frame_id = frame, child
    p = m.pose.pose
    p.position.x, p.position.y, p.position.z = 1.5 + seed, -2.25, 0.125
    p.orientation.z, p.orientation.w = math.sin(0.3), math.cos(0.3)
    m.pose.covariance = [float(i) + seed for i in range(36)]
    m.twist.twist.linear.x, m.twist.twist.angular.z = 0.4, -0.2
    m.twist.covariance = [float(-i) for i in range(36)]
    return m


@pytest.mark.parametrize('frame,child', [
    ('map', 'base_footprint'), ('odom', 'base_link'), ('', ''), ('m', 'ab'), ('abc', 'abcd'),
    ('a_long_frame_name_12345', 'x'),
])
def test_parse_odometry_matches_deserialize(frame, child):
    for seed in range(3):
        src = _odom(frame, child, seed)
        data = serialize_message(src)
        got = sub_pump.parse_odometry(data)
        assert got is not None
        assert got.header.frame_id == frame and got.child_frame_id == child
        assert (got.header.stamp.sec, got.header.stamp.nanosec) == \
            (src.header.stamp.sec, src.header.stamp.nanosec)
        for a in ('x', 'y', 'z'):
            assert getattr(got.pose.pose.position, a) == getattr(src.pose.pose.position, a)
            assert getattr(got.twist.twist.linear, a) == getattr(src.twist.twist.linear, a)
            assert getattr(got.twist.twist.angular, a) == getattr(src.twist.twist.angular, a)
        for a in ('x', 'y', 'z', 'w'):
            assert getattr(got.pose.pose.orientation, a) == \
                getattr(src.pose.pose.orientation, a)
        assert list(got.pose.covariance) == list(src.pose.covariance)
        assert list(got.twist.covariance) == list(src.twist.covariance)
        assert sub_pump.odometry_frames(data) == (frame, child)


@pytest.mark.parametrize('src_frames', [('odom', 'base_link'), ('', ''), ('m', 'ab'),
                                        ('abc', 'abcdefg'), ('map', 'base_footprint')])
@pytest.mark.parametrize('dst_frames', [('map', 'base_footprint'), ('m', ''), ('abcd', 'xyz12')])
def test_odometry_with_frames_equals_deserialize_set_serialize(src_frames, dst_frames):
    # Not byte-compared: Fast-CDR leaves uninitialised bytes in alignment padding.
    from rclpy.serialization import deserialize_message
    src = _odom(src_frames[0], src_frames[1], 1)
    expect = deserialize_message(serialize_message(src), Odometry)
    expect.header.frame_id, expect.child_frame_id = dst_frames
    expect_bytes = serialize_message(expect)
    got = sub_pump.odometry_with_frames(serialize_message(src), *dst_frames)
    assert len(got) == len(expect_bytes)
    assert deserialize_message(got, Odometry) == expect


def test_flat_parser_mower_base_status_every_field():
    parse = sub_pump.flat_parser(MowerBaseDevStatus)
    assert parse is not None
    fields = [n for n in MowerBaseDevStatus.get_fields_and_field_types() if n != 'header']
    for idx in range(len(fields)):
        for frame in ('', 'base_link'):
            src = MowerBaseDevStatus()
            src.header.frame_id = frame
            src.header.stamp.sec = 42
            for j, name in enumerate(fields):
                cur = getattr(src, name)
                value = (j == idx) if isinstance(cur, bool) else (j * 7 + idx) % 256
                setattr(src, name, value)
            got = parse(serialize_message(src))
            assert got.header.frame_id == frame and got.header.stamp.sec == 42
            for name in fields:
                assert getattr(got, name) == getattr(src, name), name


def test_parse_imu_matches_deserialize():
    from sensor_msgs.msg import Imu
    for frame in ('', 'imu_link', 'abc'):
        src = Imu()
        src.header.frame_id = frame
        src.header.stamp.sec, src.header.stamp.nanosec = 5, 6
        src.orientation.x, src.orientation.y = 0.1, -0.2
        src.orientation.z, src.orientation.w = 0.3, 0.9
        src.angular_velocity.x, src.angular_velocity.z = 1.5, -2.5
        src.linear_acceleration.y, src.linear_acceleration.z = 0.25, 9.81
        src.orientation_covariance = [float(i) for i in range(9)]
        src.angular_velocity_covariance = [float(i) * 2 for i in range(9)]
        src.linear_acceleration_covariance = [float(i) * 3 for i in range(9)]
        got = sub_pump.parse_imu(serialize_message(src))
        assert got.header.frame_id == frame and got.header.stamp.nanosec == 6
        for part in ('orientation', 'angular_velocity', 'linear_acceleration'):
            for a in ('x', 'y', 'z', 'w') if part == 'orientation' else ('x', 'y', 'z'):
                assert getattr(getattr(got, part), a) == getattr(getattr(src, part), a)
            assert list(getattr(got, part + '_covariance')) == \
                list(getattr(src, part + '_covariance'))


def test_flat_serializer_matches_rclpy():
    from rclpy.serialization import deserialize_message
    ser = sub_pump.flat_serializer(MowerBaseDevStatus)
    fields = [n for n in MowerBaseDevStatus.get_fields_and_field_types() if n != 'header']
    for idx in range(len(fields)):
        expect = MowerBaseDevStatus()
        expect.header.stamp.sec, expect.header.stamp.nanosec = 1700000001, 999
        values = {}
        for j, name in enumerate(fields):
            cur = getattr(expect, name)
            value = (j == idx) if isinstance(cur, bool) else (j * 7 + idx) % 256
            setattr(expect, name, value)
            if j != idx + 1:                 # leave one field out: default must be used
                values[name] = value
            else:
                setattr(expect, name, cur)
        got = ser(values, 1700000001 * 1_000_000_000 + 999, '')
        assert deserialize_message(got, MowerBaseDevStatus) == expect
    for v in (True, False):
        got = sub_pump.flat_serializer(Bool)({'data': v})
        assert len(got) == len(serialize_message(Bool(data=v)))
        assert deserialize_message(got, Bool).data is v
    assert sub_pump.flat_serializer(Odometry) is None


def test_odometry_bytes_matches_rclpy():
    from rclpy.serialization import deserialize_message
    for frame, child in (('odom', 'base_link'), ('', 'x'), ('abcde', '')):
        src = _odom(frame, child, 2)
        p, t = src.pose.pose, src.twist.twist
        got = sub_pump.odometry_bytes(
            src.header.stamp.sec * 1_000_000_000 + src.header.stamp.nanosec, frame, child,
            (p.position.x, p.position.y, p.position.z),
            (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            (t.linear.x, t.linear.y, t.linear.z), (t.angular.x, t.angular.y, t.angular.z),
            tuple(src.pose.covariance), tuple(src.twist.covariance))
        assert len(got) == len(serialize_message(src))
        assert deserialize_message(got, Odometry) == src


def test_flat_parser_bool_and_garbage():
    parse = sub_pump.flat_parser(Bool)
    assert parse(serialize_message(Bool(data=True))).data is True
    assert parse(serialize_message(Bool(data=False))).data is False
    assert parse(b'') is None
    assert parse(b'\x00\x00\x00\x00\x01') is None          # big endian: not handled
    assert sub_pump.parse_odometry(b'\x00\x01\x00\x00\xff\xff') is None
    assert sub_pump.flat_parser(Odometry) is None           # nested fields: no flat parser


@pytest.fixture
def node():
    if not rclpy.ok():
        rclpy.init()
    n = rclpy.create_node('sub_pump_test')
    yield n
    n.destroy_node()


def _wait(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_pump_event_sampled_and_pause(node):
    from rclpy.executors import SingleThreadedExecutor

    pump = sub_pump.SubscriptionPump(node, 'test_pump')
    events, sampled, receipts, raw = [], [], [], []
    ev_sub = pump.subscribe(Bool, '/sub_pump_test/event', events.append, 10,
                            parser=sub_pump.flat_parser(Bool))
    s_sub = pump.subscribe(Bool, '/sub_pump_test/sampled',
                           lambda m, t: (sampled.append(m.data), receipts.append(t)), 1,
                           sampled=True, with_receipt=True)
    pump.subscribe(Odometry, '/sub_pump_test/raw', raw.append, 10, raw=True)
    pub_ev = node.create_publisher(Bool, '/sub_pump_test/event', 10)
    pub_s = node.create_publisher(Bool, '/sub_pump_test/sampled', 10)
    pub_raw = node.create_publisher(Odometry, '/sub_pump_test/raw', 10)
    # The node's own executor must never run pump callbacks.
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    pump.start()
    try:
        assert _wait(lambda: pub_ev.get_subscription_count() and pub_s.get_subscription_count())
        for v in (True, False, True):
            pub_ev.publish(Bool(data=v))
        assert _wait(lambda: len(events) == 3)
        assert [m.data for m in events] == [True, False, True]

        pub_raw.publish(_odom('map', 'base_footprint', 0))
        assert _wait(lambda: len(raw) == 1)
        assert isinstance(raw[0], bytes)
        assert sub_pump.odometry_frames(raw[0]) == ('map', 'base_footprint')

        for v in (True, True, False):
            pub_s.publish(Bool(data=v))
        time.sleep(0.3)
        ex.spin_once(timeout_sec=0.1)
        assert sampled == []                     # nothing until polled
        t_before = time.monotonic()
        pump.poll()
        assert sampled == [False]                # newest only (depth 1)
        assert receipts[0] <= t_before           # receipt time, not poll time
        pump.poll()
        assert sampled == [False]                # nothing new

        pump.set_active([ev_sub], False)
        time.sleep(0.05)
        pub_ev.publish(Bool(data=False))
        time.sleep(0.3)
        assert len(events) == 3                  # paused
        pump.flush(ev_sub)
        pump.set_active([ev_sub], True)
        pub_ev.publish(Bool(data=True))
        assert _wait(lambda: len(events) == 4)
        time.sleep(0.1)
        assert [m.data for m in events][3:] == [True]   # the paused message was dropped
    finally:
        pump.stop()
        ex.remove_node(node)
