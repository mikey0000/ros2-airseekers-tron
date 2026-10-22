"""M3 import diet: heading_aligner resolves base_link <- stereo IMU from /tf_static
without tf2_ros (~30 MB)."""

import math
import subprocess
import sys
import textwrap

from mower_localization.static_tf import StaticRotations, qmul


def _qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _close(a, b):
    # q and -q are the same rotation
    return (all(abs(x - y) < 1e-9 for x, y in zip(a, b))
            or all(abs(x + y) < 1e-9 for x, y in zip(a, b)))


def test_chain_and_reverse():
    t = StaticRotations()
    t.add('base_link', 'stereo_link', _qz(90))
    t.add('/stereo_link', 'stereo_imu', _qx(180))
    want = qmul(_qz(90), _qx(180))
    assert _close(t.rotation('base_link', 'stereo_imu'), want)
    inv = (-want[0], -want[1], -want[2], want[3])
    assert _close(t.rotation('stereo_imu', 'base_link'), inv)
    assert _close(t.rotation('base_link', 'base_link'), (0.0, 0.0, 0.0, 1.0))


def test_sibling_frames_and_missing():
    t = StaticRotations()
    t.add('base_link', 'a', _qz(30))
    t.add('base_link', 'b', _qz(100))
    assert _close(t.rotation('a', 'b'), _qz(70))
    assert t.rotation('base_link', 'nope') is None
    t.add('odom', 'other', _qz(5))
    assert t.rotation('base_link', 'other') is None    # different trees


def test_matches_tf2():
    """Same answer as tf2_ros.Buffer for a 3-link static chain."""
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    links = [('base_link', 'mount', _qz(-37)), ('mount', 'cam', _qx(90)),
             ('cam', 'imu', qmul(_qz(12), _qx(-8)))]
    buf, t = tf2_ros.Buffer(), StaticRotations()
    for parent, child, q in links:
        m = TransformStamped()
        m.header.frame_id, m.child_frame_id = parent, child
        (m.transform.rotation.x, m.transform.rotation.y,
         m.transform.rotation.z, m.transform.rotation.w) = q
        buf.set_transform_static(m, 'test')
        t.add(parent, child, q)
    r = buf.lookup_transform('base_link', 'imu', __import__('rclpy').time.Time()).transform.rotation
    assert _close(t.rotation('base_link', 'imu'), (r.x, r.y, r.z, r.w))


def test_heading_aligner_does_not_import_tf2_ros():
    code = textwrap.dedent("""
        import sys
        import rclpy
        from mower_localization.heading_aligner import HeadingAligner
        rclpy.init()
        node = HeadingAligner()
        assert node._tf_static is not None   # default imu_source=auto -> lookup armed
        print('TF2', 'tf2_ros' in sys.modules)
        node.destroy_node()
        rclpy.shutdown()
    """)
    r = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=60)
    assert 'TF2 False' in r.stdout, r.stdout + r.stderr
