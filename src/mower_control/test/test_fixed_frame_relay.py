import numpy as np

from mower_control.fixed_frame_relay import relay_plan

NOW = 10**12
T_ODOM = np.eye(4)
T_ODOM[0, 3] = 1.0          # robot 1 m along odom x
T_MAP = np.eye(4)
T_MAP[1, 3] = 5.0           # odom 5 m along map y


def test_base_link_cloud_goes_to_odom_and_map_at_the_tf_stamp():
    pts = np.array([[0.4, 0.0, 0.0]], np.float32)
    out = relay_plan('base_link', pts, NOW, 300_000_000, 'base_link',
                     (NOW - 20_000_000, T_ODOM), (NOW - 60_000_000, T_MAP, False), {})
    got = {f: (ns, p) for f, ns, p in out}
    assert got['odom'][0] == NOW - 20_000_000
    np.testing.assert_allclose(got['odom'][1][0], [1.4, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(got['map'][1][0], [1.4, 5.0, 0.0], atol=1e-6)


def test_empty_heartbeat_still_relayed():
    out = relay_plan('base_link', np.zeros((0, 3), np.float32), NOW, 300_000_000, 'base_link',
                     (NOW - 20_000_000, T_ODOM), None, {})
    assert [(f, len(p)) for f, _, p in out] == [('odom', 0)]


def test_stale_or_missing_tf_skips():
    pts = np.zeros((1, 3), np.float32)
    assert relay_plan('base_link', pts, NOW, 300_000_000, 'base_link', None, None, {}) == []
    assert relay_plan('base_link', pts, NOW, 300_000_000, 'base_link',
                      (NOW - 900_000_000, T_ODOM), None, {}) == []


def test_static_child_frame_and_unknown_frame():
    pts = np.array([[0.0, 0.0, 0.0]], np.float32)
    T_bc = np.eye(4)
    T_bc[2, 3] = 0.25
    out = relay_plan('camera', pts, NOW, 300_000_000, 'base_link',
                     (NOW - 1, T_ODOM), None, {'camera': T_bc})
    np.testing.assert_allclose(out[0][2][0], [1.0, 0.0, 0.25], atol=1e-6)
    assert relay_plan('nowhere', pts, NOW, 300_000_000, 'base_link',
                      (NOW - 1, T_ODOM), None, {}) == []


def test_map_copy_dropped_when_map_tf_stale_but_static_kept():
    pts = np.zeros((1, 3), np.float32)
    stale = relay_plan('base_link', pts, NOW, 300_000_000, 'base_link',
                       (NOW - 1, T_ODOM), (NOW - 900_000_000, T_MAP, False), {})
    assert [f for f, _, _ in stale] == ['odom']
    static = relay_plan('base_link', pts, NOW, 300_000_000, 'base_link',
                        (NOW - 1, T_ODOM), (0, T_MAP, True), {})
    assert [f for f, _, _ in static] == ['odom', 'map']
