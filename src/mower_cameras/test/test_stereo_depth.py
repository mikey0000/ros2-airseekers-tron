import numpy as np
import pytest

sd = pytest.importorskip('mower_cameras.stereo_depth_node')


def _frame(raw12, grey):
    h, w = raw12.shape
    px = np.zeros((h, w, 3), np.uint8)
    px[:, :, 0] = raw12 & 0xFF
    px[:, :, 1] = ((raw12 >> 8) & 0x0F) | 0xA0     # high nibble = noise, must be ignored
    px[:, :, 2] = grey
    return px.tobytes()


def test_decode_and_depth_scale():
    raw = np.zeros((360, 640), np.uint16)
    raw[300, 320] = 1317                       # 41.16 px -> 0.551 m
    raw[10, 10] = 4095
    grey = np.full((360, 640), 77, np.uint8)
    r, g = sd.decode_simor(_frame(raw, grey))
    assert r[300, 320] == 1317 and r[10, 10] == 4095 and g[0, 0] == 77
    z = sd.disparity_to_depth(r, 22698.4, 32.0)
    assert z[300, 320] == pytest.approx(22.6984 / (1317 / 32.0), rel=1e-4)
    assert z[0, 0] == 0.0


def test_points_optical_axes_and_range():
    z = np.zeros((360, 640), np.float32)
    z[180, 480] = 2.0                          # right of centre -> +x
    z[340, 320] = 1.0                          # below centre -> +y
    z[0, 0] = 9.0                              # beyond max range -> dropped
    pts = sd.depth_to_points(z, 378.0, 378.0, 319.5, 179.5, step=1, min_z=0.2, max_z=4.0)
    assert len(pts) == 2
    by_z = {round(float(p[2]), 3): p for p in pts}
    assert by_z[2.0][0] > 0 and by_z[1.0][1] > 0


def _stamper(last_tf_ns, parent='odom', extra=None):
    from types import SimpleNamespace
    return SimpleNamespace(_stamp_parent=parent, _last_tf_ns=last_tf_ns,
                           _extra_tf_ns=dict(extra or {}),
                           _stamp_margin_ns=50_000_000, _stamp_max_lag_ns=300_000_000)


def test_cloud_stamp_also_respects_map_odom_once_seen():
    f = sd.StereoDepthNode.cloud_stamp_ns
    now = 1_000_000_000_000
    # map->odom not seen on /tf yet (static / single EKF): ignored
    assert f(_stamper(now - 20_000_000, extra={('map', 'odom'): 0}), now) == now - 70_000_000
    # map->odom lags odom->base_link: the cloud is stamped under the OLDER one
    assert f(_stamper(now - 20_000_000, extra={('map', 'odom'): now - 120_000_000}), now) \
        == now - 170_000_000
    # map->odom seen but now stale: no cloud
    assert f(_stamper(now - 20_000_000, extra={('map', 'odom'): now - 900_000_000}), now) is None


def test_cloud_stamp_never_ahead_of_odom_tf():
    f = sd.StereoDepthNode.cloud_stamp_ns
    now = 1_000_000_000_000
    # TF 20 ms behind now -> stamp 70 ms behind now (transformable on arrival in the costmap)
    assert f(_stamper(now - 20_000_000), now) == now - 70_000_000
    # no TF yet / TF too old -> NO cloud (2026-10-09: the fallback stamp froze the costmaps)
    assert f(_stamper(0), now) is None
    assert f(_stamper(now - 900_000_000), now) is None
    # TF stamped in the future never pushes the cloud ahead of now
    assert f(_stamper(now + 500_000_000), now) == now
    # disabled -> old behaviour
    assert f(_stamper(now - 20_000_000, parent=''), now) == now


# --- ~/clear_points wiring (2026-10-09): stamp, gating, fit-failure, without opening video ---

class _Pub:
    def __init__(self, subs=1):
        self.subs, self.msgs = subs, []

    def get_subscription_count(self):
        return self.subs

    def publish(self, m):
        self.msgs.append(m)


def _ground_raw(cam_h=0.24):
    from mower_cameras import depth_filters as dfl
    n, h = dfl.plane_from_pose(cam_h, np.radians(0.92))
    v, u = np.mgrid[0:360, 0:640].astype(np.float64)
    nr = np.stack([(u - 319.5) / 378.0, (v - 179.5) / 378.0, np.ones_like(u)], -1) @ n
    z = np.where(nr > 1e-3, h / np.maximum(nr, 1e-3), 0)
    z[(z < 0.2) | (z > 6.0)] = 0
    raw = np.zeros(z.shape, np.uint16)
    raw[z > 0] = np.clip(np.round(32.0 * 22.6984 / z[z > 0]), 1, 4095)
    return raw


def _fake_node(on_demand=True, pts_subs=1, clear_subs=1, clear_on=True):
    from types import MethodType, SimpleNamespace
    from mower_cameras import depth_filters as dfl
    import rclpy.time
    n = SimpleNamespace(
        cfg={'frame_id': sd.FRAME_ID, 'bxf_mm': 22698.4, 'disparity_scale': 32.0,
             'fx': 378.0, 'fy': 378.0, 'cx': 319.5, 'cy': 179.5, 'cloud_step': 4,
             'min_range_m': 0.2, 'max_range_m': 4.0, 'speckle_max_size': 200,
             'speckle_max_diff_px': 1.0, 'min_valid_neighbours': 5, 'ground_plane_fit': True,
             'ground_margin_m': 0.12, 'ground_max_tilt_deg': 8.0, 'ground_max_dh_m': 0.08,
             'clear_max_range_m': 3.0, 'clear_voxel_m': 0.10},
        on_demand=on_demand, cloud_period=0.0, _last_cloud=0.0,
        prior=dfl.plane_from_pose(0.24, np.radians(0.92)), prior_from_tf=True, _tf_buf=None,
        persist=dfl.Persistence(1), rng=np.random.default_rng(0),
        _stats={'frames': 0, 'raw': 0, 'denoised': 0, 'obstacle': 0, 'published': 0,
                'clear': 0, 'clear_msgs': 0, 'fit_ok': 0, 'ms': 0.0},
        cloud_pub=_Pub(pts_subs), clear_pub=_Pub(clear_subs) if clear_on else None,
        depth_pub=_Pub(0), mono_pub=_Pub(0), plane_pub=_Pub(), stats_pub=_Pub(),
        _stamp_parent='odom', _last_tf_ns=1_000_000_000_000 - 20_000_000, _extra_tf_ns={},
        _stamp_margin_ns=50_000_000, _stamp_max_lag_ns=300_000_000)
    n.plane = n.prior
    n.get_clock = lambda: SimpleNamespace(
        now=lambda: rclpy.time.Time(nanoseconds=1_000_000_000_000))
    for name in ('cloud_stamp_ns', 'filtered_points', '_update_prior_from_tf', '_on_frame',
                 '_publish_stats'):
        setattr(n, name, MethodType(getattr(sd.StereoDepthNode, name), n))
    return n


def _cap():
    from types import SimpleNamespace
    return SimpleNamespace(width=640, height=360, bytesperline=1920)


def test_clear_cloud_same_stamp_and_frame_as_points():
    n = _fake_node()
    n._on_frame(_frame(_ground_raw(), np.zeros((360, 640), np.uint8)), _cap())
    (pts,), (clr,) = n.cloud_pub.msgs, n.clear_pub.msgs
    assert clr.header.stamp == pts.header.stamp
    assert clr.header.frame_id == pts.header.frame_id == sd.FRAME_ID
    assert [f.name for f in clr.fields] == ['x', 'y', 'z'] and clr.point_step == 12
    assert 50 < clr.width < 2000 and pts.width == 0
    # stamp: TF-lag workaround applied (newest odom TF 20 ms old - 50 ms margin)
    assert clr.header.stamp.sec * 10**9 + clr.header.stamp.nanosec == 10**12 - 70_000_000
    n._publish_stats()
    import json
    st = json.loads(n.stats_pub.msgs[0].data)
    assert st['clear_msgs'] == 1 and st['clear_per_frame'] == clr.width


def test_clear_cloud_not_published_on_failed_fit():
    n = _fake_node()
    wall = np.full((360, 640), int(round(32.0 * 22.6984 / 1.0)), np.uint16)
    n._on_frame(_frame(wall, np.zeros((360, 640), np.uint8)), _cap())
    assert len(n.cloud_pub.msgs) == 1 and n.clear_pub.msgs == []


@pytest.mark.parametrize('pts_subs,clear_subs', [(0, 1), (1, 0), (0, 0)])
def test_clear_cloud_on_demand_gating(pts_subs, clear_subs):
    n = _fake_node(pts_subs=pts_subs, clear_subs=clear_subs)
    n._on_frame(_frame(_ground_raw(), np.zeros((360, 640), np.uint8)), _cap())
    assert len(n.cloud_pub.msgs) == pts_subs and len(n.clear_pub.msgs) == clear_subs
    assert n._stats['frames'] == (1 if pts_subs or clear_subs else 0)


def test_clear_cloud_disabled():
    n = _fake_node(clear_on=False)
    n._on_frame(_frame(_ground_raw(), np.zeros((360, 640), np.uint8)), _cap())
    assert len(n.cloud_pub.msgs) == 1 and n._stats['clear'] == 0


def test_no_cloud_without_fresh_tf_but_images_still_flow():
    """Live 2026-10-09: clouds published before the EKF's first odom TF froze the costmap."""
    n = _fake_node()
    n._last_tf_ns = 0
    n.depth_pub = _Pub(1)
    n._on_frame(_frame(_ground_raw(), np.zeros((360, 640), np.uint8)), _cap())
    assert n.cloud_pub.msgs == [] and n.clear_pub.msgs == []
    assert len(n.depth_pub.msgs) == 1
    assert n._stats['no_tf_skipped'] == 1


# --- fixed-frame clouds (2026-10-09): costmaps never wait for a transform ---
def _tr(x=0.0, y=0.0, z=0.0, yaw=0.0):
    from types import SimpleNamespace
    import math
    return SimpleNamespace(translation=SimpleNamespace(x=x, y=y, z=z),
                           rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2),
                                                    w=math.cos(yaw / 2)))


def _fixed_node():
    from types import MethodType
    n = _fake_node()
    n.fixed_frames = True
    n.map_frame = 'map'
    n._stamp_child = 'base_link'
    n.cloud_map_pub, n.clear_map_pub = _Pub(1), _Pub(1)
    n._T_child_cam = sd.tf_matrix(_tr(z=0.25))
    now = 10**12
    n._T_parent_child = (now - 20_000_000, sd.tf_matrix(_tr(x=1.0)))
    n._T_map_parent = (now - 60_000_000, sd.tf_matrix(_tr(x=10.0)), False)
    for name in ('fixed_frame_clouds', '_publish_fixed'):
        setattr(n, name, MethodType(getattr(sd.StereoDepthNode, name), n))
    return n


def test_fixed_frame_clouds_odom_and_map():
    n = _fixed_node()
    pts = np.array([[0.0, 0.0, 1.0]], np.float32)
    out = n.fixed_frame_clouds(pts, 10**12)
    frames = {f: (ns, p) for f, ns, p in out}
    assert set(frames) == {'odom', 'map'}
    ns, p_odom = frames['odom']
    assert ns == 10**12 - 20_000_000                     # stamped AT the odom TF sample
    np.testing.assert_allclose(p_odom[0], [1.0, 0.0, 1.25], atol=1e-5)
    np.testing.assert_allclose(frames['map'][1][0], [11.0, 0.0, 1.25], atol=1e-5)


def test_fixed_frame_clouds_skip_without_fresh_odom_or_camera_tf():
    n = _fixed_node()
    pts = np.array([[0.0, 0.0, 1.0]], np.float32)
    n._T_parent_child = (10**12 - 900_000_000, n._T_parent_child[1])
    assert n.fixed_frame_clouds(pts, 10**12) == []
    n = _fixed_node()
    n._T_child_cam = None
    assert n.fixed_frame_clouds(pts, 10**12) == []


def test_fixed_frame_map_copy_dropped_when_map_odom_stale_but_static_ok():
    n = _fixed_node()
    pts = np.array([[0.0, 0.0, 1.0]], np.float32)
    n._T_map_parent = (10**12 - 900_000_000, n._T_map_parent[1], False)
    assert [f for f, _, _ in n.fixed_frame_clouds(pts, 10**12)] == ['odom']
    n._T_map_parent = (0, n._T_map_parent[1], True)         # single EKF: static map->odom
    assert [f for f, _, _ in n.fixed_frame_clouds(pts, 10**12)] == ['odom', 'map']


def test_on_frame_publishes_fixed_frame_clouds():
    n = _fixed_node()
    n._on_frame(_frame(_ground_raw(), np.zeros((360, 640), np.uint8)), _cap())
    assert n.cloud_pub.msgs[0].header.frame_id == 'odom'
    assert n.cloud_map_pub.msgs[0].header.frame_id == 'map'
    assert n.clear_pub.msgs[0].header.frame_id == 'odom'
    assert n.clear_map_pub.msgs[0].header.frame_id == 'map'
    st = n.cloud_pub.msgs[0].header.stamp
    assert st.sec * 10**9 + st.nanosec == 10**12 - 20_000_000
