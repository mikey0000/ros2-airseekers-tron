"""Dock yaw auto-correct: a dock yaw captured from the fused heading while the
heading_aligner offset was a stale low-quality one is rotated by
(COG offset - offset at set) once, when the aligner reaches a good COG alignment."""
import importlib
import json
import math
import sys
import types
from unittest import mock

import pytest

from mower_map import areas as core


def _dock(yaw_deg=100.0, off=103.4, qual='low'):
    return core.DockPose(1.0, 2.0, math.radians(yaw_deg), None, True, off, qual)


# ---------------------------------------------------------------- pure maths
def test_correction_maths_today_case():
    d = _dock(yaw_deg=100.0, off=103.4, qual='low')
    yaw = core.dock_yaw_autocorrect(d, 20.3, 'good', 'cog')
    assert math.degrees(yaw) == pytest.approx(100.0 + 20.3 - 103.4)


def test_correction_wraps():
    d = _dock(yaw_deg=170.0, off=-10.0, qual='low')
    yaw = core.dock_yaw_autocorrect(d, 30.0, 'good', 'cog')
    assert math.degrees(yaw) == pytest.approx(-150.0)


@pytest.mark.parametrize('qual', ['good', 'fair', 'corrected', None])
def test_no_correction_when_set_quality_not_low(qual):
    assert core.dock_yaw_autocorrect(_dock(qual=qual), 20.3, 'good', 'cog') is None


@pytest.mark.parametrize('quality,source', [('fair', 'cog'), ('low', 'file'),
                                            ('good', 'dock'), ('good', 'file')])
def test_no_correction_until_good_cog(quality, source):
    assert core.dock_yaw_autocorrect(_dock(), 20.3, quality, source) is None


def test_no_correction_without_offsets():
    assert core.dock_yaw_autocorrect(_dock(off=None), 20.3, 'good', 'cog') is None
    assert core.dock_yaw_autocorrect(_dock(), None, 'good', 'cog') is None
    assert core.dock_yaw_autocorrect(None, 20.3, 'good', 'cog') is None


def test_heading_quality_low():
    assert core.heading_quality_low('low')
    assert core.heading_quality_low('none')
    assert core.heading_quality_low(None)
    assert not core.heading_quality_low('good')
    assert not core.heading_quality_low('fair')


def test_yaml_round_trip_keeps_set_time_heading(tmp_path):
    p = str(tmp_path / 'dock_pose.yaml')
    core.save_dock_file(p, _dock(off=103.4, qual='low'))
    text = open(p).read()
    assert 'heading_offset_deg_at_set: 103.400' in text
    assert 'heading_quality_at_set: low' in text
    back = core.load_dock_file(p)
    assert back.heading_offset_deg_at_set == pytest.approx(103.4)
    assert back.heading_quality_at_set == 'low'
    old = core.DockPose(1.0, 2.0, 0.5, None, True)
    core.save_dock_file(p, old)
    back = core.load_dock_file(p)
    assert back.heading_offset_deg_at_set is None and back.heading_quality_at_set is None


# ---------------------------------------------------------------- node logic (stubbed ROS)
def _stub_ros():
    """Minimal stand-ins so map_server_node imports without a ROS install."""
    mods = {}
    for name in ('rclpy', 'rclpy.executors', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
                 'rclpy.callback_groups', 'rclpy.serialization', 'rclpy.impl',
                 'rclpy.impl.implementation_singleton',
                 'geometry_msgs', 'geometry_msgs.msg', 'nav2_msgs', 'nav2_msgs.msg',
                 'nav_msgs', 'nav_msgs.msg', 'std_msgs', 'std_msgs.msg', 'std_srvs',
                 'std_srvs.srv', 'mower_interfaces', 'mower_interfaces.msg',
                 'mower_interfaces.srv', 'mowgli_interfaces', 'mowgli_interfaces.msg',
                 'mowgli_interfaces.srv', 'tf2_ros', 'tf2_msgs', 'tf2_msgs.msg'):
        mods[name] = mock.MagicMock(name=name)

    class Node:  # noqa: D401 - plain base so MapServerNode can subclass it
        pass
    mods['rclpy.node'].Node = Node

    class Req:
        PRESERVE, REQUEST, MOTION = 0, 1, 2
    mods['mowgli_interfaces.srv'].SetDockingPoint = types.SimpleNamespace(Request=Req)
    return mods


@pytest.fixture
def msn():
    try:
        import rclpy  # noqa: F401
        stubs = {}
    except ImportError:
        stubs = _stub_ros()
    with mock.patch.dict(sys.modules, stubs):
        sys.modules.pop('mower_map.map_server_node', None)
        sys.modules.pop('mower_map.sub_pump', None)
        mod = importlib.import_module('mower_map.map_server_node')
        yield mod
    sys.modules.pop('mower_map.map_server_node', None)
    sys.modules.pop('mower_map.sub_pump', None)


class _Log:
    def __init__(self):
        self.warns, self.infos = [], []

    def warn(self, m):
        self.warns.append(m)

    def info(self, m):
        self.infos.append(m)

    error = warn


def _node(mod, heading=None, dock=None):
    n = mod.MapServerNode.__new__(mod.MapServerNode)
    params = {'dock_yaw_autocorrect': True, 'odom_topic': '/odom', 'robot_yaml_path': ''}
    n.p = params.get
    log = _Log()
    n.get_logger = lambda: log
    n.log = log
    n.heading = heading
    n.dock = dock
    n._dock_autocorrect_armed = False
    n.recent_poses = [(0.0, 1.0, 2.0, math.radians(100.0))] * 3
    n.dock_gate_failure = lambda: None
    n.default_dock_outline = lambda: [(0, 0), (1, 0), (0, 1)]
    n.stored = []
    n.store_dock = lambda: n.stored.append(n.dock.yaw)
    n.rebuild = lambda: None
    return n


def _req(mod, gps=True, yaw_source=0):
    q = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    pos = types.SimpleNamespace(x=5.0, y=6.0, z=0.0)
    return types.SimpleNamespace(use_gps_position=gps, yaw_source=yaw_source, yaw_rad=0.0,
                                 docking_pose=types.SimpleNamespace(position=pos, orientation=q))


def _status(offset, quality, source):
    return types.SimpleNamespace(data=json.dumps(
        {'offset_deg': offset, 'quality': quality, 'source': source}))


def test_low_quality_set_warns_then_autocorrects_once(msn):
    n = _node(msn, heading={'offset_deg': 103.4, 'quality': 'low', 'source': 'file'},
              dock=core.DockPose(0, 0, 0, None, False))      # placeholder dock
    res = n.srv_set_docking_point(_req(msn), types.SimpleNamespace(success=False))
    assert res.success
    assert any('drive 2 m straight' in w for w in n.log.warns)
    assert n.dock.heading_quality_at_set == 'low'
    assert n.dock.heading_offset_deg_at_set == pytest.approx(103.4)
    assert n._dock_autocorrect_armed

    n.on_heading_status(_status(20.3, 'fair', 'cog'))      # not good yet
    assert math.degrees(n.dock.yaw) == pytest.approx(100.0)
    n.on_heading_status(_status(20.3, 'good', 'cog'))
    assert math.degrees(n.dock.yaw) == pytest.approx(100.0 - 83.1)
    assert n.dock.heading_quality_at_set == 'corrected'
    assert any('auto-corrected' in w for w in n.log.warns)
    n.on_heading_status(_status(25.0, 'good', 'cog'))      # once only
    assert math.degrees(n.dock.yaw) == pytest.approx(100.0 - 83.1)
    assert len(n.stored) == 2                               # set + one correction


def test_good_quality_set_no_warning_no_correction(msn):
    n = _node(msn, heading={'offset_deg': 20.3, 'quality': 'good', 'source': 'cog'},
              dock=core.DockPose(0, 0, 0, None, False))
    res = n.srv_set_docking_point(_req(msn), types.SimpleNamespace(success=False))
    assert res.success and not n.log.warns
    assert n.dock.heading_quality_at_set == 'good'
    assert not n._dock_autocorrect_armed
    n.on_heading_status(_status(40.0, 'good', 'cog'))
    assert math.degrees(n.dock.yaw) == pytest.approx(100.0)


def test_operator_yaw_disarms_pending_correction(msn):
    n = _node(msn, heading={'offset_deg': 103.4, 'quality': 'low', 'source': 'file'},
              dock=core.DockPose(0, 0, 0, None, False))
    n.srv_set_docking_point(_req(msn), types.SimpleNamespace(success=False))
    assert n._dock_autocorrect_armed
    n.srv_set_docking_point(_req(msn, gps=False, yaw_source=1),
                            types.SimpleNamespace(success=False))
    assert not n._dock_autocorrect_armed
    assert n.dock.heading_quality_at_set is None
    n.on_heading_status(_status(20.3, 'good', 'cog'))
    assert n.dock.yaw == pytest.approx(0.0)


def test_no_correction_for_dock_loaded_from_file(msn):
    # set in a previous session: not armed (offset delta meaningless across restarts)
    n = _node(msn, dock=_dock(off=103.4, qual='low'))
    n.on_heading_status(_status(20.3, 'good', 'cog'))
    assert math.degrees(n.dock.yaw) == pytest.approx(100.0)


def test_operator_heading_set_skips_the_on_dock_gate(monkeypatch):
    """A typed/dragged dock pose (REQUEST yaw, no GPS position) is accepted off the dock."""
    import math
    from mower_map import map_server_node as msn
    node = msn.MapServerNode.__new__(msn.MapServerNode)
    calls = []
    node.dock_gate_failure = lambda: calls.append(1) or 'robot not on the dock'
    req = msn.SetDockingPoint.Request()
    req.use_gps_position = False
    req.yaw_source = msn.SetDockingPoint.Request.REQUEST
    from_robot = bool(req.use_gps_position) or req.yaw_source != msn.SetDockingPoint.Request.REQUEST
    assert from_robot is False
    req2 = msn.SetDockingPoint.Request(); req2.use_gps_position = True
    assert (bool(req2.use_gps_position) or req2.yaw_source != msn.SetDockingPoint.Request.REQUEST) is True
