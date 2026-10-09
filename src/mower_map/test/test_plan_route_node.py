# SPDX-License-Identifier: GPL-3.0-or-later
"""map_server_node ~/plan_route (2026-10-09 route graph) on a ROS-free stub node (same stubs
as test_dock_yaw_autocorrect): request -> route, latched ~/route + /plan publish, lazy graph
rebuild when the map changes, failure reasons."""
import os
import types
from unittest import mock

from mower_map import areas as core

from test_dock_yaw_autocorrect import _Log, msn  # noqa: F401 (fixture)

DATA = os.path.join(os.path.dirname(__file__), 'data')


def _pose(x, y, th=0.0):
    return types.SimpleNamespace(x=x, y=y, theta=th)


def _node(mod, areas, dock):
    n = mod.MapServerNode.__new__(mod.MapServerNode)
    params = dict(mod.PARAMS)
    n.p = params.get
    n.params = params
    log = _Log()
    n.get_logger = lambda: log
    n.log = log
    from mower_map import area_settings as aset
    n.settings = aset.AreaSettingsStore()
    n.store = core.MapStore()
    n.store.load(areas)
    n.dock = dock
    n.map_frame = 'map'
    n.route_pub = mock.MagicMock()
    n.route_plan_pub = mock.MagicMock()
    n._route_graph = None
    n._route_key = None
    try:                                   # real messages under rclpy need a real stamp
        from builtin_interfaces.msg import Time
        stamp = Time()
    except ImportError:
        stamp = mock.MagicMock()
    n.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(
        to_msg=lambda: stamp))
    return n


def _req(start, goal=None, to_dock=False):
    return types.SimpleNamespace(start=_pose(*start), goal=_pose(*(goal or (0.0, 0.0))),
                                 to_dock=to_dock)


def _res():
    return types.SimpleNamespace(success=False, message='', path=None, single_area=0,
                                 length_m=0.0, path_length_m=0.0)


def _field(mod):
    areas, _ = core.load_areas_file(os.path.join(DATA, 'field_2026-10-08_paths.dat'))
    dock = core.load_dock_file(os.path.join(DATA, 'field_2026-10-08_dock.yaml'))
    return _node(mod, areas, dock)


def test_plan_route_dock_to_lawn_publishes(msn):
    n = _field(msn)
    res = n.srv_plan_route(_req((0.394, -1.29, 1.76), (-19.0, -6.0, 0.0)), _res())
    assert res.success, res.message
    assert 'Dock path' in res.message and 'Path 2' in res.message
    assert res.single_area == -1
    assert res.path_length_m > 20.0 and res.length_m > res.path_length_m
    n.route_pub.publish.assert_called_once()
    n.route_plan_pub.publish.assert_called_once()


def test_plan_route_to_dock_goal(msn):
    n = _field(msn)
    res = n.srv_plan_route(_req((-19.0, -6.0, 0.0), to_dock=True), _res())
    assert res.success, res.message
    # lawn section -> Path 2 -> Area 1 -> Dock path; it ends at the approach pose, so the
    # dock -> approach leg is never driven
    assert res.message.split(',')[-1].strip().startswith('Dock path')
    assert res.path_length_m > 20.0


def test_plan_route_off_graph_fails_without_publishing(msn):
    n = _field(msn)
    res = n.srv_plan_route(_req((40.0, 40.0), (-19.0, -6.0, 0.0)), _res())
    assert not res.success and 'off the route graph' in res.message
    n.route_pub.publish.assert_not_called()


def test_plan_route_disabled_and_no_dock(msn):
    n = _field(msn)
    n.params['route_enabled'] = False
    assert not n.srv_plan_route(_req((0.0, 0.0), (1.0, 1.0)), _res()).success
    n.params['route_enabled'] = True
    n.dock = None
    res = n.srv_plan_route(_req((-19.0, -6.0), to_dock=True), _res())
    assert not res.success and res.message == 'no dock pose'


def test_graph_rebuilt_when_map_changes(msn):
    n = _field(msn)
    n.srv_plan_route(_req((-19.0, -6.0), (-20.0, -5.0, 0.0)), _res())
    g1 = n._route_graph
    n.srv_plan_route(_req((-19.0, -6.0), (-20.0, -5.0, 0.0)), _res())
    assert n._route_graph is g1                       # unchanged map: same graph
    n.store.areas[1].obstacles.append(core.Obstacle([(-20, -8), (-19, -8), (-19, -7)]))
    n.srv_plan_route(_req((-19.0, -6.0), (-20.0, -5.0, 0.0)), _res())
    assert n._route_graph is not g1                   # obstacle added: rebuilt


def test_service_plans_the_same_route_as_the_pure_function_v3(msn):
    """2026-10-09: a running map_server_node logged a different graph than the offline
    RouteGraph (26 vs 116 nodes; it was a process started before the rebuild). The node's
    parameter plumbing must give exactly the pure function's route on the same map."""
    from mower_map import route_graph as rg
    areas, _ = core.load_areas_file(os.path.join(DATA, 'sim_garden_paths_v3.dat'))
    dock = core.load_dock_file(os.path.join(DATA, 'sim_garden_dock.yaml'))
    n = _node(msn, areas, dock)
    res = n.srv_plan_route(_req((0.82, 0.0, 0.0), (23.68, -9.01, 0.0)), _res())
    # the node routes on the areas WITH the automatic dock no-go (2026-10-09)
    nogo_areas = core.with_dock_nogo(areas, dock, 0.15)
    pure = rg.RouteGraph(nogo_areas, dock, rg.RouteParams()).plan((0.82, 0.0, 0.0),
                                                                  (23.68, -9.01, 0.0))
    assert res.success and isinstance(pure, rg.Route)
    assert res.message == pure.summary()
    assert abs(res.length_m - pure.length_m) < 1e-6
    assert len(n._route_graph.xy) == len(rg.RouteGraph(nogo_areas, dock).xy)
    assert n.route_params() == rg.RouteParams()       # node defaults == library defaults


def test_get_mowing_area_carries_the_dock_nogo(msn):
    """Coverage gets the dock as an automatic no-go; the stored map is unchanged."""
    areas, _ = core.load_areas_file(os.path.join(DATA, 'sim_garden_paths_v3.dat'))
    dock = core.load_dock_file(os.path.join(DATA, 'sim_garden_dock.yaml'))
    n = _node(msn, areas, dock)
    lawn = next(i for i, a in enumerate(n.store.areas) if a.name == 'lawn')
    res = n.srv_get_mowing_area(type('R', (), {'index': lawn})(), type('S', (), {})())
    assert res.success
    names = [o.name for o in res.area.obstacle_info]
    assert core.DOCK_NOGO_NAME in names
    assert core.DOCK_NOGO_NAME not in [o.name for o in n.store.areas[lawn].obstacles]
