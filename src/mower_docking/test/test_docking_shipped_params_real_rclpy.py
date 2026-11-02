"""Review 2026-11-02 H5: config/docking.yaml is the single source of every mower_docking
parameter value: the node's declared defaults (DockParams / ControllerGains / literals)
must equal it. Fails on any future drift. Needs ROS 2 (constructs the node)."""

import dataclasses

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mower_interfaces.action')

from mower_common.shipped_params import drift, shipped_params  # noqa: E402

DOMAIN = 89


@pytest.fixture(scope='module')
def declared():
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(domain_id=DOMAIN)
    from mower_docking.docking_node import DockingServer
    node = DockingServer()
    try:
        yield {name: p.value for name, p in node._parameters.items()}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_docking_defaults_match_the_shipped_yaml(declared):
    shipped = shipped_params('mower_docking', 'docking.yaml', 'mower_docking')
    assert shipped
    assert drift(declared, shipped) == {}
    assert sorted(set(shipped) - set(declared)) == []


def test_dock_params_dataclass_matches_the_yaml():
    from mower_docking import dock_logic as dl
    shipped = shipped_params('mower_docking', 'docking.yaml', 'mower_docking')
    dp = dl.DockParams()
    defaults = {f.name: getattr(dp, f.name) for f in dataclasses.fields(dp)}
    assert drift(defaults, shipped) == {}
    assert (dp.max_retries, dp.marker_lost_hold_s, dp.search_min_detections) == (8, 8.0, 3)


def test_dock_search_debounce_shipped():
    """The shipped search_min_detections (3): two detections stay in SEARCH, the third
    starts DOCKING."""
    from mower_docking import dock_logic as dl
    p = dl.DockParams(skip_nav_to_approach=True, marker_filter=False, pose_check_distance=0.0)
    m = dl.DockStateMachine(p, dl.Pose2D())
    m.start(dl.Snapshot(t=0.0))
    states = [m.step(dl.Snapshot(t=t, marker=dl.MarkerObs(-1.3, 0.0, 0.0, stamp=t))).state
              for t in (0.1, 0.15, 0.2)]
    assert states[:2] == [dl.DockState.SEARCHING] * 2 and states[2] == dl.DockState.DOCKING
