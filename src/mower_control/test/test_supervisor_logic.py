"""Supervisor liveness bookkeeping (mower_control.supervisor_logic)."""

from mower_control.supervisor_logic import LivenessTracker


def run(tr, t0, t1, present, dt=1.0):
    events = []
    t = t0
    while t <= t1 + 1e-9:
        events += tr.update(t, present)
        t += dt
    return events


def test_vanished_node_goes_down_then_respawn_counts_restart():
    tr = LivenessTracker(adopt_after_s=5, down_after_s=2.5, now=0)
    assert run(tr, 0, 10, {'/obstacle_guard', '/x'}) == []
    ev = run(tr, 11, 13, {'/x'})
    assert ev == [('down', '/obstacle_guard', 'vanished from the ROS graph')]
    st = tr.status(13)
    assert not st['ok'] and st['down'] == ['/obstacle_guard'] and st['critical_down'] == []
    assert st['nodes']['/obstacle_guard']['alive'] is False
    ev = tr.update(16, {'/x', '/obstacle_guard'})
    assert ev[0][0] == 'up' and ev[0][1] == '/obstacle_guard' and ev[0][2] == 6
    st = tr.status(16)
    assert st['ok'] and st['restarts'] == {'/obstacle_guard': 1}


def test_short_discovery_blip_is_not_a_crash():
    tr = LivenessTracker(adopt_after_s=5, down_after_s=2.5, now=0)
    run(tr, 0, 10, {'/a'})
    assert tr.update(11, set()) == []
    assert tr.update(12, {'/a'}) == []
    assert tr.status(12)['ok']


def test_transient_nodes_are_never_adopted():
    tr = LivenessTracker(adopt_after_s=10, now=0)
    run(tr, 0, 3, {'/probe'})
    assert run(tr, 4, 20, set()) == []
    assert '/probe' not in tr.status(20)['nodes']


def test_ignored_patterns():
    tr = LivenessTracker(adopt_after_s=0, now=0)
    run(tr, 0, 5, {'/_ros2cli_123', '/launch_ros_42', '/a/transform_listener_impl_5f'})
    assert tr.status(5)['nodes'] == {}


def test_bag_recorder_child_not_tracked():
    """bag_recorder stops its child on ring switches; that must not be a node_down."""
    tr = LivenessTracker(adopt_after_s=0, down_after_s=1, now=0)
    run(tr, 0, 5, {'/rosbag2_recorder', '/bag_recorder'})
    assert run(tr, 6, 12, {'/bag_recorder'}) == []
    assert '/rosbag2_recorder' not in tr.status(12)['nodes']


def test_critical_never_started_after_grace():
    tr = LivenessTracker(critical=['mower_mcu_driver'], startup_grace_s=30, now=0)
    assert run(tr, 0, 29, set()) == []
    ev = tr.update(30, set())
    assert ev == [('down', '/mower_mcu_driver', 'never started')]
    st = tr.status(30)
    assert st['down'] == ['/mower_mcu_driver'] and not st['ok']
    assert st['critical_down'] == []        # never started: not a mission emergency


def test_critical_adopted_immediately_and_reported():
    tr = LivenessTracker(critical=['/cmd_vel_slew'], adopt_after_s=60, down_after_s=2, now=0)
    tr.update(0, {'/cmd_vel_slew'})
    ev = run(tr, 1, 3, set())
    assert ev == [('down', '/cmd_vel_slew', 'vanished from the ROS graph')]
    st = tr.status(3)
    assert st['critical_down'] == ['/cmd_vel_slew'] and st['nodes']['/cmd_vel_slew']['critical']
    tr.update(4, {'/cmd_vel_slew'})
    assert tr.status(4)['critical_down'] == []


def test_down_reported_once_per_episode():
    tr = LivenessTracker(adopt_after_s=0, down_after_s=1, now=0)
    tr.update(0, {'/a'})
    ev = run(tr, 1, 30, set())
    assert len(ev) == 1


def test_default_critical_covers_the_mow_dependencies():
    """Review 2026-11-02 H3c."""
    from mower_control.supervisor_logic import DEFAULT_CRITICAL
    assert set(DEFAULT_CRITICAL) >= {
        '/mower_mcu_driver', '/cmd_vel_slew', '/twist_mux', '/behavior_tree_node',
        '/map_server_node', '/ekf_node', '/ekf_map', '/controller_server', '/mower_docking',
        '/gui_bridge'}


def test_paused_lifecycle_node_stays_up_and_exit_is_critical():
    """A Nav2 node PAUSEd at docked_idle (inactive) is still in the graph: not down. Its
    process exiting is critical_down; ekf_map never launched (single mode) is not."""
    from mower_control.supervisor_logic import DEFAULT_CRITICAL
    tr = LivenessTracker(critical=DEFAULT_CRITICAL, now=0.0)
    alive = [n for n in DEFAULT_CRITICAL if n != '/ekf_map']
    assert run(tr, 0, 200, alive) == [('down', '/ekf_map', 'never started')]
    assert tr.critical_down() == []
    gone = [n for n in alive if n != '/controller_server']
    run(tr, 201, 205, gone)
    assert tr.critical_down() == ['/controller_server']


def test_stale_heartbeat_is_down_while_the_node_stays_in_the_graph():
    """Review 2026-11-02 H4: a hung mcu_node keeps its graph entry; its silent
    /mcu/driver_heartbeat (> 2 s) makes it critical_down, the heartbeat resuming brings it
    back UP; a node that never sent a heartbeat is never flagged."""
    from mower_control.supervisor_logic import (DEFAULT_HEARTBEATS, HeartbeatWatch,
                                                parse_heartbeats)
    specs = parse_heartbeats(DEFAULT_HEARTBEATS)
    assert specs == [('/mower_mcu_driver', '/mcu/driver_heartbeat', 2.0)]
    hb = HeartbeatWatch(specs)
    tr = LivenessTracker(critical=['/mower_mcu_driver'], now=0.0)
    graph = ['/mower_mcu_driver']
    assert hb.stale(100.0) == {}                       # not armed yet
    events = []
    for t in range(0, 11):
        hb.beat('/mower_mcu_driver', float(t))
        events += tr.update(float(t), graph, hb.stale(float(t)))
    assert events == [] and tr.critical_down() == []
    events += tr.update(12.0, graph, hb.stale(12.0))   # 2.0 s: not yet
    assert events == []
    events += tr.update(13.0, graph, hb.stale(13.0))   # 3.0 s > 2.0 s: down, still in graph
    assert events == [('down', '/mower_mcu_driver', 'heartbeat silent > 2.0 s')]
    assert tr.critical_down() == ['/mower_mcu_driver']
    assert tr.update(14.0, graph, hb.stale(14.0)) == []     # reported once
    hb.beat('/mower_mcu_driver', 15.0)
    ev = tr.update(15.0, graph, hb.stale(15.0))
    assert ev and ev[0][0] == 'up' and tr.critical_down() == []
