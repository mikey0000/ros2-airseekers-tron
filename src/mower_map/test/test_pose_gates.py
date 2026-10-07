# SPDX-License-Identifier: GPL-3.0-or-later
"""Map-pose plausibility gate and lethal persistence (2026-10-07 incident: a test feeder
on the dev host published /odometry/filtered_map x=1,3,..,27 y=0 into the mowing robot's
graph; one sample at (27, 0) latched BOUNDARY_EMERGENCY_STOP)."""

from mower_map import areas as core


def test_jump_rejected_within_window():
    g = core.PoseJumpGate(2.0, 1.0)
    assert g.check(0.0, -1.44, 4.02) == (True, 0.0)
    ok, jump = g.check(0.05, 27.0, 0.0)
    assert not ok and jump > 28
    assert g.check(0.10, -1.43, 4.03)[0]          # real pose keeps flowing


def test_small_motion_accepted():
    g = core.PoseJumpGate(2.0, 1.0)
    t, x = 0.0, 0.0
    for _ in range(100):
        assert g.check(t, x, 0.0)[0]
        t += 0.05
        x += 0.3 * 0.05


def test_genuine_relocalization_recovers_after_window():
    g = core.PoseJumpGate(2.0, 1.0)
    assert g.check(0.0, 0.0, 0.0)[0]
    assert not g.check(0.5, 5.0, 0.0)[0]
    assert not g.check(0.9, 5.0, 0.0)[0]
    assert g.check(1.05, 5.0, 0.0)[0]             # nothing accepted for > 1 s: take it
    assert g.check(1.10, 5.01, 0.0)[0]


def test_disabled_gate():
    g = core.PoseJumpGate(0.0, 1.0)
    assert g.check(0.0, 0.0, 0.0)[0]
    assert g.check(0.01, 100.0, 0.0)[0]


def test_incident_replay_never_reaches_lethal():
    """Real 20 Hz poses inside the lawn interleaved with the 50 Hz injected feeder."""
    g = core.PoseJumpGate(2.0, 1.0)
    lethal = core.PersistenceFilter(1.0)
    events = [(i * 0.05, -1.44 + 0.0015 * i, 4.02) for i in range(40)]
    events += [(0.3 + k * 0.01, 1.0 + k, 0.0) for k in range(0, 27, 2)]
    events.sort()
    for t, x, y in events:
        ok, _ = g.check(t, x, y)
        outside = x > 0.5                         # the lawn edge is near x = 0.67 here
        if ok:
            assert not lethal.update(t, outside)


def test_lethal_needs_persistence():
    f = core.PersistenceFilter(1.0)
    assert not f.update(0.0, True)
    assert not f.update(0.5, True)
    assert not f.update(0.6, False)               # reset
    assert not f.update(0.7, True)
    assert not f.update(1.6, True)
    assert f.update(1.71, True)
    assert not f.update(1.8, False)


def test_lethal_zero_duration_is_immediate():
    f = core.PersistenceFilter(0.0)
    assert f.update(0.0, True)
