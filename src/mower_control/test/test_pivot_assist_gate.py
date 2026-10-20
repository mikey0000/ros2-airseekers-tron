"""cmd_vel_slew pivot assist in the loop (2026-10-13 review M1): navigation lane only,
collision gate, docking exempt, and no one-wheel tick with the slew in the loop."""

import os
import types

import pytest
import yaml

from mower_control import cmd_vel_slew as cs

DT = 0.05
HERE = os.path.dirname(os.path.abspath(__file__))
MUX_YAML = os.path.join(HERE, '..', '..', 'mower_teleop', 'config', 'twist_mux.yaml')


def _gate(state):
    g = cs.MotionGate()
    g.set_motion_enabled(True, 0.0)
    g.set_status(state, False)
    g.set_base(False, False, False, False)
    return g


def _free_costmap(ag, t, lethal_at=()):
    cm = [0] * (120 * 120)
    ag.costmap.set_grid(0.05, 120, 120, -3.0, -3.0, cm, t)
    for x, y in lethal_at:
        i, j = int((x + 3.0) / 0.05), int((y + 3.0) / 0.05)
        ag.costmap.update(i, j, 1, 1, [100], t)


def _node(state='MOWING', lethal_at=()):
    log = types.SimpleNamespace()
    for level in ('debug', 'info', 'warn', 'warning', 'error'):
        setattr(log, level, lambda text: None)
    ag = cs.PivotAssistGate()
    _free_costmap(ag, 0.0, lethal_at)
    ag.note_pose(0.0, 0.0, 0.0, 0.0)
    return types.SimpleNamespace(
        _max_linear_accel=0.5, _max_angular_accel=1.0, _cmd_timeout=0.5,
        _log_period=1.0, _cur_lin=0.0, _cur_ang=0.0, _tgt=None, _last_cmd_time=None,
        _last_tick=None, _next_log=0.0, get_logger=lambda: log, _gate=_gate(state),
        _pivot=cs.PivotAssist(), _turn=cs.TurnShaper(), _assist_gate=ag)


def run(node, seconds, lin=0.0, ang=0.3, lanes=('navigation',), controller=True,
        t0=0.0, each=None):
    """Feed a pivot command for ``seconds``; returns [(t, v, w)] of the slew output."""
    ag = node._assist_gate
    out = []
    t = t0
    n = int(round(seconds / DT))
    for _ in range(n):
        t = round(t + DT, 6)
        node._tgt, node._last_cmd_time = (lin, 0.0, 0.0, 0.0, 0.0, ang), t
        for name in lanes:
            ag.note_lane(name, t)
        if controller:
            ag.note_controller(t)
        ag.note_pose(0.0, 0.0, 0.0, t)
        node._gate.set_motion_enabled(True, t)
        ag.costmap.stamp = t if ag.costmap.stamp is not None and each != 'stale' \
            else ag.costmap.stamp
        if callable(each):
            each(node, t)
        o = cs.CmdVelSlewNode.step(node, t)
        if o is not None:
            out.append((t, o[0], o[5]))
    return out


def max_v(out):
    return max(v for _t, v, _w in out)


# ------------------------------------------------------------------------ lanes
def test_lane_table_matches_twist_mux_yaml():
    with open(MUX_YAML, encoding='utf-8') as f:
        topics = yaml.safe_load(f)['twist_mux']['ros__parameters']['topics']
    got = {name: (topic, prio, to) for name, topic, prio, to in cs.TWIST_MUX_LANES}
    want = {name: (d['topic'], d['priority'], d['timeout']) for name, d in topics.items()}
    assert got == want


def test_lane_tracker_priority_and_timeout():
    lt = cs.LaneTracker()
    assert lt.active(0.0) is None
    lt.note('navigation', 1.0)
    lt.note('bumper', 1.0)
    assert lt.active(1.1) == 'bumper'
    assert lt.active(1.55) == 'navigation'      # bumper 0.5 s timeout, nav 0.6 s
    assert lt.active(1.65) is None


def test_nav_lane_followpath_pivot_creeps():
    out = run(_node(), 3.0)
    assert max_v(out) == pytest.approx(0.14)


@pytest.mark.parametrize('lanes', [('navigation', 'bumper'), ('navigation', 'teleop'),
                                   ('navigation', 'emergency'), ('navigation', 'docking'),
                                   ('bumper',), ('teleop',), ()])
def test_never_on_other_lanes(lanes):
    """Bumper rotate-clear, teleop, emergency, docking: no creep (review M1 (ii))."""
    out = run(_node(), 4.0, lanes=lanes)
    assert max_v(out) == 0.0


def test_nav2_spin_recovery_never_creeps():
    """behavior_server Spin publishes on /cmd_vel_nav with no controller command."""
    out = run(_node(), 4.0, controller=False)
    assert max_v(out) == 0.0


def test_bumper_holdoff_after_rotate_clear():
    node = _node()
    run(node, 1.0, lanes=('bumper', 'navigation'))      # rotate-clear on the bumper lane
    out = run(node, 4.0, t0=1.0)                       # nav resumes the pivot
    assert max_v(out) == 0.0                           # within the 5 s hold-off
    out = run(node, 3.0, t0=6.5)
    assert max_v(out) == pytest.approx(0.14)           # hold-off over


# ------------------------------------------------------------------ collision gate
def test_lethal_cell_in_the_creep_sweep_holds_the_creep():
    out = run(_node(lethal_at=[(0.75, 0.05)]), 4.0)
    assert max_v(out) == 0.0


def test_lethal_cell_behind_or_aside_does_not_hold():
    out = run(_node(lethal_at=[(-0.8, 0.0), (0.3, 1.2)]), 3.0)
    assert max_v(out) == pytest.approx(0.14)


def test_obstacle_appearing_mid_creep_stops_it():
    def drop(node, t):
        if abs(t - 1.6) < 1e-6:
            _free_costmap(node._assist_gate, t, [(0.7, 0.0)])
    out = run(_node(), 3.0, each=drop)
    assert max_v(out) > 0.1
    assert out[-1][1] == 0.0
    crept = sum(v * DT for _t, v, _w in out)
    assert crept < 0.12


def test_stale_costmap_holds_the_creep():
    node = _node()
    node._assist_gate.costmap.stamp = -10.0
    out = run(node, 3.0, each='stale')
    assert max_v(out) == 0.0


@pytest.mark.parametrize('policy,held', [
    ({'kind': 'static', 'camera': 'front', 'distance_m': 1.0}, True),
    ({'kind': 'dynamic', 'camera': 'front', 'distance_m': None}, True),
    ({'kind': 'static', 'camera': 'front', 'distance_m': 2.5}, False),
    ({'kind': 'dynamic', 'camera': 'rear', 'distance_m': 0.5}, False),
    ({'kind': 'none'}, False)])
def test_vision_policy(policy, held):
    out = run(_node(), 3.0,
              each=lambda node, t: node._assist_gate.note_policy(policy, t))
    assert (max_v(out) == 0.0) == held


def test_obstacle_close_flag_holds():
    out = run(_node(), 3.0,
              each=lambda node, t: node._assist_gate.note_obstacle_close(True, t))
    assert max_v(out) == 0.0


def test_no_gate_object_never_assists():
    node = _node()
    node._assist_gate = None
    node._tgt = None
    out = []
    for k in range(1, 80):
        t = k * DT
        node._tgt, node._last_cmd_time = (0.0, 0.0, 0.0, 0.0, 0.0, 0.3), t
        o = cs.CmdVelSlewNode.step(node, t)
        if o is not None:
            out.append(o[0])
    assert max(out) == 0.0


# ------------------------------------------------------------------ docking exempt
@pytest.mark.parametrize('phase', ['RETURNING_HOME', 'UNDOCKING', 'LOW_BATTERY_DOCKING',
                                   'RAIN_DETECTED_DOCKING', 'COVERAGE_FAILED_DOCKING',
                                   'MANUAL_MOWING'])
def test_docking_and_manual_phases_exempt(phase):
    out = run(_node(state=phase), 4.0)
    assert max_v(out) == 0.0


# --------------------------------------------------------- no one-wheel tick in the loop
def _one_wheel(v, w):
    return not cs.both_wheels_legal(v, w)


@pytest.mark.parametrize('ang', [0.3, -0.3, 0.25, 0.15, 0.08])
def test_assisted_pivot_has_no_one_wheel_tick(ang):
    """Review M1 (iii): the 0 -> 0.14 ramp at |w| = 0.3 spent 5-9 ticks in the one-wheel
    zone (831 of 3251 assisted ticks in the 10-10/10-11 bags). Whole assisted pivot:
    delay, creep up, 0.3 m cap, back to a pure pivot."""
    node = _node()
    out = run(node, 5.0, ang=ang)            # delay, creep up, cap, back to a pure pivot
    assert node._assist_ramp is False        # episode over once settled on the pivot
    bad = [(t, v, w) for t, v, w in out if _one_wheel(v, w)]
    assert not bad
    if abs(ang) * 0.20 + 0.06 <= 0.14 + 1e-9:
        assert max_v(out[:100]) == pytest.approx(0.14)


def test_slew_both_wheels_accel_limits_and_reaches_targets():
    pairs = [((0.0, 0.3), (0.14, 0.3)), ((0.14, 0.3), (0.0, 0.3)),
             ((0.3, 0.0), (0.0, -0.3)), ((0.0, -0.3), (-0.2, 0.2)),
             ((0.2, 0.3), (0.0, 0.0)), ((0.18, 0.5), (0.0, 0.25))]
    for (v, w), (tv, tw) in pairs:
        for _ in range(200):
            nv, nw = cs.slew_both_wheels(v, w, tv, tw, 0.5, 1.0, DT)
            assert abs(nv - v) <= 0.5 * DT + 1e-9 and abs(nw - w) <= 1.0 * DT + 1e-9
            assert cs.both_wheels_legal(nv, nw), ((v, w), (tv, tw), (nv, nw))
            v, w = nv, nw
        assert (v, w) == pytest.approx((tv, tw))


def test_normal_arcs_slew_independently():
    assert cs.slew_both_wheels(0.2, 0.1, 0.3, 0.2, 0.5, 1.0, DT) == pytest.approx(
        (0.225, 0.15))


def test_held_creep_ramps_down_without_one_wheel_tick():
    def drop(node, t):
        if abs(t - 1.6) < 1e-6:
            _free_costmap(node._assist_gate, t, [(0.7, 0.0)])
    out = run(_node(), 3.0, each=drop)
    assert not [o for o in out if _one_wheel(o[1], o[2])]


@pytest.mark.parametrize('enabled', [False, True])
def test_normal_driving_keeps_the_plain_slew(enabled):
    """Coordinator 2026-10-13: outside an assist episode the slew is exactly the old
    per-axis slew of the TurnShaper target (no forced stop between pivot and arc)."""
    node = _node(state='TRANSIT')
    node._slew_both_wheels_always = False    # 2026-10-14: the default is now slew_coupled
    node._pivot.enabled = enabled
    seq = [(0.0, 0.3)] * 10 + [(0.25, 0.1)] * 20 + [(0.0, -0.3)] * 10 + [(0.2, 0.5)] * 10
    v = w = 0.0
    t = 0.0
    cs.CmdVelSlewNode.step(node, t)
    for lin, ang in seq:
        t = round(t + DT, 6)
        node._tgt, node._last_cmd_time = (lin, 0.0, 0.0, 0.0, 0.0, ang), t
        node._assist_gate.note_lane('navigation', t)
        node._assist_gate.note_controller(t)
        node._gate.set_motion_enabled(True, t)
        o = cs.CmdVelSlewNode.step(node, t)
        sv, sw, _ = cs.shape_for_both_wheels(lin, ang)
        v, w = cs.slew(v, sv, 0.5, DT), cs.slew(w, sw, 1.0, DT)
        v, w = cs.pivot_floor_step(v, w, sv, sw)    # 2026-10-20: no pivot under the floor
        assert (o[0], o[5]) == pytest.approx((v, w))
    assert node._assist_ramp is False
