"""Stock-parity guards (2026-11-03): battery temperature, critical battery, camera blocked,
recorded-vertex RTK quality."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import Harness, mowing  # noqa: E402


def notices(h, m, incident):
    return [e for e in h.since(m, f.Notice) if e.incident == incident]


# ---- 1. battery temperature ---------------------------------------------
def test_start_refused_when_battery_hot_or_cold():
    h = Harness()
    h.fsm.inputs.battery_temp_c = 47.0
    m = h.mark()
    assert not h.cmd(f.CMD_START)
    assert h.name == 'IDLE' and h.fsm.sub_state.startswith(
        'preflight failed: battery temperature 47 C above 45 C')
    n = notices(h, m, 'battery_temp')
    assert n and n[-1].active and n[-1].kind == 'battery'
    h.fsm.inputs.battery_temp_c = -3.0
    assert not h.cmd(f.CMD_START)
    assert 'below 0 C' in h.fsm.sub_state
    m = h.mark()
    h.fsm.inputs.battery_temp_c = 30.0
    h.tick()
    assert notices(h, m, 'battery_temp')[-1].active is False


def test_unknown_battery_temperature_does_not_block_start():
    h = Harness()
    h.fsm.inputs.battery_temp_c = None
    assert h.cmd(f.CMD_START)


def test_hot_battery_mid_mow_docks_and_resumes_when_cool():
    h = Harness(battery_temp_debounce_s=2.0)
    mowing(h)
    h.fsm.inputs.battery_temp_c = 56.0
    h.tick(n=10)
    assert h.name == 'MOWING'                       # debounce
    m = h.mark()
    h.tick(n=15)
    assert h.name == 'RETURNING_HOME' and not h.blade
    assert notices(h, m, 'battery_temp')[-1].active
    h.fsm.inputs.docked = True
    h.fsm.inputs.is_charging = True
    h.finish(f.ACT_DOCK)
    assert h.name == 'CHARGING' and 'waiting to cool' in h.fsm.sub_state
    h.fsm.inputs.battery_temp_c = 50.0              # cooler, still above max_start
    h.tick(n=5)
    assert h.name == 'CHARGING' and h.fsm.mission is None
    h.fsm.inputs.battery_temp_c = 40.0
    h.tick()
    assert h.fsm.mission is not None and h.fsm.mission.resume


# ---- 2. critical battery ------------------------------------------------
def poweroff_calls(h, m):
    return [e for e in h.since(m, f.CallService) if e.name == f.SRV_POWEROFF]


def test_critical_battery_stops_then_requests_poweroff_after_grace():
    h = Harness(battery_critical_grace_s=10.0)
    mowing(h)
    m = h.mark()
    h.fsm.inputs.battery_percent = 4.0
    h.tick()
    assert h.name == 'IDLE' and 'battery critical' in h.fsm.sub_state and not h.blade
    assert h.since(m, f.ZeroBurst)
    n = notices(h, m, 'battery_critical')
    assert n and n[-1].active and n[-1].priority == 5
    h.tick(dt=1.0, n=8)
    assert not poweroff_calls(h, m)
    h.tick(dt=1.0, n=3)
    assert len(poweroff_calls(h, m)) == 1
    h.tick(dt=1.0, n=5)
    assert len(poweroff_calls(h, m)) == 1          # asked once


def test_critical_battery_put_on_dock_in_grace_cancels_poweroff():
    h = Harness(battery_critical_grace_s=10.0)
    h.fsm.inputs.battery_percent = 3.0
    h.tick()
    m = h.mark()
    h.tick(dt=1.0, n=5)
    h.fsm.inputs.docked = True
    h.fsm.inputs.is_charging = True
    h.tick(dt=1.0, n=20)
    assert not poweroff_calls(h, m)
    assert notices(h, m, 'battery_critical')[-1].active is False


def test_critical_battery_poweroff_disabled():
    h = Harness(battery_critical_grace_s=1.0, battery_critical_poweroff=False)
    h.fsm.inputs.battery_percent = 3.0
    m = h.mark()
    h.tick(dt=1.0, n=5)
    assert not poweroff_calls(h, m) and notices(h, m, 'battery_critical')[-1].active


# ---- 4. camera blocked --------------------------------------------------
def controller_caps(h, m):
    return [e.request['params'] for e in h.since(m, f.CallService)
            if e.name == f.SRV_SET_PARAMS and e.request['node'] == f.PARAM_NODE_CONTROLLER]


def test_camera_blocked_caps_speed_and_restores():
    h = Harness()
    mowing(h)
    m = h.mark()
    h.fsm.inputs.camera_blocked = True
    h.tick()
    caps = controller_caps(h, m)
    assert caps and caps[-1][h.fsm.p.tilt_transit_speed_param] == 0.15
    assert all(v <= 0.15 for v in caps[-1].values())
    assert h.name == 'MOWING'                      # keeps mowing, slowly
    h.tick(n=5)
    assert len(controller_caps(h, m)) == 1         # sent once
    m = h.mark()
    h.fsm.inputs.camera_blocked = False
    h.tick()
    restored = controller_caps(h, m)
    assert restored and restored[-1][h.fsm.p.tilt_transit_speed_param] == 0.3


def test_camera_blocked_ignored_when_idle():
    h = Harness()
    m = h.mark()
    h.fsm.inputs.camera_blocked = True
    h.tick(n=3)
    assert not controller_caps(h, m)


# ---- 6. recorded-vertex RTK quality -------------------------------------
from test_mission_fsm import drive_rectangle, square  # noqa: E402


def record_area(h, float_corner=None):
    """Drive a 4 x 3 m rectangle; the samples near ``float_corner`` see RTK float."""
    h.areas = [square(0, 0, 5)]
    i = h.fsm.inputs
    i.hdop = 0.6
    h.cmd(f.CMD_RECORD_AREA)
    orig = h.tick

    def tick(dt=0.1, n=1):
        x, y = i.pose[:2]
        near = float_corner is not None and abs(x - float_corner[0]) < 0.3 \
            and abs(y - float_corner[1]) < 0.3
        i.fix_type, i.hdop = (2, 1.8) if near else (3, 0.6)
        orig(dt, n)
    h.tick = tick
    drive_rectangle(h)
    h.tick = orig
    m = h.mark()
    assert h.cmd(f.CMD_RECORD_FINISH)
    h.answer_services()
    return m


def test_recorded_vertices_carry_fix_type_and_hdop():
    h = Harness()
    m = record_area(h)
    q = h.since(m, f.SaveRecordingQuality)
    assert len(q) == 1 and q[0].name == 'Area 2' and q[0].kind == 'area'
    assert len(q[0].vertices) == 4
    assert all(v['fixed'] and v['fix_type'] == 3 and v['hdop'] == 0.6 for v in q[0].vertices)
    assert h.name == 'RECORDING_COMPLETE' and h.fsm.sub_state == 'Area 2'


def test_float_vertex_is_flagged_and_warned():
    h = Harness()
    m = record_area(h, float_corner=(4.0, 3.0))
    q = h.since(m, f.SaveRecordingQuality)[0]
    bad = [v for v in q.vertices if not v['fixed']]
    assert len(bad) == 1 and bad[0]['fix_type'] == 2 and bad[0]['hdop'] == 1.8
    assert '1 vertices without RTK fixed' in h.fsm.sub_state
    assert any('without RTK fixed' in e.text for e in h.since(m, f.Log))
