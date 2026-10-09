"""Pure-logic tests for mower_alerts.rules (no ROS): incidents, modes, theft, geofence,
map mask, lift/tilt, mission incidents, RTK, battery, rate limit."""

import math

import pytest

from mower_alerts import rules as r

LAT0, LON0 = 52.0, 13.0
DLAT_33M = 0.0003      # ~33.4 m north


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def run(engine, inputs, t0, seconds, dt=0.5):
    """Step the engine on [t0, t0 + seconds) and return [(t, alert), ...].
    ``inputs`` is an Inputs or a callable t -> Inputs."""
    out = []
    for k in range(int(round(seconds / dt))):
        t = t0 + k * dt
        i = inputs(t) if callable(inputs) else inputs
        out.extend((t, a) for a in engine.step(i, t))
    return out


def msgs(alerts):
    return [a.message for _, a in alerts]


def parked(**kw):
    kw.setdefault('state', r.HL_IDLE)
    kw.setdefault('state_name', 'IDLE_DOCKED')
    kw.setdefault('docked', True)
    return r.Inputs(**kw)


def mission(name='MOWING', **kw):
    kw.setdefault('state', r.HL_AUTONOMOUS)
    return r.Inputs(state_name=name, **kw)


def fix_at(dlat=0.0, acc=1.0, fix_type=3):
    return r.Fix(LAT0 + dlat, LON0, acc, fix_type)


def armed_engine(**params):
    """Engine parked since t=0; armed from t=60."""
    return r.AlertEngine(r.Params(**params))


# ---------------------------------------------------------------------------
# 1. Incident
# ---------------------------------------------------------------------------
def test_incident_debounce_needs_continuous_condition():
    inc = r.Incident(debounce_s=2.0, clear_s=5.0)
    assert inc.update(True, 0.0) is None
    assert inc.update(True, 1.5) is None
    assert inc.update(True, 2.0) == r.FIRE


def test_incident_dropout_before_debounce_restarts_timer():
    inc = r.Incident(debounce_s=2.0, clear_s=5.0)
    assert inc.update(True, 0.0) is None
    assert inc.update(False, 1.0) is None
    assert inc.update(True, 1.5) is None          # timer restarted at 1.5
    assert inc.update(True, 2.5) is None          # would have fired at 2.0 without the dropout
    assert inc.update(True, 3.5) == r.FIRE


def test_incident_fires_once_while_condition_persists():
    inc = r.Incident(debounce_s=0.0, clear_s=5.0)
    assert inc.update(True, 0.0) == r.FIRE
    assert [inc.update(True, t) for t in range(1, 100)] == [None] * 99


def test_incident_rearms_only_after_clear_s_continuous_clear():
    inc = r.Incident(debounce_s=0.0, clear_s=5.0)
    assert inc.update(True, 0.0) == r.FIRE
    assert inc.update(False, 1.0) is None
    assert inc.update(False, 5.0) is None          # 4 s clear
    assert inc.update(True, 5.5) is None           # blip: clear timer restarts, still active
    assert inc.active
    assert inc.update(False, 6.0) is None
    assert inc.update(False, 10.9) is None
    assert inc.update(False, 11.0) == r.CLEARED
    assert not inc.active
    assert inc.update(True, 12.0) == r.FIRE        # armed again


def test_incident_clear_while_inactive_returns_none():
    inc = r.Incident(debounce_s=1.0, clear_s=1.0)
    assert [inc.update(False, t) for t in range(5)] == [None] * 5


def test_incident_repeat_and_max_repeats():
    inc = r.Incident(debounce_s=0.0, clear_s=5.0, repeat_s=10.0, max_repeats=2)
    results = [(t, inc.update(True, float(t))) for t in range(0, 61)]
    fired = [(t, x) for t, x in results if x]
    assert fired == [(0, r.FIRE), (10, r.REPEAT), (20, r.REPEAT)]


def test_incident_repeat_disabled_by_default():
    inc = r.Incident(debounce_s=0.0, clear_s=5.0)
    assert inc.update(True, 0.0) == r.FIRE
    assert inc.update(True, 10000.0) is None


def test_incident_min_interval_blocks_second_fire_after_rearm():
    inc = r.Incident(debounce_s=0.0, clear_s=5.0, min_interval_s=100.0)
    assert inc.update(True, 0.0) == r.FIRE
    assert inc.update(False, 1.0) is None
    assert inc.update(False, 6.0) == r.CLEARED
    for t in (10.0, 50.0, 99.0):
        assert inc.update(True, t) is None         # re-armed but inside min_interval
    assert inc.update(True, 100.0) == r.FIRE


def test_incident_reset():
    inc = r.Incident(debounce_s=0.0, clear_s=5.0)
    inc.update(True, 0.0)
    inc.reset()
    assert not inc.active
    assert inc.update(True, 1.0) == r.FIRE


# ---------------------------------------------------------------------------
# 2. mode_for
# ---------------------------------------------------------------------------
@pytest.mark.parametrize('state,name,mode', [
    (r.HL_AUTONOMOUS, 'MOWING', r.MODE_MISSION),
    (r.HL_RECORDING, 'RECORDING', r.MODE_MANUAL),
    (r.HL_MANUAL_MOWING, 'MANUAL_MOWING', r.MODE_MANUAL),
    (r.HL_IDLE, 'IDLE_DOCKED', r.MODE_PARKED),
])
def test_mode_for_known_states(state, name, mode):
    assert r.mode_for(state, name, r.MODE_UNKNOWN) == mode
    assert r.mode_for(state, name, r.MODE_MISSION) == mode


@pytest.mark.parametrize('previous', [r.MODE_PARKED, r.MODE_MISSION, r.MODE_MANUAL,
                                      r.MODE_UNKNOWN])
def test_mode_for_null_state_keeps_previous(previous):
    assert r.mode_for(r.HL_NULL, 'EMERGENCY', previous) == previous


def test_mode_for_none_keeps_previous():
    assert r.mode_for(None, '', r.MODE_MANUAL) == r.MODE_MANUAL
    assert r.mode_for(None, '') == r.MODE_UNKNOWN


def test_engine_infers_parked_from_dock_contacts_when_state_unknown():
    e = armed_engine()
    e.step(r.Inputs(docked=True), 0.0)
    assert e.mode == r.MODE_PARKED


# ---------------------------------------------------------------------------
# 3. theft lift and the e-stop it trips
# ---------------------------------------------------------------------------
def test_theft_lift_not_before_arming():
    e = armed_engine()
    assert run(e, parked(lift=True, fix=fix_at()), 0.0, 60.0) == []
    assert not e.theft_armed(59.5)
    assert e.theft_armed(60.0)


def test_theft_lift_fires_once_after_arming_with_params():
    e = armed_engine()
    alerts = run(e, parked(lift=True, fix=fix_at()), 0.0, 90.0)
    assert len(alerts) == 1
    t, a = alerts[0]
    assert t == 62.0                                   # armed 60 + lift debounce 2
    assert (a.kind, a.message, a.priority) == ('theft', 'theftLift', 5)
    assert a.incident == 'theft_lift'
    assert a.params['lat'] == '%.6f' % LAT0
    assert a.params['lon'] == '%.6f' % LON0
    assert a.params['map'].startswith('https://maps.google.com/?q=')


def test_theft_lift_without_fix_still_alerts_with_placeholder_position():
    e = armed_engine()
    alerts = run(e, parked(lift=True), 0.0, 70.0)
    assert msgs(alerts) == ['theftLift']
    assert alerts[0][1].params['lat'] == '?'


def test_theft_lift_repeats_up_to_max():
    e = armed_engine()
    alerts = run(e, parked(lift=True, fix=fix_at()), 0.0, 3000.0)
    assert [t for t, _ in alerts] == [62.0 + 300.0 * k for k in range(7)]
    assert set(msgs(alerts)) == {'theftLift'}


def test_theft_lift_repeat_params_follow_param_values():
    e = armed_engine(theft_repeat_s=100.0, theft_max_repeats=2)
    alerts = run(e, parked(lift=True), 0.0, 1000.0)
    assert [t for t, _ in alerts] == [62.0, 162.0, 262.0]


def test_brief_lift_shorter_than_debounce_does_not_fire():
    e = armed_engine()
    run(e, parked(), 0.0, 60.0)
    assert run(e, parked(lift=True), 60.0, 1.5) == []
    assert run(e, parked(lift=False), 61.5, 30.0) == []


def test_lift_estop_is_not_a_second_emergency_alert():
    e = armed_engine()
    run(e, parked(), 0.0, 60.0)
    lifted = run(e, parked(lift=True, hl_emergency=True), 60.0, 10.0)
    assert msgs(lifted) == ['theftLift']
    # put down, e-stop still latched: still silent
    assert run(e, parked(lift=False, hl_emergency=True), 70.0, 30.0) == []
    assert e.estop_from_lift


def test_stop_button_emergency_fires_after_latch_cleared():
    e = armed_engine()
    run(e, parked(), 0.0, 60.0)
    run(e, parked(lift=True, hl_emergency=True), 60.0, 10.0)
    run(e, parked(lift=False, hl_emergency=True), 70.0, 10.0)
    assert run(e, parked(), 80.0, 5.0) == []           # latch cleared
    assert not e.estop_from_lift
    alerts = run(e, parked(hl_emergency=True, stop_button=True), 85.0, 5.0)
    assert msgs(alerts) == ['emergencyStop']
    a = alerts[0][1]
    assert a.kind == 'emergency' and a.priority == 5
    assert 'stop button' in a.params['cause']


def test_emergency_with_unknown_cause_uses_reason_text():
    e = armed_engine()
    alerts = run(e, parked(hl_emergency=True, emergency_reason='watchdog'), 0.0, 5.0)
    assert msgs(alerts) == ['emergencyStop']
    assert alerts[0][1].params['cause'] == 'watchdog'
    e = armed_engine()
    alerts = run(e, parked(hl_emergency=True), 0.0, 5.0)
    assert alerts[0][1].params['cause'] == 'firmware emergency latch'


# ---------------------------------------------------------------------------
# 4. geofence
# ---------------------------------------------------------------------------
def test_anchor_is_first_good_fix_after_arming():
    e = armed_engine()
    run(e, parked(fix=fix_at()), 0.0, 60.0)
    assert e.anchor is None and not e.anchor_changed
    e.step(parked(fix=fix_at()), 60.0)
    assert e.anchor is not None and e.anchor_changed
    assert (e.anchor.lat, e.anchor.lon) == (LAT0, LON0)


def test_noisy_fixes_are_ignored_for_anchor_and_distance():
    e = armed_engine()
    assert run(e, parked(fix=fix_at(DLAT_33M, acc=6.0)), 0.0, 100.0) == []
    assert e.anchor is None
    e.step(parked(fix=fix_at(acc=1.0)), 100.0)
    assert e.anchor is not None and e.anchor.accuracy_m == 1.0
    # noisy far fix after anchoring is ignored too
    assert run(e, parked(fix=fix_at(DLAT_33M, acc=6.0)), 100.5, 60.0) == []


def test_geofence_fires_after_debounce_with_distance():
    e = armed_engine()
    run(e, parked(fix=fix_at()), 0.0, 65.0)
    assert e.anchor is not None
    alerts = run(e, parked(fix=fix_at(DLAT_33M)), 65.0, 30.0)
    assert msgs(alerts) == ['theftGeofence']
    t, a = alerts[0]
    assert t == 75.0                                   # 10 s debounce
    assert a.kind == 'theft' and a.priority == 5
    assert a.params['distance'] == '33 m'


def test_fix_within_radius_does_not_fire():
    e = armed_engine()
    run(e, parked(fix=fix_at()), 0.0, 65.0)
    assert run(e, parked(fix=fix_at(0.00005)), 65.0, 60.0) == []   # ~5.6 m < 10 m


def test_geofence_short_excursion_does_not_fire():
    e = armed_engine()
    run(e, parked(fix=fix_at()), 0.0, 65.0)
    assert run(e, parked(fix=fix_at(DLAT_33M)), 65.0, 5.0) == []
    assert run(e, parked(fix=fix_at()), 70.0, 30.0) == []


@pytest.mark.parametrize('state,name', [(r.HL_AUTONOMOUS, 'MOWING'),
                                        (r.HL_RECORDING, 'RECORDING'),
                                        (r.HL_MANUAL_MOWING, 'MANUAL_MOWING')])
def test_leaving_parked_clears_anchor_and_repark_rearms_after_arm_s(state, name):
    e = armed_engine()
    run(e, parked(fix=fix_at()), 0.0, 65.0)
    assert e.anchor is not None
    e.anchor_changed = False
    e.step(r.Inputs(state=state, state_name=name, fix=fix_at(DLAT_33M)), 65.0)
    assert e.anchor is None and e.anchor_changed
    # parked again somewhere else: not armed for 60 s, no anchor meanwhile
    run(e, parked(fix=fix_at(DLAT_33M)), 100.0, 59.5)
    assert e.anchor is None
    e.step(parked(fix=fix_at(DLAT_33M)), 160.0)
    assert e.anchor is not None and e.anchor.lat == LAT0 + DLAT_33M


def test_restored_anchor_and_powered_up_elsewhere_fires_after_arming():
    e = armed_engine()
    e.restore_anchor(r.Fix(LAT0, LON0, 1.0))
    alerts = run(e, parked(fix=fix_at(DLAT_33M)), 0.0, 100.0)
    assert msgs(alerts) == ['theftGeofence']
    assert alerts[0][0] == 70.0                        # armed 60 + debounce 10
    assert alerts[0][1].params['distance'] == '33 m'


def test_lift_and_geofence_together_send_one_theft_push_per_window():
    e = armed_engine()
    e.restore_anchor(r.Fix(LAT0, LON0, 1.0))
    alerts = run(e, parked(lift=True, fix=fix_at(DLAT_33M)), 0.0, 340.0)
    assert msgs(alerts) == ['theftLift']               # geofence at 70 s is swallowed
    assert 'theft_geofence' in e.active_incidents()
    # the geofence repeat at 370 s falls inside the same window as the lift repeat at 362 s
    alerts = run(e, parked(lift=True, fix=fix_at(DLAT_33M)), 340.0, 60.0)
    assert msgs(alerts) == ['theftLift']


def test_lift_alert_includes_distance_when_anchor_exists():
    e = armed_engine()
    e.restore_anchor(r.Fix(LAT0, LON0, 1.0))
    alerts = run(e, parked(lift=True, fix=fix_at(DLAT_33M)), 0.0, 70.0)
    a = alerts[0][1]
    assert a.message == 'theftLift'
    assert a.params['distance'] == '33 m'
    assert 'Moved 33 m' in a.text


def test_lift_alert_has_no_distance_without_anchor_or_fix():
    e = armed_engine()
    alerts = run(e, parked(lift=True), 0.0, 70.0)
    assert 'distance' not in alerts[0][1].params


# ---------------------------------------------------------------------------
# 5. outside the map
# ---------------------------------------------------------------------------
def make_mask(free=(2.0, 8.0)):
    w = h = 100
    res = 0.1
    data = []
    for row in range(h):
        y = (row + 0.5) * res
        for col in range(w):
            x = (col + 0.5) * res
            data.append(0 if free[0] <= x <= free[1] and free[0] <= y <= free[1] else 100)
    return r.MapMask(0.0, 0.0, res, w, h, data)


def test_distance_outside_zero_inside():
    m = make_mask()
    assert m.has_free
    assert m.distance_outside(5.0, 5.0, 4.0) == 0.0
    assert m.distance_outside(2.5, 7.5, 4.0) == 0.0


def test_distance_outside_measures_to_nearest_free_cell():
    m = make_mask()
    d = m.distance_outside(1.0, 5.0, 4.0)
    assert d == pytest.approx(1.05, abs=0.1)
    d = m.distance_outside(-1.0, 5.0, 4.0)             # off the grid entirely
    assert d == pytest.approx(3.05, abs=0.1)
    d = m.distance_outside(0.0, 0.0, 4.0)              # corner: diagonal
    assert d == pytest.approx(math.hypot(2.05, 2.05), abs=0.1)


def test_distance_outside_inf_beyond_max_r():
    m = make_mask()
    assert m.distance_outside(-10.0, 5.0, 4.0) == math.inf
    assert m.distance_outside(0.0, 0.0, 2.0) == math.inf     # 2.9 m away, max_r 2


def test_distance_outside_inf_for_mask_without_free_cells():
    m = r.MapMask(0, 0, 0.1, 10, 10, [100] * 100)
    assert not m.has_free
    assert m.distance_outside(0.5, 0.5, 4.0) == math.inf
    assert r.MapMask(0, 0, 0.0, 2, 2, [0] * 4).distance_outside(0, 0, 1) == math.inf


def test_pose_far_outside_map_fires_after_debounce():
    e = armed_engine()
    e.set_mask(make_mask())
    alerts = run(e, parked(pose=(-3.0, 5.0)), 0.0, 90.0)
    assert msgs(alerts) == ['theftOutsideMap']
    t, a = alerts[0]
    assert t == 70.0
    assert a.kind == 'theft' and a.priority == 5


def test_pose_within_margin_outside_does_not_fire():
    e = armed_engine()
    e.set_mask(make_mask())
    assert run(e, parked(pose=(1.0, 5.0)), 0.0, 120.0) == []


def test_pose_inside_map_does_not_fire():
    e = armed_engine()
    e.set_mask(make_mask())
    assert run(e, parked(pose=(5.0, 5.0)), 0.0, 120.0) == []


def test_outside_map_ignored_during_mission_and_without_mask():
    e = armed_engine()
    e.set_mask(make_mask())
    assert run(e, mission(pose=(-3.0, 5.0)), 0.0, 120.0) == []
    e = armed_engine()
    assert run(e, parked(pose=(-3.0, 5.0)), 0.0, 120.0) == []


# ---------------------------------------------------------------------------
# 6. lift / tilt while mowing
# ---------------------------------------------------------------------------
def test_lift_during_mission_fires_without_emergency_alert():
    e = armed_engine()
    alerts = run(e, mission(lift=True, hl_emergency=True), 0.0, 10.0)
    assert msgs(alerts) == ['liftDuringMow']
    t, a = alerts[0]
    assert t == 0.5
    assert (a.kind, a.priority, a.incident) == ('lift', 5, 'lift_during_mow')


def test_lift_during_manual_driving_fires():
    e = armed_engine()
    i = r.Inputs(state=r.HL_MANUAL_MOWING, state_name='MANUAL_MOWING', lift=True,
                 hl_emergency=True)
    assert msgs(run(e, i, 0.0, 10.0)) == ['liftDuringMow']


def test_lift_blip_shorter_than_debounce_does_not_fire():
    e = armed_engine()
    assert run(e, mission(lift=True), 0.0, 0.5) == []
    assert run(e, mission(lift=False), 0.5, 10.0) == []


def test_tilt_over_limit_fires_after_debounce():
    e = armed_engine()
    alerts = run(e, mission(tilt_deg=(40.0, 0.0)), 0.0, 5.0)
    assert msgs(alerts) == ['tiltDuringMow']
    t, a = alerts[0]
    assert t == 1.0
    assert a.kind == 'lift' and a.priority == 5
    assert a.params['detail'] == '40°'


def test_moderate_tilt_does_not_fire():
    e = armed_engine()
    assert run(e, mission(tilt_deg=(20.0, 10.0)), 0.0, 30.0) == []


def test_tilt_ignored_while_parked():
    e = armed_engine()
    assert run(e, parked(tilt_deg=(60.0, 0.0)), 0.0, 30.0) == []


def test_lift_and_tilt_together_send_one_lift_group_push():
    e = armed_engine()
    alerts = run(e, mission(lift=True, tilt_deg=(50.0, 0.0)), 0.0, 10.0)
    assert msgs(alerts) == ['liftDuringMow']
    assert {'lift_during_mow', 'tilt_during_mow'} <= set(e.active_incidents())


def test_tilt_from_quaternion_identity_and_roll():
    roll, pitch = r.tilt_from_quaternion(0.0, 0.0, 0.0, 1.0)
    assert (roll, pitch) == (pytest.approx(0.0), pytest.approx(0.0))
    s = math.sqrt(0.5)
    roll, pitch = r.tilt_from_quaternion(s, 0.0, 0.0, s)
    assert roll == pytest.approx(90.0)
    assert pitch == pytest.approx(0.0, abs=1e-6)


def test_tilt_from_quaternion_pitch_and_unnormalised():
    s = math.sin(math.radians(15.0))
    c = math.cos(math.radians(15.0))
    roll, pitch = r.tilt_from_quaternion(0.0, 2 * s, 0.0, 2 * c)   # scaled by 2
    assert pitch == pytest.approx(30.0)
    assert roll == pytest.approx(0.0, abs=1e-6)


def test_tilt_from_quaternion_zero_is_none():
    assert r.tilt_from_quaternion(0.0, 0.0, 0.0, 0.0) is None


def test_tilt_angle():
    assert r.tilt_angle((30.0, 0.0)) == pytest.approx(30.0)
    assert r.tilt_angle((0.0, 0.0)) == pytest.approx(0.0)
    assert r.tilt_angle((0.0, -25.0)) == pytest.approx(25.0)
    assert r.tilt_angle((90.0, 0.0)) == pytest.approx(90.0)


# ---------------------------------------------------------------------------
# 7. mission incidents
# ---------------------------------------------------------------------------
def test_stuck_alert_carries_sub_state_detail():
    e = armed_engine()
    i = r.Inputs(state=r.HL_AUTONOMOUS, state_name=r.STATE_STUCK,
                 sub_state_name='escape failed 3/3', battery_percent=55.0)
    alerts = run(e, i, 0.0, 5.0)
    assert len(alerts) == 1
    a = alerts[0][1]
    assert (a.kind, a.message, a.priority) == ('blocked', 'stuck', 4)
    assert a.params['detail'] == 'escape failed 3/3'
    assert a.params['state'] == r.STATE_STUCK
    assert a.params['battery'] == '55 %'


INCIDENT_STATES = [
    (r.STATE_STUCK, 'stuck', 'blocked'),
    (r.STATE_INCOMPLETE, 'mowIncomplete', 'blocked'),
    (r.STATE_COVERAGE_FAILED, 'mowIncomplete', 'blocked'),
    (r.STATE_DOCK_FAILED, 'dockFailed', 'blocked'),
    (r.STATE_UNDOCK_FAILED, 'undockFailed', 'blocked'),
    (r.STATE_BOUNDARY_ESTOP, 'emergencyStop', 'emergency'),
]


@pytest.mark.parametrize('name,message,kind', INCIDENT_STATES)
def test_state_incident_fires_once_per_entry_and_again_after_clear(name, message, kind):
    e = armed_engine()
    inside = r.Inputs(state=r.HL_NULL, state_name=name, sub_state_name='why')
    outside = r.Inputs(state=r.HL_NULL, state_name='IDLE')
    first = run(e, inside, 0.0, 20.0)
    assert msgs(first) == [message]
    assert first[0][1].kind == kind
    assert run(e, outside, 20.0, 20.0) == []           # 20 s clear < clear_s 30
    assert run(e, inside, 40.0, 20.0) == []            # re-entered too soon: still deduped
    assert run(e, outside, 60.0, 35.0) == []           # >= clear_s: re-armed
    again = run(e, inside, 95.0, 20.0)
    assert msgs(again) == [message]


def test_boundary_estop_cause_mentions_mapped_area():
    e = armed_engine()
    i = r.Inputs(state=r.HL_NULL, state_name=r.STATE_BOUNDARY_ESTOP, hl_emergency=True)
    alerts = run(e, i, 0.0, 5.0)
    assert msgs(alerts) == ['emergencyStop']
    assert 'mapped area' in alerts[0][1].params['cause']


def test_detail_falls_back_to_state_name_for_incomplete():
    e = armed_engine()
    alerts = run(e, r.Inputs(state_name=r.STATE_INCOMPLETE), 0.0, 2.0)
    assert alerts[0][1].params['detail'] == r.STATE_INCOMPLETE.lower()


def blocked(sub='path blocked: waiting (12 s)'):
    return mission('TRANSIT', sub_state_name=sub, rtk_fix_type=3)


def test_path_blocked_fires_after_alert_s():
    e = armed_engine()
    alerts = run(e, blocked(), 0.0, 60.0)
    assert msgs(alerts) == ['pathBlocked']
    t, a = alerts[0]
    assert t == 45.0
    assert (a.kind, a.priority) == ('blocked', 3)
    assert a.params['minutes'] == '1 min'


def test_path_blocked_cleared_after_30s_is_silent():
    e = armed_engine()
    assert run(e, blocked(), 0.0, 30.0) == []
    assert run(e, mission('TRANSIT', rtk_fix_type=3), 30.0, 120.0) == []


def test_path_blocked_second_episode_within_min_interval_is_suppressed():
    e = armed_engine()
    assert msgs(run(e, blocked(), 0.0, 60.0)) == ['pathBlocked']
    assert run(e, mission('TRANSIT', rtk_fix_type=3), 60.0, 40.0) == []          # clear (>= 30 s)
    assert run(e, blocked(), 100.0, 100.0) == []                 # < 600 s after the first
    assert run(e, mission('TRANSIT', rtk_fix_type=3), 200.0, 40.0) == []
    late = run(e, blocked(), 700.0, 60.0)
    assert msgs(late) == ['pathBlocked']                         # beyond the interval


def test_path_blocked_ignored_outside_mission():
    e = armed_engine()
    i = r.Inputs(state=r.HL_RECORDING, state_name='RECORDING',
                 sub_state_name='path blocked: x')
    assert run(e, i, 0.0, 120.0) == []


# ---------------------------------------------------------------------------
# 8. RTK lost
# ---------------------------------------------------------------------------
def test_rtk_float_for_five_minutes_fires_rtk_lost():
    e = armed_engine()
    assert run(e, mission('MOWING', rtk_fix_type=2), 0.0, 299.5) == []
    alerts = run(e, mission('MOWING', rtk_fix_type=2), 299.5, 10.0)
    assert msgs(alerts) == ['rtkLost']
    t, a = alerts[0]
    assert t == 300.0
    assert a.kind == 'gpsLost'
    assert a.params['detail'] == 'RTK float'
    assert a.params['minutes'] == '5 min'


def test_rtk_recovered_after_fix_returns():
    e = armed_engine()
    run(e, mission('MOWING', rtk_fix_type=2), 0.0, 310.0)
    assert run(e, mission('MOWING', rtk_fix_type=3), 310.0, 9.5) == []
    alerts = run(e, mission('MOWING', rtk_fix_type=3), 319.5, 5.0)
    assert msgs(alerts) == ['rtkRecovered']
    assert alerts[0][0] == 320.0
    assert alerts[0][1].kind == 'gpsLost'


def test_rtk_short_dropout_never_fires():
    e = armed_engine()
    assert run(e, mission('MOWING', rtk_fix_type=2), 0.0, 200.0) == []
    assert run(e, mission('MOWING', rtk_fix_type=3), 200.0, 200.0) == []


def test_no_rtk_recovered_if_mission_ends_while_still_float():
    e = armed_engine()
    run(e, mission('MOWING', rtk_fix_type=2), 0.0, 310.0)
    ended = parked(rtk_fix_type=2)
    assert run(e, ended, 310.0, 60.0) == []
    assert 'rtk_lost' not in e.active_incidents()


def test_waiting_for_rtk_phase_does_not_trigger_rtk_lost():
    e = armed_engine()
    assert run(e, mission('WAITING_FOR_RTK', rtk_fix_type=0), 0.0, 600.0) == []


def test_missing_gnss_status_in_watched_phase_counts_as_lost():
    e = armed_engine()
    alerts = run(e, mission('MOWING', rtk_fix_type=None), 0.0, 310.0)
    assert msgs(alerts) == ['rtkLost']
    assert alerts[0][1].params['detail'] == 'no GNSS status'


# ---------------------------------------------------------------------------
# 9. battery critical
# ---------------------------------------------------------------------------
def test_battery_critical_off_dock_fires_after_debounce():
    e = armed_engine()
    i = r.Inputs(state=r.HL_AUTONOMOUS, state_name='MOWING', battery_percent=8.0,
                 docked=False, charging=False)
    assert run(e, i, 0.0, 29.5) == []
    alerts = run(e, i, 29.5, 10.0)
    assert msgs(alerts) == ['batteryCritical']
    t, a = alerts[0]
    assert t == 30.0
    assert (a.kind, a.priority) == ('battery', 4)
    assert a.params['battery'] == '8 %'


@pytest.mark.parametrize('kw', [{'docked': True}, {'charging': True}, {'hl_charging': True}])
def test_battery_critical_ignored_on_dock_or_charging(kw):
    e = armed_engine()
    i = r.Inputs(state=r.HL_AUTONOMOUS, state_name='MOWING', battery_percent=8.0, **kw)
    assert run(e, i, 0.0, 120.0) == []


@pytest.mark.parametrize('pct', [0.0, None, 50.0, 11.0])
def test_battery_unknown_or_ok_is_silent(pct):
    e = armed_engine()
    i = r.Inputs(state=r.HL_AUTONOMOUS, state_name='MOWING', battery_percent=pct)
    assert run(e, i, 0.0, 120.0) == []


def test_battery_at_threshold_fires():
    e = armed_engine()
    i = r.Inputs(state=r.HL_AUTONOMOUS, state_name='MOWING', battery_percent=10.0)
    assert msgs(run(e, i, 0.0, 40.0)) == ['batteryCritical']


# ---------------------------------------------------------------------------
# 10. rate limit
# ---------------------------------------------------------------------------
def pulse(e, name, t):
    return e.step(r.Inputs(state=r.HL_NULL, state_name=name), t)


def test_rate_limit_drops_extra_alerts_and_reports_suppressed_count():
    e = armed_engine(rate_limit_count=2, rate_limit_window_s=600.0)
    assert [a.message for a in pulse(e, r.STATE_STUCK, 0.0)] == ['stuck']
    assert [a.message for a in pulse(e, r.STATE_DOCK_FAILED, 10.0)] == ['dockFailed']
    assert pulse(e, r.STATE_UNDOCK_FAILED, 20.0) == []
    assert pulse(e, r.STATE_INCOMPLETE, 30.0) == []
    assert e.suppressed == 2
    # leave all states so every incident re-arms, then wait out the window
    run(e, r.Inputs(state=r.HL_NULL, state_name='IDLE'), 40.0, 100.0)
    out = pulse(e, r.STATE_STUCK, 700.0)
    assert [a.message for a in out] == ['stuck']
    assert out[0].params['suppressed'] == '2'
    assert e.suppressed == 0
    run(e, r.Inputs(state=r.HL_NULL, state_name='IDLE'), 701.0, 100.0)
    again = pulse(e, r.STATE_DOCK_FAILED, 900.0)
    assert 'suppressed' not in again[0].params


def test_priority_five_is_never_rate_limited():
    e = armed_engine(rate_limit_count=1, rate_limit_window_s=600.0)
    assert len(pulse(e, r.STATE_STUCK, 0.0)) == 1
    assert pulse(e, r.STATE_DOCK_FAILED, 10.0) == []      # limiter is now full
    alerts = run(e, r.Inputs(hl_emergency=True, stop_button=True), 20.0, 5.0)
    assert msgs(alerts) == ['emergencyStop']
    lifted = run(e, mission(lift=True), 30.0, 5.0)
    assert 'liftDuringMow' in msgs(lifted)


def test_priority_five_alerts_count_toward_window_but_do_not_block_others_when_below():
    e = armed_engine(rate_limit_count=2, rate_limit_window_s=600.0)
    run(e, r.Inputs(hl_emergency=True, stop_button=True), 0.0, 3.0)      # 1 sent (emergency)
    assert len(pulse(e, r.STATE_STUCK, 10.0)) == 1                        # 2nd slot
    assert pulse(e, r.STATE_DOCK_FAILED, 20.0) == []                      # full


# ---------------------------------------------------------------------------
# 11. disabled
# ---------------------------------------------------------------------------
def test_disabled_engine_never_alerts():
    e = r.AlertEngine(r.Params(enabled=False))
    everything = parked(lift=True, hl_emergency=True, stop_button=True, battery_percent=5.0,
                        fix=fix_at(DLAT_33M), tilt_deg=(80.0, 0.0),
                        state_name=r.STATE_STUCK)
    assert run(e, everything, 0.0, 400.0) == []
    assert e.mode == r.MODE_UNKNOWN


# ---------------------------------------------------------------------------
# 12. numeric helpers
# ---------------------------------------------------------------------------
def test_accuracy_from_covariance():
    cov = [0.04, 0, 0, 0, 0.09, 0, 0, 0, 0.5]
    assert r.accuracy_from_covariance(cov, 2) == pytest.approx(0.3)
    assert r.accuracy_from_covariance(cov, 0) == math.inf
    assert r.accuracy_from_covariance(None, 2) == math.inf
    assert r.accuracy_from_covariance([0.0] * 9, 2) == math.inf
    assert r.accuracy_from_covariance([0.04], 2) == math.inf


def test_accuracy_uses_larger_axis():
    cov = [0.25, 0, 0, 0, 0.01, 0, 0, 0, 0]
    assert r.accuracy_from_covariance(cov, 1) == pytest.approx(0.5)


def test_distance_m():
    assert r.distance_m(LAT0, LON0, LAT0, LON0) == 0.0
    assert r.distance_m(LAT0, LON0, LAT0 + 0.0003, LON0) == pytest.approx(33.4, abs=0.2)
    # symmetric, and a degree of longitude shrinks with latitude
    assert r.distance_m(LAT0 + 0.0003, LON0, LAT0, LON0) == \
        pytest.approx(r.distance_m(LAT0, LON0, LAT0 + 0.0003, LON0))
    assert r.distance_m(0.0, 0.0, 0.0, 0.001) > r.distance_m(60.0, 0.0, 60.0, 0.001)


def test_fix_label_and_minutes_helpers():
    assert r._fix_label(2) == 'RTK float'
    assert r._fix_label(None) == 'no GNSS status'
    assert r._fix_label(9) == 'fix type 9'
    assert r._minutes(10) == '1 min'
    assert r._minutes(300) == '5 min'
