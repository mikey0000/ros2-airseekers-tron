"""Unit tests for the /diagnostics status builder (no ROS)."""

import os
import sys

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from mower_gui_bridge import diagnostics as d  # noqa: E402

FIX_STATUS = ('um960=connected quality=RTK_FIXED sats=27/27 hdop=0.40 diff_age=2.0s '
              'corr_src=ntrip corr=streaming corr_flow=active corr_age=1.2')


def motor(rpm=0, current=0, voltage=1991, temp=29, status=0):
    return {'speed_rpm': rpm, 'current': current, 'voltage': voltage, 'temperature': temp,
            'status': status}


def snapshot(**over):
    snap = {
        'base_age': 0.1, 'sensor_age': 0.1, 'battery_age': 0.2,
        'sensor': {'cutter_board_version': 'v0.6.36', 'chassis_board_version': 'v0.6.34',
                   'rtk_board_version': '0.0.0', 'mower_package_version': '',
                   'battery_error': 0, 'is_docking_done': False,
                   'cutter_motor': motor(2800, 120), 'left_motor': motor(),
                   'right_motor': motor()},
        'battery': {'voltage': 20.0, 'current': 17.0, 'percentage': 0.34,
                    'temperature': 30.0, 'charging': False},
        'fix': {'status': 2, 'lat': -38.0, 'lon': 175.3, 'alt': 49.9}, 'fix_age': 0.2,
        'fix_type': 3, 'fix_status': FIX_STATUS, 'fix_status_age': 0.2,
        'filtered_age': 0.05, 'datum': (-38.0, 175.3),
        'imu_age': 0.01, 'imu_temperature': 42.0,
        'imu_bias': 'state=CALIBRATED gyro_bias_radps=[0,0,0]',
        'lights': '{"mode": 1, "mode_name": "Idle", "buffer_hex": "0f0f"}', 'lights_age': 0.5,
        'cameras': [],
    }
    snap.update(over)
    return snap


def by_name(statuses):
    return {s.name: s for s in statuses}


def test_all_ok_on_healthy_snapshot():
    out = by_name(d.build_statuses(snapshot()))
    assert set(out) >= {'tron: MCU link', 'tron: Battery', 'tron: Cutter motor',
                        'tron: Left drive motor', 'tron: Right drive motor', 'tron: GNSS',
                        'tron: Localization', 'IMU', 'tron: Lights'}
    assert all(s.level == d.OK for s in out.values()), \
        [(s.name, s.message) for s in out.values() if s.level]


def test_battery_values_and_percent_scaling():
    st = d.battery_status(snapshot()['battery'], 0.1, snapshot()['sensor'], 3.0)
    vals = dict(st.values)
    assert vals['Charge (%)'] == '34' and vals['Voltage (V)'] == '20.00'
    assert vals['Temperature (C)'] == '30' and vals['Error bits'] == '0x0'


def test_battery_error_bits_and_stale():
    sensor = dict(snapshot()['sensor'], battery_error=0x24)
    assert d.battery_status(snapshot()['battery'], 0.1, sensor, 3.0).level == d.ERROR
    assert d.battery_status(None, None, sensor, 3.0).level == d.STALE
    assert d.battery_status(snapshot()['battery'], 10.0, sensor, 3.0).level == d.STALE


def test_motor_units_and_status_alert_switch():
    st = d.motor_status('Cutter', motor(2800, 120, 1991, 29), 0.1, 3.0, False)
    vals = dict(st.values)
    assert vals['Speed (rpm)'] == '2800' and vals['Current (A)'] == '1.20'
    assert vals['Voltage (V)'] == '19.91' and vals['Temperature (C)'] == '29'
    # 1 = running (drive boards report it whenever enabled): never an alert
    running = motor(status=1)
    assert d.motor_status('Left drive', running, 0.1, 3.0, True).level == d.OK
    assert 'running' in d.motor_status('Left drive', running, 0.1, 3.0, True).message
    bad = motor(status=-5)
    assert d.motor_status('Left drive', bad, 0.1, 3.0, False).level == d.OK
    assert d.motor_status('Left drive', bad, 0.1, 3.0, True).level == d.WARN
    assert 'over-temperature' in d.motor_status('Left drive', bad, 0.1, 3.0, True).message
    assert d.motor_status('Cutter', None, None, 3.0, False).level == d.STALE


def test_mcu_link_lost_is_error_with_versions():
    st = d.mcu_status(5.0, snapshot()['sensor'], 5.0, 3.0)
    assert st.level == d.ERROR
    assert dict(st.values)['Cutter board firmware'] == 'v0.6.36'


def test_gnss_levels_follow_fix_type():
    s = snapshot()
    assert d.gnss_status(s['fix'], 0.2, 3, FIX_STATUS, 0.2, 3.0).level == d.OK
    assert d.gnss_status(s['fix'], 0.2, 2, FIX_STATUS, 0.2, 3.0).level == d.WARN
    assert d.gnss_status(s['fix'], 0.2, 0, FIX_STATUS, 0.2, 3.0).level == d.ERROR
    assert d.gnss_status(None, None, None, None, None, 3.0).level == d.STALE
    vals = dict(d.gnss_status(s['fix'], 0.2, 3, FIX_STATUS, 0.2, 3.0).values)
    assert vals['Satellites'] == '27/27' and vals['Correction source'] == 'ntrip'
    assert vals['Correction age'] == '1.2'


def test_localization_needs_fresh_ekf_and_datum():
    assert d.localization_status(0.1, (-38.0, 175.3), 3.0).level == d.OK
    assert d.localization_status(0.1, None, 3.0).level == d.WARN
    assert d.localization_status(0.1, (0.0, 0.0), 3.0).level == d.WARN
    assert d.localization_status(None, (-38.0, 175.3), 3.0).level == d.ERROR


def test_imu_status_is_named_for_the_gui():
    st = d.imu_status(0.01, 42.0, 'state=FAILED reason=timeout', 3.0)
    assert st.name == 'IMU' and st.level == d.OK
    assert dict(st.values)['Bias calibration'] == 'FAILED'
    assert d.imu_status(None, None, None, 3.0).level == d.STALE
    assert d.imu_status(9.0, None, None, 3.0).level == d.ERROR


def test_camera_spec_parsing():
    cam = d.parse_camera_spec('left|/left_oa_camera/image_raw|/left_oa_camera/camera_info|0')
    assert cam == {'id': 'left', 'topic': '/left_oa_camera/image_raw',
                   'freshness_topic': '/left_oa_camera/camera_info', 'on_demand': False}
    assert d.parse_camera_spec('rear|/rear_camera/image_raw||1')['on_demand'] is True
    assert d.parse_camera_spec('rear') is None and d.parse_camera_spec('|/x') is None


def test_camera_status_matches_gui_freshness_contract():
    always = d.parse_camera_spec('left|/left_oa_camera/image_raw|/left_oa_camera/camera_info')
    on_demand = d.parse_camera_spec('rear|/rear_camera/image_raw||1')
    st = d.camera_status(always, 0.1, 1, 1, 3.0)
    # cameraFreshness.ts: name contains the topic or hardware_id equals the camera id
    assert '/left_oa_camera/image_raw' in st.name and st.hardware_id == 'left'
    assert st.level == d.OK
    assert d.camera_status(always, 9.0, 1, 1, 3.0).level == d.ERROR
    assert d.camera_status(always, 0.1, 0, 1, 3.0).level == d.ERROR
    assert d.camera_status(on_demand, None, 1, 0, 3.0).message.startswith('idle')
    assert d.camera_status(on_demand, None, 1, 2, 3.0).level == d.OK
    assert d.camera_status(on_demand, None, 0, 0, 3.0).level == d.ERROR


def test_lights_status_parses_json_and_drops_buffer():
    st = d.lights_status('{"mode": 1, "mode_name": "Idle", "buffer_hex": "0f"}', 0.5, 3.0)
    assert st.message == 'Idle' and 'Buffer_hex' not in dict(st.values)
    assert d.lights_status('not json', 0.5, 3.0).level == d.WARN
    assert d.lights_status(None, None, 3.0).level == d.STALE


def test_optional_entries_only_when_configured():
    snap = snapshot()
    for key in ('imu_age', 'lights'):
        snap.pop(key)
    names = {s.name for s in d.build_statuses(snap)}
    assert 'IMU' not in names and 'tron: Lights' not in names


def test_key_values():
    assert d.parse_key_values('a=1 bare b=x=y') == {'a': '1', 'b': 'x=y'}
    assert d.parse_key_values(None) == {}


def test_heading_status_entry():
    ok = d.heading_status('{"aligned": true, "source": "cog", "offset_deg": -150.2, '
                          '"quality": "good", "yaw_sigma_deg": 3.1, "cog_updates": 4, '
                          '"heading_deg": 87.0, "event": "COG update"}')
    assert ok.level == d.OK and ok.name == 'tron: Heading' and 'cog' in ok.message
    vals = dict(ok.values)
    assert vals['Offset (deg)'] == '-150.2' and vals['Source'] == 'cog'
    warn = d.heading_status('{"aligned": false, "source": "file", "offset_deg": 10.0}')
    assert warn.level == d.WARN and 'drive straight 1 m' in warn.message
    assert d.heading_status(None).level == d.STALE
    assert d.heading_status('nope').level == d.WARN
    names = by_name(d.build_statuses(snapshot(heading='{"aligned": false, "source": "none"}')))
    assert names['tron: Heading'].level == d.WARN
    assert 'tron: Heading' not in by_name(d.build_statuses(snapshot()))
