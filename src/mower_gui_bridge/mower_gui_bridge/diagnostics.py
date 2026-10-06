"""Hardware diagnostics for the GUI's ``/diagnostics`` feed (pure Python, no ROS).

``gui_bridge`` already caches everything the drivers report (MCU sensor info, battery,
GNSS status string, ...). :func:`build_statuses` turns one snapshot of those caches into
``diagnostic_msgs/DiagnosticStatus``-shaped tuples that the node publishes as one
``DiagnosticArray`` at 1 Hz. The MowgliNext GUI renders every entry in its "ROS
Diagnostics" card, raises level >= WARN entries as alerts, and matches its profile
cameras by topic (in ``name``) or id (``hardware_id``) for the robot anatomy freshness.

Each status is ``Status(level, name, message, hardware_id, values)`` where ``values``
is a list of ``(key, str)``. Levels follow diagnostic_msgs: OK 0, WARN 1, ERROR 2,
STALE 3. Inputs that never arrived produce STALE, not a guessed value.
"""

from collections import namedtuple
import json
import math

OK, WARN, ERROR, STALE = 0, 1, 2, 3

Status = namedtuple('Status', 'level name message hardware_id values')

# Name prefix of every entry (the GUI groups alerts by the text before ':').
PREFIX = 'tron'

# MotorStatus.status is a signed int8 (vendor ROS 1 MotorStatus.msg, see
# docs/mcu_protocol_spec.md): 0 idle, 1 running, 2 locking; negative codes are faults.
# The drive boards report 1 whenever enabled, even at standstill.
MOTOR_STATUS_TEXT = {0: 'idle', 1: 'running', 2: 'locking', -1: 'error', -2: 'over-current',
                     -3: 'over-voltage', -4: 'under-voltage', -5: 'over-temperature',
                     -6: 'stalled', -7: 'overload'}


def _fmt(value, spec='%.2f'):
    if value is None:
        return '-'
    if isinstance(value, float) and not math.isfinite(value):
        return '-'
    return spec % value


def _age_text(age):
    return '-' if age is None else '%.1f s' % age


def _fresh(age, timeout):
    return age is not None and age <= timeout


def _status(level, name, message, hardware_id='', values=()):
    return Status(int(level), '%s: %s' % (PREFIX, name), str(message), str(hardware_id),
                  [(str(k), str(v)) for k, v in values])


def parse_camera_spec(spec):
    """``'id|image_topic|freshness_topic|on_demand'`` -> dict (None if malformed).

    ``freshness_topic`` is a cheap topic published with every frame (``camera_info``); empty
    means the image topic itself is not subscribed and only the publisher is checked.
    ``on_demand`` (``1``/``true``) marks a driver that only captures while someone watches,
    so "no frames and no viewer" is idle, not a fault.
    """
    parts = [p.strip() for p in str(spec).split('|')]
    if len(parts) < 2 or not parts[0] or not parts[1]:
        return None
    parts += [''] * (4 - len(parts))
    return {'id': parts[0], 'topic': parts[1], 'freshness_topic': parts[2],
            'on_demand': parts[3].lower() in ('1', 'true', 'yes', 'on_demand')}


def parse_key_values(text):
    """``'a=1 b=two c=x y'`` -> ``{'a': '1', 'b': 'two', 'c': 'x'}`` (bare words dropped)."""
    out = {}
    for token in str(text or '').split():
        key, sep, value = token.partition('=')
        if sep and key:
            out[key] = value
    return out


def battery_status(battery, battery_age, sensor, timeout):
    """battery: dict(voltage, current, percentage, temperature, charging) or None."""
    if battery is None or not _fresh(battery_age, timeout):
        return _status(STALE, 'Battery', 'no /battery data (age %s)' % _age_text(battery_age),
                       'battery')
    pct = battery.get('percentage')
    if pct is not None and math.isfinite(pct) and pct <= 1.0:
        pct *= 100.0
    error_bits = int(sensor.get('battery_error', 0)) if sensor else None
    level, message = OK, 'charging' if battery.get('charging') else 'discharging'
    if error_bits:
        level, message = ERROR, 'battery error bits 0x%x' % error_bits
    elif pct is not None and math.isfinite(pct) and pct < 15.0:
        level, message = WARN, 'battery low (%.0f %%)' % pct
    values = [('Voltage (V)', _fmt(battery.get('voltage'))),
              ('Current (raw)', _fmt(battery.get('current'), '%.1f')),
              ('Charge (%)', _fmt(pct, '%.0f')),
              ('Temperature (C)', _fmt(battery.get('temperature'), '%.0f')),
              ('Charging', str(bool(battery.get('charging')))),
              ('Error bits', '-' if error_bits is None else '0x%x' % error_bits)]
    if sensor:
        values.append(('Docked (contacts)', str(bool(sensor.get('is_docking_done')))))
    return _status(level, 'Battery', message, 'battery', values)


def motor_status(name, motor, sensor_age, timeout, alert_on_status):
    """motor: dict(speed_rpm, current, voltage, temperature, status) from MowerSensorInfo."""
    hw = '%s_motor' % name.lower().replace(' ', '_')
    if motor is None or not _fresh(sensor_age, timeout):
        return _status(STALE, '%s motor' % name, 'no MCU motor data (age %s)'
                       % _age_text(sensor_age), hw)
    code = int(motor.get('status', 0))
    text = MOTOR_STATUS_TEXT.get(code, 'code %d' % code)
    fault = code < 0
    level = WARN if fault and alert_on_status else OK
    if not fault:
        message = 'ok (%s)' % text
    elif alert_on_status:
        message = 'fault: %s' % text
    else:   # fault codes not raised as alerts until verified on Tron (README)
        message = 'ok (MCU reports fault code %d = %s, alerts disabled)' % (code, text)
    values = [('Speed (rpm)', '%d' % int(motor.get('speed_rpm', 0))),
              ('Current (A)', _fmt(motor.get('current', 0) * 0.01)),
              ('Voltage (V)', _fmt(motor.get('voltage', 0) * 0.01)),
              ('Temperature (C)', '%d' % int(motor.get('temperature', 0))),
              ('Status code', '%d (%s)' % (code, text))]
    return _status(level, '%s motor' % name, message, hw, values)


def mcu_status(base_age, sensor, sensor_age, timeout):
    alive = _fresh(base_age, timeout)
    level = OK if alive else ERROR
    message = 'connected' if alive else 'no /mower_base/status for %s' % _age_text(base_age)
    values = [('Status age', _age_text(base_age)),
              ('Sensor info age', _age_text(sensor_age))]
    if sensor:
        for key, label in (('cutter_board_version', 'Cutter board firmware'),
                           ('chassis_board_version', 'Chassis board firmware'),
                           ('rtk_board_version', 'RTK board firmware'),
                           ('mower_package_version', 'Package version')):
            values.append((label, sensor.get(key) or '-'))
        for key, label in (('bumper_triggered', 'Bumper'), ('lift_triggered', 'Lift'),
                           ('stop_triggered', 'Stop button'), ('rain_triggered', 'Rain'),
                           ('is_charging', 'Charging'), ('is_cutting', 'Cutting')):
            values.append((label, str(bool(sensor.get(key)))))
    return _status(level, 'MCU link', message, 'mcu', values)


FIX_TYPE_TEXT = {0: 'no fix', 1: 'GPS fix', 2: 'RTK float', 3: 'RTK fixed',
                 4: 'dead reckoning'}


def gnss_status(fix, fix_age, fix_type, fix_status_text, fix_status_age, timeout):
    """fix: dict(status, lat, lon, alt) from /fix; fix_type: GnssStatus.fix_type derived
    from it (state_machine.derive_gnss_status); fix_status_text: the driver's string."""
    if fix is None or not _fresh(fix_age, timeout):
        return _status(ERROR if fix is not None else STALE, 'GNSS',
                       'no /fix for %s' % _age_text(fix_age), 'gnss')
    kv = parse_key_values(fix_status_text) if _fresh(fix_status_age, timeout) else {}
    label = FIX_TYPE_TEXT.get(fix_type, 'unknown')
    level = OK if fix_type == 3 else ERROR if fix_type in (0, None) else WARN
    values = [('Fix', label),
              ('Receiver solution', kv.get('quality') or kv.get('solution') or '-'),
              ('Satellites', kv.get('sats', '-')), ('HDOP', kv.get('hdop', '-')),
              ('Correction source', kv.get('corr_src', '-')),
              ('Correction link', kv.get('corr', '-')),
              ('Correction flow', kv.get('corr_flow', '-')),
              ('Correction age', kv.get('corr_age', kv.get('diff_age', '-'))),
              ('Receiver', kv.get('um960', '-')),
              ('Latitude', _fmt(fix.get('lat'), '%.8f')),
              ('Longitude', _fmt(fix.get('lon'), '%.8f')),
              ('Altitude (m)', _fmt(fix.get('alt'), '%.2f')),
              ('Fix age', _age_text(fix_age))]
    return _status(level, 'GNSS', label, 'gnss', values)


def localization_status(filtered_age, datum, timeout):
    """robot_localization output freshness + the datum navsat_transform is pinned to."""
    lat, lon = datum if datum else (None, None)
    has_datum = lat is not None and lon is not None and not (lat == 0.0 and lon == 0.0)
    if not _fresh(filtered_age, timeout):
        level, message = ERROR, 'no /odometry/filtered for %s' % _age_text(filtered_age)
    elif not has_datum:
        level, message = WARN, 'running, no datum set (map origin follows the first fix)'
    else:
        level, message = OK, 'EKF running, datum set'
    values = [('Filtered odometry age', _age_text(filtered_age)),
              ('Datum latitude', _fmt(lat, '%.9f') if has_datum else '-'),
              ('Datum longitude', _fmt(lon, '%.9f') if has_datum else '-')]
    return _status(level, 'Localization', message, 'localization', values)


def imu_status(imu_age, temperature, bias_text, timeout):
    """Status named exactly ``IMU`` too (see :func:`build_statuses`): the GUI reads it."""
    fresh = _fresh(imu_age, timeout)
    values = [('Data age', _age_text(imu_age)),
              ('Temperature (C)', _fmt(temperature, '%.1f'))]
    kv = parse_key_values(bias_text)
    if kv:
        values.append(('Bias calibration', kv.get('state', '-')))
        for key in ('gyro_bias_radps', 'reason', 'samples'):
            if key in kv:
                values.append(('Calibration ' + key, kv[key]))
    else:
        values.append(('Bias calibration', 'no report'))
    if not fresh:
        return Status(STALE if imu_age is None else ERROR, 'IMU',
                      'no /imu/data for %s' % _age_text(imu_age), 'imu', values)
    return Status(OK, 'IMU', 'receiving', 'imu', values)


def camera_status(cam, freshness_age, publishers, viewers, timeout):
    """cam: :func:`parse_camera_spec` dict. Named ``<topic> topic status`` for the GUI."""
    name = '%s topic status' % cam['topic']
    values = [('Topic', cam['topic']), ('Publishers', str(publishers)),
              ('Subscribers', str(viewers))]
    if cam['freshness_topic']:
        values.append(('Frame age (%s)' % cam['freshness_topic'], _age_text(freshness_age)))
    if publishers == 0:
        return Status(ERROR, name, 'no publisher (camera driver not running)', cam['id'],
                      values)
    if cam['freshness_topic'] and not cam['on_demand']:
        if _fresh(freshness_age, timeout):
            return Status(OK, name, 'streaming', cam['id'], values)
        return Status(ERROR, name, 'no frames for %s' % _age_text(freshness_age), cam['id'],
                      values)
    if cam['on_demand'] and viewers == 0:
        return Status(OK, name, 'idle (captures on demand)', cam['id'], values)
    return Status(OK, name, 'publisher up', cam['id'], values)


def lights_status(state_text, state_age, timeout):
    if state_text is None or not _fresh(state_age, timeout):
        return _status(STALE, 'Lights', 'no /light_controller/state', 'lights')
    try:
        state = json.loads(state_text)
    except ValueError:
        return _status(WARN, 'Lights', 'unparsable state', 'lights', [('Raw', state_text)])
    values = [(str(k).capitalize(), str(v)) for k, v in state.items()
              if k not in ('buffer_hex',)]
    return _status(OK, 'Lights', state.get('mode_name', 'ok'), 'lights', values)


def build_statuses(snap, timeout=3.0, alert_on_motor_status=False):
    """Snapshot dict -> list of :class:`Status` (see gui_bridge_node._diagnostics_snapshot)."""
    sensor = snap.get('sensor')
    sensor_age = snap.get('sensor_age')
    out = [mcu_status(snap.get('base_age'), sensor, sensor_age, timeout),
           battery_status(snap.get('battery'), snap.get('battery_age'), sensor, timeout)]
    for label, key in (('Cutter', 'cutter_motor'), ('Left drive', 'left_motor'),
                       ('Right drive', 'right_motor')):
        out.append(motor_status(label, (sensor or {}).get(key), sensor_age, timeout,
                                alert_on_motor_status))
    out.append(gnss_status(snap.get('fix'), snap.get('fix_age'), snap.get('fix_type'),
                           snap.get('fix_status'), snap.get('fix_status_age'), timeout))
    out.append(localization_status(snap.get('filtered_age'), snap.get('datum'), timeout))
    if 'imu_age' in snap:
        out.append(imu_status(snap.get('imu_age'), snap.get('imu_temperature'),
                              snap.get('imu_bias'), timeout))
    for cam in snap.get('cameras', ()):
        out.append(camera_status(cam['spec'], cam.get('age'), cam.get('publishers', 0),
                                 cam.get('viewers', 0), timeout))
    if 'lights' in snap:
        out.append(lights_status(snap.get('lights'), snap.get('lights_age'), timeout))
    return out
