"""tilt_monitor CRITICAL stop holds the emergency lane while critical (2026-10-13 review):
the old 0.5 s burst let twist_mux (emergency timeout 0.2 s) hand the wheels back to the
navigation lane ~0.7 s after entering CRITICAL unless the mission closed the gate."""

import math
import types

from mower_control import tilt_logic
from mower_control.tilt_monitor import TiltMonitorNode

RATE = 25.0
TWIST_MUX_EMERGENCY_TIMEOUT = 0.2


class _Pub:
    def __init__(self):
        self.times = []
        self.now = None

    def publish(self, _msg):
        self.times.append(self.now)


def _node(**kw):
    log = types.SimpleNamespace()
    for level in ('debug', 'info', 'warn', 'warning', 'error'):
        setattr(log, level, lambda text: None)
    n = types.SimpleNamespace(
        _bands=tilt_logic.TiltBands(), _critical_stop=True, _zero_burst_s=0.5,
        _critical_hold_s=1.0, _status_period=0.5, _band=None, _last_status=-1e9,
        _burst_until=0.0, _inverted_warned=False, _stale_warned=False,
        _twist_pub=_Pub(), _status_pub=_Pub(), _estop_pub=None,
        _cutter_off=types.SimpleNamespace(service_is_ready=lambda: False, srv_name='/c'),
        get_logger=lambda: log)
    n._on_band = lambda old, new, now=None: TiltMonitorNode._on_band(n, old, new, now)
    for k, v in kw.items():
        setattr(n, k, v)
    return n


def drive(node, t0, t1, roll_deg, imu=True):
    t = t0
    while t < t1 - 1e-9:
        t = round(t + 1.0 / RATE, 6)
        if imu:
            node._bands.update(t, math.radians(roll_deg), 0.0)
        node._twist_pub.now = t
        TiltMonitorNode.step(node, t)
    return t


def lane_live(pub, t):
    """Would twist_mux still hold the emergency lane at t?"""
    return any(0.0 <= t - p <= TWIST_MUX_EMERGENCY_TIMEOUT for p in pub.times)


def test_zero_stream_lasts_as_long_as_critical():
    crit = tilt_logic.DEFAULTS['roll_thresholds_deg'][2] + 5.0
    n = _node()
    drive(n, 0.0, 1.0, 0.0)
    t = drive(n, 1.0, 6.0, crit)                       # 5 s on a critical slope
    entered = min(n._twist_pub.times)
    assert entered < 2.0
    for k in range(1, int((6.0 - entered) * 20)):
        assert lane_live(n._twist_pub, entered + k * 0.05), entered + k * 0.05
    # level ground again: stream ends critical_hold_s after the band drops
    drive(n, t, 12.0, 0.0)
    assert n._band != 'critical'
    last = max(n._twist_pub.times)
    assert last < 12.0 - 1.0


def test_imu_lost_while_critical_keeps_the_stop():
    crit = tilt_logic.DEFAULTS['roll_thresholds_deg'][2] + 5.0
    n = _node()
    t = drive(n, 0.0, 3.0, crit)
    t = drive(n, t, 8.0, crit, imu=False)              # IMU silent: band unknown
    assert n._band == tilt_logic.UNKNOWN
    assert lane_live(n._twist_pub, 7.95)


def test_estop_request_once_when_enabled():
    crit = tilt_logic.DEFAULTS['roll_thresholds_deg'][2] + 5.0
    e = _Pub()
    n = _node(_estop_pub=e)
    drive(n, 0.0, 4.0, crit)
    assert len(e.times) == 1


def test_no_stop_when_disabled_or_below_critical():
    lim = tilt_logic.DEFAULTS['roll_thresholds_deg'][1] + 1.0
    n = _node()
    drive(n, 0.0, 5.0, lim)
    assert n._twist_pub.times == []
    crit = tilt_logic.DEFAULTS['roll_thresholds_deg'][2] + 5.0
    n = _node(_critical_stop=False)
    drive(n, 0.0, 5.0, crit)
    assert n._twist_pub.times == []
