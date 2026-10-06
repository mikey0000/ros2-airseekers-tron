"""Pure-python logic for the MowgliNext GUI bridge (no ROS imports).

Three pieces live here so they can be unit-tested on a host without rclpy:

* :class:`HighLevelStateMachine` - the *stub* mission state machine that answers
  ``/behavior_tree_node/high_level_control`` until a real behaviour tree exists.
  It never moves the robot itself; it only decides the published state and which
  side effects (cutter on/off, zero twist, clear e-stop) the node must perform.
* :func:`cutter_request_allowed` - the blade interlock normally enforced by the
  Mowgli firmware/BT: the blade may only be switched ON in AUTONOMOUS or
  MANUAL_MOWING and never while an emergency is active.  Switching OFF is always
  allowed.
* GNSS helpers - :func:`parse_fix_status` reads the free-form ``/fix_status``
  string of ``um960_gps_driver`` and :func:`derive_gnss_status` turns it plus the
  ``NavSatFix`` fields into the values of ``mowgli_interfaces/GnssStatus``.
"""

import math
import re

# ---------------------------------------------------------------------------
# mowgli_interfaces/HighLevelStatus state codes
# ---------------------------------------------------------------------------
STATE_NULL = 0          # also used for EMERGENCY (as in the Mowgli BT XML)
STATE_IDLE = 1
STATE_AUTONOMOUS = 2
STATE_RECORDING = 3
STATE_MANUAL_MOWING = 4

# mowgli_interfaces/HighLevelControl command codes
CMD_START = 1
CMD_HOME = 2
CMD_RECORD_AREA = 3
CMD_S2 = 4
CMD_RECORD_FINISH = 5
CMD_RECORD_CANCEL = 6
CMD_MANUAL_MOW = 7
CMD_STOP = 8
CMD_RESET_EMERGENCY = 254
CMD_DELETE_MAPS = 255

# Side effects requested from the ROS node, executed in list order *after*
# the new state has been published.
ACTION_CUTTER_ON = 'cutter_on'
ACTION_CUTTER_OFF = 'cutter_off'
ACTION_ZERO_TWIST = 'zero_twist'
ACTION_CLEAR_ESTOP = 'clear_estop'

BLADE_STATES = (STATE_AUTONOMOUS, STATE_MANUAL_MOWING)


class Result:
    """Outcome of a command or input update."""

    __slots__ = ('success', 'actions', 'log', 'changed')

    def __init__(self, success=True, actions=None, log='', changed=False):
        self.success = success
        self.actions = list(actions or [])
        self.log = log
        self.changed = changed

    def __repr__(self):
        return 'Result(success=%r, actions=%r, changed=%r, log=%r)' % (
            self.success, self.actions, self.changed, self.log)


def cutter_request_allowed(enable, hl_state, emergency_active):
    """Blade interlock. Returns ``(allowed, reason)``.

    OFF requests are always allowed (never refuse making the robot safer).
    ON requests need HL state AUTONOMOUS(2) or MANUAL_MOWING(4) and no emergency.
    """
    if not enable:
        return True, 'cutter off'
    if emergency_active:
        return False, 'refused: emergency active'
    if hl_state not in BLADE_STATES:
        return False, 'refused: high-level state %d is not AUTONOMOUS/MANUAL_MOWING' % hl_state
    return True, 'cutter on'


ROUTE_CUTTER = 'cutter'      # drive /cutter_control directly (built-in FSM / autonomous)
ROUTE_MISSION = 'mission'    # forward to the mission node's ~/manual_blade (SetBool)


def mower_control_route(serve_high_level, hl_state):
    """Where a GUI mow_enabled (``mower_control``) request goes.

    With the real mission node (``serve_high_level`` false) a request during
    MANUAL_MOWING goes to the mission node, which owns the manual blade state
    (two-step manual mowing, sub_state, interlocks); everything else keeps the
    direct /cutter_control path.
    """
    if not serve_high_level and hl_state == STATE_MANUAL_MOWING:
        return ROUTE_MISSION
    return ROUTE_CUTTER


def emergency_summary(estop, stop_triggered, lift_triggered, mcu_stale=False):
    """Fold the raw inputs into Emergency message fields.

    Returns ``(active, latched, lift_warning, reason)``.
    """
    causes = []
    if estop:
        causes.append('estop latched/interlock')
    if stop_triggered:
        causes.append('stop button')
    if lift_triggered:
        causes.append('lift')
    if mcu_stale:
        causes.append('mcu telemetry stale')
    active = bool(causes)
    return active, bool(estop), bool(lift_triggered), ', '.join(causes)


def battery_percent(percentage):
    """BatteryState.percentage (0..1, or NaN) -> 0..100."""
    if percentage is None or isinstance(percentage, float) and math.isnan(percentage):
        return 0.0
    p = float(percentage)
    if p <= 1.0:
        p *= 100.0
    return max(0.0, min(100.0, p))


class HighLevelStateMachine:
    """Stub of the Mowgli behaviour tree's top-level state handling.

    States: IDLE(1) / RECORDING(3) / MANUAL_MOWING(4) / EMERGENCY(0).  The
    machine is fed with commands (:meth:`command`) and with the emergency and
    docking inputs (:meth:`set_emergency`, :meth:`set_docked`).
    """

    def __init__(self):
        self.state = STATE_IDLE
        self.docked = False
        self.emergency = False

    # -- presentation ------------------------------------------------------
    @property
    def state_name(self):
        if self.emergency or self.state == STATE_NULL:
            return 'EMERGENCY'
        if self.state == STATE_IDLE:
            return 'IDLE_DOCKED' if self.docked else 'IDLE'
        if self.state == STATE_RECORDING:
            return 'RECORDING'
        if self.state == STATE_MANUAL_MOWING:
            return 'MANUAL_MOWING'
        if self.state == STATE_AUTONOMOUS:
            return 'AUTONOMOUS'
        return 'NULL'

    def snapshot(self):
        return (self.state, self.state_name)

    # -- internal ----------------------------------------------------------
    def _goto(self, new_state, actions):
        """Change state; any exit from MANUAL_MOWING turns the blade off."""
        if self.state == STATE_MANUAL_MOWING and new_state != STATE_MANUAL_MOWING:
            if ACTION_CUTTER_OFF not in actions:
                actions.insert(0, ACTION_CUTTER_OFF)
        self.state = new_state

    # -- inputs ------------------------------------------------------------
    def set_docked(self, docked):
        before = self.snapshot()
        self.docked = bool(docked)
        return Result(changed=self.snapshot() != before)

    def set_emergency(self, active):
        """Emergency input edge handling.

        Rising edge: force the cutter off and enter EMERGENCY (state 0).
        Falling edge: return to IDLE (never straight back into a blade state).
        """
        active = bool(active)
        before = self.snapshot()
        actions = []
        log = ''
        if active and not self.emergency:
            self.emergency = True
            self._goto(STATE_NULL, actions)
            if ACTION_CUTTER_OFF not in actions:
                actions.append(ACTION_CUTTER_OFF)
            log = 'emergency active -> EMERGENCY, cutter off'
        elif not active and self.emergency:
            self.emergency = False
            self._goto(STATE_IDLE, actions)
            log = 'emergency cleared -> IDLE'
        return Result(actions=actions, log=log, changed=self.snapshot() != before)

    def command(self, cmd):
        """Handle a HighLevelControl command; returns a :class:`Result`."""
        before = self.snapshot()
        actions = []
        cmd = int(cmd)

        if cmd == CMD_RESET_EMERGENCY:
            # The state only leaves EMERGENCY once the inputs actually clear
            # (set_emergency(False)); a still-pressed stop button keeps it.
            return Result(True, [ACTION_CLEAR_ESTOP], 'reset emergency requested')

        if cmd == CMD_STOP:
            if not self.emergency:
                self._goto(STATE_IDLE, actions)
            if ACTION_CUTTER_OFF not in actions:
                actions.append(ACTION_CUTTER_OFF)
            actions.append(ACTION_ZERO_TWIST)
            return Result(True, actions, 'stop: cutter off, zero twist',
                          self.snapshot() != before)

        if cmd in (CMD_START, CMD_HOME):
            return Result(False, [], 'command %d refused: no mission layer yet' % cmd)

        if cmd not in (CMD_RECORD_AREA, CMD_RECORD_FINISH, CMD_RECORD_CANCEL, CMD_MANUAL_MOW):
            return Result(False, [], 'command %d not supported by the stub' % cmd)

        if self.emergency:
            return Result(False, [], 'command %d refused: emergency active' % cmd)

        if cmd == CMD_RECORD_AREA:
            self._goto(STATE_RECORDING, actions)
            log = 'RECORDING (joystick, blade off; area recording not implemented)'
        elif cmd in (CMD_RECORD_FINISH, CMD_RECORD_CANCEL):
            self._goto(STATE_IDLE, actions)
            log = ('record %s -> IDLE (area recording not implemented, nothing saved)'
                   % ('finish' if cmd == CMD_RECORD_FINISH else 'cancel'))
        else:  # CMD_MANUAL_MOW
            if self.state != STATE_MANUAL_MOWING:
                self._goto(STATE_MANUAL_MOWING, actions)
            actions.append(ACTION_CUTTER_ON)
            log = 'MANUAL_MOWING (joystick, blade on)'
        return Result(True, actions, log, self.snapshot() != before)


# ---------------------------------------------------------------------------
# GNSS
# ---------------------------------------------------------------------------
# mowgli_interfaces/GnssStatus constants (duplicated so this module stays ROS-free)
FIX_TYPE_NO_FIX = 0
FIX_TYPE_GPS_FIX = 1
FIX_TYPE_RTK_FLOAT = 2
FIX_TYPE_RTK_FIXED = 3
FIX_TYPE_DEAD_RECKONING = 4

RTK_MODE_UNKNOWN = 0
RTK_MODE_NONE = 1
RTK_MODE_FLOAT = 2
RTK_MODE_FIXED = 3

CAP_RTK_MODE = 1
CAP_HDOP = 2
CAP_HORIZONTAL_ACCURACY = 8
CAP_VERTICAL_ACCURACY = 16
CAP_SATELLITES_USED = 128
CAP_SATELLITES_VISIBLE = 256
CAP_DIFFERENTIAL_CORRECTIONS = 1024
CAP_CORRECTIONS_ACTIVE = 2048
CAP_CORRECTION_AGE = 4096
CAP_CORRECTION_STREAM = 8388608
CAP_CORRECTION_TRANSPORT = 33554432
CAP_CORRECTION_FLOW = 67108864

CORRECTION_TRANSPORT_STATUS_UNKNOWN = 0
CORRECTION_TRANSPORT_STATUS_DISCONNECTED = 1
CORRECTION_TRANSPORT_STATUS_CONNECTING = 2
CORRECTION_TRANSPORT_STATUS_CONNECTED = 3
CORRECTION_TRANSPORT_STATUS_STREAMING = 4
CORRECTION_TRANSPORT_STATUS_RECONNECTING = 5
CORRECTION_TRANSPORT_STATUS_FAILED = 6

CORRECTION_STREAM_STATUS_UNKNOWN = 0
CORRECTION_STREAM_STATUS_IDLE = 1
CORRECTION_STREAM_STATUS_WAITING = 2
CORRECTION_STREAM_STATUS_ACTIVE = 3
CORRECTION_STREAM_STATUS_UNAVAILABLE = 4
CORRECTION_STREAM_STATUS_ERROR = 5

CORRECTION_FLOW_STATUS_UNKNOWN = 0
CORRECTION_FLOW_STATUS_IDLE = 1
CORRECTION_FLOW_STATUS_WAITING = 2
CORRECTION_FLOW_STATUS_ACTIVE = 3
CORRECTION_FLOW_STATUS_STALE = 4
CORRECTION_FLOW_STATUS_INVALID = 5

# ``corr=`` states of um960_gps_driver's NTRIP client -> transport status.
_TRANSPORT_BY_STATE = {
    'off': CORRECTION_TRANSPORT_STATUS_DISCONNECTED,
    'connecting': CORRECTION_TRANSPORT_STATUS_CONNECTING,
    'connected': CORRECTION_TRANSPORT_STATUS_CONNECTED,
    'streaming': CORRECTION_TRANSPORT_STATUS_STREAMING,
    'reconnecting': CORRECTION_TRANSPORT_STATUS_RECONNECTING,
    'auth_failed': CORRECTION_TRANSPORT_STATUS_FAILED,
    'bad_mountpoint': CORRECTION_TRANSPORT_STATUS_FAILED,
    'error': CORRECTION_TRANSPORT_STATUS_FAILED,
}
_FLOW_BY_TOKEN = {
    'idle': CORRECTION_FLOW_STATUS_IDLE,
    'waiting': CORRECTION_FLOW_STATUS_WAITING,
    # RTCM arrives from the caster but is held until the rover board acks NRTK.
    'held': CORRECTION_FLOW_STATUS_WAITING,
    'active': CORRECTION_FLOW_STATUS_ACTIVE,
    'stale': CORRECTION_FLOW_STATUS_STALE,
    'invalid': CORRECTION_FLOW_STATUS_INVALID,
}
# Legacy ``correction_stream_status`` (the only correction field the stock GUI reads,
# normally a Universal-GNSS diagnostics summary) mirrored from the driver's corr_flow.
_STREAM_BY_TOKEN = {
    'idle': CORRECTION_STREAM_STATUS_IDLE,
    'waiting': CORRECTION_STREAM_STATUS_WAITING,
    'held': CORRECTION_STREAM_STATUS_WAITING,
    'active': CORRECTION_STREAM_STATUS_ACTIVE,
    'stale': CORRECTION_STREAM_STATUS_UNAVAILABLE,
    'invalid': CORRECTION_STREAM_STATUS_ERROR,
}

# sensor_msgs/NavSatStatus / NavSatFix constants
NAVSAT_NO_FIX = -1
NAVSAT_FIX = 0
NAVSAT_SBAS_FIX = 1
NAVSAT_GBAS_FIX = 2
COVARIANCE_TYPE_UNKNOWN = 0

QUALITY_PERCENT = {
    FIX_TYPE_NO_FIX: 0.0,
    FIX_TYPE_GPS_FIX: 40.0,
    FIX_TYPE_RTK_FLOAT: 80.0,
    FIX_TYPE_RTK_FIXED: 100.0,
    FIX_TYPE_DEAD_RECKONING: 0.0,
}

# Tokens produced by um960_gps_driver: GGA ``quality=`` labels and Unicore
# BESTNAV ``solution=`` position types.
_FIXED_TOKENS = frozenset({'RTK_FIXED', 'FIXEDPOS', 'L1_INT', 'WIDE_INT', 'NARROW_INT',
                           'INS_RTKFIXED'})
_FLOAT_TOKENS = frozenset({'RTK_FLOAT', 'L1_FLOAT', 'IONOFREE_FLOAT', 'NARROW_FLOAT',
                           'INS_RTKFLOAT'})
_DGPS_TOKENS = frozenset({'DGPS', 'PSRDIFF', 'SBAS', 'INS_PSRDIFF'})
_GPS_TOKENS = frozenset({'GPS', 'SINGLE', 'FIXEDHEIGHT', 'DOPPLER_VELOCITY', 'INS',
                         'INS_PSRSP', 'PPP', 'PPP_CONVERGING'}) | _DGPS_TOKENS
_NOFIX_TOKENS = frozenset({'INVALID', 'NONE'})
_DR_TOKENS = frozenset({'DR'})

_KV_RE = re.compile(r'([A-Za-z0-9_]+)=([^\s,;]+)')


def parse_fix_status(text):
    """Parse the ``/fix_status`` string of um960_gps_driver.

    Example: ``um960=connected quality=RTK_FIXED sats=21/28 hdop=0.70 age=0.10s``.
    Returns a dict with keys ``connected`` (bool|None), ``solution`` (upper-case
    token from ``solution=`` or ``quality=``, or None), ``sats_used``,
    ``sats_total`` (int|None), ``hdop`` (float|None), and the correction tokens
    ``corr_src`` / ``corr`` / ``corr_flow`` (lower-case str|None), ``corr_age``,
    ``corr_rate`` and the receiver's own ``diff_age`` (float seconds|None).
    """
    out = {'connected': None, 'solution': None, 'sats_used': None,
           'sats_total': None, 'hdop': None, 'corr_src': None, 'corr': None,
           'corr_flow': None, 'corr_age': None, 'corr_rate': None, 'diff_age': None}
    if not text:
        return out
    kv = {}
    for key, value in _KV_RE.findall(text):
        kv.setdefault(key.lower(), value)
    if 'um960' in kv:
        out['connected'] = kv['um960'].lower() == 'connected'
    # BESTNAV position type is more specific than the GGA label; prefer it.
    sol = kv.get('solution') or kv.get('quality')
    if sol:
        out['solution'] = sol.upper()
    sats = kv.get('sats')
    if sats:
        used, _, total = sats.partition('/')
        try:
            out['sats_used'] = int(used)
            out['sats_total'] = int(total) if total else int(used)
        except ValueError:
            pass
    if 'hdop' in kv:
        try:
            out['hdop'] = float(kv['hdop'])
        except ValueError:
            pass
    for key in ('corr_src', 'corr', 'corr_flow'):
        if key in kv:
            out[key] = kv[key].lower()
    for key in ('corr_age', 'corr_rate', 'diff_age'):
        if key in kv:
            out[key] = _leading_float(kv[key])
    return out


def _leading_float(text):
    """``'0.5s'`` / ``'230B/s'`` -> float, or None."""
    match = re.match(r'[-+]?\d+(?:\.\d*)?', text)
    return float(match.group(0)) if match else None


def classify_solution(token):
    """Upper-case solution token -> FIX_TYPE_* or None when unknown."""
    if not token:
        return None
    if token in _FIXED_TOKENS:
        return FIX_TYPE_RTK_FIXED
    if token in _FLOAT_TOKENS:
        return FIX_TYPE_RTK_FLOAT
    if token in _NOFIX_TOKENS:
        return FIX_TYPE_NO_FIX
    if token in _DR_TOKENS:
        return FIX_TYPE_DEAD_RECKONING
    if token in _GPS_TOKENS:
        return FIX_TYPE_GPS_FIX
    # Generic fall-backs for unforeseen labels.
    if token.endswith('_INT') or 'FIXED' in token:
        return FIX_TYPE_RTK_FIXED
    if 'FLOAT' in token:
        return FIX_TYPE_RTK_FLOAT
    return None


def derive_gnss_status(navsat_status, covariance=None, covariance_type=COVARIANCE_TYPE_UNKNOWN,
                       parsed=None):
    """Compute GnssStatus field values.

    ``navsat_status`` is ``NavSatFix.status.status``; ``covariance`` the 9-element
    ``position_covariance``; ``parsed`` the (fresh) result of
    :func:`parse_fix_status` or None when ``/fix_status`` is missing/stale.
    Returns a dict keyed by GnssStatus field names.
    """
    parsed = parsed or {}
    token_type = classify_solution(parsed.get('solution'))

    if navsat_status is None or navsat_status <= NAVSAT_NO_FIX:
        fix_type = FIX_TYPE_NO_FIX            # NavSatFix "no fix" always wins
    elif token_type is not None:
        fix_type = token_type
        if fix_type == FIX_TYPE_NO_FIX:
            fix_type = FIX_TYPE_GPS_FIX       # receiver label lags NavSatFix; trust the fix
    elif navsat_status >= NAVSAT_GBAS_FIX:
        fix_type = FIX_TYPE_RTK_FLOAT         # GBAS = fixed OR float; be conservative
    else:
        fix_type = FIX_TYPE_GPS_FIX

    cap = CAP_RTK_MODE | CAP_DIFFERENTIAL_CORRECTIONS | CAP_HDOP | CAP_SATELLITES_USED \
        | CAP_SATELLITES_VISIBLE | CAP_HORIZONTAL_ACCURACY | CAP_VERTICAL_ACCURACY
    val = 0

    out = {
        'fix_type': fix_type,
        'fix_valid': fix_type in (FIX_TYPE_GPS_FIX, FIX_TYPE_RTK_FLOAT, FIX_TYPE_RTK_FIXED),
        'dead_reckoning': fix_type == FIX_TYPE_DEAD_RECKONING,
        'quality_percent': QUALITY_PERCENT[fix_type],
        'rtk_mode': RTK_MODE_UNKNOWN,
        'differential_corrections': False,
        'hdop': 0.0,
        'satellites_used': 0,
        'satellites_visible': 0,
        'horizontal_accuracy_m': 0.0,
        'vertical_accuracy_m': 0.0,
    }

    # RTK mode / differential corrections are known whenever we know the fix.
    if fix_type == FIX_TYPE_RTK_FIXED:
        out['rtk_mode'] = RTK_MODE_FIXED
    elif fix_type == FIX_TYPE_RTK_FLOAT:
        out['rtk_mode'] = RTK_MODE_FLOAT
    else:
        out['rtk_mode'] = RTK_MODE_NONE
    val |= CAP_RTK_MODE
    out['differential_corrections'] = (
        fix_type in (FIX_TYPE_RTK_FLOAT, FIX_TYPE_RTK_FIXED)
        or parsed.get('solution') in _DGPS_TOKENS
        or (navsat_status is not None and navsat_status >= NAVSAT_SBAS_FIX))
    val |= CAP_DIFFERENTIAL_CORRECTIONS

    if parsed.get('hdop') is not None:
        out['hdop'] = float(parsed['hdop'])
        val |= CAP_HDOP
    if parsed.get('sats_used') is not None:
        out['satellites_used'] = int(parsed['sats_used'])
        val |= CAP_SATELLITES_USED
    if parsed.get('sats_total') is not None:
        out['satellites_visible'] = int(parsed['sats_total'])
        val |= CAP_SATELLITES_VISIBLE

    _derive_corrections(out, parsed)
    cap |= CAP_CORRECTIONS_ACTIVE | CAP_CORRECTION_AGE | CAP_CORRECTION_TRANSPORT \
        | CAP_CORRECTION_FLOW | CAP_CORRECTION_STREAM
    val |= out.pop('_corr_value_flags')

    if covariance is not None and covariance_type != COVARIANCE_TYPE_UNKNOWN \
            and len(covariance) >= 9:
        h = max(float(covariance[0]), float(covariance[4]))
        if h > 0.0 and math.isfinite(h):
            out['horizontal_accuracy_m'] = math.sqrt(h)
            val |= CAP_HORIZONTAL_ACCURACY
        v = float(covariance[8])
        if v > 0.0 and math.isfinite(v):
            out['vertical_accuracy_m'] = math.sqrt(v)
            val |= CAP_VERTICAL_ACCURACY

    out['capability_flags'] = cap
    out['value_flags'] = val
    return out


def _derive_corrections(out, parsed):
    """Fill the GnssStatus correction fields from the ``corr*`` tokens.

    * ``correction_source``: ``ntrip`` / ``lora`` / ``none`` ('' when unknown).
    * ``correction_transport_status``: NTRIP client state; the LoRa radio link is
      invisible to the host, so it stays UNKNOWN (no value flag) for ``lora``.
    * ``correction_flow_status``: the driver's ``corr_flow`` (NTRIP: age of the last
      CRC-valid RTCM frame; LoRa: the receiver's differential age).
    * ``corrections_active``: flow ACTIVE.
    * ``correction_stream_status``: legacy summary mirrored from ``corr_flow``
      (idle->IDLE, waiting/held->WAITING, active->ACTIVE, stale->UNAVAILABLE,
      invalid->ERROR) so stock GUIs, which read only this field, show the flow.
    * ``correction_age_s``: the receiver's own ``diff_age`` when it reports one (what
      the solution actually uses), else the driver's ``corr_age``.
    Adds the private key ``_corr_value_flags`` (popped by the caller).
    """
    val = 0
    src = parsed.get('corr_src')
    state = parsed.get('corr')
    flow_token = parsed.get('corr_flow')
    out['correction_source'] = src if src in ('ntrip', 'lora', 'none') else ''
    out['correction_transport_status'] = CORRECTION_TRANSPORT_STATUS_UNKNOWN
    out['correction_response_accepted'] = False
    out['correction_flow_status'] = CORRECTION_FLOW_STATUS_UNKNOWN
    out['correction_stream_status'] = CORRECTION_STREAM_STATUS_UNKNOWN
    out['corrections_active'] = False
    out['correction_age_s'] = 0.0

    if src == 'ntrip' and state in _TRANSPORT_BY_STATE:
        out['correction_transport_status'] = _TRANSPORT_BY_STATE[state]
        out['correction_response_accepted'] = state in ('connected', 'streaming')
        val |= CAP_CORRECTION_TRANSPORT
    elif src == 'none':
        out['correction_transport_status'] = CORRECTION_TRANSPORT_STATUS_DISCONNECTED
        val |= CAP_CORRECTION_TRANSPORT

    if flow_token in _FLOW_BY_TOKEN:
        out['correction_flow_status'] = _FLOW_BY_TOKEN[flow_token]
        out['correction_stream_status'] = _STREAM_BY_TOKEN[flow_token]
        val |= CAP_CORRECTION_FLOW | CAP_CORRECTION_STREAM
    if src is not None:
        out['corrections_active'] = flow_token == 'active'
        val |= CAP_CORRECTIONS_ACTIVE

    age = parsed.get('diff_age')
    if age is None:
        age = parsed.get('corr_age')
    if age is not None and math.isfinite(age) and age >= 0.0:
        out['correction_age_s'] = float(age)
        val |= CAP_CORRECTION_AGE
    out['_corr_value_flags'] = val
