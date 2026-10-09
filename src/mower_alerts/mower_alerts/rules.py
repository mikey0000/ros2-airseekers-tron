# SPDX-License-Identifier: GPL-3.0-or-later
"""Theft, lift and incident alert rules (pure logic, no ROS imports).

The node (``alert_node.py``) samples its inputs into an :class:`Inputs` snapshot a few
times per second and calls :meth:`AlertEngine.step`; every :class:`Alert` that comes out
is published once on ``/mower_alerts/events``. The MowgliNext GUI backend turns those into
pushes over the operator's channel (Telegram / Pushover / ntfy / webhook) and forwards them
to its MQTT broker; the operator toggles them per kind on the notifications settings page.

Spam control, in this order:

* debounce: a condition must hold for ``debounce_s`` before its incident fires;
* dedupe: an incident fires once, then stays silent until the condition has been clear for
  ``clear_s`` (re-arm). Optional ``repeat_s`` re-notifies a persisting incident (theft
  position updates), at most ``max_repeats`` times;
* ``min_interval_s``: an incident never fires twice within this window, even after re-arm;
* groups: incidents describing the same event (lifted while parked, moved off the parked
  spot and out of the map are one theft) share a window; only the first one is sent;
* global rate limit: at most ``rate_limit_count`` alerts per ``rate_limit_window_s``.
  Priority-5 alerts (theft, lift, emergency) bypass it; the count of suppressed alerts
  rides along on the next one that is sent.

Which mission states mean what is read from the mower_mission ``state_name`` set
(``mower_mission/mission_fsm.py`` STATE_CODES) and is not changed here.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# HighLevelStatus.state codes
HL_NULL = 0
HL_IDLE = 1
HL_AUTONOMOUS = 2
HL_RECORDING = 3
HL_MANUAL_MOWING = 4

# operating modes the rules care about
MODE_UNKNOWN = 'unknown'
MODE_PARKED = 'parked'      # idle / docked / charging: theft rules armed
MODE_MISSION = 'mission'    # autonomous mowing session (incl. docking)
MODE_MANUAL = 'manual'      # joystick driving: RECORDING / MANUAL_MOWING

# GUI notification kinds (gui/pkg/providers/notify_events.go)
KIND_THEFT = 'theft'
KIND_LIFT = 'lift'
KIND_EMERGENCY = 'emergency'
KIND_BLOCKED = 'blocked'
KIND_BATTERY = 'battery'
KIND_GPS_LOST = 'gpsLost'

PRIORITY_DEFAULT = 3
PRIORITY_HIGH = 4
PRIORITY_MAX = 5

# mower_mission state names consumed here (see mission_fsm.STATE_CODES)
STATE_STUCK = 'STUCK_NEEDS_HELP'
STATE_INCOMPLETE = 'MOWING_INCOMPLETE'
STATE_COVERAGE_FAILED = 'COVERAGE_FAILED_DOCKING'
STATE_DOCK_FAILED = 'NAV_TO_DOCK_FAILED'
STATE_UNDOCK_FAILED = 'UNDOCK_FAILED'
STATE_BOUNDARY_ESTOP = 'BOUNDARY_EMERGENCY_STOP'
# phases where the RTK-lost rule applies (WAITING_FOR_RTK has its own GUI notification)
RTK_WATCH_PHASES = ('PLANNING', 'MOWING', 'TRANSIT', 'AREA_UNREACHABLE', 'BOUNDARY_PAUSED',
                    'RETURNING_HOME', 'LOW_BATTERY_DOCKING', 'RAIN_DETECTED_DOCKING',
                    'COVERAGE_FAILED_DOCKING')
PATH_BLOCKED_PREFIX = 'path blocked'
FIX_RTK_FIXED = 3           # GnssStatus.FIX_TYPE_RTK_FIXED

EARTH_RADIUS_M = 6371008.8


@dataclass
class Params:
    enabled: bool = True
    # global spam control
    rate_limit_count: int = 6
    rate_limit_window_s: float = 600.0
    clear_s: float = 30.0               # an incident re-arms after its condition is clear this long
    stale_s: float = 3.0                # sensor inputs older than this count as missing
    # theft: armed after parked this long (the operator just stopped / carried it)
    theft_arm_s: float = 60.0
    theft_lift_debounce_s: float = 2.0
    theft_repeat_s: float = 300.0       # position updates while the theft persists
    theft_max_repeats: int = 6
    geofence_radius_m: float = 10.0
    geofence_debounce_s: float = 10.0
    geofence_max_accuracy_m: float = 5.0  # ignore fixes less accurate than this
    map_exit_margin_m: float = 3.0      # outside the mapped area by more than this
    map_exit_debounce_s: float = 10.0
    # lift / tilt while mowing or driving
    lift_debounce_s: float = 0.5
    tilt_max_deg: float = 35.0
    tilt_debounce_s: float = 1.0
    # mission incidents
    emergency_debounce_s: float = 1.0
    path_blocked_alert_s: float = 45.0  # mission gives up after blocked_wait_s (60 s)
    path_blocked_min_interval_s: float = 600.0
    rtk_lost_alert_s: float = 300.0
    rtk_recovered_s: float = 10.0
    battery_critical_pct: float = 10.0
    battery_debounce_s: float = 30.0


@dataclass
class Fix:
    lat: float
    lon: float
    accuracy_m: float           # horizontal 1-sigma, inf when unknown
    fix_type: Optional[int] = None  # GnssStatus.FIX_TYPE_*, None when unknown


@dataclass
class Inputs:
    """One snapshot. None = not received or stale (the node applies ``stale_s``)."""
    state: Optional[int] = None         # HighLevelStatus.state
    state_name: str = ''
    sub_state_name: str = ''
    battery_percent: Optional[float] = None
    hl_emergency: bool = False
    hl_charging: bool = False
    lift: Optional[bool] = None         # MowerBaseDevStatus.lift_triggered
    stop_button: Optional[bool] = None  # MowerBaseDevStatus.stop_triggered
    docked: Optional[bool] = None       # is_docking_done
    charging: Optional[bool] = None     # is_charging
    emergency_reason: str = ''          # /hardware_bridge/emergency reason
    fix: Optional[Fix] = None
    rtk_fix_type: Optional[int] = None  # /gps/status fix_type
    pose: Optional[Tuple[float, float]] = None  # map frame (x, y)
    tilt_deg: Optional[Tuple[float, float]] = None  # (roll, pitch) degrees


@dataclass
class Alert:
    incident: str
    kind: str
    message: str
    priority: int
    text: str
    params: Dict[str, str] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def distance_m(lat1, lon1, lat2, lon2):
    """Great-circle distance (haversine), metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def mode_for(state, state_name, previous=MODE_UNKNOWN):
    """HighLevelStatus -> operating mode. NULL-coded states (EMERGENCY,
    BOUNDARY_EMERGENCY_STOP, NAV_TO_DOCK_FAILED) keep the previous mode: a lift that
    trips the e-stop while parked is still a parked lift."""
    if state is None:
        return previous
    if state == HL_AUTONOMOUS:
        return MODE_MISSION
    if state in (HL_RECORDING, HL_MANUAL_MOWING):
        return MODE_MANUAL
    if state == HL_IDLE:
        return MODE_PARKED
    return previous


def tilt_from_quaternion(x, y, z, w):
    """(roll, pitch) in degrees, or None for an unset quaternion."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-6:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def tilt_angle(roll_pitch):
    """Angle of the body z axis from vertical, degrees."""
    r, p = (math.radians(v) for v in roll_pitch)
    c = math.cos(r) * math.cos(p)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def accuracy_from_covariance(cov, cov_type):
    """NavSatFix position_covariance -> horizontal 1-sigma metres (inf when unknown)."""
    if not cov_type or cov is None or len(cov) < 5:
        return math.inf
    c = max(float(cov[0]), float(cov[4]))
    return math.sqrt(c) if c > 0.0 else math.inf


class MapMask:
    """The map server's latched /keepout_mask (OccupancyGrid, row 0 = lowest y):
    0 inside a mowing/navigation area, 100 outside, in obstacles and on the dock."""

    def __init__(self, origin_x, origin_y, resolution, width, height, data: Sequence[int]):
        self.ox, self.oy = float(origin_x), float(origin_y)
        self.res = float(resolution)
        self.w, self.h = int(width), int(height)
        self.data = data
        self.has_free = any(0 <= int(v) < 50 for v in data)

    def _free(self, r, c):
        v = int(self.data[r * self.w + c])
        return 0 <= v < 50

    def distance_outside(self, x, y, max_r):
        """0 when (x, y) is in a free cell, else the distance to the nearest free cell
        centre within ``max_r`` (inf when none is that close)."""
        if self.res <= 0.0 or not self.has_free:
            return math.inf
        col = int(math.floor((x - self.ox) / self.res))
        row = int(math.floor((y - self.oy) / self.res))
        if 0 <= row < self.h and 0 <= col < self.w and self._free(row, col):
            return 0.0
        k = int(math.ceil(max_r / self.res)) + 1
        best = math.inf
        for r in range(max(0, row - k), min(self.h, row + k + 1)):
            cy = self.oy + (r + 0.5) * self.res
            for c in range(max(0, col - k), min(self.w, col + k + 1)):
                if self._free(r, c):
                    cx = self.ox + (c + 0.5) * self.res
                    d = math.hypot(cx - x, cy - y)
                    if d < best:
                        best = d
        return best if best <= max_r else math.inf


# ---------------------------------------------------------------------------
# incident state machine
# ---------------------------------------------------------------------------
FIRE = 'fire'
REPEAT = 'repeat'
CLEARED = 'cleared'


class Incident:
    """Debounce + dedupe for one condition (see module docstring)."""

    def __init__(self, debounce_s=0.0, clear_s=30.0, repeat_s=0.0, max_repeats=0,
                 min_interval_s=0.0):
        self.debounce_s = float(debounce_s)
        self.clear_s = float(clear_s)
        self.repeat_s = float(repeat_s)
        self.max_repeats = int(max_repeats)
        self.min_interval_s = float(min_interval_s)
        self.active = False         # fired and not yet re-armed
        self.since = None           # condition true since (pending or active)
        self.clear_since = None
        self.fired_at = None
        self.last_sent = None
        self.repeats = 0

    def update(self, cond, now):
        """Returns FIRE, REPEAT, CLEARED or None."""
        if cond:
            self.clear_since = None
            if self.since is None:
                self.since = now
            if not self.active:
                if now - self.since < self.debounce_s:
                    return None
                if self.fired_at is not None and now - self.fired_at < self.min_interval_s:
                    return None
                self.active = True
                self.fired_at = self.last_sent = now
                self.repeats = 0
                return FIRE
            if self.repeat_s > 0.0 and self.repeats < self.max_repeats \
                    and now - self.last_sent >= self.repeat_s:
                self.repeats += 1
                self.last_sent = now
                return REPEAT
            return None
        self.since = None if not self.active else self.since
        if not self.active:
            return None
        if self.clear_since is None:
            self.clear_since = now
        if now - self.clear_since >= self.clear_s:
            self.active = False
            self.since = None
            self.clear_since = None
            return CLEARED
        return None

    def reset(self):
        self.active = False
        self.since = self.clear_since = None


# ---------------------------------------------------------------------------
# engine
# ---------------------------------------------------------------------------
GROUPS = {
    'theft_lift': 'theft', 'theft_geofence': 'theft', 'theft_outside_map': 'theft',
    'lift_during_mow': 'lift', 'tilt_during_mow': 'lift',
}


class AlertEngine:
    def __init__(self, params: Optional[Params] = None):
        self.p = params or Params()
        p = self.p
        c = p.clear_s
        self.incidents = {
            'theft_lift': Incident(p.theft_lift_debounce_s, c, p.theft_repeat_s,
                                   p.theft_max_repeats),
            'theft_geofence': Incident(p.geofence_debounce_s, c, p.theft_repeat_s,
                                       p.theft_max_repeats),
            'theft_outside_map': Incident(p.map_exit_debounce_s, c, p.theft_repeat_s,
                                          p.theft_max_repeats),
            'lift_during_mow': Incident(p.lift_debounce_s, c),
            'tilt_during_mow': Incident(p.tilt_debounce_s, c),
            'emergency_stop': Incident(p.emergency_debounce_s, c),
            'stuck': Incident(0.0, c),
            'path_blocked': Incident(p.path_blocked_alert_s, c,
                                     min_interval_s=p.path_blocked_min_interval_s),
            'mow_incomplete': Incident(0.0, c),
            'dock_failed': Incident(0.0, c),
            'undock_failed': Incident(0.0, c),
            'battery_critical': Incident(p.battery_debounce_s, 300.0),
            'rtk_lost': Incident(p.rtk_lost_alert_s, p.rtk_recovered_s),
        }
        self.group_window = {'theft': p.theft_repeat_s, 'lift': 60.0}
        self.group_last = {}
        self.mode = MODE_UNKNOWN
        self.parked_since = None
        self.anchor: Optional[Fix] = None
        self.anchor_changed = False
        self.mask: Optional[MapMask] = None
        self.estop_from_lift = False
        self.sent_times: List[float] = []
        self.suppressed = 0

    # ---- external state -----------------------------------------------
    def set_mask(self, mask: Optional[MapMask]):
        self.mask = mask

    def restore_anchor(self, anchor: Optional[Fix]):
        """Parked position persisted across a restart (theft while powered off)."""
        self.anchor = anchor

    def active_incidents(self):
        return sorted(k for k, inc in self.incidents.items() if inc.active)

    # ---- main -----------------------------------------------------------
    def step(self, i: Inputs, now: float) -> List[Alert]:
        if not self.p.enabled:
            return []
        self._update_mode(i, now)
        out: List[Alert] = []
        for key, cond, build in self._rules(i, now):
            result = self.incidents[key].update(bool(cond), now)
            if result in (FIRE, REPEAT):
                alert = build()
                if alert is not None and self._group_allows(key, now):
                    out.append(alert)
            elif result == CLEARED and key == 'rtk_lost' and i.rtk_fix_type is not None \
                    and i.rtk_fix_type >= FIX_RTK_FIXED:
                # only a real recovery; a mission that ended while still without RTK is not
                out.append(Alert('rtk_lost', KIND_GPS_LOST, 'rtkRecovered', PRIORITY_DEFAULT,
                                 'RTK fix recovered.', {}))
        return self._rate_limit(out, now)

    def _update_mode(self, i, now):
        mode = mode_for(i.state, i.state_name, self.mode)
        if mode == MODE_UNKNOWN and (i.docked or i.charging):
            mode = MODE_PARKED
        if mode != self.mode:
            if mode == MODE_PARKED:
                self.parked_since = now
            elif self.mode == MODE_PARKED or mode in (MODE_MISSION, MODE_MANUAL):
                # the robot drives off on purpose: forget the parked spot
                self.parked_since = None
                if self.anchor is not None:
                    self.anchor = None
                    self.anchor_changed = True
                for k in ('theft_lift', 'theft_geofence', 'theft_outside_map'):
                    self.incidents[k].reset()
            self.mode = mode

    def theft_armed(self, now):
        return self.mode == MODE_PARKED and self.parked_since is not None \
            and now - self.parked_since >= self.p.theft_arm_s

    def _rules(self, i: Inputs, now):
        p = self.p
        armed = self.theft_armed(now)
        driving = self.mode in (MODE_MISSION, MODE_MANUAL)
        name = i.state_name or ''

        geofence_d = self._geofence_distance(i, armed)
        map_out = self._outside_map(i, armed)
        tilt = tilt_angle(i.tilt_deg) if i.tilt_deg is not None else None
        rtk_bad = (self.mode == MODE_MISSION and name in RTK_WATCH_PHASES
                   and (i.rtk_fix_type is None or i.rtk_fix_type < FIX_RTK_FIXED))
        on_dock = bool(i.docked) or bool(i.charging) or bool(i.hl_charging)
        # An e-stop the lift tripped is reported by the lift / theft rules; keep it silent
        # until the latch is cleared, also after the robot is put down again.
        estop = bool(i.hl_emergency) or name == STATE_BOUNDARY_ESTOP
        if not estop:
            self.estop_from_lift = False
        elif bool(i.lift) and (armed or driving):
            self.estop_from_lift = True

        return [
            ('theft_lift', armed and bool(i.lift),
             lambda: self._theft('theft_lift', 'theftLift', i,
                                 'THEFT ALERT: the robot was lifted while parked.')),
            ('theft_geofence', geofence_d is not None and geofence_d > p.geofence_radius_m,
             lambda: self._theft('theft_geofence', 'theftGeofence', i,
                                 'THEFT ALERT: the robot moved %.0f m from where it was '
                                 'parked.' % (geofence_d or 0.0), distance=geofence_d)),
            ('theft_outside_map', map_out is not None and map_out > p.map_exit_margin_m,
             lambda: self._theft('theft_outside_map', 'theftOutsideMap', i,
                                 'THEFT ALERT: the robot is outside the mapped area while '
                                 'not mowing.', distance=map_out)),
            ('lift_during_mow', driving and bool(i.lift),
             lambda: Alert('lift_during_mow', KIND_LIFT, 'liftDuringMow', PRIORITY_MAX,
                           'Robot LIFTED while mowing: blade and wheels stopped.',
                           self._base_params(i))),
            ('tilt_during_mow', driving and tilt is not None and tilt > p.tilt_max_deg,
             lambda: Alert('tilt_during_mow', KIND_LIFT, 'tiltDuringMow', PRIORITY_MAX,
                           'Robot TILTED %.0f deg while mowing.' % tilt,
                           dict(self._base_params(i), detail='%.0f°' % tilt))),
            ('emergency_stop', estop and not self.estop_from_lift,
             lambda: self._emergency(i)),
            ('stuck', name == STATE_STUCK,
             lambda: Alert('stuck', KIND_BLOCKED, 'stuck', PRIORITY_HIGH,
                           'Stuck: the robot could not free itself and needs help.',
                           dict(self._base_params(i), detail=i.sub_state_name))),
            ('path_blocked', self.mode == MODE_MISSION
             and (i.sub_state_name or '').startswith(PATH_BLOCKED_PREFIX),
             lambda: Alert('path_blocked', KIND_BLOCKED, 'pathBlocked', PRIORITY_DEFAULT,
                           'Path blocked: the robot is waiting for the obstacle to clear.',
                           dict(self._base_params(i),
                                minutes=_minutes(p.path_blocked_alert_s)))),
            ('mow_incomplete', name in (STATE_INCOMPLETE, STATE_COVERAGE_FAILED),
             lambda: Alert('mow_incomplete', KIND_BLOCKED, 'mowIncomplete', PRIORITY_DEFAULT,
                           'Mowing incomplete: %s.' % (i.sub_state_name or name),
                           dict(self._base_params(i),
                                detail=i.sub_state_name or name.lower()))),
            ('dock_failed', name == STATE_DOCK_FAILED,
             lambda: Alert('dock_failed', KIND_BLOCKED, 'dockFailed', PRIORITY_HIGH,
                           'Docking failed: %s.' % (i.sub_state_name or name),
                           dict(self._base_params(i), detail=i.sub_state_name or '-'))),
            ('undock_failed', name == STATE_UNDOCK_FAILED,
             lambda: Alert('undock_failed', KIND_BLOCKED, 'undockFailed', PRIORITY_DEFAULT,
                           'Undocking failed: %s.' % (i.sub_state_name or name),
                           dict(self._base_params(i), detail=i.sub_state_name or '-'))),
            ('battery_critical', i.battery_percent is not None and i.battery_percent > 0.0
             and i.battery_percent <= p.battery_critical_pct and not on_dock,
             lambda: Alert('battery_critical', KIND_BATTERY, 'batteryCritical', PRIORITY_HIGH,
                           'Battery critical (%.0f %%) and the robot is not on the dock.'
                           % i.battery_percent, self._base_params(i))),
            ('rtk_lost', rtk_bad,
             lambda: Alert('rtk_lost', KIND_GPS_LOST, 'rtkLost', PRIORITY_DEFAULT,
                           'RTK fix lost for %s during the mission.'
                           % _minutes(p.rtk_lost_alert_s),
                           dict(self._base_params(i), minutes=_minutes(p.rtk_lost_alert_s),
                                detail=_fix_label(i.rtk_fix_type)))),
        ]

    # ---- rule helpers ---------------------------------------------------
    def _geofence_distance(self, i, armed):
        """Distance from the parked anchor (m), or None when not applicable. Keeps the
        anchor: first trusted fix after arming, refined by a clearly better nearby one."""
        if not armed:
            return None
        f = i.fix
        if f is None or not (f.accuracy_m <= self.p.geofence_max_accuracy_m):
            return None
        if self.anchor is None:
            self.anchor = Fix(f.lat, f.lon, f.accuracy_m, f.fix_type)
            self.anchor_changed = True
            return 0.0
        d = distance_m(self.anchor.lat, self.anchor.lon, f.lat, f.lon)
        if f.accuracy_m < 0.5 * self.anchor.accuracy_m and d < 0.5 * self.p.geofence_radius_m:
            self.anchor = Fix(f.lat, f.lon, f.accuracy_m, f.fix_type)
            self.anchor_changed = True
            return 0.0
        return d

    def _outside_map(self, i, armed):
        if not armed or self.mask is None or i.pose is None:
            return None
        return self.mask.distance_outside(i.pose[0], i.pose[1],
                                          self.p.map_exit_margin_m + 1.0)

    def _base_params(self, i):
        out = {'state': i.state_name or '-'}
        if i.battery_percent is not None:
            out['battery'] = '%.0f %%' % i.battery_percent
        return out

    def _position_params(self, i):
        f = i.fix
        if f is None:
            return {'lat': '?', 'lon': '?', 'map': '(no GPS fix)'}
        return {'lat': '%.6f' % f.lat, 'lon': '%.6f' % f.lon,
                'map': 'https://maps.google.com/?q=%.6f,%.6f' % (f.lat, f.lon)}

    def _theft(self, key, message, i, text, distance=None):
        params = dict(self._base_params(i))
        params.update(self._position_params(i))
        if distance is None and self.anchor is not None and i.fix is not None:
            # a lift (or map exit) also says how far it has gone from the parked spot
            distance = distance_m(self.anchor.lat, self.anchor.lon, i.fix.lat, i.fix.lon)
            if distance >= 1.0:
                text += ' Moved %.0f m from where it was parked.' % distance
        if distance is not None and math.isfinite(distance):
            params['distance'] = '%.0f m' % distance
        if i.fix is not None:
            text += ' Last position: %s' % params['map']
        return Alert(key, KIND_THEFT, message, PRIORITY_MAX, text, params)

    def _emergency(self, i):
        causes = []
        if i.stop_button:
            causes.append('stop button')
        if i.lift:
            causes.append('lift')
        if (i.state_name or '') == STATE_BOUNDARY_ESTOP:
            causes.append('left the mapped area while mowing')
        if not causes:
            causes.append(i.emergency_reason or 'firmware emergency latch')
        cause = ', '.join(causes)
        return Alert('emergency_stop', KIND_EMERGENCY, 'emergencyStop', PRIORITY_MAX,
                     'Emergency stop: %s.' % cause, dict(self._base_params(i), cause=cause))

    # ---- spam control -----------------------------------------------------
    def _group_allows(self, key, now):
        group = GROUPS.get(key)
        if group is None:
            return True
        last = self.group_last.get(group)
        if last is not None and last[0] != key and now - last[1] < self.group_window[group]:
            return False
        self.group_last[group] = (key, now)
        return True

    def _rate_limit(self, alerts, now):
        p = self.p
        window = float(p.rate_limit_window_s)
        self.sent_times = [t for t in self.sent_times if now - t < window]
        out = []
        for a in alerts:
            if a.priority < PRIORITY_MAX and len(self.sent_times) >= int(p.rate_limit_count):
                self.suppressed += 1
                continue
            if self.suppressed:
                a.params['suppressed'] = str(self.suppressed)
                self.suppressed = 0
            self.sent_times.append(now)
            out.append(a)
        return out


def _minutes(seconds):
    return '%d min' % max(1, int(round(float(seconds) / 60.0)))


def _fix_label(fix_type):
    return {None: 'no GNSS status', 0: 'no fix', 1: 'GPS only', 2: 'RTK float',
            4: 'dead reckoning'}.get(fix_type, 'fix type %s' % fix_type)
