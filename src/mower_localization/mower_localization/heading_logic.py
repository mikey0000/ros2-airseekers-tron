# SPDX-License-Identifier: GPL-3.0-or-later
"""Absolute-heading alignment for a gyro-integrated IMU yaw (pure logic, no ROS imports).

Problem: the WIT JY61P is a 6-axis IMU. Its yaw is integrated from the gyro since power-on,
so ``/imu/data`` yaw has an arbitrary, constant-ish offset to ENU (east = 0, CCW+) plus slow
gyro-bias drift. ``ekf_node`` fuses that yaw as an absolute pose yaw, so without alignment
Nav2 steers with a heading that can be off by anything up to 180 deg.

Model: ``yaw_enu = yaw_imu + offset``. :class:`HeadingEstimator` estimates ``offset`` from,
in priority order:

``cog``   course over ground. While the commanded motion is a straight line
          (|v| >= ``min_speed``, |w| < ``max_turn_rate``) and RTK (float or fixed) is up,
          the GPS displacement is accumulated. After ``min_distance`` m (``float_min_distance``
          with RTK float) of straight travel, ``cog = atan2(dN, dE)`` (+pi when reversing) and
          ``offset = cog - circular_mean(imu yaw over the window)``. Windows whose GPS track is
          not straight (lateral deviation from the chord) or whose IMU yaw changes too much
          are rejected. Measurements are low-passed into the offset (``alpha_initial`` for the
          first ``initial_updates`` COG updates, ``alpha`` afterwards); once aligned on COG a
          measurement more than ``outlier_deg`` away is rejected, unless ``reseed_after``
          consecutive rejected measurements agree with each other (the IMU yaw jumped). Every
          window is non-overlapping, so long straight runs keep tracking gyro drift.
``dock``  while docked (is_charging / is_docking_done) and no fresh COG: the robot sits in
          the dock with its heading equal to the dock pose yaw (mower_docking convention: the
          robot reverses onto the charger, so docked it faces AWAY from the charger along the
          dock yaw; undocking drives forward along +X = dock yaw). ``offset = dock_yaw +
          dock_heading_offset - mean imu yaw``.
``file``  the persisted offset of the last COG update. Low quality (the IMU yaw restarts at
          an arbitrary value on every IMU power cycle), never counts as aligned.

The estimator also detects an IMU yaw reset (a step in yaw that the gyro rate does not
explain) and drops back to unaligned.
"""

import json
import math
import os
import struct
import tempfile
import time

SOURCE_NONE = 'none'
SOURCE_FILE = 'file'
SOURCE_DOCK = 'dock'
SOURCE_COG = 'cog'
ALIGNED_SOURCES = (SOURCE_DOCK, SOURCE_COG)

TWO_PI = 2.0 * math.pi


def wrap(a):
    """Angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, TWO_PI)
    if a <= 0.0:
        a += TWO_PI
    return a - math.pi


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotate_yaw(q, offset):
    """World-frame yaw rotation: Rz(offset) * q, for q = (x, y, z, w). Keeps roll/pitch."""
    x, y, z, w = q
    s, c = math.sin(offset / 2.0), math.cos(offset / 2.0)
    # (0, 0, s, c) (x) (x, y, z, w)
    return (c * x - s * y, c * y + s * x, c * z + s * w, c * w - s * z)


class CircularMean:
    __slots__ = ('s', 'c', 'n')

    def __init__(self):
        self.s = self.c = 0.0
        self.n = 0

    def add(self, a):
        self.s += math.sin(a)
        self.c += math.cos(a)
        self.n += 1

    def mean(self):
        return math.atan2(self.s, self.c) if self.n else None


class AlignParams:
    """Tunables (all angles in degrees in the ROS parameters, radians here)."""

    def __init__(self, **kw):
        self.min_speed = 0.12            # m/s |cmd linear| (the undock runs at 0.15)
        self.max_turn_rate = 0.05        # rad/s |cmd angular|
        self.cmd_timeout = 0.5           # s; older /cmd_vel counts as "stopped"
        self.min_fix_type = 2            # GnssStatus: 2 RTK float, 3 RTK fixed
        self.max_position_variance = 1.0  # m^2 sanity cap (UM960 reports HDOP-based cov)
        self.gps_timeout = 0.5           # s
        self.imu_timeout = 0.2           # s
        self.min_distance = 0.6          # m of straight travel per COG measurement (RTK fixed)
        self.float_min_distance = 1.5    # m with RTK float (decimetre noise)
        self.max_window_time = 30.0      # s; a slower window is restarted
        self.max_lateral = 0.08          # m GPS deviation from the chord
        self.max_imu_yaw_change = math.radians(6.0)   # IMU yaw span inside a window
        self.alpha_initial = 0.3
        self.initial_updates = 3         # COG updates that use alpha_initial
        self.alpha = 0.1
        self.outlier = math.radians(30.0)
        self.reseed_after = 3            # consecutive agreeing outliers -> re-seed
        self.reseed_agree = math.radians(10.0)
        self.dock_settle = 2.0           # s docked before seeding
        self.dock_heading_offset = 0.0   # rad added to the dock yaw (pi if docked facing in)
        self.cog_stale = 1800.0          # s; older COG gives way to a dock seed
        self.imu_jump = math.radians(20.0)   # yaw step not explained by the gyro = IMU reset
        # yaw standard deviations advertised on /imu/data_aligned
        self.sigma_cog = math.radians(3.0)
        self.sigma_dock = math.radians(5.0)
        self.sigma_file = math.radians(45.0)
        self.sigma_none = 10.0           # rad: effectively "do not use"
        self.drift_rate = math.radians(1.0) / 60.0   # rad/s yaw drift growth after an update
        for k, v in kw.items():
            if not hasattr(self, k):
                raise AttributeError('unknown parameter %s' % k)
            setattr(self, k, v)


class _Window:
    __slots__ = ('t0', 'x0', 'y0', 'pts', 'imu', 'imu_first', 'imu_span', 'direction',
                 'fix_type')

    def __init__(self, t, x, y, direction, fix_type):
        self.t0, self.x0, self.y0 = t, x, y
        self.pts = [(x, y)]
        self.imu = CircularMean()
        self.imu_first = None
        self.imu_span = 0.0
        self.direction = direction
        self.fix_type = fix_type


class HeadingEstimator:
    """Feed it timestamped inputs (monotonic seconds); read :attr:`offset` / :meth:`status`."""

    def __init__(self, params=None):
        self.p = params or AlignParams()
        self.offset = None
        self.source = SOURCE_NONE
        self.updated_at = None       # time of the last offset update
        self.n_cog = 0
        self.last_cog = None         # (t, cog heading, measured offset, distance)
        self.last_event = ''
        self.rejected = []           # consecutive outlier measurements (offsets)
        self.dirty = False           # an update that should be persisted
        # latest inputs
        self.imu_yaw = None
        self.imu_t = None
        self.gyro_z = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_t = None
        self.fix_type = None
        self.fix_t = None
        self.docked = False
        self.docked_since = None
        self.dock_yaw = None
        self.dock_xy = None
        self._dock_mean = None
        self._win = None

    # ------------------------------------------------------------------ properties
    @property
    def aligned(self):
        return self.offset is not None and self.source in ALIGNED_SOURCES

    def yaw_sigma(self, now):
        p = self.p
        base = {SOURCE_COG: p.sigma_cog, SOURCE_DOCK: p.sigma_dock,
                SOURCE_FILE: p.sigma_file}.get(self.source, p.sigma_none)
        if self.offset is None or self.source == SOURCE_NONE:
            return p.sigma_none
        if self.updated_at is not None and self.source in ALIGNED_SOURCES:
            base += p.drift_rate * max(0.0, now - self.updated_at)
        return min(base, p.sigma_none)

    def quality(self, now):
        if not self.aligned:
            return 'low' if self.source == SOURCE_FILE else 'none'
        s = self.yaw_sigma(now)
        return 'good' if s <= math.radians(5.0) else 'fair' if s <= math.radians(15.0) \
            else 'poor'

    def status(self, now):
        d = {
            'aligned': self.aligned,
            'source': self.source,
            'offset_deg': None if self.offset is None else round(math.degrees(self.offset), 2),
            'quality': self.quality(now),
            'yaw_sigma_deg': round(math.degrees(self.yaw_sigma(now)), 2),
            'cog_updates': self.n_cog,
            'age_s': None if self.updated_at is None else round(now - self.updated_at, 1),
            'imu_yaw_deg': None if self.imu_yaw is None else round(math.degrees(self.imu_yaw), 2),
            'heading_deg': None if (self.imu_yaw is None or self.offset is None)
            else round(math.degrees(wrap(self.imu_yaw + self.offset)), 2),
            'docked': self.docked,
            'event': self.last_event,
        }
        if self.last_cog is not None:
            t, cog, meas, dist = self.last_cog
            d['last_cog_deg'] = round(math.degrees(cog), 2)
            d['last_cog_offset_deg'] = round(math.degrees(meas), 2)
            d['last_cog_distance_m'] = round(dist, 2)
            d['last_cog_age_s'] = round(now - t, 1)
        if self._win is not None:
            w = self._win
            d['window_m'] = round(math.hypot(w.pts[-1][0] - w.x0, w.pts[-1][1] - w.y0), 2)
        return d

    def apply(self, yaw_imu):
        """ENU yaw for an IMU yaw (raw yaw when there is no offset at all)."""
        return yaw_imu if self.offset is None else wrap(yaw_imu + self.offset)

    # ------------------------------------------------------------------ persistence
    def load_persisted(self, offset, now):
        """Boot seed from the file (never 'aligned')."""
        if self.offset is None:
            self.offset = wrap(offset)
            self.source = SOURCE_FILE
            self.updated_at = now
            self.last_event = 'loaded persisted offset (low quality until dock/COG)'

    # ------------------------------------------------------------------ inputs
    def on_imu(self, now, yaw, gyro_z, dt=None):
        p = self.p
        if self.imu_yaw is not None and self.imu_t is not None:
            dt = now - self.imu_t if dt is None else dt
            step = wrap(yaw - self.imu_yaw)
            if 0.0 < dt < 1.0 and abs(step) > p.imu_jump and \
                    abs(step) > 3.0 * abs(gyro_z) * dt + math.radians(2.0):
                self._imu_reset(now, step)
        self.imu_yaw, self.imu_t, self.gyro_z = yaw, now, gyro_z
        w = self._win
        if w is not None:
            if w.imu_first is None:
                w.imu_first = yaw
            w.imu.add(yaw)
            w.imu_span = max(w.imu_span, abs(wrap(yaw - w.imu_first)))
        if self._dock_mean is not None:
            self._dock_mean.add(yaw)

    def on_cmd(self, now, linear, angular):
        self.cmd, self.cmd_t = (linear, angular), now

    def on_fix_type(self, now, fix_type):
        self.fix_type, self.fix_t = fix_type, now

    def on_dock_pose(self, x, y, yaw):
        self.dock_xy = (x, y)
        self.dock_yaw = yaw

    def on_docked(self, now, docked):
        """Docked flag (is_charging or is_docking_done); seeds from the dock pose."""
        p = self.p
        if docked and not self.docked:
            self.docked_since = now
            self._dock_mean = CircularMean()
            if self.imu_yaw is not None:
                self._dock_mean.add(self.imu_yaw)
        elif not docked:
            self.docked_since = None
            self._dock_mean = None
        self.docked = docked
        if not docked or self.dock_yaw is None or self.imu_yaw is None:
            return
        if now - self.docked_since < p.dock_settle:
            return
        cog_fresh = (self.source == SOURCE_COG and self.updated_at is not None
                     and now - self.updated_at < p.cog_stale)
        if cog_fresh:
            return
        mean = self._dock_mean.mean() if self._dock_mean and self._dock_mean.n else self.imu_yaw
        new = wrap(self.dock_yaw + p.dock_heading_offset - mean)
        if self.source != SOURCE_DOCK:
            self.last_event = 'dock seed: offset %.1f deg (dock yaw %.1f, imu %.1f)' % (
                math.degrees(new), math.degrees(self.dock_yaw), math.degrees(mean))
        self.offset = new
        self.source = SOURCE_DOCK
        self.updated_at = now
        self.rejected = []

    def on_gps(self, now, x, y, variance):
        """GPS position in the map/ENU frame. Returns a measurement dict when a COG
        window completed (accepted or not), else None."""
        p = self.p
        ok, direction = self._straight_motion(now, variance)
        w = self._win
        if not ok:
            self._win = None
            return None
        if w is None or w.direction != direction or now - w.t0 > p.max_window_time:
            self._win = _Window(now, x, y, direction, self.fix_type)
            return None
        w.pts.append((x, y))
        w.fix_type = min(w.fix_type, self.fix_type)
        dx, dy = x - w.x0, y - w.y0
        dist = math.hypot(dx, dy)
        need = p.min_distance if w.fix_type >= 3 else p.float_min_distance
        if dist < need:
            return None
        # window complete: start the next one here (non-overlapping)
        self._win = _Window(now, x, y, direction, self.fix_type)
        return self._finish_window(now, w, dx, dy, dist)

    # ------------------------------------------------------------------ internals
    def _straight_motion(self, now, variance):
        p = self.p
        if self.cmd_t is None or now - self.cmd_t > p.cmd_timeout:
            return False, 0
        v, wz = self.cmd
        if abs(v) < p.min_speed or abs(wz) >= p.max_turn_rate:
            return False, 0
        if self.fix_type is None or self.fix_t is None or now - self.fix_t > p.gps_timeout \
                or self.fix_type < p.min_fix_type or self.fix_type > 3:
            return False, 0
        if variance is not None and p.max_position_variance > 0 and \
                variance > p.max_position_variance:
            return False, 0
        if self.imu_t is None or now - self.imu_t > p.imu_timeout:
            return False, 0
        return True, (1 if v > 0 else -1)

    def _finish_window(self, now, w, dx, dy, dist):
        p = self.p
        res = {'distance': dist, 'accepted': False}
        # straightness: max perpendicular distance of the track from the chord
        ux, uy = dx / dist, dy / dist
        lateral = max(abs((px - w.x0) * uy - (py - w.y0) * ux) for px, py in w.pts)
        res['lateral'] = lateral
        if lateral > p.max_lateral:
            res['reason'] = 'track not straight (%.2f m)' % lateral
            self.last_event = 'COG window rejected: ' + res['reason']
            return res
        if w.imu.n == 0:
            res['reason'] = 'no IMU samples'
            return res
        if w.imu_span > p.max_imu_yaw_change:
            res['reason'] = 'IMU yaw changed %.1f deg' % math.degrees(w.imu_span)
            self.last_event = 'COG window rejected: ' + res['reason']
            return res
        cog = math.atan2(dy, dx)
        if w.direction < 0:
            cog = wrap(cog + math.pi)
        imu_mean = w.imu.mean()
        meas = wrap(cog - imu_mean)
        res.update(cog=cog, imu_mean=imu_mean, measured=meas)
        self.last_cog = (now, cog, meas, dist)
        self._update_offset(now, meas, res)
        return res

    def _update_offset(self, now, meas, res):
        p = self.p
        if self.offset is None or self.source in (SOURCE_NONE, SOURCE_FILE):
            self._set_cog(now, meas, 'COG seed', res)
            return
        innov = wrap(meas - self.offset)
        res['innovation'] = innov
        if self.source == SOURCE_DOCK:
            if abs(innov) > p.outlier:
                self._set_cog(now, meas, 'COG replaces dock seed (%.1f deg off: check the dock '
                                         'pose yaw)' % math.degrees(innov), res)
            else:
                self._blend(now, innov, p.alpha_initial, 'COG refines dock seed', res)
            return
        # source == cog
        if abs(innov) > p.outlier:
            self.rejected.append(meas)
            res['reason'] = 'outlier %.1f deg' % math.degrees(innov)
            self.last_event = 'COG outlier rejected (%.1f deg)' % math.degrees(innov)
            if len(self.rejected) >= p.reseed_after:
                cm = CircularMean()
                for m in self.rejected:
                    cm.add(m)
                mean = cm.mean()
                if all(abs(wrap(m - mean)) <= p.reseed_agree for m in self.rejected):
                    self._set_cog(now, mean, 're-seeded from %d consistent COG outliers'
                                  % len(self.rejected), res)
                else:
                    self.rejected = self.rejected[1:]
            return
        alpha = p.alpha_initial if self.n_cog < p.initial_updates else p.alpha
        self._blend(now, innov, alpha, 'COG update', res)

    def _set_cog(self, now, meas, why, res):
        self.offset = wrap(meas)
        self._accepted(now, why, res)

    def _blend(self, now, innov, alpha, why, res):
        self.offset = wrap(self.offset + alpha * innov)
        self._accepted(now, '%s (alpha %.2f, innovation %.1f deg)'
                       % (why, alpha, math.degrees(innov)), res)

    def _accepted(self, now, why, res):
        self.source = SOURCE_COG
        self.updated_at = now
        self.n_cog += 1
        self.rejected = []
        self.dirty = True
        self.last_event = '%s: offset %.1f deg' % (why, math.degrees(self.offset))
        res['accepted'] = True
        res['offset'] = self.offset

    def _imu_reset(self, now, step):
        self.last_event = 'IMU yaw jumped %.1f deg without rotation: IMU reset, unaligned' \
            % math.degrees(step)
        self._win = None
        if self._dock_mean is not None:
            self._dock_mean = CircularMean()
            self.docked_since = now
        # the old offset is meaningless against the restarted yaw
        self.offset = None
        self.source = SOURCE_NONE
        self.updated_at = None
        self.rejected = []


# ---------------------------------------------------------------------- persistence
FILE_HEADER = '# mower_localization_heading_offset_v1'


def save_offset(path, offset, source, n_cog, now_wall=None):
    """Atomic write of the offset file (flat YAML)."""
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    stamp = time.strftime('%Y-%m-%dT%H:%M:%S%z', time.localtime(now_wall or time.time()))
    text = ('%s\noffset_rad: %.6f\noffset_deg: %.3f\nsource: %s\ncog_updates: %d\n'
            'saved_at: \'%s\'\n' % (FILE_HEADER, offset, math.degrees(offset), source, n_cog,
                                    stamp))
    fd, tmp = tempfile.mkstemp(prefix='.heading_offset.', dir=os.path.dirname(path))
    try:
        with os.fdopen(fd, 'w') as fh:
            fh.write(text)
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def load_offset(path):
    """Offset (rad) from the file, or None (missing / malformed)."""
    path = os.path.abspath(os.path.expanduser(path))
    try:
        with open(path) as fh:
            lines = fh.read().splitlines()
    except OSError:
        return None
    if not lines or lines[0].strip() != FILE_HEADER:
        return None
    for line in lines[1:]:
        key, _, value = line.partition(':')
        if key.strip() == 'offset_rad':
            try:
                v = float(value)
            except ValueError:
                return None
            return wrap(v) if math.isfinite(v) else None
    return None


def status_json(d):
    return json.dumps(d, sort_keys=True)


# ---------------------------------------------------------------------- fast IMU CDR
_ORIENT = struct.Struct('<4d')
_DOUBLE = struct.Struct('<d')


def imu_tail_offset(data):
    """Payload offset of Imu.orientation in serialized (XCDR1 LE) sensor_msgs/Imu, or None."""
    if len(data) < 16 or data[0] != 0 or data[1] != 1:
        return None
    (n,) = struct.unpack_from('<I', data, 12)
    pos = 16 + n                               # header: stamp 8 + string len 4 + chars
    pos = 4 + ((pos - 4 + 7) & -8)             # align 8 relative to the payload
    if pos + 8 * 37 > len(data):
        return None
    return pos


def imu_fields(data, pos):
    """(quaternion xyzw, gyro z) from serialized Imu at orientation offset ``pos``."""
    q = _ORIENT.unpack_from(data, pos)
    gz = _DOUBLE.unpack_from(data, pos + 8 * (4 + 9 + 2))[0]
    return q, gz


def imu_rewrite(data, pos, q, yaw_variance):
    """Copy of serialized Imu with orientation ``q`` and orientation_covariance[8]
    (yaw variance) replaced."""
    out = bytearray(data)
    _ORIENT.pack_into(out, pos, *q)
    _DOUBLE.pack_into(out, pos + 8 * 4 + 8 * 8, yaw_variance)
    return bytes(out)
