# SPDX-License-Identifier: GPL-3.0-or-later
"""Magnetometer heading for the LIS2MDL / IIS2MDC on the Metoak stereo module (pure, numpy).

Experiment (2026-11-03): can the magnetometer seed an absolute heading after a reboot off
the dock, so the mission does not need a blind calibration drive? See
docs/HEADING_RECEIVER_2026-11.md for the hardware alternative (dual-antenna GNSS).

Frames and signs
----------------
* ``mag`` and ``accel`` are base_link vectors (x forward, y left, z up; REP-103). The node
  rotates the chip axes into base_link with its ``*_to_base_rpy_deg`` params first.
* ``accel`` is the specific force (REP-145: +9.8 m/s^2 on z when level and at rest), so its
  direction is "up" in the body frame.
* Magnetic bearing: clockwise from magnetic north of the base_link +x axis. The horizontal
  field points to magnetic north, so a robot facing north sees it on +x and a robot facing
  east sees it on +y (north is on its left): ``bearing = atan2(h . left, h . forward)``.
* Map heading: ENU yaw (east = 0, CCW+) in the UTM-grid map frame, like
  ``heading_logic.rtk_heading_enu``: ``yaw = pi/2 - (bearing + declination - convergence)``
  (declination east-positive, convergence = UTM meridian convergence at the robot).

Calibration (``MagCalibration``, yaml): ``m_cal = soft_iron @ (m_raw - hard_iron)`` in
base_link, microtesla. A ground robot only turns about z, so a mow constrains the
horizontal hard / soft iron (2-D ellipse fit) but not the z offset: z comes from a 3-D
ellipsoid fit when the samples span enough tilt (the module tipped on purpose), else from the
site's vertical field (``vertical_ut``, WMM "Z", down positive) when given, else it stays 0
and ``z_observed`` is false (the tilt compensation then carries ``|bz| * sin(tilt)`` error).
"""
from __future__ import annotations

import math

import numpy as np

UT_PER_GAUSS = 100.0
TESLA_PER_UT = 1e-6
LIS2MDL_UT_PER_LSB = 0.15        # 1.5 mgauss / LSB (datasheet), both LIS2MDL and IIS2MDC
G = 9.80665


def wrap(a):
    """(-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def wrap_deg(a):
    return math.degrees(wrap(math.radians(a)))


def rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis rpy (rad) -> R (child -> parent): Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def rpy_deg_matrix(rpy_deg):
    r, p, y = (math.radians(float(v)) for v in rpy_deg)
    return rpy_matrix(r, p, y)


def _up(accel):
    if accel is None:
        return np.array([0.0, 0.0, 1.0]), False
    a = np.asarray(accel, dtype=float)
    n = float(np.linalg.norm(a))
    if not math.isfinite(n) or n < 1e-6:
        return np.array([0.0, 0.0, 1.0]), False
    return a / n, True


def level_field(mag, accel=None):
    """Tilt compensation. -> dict(bearing rad, tilt rad, horizontal |h|, inclination rad
    (positive = field points below the horizon), tilt_known).

    The field is projected onto the plane normal to gravity (``accel`` direction; level if
    None) and its angle is measured against the base_link x axis projected onto that plane,
    which is the yaw of the robot whatever its roll / pitch."""
    m = np.asarray(mag, dtype=float)
    u, known = _up(accel)
    mv = float(m @ u)
    mh = m - mv * u
    f = np.array([1.0, 0.0, 0.0]) - u[0] * u
    fn = float(np.linalg.norm(f))
    if fn < 1e-6:                      # standing on its nose: yaw undefined
        f = np.array([0.0, 0.0, -1.0]) - (-u[2]) * u
        fn = float(np.linalg.norm(f))
    f /= fn
    left = np.cross(u, f)
    h = float(np.linalg.norm(mh))
    return {'bearing': math.atan2(float(mh @ left), float(mh @ f)),
            'tilt': math.acos(max(-1.0, min(1.0, float(u[2])))),
            'horizontal': h, 'inclination': math.atan2(-mv, h), 'tilt_known': known}


def enu_yaw(bearing, declination=0.0, convergence=0.0):
    """Magnetic bearing (rad, CW from magnetic north) -> map-frame ENU yaw (rad)."""
    return wrap(math.pi / 2.0 - (bearing + declination - convergence))


def utm_grid_convergence(lat_deg, lon_deg):
    """UTM meridian convergence (rad); same formula as heading_logic.utm_grid_convergence_deg
    (kept local: mower_cameras does not depend on mower_localization)."""
    zone = int(math.floor((lon_deg + 180.0) / 6.0)) % 60 + 1
    lon0 = zone * 6.0 - 183.0
    return math.atan(math.tan(math.radians(lon_deg - lon0)) * math.sin(math.radians(lat_deg)))


# ---------------------------------------------------------------------- calibration
class MagCalibration:
    """Hard / soft iron in base_link (uT) plus the reference field the quality compares to.

    ``yaw_offset_deg``: residual heading bias measured against the COG heading by
    scripts/mag_cal.py (mount yaw + anything the declination param did not cover), added to
    the bearing. It is relative to the declination used for that evaluation: keep that
    declination on the node."""

    KEYS = ('calibrated', 'fit', 'z_observed', 'hard_iron_ut', 'soft_iron', 'field_ut',
            'horizontal_ut', 'inclination_deg', 'yaw_offset_deg', 'mag_to_base_rpy_deg',
            'samples')

    def __init__(self, hard_iron=(0.0, 0.0, 0.0), soft_iron=None, field_ut=0.0,
                 horizontal_ut=0.0, inclination_deg=0.0, calibrated=False, fit='none',
                 z_observed=False, yaw_offset_deg=0.0, mag_to_base_rpy_deg=None, samples=0,
                 extra=None):
        self.hard_iron = np.asarray(hard_iron, dtype=float).reshape(3)
        self.soft_iron = (np.eye(3) if soft_iron is None
                          else np.asarray(soft_iron, dtype=float).reshape(3, 3))
        self.field_ut = float(field_ut)
        self.horizontal_ut = float(horizontal_ut)
        self.inclination_deg = float(inclination_deg)
        self.calibrated = bool(calibrated)
        self.fit = str(fit)
        self.z_observed = bool(z_observed)
        self.yaw_offset_deg = float(yaw_offset_deg)
        self.mag_to_base_rpy_deg = (None if mag_to_base_rpy_deg is None
                                    else [float(v) for v in mag_to_base_rpy_deg])
        self.samples = int(samples)
        self.extra = dict(extra or {})

    def apply(self, m):
        return self.soft_iron @ (np.asarray(m, dtype=float) - self.hard_iron)

    def to_dict(self):
        d = {'calibrated': self.calibrated, 'fit': self.fit, 'z_observed': self.z_observed,
             'hard_iron_ut': [round(float(v), 4) for v in self.hard_iron],
             'soft_iron': [round(float(v), 6) for v in self.soft_iron.reshape(9)],
             'field_ut': round(self.field_ut, 4), 'horizontal_ut': round(self.horizontal_ut, 4),
             'inclination_deg': round(self.inclination_deg, 3),
             'yaw_offset_deg': round(self.yaw_offset_deg, 3),
             'mag_to_base_rpy_deg': self.mag_to_base_rpy_deg, 'samples': self.samples}
        d.update(self.extra)
        return d

    @classmethod
    def from_dict(cls, d):
        d = dict(d or {})
        extra = {k: v for k, v in d.items() if k not in cls.KEYS}
        return cls(hard_iron=d.get('hard_iron_ut', (0.0, 0.0, 0.0)),
                   soft_iron=d.get('soft_iron'), field_ut=d.get('field_ut', 0.0) or 0.0,
                   horizontal_ut=d.get('horizontal_ut', 0.0) or 0.0,
                   inclination_deg=d.get('inclination_deg', 0.0) or 0.0,
                   calibrated=d.get('calibrated', False), fit=d.get('fit', 'none'),
                   z_observed=d.get('z_observed', False),
                   yaw_offset_deg=d.get('yaw_offset_deg', 0.0) or 0.0,
                   mag_to_base_rpy_deg=d.get('mag_to_base_rpy_deg'),
                   samples=d.get('samples', 0) or 0, extra=extra)

    @classmethod
    def load(cls, path):
        """-> MagCalibration (uncalibrated identity when the file is missing / malformed)."""
        try:
            import yaml
            with open(path) as fh:
                d = yaml.safe_load(fh)
            if not isinstance(d, dict):
                return cls()
            return cls.from_dict(d)
        except (OSError, ValueError, TypeError, ImportError):
            return cls()
        except Exception:  # noqa: BLE001 - yaml.YAMLError without importing yaml at top
            return cls()

    def to_yaml(self, header=''):
        lines = ['# ' + h if h else '#' for h in header.splitlines()] if header else []
        for k, v in self.to_dict().items():
            if isinstance(v, bool):
                txt = 'true' if v else 'false'
            elif v is None:
                txt = 'null'
            elif isinstance(v, (list, tuple)):
                txt = '[' + ', '.join(repr(float(x)) if isinstance(x, float) else str(x)
                                      for x in v) + ']'
            elif isinstance(v, str):
                txt = v
            else:
                txt = repr(v)
            lines.append('%s: %s' % (k, txt))
        return '\n'.join(lines) + '\n'


def _sphere_map(a3, k):
    """Quadric ``(x-c)^T A (x-c) = k`` -> (W, radius): W maps the ellipsoid to a sphere of the
    geometric-mean radius (the field scale is kept)."""
    m = a3 / k
    w, v = np.linalg.eigh(m)
    if (w <= 0).any():
        raise ValueError('fit is not an ellipsoid (eigenvalues %s)' % w)
    axes = 1.0 / np.sqrt(w)
    r = float(np.prod(axes) ** (1.0 / len(axes)))
    return r * (v @ np.diag(np.sqrt(w)) @ v.T), r


def fit_ellipsoid(m):
    """Algebraic 9-parameter ellipsoid fit of Nx3 samples -> (center, W, radius).

    ``a x^2 + b y^2 + c z^2 + 2d xy + 2e xz + 2f yz + 2g x + 2h y + 2i z = 1``; centring the
    data first keeps the least squares well scaled."""
    m = np.asarray(m, dtype=float)
    mu = m.mean(axis=0)
    x, y, z = (m - mu).T
    dmat = np.column_stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z,
                            2 * x, 2 * y, 2 * z])
    p, *_ = np.linalg.lstsq(dmat, np.ones(len(m)), rcond=None)
    a3 = np.array([[p[0], p[3], p[4]], [p[3], p[1], p[5]], [p[4], p[5], p[2]]])
    g = p[6:9]
    c = -np.linalg.solve(a3, g)
    k = 1.0 + float(c @ a3 @ c)
    w, r = _sphere_map(a3, k)
    return c + mu, w, r


def fit_ellipse_2d(xy):
    """Algebraic conic fit ``a x^2 + b xy + c y^2 + d x + e y = 1`` -> (center, W 2x2, r)."""
    xy = np.asarray(xy, dtype=float)
    mu = xy.mean(axis=0)
    x, y = (xy - mu).T
    dmat = np.column_stack([x * x, 2 * x * y, y * y, 2 * x, 2 * y])
    p, *_ = np.linalg.lstsq(dmat, np.ones(len(xy)), rcond=None)
    a2 = np.array([[p[0], p[1]], [p[1], p[2]]])
    c = -np.linalg.solve(a2, p[3:5])
    k = 1.0 + float(c @ a2 @ c)
    w, r = _sphere_map(a2, k)
    return c + mu, w, r


def spread_ratio(m):
    """Smallest / largest principal std of the samples (0 = planar cloud)."""
    m = np.asarray(m, dtype=float)
    s = np.linalg.svd(m - m.mean(axis=0), compute_uv=False)
    return float(s[-1] / s[0]) if s[0] > 0 else 0.0


def calibrate(mag, accel=None, vertical_ut=None, min_spread=0.25, max_level_tilt_deg=5.0,
              min_samples=200, mag_to_base_rpy_deg=None):
    """Hard / soft iron from base_link samples (uT). -> MagCalibration (calibrated=True).

    3-D ellipsoid fit when the cloud is not planar (``spread_ratio >= min_spread``), else a
    2-D ellipse fit on the near-level samples (tilt <= ``max_level_tilt_deg`` when ``accel``
    is given) with the z offset from ``vertical_ut`` (site vertical field, WMM Z, down
    positive) or 0. The reference field / horizontal / inclination are the medians over the
    calibrated samples (what :func:`quality` compares against)."""
    mag = np.asarray(mag, dtype=float)
    if len(mag) < min_samples:
        raise ValueError('need >= %d samples, got %d' % (min_samples, len(mag)))
    acc = None if accel is None else np.asarray(accel, dtype=float)
    ratio = spread_ratio(mag)
    if ratio >= min_spread:
        c, w, _r = fit_ellipsoid(mag)
        cal = MagCalibration(hard_iron=c, soft_iron=w, calibrated=True, fit='3d',
                             z_observed=True)
    else:
        if acc is not None:
            ups = acc / np.linalg.norm(acc, axis=1, keepdims=True)
            level = np.degrees(np.arccos(np.clip(ups[:, 2], -1, 1))) <= max_level_tilt_deg
        else:
            ups = np.tile([0.0, 0.0, 1.0], (len(mag), 1))
            level = np.ones(len(mag), bool)
        if level.sum() < min_samples:
            raise ValueError('only %d near-level samples' % int(level.sum()))
        c2, w2, _r = fit_ellipse_2d(mag[level, :2])
        bz, z_obs = 0.0, False
        if vertical_ut is not None:
            # (m - b) . u = -Z  (body z up, WMM Z down positive), solved for bz on level samples
            mx = mag[level]
            u = ups[level]
            num = (mx[:, 0] - c2[0]) * u[:, 0] + (mx[:, 1] - c2[1]) * u[:, 1] \
                + mx[:, 2] * u[:, 2] + float(vertical_ut)
            bz, z_obs = float(np.median(num / u[:, 2])), True
        w = np.eye(3)
        w[:2, :2] = w2
        cal = MagCalibration(hard_iron=(c2[0], c2[1], bz), soft_iron=w, calibrated=True,
                             fit='2d', z_observed=z_obs)
    cal.samples = len(mag)
    cal.mag_to_base_rpy_deg = mag_to_base_rpy_deg
    cal.extra['spread_ratio'] = round(ratio, 4)
    fields, horiz, incl = [], [], []
    for i in range(len(mag)):
        mc = cal.apply(mag[i])
        lf = level_field(mc, None if acc is None else acc[i])
        fields.append(float(np.linalg.norm(mc)))
        horiz.append(lf['horizontal'])
        incl.append(lf['inclination'])
    cal.field_ut = float(np.median(fields))
    cal.horizontal_ut = float(np.median(horiz))
    cal.inclination_deg = float(np.degrees(np.median(incl)))
    return cal


# ---------------------------------------------------------------------- heading + quality
class QualityParams:
    def __init__(self, **kw):
        self.max_field_dev = 0.20       # |F - F0| / F0 (or |H - H0| / H0) at quality 0
        self.max_dip_dev_deg = 15.0     # |inclination - I0| at quality 0
        self.max_tilt_deg = 30.0        # tilt at quality 0
        self.max_accel_dev = 2.0        # m/s^2 | |a| - g | at quality 0 (tilt unreliable)
        self.no_accel_cap = 0.5         # quality cap without an accelerometer (tilt unknown)
        self.z_unobserved_cap = 0.8     # quality cap with an unobserved z offset
        for k, v in kw.items():
            if not hasattr(self, k):
                raise AttributeError('unknown parameter %s' % k)
            setattr(self, k, float(v))


def _ramp(dev, full):
    return max(0.0, min(1.0, 1.0 - dev / full)) if full > 0 else 1.0


def heading(mag, accel, cal, declination=0.0, convergence=0.0, qp=None):
    """One base_link sample (uT, m/s^2) -> dict(yaw (ENU map, rad), bearing, quality 0..1,
    field, horizontal, inclination, tilt and the quality terms).

    Quality is the minimum of: field magnitude and horizontal magnitude deviation from the
    calibration reference (interference, a steel object, the dock coil), inclination (dip)
    deviation (a distorted field keeps its magnitude more often than its direction), tilt,
    and the accelerometer's deviation from 1 g (accelerating: the tilt estimate is off). An
    uncalibrated sensor has quality 0 (the heading is still reported)."""
    qp = qp or QualityParams()
    mc = cal.apply(mag)
    lf = level_field(mc, accel)
    bearing = lf['bearing'] + math.radians(cal.yaw_offset_deg)
    f = float(np.linalg.norm(mc))
    out = {'yaw': enu_yaw(bearing, declination, convergence), 'bearing': wrap(bearing),
           'field': f, 'horizontal': lf['horizontal'],
           'inclination': lf['inclination'], 'tilt': lf['tilt']}
    if not cal.calibrated or cal.field_ut <= 0 or cal.horizontal_ut <= 0:
        out.update(quality=0.0, q_terms={'calibrated': 0.0})
        return out
    terms = {
        'field': _ramp(abs(f - cal.field_ut) / cal.field_ut, qp.max_field_dev),
        'horizontal': _ramp(abs(lf['horizontal'] - cal.horizontal_ut) / cal.horizontal_ut,
                            qp.max_field_dev),
        'dip': _ramp(abs(math.degrees(lf['inclination']) - cal.inclination_deg),
                     qp.max_dip_dev_deg),
        'tilt': _ramp(math.degrees(lf['tilt']), qp.max_tilt_deg),
    }
    if lf['tilt_known']:
        terms['accel'] = _ramp(abs(float(np.linalg.norm(accel)) - G), qp.max_accel_dev)
    else:
        terms['accel'] = qp.no_accel_cap
    if not cal.z_observed:
        terms['z'] = qp.z_unobserved_cap
    out.update(quality=float(min(terms.values())), q_terms=terms)
    return out


# ---------------------------------------------------------------------- evaluation
def circ_mean_deg(a):
    a = np.radians(np.asarray(a, dtype=float))
    return float(np.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean()))) if len(a) else 0.0


def error_stats(err_deg):
    """Heading errors (deg, any wrap) -> dict(n, bias, std, p95 (|err - bias|),
    p95_raw (|err|), max)."""
    e = np.asarray([wrap_deg(x) for x in err_deg], dtype=float)
    if not len(e):
        return {'n': 0, 'bias': None, 'std': None, 'p95': None, 'p95_raw': None, 'max': None}
    b = circ_mean_deg(e)
    r = np.array([wrap_deg(x - b) for x in e])
    return {'n': int(len(e)), 'bias': round(b, 2), 'std': round(float(r.std()), 2),
            'p95': round(float(np.percentile(np.abs(r), 95)), 2),
            'p95_raw': round(float(np.percentile(np.abs(e), 95)), 2),
            'max': round(float(np.abs(r).max()), 2)}


def mirrored(mag_yaw_deg, ref_yaw_deg):
    """True when the magnetometer heading turns the wrong way (an axis sign in the mount
    rotation is flipped): (mag + ref) is steadier than (mag - ref)."""
    a = np.asarray(mag_yaw_deg, dtype=float)
    b = np.asarray(ref_yaw_deg, dtype=float)
    if len(a) < 10:
        return False
    d = error_stats(a - b)['std']
    s = error_stats(a + b)['std']
    return s < 0.5 * d


SEED_P95_DEG = 10.0
WATCHDOG_P95_DEG = 25.0


def verdict(p95):
    """p95 heading error (deg, bias removed) -> 'seed' | 'watchdog' | 'not usable'."""
    if p95 is None:
        return 'not usable'
    if p95 < SEED_P95_DEG:
        return 'seed'
    if p95 < WATCHDOG_P95_DEG:
        return 'watchdog'
    return 'not usable'
