# SPDX-License-Identifier: GPL-3.0-or-later
"""RTK solution class and fix-quality-aware GNSS covariance (pure logic, no ROS imports).

2026-10-09 (localization item 6): the EKF must lean on wheel + IMU (+ VIO) during RTK
float instead of being dragged around by decimetre-noisy float fixes, and must never
see a single-point fix as if it were centimetre-accurate. NavSatFix cannot say which
RTK solution a fix is: um960_gps_driver maps GGA 4 and 5 to GBAS_FIX, and its BESTNAV
path reports every non-integer type (float included) as STATUS_FIX. The solution class
therefore comes from ``/fix_status`` (``solution=<BESTNAV type>`` or ``quality=<GGA
label>``, the same tokens mower_gui_bridge.state_machine.classify_solution reads), with
the NavSatStatus only as a conservative fallback when /fix_status is missing or stale.

:func:`fused_variance` turns (class, receiver variance) into the horizontal variance the
filter is given: never smaller than the receiver says, scaled up and floored per class.
The UM960 sigma in float is optimistic (0.05-0.15 m reported while the solution wanders
0.2-0.5 m), so float gets x4 and a 0.4 m sigma floor; fixed keeps the receiver sigma
with a 3 cm floor; DGPS/single are rejected by default (``single_policy``).
"""

from dataclasses import dataclass
from typing import Optional

RTK_FIXED = 'fixed'
RTK_FLOAT = 'float'
RTK_DGPS = 'dgps'
RTK_SINGLE = 'single'
RTK_NONE = 'none'       # receiver says no solution
RTK_STALE = 'stale'     # no /fix_status recently (receiver or driver silent)
RTK_CLASSES = (RTK_FIXED, RTK_FLOAT, RTK_DGPS, RTK_SINGLE, RTK_NONE, RTK_STALE)

# Same vocabulary as mower_gui_bridge.state_machine (BESTNAV position types + GGA labels).
_FIXED = frozenset({'RTK_FIXED', 'FIXEDPOS', 'L1_INT', 'WIDE_INT', 'NARROW_INT',
                    'INS_RTKFIXED'})
_FLOAT = frozenset({'RTK_FLOAT', 'L1_FLOAT', 'IONOFREE_FLOAT', 'NARROW_FLOAT',
                    'INS_RTKFLOAT'})
_DGPS = frozenset({'DGPS', 'PSRDIFF', 'SBAS', 'INS_PSRDIFF'})
_SINGLE = frozenset({'GPS', 'SINGLE', 'FIXEDHEIGHT', 'DOPPLER_VELOCITY', 'INS',
                     'INS_PSRSP', 'PPP', 'PPP_CONVERGING', 'DR', 'FIX'})
_NONE = frozenset({'INVALID', 'NONE'})

# sensor_msgs/NavSatStatus (Humble): -1 NO_FIX, 0 FIX, 1 SBAS_FIX, 2 GBAS_FIX
NAVSAT_NO_FIX, NAVSAT_FIX, NAVSAT_SBAS, NAVSAT_GBAS = -1, 0, 1, 2


def _solution_token(text):
    """Upper-case value of ``solution=`` (preferred, BESTNAV) or ``quality=``, or None."""
    sol = qual = None
    for tok in text.split():
        key, sep, value = tok.partition('=')
        if not sep:
            continue
        if key == 'solution' and sol is None:
            sol = value.upper()
        elif key == 'quality' and qual is None:
            qual = value.upper()
    return sol or qual


def classify_fix_status(text):
    """RTK class of a ``/fix_status`` line, or None when it names no solution.

    A ``STALE ...`` line (driver: no fix for a while) is :data:`RTK_STALE`.
    """
    if not text:
        return None
    if text.startswith('STALE'):
        return RTK_STALE
    tok = _solution_token(text)
    if not tok:
        return None
    if tok in _FIXED:
        return RTK_FIXED
    if tok in _FLOAT:
        return RTK_FLOAT
    if tok in _DGPS:
        return RTK_DGPS
    if tok in _NONE:
        return RTK_NONE
    if tok in _SINGLE:
        return RTK_SINGLE
    # unforeseen labels: same fall-backs as the GUI classifier
    if tok.endswith('_INT') or 'FIXED' in tok:
        return RTK_FIXED
    if 'FLOAT' in tok:
        return RTK_FLOAT
    return RTK_SINGLE


def class_from_navsat(status):
    """Conservative class from NavSatStatus alone (/fix_status missing or stale):
    GBAS is fixed OR float -> float; SBAS -> dgps; FIX -> single."""
    if status is None or status <= NAVSAT_NO_FIX:
        return RTK_NONE
    if status >= NAVSAT_GBAS:
        return RTK_FLOAT
    if status == NAVSAT_SBAS:
        return RTK_DGPS
    return RTK_SINGLE


@dataclass
class CovariancePolicy:
    """Per-class horizontal sigma floor (m) and variance scale; see the module docstring."""
    fixed_sigma_floor: float = 0.03     # m; UM960 NARROW_INT reports 0.01-0.02
    fixed_scale: float = 1.0
    float_sigma_floor: float = 0.4      # m; float wanders 0.2-0.5 m on this site class
    float_scale: float = 4.0            # (2x sigma) the receiver's float sigma is optimistic
    single_policy: str = 'reject'       # reject | loose  (DGPS + single-point)
    single_sigma_floor: float = 3.0     # m, only with single_policy loose
    single_scale: float = 4.0
    unknown_variance: float = 25.0      # m^2 when the receiver reported no covariance

    def floor_scale(self, rtk_class):
        if rtk_class == RTK_FIXED:
            return self.fixed_sigma_floor, self.fixed_scale
        if rtk_class == RTK_FLOAT:
            return self.float_sigma_floor, self.float_scale
        return self.single_sigma_floor, self.single_scale


def fused_variance(rtk_class, receiver_variance, policy=None):
    """Horizontal variance (m^2) to hand the filter for one fix, or None = drop the fix.

    ``receiver_variance``: the larger of the receiver's E/N variances, or None when the
    NavSatFix carries COVARIANCE_TYPE_UNKNOWN. The result is monotone: never below the
    receiver's own variance, never below the class floor.
    """
    p = policy or CovariancePolicy()
    if rtk_class in (RTK_NONE, RTK_STALE, None):
        return None
    if rtk_class in (RTK_DGPS, RTK_SINGLE) and str(p.single_policy).lower() != 'loose':
        return None
    floor, scale = p.floor_scale(rtk_class)
    if receiver_variance is None or receiver_variance <= 0.0:
        rx = p.unknown_variance if rtk_class not in (RTK_FIXED, RTK_FLOAT) else floor * floor
    else:
        rx = receiver_variance * max(1.0, scale)
    return max(rx, floor * floor)
