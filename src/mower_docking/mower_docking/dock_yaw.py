"""Dock yaw re-measurement from the forward undock (pure logic, no ROS).

The stored dock yaw (map_server ``dock_pose_yaw``) is only ever replaced by an operator edit
or a motion-derived heading (``SetDockingPoint`` ``yaw_source`` MOTION); a "Set docking point"
capture while docked PRESERVES it, because a single-antenna robot standing still has no
heading of its own (its EKF yaw on the dock is not a measurement: it read 113.8 deg on
2026-10-21 whether parked crooked or straight).

The forward undock (``/mower_docking/undock``, 0.8 m straight on a heading hold) is such a
motion: under RTK the fused track is a straight chord along the docked heading. The robot
leaves the dock along its own heading, so

    dock yaw = direction of the fitted track line - (mean odom yaw along it - odom yaw at the
               start of the undock)

(the second term removes any turn the heading hold let through). The docked point itself is
NOT used: while charging the fused pose is gauge-reset onto the stored dock pose, so it is
circular; the fit starts ``skip_m`` along the track.

Quality gates (all must hold, else no measurement): RTK fixed on every sample, >= ``min_n``
samples spanning >= ``min_span_m``, odom yaw change <= ``max_turn_deg`` over the fitted
part, RMS distance of the samples from the fitted line <= ``max_rms_m``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


from mower_common.angles import wrap_angle_trig as _wrap  # noqa: E402


@dataclass
class ChordParams:
    skip_m: float = 0.15
    min_span_m: float = 0.45
    min_n: int = 10
    max_turn_deg: float = 3.0
    max_rms_m: float = 0.02
    max_change_deg: float = 20.0     # a bigger change than this is not applied automatically


@dataclass
class ChordResult:
    ok: bool
    yaw: float = float('nan')        # rad, ENU map frame: the docked chassis heading
    span_m: float = 0.0
    n: int = 0
    rms_m: float = float('nan')
    turn_deg: float = float('nan')
    reason: str = ''

    def describe(self) -> str:
        if not self.ok:
            return 'no dock yaw measurement: %s' % self.reason
        return ('dock yaw %.1f deg from the undock chord (%d samples over %.2f m, RMS %.3f m, '
                'turn %.1f deg)' % (math.degrees(self.yaw), self.n, self.span_m, self.rms_m,
                                    self.turn_deg))


def undock_chord_yaw(samples: Sequence[Tuple[float, float, float, bool]], odom_yaw0: float,
                     p: Optional[ChordParams] = None) -> ChordResult:
    """``samples``: (map x, map y, odom yaw, rtk_fixed) along the forward undock, in order;
    ``odom_yaw0``: the odom yaw when the undock started (on the dock)."""
    p = p or ChordParams()
    if not samples:
        return ChordResult(False, reason='no samples')
    if not all(s[3] for s in samples):
        return ChordResult(False, reason='RTK not fixed during the undock')
    x0, y0 = samples[0][0], samples[0][1]
    pts: List[Tuple[float, float, float]] = [
        (x, y, yaw) for x, y, yaw, _ in samples if math.hypot(x - x0, y - y0) >= p.skip_m]
    if len(pts) < p.min_n:
        return ChordResult(False, n=len(pts), reason='only %d samples past %.2f m (need %d)'
                           % (len(pts), p.skip_m, p.min_n))
    mx = sum(q[0] for q in pts) / len(pts)
    my = sum(q[1] for q in pts) / len(pts)
    sxx = sum((q[0] - mx) ** 2 for q in pts)
    syy = sum((q[1] - my) ** 2 for q in pts)
    sxy = sum((q[0] - mx) * (q[1] - my) for q in pts)
    ang = 0.5 * math.atan2(2.0 * sxy, sxx - syy)          # principal axis (mod pi)
    # orient it along the motion (first -> last sample)
    dx, dy = pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]
    if math.cos(ang) * dx + math.sin(ang) * dy < 0:
        ang = _wrap(ang + math.pi)
    c, s = math.cos(ang), math.sin(ang)
    along = [c * (q[0] - mx) + s * (q[1] - my) for q in pts]
    perp = [-s * (q[0] - mx) + c * (q[1] - my) for q in pts]
    span = max(along) - min(along)
    rms = math.sqrt(sum(d * d for d in perp) / len(perp))
    yaws = [_wrap(q[2] - odom_yaw0) for q in pts]
    turn = max(abs(y) for y in yaws)
    mean_turn = math.atan2(sum(math.sin(y) for y in yaws), sum(math.cos(y) for y in yaws))
    res = ChordResult(True, _wrap(ang - mean_turn), span, len(pts), rms, math.degrees(turn))
    if span < p.min_span_m:
        res.ok, res.reason = False, 'track span %.2f m < %.2f m' % (span, p.min_span_m)
    elif math.degrees(turn) > p.max_turn_deg:
        res.ok, res.reason = False, 'robot turned %.1f deg during the undock (max %.1f)' % (
            math.degrees(turn), p.max_turn_deg)
    elif rms > p.max_rms_m:
        res.ok, res.reason = False, 'track not straight (RMS %.3f m > %.3f m)' % (rms, p.max_rms_m)
    return res


def apply_decision(measured: ChordResult, stored_yaw: Optional[float],
                   p: Optional[ChordParams] = None) -> Tuple[bool, str]:
    """Whether a measurement may replace the stored dock yaw (automatic or on request)."""
    p = p or ChordParams()
    if not measured.ok:
        return False, measured.reason
    if stored_yaw is None:
        return True, 'no stored dock yaw'
    d = math.degrees(_wrap(measured.yaw - stored_yaw))
    if abs(d) > p.max_change_deg:
        return False, ('measured %.1f deg differs from the stored %.1f deg by %.1f deg (> %.0f): '
                       'not applied; check the dock / RTK, re-measure' % (
                           math.degrees(measured.yaw), math.degrees(stored_yaw), d,
                           p.max_change_deg))
    return True, 'stored %.1f deg -> measured %.1f deg (%+.1f deg)' % (
        math.degrees(stored_yaw), math.degrees(measured.yaw), d)
