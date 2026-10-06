"""Pure decision logic for ``vio_gate`` (no ROS imports, unit-tested on the host).

The gate forwards visual-inertial odometry (``/odometry/vio``, produced by
``stereo_vio_bridge/vio_odom_bridge`` from OpenVINS) to the EKF only when

1. RTK is NOT fixed (RTK FIX already pins x/y to centimetres; VIO would only add
   noise), debounced by ``rtk_holdoff_s`` so a single FLOAT epoch does not toggle it,
2. VIO is initialised and has produced ``warmup_msgs`` consecutive healthy messages
   (no gap longer than ``max_gap_s``),
3. the VIO body velocity is plausible (finite, below ``max_speed``, covariance below
   ``max_twist_var``), and
4. it agrees with wheel odometry: if |vx_vio - vx_wheel| exceeds
   ``max_wheel_disagreement`` for ``disagreement_hold_s`` the VIO is declared diverged
   and stays blocked until it has agreed again for ``recover_s``.

All times are seconds on one monotonic clock supplied by the caller.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# Tokens in um960_gps_driver's /fix_status that mean RTK FIXED. NavSatFix cannot carry
# it: GGA quality 4 (fixed) and 5 (float) both map to STATUS_GBAS_FIX / STATUS_FIX.
# The string is "quality=<GGA label>" (NMEA) or "solution=<BESTPOS type>" (Unicore).
RTK_FIXED_TOKENS = (
    "quality=RTK_FIXED",
    "solution=NARROW_INT",
    "solution=WIDE_INT",
    "solution=L1_INT",
    "solution=INS_RTKFIXED",
)


def rtk_is_fixed(status_text: str) -> bool:
    """True iff a /fix_status line reports an RTK fixed (integer) solution."""
    if not status_text or status_text.startswith("STALE"):
        return False
    tokens = status_text.split()
    return any(tok in tokens for tok in RTK_FIXED_TOKENS)


@dataclass
class VioGateParams:
    rtk_holdoff_s: float = 1.0          # RTK must be non-FIX this long before VIO passes
    fix_status_timeout_s: float = 2.0   # no /fix_status this long -> RTK treated as lost
    warmup_msgs: int = 20               # healthy msgs after (re)init before passing
    max_gap_s: float = 0.5              # VIO gap that resets the warm-up
    max_speed: float = 1.2              # m/s, mower tops out ~0.6
    max_twist_var: float = 0.25         # (m/s)^2 on vx/vy, i.e. sigma 0.5 m/s
    max_wheel_disagreement: float = 0.4  # m/s |vx_vio - vx_wheel|
    disagreement_hold_s: float = 1.0    # sustained disagreement -> diverged
    recover_s: float = 3.0              # sustained agreement to clear "diverged"
    wheel_timeout_s: float = 0.5        # wheel odom older than this is not compared


class VioGateLogic:
    """State machine behind the gate. Feed it events, ask ``on_vio`` for a verdict."""

    def __init__(self, params: Optional[VioGateParams] = None) -> None:
        self.p = params or VioGateParams()
        self.rtk_fixed = False
        self._last_status_t: Optional[float] = None
        self._rtk_lost_since: Optional[float] = 0.0  # no status yet == lost since t=0
        self._wheel_vx: Optional[float] = None
        self._wheel_t: Optional[float] = None
        self._last_vio_t: Optional[float] = None
        self.healthy_streak = 0
        self.diverged = False
        self._disagree_since: Optional[float] = None
        self._agree_since: Optional[float] = None

    # -- inputs ---------------------------------------------------------------
    def on_fix_status(self, text: str, now: float) -> None:
        fixed = rtk_is_fixed(text)
        self._last_status_t = now
        if fixed:
            self._rtk_lost_since = None
        elif self.rtk_fixed or self._rtk_lost_since is None:
            self._rtk_lost_since = now
        self.rtk_fixed = fixed

    def on_wheel(self, vx: float, now: float) -> None:
        self._wheel_vx = vx
        self._wheel_t = now

    def reset(self) -> None:
        """VIO restarted (or re-initialised): start the warm-up over."""
        self.healthy_streak = 0
        self.diverged = False
        self._disagree_since = None
        self._agree_since = None
        self._last_vio_t = None

    # -- queries --------------------------------------------------------------
    def rtk_available(self, now: float) -> bool:
        """RTK FIX is current (fresh status that says FIXED)."""
        if self._last_status_t is None:
            return False
        if now - self._last_status_t > self.p.fix_status_timeout_s:
            return False
        return self.rtk_fixed

    def _rtk_reason(self, now: float) -> Optional[str]:
        if self.rtk_available(now):
            return "RTK fixed"
        stale = (self._last_status_t is not None
                 and now - self._last_status_t > self.p.fix_status_timeout_s)
        if stale:
            return None  # receiver silent: that is exactly when VIO is needed
        since = self._rtk_lost_since if self._rtk_lost_since is not None else now
        if now - since < self.p.rtk_holdoff_s:
            return "RTK lost %.1fs ago (holdoff %.1fs)" % (now - since, self.p.rtk_holdoff_s)
        return None

    def on_vio(self, now: float, vx: float, vy: float,
               var_vx: float, var_vy: float) -> Optional[str]:
        """Return None to forward this VIO message, else the reason it is dropped.

        Health bookkeeping runs on every message, whether or not RTK blocks it, so VIO
        is already warm the moment RTK drops.
        """
        p = self.p
        if self._last_vio_t is not None and now - self._last_vio_t > p.max_gap_s:
            self.healthy_streak = 0
        self._last_vio_t = now

        health = self._health_reason(vx, vy, var_vx, var_vy)
        if health is not None:
            self.healthy_streak = 0
        else:
            self.healthy_streak += 1
        self._update_divergence(now, vx)

        if health is not None:
            return health
        if self.diverged:
            return "VIO diverged from wheel odometry"
        if self.healthy_streak < p.warmup_msgs:
            return "VIO warming up (%d/%d)" % (self.healthy_streak, p.warmup_msgs)
        return self._rtk_reason(now)

    # -- internals --------------------------------------------------------------
    def _health_reason(self, vx, vy, var_vx, var_vy) -> Optional[str]:
        p = self.p
        values = (vx, vy, var_vx, var_vy)
        if not all(math.isfinite(v) for v in values):
            return "non-finite VIO velocity/covariance"
        if var_vx < 0.0 or var_vy < 0.0:
            return "negative VIO covariance"
        speed = math.hypot(vx, vy)
        if speed > p.max_speed:
            return "VIO speed %.2f m/s > max_speed %.2f" % (speed, p.max_speed)
        if max(var_vx, var_vy) > p.max_twist_var:
            return "VIO twist variance %.3f > max_twist_var %.3f" % (
                max(var_vx, var_vy), p.max_twist_var)
        return None

    def _update_divergence(self, now: float, vx: float) -> None:
        p = self.p
        if (self._wheel_t is None or self._wheel_vx is None
                or now - self._wheel_t > p.wheel_timeout_s or not math.isfinite(vx)):
            return
        disagree = abs(vx - self._wheel_vx) > p.max_wheel_disagreement
        if disagree:
            self._agree_since = None
            if self._disagree_since is None:
                self._disagree_since = now
            if now - self._disagree_since >= p.disagreement_hold_s:
                self.diverged = True
        else:
            self._disagree_since = None
            if self.diverged:
                if self._agree_since is None:
                    self._agree_since = now
                if now - self._agree_since >= p.recover_s:
                    self.diverged = False
                    self._agree_since = None
