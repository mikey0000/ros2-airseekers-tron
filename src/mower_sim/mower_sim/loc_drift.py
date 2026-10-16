# SPDX-License-Identifier: Apache-2.0
"""ROS-free model of the field's two-filter localisation: a drifting map -> odom.

On the robot (dual EKF, docs/localization.md) ``ekf_node`` integrates wheels + IMU into a
smooth but drifting odom frame and ``ekf_map`` (RTK) estimates the map pose; ``map -> odom``
is their difference and moves continuously. The sim used to publish a perfect static
identity map -> odom, which hid every bug that latches on a goal's odom-frame position
(2026-10-16 field run 6: Nav2 re-transforms the map-frame goal into odom every cycle and
LegGoalChecker treated each > 1 mm move as a new leg, so no corner leg ever ended).

This model reproduces both halves, seeded:

* **odom** is integrated from the TRUE body-frame motion with a linear scale error
  (``odom_scale_err``, sign drawn from the seed), a yaw-rate scale error
  (``odom_yaw_scale_err``) and a random-walk yaw-rate bias (``odom_yaw_bias_walk``): smooth,
  and drifting away from the truth over metres / minutes like wheel odometry;
* the **map estimate** is the truth plus an error that is an Ornstein-Uhlenbeck process
  (``map_err_std``, ``map_err_tau_s``) with occasional RTK-correction jumps
  (``jump_rate_hz``, ``jump_std``) and white per-sample jitter (``jitter_std``, scaled by
  ``still_jitter_scale`` while the robot stands still), plus a small yaw jitter.

``map -> odom`` = map estimate (+) inverse(odom), sampled at ``rate_hz`` (ekf_map's 10 Hz).
Defaults are calibrated against run 6 (2026-10-16 field bags, /odometry/filtered_map vs
/odometry/filtered while driving: map -> odom 10 Hz step median 5.4 mm, p90 13 mm,
p99 32 mm, 97 % of steps > 1 mm; drift over 10 s median 0.05 m, over 60 s 0.23 m; yaw step
p90 0.13 deg; see docs/simulation.md "Localisation drift").
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Tuple

Pose = Tuple[float, float, float]


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def compose(a: Pose, b: Pose) -> Pose:
    """a (+) b: pose b expressed in a's frame, mapped to a's parent frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _wrap(a[2] + b[2]))


def inverse(a: Pose) -> Pose:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), _wrap(-a[2]))


@dataclass
class LocDriftParams:
    enabled: bool = False
    seed: int = 1
    rate_hz: float = 10.0            # map estimate / map -> odom sample rate (ekf_map)
    # odom (wheel + IMU) integration errors
    odom_scale_err: float = 0.05     # |linear scale error|, sign from the seed
    odom_yaw_scale_err: float = 0.002
    odom_yaw_bias_walk: float = 0.0003   # rad/s per sqrt(s): random walk of the yaw-rate bias
    odom_yaw_bias_max: float = 0.0012    # rad/s clamp of that bias
    # map (RTK EKF) estimate error, world frame, per axis
    map_err_std: float = 0.012       # stationary std of the slow (OU) error, m
    map_err_tau_s: float = 3.0       # its correlation time
    jump_rate_hz: float = 0.25       # RTK correction jumps (Poisson), added to the slow error
    jump_std: float = 0.025          # per-axis size of a jump, m
    jitter_std: float = 0.0019       # white per-sample jitter, m per axis (while moving)
    still_jitter_scale: float = 0.1  # jitter (and jumps) scale while standing still
    yaw_jitter_std: float = 0.0008   # rad per sample (white)

    def active(self) -> bool:
        return bool(self.enabled)


class LocDrift:
    """Feed it the true pose every physics step (``step``); read ``odom`` (smooth, drifting)
    every step and ``sample()`` -> (map estimate, map -> odom) at ``rate_hz``."""

    def __init__(self, p: LocDriftParams, truth: Pose):
        self.p = p
        self.rng = random.Random(int(p.seed) * 7919 + 17)
        self.k_v = abs(p.odom_scale_err) * (1.0 if self.rng.random() < 0.5 else -1.0)
        self.k_w = abs(p.odom_yaw_scale_err) * (1.0 if self.rng.random() < 0.5 else -1.0)
        self.bias_w = 0.0
        self.last_truth = truth
        self.odom: Pose = (0.0, 0.0, 0.0)
        # odom starts where map -> odom is the identity at the start (as ekf_node after boot)
        self.odom = truth
        self.err = [0.0, 0.0]
        self.moving = False
        self.t_since_sample = 0.0
        self.map_to_odom: Pose = (0.0, 0.0, 0.0)
        self.map_est: Pose = truth

    def step(self, truth: Pose, dt: float) -> Pose:
        """Integrate odom over one physics step from the true motion; advance the map error
        process. Returns the odom pose (odom -> base_link)."""
        p = self.p
        if not p.active():
            self.last_truth = truth
            self.odom = truth
            self.map_est = truth
            return truth
        lt = self.last_truth
        # true motion in the previous body frame
        dx, dy = truth[0] - lt[0], truth[1] - lt[1]
        c, s = math.cos(lt[2]), math.sin(lt[2])
        bx, by = c * dx + s * dy, -s * dx + c * dy
        dth = _wrap(truth[2] - lt[2])
        self.last_truth = truth
        # moving: > 0.02 m/s or > 0.02 rad/s (a pivot counts: ekf_map is noisier then too)
        self.moving = math.hypot(bx, by) > 0.02 * dt or abs(dth) > 0.02 * dt
        # odom integration with scale errors and a slowly walking yaw-rate bias
        if dt > 0.0:
            self.bias_w += self.rng.gauss(0.0, p.odom_yaw_bias_walk * math.sqrt(dt))
            self.bias_w = max(-p.odom_yaw_bias_max, min(p.odom_yaw_bias_max, self.bias_w))
        moving_any = self.moving
        odth = dth * (1.0 + self.k_w) + (self.bias_w * dt if moving_any else 0.0)
        obx, oby = bx * (1.0 + self.k_v), by * (1.0 + self.k_v)
        ox, oy, oth = self.odom
        mid = oth + 0.5 * odth
        cm, sm = math.cos(mid), math.sin(mid)
        self.odom = (ox + cm * obx - sm * oby, oy + sm * obx + cm * oby, _wrap(oth + odth))
        # slow map error: OU + Poisson jumps
        if dt > 0.0 and p.map_err_tau_s > 0.0:
            a = math.exp(-dt / p.map_err_tau_s)
            q = p.map_err_std * math.sqrt(max(0.0, 1.0 - a * a))
            scale = 1.0 if self.moving else p.still_jitter_scale
            for i in range(2):
                self.err[i] = a * self.err[i] + self.rng.gauss(0.0, q * scale)
            if self.rng.random() < p.jump_rate_hz * dt * scale:
                for i in range(2):
                    self.err[i] += self.rng.gauss(0.0, p.jump_std)
        self.t_since_sample += dt
        return self.odom

    def due(self) -> bool:
        """True when a new map estimate / map -> odom sample is due (``rate_hz``)."""
        if self.p.rate_hz <= 0.0:
            return True
        if self.t_since_sample >= 1.0 / self.p.rate_hz - 1e-9:
            self.t_since_sample = 0.0
            return True
        return False

    def sample(self, truth: Pose) -> Tuple[Pose, Pose]:
        """New map estimate of the true pose and the map -> odom it implies."""
        p = self.p
        if not p.active():
            self.map_est, self.map_to_odom = truth, (0.0, 0.0, 0.0)
            return self.map_est, self.map_to_odom
        scale = 1.0 if self.moving else p.still_jitter_scale
        jx = self.rng.gauss(0.0, p.jitter_std * scale)
        jy = self.rng.gauss(0.0, p.jitter_std * scale)
        jth = self.rng.gauss(0.0, p.yaw_jitter_std * scale)
        self.map_est = (truth[0] + self.err[0] + jx, truth[1] + self.err[1] + jy,
                        _wrap(truth[2] + jth))
        self.map_to_odom = compose(self.map_est, inverse(self.odom))
        return self.map_est, self.map_to_odom
