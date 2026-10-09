# SPDX-License-Identifier: Apache-2.0
"""ROS-free diff-drive model with the Tron MCU's drive behaviour.

What the real drive does with a /cmd_vel (docs/wheel_control_semantics.md,
mower_mcu_driver ``_map_command``, nav2_params.yaml comments):

* linear and angular are clamped SEPARATELY to +-0.3 m/s and +-0.3 rad/s
  (not curvature preserving);
* while TURNING, a wheel asked for less than ~0.06 m/s does not turn (the speed
  floor: "wheel 0.04 m/s, under the MCU's ~0.06 m/s floor ... never broke free", the
  turn shaper's "inner >= 0.06"). Straight driving is not floored: the vendor's final
  dock reverse at 0.05 m/s works on hardware (``floor_straight`` to floor it too);
* the command is held until a 0.5 s timeout (the driver's ``cmd_vel_timeout``);
* the wheels lag the command (first-order, ``wheel_tau_s``).

Collisions: a step whose footprint would touch an obstacle is not taken (the
robot stops dead, as against a solid object) and the contact is reported so the
node can raise the bumper. The dock is a hard stop behind the charge point.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

from .world import Obstacle, Pose2D, World, to_local


def _clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class DriveParams:
    linear_max: float = 0.3          # MCU clamp, m/s
    angular_max: float = 0.3         # MCU clamp, rad/s
    track_width: float = 0.48        # m (cmd_vel_slew TRACK_WIDTH_M)
    min_wheel_speed: float = 0.06    # m/s, MCU floor while turning; 0 disables
    floor_straight: bool = False     # also floor straight (equal wheel) commands
    wheel_tau_s: float = 0.15        # first-order wheel lag; 0 = instant
    cmd_timeout_s: float = 0.5
    # dock hard stop: the charger contacts sit at the dock pose; the robot cannot
    # reverse past them while roughly lined up with the dock.
    dock_stop_lateral_m: float = 0.25
    dock_contact_xy_m: float = 0.03
    dock_contact_lateral_m: float = 0.06
    dock_contact_yaw_rad: float = 0.2


@dataclass
class StepResult:
    blocked: bool = False
    contacts: List[Obstacle] = field(default_factory=list)
    dock_stop: bool = False


def wheel_speeds(v: float, w: float, p: DriveParams):
    """(left, right) after the MCU clamp and speed floor."""
    v = _clamp(v, p.linear_max)
    w = _clamp(w, p.angular_max)
    half = p.track_width / 2.0
    left, right = v - w * half, v + w * half
    turning = abs(left - right) > 1e-6
    if p.min_wheel_speed > 0.0 and (turning or p.floor_straight):
        left = 0.0 if abs(left) < p.min_wheel_speed else left
        right = 0.0 if abs(right) < p.min_wheel_speed else right
    return left, right


class DiffDrive:
    def __init__(self, world: World, params: Optional[DriveParams] = None,
                 pose: Optional[Pose2D] = None):
        self.world = world
        self.p = params or DriveParams()
        start = pose or world.start
        self.pose = Pose2D(start.x, start.y, start.yaw)
        self.cmd = (0.0, 0.0)
        self.cmd_age = math.inf
        self.left = self.right = 0.0       # actual wheel speeds, m/s
        self.v = self.w = 0.0              # actual body twist
        self.enabled = True                # False = e-stop: wheels commanded to 0
        self.collisions_total = 0
        self.blocked = False

    def command(self, v: float, w: float):
        self.cmd = (float(v), float(w))
        self.cmd_age = 0.0

    def dock_frame(self, pose: Optional[Pose2D] = None):
        pose = pose or self.pose
        d = self.world.dock
        xd, yd = to_local((pose.x, pose.y), d.x, d.y, d.yaw)
        return xd, yd, wrap(pose.yaw - d.yaw)

    def on_dock(self) -> bool:
        xd, yd, yawd = self.dock_frame()
        p = self.p
        return (abs(xd) <= p.dock_contact_xy_m and abs(yd) <= p.dock_contact_lateral_m
                and abs(yawd) <= p.dock_contact_yaw_rad)

    def step(self, dt: float, t: float) -> StepResult:
        """Advance ``dt`` seconds; ``t`` is the sim time used for moving obstacles."""
        p = self.p
        self.cmd_age += dt
        if self.cmd_age > p.cmd_timeout_s or not self.enabled:
            target = (0.0, 0.0)
        else:
            target = wheel_speeds(self.cmd[0], self.cmd[1], p)
        a = 1.0 if p.wheel_tau_s <= 0.0 else min(1.0, dt / p.wheel_tau_s)
        self.left += a * (target[0] - self.left)
        self.right += a * (target[1] - self.right)
        if abs(self.left) < 1e-4:
            self.left = 0.0
        if abs(self.right) < 1e-4:
            self.right = 0.0
        v = 0.5 * (self.left + self.right)
        w = (self.right - self.left) / p.track_width
        res = StepResult()
        if v == 0.0 and w == 0.0:
            self.v = self.w = 0.0
            self.blocked = False
            return res
        cur = self.pose
        yaw_mid = cur.yaw + 0.5 * w * dt
        cand = Pose2D(cur.x + v * dt * math.cos(yaw_mid), cur.y + v * dt * math.sin(yaw_mid),
                      wrap(cur.yaw + w * dt))
        # dock hard stop (charger contacts at the dock pose, robot reverses in)
        xd, yd, yawd = self.dock_frame(cand)
        if xd < 0.0 and abs(yd) <= p.dock_stop_lateral_m and abs(yawd) < math.pi / 2:
            cxd, _, _ = self.dock_frame(cur)
            if xd < cxd:  # moving further in
                res.dock_stop = True
        hits = self.world.collisions(cand.x, cand.y, cand.yaw, t)
        if hits:
            now_hits = self.world.collisions(cur.x, cur.y, cur.yaw, t)
            if now_hits:
                # already touching (something walked into us): allow moves that back away
                cpos = [o.position(t) for o in hits]
                before = min(math.hypot(c[0] - cur.x, c[1] - cur.y) for c in cpos)
                after = min(math.hypot(c[0] - cand.x, c[1] - cand.y) for c in cpos)
                if after > before:
                    hits = []
        res.contacts = hits
        if hits or res.dock_stop:
            res.blocked = True
            if hits and not self.blocked:
                self.collisions_total += 1
            self.blocked = bool(hits)
            # stopped dead: the wheels stall
            self.left = self.right = 0.0
            self.v = self.w = 0.0
            return res
        if self.blocked and self.world.clearance(cand.x, cand.y, cand.yaw, t) > 0.02:
            self.blocked = False   # contact ends only once clear (no recount on creep)
        self.pose = cand
        self.v, self.w = v, w
        return res
