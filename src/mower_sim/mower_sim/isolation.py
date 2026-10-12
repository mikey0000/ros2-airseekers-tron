# SPDX-License-Identifier: Apache-2.0
"""DDS isolation guard for everything in mower_sim (B6, docs/REVIEW_2026-10-13.md).

The sim publishes a full fake driver layer (/odom, /imu/data, /fix, /estop,
/mower_base/status, TF odom->base_link, /odometry/filtered, latched
/heading_aligner/status, stereo clouds), serves /clear_estop, /cutter_control,
/charging, /cutter_off, and sim.launch.py adds a second twist_mux, Nav2, mission,
docking and gui_bridge; ``scenario --stub-state`` latches /motion_enabled=true and
fakes an AUTONOMOUS high-level state. The robot runs ROS_DOMAIN_ID=0 with an
unrestricted UDPv4 transport (docker/docker-compose.yml), so any of this started
on domain 0 or with LAN discovery joins the live robot's graph (2026-10-07: a
dev-host test on domain 0 fed fake poses into the mowing robot and latched a
boundary emergency stop).

``check_isolation()`` therefore refuses unless

* ``ROS_DOMAIN_ID`` is set to an integer other than 0, and
* discovery is restricted to this host: ``ROS_LOCALHOST_ONLY=1`` (deprecated on
  Jazzy, still honoured) or ``ROS_AUTOMATIC_DISCOVERY_RANGE`` LOCALHOST / OFF,
  and ``ROS_STATIC_PEERS`` is empty (static peers are contacted even with
  LOCALHOST), and
* this is not the mower itself (see ``on_robot_markers``).

Overrides (each logs a loud warning):

* ``MOWER_SIM_ALLOW_LAN=1``       skip the domain / discovery checks (a sim shared
                                  over the LAN on a private domain, on purpose).
* ``MOWER_SIM_ALLOW_ON_ROBOT=1``  skip the on-robot refusal.

On-robot detection: the mower keeps its stack and state on its own /userdata
partition, which no dev host or dev container has. It is "the mower" when

* ``/userdata`` is a mount point (robot host: the 32 GB data partition), or
* ``/userdata/ros2`` is a mount point (robot container: the compose bind mount), or
* ``/userdata/ros2_stack`` (host checkout, deploy target) or ``/userdata/mower``
  (vendor files, read-only bind in the container) exists.

Plain existence of ``/userdata/ros2`` is NOT a marker: the robot image mkdirs it
and several nodes (slip_detector, crash_record, ...) create directories under it
in any container, the dev image included. The hostname is reported in the error
but not relied on (it is user-settable and the container shares it).
"""

from __future__ import annotations

import os
import socket
from typing import Callable, Mapping, Optional

ON_ROBOT_MOUNTS = ('/userdata', '/userdata/ros2')
ON_ROBOT_PATHS = ('/userdata/ros2_stack', '/userdata/mower')
LOCAL_RANGES = ('LOCALHOST', 'OFF')
_TRUE = ('1', 'true', 'yes', 'on')


class IsolationError(RuntimeError):
    """The sim would join a non-isolated DDS graph (possibly the live robot's)."""


def _flag(env: Mapping[str, str], name: str) -> bool:
    return env.get(name, '').strip().lower() in _TRUE


def on_robot_markers(exists: Callable[[str], bool] = os.path.exists,
                     ismount: Callable[[str], bool] = os.path.ismount) -> list:
    """The on-robot markers present here (empty = not the mower)."""
    found = ['%s (mount point)' % m for m in ON_ROBOT_MOUNTS if ismount(m)]
    return found + [p for p in ON_ROBOT_PATHS if exists(p)]


def isolation_problems(env: Mapping[str, str]) -> list:
    """Why ``env`` is not an isolated graph (empty list = isolated)."""
    problems = []
    dom = env.get('ROS_DOMAIN_ID', '').strip()
    if not dom:
        problems.append('ROS_DOMAIN_ID is not set (defaults to 0, the robot\'s domain)')
    else:
        try:
            if int(dom) == 0:
                problems.append('ROS_DOMAIN_ID=0 is the robot\'s domain')
        except ValueError:
            problems.append('ROS_DOMAIN_ID=%r is not an integer' % dom)
    localhost_only = env.get('ROS_LOCALHOST_ONLY', '').strip() == '1'
    rng = env.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '').strip().upper()
    if not (localhost_only or rng in LOCAL_RANGES):
        problems.append('discovery is not localhost-only (set ROS_AUTOMATIC_DISCOVERY_RANGE='
                        'LOCALHOST, or ROS_LOCALHOST_ONLY=1; got ROS_LOCALHOST_ONLY=%r, '
                        'ROS_AUTOMATIC_DISCOVERY_RANGE=%r)'
                        % (env.get('ROS_LOCALHOST_ONLY', ''),
                           env.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '')))
    if env.get('ROS_STATIC_PEERS', '').strip():
        problems.append('ROS_STATIC_PEERS=%r adds remote peers' % env['ROS_STATIC_PEERS'])
    return problems


def check_isolation(what: str = 'mower_sim', env: Optional[Mapping[str, str]] = None,
                    exists: Callable[[str], bool] = os.path.exists,
                    ismount: Callable[[str], bool] = os.path.ismount,
                    warn: Optional[Callable[[str], None]] = None) -> list:
    """Raise IsolationError unless ``env`` is an isolated graph off the robot.

    Returns the warnings issued for active overrides (also passed to ``warn``,
    default: stderr). Call before rclpy.init() / before any publisher exists.
    """
    env = os.environ if env is None else env
    if warn is None:
        def warn(msg):
            import sys
            print(msg, file=sys.stderr, flush=True)
    warnings = []
    found = on_robot_markers(exists, ismount)
    if found:
        if _flag(env, 'MOWER_SIM_ALLOW_ON_ROBOT'):
            warnings.append(
                '!!! %s: MOWER_SIM_ALLOW_ON_ROBOT=1: starting the simulator ON THE MOWER '
                '(%s, markers %s). It competes with the real stack for CPU and, without '
                'isolation, for its topics. !!!' % (what, socket.gethostname(), ', '.join(found)))
        else:
            raise IsolationError(
                '%s refuses to run on the mower itself (host %s, found %s). The simulator '
                'replaces the hardware drivers and must run on a dev host. Override: '
                'MOWER_SIM_ALLOW_ON_ROBOT=1.' % (what, socket.gethostname(), ', '.join(found)))
    problems = isolation_problems(env)
    if problems:
        if _flag(env, 'MOWER_SIM_ALLOW_LAN'):
            warnings.append(
                '!!! %s: MOWER_SIM_ALLOW_LAN=1: running WITHOUT DDS isolation (%s). Fake '
                'odometry / GPS / e-stop / motion_enabled can reach any robot on this '
                'domain and network. Never do this on ROS_DOMAIN_ID=0. !!!'
                % (what, '; '.join(problems)))
        else:
            raise IsolationError(
                '%s refuses to start: %s. The sim would join the live robot\'s DDS graph '
                '(robot: domain 0, LAN discovery) and inject fake drivers, TF and '
                '/motion_enabled. Run e.g.\n'
                '  export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST '
                'ROS_LOCALHOST_ONLY=1\n'
                'or use ./scripts/sim_gui.sh / ./scripts/dev_build.sh. Deliberate LAN sim: '
                'MOWER_SIM_ALLOW_LAN=1 (never on domain 0).' % (what, '; '.join(problems)))
    for w in warnings:
        warn(w)
    return warnings


def lan_allowed(env: Optional[Mapping[str, str]] = None) -> bool:
    """True when MOWER_SIM_ALLOW_LAN=1 (the launch then leaves discovery as it is)."""
    return _flag(os.environ if env is None else env, 'MOWER_SIM_ALLOW_LAN')
