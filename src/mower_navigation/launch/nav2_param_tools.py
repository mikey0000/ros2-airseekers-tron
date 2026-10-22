# Copyright 2026 ROS 2 port team
# SPDX-License-Identifier: Apache-2.0
"""Launch-time rewrites of nav2_params.yaml (pure python + pyyaml, unit-tested).

navigation.launch.py: FollowPath is the MPPI transit block. The GUI's ``transit_speed``
reaches the file as ``FollowPath.desired_linear_vel`` (launch/robot_settings.py, an RPP
parameter name); MPPI has no such parameter, so it is mapped onto ``vx_max`` (capped at the
MCU's 0.3 m/s) and removed. (The ``transit_controller:=rpp`` mode, which swapped the old
FollowPathRPP block back in, was deleted on 2026-10-23 with the RPP configs.)
"""

import copy

MCU_LINEAR_MAX = 0.3


def _controller_params(doc):
    cs = doc.get('controller_server') or doc.get('/controller_server') or {}
    return cs.get('ros__parameters', {})


def apply_transit_speed(doc):
    """Copy of a nav2 params document with a GUI transit speed mapped onto MPPI vx_max."""
    doc = copy.deepcopy(doc)
    params = _controller_params(doc)
    fp = params.get('FollowPath')
    if not isinstance(fp, dict):
        return doc
    speed = fp.get('desired_linear_vel')
    if 'MPPIController' in str(fp.get('plugin', '')) and speed is not None:
        fp['vx_max'] = min(float(speed), MCU_LINEAR_MAX)
        del fp['desired_linear_vel']
    return doc
