# Copyright 2026 ROS 2 port team
# SPDX-License-Identifier: Apache-2.0
"""Launch-time rewrites of nav2_params.yaml (pure python + pyyaml, unit-tested).

navigation.launch.py ``transit_controller``:

* ``mppi`` (default): FollowPath is the MPPI block. The GUI's ``transit_speed`` reaches the
  file as ``FollowPath.desired_linear_vel`` (launch/robot_settings.py, an RPP parameter);
  MPPI has no such parameter, so it is mapped onto ``vx_max`` (capped at the MCU's 0.3 m/s)
  and removed.
* ``rpp``: the FollowPathRPP block (the pre-2026-10-09 transit RPP) replaces the FollowPath
  block, so the BT's controller_id="FollowPath" drives RPP again. A transit_speed override
  is kept as RPP desired_linear_vel.
"""

import copy

TRANSIT_CONTROLLERS = ('mppi', 'rpp')
MCU_LINEAR_MAX = 0.3


def _controller_params(doc):
    cs = doc.get('controller_server') or doc.get('/controller_server') or {}
    return cs.get('ros__parameters', {})


def apply_transit_controller(doc, mode):
    """Copy of a nav2 params document with FollowPath set up for ``mode``."""
    mode = str(mode).strip().lower()
    if mode not in TRANSIT_CONTROLLERS:
        raise ValueError('transit_controller must be one of %s, got %r'
                         % (', '.join(TRANSIT_CONTROLLERS), mode))
    doc = copy.deepcopy(doc)
    params = _controller_params(doc)
    fp = params.get('FollowPath')
    if not isinstance(fp, dict):
        return doc
    speed = fp.get('desired_linear_vel')
    if mode == 'rpp':
        rpp = params.get('FollowPathRPP')
        if not isinstance(rpp, dict):
            raise ValueError('transit_controller:=rpp needs a FollowPathRPP block')
        fp = copy.deepcopy(rpp)
        if speed is not None:
            fp['desired_linear_vel'] = float(speed)
        params['FollowPath'] = fp
        return doc
    # mppi
    if 'MPPIController' in str(fp.get('plugin', '')) and speed is not None:
        fp['vx_max'] = min(float(speed), MCU_LINEAR_MAX)
        del fp['desired_linear_vel']
    return doc
