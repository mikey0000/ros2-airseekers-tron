# SPDX-License-Identifier: GPL-3.0-or-later
"""Charger-moved check (stock parity, 2026-11-03).

After a successful dock the robot sits on the charger contacts, so its RTK-fixed map pose
is the charger's docked pose. When that is more than ``dock_moved_warn_m`` from the stored
dock pose (``/map_server_node/docking_pose``) the charger was probably moved: the owner
is alerted and offered the dock re-calibration. ROS-free, unit tested.
"""

import math
from typing import Optional


def dock_moved(map_pose, dock_pose, rtk_fixed: bool, measured: bool,
               warn_m: float) -> Optional[dict]:
    """``map_pose`` / ``dock_pose`` (x, y, yaw) or None. Returns the diagnostic dict
    ``{moved, distance_m, docked, stored}`` when the check could run (RTK fixed, both poses,
    a measured dock pose, ``warn_m`` > 0), else None (not evaluated)."""
    if warn_m <= 0.0 or not rtk_fixed or not measured or map_pose is None or dock_pose is None:
        return None
    d = math.hypot(map_pose[0] - dock_pose[0], map_pose[1] - dock_pose[1])
    return {'moved': d > warn_m, 'distance_m': round(d, 3),
            'docked': [round(float(map_pose[0]), 3), round(float(map_pose[1]), 3)],
            'stored': [round(float(dock_pose[0]), 3), round(float(dock_pose[1]), 3)],
            'warn_m': float(warn_m)}


def notice(result) -> dict:
    """mower_alerts notice (rules.parse_notice) for a moved charger, one-shot."""
    return {'incident': 'dock_moved', 'active': True, 'oneshot': True, 'kind': 'blocked',
            'priority': 3, 'message': 'dockMoved',
            'text': 'The charging dock appears to have moved (%.2f m from the stored dock '
                    'pose): re-calibrate the dock in the GUI.' % result['distance_m']}
