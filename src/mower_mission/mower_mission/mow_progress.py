# Copyright 2026 Airseekers Tron ROS 2 port contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Live mow progress for the GUI (no ROS).

Read-only view of a running MissionFSM: which poses of the current area's
drivable sub-paths have been mowed, which were skipped, where the robot is on
the plan, and an elapsed / ETA estimate.  mission_node publishes two latched
std_msgs/String JSON topics from it:

* ``~/mow_plan``: ``{plan_id, area, subpaths: [[[x, y], ...], ...]}``, the
  drivable sub-paths in map frame, republished only when the plan changes.
  Pose indices in ``~/mow_progress`` are absolute indices into the
  concatenation of these sub-paths (the resume cursor's units).
* ``~/mow_progress``: ``{plan_id, area, state, sub_state, sub_path, sub_paths,
  pose_index, total_poses, mowed_segments: [[from, to], ...], current_segment,
  skipped: [[from, to], ...], blade_on, percent, mowed_m, remaining_m,
  elapsed_s, eta_s, why: {...}}`` at 1 Hz.

Only attributes are read; the FSM is never modified.
"""

from mower_mission.resume import subpath_offsets

def plan_geometry(fsm):
    """(plan_id, area, [[[x, y], ...]]) of the current area, or None."""
    m = getattr(fsm, 'mission', None)
    if m is None or m.area_idx is None or not m.subpaths:
        return None
    subs = [[[round(float(p[0]), 2), round(float(p[1]), 2)] for p in sp] for sp in m.subpaths]
    _offs, total = subpath_offsets(m.subpaths)
    pid = '%d:%d:%d:%d' % (int(m.area_idx), int(getattr(m, 'run_i', 0)), len(subs), total)
    if subs and subs[0]:
        pid += ':%.2f,%.2f' % tuple(subs[0][0])
    return pid, int(m.area_idx), subs


def _merge(ranges):
    out = []
    for a, b in sorted((int(a), int(b)) for a, b in ranges if b >= a):
        if out and a <= out[-1][1] + 1:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return out


def progress(fsm, plan_id, blade_on, elapsed_s, session_mowed_m, why=None):
    """The ``~/mow_progress`` dict (see module docstring)."""
    m = getattr(fsm, 'mission', None)
    st = {
        'plan_id': plan_id, 'area': -1, 'state': getattr(fsm, 'phase', ''),
        'sub_state': '', 'sub_path': -1, 'sub_paths': 0, 'pose_index': -1,
        'total_poses': 0, 'mowed_segments': [], 'current_segment': None, 'skipped': [],
        'blade_on': bool(blade_on), 'percent': 0.0, 'mowed_m': 0.0, 'remaining_m': 0.0,
        'elapsed_s': round(float(elapsed_s), 1) if elapsed_s is not None else None,
        'eta_s': None, 'why': why or {},
    }
    try:
        st['sub_state'] = fsm.display_sub_state()
    except Exception:  # noqa: BLE001 - presentation only
        st['sub_state'] = str(getattr(fsm, 'sub_state', ''))
    if m is None or m.area_idx is None or not m.subpaths:
        return st
    offs, total = subpath_offsets(m.subpaths)
    ac = fsm.cursor.areas.get(m.area_idx)
    completed = set(ac.completed) if ac else set()
    n = len(m.subpaths)
    sub_i = int(m.sub_i)
    local = int(m.progress_local if m.step == 'follow' else m.start_local)
    mowed, skipped = [], []
    for i in completed:
        if 0 <= i < n:
            mowed.append((offs[i], offs[i] + len(m.subpaths[i]) - 1))
    cur = None
    if 0 <= sub_i < n and sub_i not in completed:
        last = len(m.subpaths[sub_i]) - 1
        local = max(0, min(local, last))
        if local > 0:
            mowed.append((offs[sub_i], offs[sub_i] + local))
        cur = [offs[sub_i] + local, offs[sub_i] + last]
    for s in (ac.skipped if ac else ()):
        skipped.append((s[0], s[1]))
    for f in getattr(m, 'failed', ()) or ():
        try:
            area, sub, _why, idx = f
        except (TypeError, ValueError):
            continue
        if area != m.area_idx or sub is None or not 0 <= sub < n:
            continue
        start = idx if isinstance(idx, int) and idx >= offs[sub] else offs[sub]
        skipped.append((start, offs[sub] + len(m.subpaths[sub]) - 1))
    lengths = list(m.lengths or [])
    tot_len = sum(lengths)
    done = sum(lengths[i] for i in completed if i < len(lengths))
    if cur is not None and sub_i < len(m.cum) and m.cum[sub_i]:
        done += m.cum[sub_i][min(local, len(m.cum[sub_i]) - 1)]
    remaining = max(0.0, tot_len - done)
    eta = None
    if elapsed_s and elapsed_s > 60 and session_mowed_m and session_mowed_m > 1.0:
        rate = session_mowed_m / elapsed_s
        eta = round(remaining / rate) if rate > 0 else None
    st.update({
        'area': int(m.area_idx), 'sub_path': sub_i if sub_i < n else n, 'sub_paths': n,
        'pose_index': cur[0] if cur else -1, 'total_poses': total,
        'mowed_segments': _merge(mowed), 'current_segment': cur, 'skipped': _merge(skipped),
        'percent': round(min(100.0, 100.0 * done / tot_len), 1) if tot_len > 0 else 0.0,
        'mowed_m': round(done, 1), 'remaining_m': round(remaining, 1), 'eta_s': eta,
    })
    return st
