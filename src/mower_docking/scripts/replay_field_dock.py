#!/usr/bin/env python3
"""Offline replay of a field docking bag export through the dock_logic estimator/controller.

Input: a directory of JSONL topic exports (one object per message: {"t": receipt, "m": msg})
as made for 2026-10-21 night (accuracy-simout/field_dock_2026-10-21/ex082701):
mower_docking__marker_pose.jsonl (PoseStamped, base_link, uncorrected camera),
odometry__filtered_map.jsonl, mower_docking__state.jsonl, cmd_vel_docking.jsonl.

Open loop: the robot moved as it did that night. For every SEARCHING -> DOCKING window the
recorded markers and poses are fed to a fresh DockStateMachine (SEARCHING, Nav2 / lateral
align / pose check off) and its filter heading and commanded w are compared with the RTK
heading (dock pose from --dock) and with what the deployed code commanded.

    python3 replay_field_dock.py DIR --dock 0.417016 -1.306964 1.757547 [--old]
"""
import argparse
import bisect
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from mower_docking import dock_logic as dl  # noqa: E402

CAM_X = -0.201


def load(d, name):
    out = []
    with open(os.path.join(d, name)) as f:
        for line in f:
            out.append(json.loads(line))
    return out


def stamp(m):
    s = m['header']['stamp']
    return s['sec'] + s['nanosec'] * 1e-9


def yaw_q(q):
    return 2.0 * math.atan2(q['z'], q['w'])


def windows(states):
    """(t_search, t_end) for each SEARCHING that reached DOCKING."""
    out, t_s, docked = [], None, False
    prev = None
    for r in states:
        s, t = r['m']['data'], r['t']
        if s != prev:
            if s == 'SEARCHING':
                t_s, docked = t, False
            elif s == 'DOCKING' and t_s is not None:
                docked = True
            elif t_s is not None and prev in ('DOCKING', 'SEARCHING'):
                if docked:
                    out.append((t_s, t))
                t_s = None
            prev = s
    return out


def sign_flips(ws, thr=0.05):
    n, prev = 0, 0
    for w in ws:
        sg = 0 if abs(w) < thr else (1 if w > 0 else -1)
        if sg and prev and sg != prev:
            n += 1
        if sg:
            prev = sg
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('--dock', nargs=3, type=float, required=True)
    ap.add_argument('--yaw-correction-deg', type=float, default=-6.5)
    ap.add_argument('--old', action='store_true', help='the deployed (pre-fix) settings')
    a = ap.parse_args()
    dock = dl.Pose2D(*a.dock)
    odom = [(stamp(r['m']), r['m']['pose']['pose']) for r in load(a.dir, 'odometry__filtered_map.jsonl')]
    ot = [o[0] for o in odom]
    poses = [dl.Pose2D(p['position']['x'], p['position']['y'], yaw_q(p['orientation']))
             for _, p in odom]
    corr = 0.0 if a.old else math.radians(a.yaw_correction_deg)
    c, s = math.cos(corr), math.sin(corr)
    marks = []
    for r in load(a.dir, 'mower_docking__marker_pose.jsonl'):
        m = r['m']
        p, q = m['pose']['position'], m['pose']['orientation']
        x, y = p['x'] - CAM_X, p['y']                     # rotate about the camera point
        marks.append(dl.MarkerObs(CAM_X + c * x - s * y, s * x + c * y,
                                  dl.wrap_angle(yaw_q(q) + corr), stamp=stamp(m)))
    mt = [m.stamp for m in marks]
    rec = [(r['t'], r['m']['angular']['z'], r['m']['linear']['x'])
           for r in load(a.dir, 'cmd_vel_docking.jsonl')]
    rt = [r[0] for r in rec]
    if a.old:
        p = dl.DockParams(skip_nav_to_approach=True, lateral_align=False, pose_check_distance=0.0,
                          final_max_lateral=9.0, final_max_heading_deg=180.0,
                          search_min_detections=1, marker_position_only=False, kf_gate_chi2=0.0,
                          kf_q=0.01, kf_r=0.1, docked_marker_offset=0.45,
                          gains=dl.ControllerGains(k_lateral=7.0, max_approach_angle=0.5,
                                                   max_angular_accel=0.0))
    else:
        p = dl.DockParams(skip_nav_to_approach=True, lateral_align=False, pose_check_distance=0.0,
                          final_max_lateral=9.0, final_max_heading_deg=180.0,
                          search_min_detections=3, docked_marker_offset=0.45)
    tot = {'n': 0, 'new_flips': 0, 'rec_flips': 0, 'dur': 0.0, 'err': []}
    print('%s settings, camera yaw correction %.1f deg' % ('OLD' if a.old else 'NEW',
                                                          math.degrees(corr)))
    for t0, t1 in windows(load(a.dir, 'mower_docking__state.jsonl')):
        fsm = dl.DockStateMachine(p, dock, goal_timeout_s=1e9, dock_pose_measured=True)
        i0 = bisect.bisect(ot, t0)
        fsm.start(dl.Snapshot(t=t0, odom=poses[i0], map_pose=poses[i0], rtk_fixed=True))
        t, ws, errs, accepted, last_m = t0, [], [], None, None
        while t < t1:
            t += 0.05
            i = min(bisect.bisect(ot, t), len(poses) - 1)
            j = bisect.bisect(mt, t) - 1
            mk = marks[j] if j >= 0 and t - marks[j].stamp < 0.5 else None
            pose = poses[i]
            out = fsm.step(dl.Snapshot(t=t, odom=pose, map_pose=pose, marker=mk, rtk_fixed=True,
                                       progress_pose=pose, nav_status='succeeded'))
            if fsm.state == dl.DockState.DOCKING:
                accepted = accepted or t
                ws.append(out.angular)
                if fsm._kf.x is not None:
                    r = dl.rtk_dock_errors(pose, dock)
                    errs.append(math.degrees(dl.wrap_angle(fsm._kf.x.heading - r.heading)))
            elif accepted:
                break
        k0, k1 = bisect.bisect(rt, accepted or t0), bisect.bisect(rt, t1)
        rws = [w for tt, w, v in rec[k0:k1] if v < 0]
        dur = len(ws) * 0.05
        es = sorted(errs)
        med = es[len(es) // 2] if es else float('nan')
        spread = max(abs(e - med) for e in es) if es else float('nan')
        print('window %.1f-%.1f: accepted after %s s, DOCKING %.1f s; filter heading - RTK: median '
              '%+.1f deg, max dev from median %.1f deg; new w sign changes %d (%.2f/s), max |w| '
              '%.2f; recorded %d over %.1f s' % (
                  t0 % 1000, t1 % 1000, '%.1f' % (accepted - t0) if accepted else '-', dur, med,
                  spread, sign_flips(ws), sign_flips(ws) / max(dur, 1e-6),
                  max([abs(w) for w in ws] or [0]), sign_flips(rws), len(rws) * 0.05))
        tot['n'] += 1
        tot['new_flips'] += sign_flips(ws)
        tot['rec_flips'] += sign_flips(rws)
        tot['dur'] += dur
        tot['err'] += errs
    es = sorted(tot['err'])
    if es:
        med = es[len(es) // 2]
        print('ALL: %d windows, %.1f s DOCKING; heading vs RTK median %+.1f deg, p5..p95 %+.1f..%+.1f'
              '; new w sign changes %.2f/s (recorded %d)' % (
                  tot['n'], tot['dur'], med, es[int(len(es) * .05)], es[int(len(es) * .95)],
                  tot['new_flips'] / max(tot['dur'], 1e-6), tot['rec_flips']))


if __name__ == '__main__':
    main()
