#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Offline VIO evaluation steps (2026-11-14, docs/vio.md section 8). Driven by
``scripts/vio/eval_vio_bag.sh``; run in the dev image (rclpy for the bag reader).

    eval_vio_bag.py start --bag BAG        stamp where OpenVINS should start (stdout)
    eval_vio_bag.py bridge OUT --bag BAG   OUT/ov/odomimu.csv -> OUT/vio.csv (+ live-RTK gate)
    eval_vio_bag.py oracle OUT --bag BAG   OUT/ov/odomimu.csv made from RTK (plumbing test)
    eval_vio_bag.py report OUT             VIO health report (markdown on stdout, report.json)

``start``: a bag recorded from the dock begins in the camera nodes' idle duty cycle (1 Hz
pairs, 100 Hz IMU) and stereo_imu drops up to 1.2 s of samples when the mission starts and it
switches to 200 Hz. OpenVINS initialised in that gap diverged and never recovered (docs/vio.md
8.8), so it starts at the first IMU stamp after the first non-idle activity that has 2 s of
gap-free IMU behind it (0 when the bag does not start idle).

``bridge``: OpenVINS odomimu (``ros2_serial_msckf``) through vio_odom_bridge's conversion
into /odometry/vio rows, plus vio_gate's verdict on the recorded /fix_status and /odom (what
the gate would have done live). Column ``gate`` is empty when passed, else the reason.

``oracle``: NOT a VIO. Writes an odomimu.csv whose velocity is the RTK-fixed body velocity
plus noise (``--noise`` m/s), rotated into the Metoak IMU frame with the lever arm added back,
so the rest of the pipeline (bridge, gate, EKF replay, tables) can be tested on any field bag
without images. Its EKF results are an upper bound for a VIO that is right whenever RTK was.

``report``: OpenVINS health (init, update rate, CPU per update, features, camera-IMU time
offset), velocity against RTK and wheels per terrain class (flat, slope, side slope, pivot,
reverse, stationary), gate statistics (recorded RTK, and inside ekf_replay's simulated
dropouts from OUT/replay/gate.json).
"""

import argparse
import csv
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(STACK, 'scripts', 'replay'))

import vio_offline as vo  # noqa: E402

GATE_TOPICS = ('/fix_status', '/odom', '/mcu/measured_speed')
MOVING = 0.05          # m/s
PIVOT_WZ = 0.2         # rad/s with |vx| < MOVING
STILL = 0.02           # m/s RTK speed and wheel speed for "stationary"
SLOPE_DEG, FLAT_DEG = 8.0, 4.0
SAMPLE_HZ = 5.0        # velocity comparison samples per second


# ---------------------------------------------------------------------------- start
def cmd_start(a):
    import ekf_replay
    msgs = ekf_replay.read_bag(a.bag, 0, 4e9, ('/mission/activity',))
    acts = [(t, m.data) for t, _n, m in msgs]
    t_act = vo.full_rate_activity(acts)
    start = 0.0
    if t_act:
        imu = ekf_replay.read_bag(a.bag, t_act, t_act + 30.0, ('/stereo_imu/data',))
        stamps = [m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for _t, _n, m in imu]
        start = vo.imu_settled(stamps) or t_act
    print('%.6f' % start)


# ---------------------------------------------------------------------------- bridge
def cmd_bridge(a):
    import ekf_replay
    ov = vo.read_csv_rows(os.path.join(a.out, 'ov', 'odomimu.csv'))
    vio = vo.bridge_rows(ov)
    aux = {}
    ekf_replay.build_inputs(ekf_replay.read_bag(a.bag, 0, 4e9, GATE_TOPICS), aux)
    verdicts = vo.run_gate(vio, aux['status'], aux['wheel'])
    with open(os.path.join(a.out, 'vio.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(list(vo.VIO_COLUMNS) + ['gate'])
        for row, (_t, r) in zip(vio, verdicts):
            w.writerow(['%.9f' % row[0]] + ['%.6g' % x for x in row[1:]] + [r or ''])
    summ = vo.gate_summary(verdicts)
    with open(os.path.join(a.out, 'gate_recorded.json'), 'w') as fh:
        json.dump(summ, fh, indent=1)
    print('bridge: %d odomimu rows -> %d /odometry/vio rows; gate on the recorded RTK: '
          'passing %.0f %%' % (len(ov), len(vio), 100 * summ['passing_share']))


# ---------------------------------------------------------------------------- oracle
def cmd_oracle(a):
    import ekf_replay
    from stereo_vio_bridge.odom_convert import rpy_matrix
    msgs = ekf_replay.read_bag(a.bag, 0, 4e9)
    odom, _imu, fixes, field = ekf_replay.build_inputs(msgs, {})
    fit = ekf_replay.fit_enu_to_map(fixes, field)
    gps = [(t, x, y) for t, x, y, _v, c in ekf_replay.gps_stream(fixes, field, fit)
           if x is not None and c == ekf_replay.rtk_quality.RTK_FIXED]
    gt = [g[0] for g in gps]
    yaw = vo.Interp(field, 0.5, angle_cols=(3,))
    wz = vo.Interp([(t, w) for t, _vx, w in odom], 0.5)
    p = vo.METOAK_BRIDGE
    r = rpy_matrix(*p['imu_to_base_rpy'])
    rt = [[r[j][i] for j in range(3)] for i in range(3)]          # base -> imu
    lx, ly, lz = p['imu_xyz_in_base']
    rnd = random.Random(a.seed)
    os.makedirs(os.path.join(a.out, 'ov'), exist_ok=True)
    n = 0
    with open(os.path.join(a.out, 'ov', 'odomimu.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(vo.OV_COLUMNS)
        t, t_end = gt[0] + 1.0, gt[-1] - 1.0
        while t < t_end:
            v = vo.rtk_body_velocity(gps, gt, lambda s: (yaw.at(s) or (None,) * 3)[2], t)
            om = wz.at(t)
            if v is not None and om is not None:
                vb = (v[0] + rnd.gauss(0, a.noise), v[1] + rnd.gauss(0, a.noise), 0.0)
                wb = (0.0, 0.0, om[0])
                # v_imu = R^T (v_base + w x r): the inverse of odom_convert.base_twist
                wxr = (wb[1] * lz - wb[2] * ly, wb[2] * lx - wb[0] * lz, wb[0] * ly - wb[1] * lx)
                vb2 = [vb[i] + wxr[i] for i in range(3)]
                vi = [sum(rt[i][k] * vb2[k] for k in range(3)) for i in range(3)]
                wi = [sum(rt[i][k] * wb[k] for k in range(3)) for i in range(3)]
                var = a.noise ** 2 / 10.0       # the bridge scales by 10 again
                w.writerow(['%.6f' % t, 0, 0, 0, 1, 0, 0, 0] + ['%.6f' % x for x in vi + wi]
                           + [var, var, var])
                n += 1
            t += 1.0 / a.rate
    print('oracle: %d rows (RTK body velocity + %.3f m/s noise, %.0f Hz): NOT a VIO'
          % (n, a.noise, a.rate))


# ---------------------------------------------------------------------------- report
def load_inputs(path):
    """inputs.csv (ekf_replay) -> dict of row lists by kind."""
    out = {'field': [], 'gps': [], 'odom': [], 'tilt': []}
    with open(path) as fh:
        r = csv.reader(fh)
        next(r)
        for t, k, x, y, v1, v2 in r:
            if k == 'field':
                out['field'].append((float(t), float(x), float(y), float(v1)))
            elif k == 'gps' and x and v2 == 'fixed':
                out['gps'].append((float(t), float(x), float(y)))
            elif k == 'odom':
                out['odom'].append((float(t), float(v1), float(v2)))
            elif k == 'tilt':
                out['tilt'].append((float(t), float(x), float(y)))
    for v in out.values():
        v.sort()
    return out


def velocity_samples(vio, inp):
    """Samples at SAMPLE_HZ where VIO, wheels and RTK all have a value: dicts with vio
    vx / vy, wheel vx / wz, rtk vx / vy, roll / pitch (deg)."""
    yaw = vo.Interp(inp['field'], 0.5, angle_cols=(3,))
    wheel = vo.Interp(inp['odom'], 0.5)
    tilt = vo.Interp(inp['tilt'], 2.0)
    vv = vo.Interp([(r[0], r[1], r[2]) for r in vio], 0.5)
    gt = [g[0] for g in inp['gps']]
    out, last = [], -math.inf
    for r in vio:
        t = r[0]
        if t - last < 1.0 / SAMPLE_HZ:
            continue
        last = t
        v, wh = vv.at(t), wheel.at(t)
        rtk = vo.rtk_body_velocity(inp['gps'], gt, lambda s: (yaw.at(s) or (None,) * 3)[2], t)
        if v is None or wh is None or rtk is None:
            continue
        tl = tilt.at(t) or (float('nan'), float('nan'))
        out.append({'t': t, 'vio_vx': v[0], 'vio_vy': v[1], 'wh_vx': wh[0], 'wh_wz': wh[1],
                    'rtk_vx': rtk[0], 'rtk_vy': rtk[1], 'roll': tl[0], 'pitch': tl[1]})
    return out


SAMPLE_CLASSES = {
    'moving': lambda s: abs(s['wh_vx']) >= MOVING,
    'flat, moving': lambda s: abs(s['wh_vx']) >= MOVING
    and math.hypot(s['roll'], s['pitch']) < FLAT_DEG,
    'slope (tilt >= 8), moving': lambda s: abs(s['wh_vx']) >= MOVING
    and math.hypot(s['roll'], s['pitch']) >= SLOPE_DEG,
    'side slope (|roll| >= 8), moving': lambda s: abs(s['wh_vx']) >= MOVING
    and abs(s['roll']) >= SLOPE_DEG,
    'reverse': lambda s: s['wh_vx'] <= -MOVING,
    'pivot': lambda s: abs(s['wh_vx']) < MOVING and abs(s['wh_wz']) >= PIVOT_WZ,
    'stationary': lambda s: abs(s['wh_vx']) < STILL and abs(s['wh_wz']) < 0.05
    and math.hypot(s['rtk_vx'], s['rtk_vy']) < STILL,
}


def velocity_table(samples):
    """Per class: n; VIO and wheel vx error against RTK (signed p50, |.| p90); VIO vy error
    and the RTK vy the wheels miss (|.| p90); VIO against wheel vx (|.| p90)."""
    P = vo.percentile
    res = {}
    for name, pred in SAMPLE_CLASSES.items():
        sel = [s for s in samples if pred(s)]
        ev = [s['vio_vx'] - s['rtk_vx'] for s in sel]
        ew = [s['wh_vx'] - s['rtk_vx'] for s in sel]
        ey = [s['vio_vy'] - s['rtk_vy'] for s in sel]
        res[name] = {
            'n': len(sel),
            'vio_vx_err_p50': P(ev, 50), 'vio_vx_err_p90': P([abs(x) for x in ev], 90),
            'wheel_vx_err_p50': P(ew, 50), 'wheel_vx_err_p90': P([abs(x) for x in ew], 90),
            'vio_vy_err_p50': P(ey, 50), 'vio_vy_err_p90': P([abs(x) for x in ey], 90),
            'rtk_vy_p90': P([abs(s['rtk_vy']) for s in sel], 90),
            'vio_minus_wheel_vx_p90': P([abs(s['vio_vx'] - s['wh_vx']) for s in sel], 90),
        }
    return res


def ov_health(out_dir, inp, vio):
    """OpenVINS figures from updates.csv / odomimu rows."""
    P = vo.percentile
    path = os.path.join(out_dir, 'ov', 'updates.csv')
    upd = vo.read_csv_rows(path) if os.path.exists(path) else []
    h = {'updates': len(upd)}
    if upd:
        span = upd[-1]['t'] - upd[0]['t']
        h['update_hz'] = (len(upd) - 1) / span if span > 0 else float('nan')
        wall = [u['wall_ms'] for u in upd]
        h['wall_ms_p50'], h['wall_ms_p90'], h['wall_ms_max'] = P(wall, 50), P(wall, 90), max(wall)
        h['cpu_share_x86'] = sum(wall) / 1000.0 / span if span > 0 else float('nan')
        init = next((u['t'] for u in upd if u['initialized'] >= 1), None)
        move = next((t for t, vx, _wz in inp['odom'] if abs(vx) >= MOVING), None)
        h['first_image_t'] = upd[0]['t']
        h['init_after_first_image_s'] = init - upd[0]['t'] if init is not None else None
        h['init_after_first_motion_s'] = (init - move) if init is not None and move else None
        ok = [u for u in upd if u['initialized'] >= 1]
        h['msckf_feats_p10'] = P([u['msckf_feats'] for u in ok], 10)
        h['msckf_feats_p50'] = P([u['msckf_feats'] for u in ok], 50)
        h['slam_feats_p50'] = P([u['slam_feats'] for u in ok], 50)
        dts = [u['cam_imu_dt'] for u in ok]
        if dts:
            h['cam_imu_dt_final_ms'] = 1000 * dts[-1]
            h['cam_imu_dt_range_ms'] = (1000 * min(dts), 1000 * max(dts))
    if len(vio) > 1:
        span = vio[-1][0] - vio[0][0]
        h['odom_rows'] = len(vio)
        h['odom_hz'] = (len(vio) - 1) / span if span > 0 else float('nan')
        gaps = [b[0] - a[0] for a, b in zip(vio, vio[1:])]
        h['odom_gaps_over_0p5s'] = sum(1 for g in gaps if g > 0.5)
    return h


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and v != v):
        return '-'
    if isinstance(v, (tuple, list)):
        return ' .. '.join(fmt(x, nd) for x in v)
    return ('%.*f' % (nd, v)) if isinstance(v, float) else str(v)


def cmd_report(a):
    inp = load_inputs(os.path.join(a.out, 'replay', 'inputs.csv'))
    vio = []
    with open(os.path.join(a.out, 'vio.csv'), newline='') as fh:
        for row in csv.DictReader(fh):
            vio.append(tuple(float(row[k]) for k in vo.VIO_COLUMNS))
    rep = {'openvins': ov_health(a.out, inp, vio)}
    rep['velocity'] = velocity_table(velocity_samples(vio, inp))
    for name in ('gate_recorded.json', os.path.join('replay', 'gate.json')):
        p = os.path.join(a.out, name)
        if os.path.exists(p):
            with open(p) as fh:
                rep[name] = json.load(fh)
    with open(os.path.join(a.out, 'report.json'), 'w') as fh:
        json.dump(rep, fh, indent=1, default=str)

    print('# VIO health: %s\n' % os.path.basename(os.path.normpath(a.out)))
    if a.oracle:
        print('**Oracle run: the "VIO" is RTK velocity plus noise, not OpenVINS. '
              'Plumbing test / upper bound only.**\n')
    print('## OpenVINS\n')
    for k, v in rep['openvins'].items():
        print('- %s: %s' % (k, fmt(v)))
    print('\ncpu_share_x86 is ov_msckf update time over bag time on the dev box (one core); '
          'expect roughly 3-4x on one RK3588 A76 core.\n')
    print('## Velocity against RTK (base_link, m/s; RTK from 1 s central differences of FIXED '
          'fixes)\n')
    print('| class | n | VIO vx err p50 / p90 | wheel vx err p50 / p90 | VIO vy err p50 / p90 '
          '| RTK vy p90 (wheels say 0) | VIO - wheel vx p90 |')
    print('|---|---|---|---|---|---|---|')
    for c, v in rep['velocity'].items():
        print('| %s | %d | %s / %s | %s / %s | %s / %s | %s | %s |' % (
            c, v['n'], fmt(v['vio_vx_err_p50']), fmt(v['vio_vx_err_p90']),
            fmt(v['wheel_vx_err_p50']), fmt(v['wheel_vx_err_p90']), fmt(v['vio_vy_err_p50']),
            fmt(v['vio_vy_err_p90']), fmt(v['rtk_vy_p90']), fmt(v['vio_minus_wheel_vx_p90'])))
    g = rep.get('gate_recorded.json')
    if g:
        print('\n## Gate on the recorded RTK status\n')
        print('- %.0f s of VIO; passing %.1f %%; diverged latched %d times, %.1f s' % (
            g['seconds'], 100 * g['passing_share'], g['diverged_episodes'], g['diverged_s']))
        for k, v in g['share'].items():
            print('  - %s: %.1f %%' % (k, 100 * v))
    g = rep.get(os.path.join('replay', 'gate.json'))
    if g:
        print('\n## Gate inside the simulated RTK dropouts (per variant)\n')
        print('| variant | s | passing | warming up | diverged (episodes, s) | other |')
        print('|---|---|---|---|---|---|')
        for n, s in sorted(g.items()):
            sh = s['share']
            warm = sum(v for k, v in sh.items() if 'warming' in k)
            other = 1.0 - s['passing_share'] - warm - sum(
                v for k, v in sh.items() if 'diverged' in k)
            print('| %s | %.0f | %.1f %% | %.1f %% | %d, %.1f | %.1f %% |' % (
                n, s['seconds'], 100 * s['passing_share'], 100 * warm,
                s['diverged_episodes'], s['diverged_s'], 100 * other))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    st = sub.add_parser('start')
    st.add_argument('--bag', required=True)
    b = sub.add_parser('bridge')
    b.add_argument('out')
    b.add_argument('--bag', required=True)
    o = sub.add_parser('oracle')
    o.add_argument('out')
    o.add_argument('--bag', required=True)
    o.add_argument('--noise', type=float, default=0.03)
    o.add_argument('--rate', type=float, default=50.0)
    o.add_argument('--seed', type=int, default=1)
    r = sub.add_parser('report')
    r.add_argument('out')
    r.add_argument('--oracle', action='store_true')
    a = ap.parse_args(argv)
    {'start': cmd_start, 'bridge': cmd_bridge, 'oracle': cmd_oracle, 'report': cmd_report}[a.cmd](a)


if __name__ == '__main__':
    main()
