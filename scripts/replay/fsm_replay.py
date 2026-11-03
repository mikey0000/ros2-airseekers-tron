#!/usr/bin/env python3
"""Incident fixtures for the mission FSM: extract them from a bag + mission log, replay them.

  # in the dev image (./scripts/dev_build.sh shell), workspace built and sourced:
  scripts/replay/fsm_replay.py extract --bag /tmp/bags/20261102-021049 \
      --log /tmp/python3_171_1793571791990.log --from 2026-11-02T02:14:30 --until 02:15:50 \
      --areas /tmp/areas.dat --area 5 --id 20261102_side_slope_boundary \
      --out src/mower_mission/test/replay/fixtures/20261102_side_slope_boundary.json
  scripts/replay/fsm_replay.py run 20261102_side_slope_boundary [--until 30]

``--bag`` takes bag directories or split files (.db3 / .db3.zstd; incident directories have no
metadata.yaml, that is fine). ``--log`` is the mission node's ROS log file
(/userdata/ros2/log/python3_<pid>_<start ms>.log on the mower: the one whose first line is
``[behavior_tree_node]``). Times: epoch seconds, ISO UTC, or HH:MM[:SS] (same day as --from).

``run`` drives a Harness (test/test_mission_fsm.py) with the fixture: recorded inputs through the
real MissionNode handlers, the recorded high_level_control commands at their times, services
answered by the Harness (areas from the fixture), and prints replayed FSM transitions beside the
recorded ``state A -> B`` lines. Scenario-specific action outcomes belong in the regression test
(src/mower_mission/test/replay/test_incident_regressions.py), not here.
"""
import argparse
import datetime as dt
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(STACK, 'src', 'mower_mission', 'test', 'replay'))

LOG_RE = re.compile(r'^\[(\w+)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$')
STRING_TOPICS = ('/heading_aligner/status', '/localization/status', '/tilt/status',
                 '/obstacle_policy', '/obstacle_tracker/nogo_zones', '/supervisor/status')
SCALAR_TOPICS = ('/map_server_node/outside_m', '/map_server_node/lethal_boundary_violation')
ODOM_TOPICS = ('/odometry/filtered_map', '/odom')
FIX_STATUS = '/fix_status'
# emitted on every change even inside their sampling period
CHANGE_TOPICS = ('/gps/status', '/obstacle_policy', '/obstacle_tracker/nogo_zones')


def parse_time(s, ref=None):
    try:
        return float(s)
    except ValueError:
        pass
    if re.match(r'^\d\d:\d\d(:\d\d(\.\d+)?)?$', s):
        if ref is None:
            raise SystemExit('time %s needs a full --from date' % s)
        day = dt.datetime.fromtimestamp(ref, dt.timezone.utc).strftime('%Y-%m-%d')
        s = day + 'T' + s
    d = dt.datetime.fromisoformat(s.replace('Z', ''))
    return d.replace(tzinfo=dt.timezone.utc).timestamp()


def utc(t):
    return dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'


def read_areas(path, indices):
    kv = {}
    with open(path) as fh:
        for line in fh:
            if ':' in line and not line.startswith('#'):
                k, v = line.split(':', 1)
                kv[k.strip()] = v.strip()

    def poly(s):
        return [[round(float(a), 4), round(float(b), 4)]
                for a, b in (p.split(',') for p in s.split(';') if p)]
    out = []
    for i in indices:
        n = int(kv.get('area_%d_obstacle_count' % i, 0))
        out.append({'index': i, 'name': kv['area_%d_name' % i],
                    'outer': poly(kv['area_%d_polygon' % i]),
                    'obstacles': [poly(kv['area_%d_obstacle_%d' % (i, k)]) for k in range(n)
                                  if 'area_%d_obstacle_%d' % (i, k) in kv],
                    'is_navigation_area': kv.get('area_%d_is_navigation' % i, '0') in ('1', 'true')})
    return out


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def gnss_from_fix_status(text):
    """/gps/status is not recorded: derive fix_type from the um960 /fix_status line."""
    m = re.search(r'quality=(\w+)', text)
    q = m.group(1) if m else ''
    fix = {'RTK_FIXED': 3, 'RTK_FLOAT': 2}.get(q, 1 if q not in ('', 'NONE', 'INVALID') else 0)
    return {'fix_type': fix, 'quality_percent': {3: 100.0, 2: 50.0}.get(fix, 0.0)}


def extract(a):
    import bagread
    t_from = parse_time(a.from_)
    t_until = parse_time(a.until, t_from)
    t0 = t_from
    wanted = set(STRING_TOPICS + SCALAR_TOPICS + ODOM_TOPICS + (FIX_STATUS,))
    events, last_emit, last_data, last_kind = [], {}, {}, {}
    period = {'/odometry/filtered_map': 1.0 / a.odom_hz, '/odom': 1.0 / a.odom_hz,
              '/gps/status': 1.0, '/tilt/status': 0.5,
              '/map_server_node/outside_m': 0.2, '/map_server_node/lethal_boundary_violation': 1.0,
              '/heading_aligner/status': 1.0, '/localization/status': 1.0,
              '/obstacle_policy': 0.2}
    for spec in a.every or []:
        topic, sec = spec.rsplit('=', 1)
        period[topic] = float(sec)
    wanted -= set(a.drop or [])
    for t, topic, msg in bagread.messages(a.bag, wanted, t_from, t_until):
        derived = None
        if topic in ODOM_TOPICS:
            p = msg.pose.pose
            body = {'x': round(p.position.x, 4), 'y': round(p.position.y, 4),
                    'yaw': round(_yaw(p.orientation), 4)}
        elif topic == FIX_STATUS:
            topic, body, derived = '/gps/status', gnss_from_fix_status(msg.data), \
                'from /fix_status quality= (um960 driver line)'
        elif topic in SCALAR_TOPICS:
            body = {'data': round(float(msg.data), 4) if isinstance(msg.data, float)
                    else bool(msg.data)}
        elif topic == '/supervisor/status':
            # only what mission_node reads (critical_down); the full status is ~7 KB
            try:
                down = json.loads(msg.data).get('critical_down', [])
            except (ValueError, AttributeError):
                down = []
            body = {'data': json.dumps({'critical_down': down})}
            derived = 'supervisor status reduced to critical_down'
        else:
            body = {'data': msg.data}
        key = json.dumps(body, sort_keys=True)
        changed = last_data.get(topic) != key
        if topic == '/obstacle_policy':     # a kind change passes, distance jitter does not
            try:
                kind = json.loads(msg.data).get('kind')
            except (ValueError, AttributeError):
                kind = None
            changed, last_kind[topic] = kind != last_kind.get(topic), kind
        due = t - last_emit.get(topic, -1e9) >= period.get(topic, 0.0)
        if not (due or (changed and topic in CHANGE_TOPICS)):
            continue
        last_emit[topic], last_data[topic] = t, key
        e = {'t': round(t - t0, 3), 'topic': topic, 'msg': body}
        if derived:
            e['derived'] = derived
        events.append(e)
    if a.plan_slice:
        # the mission's own plan (/behavior_tree_node/mow_plan, latched on change): the last
        # one published at or before --until, sliced to SUB:FROM:TO poses
        sub, lo, hi = (int(v) for v in a.plan_slice.split(':'))
        plan = None
        for t, _topic, msg in bagread.messages(a.bag, {'/behavior_tree_node/mow_plan'},
                                               None, t_until):
            plan = (t, json.loads(msg.data))
        if plan is None:
            raise SystemExit('no /behavior_tree_node/mow_plan in the bag up to --until')
        a.plan_out = {'t': round(plan[0] - t0, 3), 'plan_id': plan[1].get('plan_id'),
                      'area': plan[1].get('area'), 'sub_path': sub, 'from_pose': lo,
                      'poses': plan[1]['subpaths'][sub][lo:hi]}
    if a.log:
        with open(a.log, errors='replace') as fh:
            for line in fh:
                m = LOG_RE.match(line.rstrip('\n'))
                if m and m.group(3) == a.log_node and t_from <= float(m.group(2)) <= t_until:
                    events.append({'t': round(float(m.group(2)) - t0, 3),
                                   'log': '%s %s' % (m.group(1)[0], m.group(4))})
    events.sort(key=lambda e: e['t'])
    fx = {'id': a.id, 'bag': a.bag_id or ', '.join(os.path.basename(p.rstrip('/')) for p in a.bag),
          'log': os.path.basename(a.log) if a.log else None,
          'window_utc': [utc(t_from), utc(t_until)], 't0': t0, 'fix': a.fix,
          'pre_fix': a.pre_fix, 'notes': a.note or [],
          'areas': read_areas(a.areas, a.area) if a.areas else [], 'events': events}
    if getattr(a, 'plan_out', None):
        fx['plan'] = a.plan_out
    for path in a.merge or []:
        # hand-derived inputs for what the bag did not record (each event carries "derived")
        with open(path) as fh:
            extra = json.load(fh)
        for e in extra.pop('events', []):
            if 'stamp_t' in e:                     # {"stamp_t": true}: stamp = t0 + t
                e.pop('stamp_t')
                d = json.loads(e['msg']['data'])
                d['stamp'] = round(t0 + e['t'], 2)
                e['msg']['data'] = json.dumps(d)
            fx['events'].append(e)
        fx['notes'] += extra.pop('notes', [])
        fx.update(extra)
    fx['events'].sort(key=lambda e: e['t'])
    text = json.dumps(fx, separators=(',', ':'))
    with open(a.out, 'w') as fh:
        fh.write(text + '\n')
    n_topic = sum(1 for e in events if 'topic' in e)
    print('%s: %d input events, %d log lines, %.1f KB' % (a.out, n_topic, len(events) - n_topic,
                                                          len(text) / 1024.0))
    if len(text) > 200 * 1024:
        print('warning: over the 200 KB fixture budget: narrow the window or lower --odom-hz')


def extract_dets(a):
    """Obstacle-tracker fixture: map-frame detection observations (obstacle_tracker_node
    det_observation on the recorded /ai/det/detections_ranged and the latest
    /odometry/filtered_map pose, as the node does) plus 2 Hz robot poses."""
    import bagread
    from mower_map import obstacle_tracker_node as otn
    t_from = parse_time(a.from_)
    t_until = parse_time(a.until, t_from)
    t0 = t_from
    classes = list(otn.PARAMS['det_classes'])
    cam = float(otn.PARAMS['camera_x_m'])
    pose, events, last_pose_t = None, [], -1e9
    for t, topic, msg in bagread.messages(a.bag, {'/odometry/filtered_map',
                                                  '/ai/det/detections_ranged'}, t_from, t_until):
        if topic == '/odometry/filtered_map':
            p = msg.pose.pose
            pose = (p.position.x, p.position.y, _yaw(p.orientation))
            if t - last_pose_t >= 0.5:
                last_pose_t = t
                events.append({'t': round(t - t0, 3), 'pose': [round(v, 4) for v in pose]})
            continue
        obs = []
        for det in msg.detections:
            o = otn.det_observation(det, pose, classes, cam)
            if o is not None and o[2] and not o[2].lstrip('-').isdigit():
                obs.append([round(o[0], 3), round(o[1], 3), o[2], round(o[3], 3), round(o[4], 3)])
        if obs:
            events.append({'t': round(t - t0, 3), 'det': obs})
    log = []
    for path in a.log or []:
        with open(path, errors='replace') as fh:
            for line in fh:
                m = LOG_RE.match(line.rstrip('\n'))
                if m and t_from <= float(m.group(2)) <= t_until and m.group(3) in (
                        'obstacle_tracker', a.log_node):
                    log.append({'t': round(float(m.group(2)) - t0, 3),
                                'log': '%s [%s] %s' % (m.group(1)[0], m.group(3), m.group(4))})
    fx = {'id': a.id, 'bag': a.bag_id, 'window_utc': [utc(t_from), utc(t_until)], 't0': t0,
          'fix': a.fix, 'pre_fix': a.pre_fix, 'notes': a.note or [],
          'areas': read_areas(a.areas, a.area) if a.areas else [],
          'events': sorted(events + log, key=lambda e: e['t'])}
    text = json.dumps(fx, separators=(',', ':'))
    with open(a.out, 'w') as fh:
        fh.write(text + '\n')
    print('%s: %d detection msgs, %.1f KB' % (a.out, sum('det' in e for e in events),
                                             len(text) / 1024.0))


def run(a):
    import replay_lib as rl
    from test_mission_fsm import Harness
    from mower_mission import mission_fsm as f
    fx = rl.load(a.fixture)
    h = Harness(route_transits=a.route_transits)
    h.areas = [rl.area_dict(x) for x in fx['areas']] or h.areas
    r = rl.Replay(h, fx)
    first = r.last('/odometry/filtered_map', 0.0) or next(
        (e for e in r.events if e['topic'] == '/odometry/filtered_map'), None)
    if first:
        r.apply(first)
    cmds = [(t, int(m.group(1))) for t, text in rl.recorded_log(fx, 'high_level_control(')
            for m in [re.search(r'high_level_control\((\d+)\)', text)] if m]
    until = a.until if a.until is not None else max([e['t'] for e in fx['events']] or [0.0])
    phase = h.fsm.phase
    print('replay %s (%s, window %s .. %s)' % (fx['id'], fx['bag'], *fx['window_utc']))
    for t, text in rl.recorded_log(fx, 'state '):
        if t <= until:
            print('  recorded %8.2f  %s' % (t, text[2:]))
    marks = sorted(set([c[0] for c in cmds if c[0] <= until] + [until]))
    for tm in marks:
        k0 = h.mark()
        r.run_until(tm)
        for t, c in cmds:
            if t == tm:
                h.cmd(c)
                h.answer_services()
        for e in h.since(k0, f.PublishStatus):
            if e.status['state_name'] != phase:
                print('  replayed %8.2f  %s -> %s' % (h.t - r.base, phase, e.status['state_name']))
                phase = e.status['state_name']
    print('final phase: %s' % h.fsm.phase)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    x = sub.add_parser('extract', help='bag + mission log window -> JSON fixture')
    x.add_argument('--bag', nargs='+', required=True)
    x.add_argument('--log')
    x.add_argument('--log-node', default='behavior_tree_node')
    x.add_argument('--from', dest='from_', required=True)
    x.add_argument('--until', required=True)
    x.add_argument('--areas', help='areas.dat (map_server_node) for the area outlines')
    x.add_argument('--area', type=int, nargs='*', default=[])
    x.add_argument('--odom-hz', type=float, default=5.0)
    x.add_argument('--every', action='append', metavar='TOPIC=S',
                   help='sample TOPIC at most every S seconds (changes of the CHANGE_TOPICS '
                        'still pass)')
    x.add_argument('--drop', action='append', metavar='TOPIC', help='leave TOPIC out')
    x.add_argument('--plan-slice', metavar='SUB:FROM:TO',
                   help='store poses FROM..TO of sub-path SUB of the last recorded mow_plan')
    x.add_argument('--id', required=True)
    x.add_argument('--bag-id')
    x.add_argument('--fix', default='')
    x.add_argument('--pre-fix', default='')
    x.add_argument('--note', action='append')
    x.add_argument('--merge', action='append', metavar='JSON',
                   help='merge {"events": [...], "notes": [...], <keys>} (derived inputs)')
    x.add_argument('--out', required=True)
    d = sub.add_parser('extract-dets', help='bag window -> obstacle-tracker detection fixture')
    for k in ('--from', '--until', '--id', '--out'):
        d.add_argument(k, dest='from_' if k == '--from' else k[2:], required=True)
    d.add_argument('--bag', nargs='+', required=True)
    d.add_argument('--log', nargs='*', help='ROS log files (obstacle_tracker / mission lines kept)')
    d.add_argument('--log-node', default='behavior_tree_node')
    d.add_argument('--areas')
    d.add_argument('--area', type=int, nargs='*', default=[])
    d.add_argument('--bag-id', default='')
    d.add_argument('--fix', default='')
    d.add_argument('--pre-fix', default='')
    d.add_argument('--note', action='append')
    r = sub.add_parser('run', help='replay a fixture into the FSM and print transitions')
    r.add_argument('fixture')
    r.add_argument('--until', type=float)
    r.add_argument('--route-transits', action='store_true')
    a = ap.parse_args(argv)
    {'extract': extract, 'extract-dets': extract_dets, 'run': run}[a.cmd](a)


if __name__ == '__main__':
    main()
