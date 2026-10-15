#!/usr/bin/env python3
"""Dump a sim gate bag (sqlite3 storage) to one JSON file for evaluate.py.

Runs in the dev image (needs rclpy for deserialisation). Reads a COPY of the bag: never open
the recorder's live .db3 with sqlite (SQLITE_BUSY kills the recorder).
usage: extract.py <bag_dir> <out.json>"""
import glob
import json
import math
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LOGNODES = ('behavior_tree_node', 'mower_docking', 'docking_server', 'bt_navigator',
            'controller_server', 'cmd_vel_slew', 'map_server_node', 'mower_sim',
            'behavior_server', 'planner_server')
TWIST = {'/cmd_vel': 'cmd', '/cmd_vel_nav': 'cmd_nav', '/cmd_vel_emergency': 'cmd_emerg',
         '/cmd_vel_docking': 'cmd_dock'}
STATUS = {'/navigate_to_pose/_action/status': 'nav_status', '/spin/_action/status': 'spin_status',
          '/follow_path/_action/status': 'fp_status',
          '/mower_docking/dock/_action/status': 'dock_status'}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main(bagdir, out):
    keys = ['gt', 'status', 'hl', 'cmd', 'cmd_nav', 'cmd_emerg', 'cmd_dock', 'outside', 'plan',
            'rosout', 'nav_status', 'spin_status', 'fp_status', 'dock_status', 'shape',
            'incident', 'progress', 'mow_plan', 'odom_map', 'croute']
    D = {k: [] for k in keys}
    types = {}
    for f in sorted(glob.glob(bagdir + '/*.db3')):
        c = sqlite3.connect('file:' + f + '?mode=ro', uri=True)
        tmap = {r[0]: (r[1], r[2]) for r in c.execute('select id,name,type from topics')}
        last = -1
        while True:
            try:
                batch = c.execute('select id,topic_id,timestamp,data from messages where id > ? '
                                  'order by id limit 5000', (last,)).fetchall()
            except sqlite3.DatabaseError as e:     # a torn last page of a live copy
                print('warning: %s after id %d' % (e, last))
                break
            if not batch:
                break
            last = batch[-1][0]
            for _id, tid, ts, data in batch:
                name, typ = tmap[tid]
                if typ not in types:
                    types[typ] = get_message(typ)
                m = deserialize_message(data, types[typ])
                t = ts / 1e9
                if name == '/sim/ground_truth':
                    p = m.pose.pose
                    D['gt'].append([t, p.position.x, p.position.y, yaw(p.orientation),
                                    m.twist.twist.linear.x, m.twist.twist.angular.z])
                elif name == '/odometry/filtered_map':
                    p = m.pose.pose
                    D['odom_map'].append([t, p.position.x, p.position.y, yaw(p.orientation)])
                elif name == '/sim/status':
                    try:
                        s = json.loads(m.data)
                    except ValueError:
                        continue
                    D['status'].append([t, s.get('docked'), s.get('charging'),
                                        s.get('collisions'), s.get('cutting'), s.get('estop')])
                elif name == '/behavior_tree_node/high_level_status':
                    D['hl'].append([t, m.state_name, m.sub_state_name, m.current_area,
                                    m.current_path, m.current_path_index])
                elif name in TWIST:
                    D[TWIST[name]].append([t, m.linear.x, m.angular.z])
                elif name == '/map_server_node/outside_m':
                    D['outside'].append([t, m.data])
                elif name == '/plan':
                    D['plan'].append([t, [[q.pose.position.x, q.pose.position.y]
                                          for q in m.poses]])
                elif name == '/rosout':
                    if any(n in m.name for n in LOGNODES):
                        D['rosout'].append([t, m.name, m.level, m.msg])
                elif name in STATUS:
                    D[STATUS[name]].append([t, [[bytes(s.goal_info.goal_id.uuid).hex()[:8],
                                                 s.status] for s in m.status_list]])
                elif name == '/cmd_vel_slew/shape_status':
                    D['shape'].append([t, m.data])
                elif name == '/mission/incident':
                    D['incident'].append([t, m.data])
                elif name == '/behavior_tree_node/conditioned_route':
                    D['croute'].append([t, m.data])
                elif name == '/behavior_tree_node/mow_plan':
                    D['mow_plan'].append([t, m.data])
                elif name == '/behavior_tree_node/mow_progress':
                    try:
                        s = json.loads(m.data)
                    except ValueError:
                        continue
                    D['progress'].append([t, s.get('state'), s.get('sub_path'),
                                          s.get('sub_paths'), s.get('blade_on')])
    for k in D:
        D[k].sort(key=lambda r: r[0])
    with open(out, 'w') as fh:
        json.dump(D, fh)
    print('extracted', {k: len(v) for k, v in D.items()})


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
