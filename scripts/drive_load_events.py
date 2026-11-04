#!/usr/bin/env python3
"""Drive-load events in a mow bag: the motor-current DriveLoadDetector, log-only (B11).

``mower_control/motor_load.py`` (high |left| + |right| drive current while the ground speed
stays far below the commanded speed) is not wired into the robot. This replays it over a
recorded mow so its hits can be compared with the stuck guard's ``/stuck`` latches and the
field notes before anyone wires it: it has no effect on the robot. The rule uses the sum of
both drive currents and the mean |rpm|, so the left/right motor label question (B10) does not
change the result; the output prints only the sum.

Inputs (bag_recorder records all of them during a mow): ``/mower_sensor_info`` (MotorInfo
current in 10 mA, rpm; a new frame about once a second), ``/cmd_vel`` (commanded linear) and
``/fix`` (ground speed from consecutive positions over ``--window`` seconds).

Run in the dev image, like wheel_cal.py:

    docker run --rm --network none -v "$PWD":/work:ro -v /path/to/bag:/bag:ro \\
      mower:jazzy-dev-amd64 bash -c 'source /opt/ros/jazzy/setup.bash && \\
      source /work/install/setup.bash && python3 /work/scripts/drive_load_events.py /bag'
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src',
                                'mower_control'))
from wheel_bag_metrics import bag_files  # noqa: E402
from mower_control.motor_load import DriveLoadDetector, replay  # noqa: E402

MOTOR_CURRENT_A_PER_UNIT = 0.01     # MotorInfo.current is 10 mA (mcu_node motor_msg_fields)
EARTH_R = 6378137.0


def ground_speed(fix, window_s=1.0):
    """[(t, lat, lon, status)] -> [(t, speed m/s)] over a trailing ``window_s`` window."""
    if not fix:
        return []
    lat0 = math.radians(fix[0][1])
    xy = [(t, math.radians(lon - fix[0][2]) * EARTH_R * math.cos(lat0),
           math.radians(lat - fix[0][1]) * EARTH_R) for t, lat, lon, _st in fix]
    out, j = [], 0
    for i, (t, x, y) in enumerate(xy):
        while j < i and t - xy[j + 1][0] >= window_s:
            j += 1
        dt = t - xy[j][0]
        if dt >= 0.5 * window_s:
            out.append((t, math.hypot(x - xy[j][1], y - xy[j][2]) / dt))
    return out


def motion_rows(cmd, speed):
    """Commanded linear per /cmd_vel row, with the latest ground speed at or before it."""
    out, k = [], -1
    for t, v, _w in cmd:
        while k + 1 < len(speed) and speed[k + 1][0] <= t:
            k += 1
        if k >= 0:
            out.append((t, v, speed[k][1]))
    return out


def motor_frames(msi):
    """MowerSensorInfo rows (t, cutting, l_rpm, r_rpm, l_cur, r_cur) -> NEW frames in A."""
    out, last = [], None
    for t, _cut, lr, rr, lc, rc in msi:
        key = (lr, rr, lc, rc)
        if key == last:
            continue
        last = key
        out.append((t, lc * MOTOR_CURRENT_A_PER_UNIT, rc * MOTOR_CURRENT_A_PER_UNIT, lr, rr))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag', help='bag split, pinned mow/incident dir or session dir')
    ap.add_argument('--window', type=float, default=1.0, help='s, ground speed window')
    a = ap.parse_args(argv)
    from wheel_cal import read_bag   # needs numpy + rclpy (dev image)
    paths = bag_files(a.bag)
    if not paths:
        return 1
    d = {k: v.tolist() for k, v in read_bag(paths).items()}
    if not d.get('msi') or not d.get('cmd') or not d.get('fix'):
        print('need /mower_sensor_info, /cmd_vel and /fix in the bag')
        return 1
    frames = motor_frames(d['msi'])
    events = replay(motion_rows(d['cmd'], ground_speed(d['fix'], a.window)), frames,
                    DriveLoadDetector())
    t0 = d['cmd'][0][0]
    print('%d motor frames over %.0f s, %d drive-load event(s)' % (
        len(frames), d['cmd'][-1][0] - t0, len(events)))
    for t, e in events:
        print('  %s (+%.1f s)  %-5s  load %.1f A  cmd %.3f m/s  ground %.3f m/s' % (
            time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(t)), t - t0, e['kind'],
            e['load_a'], e['cmd_mps'], e['ground_mps']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
