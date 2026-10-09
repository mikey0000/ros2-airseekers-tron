"""Dual EKF smoke test (2026-10-09): the real robot_localization ekf_node twice, synthetic inputs.

Runs config/ekf_dual.yaml as ekf_node (odom) + ekf_map (map, /odometry/filtered_map) exactly
as launch/nav2.launch.py does, feeds /odom (0.2 m/s forward), /imu/data_aligned (yaw 0) and
/odometry/gps in the map frame offset by +1.0 m in y from where the wheels put the robot
(a GPS correction), and checks REP-105: the correction ends up in map -> odom, the odom pose
never sees it (continuous), and /odometry/filtered_map follows the GPS. No rosbag, no
navsat (it would need a NavSatFix stream + datum; /odometry/gps is injected directly).

Needs real rclpy, robot_localization and the built mower_localization share (dev image);
skipped otherwise. Isolated by conftest (ROS_LOCALHOST_ONLY, private domain).
"""

import math
import os
import shutil
import subprocess
import time

import pytest

rclpy = pytest.importorskip('rclpy')
if getattr(rclpy, '__file__', None) is None:   # conftest shim
    pytest.skip('needs real rclpy', allow_module_level=True)


def _ekf_yaml():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('mower_localization'), 'config',
                            'ekf_dual.yaml')
        get_package_share_directory('robot_localization')
    except Exception:  # noqa: BLE001
        return None
    return path if os.path.exists(path) else None


EKF_YAML = _ekf_yaml()
pytestmark = pytest.mark.skipif(EKF_YAML is None or shutil.which('ros2') is None,
                                reason='needs robot_localization + built mower_localization')


def _spawn(name, remaps):
    cmd = ['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
           '-r', '__node:=%s' % name, '--params-file', EKF_YAML, '--log-level', 'warn']
    for a, b in remaps:
        cmd += ['-r', '%s:=%s' % (a, b)]
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)


def test_dual_ekf_puts_gps_correction_in_map_to_odom():
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    import tf2_ros

    procs = [_spawn('ekf_node', []),
             _spawn('ekf_map', [('odometry/filtered', '/odometry/filtered_map')])]
    node = Node('dual_ekf_smoke_feeder')
    try:
        odom_pub = node.create_publisher(Odometry, '/odom', 10)
        imu_pub = node.create_publisher(Imu, '/imu/data_aligned', 10)
        gps_pub = node.create_publisher(Odometry, '/odometry/gps', 10)
        got = {'odom': [], 'map': []}
        node.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: got['odom'].append(m), 50)
        node.create_subscription(Odometry, '/odometry/filtered_map',
                                 lambda m: got['map'].append(m), 50)
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, node, spin_thread=False)

        v, t0 = 0.2, time.monotonic()
        last_gps = 0.0
        while time.monotonic() - t0 < 12.0:
            now = node.get_clock().now().to_msg()
            el = time.monotonic() - t0
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_link'
            o.twist.twist.linear.x = v
            cov = [0.0] * 36
            for i in range(6):
                cov[i * 7] = 0.02
            o.twist.covariance = cov
            odom_pub.publish(o)
            m = Imu()
            m.header.stamp, m.header.frame_id = now, 'base_link'
            m.orientation.w = 1.0
            m.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.001]
            m.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            imu_pub.publish(m)
            if el > 2.0 and el - last_gps >= 0.1:
                last_gps = el
                g = Odometry()
                g.header.stamp, g.header.frame_id, g.child_frame_id = now, 'map', 'base_link'
                g.pose.pose.position.x = v * el
                g.pose.pose.position.y = 1.0          # the GPS says: 1 m left of dead reckoning
                g.pose.pose.orientation.w = 1.0
                pc = [0.0] * 36
                pc[0] = pc[7] = 0.0009             # RTK fixed (gps_gate floor 3 cm)
                pc[14] = pc[21] = pc[28] = pc[35] = 1e6
                g.pose.covariance = pc
                gps_pub.publish(g)
            rclpy.spin_once(node, timeout_sec=0.02)

        assert got['odom'] and got['map'], 'both filters publish'
        assert got['odom'][-1].header.frame_id == 'odom'
        assert got['map'][-1].header.frame_id == 'map'
        # odom filter: no GPS -> y stays at dead reckoning (0), x follows the wheels
        ys = [m.pose.pose.position.y for m in got['odom']]
        assert max(abs(y) for y in ys) < 0.05, 'odom pose must not see the GPS correction'
        assert got['odom'][-1].pose.pose.position.x > 1.0
        # map filter follows the GPS
        assert abs(got['map'][-1].pose.pose.position.y - 1.0) < 0.15
        # ... and the difference is map -> odom
        tr = None
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                tr = buf.lookup_transform('map', 'odom', rclpy.time.Time())
                break
            except Exception:  # noqa: BLE001
                continue
        assert tr is not None, 'ekf_map publishes map -> odom'
        assert abs(tr.transform.translation.y - 1.0) < 0.15
        assert abs(tr.transform.translation.x) < 0.3
        # continuity: no step in the odom pose larger than a few control periods of motion
        xs = [m.pose.pose.position.x for m in got['odom']]
        assert max(abs(b - a) for a, b in zip(xs, xs[1:])) < 0.1
        assert all(math.isfinite(x) for x in xs)
    finally:
        node.destroy_node()
        for p in procs:
            try:
                os.killpg(p.pid, 2)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, 9)
