#!/usr/bin/env python3
"""In-container integration smoke test for mower_navigation navigation.launch.py.

Run INSIDE the dev image (ROS_DOMAIN_ID=81, ROS_LOCALHOST_ONLY=1) next to the launch:

  ros2 launch mower_navigation navigation.launch.py > /tmp/launch.log 2>&1 &
  python3 src/mower_navigation/test/nav_stack_smoke.py --log /tmp/launch.log
  # negative control (launch with a local_costmap lacking bounds_keeper):
  python3 src/mower_navigation/test/nav_stack_smoke.py --log /tmp/launch.log --negative

It fakes the world: static TFs, a unicycle robot driven by the last /cmd_vel_nav, the
nav mask / terrain grids, an (optionally box-containing) /stereo_depth/points cloud and an
empty /bumper_cloud. Prints PASS/FAIL per check, exits non-zero on any failure.
"""

import argparse
import glob
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

LIFECYCLE_NODES = ["controller_server", "planner_server", "behavior_server",
                   "bt_navigator", "velocity_smoother", "collision_monitor"]
GRID_N = 600          # 60 m / 0.1 m
GRID_RES = 0.1
GRID_ORIGIN = -30.0

RESULTS = []          # (name, ok, detail)


def report(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print("%s  %s  %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def info(msg):
    print("      " + msg, flush=True)


def yaw_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return Quaternion(x=sr * cp * cy - cr * sp * sy, y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy, w=cr * cp * cy + sr * sp * sy)


class World(Node):
    """Fake robot + sensors + masks, and recorder of the command lanes."""

    def __init__(self):
        super().__init__("smoke_world")
        self.lock = threading.Lock()
        self.x = self.y = self.yaw = 0.0
        self.last_cmd = Twist()
        self.last_cmd_t = -1e9
        self.cur_v = 0.0
        self.cur_w = 0.0
        self.box = None            # Nx3 ndarray in odom
        self.last_tf_stamp = None
        # recordings
        self.pose_log = []         # (t, x, y, yaw)
        self.nav_log = []          # (t, vx, wz, smoothed_vx at that time, smoothed_wz)
        self.smoothed_last = (0.0, 0.0)
        self.raw_log = []          # (t, vx, wz)

        self.static_br = StaticTransformBroadcaster(self)
        self.tf_br = TransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        self.static_br.sendTransform([
            self._tf("map", "odom", 0, 0, 0, Quaternion(w=1.0), now),
            self._tf("base_link", "stereo_camera_optical", 0.466, 0.0, 0.240,
                     quat_from_rpy(-1.5547956, 0.0, -1.5707963), now),
            self._tf("base_link", "base_footprint", 0, 0, 0, Quaternion(w=1.0), now),
        ])

        self.create_subscription(Twist, "/cmd_vel_nav", self._on_nav, 10)
        self.create_subscription(Twist, "/cmd_vel_nav_smoothed", self._on_smoothed, 10)
        self.create_subscription(Twist, "/cmd_vel_nav_raw", self._on_raw, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mask_pub = self.create_publisher(OccupancyGrid, "/nav_keepout_mask", latched)
        self.terrain_pub = self.create_publisher(
            OccupancyGrid, "/map_server_node/terrain_cost", latched)
        self.cloud_pub = self.create_publisher(PointCloud2, "/stereo_depth/points", 5)
        self.bumper_pub = self.create_publisher(PointCloud2, "/bumper_cloud", 5)
        # The nav mask only reaches the global costmap through the KeepoutFilter: in a rolling
        # window the terrain_layer (a second StaticLayer, all zeros) overwrites the static_layer
        # (Humble ignores use_maximum there). Publish the filter info as map_server_node does.
        self.info_pub = self.create_publisher(CostmapFilterInfo, "/costmap_filter_info", latched)
        fi = CostmapFilterInfo()
        fi.header.frame_id = "map"
        fi.type = 0
        fi.filter_mask_topic = "/nav_keepout_mask"
        fi.base = 0.0
        fi.multiplier = 1.0
        self.info_pub.publish(fi)
        self.set_mask(None)
        self.terrain_pub.publish(self._grid(np.zeros((GRID_N, GRID_N), np.int8)))

        self.t_last = time.monotonic()
        self.create_timer(0.02, self._step)
        self.create_timer(0.2, self._publish_clouds)

    # ----- helpers
    @staticmethod
    def _tf(parent, child, x, y, z, q, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation = q
        return t

    def _grid(self, data2d):
        g = OccupancyGrid()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = GRID_RES
        g.info.width = GRID_N
        g.info.height = GRID_N
        g.info.origin.position.x = GRID_ORIGIN
        g.info.origin.position.y = GRID_ORIGIN
        g.info.origin.orientation.w = 1.0
        g.data = data2d.reshape(-1).tolist()
        return g

    def set_mask(self, gap):
        """gap None -> free map; else wall x in [2.0, 2.2] with a gap |y| < gap/2."""
        d = np.zeros((GRID_N, GRID_N), np.int8)   # d[row=y, col=x]
        if gap is not None:
            xs = GRID_ORIGIN + GRID_RES * np.arange(GRID_N)
            ys = GRID_ORIGIN + GRID_RES * (np.arange(GRID_N) + 0.5)
            cols = np.where((xs >= 2.0 - 1e-6) & (xs <= 2.2 + 1e-6))[0]
            n = int(gap / GRID_RES + 0.5)          # gap in whole cells (G=0.55 -> 0.6 m)
            lo, hi = 300 - (n + 1) // 2, 300 + n // 2 - 1
            rows = np.array([r for r in range(GRID_N) if not lo <= r <= hi])
            for c in cols:
                d[rows, c] = 100
        self.mask_pub.publish(self._grid(d))

    def teleport(self, x=0.0, y=0.0, yaw=0.0):
        with self.lock:
            self.x, self.y, self.yaw = x, y, yaw
            self.last_cmd_t = -1e9

    def set_box(self, cx, cy=0.0, sx=0.4, sy=0.4, z0=0.15, z1=0.5, spacing=0.05):
        """Box centred (cx, cy) in odom (None clears)."""
        if cx is None:
            with self.lock:
                self.box = None
            return
        xs = np.arange(cx - sx / 2, cx + sx / 2 + 1e-9, spacing)
        ys = np.arange(cy - sy / 2, cy + sy / 2 + 1e-9, spacing)
        zs = np.arange(z0, z1 + 1e-9, spacing)
        g = np.array(np.meshgrid(xs, ys, zs, indexing="ij")).reshape(3, -1).T
        with self.lock:
            self.box = g.astype(np.float32)

    def pose(self):
        with self.lock:
            return self.x, self.y, self.yaw

    # ----- callbacks
    def _on_nav(self, m):
        t = time.monotonic()
        with self.lock:
            self.last_cmd = m
            self.last_cmd_t = t
            self.nav_log.append((t, m.linear.x, m.angular.z) + self.smoothed_last)

    def _on_smoothed(self, m):
        with self.lock:
            self.smoothed_last = (m.linear.x, m.angular.z)

    def _on_raw(self, m):
        with self.lock:
            self.raw_log.append((time.monotonic(), m.linear.x, m.angular.z))

    def _step(self):
        t = time.monotonic()
        dt = min(t - self.t_last, 0.1)
        self.t_last = t
        with self.lock:
            if t - self.last_cmd_t > 0.6:
                v = w = 0.0
            else:
                v, w = self.last_cmd.linear.x, self.last_cmd.angular.z
            self.cur_v, self.cur_w = v, w
            self.yaw += w * dt
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            x, y, yaw = self.x, self.y, self.yaw
            self.pose_log.append((t, x, y, yaw))
        stamp = self.get_clock().now().to_msg()
        self.last_tf_stamp = stamp
        self.tf_br.sendTransform(self._tf("odom", "base_link", x, y, 0.0, yaw_quat(yaw), stamp))
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.orientation = yaw_quat(yaw)
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        self.odom_pub.publish(o)

    def _publish_clouds(self):
        if self.last_tf_stamp is None:
            return
        hdr = Header()
        hdr.stamp = self.last_tf_stamp      # = newest odom TF, so lookups never extrapolate
        hdr.frame_id = "base_link"
        with self.lock:
            box = None if self.box is None else self.box.copy()
            x, y, yaw = self.x, self.y, self.yaw
        pts = []
        if box is not None:
            dx, dy = box[:, 0] - x, box[:, 1] - y
            c, s = math.cos(yaw), math.sin(yaw)
            xb, yb = c * dx + s * dy, -s * dx + c * dy
            # crude camera frustum: ahead of the nose region, <= 3 m, +-0.6 rad from the camera
            vis = (xb > 0.3) & (np.hypot(xb - 0.466, yb) <= 3.0) & \
                  (np.abs(np.arctan2(yb, xb - 0.466)) < 0.6)
            pts = np.stack([xb[vis], yb[vis], box[vis, 2]], axis=1).astype(np.float32).tolist()
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts))
        self.bumper_pub.publish(point_cloud2.create_cloud_xyz32(hdr, []))


class Harness:
    def __init__(self, world, log_path):
        self.w = world
        self.log_path = log_path
        self.nav = ActionClient(world, NavigateToPose, "/navigate_to_pose")
        self.plan = ActionClient(world, ComputePathToPose, "/compute_path_to_pose")
        self.clear_local = world.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")
        self.cancel_all = world.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.cmd_pub = world.create_publisher(Twist, "/cmd_vel_nav_smoothed", 10)

    # ----- utils
    @staticmethod
    def wait(fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end:
                return False
            time.sleep(0.02)
        return True

    def log_tail(self, n=80):
        try:
            with open(self.log_path, errors="replace") as f:
                return "".join(f.readlines()[-n:])
        except OSError as e:
            return "<cannot read %s: %s>" % (self.log_path, e)

    def log_lines(self, needle):
        try:
            with open(self.log_path, errors="replace") as f:
                return [ln.rstrip() for ln in f if needle in ln]
        except OSError:
            return []

    @staticmethod
    def goal_msg(x, y, yaw=0.0):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        g.pose.pose.orientation = yaw_quat(yaw)
        return g

    def send_nav(self, x, y, yaw=0.0):
        """Returns (goal_handle or None, result_future or None)."""
        if not self.nav.wait_for_server(timeout_sec=30):
            return None, None
        f = self.nav.send_goal_async(self.goal_msg(x, y, yaw))
        if not self.wait(f, 20):
            return None, None
        gh = f.result()
        if not gh.accepted:
            return None, None
        return gh, gh.get_result_async()

    def cancel_everything(self):
        if self.cancel_all.wait_for_service(timeout_sec=5):
            f = self.cancel_all.call_async(CancelGoal.Request())
            self.wait(f, 5)
        time.sleep(1.0)

    def idle(self, seconds):
        time.sleep(seconds)

    # ----- check 1
    def wait_active(self, timeout=60.0):
        clients = {n: self.w.create_client(GetState, "/%s/get_state" % n) for n in LIFECYCLE_NODES}
        states = {n: "?" for n in LIFECYCLE_NODES}
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for n, c in clients.items():
                if states[n] == "active" or not c.service_is_ready():
                    continue
                f = c.call_async(GetState.Request())
                if self.wait(f, 2.0) and f.result() is not None:
                    states[n] = f.result().current_state.label
            if all(s == "active" for s in states.values()):
                break
            time.sleep(0.5)
        return states

    def check1(self):
        t0 = time.monotonic()
        states = self.wait_active()
        ok = all(s == "active" for s in states.values())
        report("1 lifecycle nodes active", ok,
               "%s (%.1fs)" % (states, time.monotonic() - t0))
        if not ok:
            print("---- last 80 lines of launch output ----\n" + self.log_tail(80))
        return ok

    # ----- CPU
    @staticmethod
    def pid_of(exe_name):
        for p in glob.glob("/proc/[0-9]*/cmdline"):
            try:
                with open(p, "rb") as f:
                    argv = f.read().split(b"\0")
                if argv and argv[0].endswith(b"/" + exe_name.encode()):
                    return int(p.split("/")[2])
            except OSError:
                pass
        return None

    @staticmethod
    def cpu_ticks(pid):
        try:
            with open("/proc/%d/stat" % pid) as f:
                rest = f.read().rsplit(")", 1)[1].split()
            return int(rest[11]) + int(rest[12])      # utime + stime
        except (OSError, IndexError, TypeError):
            return None

    # ----- check 2 (+ 7)
    def check2_7(self):
        w = self.w
        w.teleport(0, 0, 0)
        w.set_box(None)
        time.sleep(1.5)
        pids = {n: self.pid_of(n) for n in ("controller_server", "planner_server")}
        c0 = {n: self.cpu_ticks(p) for n, p in pids.items() if p}
        tw0 = time.monotonic()
        t0 = tw0
        gh, rf = self.send_nav(3.0, 0.5, 0.0)
        if gh is None:
            report("2 transit free field", False, "goal rejected / server unavailable")
            return
        done = self.wait(rf, 90)
        t1 = time.monotonic()
        c1 = {n: self.cpu_ticks(p) for n, p in pids.items() if p}
        status = rf.result().status if done else None
        if not done:
            self.cancel_everything()
        with w.lock:
            nav = [r for r in w.nav_log if t0 <= r[0] <= t1]
            pose = [r for r in w.pose_log if t0 <= r[0] <= t1]
        vmax = max([r[1] for r in nav], default=0.0)
        wmax = max([abs(r[2]) for r in nav], default=0.0)
        fx, fy = (pose[-1][1], pose[-1][2]) if pose else (float('nan'), float('nan'))
        ok = (status == GoalStatus.STATUS_SUCCEEDED and vmax > 0.05
              and wmax <= 0.3 + 1e-3 and vmax <= 0.3 + 1e-3)
        report("2 transit free field", ok,
               "status=%s t=%.1fs max_vx=%.3f max|wz|=%.3f final=(%.2f,%.2f)"
               % (status, t1 - t0, vmax, wmax, fx, fy))
        for n in c0:
            if c1.get(n) is not None and c0[n] is not None and t1 > tw0:
                pct = (c1[n] - c0[n]) / os.sysconf("SC_CLK_TCK") / (t1 - tw0) * 100.0
                report("7 CPU %s" % n, True, "%.1f %% of one core (indicative, x86)" % pct)
        if not c0:
            report("7 CPU", False, "pids not found")

    # ----- check 3
    def plan_to(self, x, y, planner_id):
        if not self.plan.wait_for_server(timeout_sec=30):
            return None, "server unavailable"
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x = x
        g.goal.pose.position.y = y
        g.goal.pose.orientation.w = 1.0
        g.planner_id = planner_id
        g.use_start = False
        f = self.plan.send_goal_async(g)
        if not self.wait(f, 20) or not f.result().accepted:
            return None, "not accepted"
        rf = f.result().get_result_async()
        if not self.wait(rf, 20):
            return None, "timeout"
        r = rf.result()
        if r.status != GoalStatus.STATUS_SUCCEEDED or len(r.result.path.poses) == 0:
            return None, "status=%d (empty path)" % r.status
        return [(p.pose.position.x, p.pose.position.y) for p in r.result.path.poses], "ok"

    @staticmethod
    def gap_crossing(path):
        pts = [(x, y) for x, y in path if 2.0 <= x <= 2.2]
        return pts

    def check3(self):
        w = self.w
        w.teleport(0, 0, 0)
        w.set_box(None)
        w.set_mask(None)
        time.sleep(2.5)
        p, why = self.plan_to(4.0, 0.0, "GridBased")
        report("3a GridBased free (0,0)->(4,0)", p is not None,
               "%s, %d poses" % (why, len(p) if p else 0))
        for gap in (0.55, 0.5, 0.4, 0.9):
            w.set_mask(gap)
            time.sleep(2.5)
            p, why = self.plan_to(4.0, 0.0, "GridBased")
            eff = int(gap / GRID_RES + 0.5) * GRID_RES
            if eff <= 0.6:
                pn, whyn = self.plan_to(4.0, 0.0, "GridBasedNavFn")
                if pn is None:
                    info("INFO GridBasedNavFn gap=%.2f: no path (%s)" % (gap, whyn))
                else:
                    cr = self.gap_crossing(pn)
                    ymax = max((abs(y) for _, y in cr), default=None)
                    info("INFO GridBasedNavFn gap=%.2f: path of %d poses, wall x-range max|y|=%s -> %s"
                         % (gap, len(pn), None if ymax is None else "%.2f" % ymax,
                            "through the gap (old bug)" if ymax is not None and ymax < 0.5
                            else "NOT through the gap"))

            if eff < 0.58:
                if p is None:
                    report("3b GridBased gap=%.2f (%.1f m cells)" % (gap, int(gap / GRID_RES + 0.5) * GRID_RES), True, "planning failed as expected (%s)" % why)
                else:
                    cr = self.gap_crossing(p)
                    ymin = min((abs(y) for _, y in cr), default=None)
                    ok = bool(cr) and ymin is not None and ymin > 1.0
                    report("3b GridBased gap=%.2f (%.1f m cells)" % (gap, int(gap / GRID_RES + 0.5) * GRID_RES), ok,
                           "path returned, %d poses, min|y| in wall x-range=%s (need >1)"
                           % (len(p), None if ymin is None else "%.2f" % ymin))
            elif eff < 0.7:
                # 0.6 m in 0.1 m cells vs the 0.58 m padded body: one cell of slack, which the
                # outline check on the 0.1 m grid may or may not grant. Informational only.
                info("INFO GridBased gap=%.2f (%.1f m cells, body 0.58): %s"
                     % (gap, eff, "no path" if p is None else "path of %d poses" % len(p)))
            else:
                if p is None:
                    report("3c GridBased gap=%.2f" % gap, False, "no path: %s" % why)
                else:
                    cr = self.gap_crossing(p)
                    ymax = max((abs(y) for _, y in cr), default=9.9)
                    report("3c GridBased gap=%.2f" % gap, bool(cr) and ymax < 0.5,
                           "%d poses, wall x-range max|y|=%.2f" % (len(p), ymax))
        w.set_mask(None)
        time.sleep(2.5)
        p, why = self.plan_to(4.0, 0.0, "GridBased")
        report("3d GridBased after restoring free map", p is not None, why)

    # ----- check 4
    def check4(self):
        w = self.w
        w.teleport(0, 0, 0)
        w.set_box(None)
        w.set_mask(None)
        time.sleep(2.0)
        if not self.clear_local.wait_for_service(timeout_sec=10):
            report("4 clear local costmap", False, "service not available")
            return
        f = self.clear_local.call_async(ClearEntireCostmap.Request())
        ok_call = self.wait(f, 10)
        time.sleep(1.0)
        t0 = time.monotonic()
        gh, rf = self.send_nav(2.0, 0.0, 0.0)
        if gh is None:
            report("4 clear -> navigate", False, "goal rejected")
            return
        first = None
        while time.monotonic() - t0 < 10.0:
            with w.lock:
                for r in w.nav_log:
                    if r[0] >= t0 and r[1] > 0.05:
                        first = r[0] - t0
                        break
            if first is not None:
                break
            time.sleep(0.1)
        done = self.wait(rf, 90)
        status = rf.result().status if done else None
        if not done:
            self.cancel_everything()
        ok = ok_call and first is not None and status == GoalStatus.STATUS_SUCCEEDED
        report("4 clear local costmap -> navigate 2 m", ok,
               "clear_call=%s first vx>0.05 after %s s, status=%s"
               % (ok_call, None if first is None else "%.2f" % first, status))

    def check4_negative(self):
        """Run against a launch whose local_costmap lacks bounds_keeper: expect NO cmd_vel."""
        w = self.w
        w.teleport(0, 0, 0)
        w.set_box(None)
        time.sleep(2.0)
        f = self.clear_local.call_async(ClearEntireCostmap.Request())
        self.wait(f, 10)
        time.sleep(1.0)
        t0 = time.monotonic()
        gh, rf = self.send_nav(2.0, 0.0, 0.0)
        time.sleep(12.0)
        with w.lock:
            vmax = max([r[1] for r in w.nav_log if r[0] >= t0], default=0.0)
            px = w.x
        state = rf.done() and rf.result().status if rf is not None else None
        self.cancel_everything()
        bug = vmax <= 0.05
        report("4N negative control (no bounds_keeper)", bug,
               "max cmd_vel_nav vx after clear=%.3f, robot x=%.2f, goal status at 12 s=%s -> %s"
               % (vmax, px, state, "BUG REPRODUCED (no cmd_vel)" if bug else "controller still drove (bug NOT reproduced)"))

    # ----- check 5
    def check5(self):
        w = self.w
        w.teleport(0, 0, 0)
        w.set_mask(None)
        w.set_box(1.8, 0.0)
        n_log0 = len(self.log_lines("Robot to"))
        time.sleep(2.0)
        t0 = time.monotonic()
        gh, rf = self.send_nav(4.0, 0.0, 0.0)
        if gh is None:
            report("5 obstacle transit", False, "goal rejected")
            return
        done = self.wait(rf, 150)
        t1 = time.monotonic()
        status = rf.result().status if done else None
        if not done:
            self.cancel_everything()
        with w.lock:
            pose = [r for r in w.pose_log if t0 <= r[0] <= t1]
            nav = [r for r in w.nav_log if t0 <= r[0] <= t1]
        maxy = max([abs(r[2]) for r in pose], default=0.0)
        mind = min([math.hypot(r[1] - 1.8, r[2] - 0.0) for r in pose], default=9.9)
        reduced = [r for r in nav if r[3] > 0.02 and r[1] < 0.9 * r[3]]
        ok = status == GoalStatus.STATUS_SUCCEEDED
        report("5 obstacle transit (box at 1.8,0 -> goal 4,0)", ok,
               "status=%s t=%.1fs max|y|=%.2f min dist(box centre, base_link)=%.2f final=(%.2f,%.2f)"
               % (status, t1 - t0, maxy, mind, pose[-1][1] if pose else float('nan'),
                  pose[-1][2] if pose else float('nan')))
        info("collision_monitor reduced command (nav < 0.9*smoothed): %d of %d nav samples" % (
            len(reduced), len(nav)))
        for r in reduced[:10]:
            info("  t+%.2f nav_vx=%.3f smoothed_vx=%.3f" % (r[0] - t0, r[1], r[3]))
        lines = self.log_lines("Robot to")
        info("collision_monitor 'Robot to ...' log lines (%d total, %d new in this check):"
             % (len(lines), len(lines) - n_log0))
        for ln in lines[-12:]:
            info("  " + ln)
        w.set_box(None)

    # ----- check 6
    def drive(self, vx, wz, seconds, hz=20.0):
        """Publish on /cmd_vel_nav_smoothed; returns (t0, t1)."""
        t0 = time.monotonic()
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        while time.monotonic() - t0 < seconds:
            self.cmd_pub.publish(msg)
            time.sleep(1.0 / hz)
        return t0, time.monotonic()

    def check6(self):
        w = self.w
        self.cancel_everything()
        w.set_mask(None)
        time.sleep(1.5)

        def place():
            w.teleport(0, 0, 0)
            w.set_box(0.67, 0.0, sx=0.1, sy=0.4)     # near face x=0.62, points in [0.62, 0.72]
            time.sleep(1.5)

        place()
        t0, t1 = self.drive(0.2, 0.0, 1.5)
        with w.lock:
            vx = [r[1] for r in w.nav_log if t0 <= r[0] <= t1]
        m = max([abs(v) for v in vx], default=0.0)
        report("6a forward +0.2 into stop zone is stopped", m < 0.02,
               "max|nav vx|=%.3f over %d msgs (smoothed 0.2 in)" % (m, len(vx)))
        time.sleep(1.0)
        place()
        t0, t1 = self.drive(-0.15, 0.0, 1.5)
        with w.lock:
            rows = [r for r in w.nav_log if t0 <= r[0] <= t1]
        t_ok = next((r[0] - t0 for r in rows if abs(r[1] + 0.15) < 0.02), None)
        late = [r[1] for r in rows if r[0] - t0 > 0.5]
        ok = t_ok is not None and t_ok <= 0.5 and all(abs(v + 0.15) < 0.02 for v in late)
        report("6b reverse -0.15 passes", ok,
               "first vx~-0.15 after %s s; vx after 0.5s: min=%.3f max=%.3f"
               % (None if t_ok is None else "%.2f" % t_ok, min(late, default=float('nan')),
                  max(late, default=float('nan'))))
        time.sleep(1.0)
        place()
        t0, t1 = self.drive(0.0, 0.3, 1.5)
        with w.lock:
            rows = [r for r in w.nav_log if t0 <= r[0] <= t1]
        late = [r[2] for r in rows if r[0] - t0 > 0.3]
        ok = bool(late) and all(abs(v - 0.3) < 0.03 for v in late)
        report("6c pivot wz=0.3 passes", ok,
               "wz after 0.3s: min=%.3f max=%.3f (n=%d)"
               % (min(late, default=float('nan')), max(late, default=float('nan')), len(late)))
        w.set_box(None)
        self.drive(0.0, 0.0, 0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="/tmp/launch.log", help="launch output file (for dumps)")
    ap.add_argument("--checks", default="1,2,3,4,5,6", help="comma list (7 runs with 2)")
    ap.add_argument("--negative", action="store_true",
                    help="only the check-4 negative control (launch w/o bounds_keeper)")
    a = ap.parse_args()

    rclpy.init()
    world = World()
    ex = SingleThreadedExecutor()
    ex.add_node(world)
    threading.Thread(target=ex.spin, daemon=True).start()
    h = Harness(world, a.log)
    try:
        if not h.check1():
            if not a.negative:
                raise SystemExit(1)
        if a.negative:
            h.check4_negative()
        else:
            todo = a.checks.split(",")
            for k, fn in (("2", h.check2_7), ("3", h.check3), ("4", h.check4),
                          ("5", h.check5), ("6", h.check6)):
                if k in todo:
                    try:
                        fn()
                    except Exception as e:      # noqa: BLE001 - report and carry on
                        import traceback
                        traceback.print_exc()
                        report("%s (exception)" % k, False, repr(e))
    finally:
        failed = [r for r in RESULTS if not r[1]]
        print("\nSUMMARY: %d checks, %d failed" % (len(RESULTS), len(failed)), flush=True)
        for r in failed:
            print("  FAILED: %s %s" % (r[0], r[2]))
        rclpy.try_shutdown()
    os._exit(1 if [r for r in RESULTS if not r[1]] else 0)


if __name__ == "__main__":
    main()
