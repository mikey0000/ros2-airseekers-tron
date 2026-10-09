# SPDX-License-Identifier: Apache-2.0
"""mower_sim: kinematic stand-in for the Tron's hardware drivers.

Replaces mcu_node, wit_node, um960_node, stereo_depth and the rear camera with
one rclpy node driven by a world YAML (mower_sim/world.py). Everything above the
drivers (twist_mux, cmd_vel_slew, bumper_controller, Nav2, mower_map,
coverage, docking, mission, gui_bridge) runs unmodified.

Driver contract reproduced (see docs/simulation.md for the full table):

  sub   /cmd_vel (Twist)                    MCU clamp +-0.3 / +-0.3, 0.06 m/s wheel floor,
                                            0.5 s timeout, wheel lag; /estop_request (Bool)
  pub   /odom 50 Hz, /battery 1 Hz, /mower_base/status 20 Hz, /mower_sensor_info 10 Hz,
        /estop 10 Hz, /rain + /cutter/height_mm (latched)          [mcu_node]
        /imu/data 50 Hz                                            [wit_node]
        /fix, /fix_status, /vel 10 Hz (RTK fixed)                  [um960_node]
        /stereo_depth/points + /clear_points 5 Hz, sensor QoS      [stereo_depth]
        /rear_camera/image_raw + camera_info 10 Hz, on demand      [rear camera]
  srv   /cutter_control, /charging, /clear_estop, /cutter_off, /cutter/set_height
  localization:=sim (default) also publishes what the localization stack would:
        TF odom->base_link, /odometry/filtered, /heading_aligner/status (aligned, dock).
  sim   /sim/ground_truth (Odometry, map), /sim/status (JSON 10 Hz), /sim/markers
"""

from __future__ import annotations

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import (BatteryState, CameraInfo, Image, Imu, NavSatFix,
                             NavSatStatus, PointCloud2, PointField)
from std_msgs.msg import Bool, Int16, String
from std_srvs.srv import Empty, Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from mower_interfaces.msg import MowerBaseDevStatus, MowerSensorInfo
from mower_interfaces.srv import ChargingControl, CutterControl, SetCutterHeight

from .kinematics import DiffDrive, DriveParams
from .marker import RearCamera, RearCameraParams
from .sensors import StereoParams, StereoRaycaster, bumper_sides
from .world import enu_to_latlon, load_world

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def _quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_cloud(points, frame_id, stamp) -> PointCloud2:
    """x/y/z float32, point_step 12 (stereo_depth layout)."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = int(len(points))
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = points.astype('<f4').tobytes()
    return msg


class SimNode(Node):
    def __init__(self):
        super().__init__('mower_sim')
        d = self.declare_parameter
        d('world_file', '')
        d('localization', 'sim')            # sim: publish TF/filtered; ekf: real localization
        d('physics_rate_hz', 50.0)
        d('stereo', True)
        d('stereo_rate_hz', 5.0)
        d('stereo_noise_m', 0.0)
        d('rear_camera', True)
        d('rear_camera_rate_hz', 10.0)
        d('rear_marker_size', 0.04)           # = mower_docking marker_size
        d('linear_max', 0.3)
        d('angular_max', 0.3)
        d('min_wheel_speed_mps', 0.06)
        d('wheel_tau_s', 0.15)
        d('cmd_vel_timeout', 0.5)
        d('battery_drain_per_s', 0.00005)
        d('battery_charge_per_s', 0.001)
        d('gps_sigma_m', 0.015)
        d('bumper_hold_s', 0.3)
        gp = self.get_parameter
        path = gp('world_file').value
        if not path or not os.path.isfile(path):
            raise RuntimeError('mower_sim: world_file %r not found' % path)
        self.world = load_world(path)
        self.loc_sim = str(gp('localization').value).lower() == 'sim'
        self.drive = DiffDrive(self.world, DriveParams(
            linear_max=float(gp('linear_max').value), angular_max=float(gp('angular_max').value),
            min_wheel_speed=float(gp('min_wheel_speed_mps').value),
            wheel_tau_s=float(gp('wheel_tau_s').value),
            cmd_timeout_s=float(gp('cmd_vel_timeout').value)))
        self.stereo = StereoRaycaster(StereoParams(noise_m=float(gp('stereo_noise_m').value)))
        self.rear = RearCamera(RearCameraParams(
            marker_size=float(gp('rear_marker_size').value)))
        self.t0 = time.monotonic()
        self.dt = 1.0 / float(gp('physics_rate_hz').value)
        self.tick_n = 0
        self._last_t = None
        # MCU state
        self.charging_enabled = self.world.start_docked
        self.cutting = False
        self.height_mm = 50
        self.estop_latched = False
        self.battery = self.world.battery_percentage
        self.bumper_until = 0.0
        self.bumper_lr = (False, False)
        self.min_clearance = math.inf
        self.last_contact = ''

        # --- publishers (driver contract) ---
        P = self.create_publisher
        self.pub_odom = P(Odometry, '/odom', 10)
        self.pub_batt = P(BatteryState, '/battery', 10)
        self.pub_status = P(MowerBaseDevStatus, '/mower_base/status', 10)
        self.pub_sensor = P(MowerSensorInfo, '/mower_sensor_info', 10)
        self.pub_estop = P(Bool, '/estop', 10)
        self.pub_rain = P(Bool, '/rain', LATCHED)
        self.pub_height = P(Int16, '/cutter/height_mm', LATCHED)
        self.pub_imu = P(Imu, '/imu/data', 10)
        self.pub_fix = P(NavSatFix, '/fix', 10)
        self.pub_fix_status = P(String, '/fix_status', 10)
        self.pub_vel = P(TwistStamped, '/vel', 10)
        self.pub_points = P(PointCloud2, '/stereo_depth/points', qos_profile_sensor_data)
        self.pub_clear = P(PointCloud2, '/stereo_depth/clear_points', qos_profile_sensor_data)
        # 2026-10-09: the global costmap reads stereo_depth's map-frame copies; the sim sends the
        # same camera-frame clouds there (no CPU load in the sim, so the TF wait is harmless)
        self.pub_points_map = P(PointCloud2, '/stereo_depth/points_map', qos_profile_sensor_data)
        self.pub_clear_map = P(PointCloud2, '/stereo_depth/clear_points_map',
                               qos_profile_sensor_data)
        self.pub_image = P(Image, '/rear_camera/image_raw', qos_profile_sensor_data)
        self.pub_info = P(CameraInfo, '/rear_camera/camera_info', qos_profile_sensor_data)
        self.pub_truth = P(Odometry, '/sim/ground_truth', 10)
        self.pub_sim = P(String, '/sim/status', 10)
        self.pub_markers = P(MarkerArray, '/sim/markers', 10)
        if self.loc_sim:
            self.tf = TransformBroadcaster(self)
            self.pub_filtered = P(Odometry, '/odometry/filtered', 10)
            self.pub_heading = P(String, '/heading_aligner/status', LATCHED)
        # --- inputs ---
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Bool, '/estop_request', self._on_estop_request, 10)
        S = self.create_service
        S(CutterControl, '/cutter_control', self._srv_cutter)
        S(ChargingControl, '/charging', self._srv_charging)
        S(Empty, '/clear_estop', self._srv_clear_estop)
        S(Trigger, '/cutter_off', self._srv_cutter_off)
        S(SetCutterHeight, '/cutter/set_height', self._srv_height)

        self.pub_rain.publish(Bool(data=False))
        self.pub_height.publish(Int16(data=self.height_mm))
        self.create_timer(self.dt, self._tick)
        if gp('stereo').value:
            self.create_timer(1.0 / float(gp('stereo_rate_hz').value), self._stereo_tick)
        if gp('rear_camera').value:
            self.create_timer(1.0 / float(gp('rear_camera_rate_hz').value), self._rear_tick)
        self.create_timer(1.0, self._slow_tick)
        self.get_logger().info(
            'world %s: %d obstacles, start (%.2f, %.2f, %.2f)%s, localization=%s'
            % (self.world.name, len(self.world.obstacles), self.drive.pose.x, self.drive.pose.y,
               self.drive.pose.yaw, ' docked' if self.world.start_docked else '',
               'sim' if self.loc_sim else 'ekf'))

    # ------------------------------------------------------------------ inputs
    def sim_time(self):
        return time.monotonic() - self.t0

    def _on_cmd(self, msg: Twist):
        self.drive.command(msg.linear.x, msg.angular.z)

    def _on_estop_request(self, msg: Bool):
        if msg.data:
            self.estop_latched = True
            self.cutting = False

    def _srv_cutter(self, req, resp):
        if self.estop_latched and req.cutter.enable:
            resp.result = False
            return resp
        self.cutting = bool(req.cutter.enable)
        if req.height.enable and req.height.position:
            self.height_mm = max(30, min(90, int(req.height.position)))
            self.pub_height.publish(Int16(data=self.height_mm))
        resp.result = True
        return resp

    def _srv_charging(self, req, resp):
        self.charging_enabled = bool(req.enable_charging)
        self.get_logger().info('/charging enable_charging=%s' % self.charging_enabled)
        return resp

    def _srv_clear_estop(self, req, resp):
        self.estop_latched = False
        return resp

    def _srv_cutter_off(self, req, resp):
        self.cutting = False
        resp.success = True
        resp.message = 'cutter off'
        return resp

    def _srv_height(self, req, resp):
        if self.estop_latched:
            resp.ok, resp.message, resp.height_mm = False, 'e-stop latched', self.height_mm
            return resp
        self.height_mm = max(30, min(90, int(req.height_mm)))
        self.pub_height.publish(Int16(data=self.height_mm))
        resp.ok, resp.message, resp.height_mm = True, 'ok', self.height_mm
        return resp

    # ------------------------------------------------------------------ physics
    def _tick(self):
        t = self.sim_time()
        # integrate the real elapsed time (a late timer must not slow the robot down),
        # in steps of at most the nominal period
        elapsed = min(0.5, t - self._last_t) if self._last_t is not None else self.dt
        self._last_t = t
        self.drive.enabled = not self.estop_latched
        n = max(1, int(math.ceil(elapsed / self.dt - 1e-6)))
        contacts = []
        for i in range(n):
            res = self.drive.step(elapsed / n, t - elapsed + (i + 1) * elapsed / n)
            contacts = res.contacts or contacts
        res.contacts = contacts
        pose = self.drive.pose
        if res.contacts:
            lr = bumper_sides(res.contacts, pose.x, pose.y, pose.yaw, t)
            if lr[0] or lr[1]:
                self.bumper_lr = lr
                self.bumper_until = t + float(self.get_parameter('bumper_hold_s').value)
            self.last_contact = ','.join(o.name for o in res.contacts)
        self.min_clearance = min(self.min_clearance, self.world.clearance(
            pose.x, pose.y, pose.yaw, t))
        stamp = self.get_clock().now().to_msg()
        self.tick_n += 1

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = pose.x, pose.y
        q = _quat(pose.yaw)
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.drive.v, self.drive.w
        for i, v in enumerate((0.001, 0.001, 1e6, 1e6, 1e6, 0.01)):
            odom.pose.covariance[7 * i] = v
        for i, v in enumerate((0.001, 0.001, 1e6, 1e6, 1e6, 0.01)):
            odom.twist.covariance[7 * i] = v
        self.pub_odom.publish(odom)
        if self.loc_sim:
            self.pub_filtered.publish(odom)
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = pose.x, pose.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)
        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id, truth.child_frame_id = 'map', 'base_link'
        truth.pose.pose = odom.pose.pose
        truth.twist.twist = odom.twist.twist
        self.pub_truth.publish(truth)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation = odom.pose.pose.orientation
        imu.orientation_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0]
        imu.angular_velocity.z = self.drive.w
        imu.angular_velocity_covariance = [0.005, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.005]
        imu.linear_acceleration.z = 9.81
        imu.linear_acceleration_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
        self.pub_imu.publish(imu)

        if self.tick_n % max(1, int(round(0.05 / self.dt))) == 0:
            self._publish_status(stamp, t)
        if self.tick_n % max(1, int(round(0.1 / self.dt))) == 0:
            self._publish_10hz(stamp, t)

    def _docked(self):
        return self.drive.on_dock()

    def _publish_status(self, stamp, t):
        docked = self._docked()
        bl, br = self.bumper_lr if t < self.bumper_until else (False, False)
        s = MowerBaseDevStatus()
        s.header.stamp = stamp
        s.is_cutting = self.cutting
        s.is_moving = abs(self.drive.v) > 0.01 or abs(self.drive.w) > 0.01
        s.is_cmd_moving = self.drive.cmd_age < 0.5 and (self.drive.cmd[0] != 0.0
                                                       or self.drive.cmd[1] != 0.0)
        s.is_docking_done = docked
        s.is_charging = docked and self.charging_enabled
        s.bumper_routing_enabled = True
        s.stop_triggered = self.estop_latched
        s.bumper_triggered = bl or br
        s.left_bumper_triggered = bl
        s.right_bumper_triggered = br
        self.pub_status.publish(s)
        self._status = s

    def _publish_10hz(self, stamp, t):
        s = getattr(self, '_status', None)
        if s is None:
            return
        info = MowerSensorInfo()
        info.header.stamp = stamp
        info.cutter_board_version = info.chassis_board_version = 'sim'
        info.rtk_board_version = info.mower_package_version = 'sim'
        info.bumper_triggered = s.bumper_triggered
        info.stop_triggered = s.stop_triggered
        info.is_cutting, info.is_moving, info.is_cmd_moving = s.is_cutting, s.is_moving, \
            s.is_cmd_moving
        info.is_charging = info.is_docking_done = s.is_docking_done
        info.bumper_routing_enabled = True
        info.cutter_size = MowerSensorInfo.CUTTER_SIZE_SMALL
        info.cutter_motor.speed_rpm = 2800 if self.cutting else 0
        info.left_motor.speed_rpm = int(self.drive.left * 300)
        info.right_motor.speed_rpm = int(self.drive.right * 300)
        self.pub_sensor.publish(info)
        self.pub_estop.publish(Bool(data=self.estop_latched))

        pose = self.drive.pose
        lat, lon = enu_to_latlon(pose.x, pose.y, self.world.datum_lat, self.world.datum_lon)
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
                              | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO)
        fix.latitude, fix.longitude, fix.altitude = lat, lon, 300.0
        sig2 = float(self.get_parameter('gps_sigma_m').value) ** 2
        fix.position_covariance = [sig2, 0.0, 0.0, 0.0, sig2, 0.0, 0.0, 0.0, 4 * sig2]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)
        self.pub_fix_status.publish(String(
            data='um960=connected quality=RTK_FIXED solution=NARROW_INT sats=24/30 '
                 'hdop=0.60 diff_age=1.0s age=0.10s (sim)'))
        vel = TwistStamped()
        vel.header = fix.header
        vel.twist.linear.x = self.drive.v
        self.pub_vel.publish(vel)

        st = {'t': round(t, 2), 'x': round(pose.x, 3), 'y': round(pose.y, 3),
              'yaw': round(pose.yaw, 3), 'v': round(self.drive.v, 3), 'w': round(self.drive.w, 3),
              'clearance': None if math.isinf(self.world.clearance(pose.x, pose.y, pose.yaw, t))
              else round(self.world.clearance(pose.x, pose.y, pose.yaw, t), 3),
              'min_clearance': None if math.isinf(self.min_clearance)
              else round(self.min_clearance, 3),
              'blocked': self.drive.blocked, 'collisions': self.drive.collisions_total,
              'last_contact': self.last_contact, 'bumper': s.bumper_triggered,
              'docked': s.is_docking_done, 'charging': s.is_charging,
              'cutting': self.cutting, 'estop': self.estop_latched,
              'battery': round(self.battery, 4)}
        self.pub_sim.publish(String(data=json.dumps(st)))

    def _slow_tick(self):
        stamp = self.get_clock().now().to_msg()
        s = getattr(self, '_status', None)
        charging = bool(s and s.is_charging)
        gp = self.get_parameter
        if charging:
            self.battery = min(1.0, self.battery + float(gp('battery_charge_per_s').value))
        else:
            self.battery = max(0.0, self.battery - float(gp('battery_drain_per_s').value))
        b = BatteryState()
        b.header.stamp = stamp
        b.header.frame_id = 'base_link'
        b.voltage = 21.0 + 4.2 * self.battery
        b.current = 2.0 if charging else -1.5
        b.percentage = self.battery
        b.temperature = 25.0
        b.present = True
        b.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING
                                 if s is not None and s.is_docking_done
                                 else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.pub_batt.publish(b)
        if self.loc_sim:
            self.pub_heading.publish(String(data=json.dumps({
                'aligned': True, 'source': 'dock', 'offset_deg': 0.0, 'quality': 'good',
                'event': 'sim'})))
        self._publish_markers(stamp)

    # ------------------------------------------------------------------ sensors
    def _stereo_tick(self):
        if not any(p.get_subscription_count() for p in (
                self.pub_points, self.pub_clear, self.pub_points_map, self.pub_clear_map)):
            return
        pose = self.drive.pose
        obs, ground = self.stereo.cast(pose.x, pose.y, pose.yaw, self.world.obstacles,
                                       self.sim_time())
        stamp = self.get_clock().now().to_msg()
        obs_msg = make_cloud(obs, 'stereo_camera_optical', stamp)
        ground_msg = make_cloud(ground, 'stereo_camera_optical', stamp)
        self.pub_points.publish(obs_msg)
        self.pub_clear.publish(ground_msg)
        self.pub_points_map.publish(obs_msg)
        self.pub_clear_map.publish(ground_msg)

    def _rear_tick(self):
        if self.pub_image.get_subscription_count() == 0:
            return
        pose, dock = self.drive.pose, self.world.dock
        img = self.rear.render(pose.x, pose.y, pose.yaw, dock.x, dock.y, dock.yaw)
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'rear_camera_optical'
        msg.height, msg.width = img.shape
        msg.encoding = 'mono8'
        msg.step = msg.width
        msg.data = img.tobytes()
        self.pub_image.publish(msg)
        info = CameraInfo()
        info.header = msg.header
        info.height, info.width = msg.height, msg.width
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [float(v) for v in self.rear.K]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        k = info.k
        info.p = [k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

    def _publish_markers(self, stamp):
        arr = MarkerArray()
        t = self.sim_time()
        for i, o in enumerate(self.world.obstacles):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns, m.id = 'sim_obstacles', i
            m.type = Marker.CUBE if o.type == 'box' else Marker.CYLINDER
            cx, cy = o.position(t)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, o.height / 2
            q = _quat(o.yaw)
            (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
             m.pose.orientation.w) = q
            if o.type == 'box':
                m.scale.x, m.scale.y = o.size_x, o.size_y
            else:
                m.scale.x = m.scale.y = 2 * o.radius
            m.scale.z = o.height
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.4, 0.1, 0.8
            arr.markers.append(m)
        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
