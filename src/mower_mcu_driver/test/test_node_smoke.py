#!/usr/bin/env python3
"""Smoke test for :class:`mower_mcu_driver.mcu_node.McuNode` without a ROS installation.

A fake serial port is injected into the node; MCU frames are pushed through ``_poll_serial()``
exactly as they would arrive from the wire, and the captured TX bytes are decoded back with
the same parser.  This covers battery/IMU/odometry publishing, the ``/cmd_vel`` -> SpeedData
mapping (scale + clamp + timeout), the heartbeat, and the "no mower_interfaces -> skip
/mower_sensor_info" path.

    python3 test/test_node_smoke.py
"""

import os
import struct
import sys
import time
import types
import unittest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

import ros_stubs  # noqa: E402  (same directory as this test)

ros_stubs.install()

from mower_mcu_driver import mcu_node as mn  # noqa: E402

from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402


class FakeStatus:
    """Stands in for the nested MotorStatus message of MowerSensorInfo."""

    def __init__(self):
        self.status = 0


class FakeMotor:
    """Vendor-shaped MotorInfo: speed_rpm (alias!), current, voltage, temp, nested status."""

    def __init__(self):
        self.speed_rpm = 0
        self.current = 0
        self.voltage = 0
        self.temperature = 0
        self._status = FakeStatus()

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, value):
        if not isinstance(value, FakeStatus):
            raise TypeError('status must be a MotorStatus message, not %r' % (value,))


class FakeMowerSensorInfo:
    """Shape of mower_interfaces/MowerSensorInfo - but without rain_sensor_value/key_pressed
    on purpose, so the guarded field assignment is exercised."""

    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')
        self.bumper_triggered = False
        self.rain_triggered = False
        self.lift_triggered = False
        self.stop_triggered = False
        self.battery_gate_open = False
        self.press_module = False
        self.cutter_size = 0
        self.battery_temperature = 0
        self.battery_error = 0
        self.is_charging = False
        self.is_moving = False
        self.is_cmd_moving = False
        self.is_cutting = False
        self.cutter_board_version = ''
        self.chassis_board_version = ''
        self.rtk_board_version = ''
        self.cutter_motor = FakeMotor()
        self.left_motor = FakeMotor()
        self.right_motor = FakeMotor()
        self.height_motor = FakeMotor()


class FakeSerial:
    """Byte pipes standing in for the pty/UART."""

    def __init__(self):
        self.rx = bytearray()
        self.tx = bytearray()
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        chunk = bytes(self.rx[:size])
        del self.rx[:size]
        return chunk

    def write(self, data):
        self.tx += bytes(data)
        return len(data)

    def close(self):
        self.closed = True


def make_node(**overrides):
    node = mn.McuNode()
    node._ser = FakeSerial()
    for name, value in overrides.items():
        setattr(node, name, value)
    return node


def inject(node, *subpackets):
    """Push an MCU frame to the node as if it had arrived on the wire."""
    node._ser.rx += mn.build_frame(subpackets)
    node._poll_serial()


def drain_tx(port):
    """Return every sub-packet written to ``port``, decoded with the node's own parser."""
    parser = mn.FrameParser()
    subpackets = parser.feed(bytes(port.tx))
    port.tx.clear()
    return subpackets


def cmd_vel(linear, angular):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    return msg


@unittest.skipUnless(ros_stubs.is_stubbed(), 'stub-only test (needs ros_stubs internals; real rclpy present)')
class TestMcuNode(unittest.TestCase):

    def setUp(self):
        self.node = make_node()

    def tearDown(self):
        self.node.shutdown()

    # ------------------------------------------------------------- telemetry
    def test_battery_publishes_battery_state(self):
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_BATTERY,
                           struct.pack(mn.BATTERY_FMT, 248, 5, 99, 0, 15, 0)))
        published = self.node.published['/battery']
        self.assertEqual(len(published), 1)
        msg = published[0]
        self.assertAlmostEqual(msg.voltage, 24.8)          # 0.1 V raw units
        self.assertAlmostEqual(msg.current, -0.5)          # 0.1 A, +raw = discharging
        self.assertAlmostEqual(msg.percentage, 0.99)
        self.assertEqual(msg.temperature, 15.0)
        self.assertTrue(msg.present)
        self.assertEqual(msg.power_supply_status,
                         ros_stubs.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.assertEqual(msg.header.frame_id, 'base_link')

    def test_battery_dock_flag_maps_to_charging(self):
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_BATTERY,
                           struct.pack(mn.BATTERY_FMT, 250, -10, 100, 1, 20, 0)))
        self.assertEqual(self.node.published['/battery'][0].power_supply_status,
                         ros_stubs.BatteryState.POWER_SUPPLY_STATUS_CHARGING)

    def test_nothing_published_on_imu_until_module_9_arrives(self):
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_SENSOR, struct.pack(mn.SENSOR_FMT, *([0] * 10))))
        self.assertEqual(self.node.published['/mcu/imu'], [])
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_IMU,
                           struct.pack(mn.IMU_FMT, 0, 0, 0, 0, 0, 2048, 0, 0, 0)))
        imu = self.node.published['/mcu/imu']
        self.assertEqual(len(imu), 1)
        self.assertAlmostEqual(imu[0].linear_acceleration.z, 9.80665, places=3)  # raw 2048 = 1 g

    def test_mower_sensor_info_skipped_without_mower_interfaces(self):
        # When mower_interfaces has not been created the publisher must be None and incoming
        # SensorInfo frames must not raise. (Skipped in an environment where the package
        # exists - the sibling test covers that half.)
        if mn.MowerSensorInfo is not None:
            self.skipTest('mower_interfaces is installed in this environment')
        self.assertIsNone(self.node._sensor_pub)
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_SENSOR, struct.pack(mn.SENSOR_FMT, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0)))
        self.assertEqual(self.node.published.get('/mower_sensor_info', []), [])

    def test_mower_sensor_info_published_when_interface_available(self):
        # Simulate "mower_interfaces was created" and check every filled field.
        published = self.node.published['/mower_sensor_info']
        self.node._sensor_pub = types.SimpleNamespace(publish=published.append)
        previous = mn.MowerSensorInfo
        mn.MowerSensorInfo = FakeMowerSensorInfo
        self.addCleanup(setattr, mn, 'MowerSensorInfo', previous)

        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_VERSION,
                           struct.pack(mn.VERSION_FMT, 0, 6, 36, 0, 6, 34, 0, 9, 50)))
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_BATTERY,
                           struct.pack(mn.BATTERY_FMT, 248, 5, 99, 1, 15, 0)))
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_MOTORS,
                           b''.join(struct.pack(mn.MOTOR_FMT, 0, 0, 2478, 14, 0)
                                    for _ in range(4))))
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_SPEED, mn.speed_payload(0.4, 0.0)))
        self.node._on_cmd_vel(cmd_vel(0.4, 0.0))
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_SENSOR,
                           struct.pack(mn.SENSOR_FMT, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0)))

        self.assertEqual(len(published), 1)
        msg = published[0]
        self.assertTrue(msg.bumper_triggered)
        self.assertTrue(msg.stop_triggered)
        self.assertTrue(msg.battery_gate_open)
        self.assertTrue(msg.is_charging)
        self.assertTrue(msg.is_moving)
        self.assertTrue(msg.is_cmd_moving)
        self.assertEqual(msg.cutter_board_version, 'v0.6.36')
        self.assertEqual(msg.chassis_board_version, 'v0.6.34')
        self.assertEqual(msg.rtk_board_version, '0.9.50')
        self.assertEqual(msg.battery_temperature, 15)
        self.assertEqual(msg.cutter_size, 1)
        # 'speed' -> 'speed_rpm' alias, and the nested status message was left alone
        # (assigning the raw int would raise TypeError, which _set_if swallows).
        self.assertEqual(msg.cutter_motor.voltage, 2478)
        self.assertEqual(msg.cutter_motor.speed_rpm, 0)
        self.assertIsInstance(msg.cutter_motor.status, FakeStatus)

    # ----------------------------------------------------------------- cmd_vel
    # SpeedData TX policy (docs/wheel_control_semantics.md 5): one frame per /cmd_vel,
    # 3 zeros 100 ms apart on stop/timeout/interlock, silence otherwise.
    def _clock(self, t0=1000.0):
        self.t = t0
        self.node._mono = lambda: self.t

    def _speeds(self):
        return [struct.unpack(mn.SPEED_FMT, p) for (_t, m, p) in drain_tx(self.node._ser)
                if m == mn.MOD_SPEED]

    def _tick_until(self, t_end, step=0.02):
        """Run the 20 ms I/O-thread tick up to t_end; return [(t, frame)] sent."""
        out = []
        while self.t < t_end - 1e-9:
            self.t = round(self.t + step, 6)
            self.node._speed_tick(self.t)
            out += [(self.t, f) for f in self._speeds()]
        return out

    def test_defaults_are_event_driven_and_clamped_to_vendor_limits(self):
        self.assertFalse(self.node.get_parameter('speed_stream_enabled').value)
        self.assertEqual((self.node.linear_max, self.node.angular_max), (0.3, 0.3))
        self.assertEqual((self.node.stop_frames, self.node.stop_frame_spacing), (3, 0.1))
        for gone in ('brake_gain', 'pid_enabled', 'brake_deadband_linear', 'linear_kp'):
            self.assertNotIn(gone, self.node._params)
        self.assertFalse(hasattr(mn, 'VelocityPID'))

    def test_cmd_vel_is_scaled_clamped_and_sent_as_one_frame(self):
        self._clock()
        self.node.linear_scale = 2.0
        self.node.angular_scale = 0.5
        self.node._on_cmd_vel(cmd_vel(5.0, 10.0))          # both above the clamps
        self.node._speed_tick(self.t)
        (type_id, mod_id, payload), = drain_tx(self.node._ser)
        self.assertEqual((type_id, mod_id), (mn.TYPE_ROS_MOWER, mn.MOD_SPEED))
        linear, angular = struct.unpack(mn.SPEED_FMT, payload)
        self.assertAlmostEqual(linear, 0.3)
        self.assertAlmostEqual(angular, 0.3)
        self.node._on_cmd_vel(cmd_vel(-0.1, -9.0))
        self.node._speed_tick(self.t)
        (lin, ang), = self._speeds()
        self.assertAlmostEqual(lin, -0.2, places=6)
        self.assertAlmostEqual(ang, -0.3, places=6)

    def test_one_frame_per_command_no_resend(self):
        self._clock()
        for k in range(5):
            self.node._on_cmd_vel(cmd_vel(0.1, 0.05 * k))
        self.node._speed_tick(self.t)
        frames = self._speeds()
        self.assertEqual(len(frames), 5)
        self.assertAlmostEqual(frames[-1][1], 0.2, places=6)
        # nothing is re-sent while commands keep arriving within the timeout
        self.assertEqual(self._tick_until(self.t + 0.4), [])
        sent = self.node.published['/mcu/sent_speed']
        self.assertEqual(len(sent), 5)

    def test_cmd_vel_stamped_takes_the_same_path(self):
        self._clock()
        msg = TwistStamped()
        msg.twist.linear.x = 0.1
        self.node._on_cmd_vel_stamped(msg)
        self.node._speed_tick(self.t)
        self.assertEqual(len(self._speeds()), 1)

    def test_zero_after_motion_sends_three_spaced_zeros_then_silence(self):
        self._clock()
        self.node._on_cmd_vel(cmd_vel(0.2, 0.0))
        self.node._speed_tick(self.t)
        self.assertEqual(len(self._speeds()), 1)
        self.t += 0.05
        self.node._on_cmd_vel(cmd_vel(0.0, 0.0))
        t0 = self.t
        self.node._speed_tick(self.t)
        self.assertEqual(self._speeds(), [(0.0, 0.0)])
        later = self._tick_until(t0 + 2.0)
        self.assertEqual([f for _t, f in later], [(0.0, 0.0), (0.0, 0.0)])
        self.assertAlmostEqual(later[0][0] - t0, 0.1, delta=0.021)
        self.assertAlmostEqual(later[1][0] - later[0][0], 0.1, delta=0.021)
        # further zeros while stopped: silence
        for _ in range(3):
            self.node._on_cmd_vel(cmd_vel(0.0, 0.0))
            self.node._speed_tick(self.t)
        self.assertEqual(self._tick_until(self.t + 1.0), [])

    def test_no_zeros_when_already_stopped(self):
        self._clock()
        for _ in range(10):
            self.node._on_cmd_vel(cmd_vel(0.0, 0.0))
            self.node._speed_tick(self.t)
        self.assertEqual(self._speeds(), [])
        self.assertEqual(self._tick_until(self.t + 5.0), [])     # idle = silence
        self.assertEqual(self.node.published['/mcu/sent_speed'], [])

    def test_cmd_vel_timeout_sends_stop_sequence(self):
        self._clock()
        self.node._on_cmd_vel(cmd_vel(0.25, 0.1))
        self.node._speed_tick(self.t)
        self.assertEqual(len(self._speeds()), 1)
        t0 = self.t
        frames = self._tick_until(t0 + 3.0)
        self.assertEqual([f for _t, f in frames], [(0.0, 0.0)] * 3)
        self.assertGreater(frames[0][0] - t0, self.node.cmd_vel_timeout)
        self.assertLess(frames[0][0] - t0, self.node.cmd_vel_timeout + 0.05)

    def test_new_motion_cancels_pending_stop_zeros(self):
        self._clock()
        self.node._on_cmd_vel(cmd_vel(0.2, 0.0))
        self.node._on_cmd_vel(cmd_vel(0.0, 0.0))
        self.node._speed_tick(self.t)
        self.assertEqual(len(self._speeds()), 2)
        self.t += 0.04
        self.node._on_cmd_vel(cmd_vel(0.1, 0.0))
        self.node._speed_tick(self.t)
        self.assertEqual(len(self._speeds()), 1)
        frames = self._tick_until(self.t + 0.3)
        self.assertEqual(frames, [])

    def test_interlock_while_moving_stops_and_blocks_until_fresh_command(self):
        self._clock()
        self.node._on_cmd_vel(cmd_vel(0.2, 0.0))
        self.node._speed_tick(self.t)
        self._speeds()
        self.node._estop_latched = True
        t0 = self.t
        frames = self._tick_until(t0 + 1.0)
        self.assertEqual([f for _t, f in frames], [(0.0, 0.0)] * 3)
        # commands while interlocked produce nothing
        self.node._on_cmd_vel(cmd_vel(0.2, 0.0))
        self.node._speed_tick(self.t)
        self.assertEqual(self._tick_until(self.t + 1.0), [])
        # cleared: still silent until a fresh non-zero command
        self.node._estop_latched = False
        self.assertEqual(self._tick_until(self.t + 1.0), [])
        self.node._on_cmd_vel(cmd_vel(0.0, 0.0))
        self.node._speed_tick(self.t)
        self.assertEqual(self._speeds(), [])
        self.node._on_cmd_vel(cmd_vel(0.15, 0.0))
        self.node._speed_tick(self.t)
        self.assertEqual(len(self._speeds()), 1)

    def test_interlock_at_rest_sends_nothing(self):
        self._clock()
        self.node._estop_latched = True
        self.assertEqual(self._tick_until(self.t + 1.0), [])

    def test_estop_request_while_moving_sends_first_zero_immediately(self):
        self._clock()
        self.node._on_cmd_vel(cmd_vel(0.2, 0.0))
        self.node._speed_tick(self.t)
        self._speeds()
        self.node._on_estop_request(types.SimpleNamespace(data=True))
        self.assertEqual(self._speeds(), [(0.0, 0.0)])

    def test_debug_stream_mode_streams_and_ends_with_stop(self):
        self._clock()
        self.node._params['speed_stream_enabled'] = True
        self.node._on_cmd_vel(cmd_vel(0.1, 0.0))
        frames = self._tick_until(self.t + 0.3)
        self.assertGreaterEqual(len(frames), 5)            # ~20 Hz
        self.assertLessEqual(len(frames), 7)
        self.assertTrue(all(f == (0.1, 0.0) or abs(f[0] - 0.1) < 1e-6 for _t, f in frames))
        self.node._params['speed_stream_enabled'] = False
        frames = self._tick_until(self.t + 1.0)
        self.assertEqual([f for _t, f in frames], [(0.0, 0.0)] * 3)
        self.assertEqual(self._tick_until(self.t + 1.0), [])

    # ---- docked zero-speed keepalive
    def _dock(self, activity='docked_idle', dock_ok=True):
        self.node._params['docked_zero_keepalive'] = True   # off by default; tests opt in
        self.node._on_activity(types.SimpleNamespace(data=activity))
        self.node._battery = {'dock_ok': dock_ok}

    def test_keepalive_defaults(self):
        self.assertFalse(self.node.get_parameter('docked_zero_keepalive').value)
        self.assertAlmostEqual(self.node.docked_zero_keepalive_period, 1.0)

    def test_keepalive_runs_only_when_idle_or_docked_idle(self):
        self._clock()
        self._dock()
        frames = self._tick_until(self.t + 3.5)
        self.assertEqual([f for _t, f in frames], [(0.0, 0.0)] * 4)   # t+0.02,1.02,2.02,3.02
        gaps = [b[0] - a[0] for a, b in zip(frames, frames[1:])]
        for g in gaps:
            self.assertAlmostEqual(g, 1.0, delta=0.021)
        for activity in ('undocking', 'mowing', 'docking', 'returning', '', None):
            self._dock(activity, True)
            self.assertEqual(self._tick_until(self.t + 3.0), [], activity)
        for activity, dock_ok in (('docked_idle', False), ('idle', True), ('idle', False)):
            self._dock(activity, dock_ok)
            self.assertEqual(len(self._tick_until(self.t + 3.0)), 3, (activity, dock_ok))

    def test_keepalive_survives_contact_loss_while_idle_on_dock(self):
        # live 2026-10-08: dock_ok dropped while parked, mission went CHARGING -> IDLE
        self._clock()
        self._dock()
        self.assertEqual(len(self._tick_until(self.t + 1.5)), 2)
        self._dock('idle', False)
        self.assertEqual(len(self._tick_until(self.t + 3.0)), 3)

    def test_keepalive_resumes_after_undock_dock_cycle(self):
        """Replay: docked_idle -> undock (mowing, reverse) -> idle -> docking (drive in,
        zero) -> contacts flap -> docked_idle -> CHARGING->IDLE contact loss."""
        self._clock()
        self._dock()
        self.assertTrue(self._tick_until(self.t + 1.5))
        self.node._on_activity(types.SimpleNamespace(data='mowing'))
        for _ in range(10):                                 # undock: reverse
            self.node._on_cmd_vel(cmd_vel(-0.1, 0.0))
            self._tick_until(self.t + 0.1)
        self.node._battery = {'dock_ok': False}
        self.node._on_cmd_vel(cmd_vel(0.0, 0.0))
        self._tick_until(self.t + 0.5)
        self.node._on_activity(types.SimpleNamespace(data='idle'))
        self.assertEqual(len(self._tick_until(self.t + 2.0)), 2)  # idle at rest: keepalive
        self.node._on_activity(types.SimpleNamespace(data='docking'))
        for k in range(10):                                 # drive in, contacts flap
            self.node._on_cmd_vel(cmd_vel(0.05, 0.0))
            self.node._battery = {'dock_ok': k % 2 == 0}
            self._tick_until(self.t + 0.1)
        self.node._on_cmd_vel(cmd_vel(0.0, 0.0))
        self.node._battery = {'dock_ok': True}
        self._tick_until(self.t + 0.5)
        self._dock()                                        # docked_idle (CHARGING)
        self._tick_until(self.t + 0.6)
        self.assertTrue(self.node._keepalive_on)
        frames = self._tick_until(self.t + 3.0)
        self.assertEqual(len(frames), 3)
        self._dock('idle', False)                           # CHARGING -> IDLE, contacts lost
        self.assertEqual(len(self._tick_until(self.t + 3.0)), 3)

    def test_boot_into_idle_off_contact_runs_after_one_period(self):
        self._clock()
        self.node._last_nonzero_t = self.t                 # conservative: a command just now
        self._dock('idle', False)
        frames = self._tick_until(self.t + 2.5)
        self.assertEqual(len(frames), 2)
        self.assertGreaterEqual(frames[0][0] - 1000.0, 1.0)

    def test_keepalive_disabled_by_parameter(self):
        self._clock()
        self._tick_until(self.t + 0.1)
        self.node._on_activity(types.SimpleNamespace(data='docked_idle'))
        self.node._battery = {'dock_ok': True}             # default: off
        self.assertEqual(self._tick_until(self.t + 3.0), [])
        self._dock()                                       # live enable ...
        self.assertEqual(len(self._tick_until(self.t + 1.5)), 2)
        self.node._params['docked_zero_keepalive'] = False  # ... and live disable
        self.assertEqual(self._tick_until(self.t + 3.0), [])

    def test_keepalive_stops_on_undock_and_motion(self):
        self._clock()
        self._dock()
        self.assertEqual(len(self._tick_until(self.t + 1.5)), 2)
        self.node._on_activity(types.SimpleNamespace(data='undocking'))
        self.assertEqual(self._tick_until(self.t + 3.0), [])
        # motion while (still) docked_idle: non-zero takes over, normal stop, then a full
        # period of quiet before the keepalive resumes
        self._dock()
        self._tick_until(self.t + 0.1)
        self._speeds()
        self.node._on_cmd_vel(cmd_vel(0.1, 0.0))
        self.node._speed_tick(self.t)
        (lin, _ang), = self._speeds()
        self.assertAlmostEqual(lin, 0.1, places=6)
        t_cmd = self.t
        frames = self._tick_until(t_cmd + 0.3)
        self.assertEqual(frames, [])                       # no zeros while moving
        frames = self._tick_until(t_cmd + 3.0)
        stop = [t for t, _f in frames if t - t_cmd < 1.0]
        self.assertEqual(len(stop), 3)                     # cmd_vel timeout stop sequence
        ka = [t for t, _f in frames if t - t_cmd >= 1.0]
        self.assertTrue(ka)
        self.assertGreaterEqual(ka[0] - t_cmd, self.node.docked_zero_keepalive_period)

    def test_new_cmd_vel_takes_over_immediately_while_docked(self):
        self._clock()
        self._dock()
        self._tick_until(self.t + 0.5)
        self._speeds()
        self.node._on_cmd_vel(cmd_vel(0.2, 0.0))
        self.node._speed_tick(self.t)
        (lin, _ang), = self._speeds()
        self.assertAlmostEqual(lin, 0.2, places=6)

    def test_keepalive_never_under_interlock(self):
        self._clock()
        self._dock()
        self.node._estop_latched = True
        self.assertEqual(self._tick_until(self.t + 3.0), [])
        self.node._estop_latched = False
        self.assertEqual(len(self._tick_until(self.t + 1.5)), 2)

    def test_keepalive_logs_start_stop_once(self):
        self._clock()
        logs = []
        self.node.get_logger = lambda: types.SimpleNamespace(
            info=lambda m, **k: logs.append(m), warn=lambda m, **k: None)
        self._dock()
        self._tick_until(self.t + 3.0)
        self.node._on_activity(types.SimpleNamespace(data='undocking'))
        self._tick_until(self.t + 3.0)
        ka = [m for m in logs if 'keepalive' in m]
        self.assertEqual(len(ka), 2)

    def test_measured_and_commanded_telemetry_published(self):
        self._clock()
        self._tick_until(self.t + 1.0)
        self.assertTrue(self.node.published['/mcu/measured_speed'])
        self.assertTrue(self.node.published['/mcu/commanded_speed'])
        self.assertEqual(self.node.published['/mcu/sent_speed'], [])

    def test_shutdown_sends_zero_speed_before_closing(self):
        self.node._on_cmd_vel(cmd_vel(1.0, 1.0))
        serial_port = self.node._ser
        self.node.shutdown()
        self.assertTrue(serial_port.closed)
        frames = drain_tx(serial_port)
        self.assertEqual([m for (_, m, _) in frames], [mn.MOD_CUTTER, mn.MOD_SPEED])
        self.assertEqual(frames[0][2][0], 0)  # cutter enable byte off
        self.assertEqual(struct.unpack(mn.SPEED_FMT, frames[1][2]), (0.0, 0.0))
        self.node._ser = serial_port  # keep tearDown/shutdown idempotent

    # -------------------------------------------------------------- heartbeat
    def test_heartbeat_frame_shape(self):
        self.node._send_heartbeat()
        (type_id, mod_id, payload), = drain_tx(self.node._ser)
        self.assertEqual(type_id, mn.TYPE_HEARTBEAT)
        self.assertEqual(mod_id, mn.MOD_ALL)
        self.assertEqual(len(payload), 8)
        year, month, day = struct.unpack('<3B', payload[:3])
        self.assertTrue(1 <= month <= 12 and 1 <= day <= 31)
        self.assertGreaterEqual(year, 20)                  # sent as year - 2000

    # --------------------------------------------------------------- odometry
    def test_odom_dead_reckons_from_measured_speed(self):
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_SPEED, mn.speed_payload(0.5, 0.1)))
        self.node._last_reckon -= 0.1                      # pretend 100 ms elapsed
        self.node._reckon()
        odom = self.node.published['/odom'][0]
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.05, places=3)
        self.assertEqual(odom.header.frame_id, 'odom')
        self.assertEqual(odom.child_frame_id, 'base_link')
        self.assertAlmostEqual(odom.twist.twist.linear.x, 0.5)
        self.assertAlmostEqual(odom.twist.twist.angular.z, 0.1)
        self.assertEqual(odom.twist.covariance, mn._TWIST_COVARIANCE_MEASURED)

    def test_odom_degrades_to_commanded_velocity(self):
        self.node._on_cmd_vel(cmd_vel(0.3, 0.0))
        self.node._last_reckon -= 0.1
        self.node._reckon()
        odom = self.node.published['/odom'][0]
        self.assertAlmostEqual(odom.twist.twist.linear.x, 0.3)
        self.assertEqual(odom.pose.pose.position.x > 0.0, True)
        # degraded -> the node has to admit it does not have a measurement
        self.assertEqual(odom.twist.covariance, mn._TWIST_COVARIANCE_COMMANDED)

    def test_odom_uses_imu_yaw_when_available(self):
        inject(self.node, (mn.TYPE_MOWER_ROS, mn.MOD_IMU,
                           struct.pack(mn.IMU_FMT, 0, 0, 32768 // 4, 0, 0, 2048, 0, 0, 0)))
        self.node._last_reckon -= 0.1
        self.node._reckon()
        odom = self.node.published['/odom'][0]
        # raw yaw 8192 == 45 deg -> z = w = sin/cos(22.5 deg)
        self.assertAlmostEqual(odom.pose.pose.orientation.z, 0.38268, places=4)
        self.assertAlmostEqual(odom.pose.pose.orientation.w, 0.92388, places=4)


if __name__ == '__main__':
    unittest.main(verbosity=2)


@unittest.skipUnless(ros_stubs.is_stubbed(), 'stub-only test (needs ros_stubs internals; real rclpy present)')
class SafetyAndServicesTest(unittest.TestCase):
    """Interlocks, cutter/charging services and the host e-stop latch (stubbed ROS)."""

    def setUp(self):
        ros_stubs.install()
        self.node = mn.McuNode()
        self.tx = bytearray()
        self.node._ser = types.SimpleNamespace(
            write=lambda b: self.tx.extend(b) or len(b), in_waiting=0,
            read=lambda n: b'', close=lambda: None)
        self.node._write = lambda data: self.tx.extend(data)

    def _frames(self):
        parser = mn.FrameParser()
        out = []
        for f in parser.feed(bytes(self.tx)):
            out.append(f)
        return out

    def _sensor(self, **flags):
        names = ('bumper', 'rain', 'lift', 'stop', 'power_off', 'battery_gate',
                 'cutter_size', 'press_module', 'bumper_r', 'bumper_l')
        vals = [int(flags.get(n, 0)) for n in names]
        self.node._on_sensor_info(struct.pack(mn.SENSOR_FMT, *vals))

    def test_cutter_payload_layout(self):
        pl = mn.cutter_payload(True, False, 100, True, False, 60)
        self.assertEqual(len(pl), 12)
        self.assertEqual(struct.unpack('<BBHH', pl[:6]), (1, 0, 100, 0))
        self.assertEqual(struct.unpack('<BBHH', pl[6:]), (1, 0, 0, 60))

    def test_lift_interlock_zeroes_speed_and_sends_cutter_off(self):
        cmd = Twist()
        cmd.linear.x = 0.3
        self.node._on_cmd_vel(cmd)
        self.assertEqual(self.node._commanded(), (0.3, 0.0))
        self._sensor(lift=1)
        self.assertEqual(self.node._commanded(), (0.0, 0.0))
        mods = [m for (_t, m, _p) in self._frames()]
        self.assertIn(mn.MOD_CUTTER, mods)
        # cutter off payload: enable byte 0
        cutter_payloads = [p for (_t, m, p) in self._frames() if m == mn.MOD_CUTTER]
        self.assertEqual(cutter_payloads[-1][0], 0)
        # released -> motion allowed again
        self.node._on_cmd_vel(cmd)
        self._sensor(lift=0)
        self.assertEqual(self.node._commanded(), (0.3, 0.0))

    def test_estop_latch_blocks_until_cleared(self):
        cmd = Twist()
        cmd.linear.x = 0.2
        self.node._on_cmd_vel(cmd)
        self.node._on_estop_request(types.SimpleNamespace(data=True))
        self.assertEqual(self.node._commanded(), (0.0, 0.0))
        self.node._on_cmd_vel(cmd)
        self.assertEqual(self.node._commanded(), (0.0, 0.0))
        self.node._srv_clear_estop(None, types.SimpleNamespace())
        self.node._on_cmd_vel(cmd)
        self.assertEqual(self.node._commanded(), (0.2, 0.0))

    def test_cutter_service_refused_while_interlocked(self):
        self._sensor(stop=1)
        req = types.SimpleNamespace(
            cutter=types.SimpleNamespace(enable=True, direction=False, speed=0, position=0),
            height=types.SimpleNamespace(enable=False, direction=False, speed=0, position=0))
        resp = self.node._srv_cutter_control(req, types.SimpleNamespace(result=None))
        self.assertFalse(resp.result)
        self._sensor(stop=0)
        self.tx.clear()
        resp = self.node._srv_cutter_control(req, types.SimpleNamespace(result=None))
        self.assertTrue(resp.result)
        payloads = [p for (_t, m, p) in self._frames() if m == mn.MOD_CUTTER]
        self.assertEqual(payloads[-1][0], 1)
        self.assertEqual(struct.unpack('<H', payloads[-1][2:4])[0], self.node.cutter_default_speed)

    def _height_req(self, cutter_on, height=None):
        return types.SimpleNamespace(
            cutter=types.SimpleNamespace(enable=cutter_on, direction=False, speed=0, position=0),
            height=types.SimpleNamespace(enable=height is not None, direction=False, speed=0,
                                         position=height or 0))

    def _last_height(self):
        payloads = [p for (_t, m, p) in self._frames() if m == mn.MOD_CUTTER]
        return struct.unpack('<BBHH', payloads[-1][6:])

    def test_height_mm_passthrough_and_clamp(self):
        self.assertEqual(self.node._height_mm, 90)     # vendor resting default
        for asked, sent in ((50, 50), (30, 30), (90, 90), (10, 30), (120, 90), (0, 30)):
            self.tx.clear()
            resp = self.node._srv_cutter_control(self._height_req(False, asked),
                                                 types.SimpleNamespace(result=None))
            self.assertTrue(resp.result)
            # height motor: enable/direction/speed 0, position = absolute mm
            self.assertEqual(self._last_height(), (0, 0, 0, sent))

    def test_blade_off_holds_last_height(self):
        self.node._srv_cutter_control(self._height_req(False, 50),
                                      types.SimpleNamespace(result=None))
        self.node._srv_cutter_control(self._height_req(True),     # ON, no height given
                                      types.SimpleNamespace(result=None))
        self.assertEqual(self._last_height()[3], 50)
        self.tx.clear()
        self.node._srv_cutter_off(None, types.SimpleNamespace(success=None, message=''))
        self.assertEqual(self._last_height()[3], 50)
        self.tx.clear()
        self._sensor(lift=1)                                       # interlock cutter-off
        self.assertEqual(self._last_height()[3], 50)

    def test_blade_off_vendor_height_option(self):
        self.node.cutter_off_height_mm = 90
        self.node._srv_cutter_control(self._height_req(True, 40),
                                      types.SimpleNamespace(result=None))
        self.assertEqual(self._last_height()[3], 40)
        self.tx.clear()
        self.node._srv_cutter_off(None, types.SimpleNamespace(success=None, message=''))
        self.assertEqual(self._last_height()[3], 90)
        self.tx.clear()
        self.node._srv_cutter_control(self._height_req(True),
                                      types.SimpleNamespace(result=None))
        self.assertEqual(self._last_height()[3], 40)              # remembered height restored

    def _set_height(self, mm):
        return self.node._srv_set_height(types.SimpleNamespace(height_mm=mm),
                                         types.SimpleNamespace(ok=None, message='', height_mm=0))

    def test_height_topic_latched_on_startup_and_change(self):
        pub = self.node.published['/cutter/height_mm']
        self.assertEqual([m.data for m in pub], [90])
        self._set_height(55)
        self._set_height(55)                                       # unchanged -> no repeat
        self.assertEqual([m.data for m in pub], [90, 55])

    def test_set_height_keeps_blade_on_with_its_speed(self):
        req = self._height_req(True, 40)
        req.cutter.speed = 70
        self.node._srv_cutter_control(req, types.SimpleNamespace(result=None))
        self.tx.clear()
        resp = self._set_height(62)
        self.assertTrue(resp.ok)
        self.assertEqual(resp.height_mm, 62)
        payloads = [p for (_t, m, p) in self._frames() if m == mn.MOD_CUTTER]
        self.assertEqual(struct.unpack('<BBHH', payloads[-1][:6]), (1, 0, 70, 0))
        self.assertEqual(self._last_height(), (0, 0, 0, 62))
        self.assertTrue(self.node._cutter_requested_on)

    def test_set_height_blade_off_stays_off_and_clamps(self):
        resp = self._set_height(5)
        self.assertTrue(resp.ok)
        self.assertEqual(resp.height_mm, 30)
        self.assertIn('clamped', resp.message)
        payloads = [p for (_t, m, p) in self._frames() if m == mn.MOD_CUTTER]
        self.assertEqual(payloads[-1][0], 0)
        self.assertEqual(self._last_height()[3], 30)

    def test_set_height_refused_while_estop(self):
        self.node._on_estop_request(types.SimpleNamespace(data=True))
        self.tx.clear()
        resp = self._set_height(50)
        self.assertFalse(resp.ok)
        self.assertEqual(resp.height_mm, 90)
        self.assertEqual(self._frames(), [])

    def test_charging_service_sends_charge_control(self):
        resp = self.node._srv_charging(types.SimpleNamespace(enable_charging=True),
                                       types.SimpleNamespace(result=None))
        self.assertTrue(resp.result)
        payloads = [p for (_t, m, p) in self._frames() if m == mn.MOD_CHARGE]
        self.assertEqual(payloads, [b'\x01'])


class ImuForwardTest(unittest.TestCase):
    def test_imu_payload_matches_vendor_scaling(self):
        import math
        # 1 g on z, level, yaw 0 -> accz 2048 counts, angles 0
        pl = mn.imu_payload(0.0, 0.0, 0.0, (0.0, 0.0, 9.80665), (0.0, 0.0, 0.0))
        self.assertEqual(len(pl), 18)
        vals = struct.unpack(mn.IMU_FMT, pl)
        self.assertEqual(vals[:3], (0, 0, 0))
        self.assertEqual(vals[5], 2048)
        # roll -177.8 deg (IMU mounted upside down, as in the vendor capture) -> ~ -32360 counts
        pl = mn.imu_payload(math.radians(-177.8), math.radians(-7.14), 0.0, (0, 0, 0), (0, 0, 0))
        pitch, roll = struct.unpack(mn.IMU_FMT, pl)[:2]
        self.assertAlmostEqual(roll, -32368, delta=12)
        self.assertAlmostEqual(pitch, -1300, delta=3)

    def test_quat_to_rpy_roundtrip(self):
        import math
        r, p, y = 0.3, -0.2, 1.1
        cy, sy = math.cos(y / 2), math.sin(y / 2); cp, sp = math.cos(p / 2), math.sin(p / 2)
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
             cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
        rr, pp, yy = mn._quat_to_rpy(*q)
        for a, b in ((rr, r), (pp, p), (yy, y)):
            self.assertAlmostEqual(a, b, places=6)


class ClearEstopFrameTest(unittest.TestCase):
    def test_clear_estop_sends_vendor_module10_zero_frame(self):
        ros_stubs.install()
        node = mn.McuNode()
        tx = bytearray()
        node._write = lambda data: tx.extend(data)
        node._srv_clear_estop(None, types.SimpleNamespace())
        frames = list(mn.FrameParser().feed(bytes(tx)))
        self.assertEqual([(t, m, len(p)) for (t, m, p) in frames], [(mn.TYPE_ROS_MOWER, mn.MOD_SENSOR, 8)])
        self.assertEqual(frames[0][2], bytes(8))
