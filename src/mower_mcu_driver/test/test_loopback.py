#!/usr/bin/env python3
"""End-to-end offline loopback test: socat pty pair + fake MCU + the driver node.

This is the automated version of ``scripts/loopback_test.sh``: instead of ``ros2 run`` it
drives :class:`mower_mcu_driver.mcu_node.McuNode` directly in-process, with the serial port
opened for real over a socat-created pty (pyserial when present, termios fallback otherwise).
Nothing from the mower hardware is needed and no root is required.

Skipped automatically when ``socat`` is not installed.

    python3 test/test_loopback.py
"""

import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

import ros_stubs  # noqa: E402  (same directory as this test)

ros_stubs.install()

from mower_mcu_driver import fake_mcu, mcu_node as mn  # noqa: E402

from geometry_msgs.msg import TwistStamped  # noqa: E402


def _temp_root():
    # Prefer the approved scratch dir when it exists.
    preferred = '/tmp/opencode' if os.path.isdir('/tmp/opencode') else None
    return tempfile.mkdtemp(prefix='mower_loopback_', dir=preferred)


@unittest.skipUnless(shutil.which('socat'), 'socat is not installed')
class TestLoopback(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.workdir = _temp_root()
        cls.host_pty = os.path.join(cls.workdir, 'host')
        cls.fake_pty = os.path.join(cls.workdir, 'fake')
        cls.socat = subprocess.Popen(
            ['socat', '-d', '-d',
             'PTY,link=%s,raw,echo=0,mode=666' % cls.host_pty,
             'PTY,link=%s,raw,echo=0,mode=666' % cls.fake_pty],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if os.path.exists(cls.host_pty) and os.path.exists(cls.fake_pty):
                return
            time.sleep(0.05)
        raise unittest.SkipTest('socat did not create the pty pair')

    @classmethod
    def tearDownClass(cls):
        cls.socat.terminate()
        try:
            cls.socat.wait(timeout=5)
        except subprocess.TimeoutExpired:
            cls.socat.kill()
        shutil.rmtree(cls.workdir, ignore_errors=True)

    def test_bidirectional_traffic_over_pty(self):
        node = mn.McuNode()
        node.port = self.host_pty
        node._open_serial()
        self.assertIsNotNone(node._ser, 'could not open the pty - see node log')
        self.addCleanup(node.shutdown)

        fake = threading.Thread(
            target=fake_mcu.main,
            args=(['--port', self.fake_pty, '--imu', '--duration', '3.0'],),
            daemon=True)
        fake.start()
        self.addCleanup(fake.join, 5.0)

        deadline = time.time() + 1.5
        next_tx = 0.0
        while time.time() < deadline:
            node._poll_serial()
            now = time.time()
            if now >= next_tx:
                next_tx = now + 0.05
                # keep /cmd_vel alive so it does not hit the stale-command timeout
                cmd = TwistStamped()
                cmd.twist.linear.x = 0.4
                cmd.twist.angular.z = 0.1
                node._on_cmd_vel(cmd)
                node._send_speed()
                node._send_heartbeat()
            node._reckon()
            time.sleep(0.005)

        # MCU -> host
        self.assertGreater(node._parser.frames_ok, 0, 'no valid frame received from fake MCU')
        self.assertEqual(node._parser.frames_bad, 0, 'corrupt frames on the loopback')
        self.assertIsNotNone(node._sensor_info, 'SensorInfo never arrived')
        self.assertTrue(node.published['/battery'], 'no /battery published')
        self.assertAlmostEqual(node.published['/battery'][0].voltage, 24.8, places=3)
        self.assertTrue(node.published['/imu'], 'module 9 (--imu) did not reach /imu')

        # host -> MCU -> host round trip: the fake MCU echoes the command as measured speed
        self.assertAlmostEqual(node._meas_linear, 0.4, places=2)
        self.assertAlmostEqual(node._meas_angular, 0.1, places=2)

        # dead reckoning integrated the measured motion
        self.assertTrue(node.published['/odom'], 'no /odom published')
        self.assertGreater(node.published['/odom'][-1].pose.pose.position.x, 0.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
