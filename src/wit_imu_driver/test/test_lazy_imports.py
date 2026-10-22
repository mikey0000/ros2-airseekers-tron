"""M3 import diet: tf2_ros (~30 MB) is only imported when publish_tf is enabled."""

import subprocess
import sys
import textwrap


def _construct(publish_tf):
    code = textwrap.dedent("""
        import sys
        import rclpy
        from wit_imu_driver.wit_node import WitImuNode
        assert 'tf2_ros' not in sys.modules
        rclpy.init(args=['--ros-args', '-p', 'publish_tf:=%s', '-p', 'port:=/dev/null_wit'])
        node = WitImuNode()
        print('TF2', 'tf2_ros' in sys.modules, node.tf_broadcaster is not None)
        node.destroy_node()
        rclpy.shutdown()
    """ % ('true' if publish_tf else 'false'))
    r = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=60)
    assert 'TF2' in r.stdout, r.stdout + r.stderr
    return r.stdout.split('TF2', 1)[1].split()[:2]


def test_no_tf2_ros_by_default():
    assert _construct(False) == ['False', 'False']


def test_tf2_ros_loaded_when_publish_tf():
    assert _construct(True) == ['True', 'True']
