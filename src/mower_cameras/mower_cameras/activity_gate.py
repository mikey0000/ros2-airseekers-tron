# SPDX-License-Identifier: GPL-3.0-or-later
"""Idle duty cycle for the camera / IMU nodes, driven by ``/mission/activity``.

``/mission/activity`` (latched std_msgs/String, published by mower_mission) is one of
``docked_idle | idle | manual | mowing | docking``. Only ``docked_idle`` and ``idle``
allow throttling; anything else, including no message at all, means full rate (fail
active). Kept self-contained here (mirrors mower_mission/activity.py) so the camera
package does not depend on the mission package.
"""

ACTIVITY_TOPIC = '/mission/activity'
LOW_POWER = frozenset(('docked_idle', 'idle'))
IDLE_PERIOD_S = 1.0          # 1 fps while idle (keeps every consumer alive, ~15x less work)
IMU_IDLE_DECIMATION = 2      # 200 -> 100 Hz while idle
IMU_IDLE_BATCH_S = 0.1       # idle: drain the IIO FIFO in 100 ms bursts (512-sample buffer)


def is_low_power(activity):
    return activity in LOW_POWER


def capture_period(base_period, low_power, gui_watching=False, idle_period=IDLE_PERIOD_S):
    """Publish period for a camera: the idle cap applies only while idle and no GUI client
    watches the (compressed) stream."""
    base = float(base_period or 0.0)
    if low_power and not gui_watching:
        return max(base, float(idle_period))
    return base


def imu_decimation(low_power):
    return IMU_IDLE_DECIMATION if low_power else 1


def imu_batch_wait(low_power):
    """Extra sleep after each FIFO drain (0 = event driven at the watermark)."""
    return IMU_IDLE_BATCH_S if low_power else 0.0


class ActivityWatch:
    """Holds the latest activity; ``subscribe(node)`` wires it to the latched topic."""

    def __init__(self, on_change=None):
        self.activity = None
        self.on_change = on_change

    @property
    def low_power(self):
        return is_low_power(self.activity)

    def update(self, activity):
        prev, self.activity = self.activity, str(activity)
        if prev != self.activity and self.on_change is not None:
            self.on_change(self.activity)
        return self.low_power

    def subscribe(self, node, topic=ACTIVITY_TOPIC):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        return node.create_subscription(String, topic, lambda m: self.update(m.data), qos)
