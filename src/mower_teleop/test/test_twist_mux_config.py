"""Guard the Jazzy twist_mux message-type pin.

twist_mux 4.5.0 (Jazzy) **defaults to stamped** messages when ``use_stamped`` is
absent (``"use_stamped" is not declared as parameter, defaulting to "true"``):
it then subscribes every lane as ``TwistStamped`` and publishes ``cmd_vel_out``
as ``TwistStamped``. The mower's whole command chain (Nav2, docking, teleop,
emergency, bumper -> twist_mux -> cmd_vel_slew -> mcu_node) is unstamped
``geometry_msgs/Twist``, and a mismatch is silent (same topic, two types: the
endpoints never connect).

2026-10-10: without the pin, starting a mow put +0.15 m/s on ``/cmd_vel_docking``
for ~16 s while ``/cmd_vel`` / ``/mcu/commanded_speed`` stayed 0.0, and undock
failed ``DRIVE_TIMEOUT``. See docs/JAZZY_MIGRATION_CHECKLIST_2026-10-09.md.
"""

import os

import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'twist_mux.yaml')


def _params():
    with open(CONFIG, encoding='utf-8') as f:
        return yaml.safe_load(f)['twist_mux']['ros__parameters']


def test_use_stamped_pinned_false():
    assert _params()['use_stamped'] is False


def test_lanes_and_estop_lock_unchanged():
    params = _params()
    assert set(params['topics']) == {
        'navigation', 'docking', 'teleop', 'bumper', 'emergency'}
    assert params['topics']['docking']['topic'] == '/cmd_vel_docking'
    assert params['topics']['docking']['priority'] == 15
    assert params['locks']['estop']['topic'] == '/estop'
