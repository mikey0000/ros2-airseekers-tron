# SPDX-License-Identifier: GPL-3.0-or-later
"""heading_aligner reads mower_map's dock_pose_measured flag (needs rclpy for the import)."""
import pytest

try:
    from mower_localization.heading_aligner import _read_dock_measured
except ImportError:  # no real rclpy here
    pytest.skip('needs rclpy', allow_module_level=True)


def test_read_dock_measured(tmp_path):
    p = tmp_path / 'dock_pose.yaml'
    p.write_text('dock_pose_x: 0.0\ndock_pose_y: 0.0\ndock_pose_yaw: 0.0\n')
    assert _read_dock_measured(str(p)) is False
    p.write_text('dock_pose_x: 1.0\ndock_pose_measured: true\n')
    assert _read_dock_measured(str(p)) is True
    assert _read_dock_measured(str(tmp_path / 'missing.yaml')) is False
