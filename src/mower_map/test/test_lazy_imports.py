"""M3 import diet: obstacle_tracker must not load cv2 (~45 MB) until a snapshot is encoded."""

import subprocess
import sys
import textwrap

import pytest


def _run(code):
    return subprocess.run([sys.executable, '-c', textwrap.dedent(code)],
                          capture_output=True, text=True, timeout=60)


def test_cv2_not_loaded_by_node_construction():
    r = _run("""
        import sys
        import rclpy
        from mower_map.obstacle_tracker_node import ObstacleTrackerNode
        rclpy.init()
        node = ObstacleTrackerNode()
        assert 'cv2' not in sys.modules, 'cv2 imported eagerly'
        node.destroy_node()
        rclpy.shutdown()
        print('OK')
    """)
    assert r.returncode == 0 and 'OK' in r.stdout, r.stdout + r.stderr


def test_cv2_loaded_on_first_snapshot(tmp_path):
    pytest.importorskip('cv2')
    r = _run("""
        import sys
        from mower_map import obstacle_snapshot as snap
        assert 'cv2' not in sys.modules
        path = snap.save_snapshot(%r, 3, bytes(4 * 4 * 3), 'bgr8', 4, 4)
        assert path and path.endswith('3.jpg'), path
        assert 'cv2' in sys.modules
        print('OK')
    """ % str(tmp_path))
    assert r.returncode == 0 and 'OK' in r.stdout, r.stdout + r.stderr
