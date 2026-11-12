# Copyright 2026 Airseekers Tron ROS 2 port contributors
# SPDX-License-Identifier: GPL-3.0-or-later
from types import SimpleNamespace

from mower_mission import mow_progress
from mower_mission.geometry import cumulative_lengths
from mower_mission.resume import AreaCursor, ResumeCursor


def _fsm(sub_i=1, step='follow', local=2, completed=(0,), skipped=(), failed=()):
    subs = [[(float(x), 0.0, 0.0) for x in range(4)], [(float(x), 1.0, 0.0) for x in range(5)],
            [(float(x), 2.0, 0.0) for x in range(3)]]
    cum = [cumulative_lengths(sp) for sp in subs]
    m = SimpleNamespace(area_idx=0, subpaths=subs, cum=cum, lengths=[c[-1] for c in cum],
                        sub_i=sub_i, step=step, progress_local=local, start_local=0,
                        run_i=0, failed=list(failed))
    cur = ResumeCursor()
    cur.areas[0] = AreaCursor(12, 1, -1, set(completed), list(skipped))
    return SimpleNamespace(mission=m, cursor=cur, phase='MOWING', sub_state='x',
                           display_sub_state=lambda: 'mowing')


def test_geometry_and_progress():
    f = _fsm()
    pid, area, subs = mow_progress.plan_geometry(f)
    assert area == 0 and len(subs) == 3 and subs[1][0] == [0.0, 1.0]
    st = mow_progress.progress(f, pid, True, 120.0, 4.0)
    assert st['total_poses'] == 12 and st['sub_paths'] == 3 and st['sub_path'] == 1
    # sub 0 (0..3) + current 4..6 merge into one stretch
    assert st['mowed_segments'] == [[0, 6]]
    assert st['current_segment'] == [6, 8]
    assert st['pose_index'] == 6
    assert st['mowed_m'] == 5.0 and st['remaining_m'] == 4.0
    assert st['percent'] == round(100 * 5 / 9, 1)
    assert st['eta_s'] == 120  # 4 m in 120 s -> 4 m left in 120 s


def test_skipped_and_failed():
    f = _fsm(sub_i=2, step='transit', local=0, completed=(0,), skipped=[(5, 6)],
             failed=[(0, 1, 'blocked', 6), (1, 0, 'other area', 0)])
    st = mow_progress.progress(f, 'p', False, None, 0.0)
    assert st['skipped'] == [[5, 8]]
    assert st['current_segment'] == [9, 11]
    assert st['eta_s'] is None


def test_idle():
    f = SimpleNamespace(mission=None, phase='IDLE', display_sub_state=lambda: '')
    assert mow_progress.plan_geometry(f) is None
    st = mow_progress.progress(f, None, False, None, 0.0)
    assert st['area'] == -1 and st['mowed_segments'] == []


def test_plan_geometry_keeps_a_straight_row_straight():
    # A 105 deg row with 5 cm pose spacing: at 1 cm rounding the published points zigzagged
    # +-5 mm across the row, which the GUI drew as a wobble. 1 mm keeps them on the line.
    import math
    d = math.radians(105.0)
    row = [(-23.7213 + 0.05 * k * math.cos(d), -8.2717 + 0.05 * k * math.sin(d), d)
           for k in range(200)]
    m = SimpleNamespace(area_idx=4, subpaths=[row], run_i=0)
    pid, _area, subs = mow_progress.plan_geometry(SimpleNamespace(mission=m))
    nx, ny = -math.sin(d), math.cos(d)
    off = [(x - row[0][0]) * nx + (y - row[0][1]) * ny for x, y in subs[0]]
    assert max(abs(o) for o in off) <= 0.0008
    assert pid == '4:0:1:200:-23.72,-8.27'
