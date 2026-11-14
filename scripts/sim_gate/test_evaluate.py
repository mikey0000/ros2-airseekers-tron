"""evaluate.py INVALID guards (python3 -m pytest scripts/sim_gate, in the dev image)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate as E  # noqa: E402

START = ('[mission_node-14] [INFO] [1793628266.171288031] [behavior_tree_node]: '
         'high_level_control(1) -> True [PREFLIGHT_CHECK]')


def missed(desired, current=16.6667):
    return ('[controller_server-5] [WARN] [1793628297.687577293] [controller_server]: Control '
            'loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz.'
            % (desired, current))


def test_unscaled_rate_after_start_is_a_race():
    assert E.rate_race(['boot', START, missed(20.0)], 3.0)


def test_scaled_rate_is_not_a_race():
    assert not E.rate_race(['boot', START, missed(60.0), missed(60.0, 20.0)], 3.0)
    assert not E.rate_race([START, missed(40.0)], 2.0)


def test_real_time_never_a_race():
    assert not E.rate_race([START, missed(20.0)], 1.0)


def test_warnings_before_start_ignored():
    assert not E.rate_race([missed(20.0), START, missed(60.0)], 3.0)


def test_no_start_line_checks_the_whole_log():
    assert E.rate_race([missed(20.0)], 3.0)
    assert not E.rate_race([], 3.0)


def test_run_rate(tmp_path):
    assert E.run_rate(str(tmp_path)) == 1.0
    (tmp_path / 'run.txt').write_text('world=w.yaml\nrate=3\ncommit=abc\n')
    assert E.run_rate(str(tmp_path)) == 3.0


# ---- 2026-11-03 sim fidelity: slip windows, RTK-loss handling ------------------------------

def test_slip_events_flag_spinning_wheels_only():
    gt = [[t * 0.1, 0.0, 0.001 * t, 0.0] for t in range(200)]          # 1 cm/s real
    spin = [[t * 0.1, 0.0, -0.01 * t] for t in range(200)]             # 0.1 m/s wheels
    ev = E.slip_events(spin, gt)
    assert len(ev) == 1 and ev[0][0] == 0.0 and ev[0][2] >= 0.29
    honest = [[t * 0.1, 0.0, 0.001 * t] for t in range(200)]
    assert E.slip_events(honest, gt) == []


def test_rtk_windows():
    simx = [[t, None, None, None, m] for t, m in
            [(0, 'fixed'), (1, 'float'), (2, 'float'), (3, 'fixed'), (5, 'dropout'), (6, 'dropout')]]
    assert E.rtk_windows(simx) == [[1, 3, 'float'], [5, 6, 'dropout']]
    simx = [[t, None, None, None, m] for t, m in
            [(0, 'fixed'), (1, 'single'), (2, 'single'), (3, 'float'), (4, 'float'), (5, 'fixed')]]
    assert E.rtk_windows(simx) == [[1, 5, 'single+float']]


def test_rtk_handling_hold_resume():
    win = [[100.0, 160.0, 'dropout']]
    loc = [[90.0, 'ok'], [130.0, 'degraded'], [175.0, 'ok']]
    bt = [(131.0, 'localization degraded (estimated drift 0.31 m > budget 0.30 m (rtk stale)): '
                  'stopped, blade off, waiting for RTK fixed'),
          (179.0, 'localization ok again after 48 s: resuming')]
    cut = [(130.5, True), (131.5, False), (133.0, False)]
    e, = E.rtk_handling(win, loc, bt, cut, [])
    assert e['held'] and e['blade_off'] and e['resumed'] and e['ok']
    late = [(136.0, bt[0][1]), bt[1]]
    assert not E.rtk_handling(win, loc, late, cut, [])[0]['ok']
    assert not E.rtk_handling(win, loc, bt, cut, [150.0])[0]['ok']     # excursion
    quiet, = E.rtk_handling(win, [[90.0, 'ok']], [], cut, [])
    assert quiet['ok'] and 'no hold needed' in quiet['note']


def test_coverage_report_ground_truth_cut_and_recut_after_a_resume():
    lawn = [(0.0, 0.0), (4.0, 0.0), (4.0, 1.0), (0.0, 1.0)]
    gt, states = [], []
    # base_link 0.23 m behind the blade; row y 0.5 x 0 -> 3, STOP, back to x 2 then on to 3.5
    xs = [0.05 * k for k in range(61)] + [3.0 - 0.05 * k for k in range(21)] + \
        [2.0 + 0.05 * k for k in range(31)]
    for k, x in enumerate(xs):
        t = 0.1 * k
        yaw = 0.0
        gt.append([t, x - 0.23, 0.5, yaw, 0.0, 0.0])
        states.append((t, 'MOWING' if k < 61 or k >= 81 else 'IDLE'))
    status = [[t, False, False, 0, s == 'MOWING', False] for t, s in states]
    T = [t for t, _ in states]

    def st(t):
        import bisect
        return states[max(0, bisect.bisect_right(T, t) - 1)][1]
    progress = [[10.0, "MOWING", 0, 1, True, 80.0, 25.0, 0], [11.0, "IDLE", -1, 0, False, 0.0, None, -1]]
    cov = E.coverage_report(gt, status, progress, st, lawn, 0.0)
    assert cov['mission_percent'] == 25.0 and cov['plan_percent'] == 80.0
    assert 16.0 < cov['gt_percent'] < 21.0       # ~3.5 x 0.22 of 4 m2 (4 cell rows at 5 cm)
    assert cov['stop_t'] == 6.1 and cov['resume_t'] == 8.1
    assert 0.9 <= cov['recut_after_resume_m'] <= 1.1          # x 2 -> 3 again
    assert 1.4 <= cov['mowed_after_resume_m'] <= 1.6
