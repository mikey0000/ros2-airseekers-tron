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
