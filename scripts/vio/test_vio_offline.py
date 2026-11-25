# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for scripts/vio/vio_offline.py (bridge + gate replay, RTK velocity) and the
ekf_replay / drift_analyse hooks it feeds."""

import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(STACK, 'scripts', 'replay'))

import vio_offline as vo  # noqa: E402
from stereo_vio_bridge.odom_convert import rpy_matrix  # noqa: E402

FIXED = 'quality=RTK_FIXED'
FLOAT = 'quality=RTK_FLOAT'


def test_metoak_bridge_parameters_match_launch():
    with open(os.path.join(STACK, 'launch', 'vio.launch.py')) as fh:
        text = fh.read()
    nums = lambda s: [float(x) for x in re.findall(r'-?\d+\.\d+|-?\d+', s)]  # noqa: E731
    rpy = re.search(r"'imu_to_base_rpy': \[([^\]]*)\]", text).group(1)
    xyz = re.search(r"'imu_xyz_in_base': \[([^\]]*)\]", text).group(1)
    assert nums(rpy) == list(vo.METOAK_BRIDGE['imu_to_base_rpy'])
    assert nums(xyz) == list(vo.METOAK_BRIDGE['imu_xyz_in_base'])


def _ov_row(t, v_imu, w_imu=(0.0, 0.0, 0.0), var=1e-4):
    row = dict(zip(vo.OV_COLUMNS, [t] + [0.0] * 16))
    row['vx'], row['vy'], row['vz'] = v_imu
    row['wx'], row['wy'], row['wz'] = w_imu
    row['var_vx'] = row['var_vy'] = row['var_vz'] = var
    return row


def _to_imu(v_base):
    r = rpy_matrix(*vo.METOAK_BRIDGE['imu_to_base_rpy'])
    return tuple(sum(r[k][i] * v_base[k] for k in range(3)) for i in range(3))


def test_bridge_forward_drive_and_variance_floor():
    (t, vx, vy, vz, var_vx, var_vy, wz), = vo.bridge_rows([_ov_row(5.0, _to_imu((0.3, 0, 0)))])
    assert t == 5.0
    assert abs(vx - 0.3) < 1e-6 and abs(vy) < 1e-6 and abs(vz) < 1e-6
    assert var_vx == 0.04 and var_vy == 0.04          # 10 x 1e-4 floored at (0.2 m/s)^2


def test_bridge_removes_lever_arm_in_a_pivot():
    # turning in place at 0.5 rad/s: the IMU 0.45 m ahead moves sideways at ~0.23 m/s
    wb = (0.0, 0.0, 0.5)
    lx, ly, lz = vo.METOAK_BRIDGE['imu_xyz_in_base']
    v_imu_base = (wb[1] * lz - wb[2] * ly, wb[2] * lx - wb[0] * lz, wb[0] * ly - wb[1] * lx)
    row = _ov_row(0.0, _to_imu(v_imu_base), _to_imu(wb))
    (_t, vx, vy, _vz, _a, _b, wz), = vo.bridge_rows([row])
    assert abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(wz - 0.5) < 1e-6


def _stream(t0, t1, hz=20.0, vx=0.3):
    n = int((t1 - t0) * hz)
    return [(t0 + k / hz, vx, 0.0, 0.0, 0.04, 0.04, 0.0) for k in range(n)]


def test_gate_blocks_while_rtk_fixed_and_passes_in_a_simulated_dropout():
    vio = _stream(0.0, 30.0)
    status = [(0.1 * k, FIXED) for k in range(300)]
    wheel = [(0.02 * k, 0.3) for k in range(1500)]
    verdicts = vo.run_gate(vio, status, wheel, windows=[(10.0, 20.0)])
    by_t = dict(verdicts)
    assert by_t[5.0] == 'RTK fixed'
    assert by_t[10.5] is not None and 'holdoff' in by_t[10.5]   # 1 s hold-off
    assert by_t[11.5] is None and by_t[19.5] is None
    assert by_t[21.0] == 'RTK fixed'
    s = vo.gate_summary([v for v in verdicts if 10.0 <= v[0] < 20.0])
    assert 0.85 < s['passing_share'] < 0.95 and s['diverged_episodes'] == 0


def test_gate_latches_diverged_when_vio_disagrees_with_wheels():
    vio = _stream(0.0, 10.0, vx=0.9)                  # 0.6 m/s off the wheels
    status = [(0.1 * k, FLOAT) for k in range(100)]
    wheel = [(0.02 * k, 0.3) for k in range(500)]
    verdicts = vo.run_gate(vio, status, wheel)
    s = vo.gate_summary(verdicts)
    assert s['diverged_episodes'] == 1 and s['diverged_s'] > 7.0
    assert s['passing_share'] < 0.1


def test_reason_class_strips_numbers():
    assert vo.reason_class(None) == 'PASSING'
    assert vo.reason_class('VIO warming up (3/20)') == vo.reason_class('VIO warming up (19/20)')
    assert vo.reason_class('RTK lost 0.4s ago (holdoff 1.0s)') == \
        vo.reason_class('RTK lost 0.9s ago (holdoff 1.0s)')


def test_full_rate_activity_skips_the_idle_duty_cycle():
    acts = [(10.0, 'docked_idle'), (100.0, 'mowing'), (600.0, 'idle'), (900.0, 'mowing')]
    assert vo.full_rate_activity(acts) == 100.0
    assert vo.full_rate_activity([(10.0, 'manual'), (50.0, 'idle')]) == 0.0
    assert vo.full_rate_activity([]) == 0.0


def test_imu_settled_waits_out_the_rate_switch_gap():
    before = [100.0 + 0.005 * k for k in range(100)]           # 100.0 .. 100.495
    after = [101.7 + 0.005 * k for k in range(1000)]           # 1.2 s gap, then 200 Hz
    assert vo.imu_settled(before + after) == 101.7
    assert vo.imu_settled(after) == 101.7
    assert vo.imu_settled(before) is None


def test_rtk_body_velocity_rotates_into_base_link():
    # driving north-east at 0.5 m/s with yaw 45 deg: pure forward body velocity
    fixes = [(0.1 * k, 0.05 * k * math.cos(math.pi / 4), 0.05 * k * math.sin(math.pi / 4))
             for k in range(50)]
    ts = [f[0] for f in fixes]
    v = vo.rtk_body_velocity(fixes, ts, lambda t: math.pi / 4, 2.0)
    assert abs(v[0] - 0.5) < 1e-9 and abs(v[1]) < 1e-9
    assert vo.rtk_body_velocity(fixes, ts, lambda t: None, 2.0) is None
    assert vo.rtk_body_velocity(fixes, ts, lambda t: 0.0, 4.8) is None      # past the end


def test_percentile_and_interp():
    assert vo.percentile([1, 2, 3, 4], 50) == 2.5
    assert vo.percentile([], 90) != vo.percentile([], 90)                   # nan
    it = vo.Interp([(0.0, 0.0, math.pi - 0.1), (1.0, 1.0, -math.pi + 0.1)], 2.0,
                   angle_cols=(2,))
    x, yaw = it.at(0.5)
    assert abs(x - 0.5) < 1e-12 and abs(abs(yaw) - math.pi) < 1e-9
    assert it.at(2.0) is None


def test_ekf_replay_vio_variant_params_and_thinning(tmp_path):
    import ekf_replay
    name, opts = ekf_replay.parse_variant('o0:fuse_vy=1,vy_var=0.05,vio=1,vio_hz=10,wh_len=100')
    assert name == 'o0' and opts['vio'] == 1 and opts['vio_hz'] == 10
    import yaml
    with open(ekf_replay.write_params(str(tmp_path), name, opts)) as fh:
        p = yaml.safe_load(fh)['/**']['ros__parameters']
    assert p['odom2'] == '/rp/o0/vio'
    assert p['odom2_config'][6] and p['odom2_config'][7] and not p['odom2_config'][0]
    assert p['odom1'] == '/rp/o0/gps'
    _n, plain = ekf_replay.parse_variant('v0:fuse_vy=1')
    with open(ekf_replay.write_params(str(tmp_path), 'v0', plain)) as fh:
        assert 'odom2' not in yaml.safe_load(fh)['/**']['ros__parameters']
    rows = [(0.005 * k, k) for k in range(400)]                            # 200 Hz, 2 s
    assert len(ekf_replay.thin(rows, 10.0)) == 20
    assert ekf_replay.thin(rows, 0.0) == rows


def test_drift_analyse_knows_the_vio_prefix():
    import drift_analyse
    prefixes = dict(drift_analyse.PREFIXES)
    assert prefixes['o'] == 'wheel + VIO' and 'v' in prefixes and 'b' in prefixes
