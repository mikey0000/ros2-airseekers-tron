"""Static checks of the dual-EKF parameter files (PyYAML only, no ROS).

robot_localization fails silently or at launch on exactly these mistakes: a gap in the
odomN numbering (it stops at the first missing index), a mixed int/float array (rcl
refuses it), a wrong-length config row.
"""

import os
import re

import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load(name):
    with open(os.path.join(PKG, 'config', name)) as f:
        return yaml.safe_load(f)


def _params(doc, node):
    return doc[node]['ros__parameters']


def _merge(*docs):
    """Overlay semantics of passing several --params-file / parameter dicts in order."""
    out = {}
    for doc in docs:
        for node, body in doc.items():
            out.setdefault(node, {}).update(body['ros__parameters'])
    return out


def _indices(params, prefix):
    return sorted(int(m.group(1)) for k in params
                  for m in [re.fullmatch(prefix + r'(\d+)', k)] if m)


@pytest.fixture(scope='module')
def dual():
    return _load('ekf_dual.yaml')


@pytest.fixture(scope='module')
def single():
    return _load('ekf.yaml')


class TestDualStructure:
    def test_sections(self, dual):
        assert set(dual) == {'ekf_node', 'ekf_map'}

    def test_world_frames_and_tf(self, dual):
        assert _params(dual, 'ekf_node')['world_frame'] == 'odom'
        assert _params(dual, 'ekf_map')['world_frame'] == 'map'
        for node in dual:
            p = _params(dual, node)
            assert p['publish_tf'] is True
            assert p['two_d_mode'] is True
            assert (p['map_frame'], p['odom_frame'], p['base_link_frame']) == \
                ('map', 'odom', 'base_link')

    def test_odom_filter_has_no_gps(self, dual):
        p = _params(dual, 'ekf_node')
        assert not [k for k, v in p.items()
                    if re.fullmatch(r'odom\d+', k) and v == '/odometry/gps']
        assert p['odom0'] == '/odom'
        assert p['imu0'] == '/imu/data_aligned'

    def test_map_filter_fuses_gps_xy_only(self, dual):
        p = _params(dual, 'ekf_map')
        assert p['odom1'] == '/odometry/gps'
        cfg = p['odom1_config']
        assert [i for i, v in enumerate(cfg) if v] == [0, 1]

    def test_no_gps_pose_rejection_gate_in_map_filter(self, dual):
        # a Mahalanobis gate would reject the first fixes away from the datum
        assert 'odom1_pose_rejection_threshold' not in _params(dual, 'ekf_map')

    def test_config_rows_are_15_booleans(self, dual):
        for node in dual:
            for k, v in _params(dual, node).items():
                if k.endswith('_config'):
                    assert len(v) == 15 and all(isinstance(b, bool) for b in v), (node, k)

    def test_process_noise_is_225_floats(self, dual):
        for node in dual:
            q = _params(dual, node)['process_noise_covariance']
            assert len(q) == 225
            assert all(isinstance(v, float) for v in q), node


class TestSameSensorRowsAsSingleFilter:
    def test_odom0_imu0_equal_ekf_yaml(self, dual, single):
        ref = _params(single, 'ekf_node')
        for node in dual:
            p = _params(dual, node)
            assert p['odom0_config'] == ref['odom0_config'], node
            assert p['imu0_config'] == ref['imu0_config'], node


class TestVioOverlay:
    def test_dual_odom_indices_contiguous(self, dual):
        merged = _merge(dual, _load('ekf_dual_vio.yaml'))
        for node, p in merged.items():
            idx = _indices(p, 'odom')
            assert idx == list(range(len(idx))), (node, idx)

    def test_vio_fuses_only_vx_vy_twist(self, dual):
        merged = _merge(dual, _load('ekf_dual_vio.yaml'))
        found = 0
        for node, p in merged.items():
            for k, v in p.items():
                if re.fullmatch(r'odom\d+', k) and v == '/odometry/vio_gated':
                    cfg = p[k + '_config']
                    assert [i for i, b in enumerate(cfg) if b] == [6, 7], (node, k)
                    found += 1
        assert found == 2   # both filters take VIO

    def test_single_filter_overlay_contiguous(self, single):
        merged = _merge(single, _load('ekf_vio.yaml'))
        idx = _indices(merged['ekf_node'], 'odom')
        assert idx == list(range(len(idx)))
        assert merged['ekf_node']['odom2'] == '/odometry/vio_gated'

    def test_vio_overlay_arrays_are_valid(self):
        for name in ('ekf_dual_vio.yaml', 'ekf_vio.yaml'):
            doc = _load(name)
            for node in doc:
                for k, v in _params(doc, node).items():
                    if k.endswith('_config'):
                        assert len(v) == 15 and all(isinstance(b, bool) for b in v)


class TestPackaging:
    def _setup_py(self):
        with open(os.path.join(PKG, 'setup.py')) as f:
            return f.read()

    def test_configs_installed(self):
        src = self._setup_py()
        assert 'config/ekf_dual.yaml' in src
        assert 'config/ekf_dual_vio.yaml' in src

    def test_monitor_entry_point(self):
        assert 'localization_monitor = mower_localization.localization_monitor:main' \
            in self._setup_py()
