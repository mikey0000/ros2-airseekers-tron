"""Tests for the stdlib-only host power-off helper and its shell glue (no real /userdata, /run,
/etc, systemctl or sleeping)."""
import ast
import json
import os
import re
import sys

import pytest

from base_keys import poweroff_host as ph

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.dirname(PKG_DIR)                       # .../ros2_stack/src
STACK_DIR = os.path.dirname(SRC_DIR)                     # .../ros2_stack
HOST_PY = os.path.join(PKG_DIR, 'base_keys', 'poweroff_host.py')
MCU_NODE = os.path.join(SRC_DIR, 'mower_mcu_driver', 'mower_mcu_driver', 'mcu_node.py')
MCU_SCRIPT = os.path.join(STACK_DIR, 'docker', 'host', 'power', 'mower-mcu-poweroff')
SYS_CLOSE = os.path.join(STACK_DIR, 'docker', 'host', 'power', 'sys_close')

BOOT = 'boot-abc'


def _read(p):
    with open(p) as f:
        return f.read()


# ---- build_frame: mcu_node imports rclpy at module level, so the encoder is copied here.
# Source of truth: mower_mcu_driver/mcu_node.py escape() / build_frame(); a source-text test
# below pins the constants so the copy cannot silently drift.
SOF, EOF, ESC = 0xA5, 0x5A, 0xF5
TYPE_ROS_MOWER, MOD_SENSOR = 0, 10


def _escape(data):
    out = bytearray()
    for x in data:
        if x == SOF:
            out += b'\xF5\x01'
        elif x == EOF:
            out += b'\xF5\x02'
        elif x == ESC:
            out += b'\xF5\x03'
        else:
            out.append(x)
    return bytes(out)


def build_frame(subpackets):
    body = bytearray()
    for type_id, mod_id, payload in subpackets:
        body += bytes([len(payload), type_id, mod_id]) + bytes(payload)
    inner = bytes([len(body)]) + bytes(body)
    checksum = sum(inner) & 0xFF
    return bytes([SOF]) + _escape(inner + bytes([checksum])) + bytes([EOF])


@pytest.fixture
def cfg():
    c = dict(ph.DEFAULTS)
    c['KERNEL_GRACE_S'] = '0'
    return c


@pytest.fixture
def host(tmp_path, monkeypatch):
    """Isolated host: fake boot id/uptime, marker under tmp_path, no real config file."""
    monkeypatch.setattr(ph, 'read_boot_id', lambda *a, **k: BOOT)
    monkeypatch.setattr(ph, 'read_uptime', lambda *a, **k: 1000.0)
    monkeypatch.setattr(ph, 'MCU_CUT_MARKER', str(tmp_path / 'run' / 'mcu_cut'))
    monkeypatch.setattr(ph, 'CONFIG_FILE', str(tmp_path / 'nonexistent.cfg'))
    d = tmp_path / 'power'
    d.mkdir()
    return d


class Runner:
    def __init__(self, rc=0):
        self.calls = []
        self.rc = rc

    def __call__(self, argv):
        self.calls.append(list(argv))
        return self.rc


def good_req(source='base_keys', uptime=990.0, **kw):
    return ph.make_request(source, BOOT, uptime, wall_time=1.0, **kw)


# ---------------------------------------------------------------- make_request
def test_make_request_rejects_unknown_source():
    with pytest.raises(ValueError):
        ph.make_request('bogus', BOOT, 100.0)


def test_make_request_fields():
    r = ph.make_request('manual', BOOT, 123.45678, wall_time=5.0, note='x', n=2)
    assert r['boot_id'] == BOOT and r['source'] == 'manual'
    assert r['uptime_s'] == 123.457
    assert r['note'] == 'x' and r['n'] == 2 and r['version'] == 1
    assert r['wall_time'] == 5.0


# ---------------------------------------------------------------- write/read
def test_write_request_atomic_and_roundtrip(tmp_path):
    req = good_req()
    p = ph.write_request(req, str(tmp_path))
    assert p == str(tmp_path / ph.REQUEST_NAME)
    assert sorted(os.listdir(tmp_path)) == [ph.REQUEST_NAME]      # no .tmp leftovers
    assert ph.read_request(p) == req
    ph.write_request(dict(req, source='kernel'), str(tmp_path))   # overwrite
    assert ph.read_request(p)['source'] == 'kernel'
    assert sorted(os.listdir(tmp_path)) == [ph.REQUEST_NAME]


def test_read_request_missing_and_garbage(tmp_path):
    assert ph.read_request(str(tmp_path / 'nope')) is None
    g = tmp_path / 'g'
    g.write_text('{not json')
    assert '_error' in ph.read_request(str(g))
    g.write_text('[1, 2]')
    assert ph.read_request(str(g)) == {'_error': 'not an object'}


# ---------------------------------------------------------------- validate_request
def val(req, uptime=1000.0, boot=BOOT, **kw):
    return ph.validate_request(req, boot, uptime, **kw)


def test_validate_ok():
    ok, why = val(good_req(uptime=990.0))
    assert ok and why.startswith('ok')


def test_validate_boot_id_mismatch():
    ok, why = val(good_req(), boot='other')
    assert not ok and 'boot_id' in why


def test_validate_empty_current_boot_id():
    ok, _ = val(good_req(), boot='')
    assert not ok


def test_validate_too_old():
    ok, why = val(good_req(uptime=500.0), uptime=1000.0, max_age_s=120.0)
    assert not ok and 'old' in why
    assert val(good_req(uptime=880.0), uptime=1000.0, max_age_s=120.0)[0]   # exactly at limit


def test_validate_future_uptime():
    ok, why = val(good_req(uptime=1100.0), uptime=1000.0)
    assert not ok and 'future' in why
    assert val(good_req(uptime=1000.5), uptime=1000.0)[0]                    # 1 s tolerance


def test_validate_too_early_after_boot():
    ok, why = val(good_req(uptime=10.0), uptime=20.0, min_uptime_s=30.0)
    assert not ok and 'after boot' in why


def test_validate_unknown_source():
    r = good_req()
    r['source'] = 'evil'
    ok, why = val(r)
    assert not ok and 'source' in why


def test_validate_error_passthrough_and_none():
    assert val({'_error': 'bad json: x'}) == (False, 'bad json: x')
    assert val(None) == (False, 'no request')


def test_validate_missing_uptime():
    r = good_req()
    del r['uptime_s']
    assert not val(r)[0]


# ---------------------------------------------------------------- config
def test_parse_config():
    text = '''# comment
DRY_RUN=0   # trailing
export MCU_POWER_CUT="1"
MIN_UPTIME_S = '45'
garbage line
'''
    c = ph.parse_config(text)
    assert c['DRY_RUN'] == '0'
    assert c['MCU_POWER_CUT'] == '1'
    assert c['MIN_UPTIME_S'] == '45'
    assert c['MAX_AGE_S'] == ph.DEFAULTS['MAX_AGE_S']             # default preserved
    assert c['KERNEL_GRACE_S'] == ph.DEFAULTS['KERNEL_GRACE_S']
    assert ph.DEFAULTS['DRY_RUN'] == '1'                          # DEFAULTS not mutated


@pytest.mark.parametrize('v', ['1', 'true', 'TRUE', 'Yes', 'on', ' 1 '])
def test_cfg_bool_truthy(v):
    assert ph.cfg_bool({'DRY_RUN': v}, 'DRY_RUN')


@pytest.mark.parametrize('v', ['0', 'false', 'no', 'off', '', 'maybe'])
def test_cfg_bool_falsy(v):
    assert not ph.cfg_bool({'DRY_RUN': v}, 'DRY_RUN')


def test_cfg_float_fallback():
    assert ph.cfg_float({'MAX_AGE_S': 'abc'}, 'MAX_AGE_S') == 120.0
    assert ph.cfg_float({'MAX_AGE_S': '7.5'}, 'MAX_AGE_S') == 7.5
    assert ph.cfg_float({}, 'MIN_UPTIME_S') == 30.0


# ---------------------------------------------------------------- plan_actions
def test_plan_dry_run():
    acts = ph.plan_actions({'DRY_RUN': '1', 'MCU_POWER_CUT': '1'})
    assert acts[-1] == 'log_dry_run' and 'poweroff' not in acts


def test_plan_real_with_mcu_cut():
    assert ph.plan_actions({'DRY_RUN': '0', 'MCU_POWER_CUT': '1'}) == \
        ['archive_request', 'arm_mcu_cut', 'sync', 'poweroff']


def test_plan_no_mcu_cut():
    acts = ph.plan_actions({'DRY_RUN': '0', 'MCU_POWER_CUT': '0'})
    assert 'arm_mcu_cut' not in acts
    assert acts == ['archive_request', 'sync', 'poweroff']


# ---------------------------------------------------------------- wait_for_stack
def _no_sleep(_):
    raise AssertionError('must not sleep')


def test_wait_non_kernel_returns_immediately():
    req = good_req('base_keys')
    assert ph.wait_for_stack('p', req, 6.0, sleep=_no_sleep,
                             reader=lambda p: pytest.fail('no read')) is req
    assert ph.wait_for_stack('p', None, 6.0, sleep=_no_sleep) is None
    kreq = good_req('kernel')
    assert ph.wait_for_stack('p', kreq, 0, sleep=_no_sleep) is kreq


class FakeClock:
    def __init__(self):
        self.t = 0.0
        self.slept = []

    def clock(self):
        return self.t

    def sleep(self, s):
        self.slept.append(s)
        self.t += s


def test_wait_kernel_picks_up_base_keys_request():
    kreq, breq = good_req('kernel'), good_req('base_keys')
    reads = iter([kreq, kreq, breq])
    fc = FakeClock()
    out = ph.wait_for_stack('p', kreq, 6.0, sleep=fc.sleep, reader=lambda p: next(reads),
                            clock=fc.clock)
    assert out is breq
    assert len(fc.slept) == 3 and fc.t < 6.0


def test_wait_kernel_times_out_with_kernel_request():
    kreq = good_req('kernel')
    fc = FakeClock()
    out = ph.wait_for_stack('p', kreq, 1.0, sleep=fc.sleep, reader=lambda p: kreq, clock=fc.clock)
    assert out is kreq
    assert fc.t >= 1.0


def test_wait_kernel_request_vanished_falls_back_to_original():
    kreq = good_req('kernel')
    fc = FakeClock()
    out = ph.wait_for_stack('p', kreq, 0.5, sleep=fc.sleep, reader=lambda p: None, clock=fc.clock)
    assert out is kreq


# ---------------------------------------------------------------- handle
def test_handle_missing_request(host, cfg):
    run = Runner()
    assert ph.handle(str(host), cfg, run=run) == 0
    assert run.calls == []


def test_handle_stale_request_deleted_no_systemctl(host, cfg):
    cfg['DRY_RUN'] = '0'
    ph.write_request(ph.make_request('base_keys', 'old-boot', 990.0), str(host))
    run = Runner()
    assert ph.handle(str(host), cfg, run=run) == 0
    assert not (host / ph.REQUEST_NAME).exists()
    assert run.calls == []
    assert not (host / ph.LAST_NAME).exists()


def test_handle_valid_dry_run(host, cfg, tmp_path):
    cfg.update(DRY_RUN='1', MCU_POWER_CUT='1')
    ph.write_request(good_req(), str(host))
    run = Runner()
    assert ph.handle(str(host), cfg, run=run) == 0
    assert not (host / ph.REQUEST_NAME).exists()
    last = json.loads((host / ph.LAST_NAME).read_text())
    assert last['dry_run'] is True and last['source'] == 'base_keys'
    assert run.calls == [['sync']]
    assert not os.path.exists(ph.MCU_CUT_MARKER)


def test_handle_valid_real_poweroff(host, cfg):
    cfg.update(DRY_RUN='0', MCU_POWER_CUT='1')
    ph.write_request(good_req(), str(host))
    run = Runner(rc=0)
    assert ph.handle(str(host), cfg, run=run) == 0
    assert run.calls == [['sync'], ['systemctl', 'poweroff', '--no-block']]
    assert os.path.isfile(ph.MCU_CUT_MARKER)
    assert open(ph.MCU_CUT_MARKER).read().strip() == 'base_keys'
    assert json.loads((host / ph.LAST_NAME).read_text())['dry_run'] is False
    assert not (host / ph.REQUEST_NAME).exists()


def test_handle_real_without_mcu_cut_creates_no_marker(host, cfg):
    cfg.update(DRY_RUN='0', MCU_POWER_CUT='0')
    ph.write_request(good_req(), str(host))
    run = Runner()
    ph.handle(str(host), cfg, run=run)
    assert not os.path.exists(ph.MCU_CUT_MARKER)
    assert run.calls[-1] == ['systemctl', 'poweroff', '--no-block']


def test_handle_propagates_systemctl_rc(host, cfg):
    cfg.update(DRY_RUN='0')
    ph.write_request(good_req(), str(host))
    assert ph.handle(str(host), cfg, run=Runner(rc=5)) == 5


# ---------------------------------------------------------------- main
def test_main_request_kernel(host, monkeypatch, capsys):
    assert ph.main(['--dir', str(host), 'request', '--source', 'kernel']) == 0
    req = ph.read_request(str(host / ph.REQUEST_NAME))
    assert req['source'] == 'kernel' and req['boot_id'] == BOOT and req['uptime_s'] == 1000.0
    assert 'request written' in capsys.readouterr().out


def test_main_request_missing_dir(host, tmp_path, capsys):
    assert ph.main(['--dir', str(tmp_path / 'missing'), 'request']) == 1
    assert not (tmp_path / 'missing').exists()


def test_main_frame_prints_octal(capsys):
    assert ph.main(['frame']) == 0
    out = capsys.readouterr().out.strip()
    assert out == ''.join('\\%03o' % b for b in ph.MCU_POWEROFF_FRAME)
    assert out.startswith('\\245\\013')


def test_main_usage(capsys):
    assert ph.main(['bogus']) == 2


# ---------------------------------------------------------------- frame / host scripts
def test_mcu_poweroff_frame_matches_build_frame():
    assert ph.MCU_POWEROFF_FRAME == build_frame([(TYPE_ROS_MOWER, MOD_SENSOR,
                                                  bytes([0, 0, 0, 0, 1, 0, 0, 0]))])


def test_mcu_node_constants_match_test_copy():
    src = _read(MCU_NODE)
    assert re.search(r'^SOF, EOF, ESC = 0xA5, 0x5A, 0xF5$', src, re.M)
    assert re.search(r'^TYPE_ROS_MOWER = 0\b', src, re.M)
    assert re.search(r'^MOD_SENSOR = 10\b', src, re.M)
    assert "b'\\xF5\\x01'" in src and "b'\\xF5\\x02'" in src and "b'\\xF5\\x03'" in src
    assert 'bytes([len(payload), type_id, mod_id])' in src


def test_shell_script_printf_decodes_to_frame():
    text = _read(MCU_SCRIPT)
    m = re.search(r"printf '((?:\\[0-7]{3})+)'", text)
    assert m, 'printf octal string not found'
    decoded = bytes(int(o, 8) for o in re.findall(r'\\([0-7]{3})', m.group(1)))
    assert decoded == ph.MCU_POWEROFF_FRAME


def test_shell_script_guards_and_tty():
    text = _read(MCU_SCRIPT)
    assert '[ "$1" = "poweroff" ] || exit 0' in text
    assert '[ -e /run/mower-poweroff/mcu_cut ] || exit 0' in text
    assert 'TTY=/dev/ttyS9' in text
    assert ph.MCU_CUT_MARKER == '/run/mower-poweroff/mcu_cut'


def test_sys_close_requests_kernel_source():
    text = _read(SYS_CLOSE)
    assert 'request --source kernel' in text
    assert 'mower-poweroff-host' in text


# ---------------------------------------------------------------- py3.8 / stdlib only
def test_poweroff_host_stdlib_only_and_py38_syntax():
    src = _read(HOST_PY)
    tree = ast.parse(src, feature_version=(3, 8))        # rejects match / newer syntax
    allowed = {'json', 'os', 'subprocess', 'sys', 'time'}
    mods = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            mods |= {a.name.split('.')[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom):
            assert n.level == 0, 'relative import'
            mods.add((n.module or '').split('.')[0])
    assert mods <= allowed, mods - allowed
    assert not any(isinstance(n, ast.NamedExpr) for n in ast.walk(tree))
    assert not hasattr(ast, 'Match') or not any(isinstance(n, ast.Match) for n in ast.walk(tree))
