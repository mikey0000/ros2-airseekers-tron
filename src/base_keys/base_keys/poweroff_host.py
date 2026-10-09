#!/usr/bin/env python3
"""Power-button shutdown: the request file shared by the container and the mower HOST.

This one file is used in two places:

* inside ``mower_humble``: ``base_keys`` imports :func:`make_request` / :func:`write_request`
  to ask the host for a power-off after the 3 s power-key hold (``power_long_action:=sequence``);
* on the HOST (Ubuntu 20.04, python3.8, stdlib only): installed as
  ``/usr/local/sbin/mower-poweroff-host`` by ``scripts/install_power_button.sh`` and run by
  ``mower-poweroff.service`` (``handle``) and by the kernel hook ``/usr/bin/sys_close``
  (``request --source kernel``).

Keep it python3.8 compatible and free of ROS imports.

Flow (docs/buttons.md "Power off"):

1. vendor kernel module ``init_test.ko`` polls the power GPIO every 40 ms and emits KEY_L after
   75 ticks (3.0 s) while the key is still held, then KEY_R + ``call_usermodehelper
   /usr/bin/sys_close`` on release;
2. ``base_keys`` (KEY_L): e-stop latch + mission STOP + cutter off + PowerOff light, waits
   ``settle_s``, then writes the request (``source=base_keys``);
3. ``/usr/bin/sys_close`` (kernel, on release, works even when the stack is down) writes a
   ``source=kernel`` request; ``handle`` then gives the stack ``KERNEL_GRACE_S`` to replace it;
4. ``mower-poweroff.path`` (PathExists) starts ``mower-poweroff.service`` -> ``handle``: validate
   (same boot_id, request fresh, uptime >= MIN_UPTIME_S), delete + archive the request, arm the
   MCU power cut marker, ``sync``, ``systemctl poweroff --no-block``;
5. systemd stops docker (mower_humble gets SIGINT, 30 s grace: bag/mow recorders close),
   unmounts everything, then ``/usr/lib/systemd/system-shutdown/mower-mcu-poweroff`` sends the
   vendor ``/poweroff`` frame (module 10, byte 4 = 1) so the MCU cuts the battery.
"""
import json
import os
import subprocess
import sys
import time

REQUEST_DIR = '/userdata/ros2/power'
REQUEST_NAME = 'shutdown_request'
LAST_NAME = 'last_poweroff.json'
CONFIG_FILE = '/etc/default/mower-poweroff'
MCU_CUT_MARKER = '/run/mower-poweroff/mcu_cut'
BOOT_ID_PATH = '/proc/sys/kernel/random/boot_id'
UPTIME_PATH = '/proc/uptime'

SOURCES = ('base_keys', 'kernel', 'manual')

# Vendor PowerManager::poweroffROS: SensorInfoControl (module 10, type 0 host->MCU), 8 bytes,
# byte 4 (power_off) = 1. Wire frame per mcu_node.build_frame (SOF, len, [len,type,mod,payload],
# sum8, EOF); nothing needs escaping. Test: test_poweroff_host.py checks it against build_frame.
MCU_POWEROFF_FRAME = bytes([0xA5, 0x0B, 0x08, 0x00, 0x0A,
                            0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                            0x1E, 0x5A])

DEFAULTS = {
    'DRY_RUN': '1',          # 1: log what would happen, never power off (first install)
    'MCU_POWER_CUT': '0',    # 1: arm the system-shutdown hook that tells the MCU to cut power
    'MIN_UPTIME_S': '30',    # ignore requests this early after boot
    'MAX_AGE_S': '120',      # ignore requests older than this (kernel uptime clock)
    'KERNEL_GRACE_S': '6',   # source=kernel: wait this long for the stack's own request
}


# --------------------------------------------------------------------------- helpers
def read_boot_id(path=BOOT_ID_PATH):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ''


def read_uptime(path=UPTIME_PATH):
    try:
        with open(path) as f:
            return float(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        return 0.0


def parse_config(text, defaults=None):
    """KEY=VALUE lines (shell-style /etc/default file); comments and quotes tolerated."""
    cfg = dict(DEFAULTS if defaults is None else defaults)
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if '=' not in line:
            continue
        k, v = line.split('=', 1)
        k = k.strip()
        if k.startswith('export '):
            k = k[len('export '):].strip()
        cfg[k] = v.strip().strip('"').strip("'")
    return cfg


def load_config(path=CONFIG_FILE):
    try:
        with open(path) as f:
            return parse_config(f.read())
    except OSError:
        return dict(DEFAULTS)


def cfg_bool(cfg, key):
    return str(cfg.get(key, DEFAULTS.get(key, '0'))).strip().lower() in ('1', 'true', 'yes', 'on')


def cfg_float(cfg, key):
    try:
        return float(cfg.get(key, DEFAULTS[key]))
    except (TypeError, ValueError):
        return float(DEFAULTS[key])


# --------------------------------------------------------------------------- request
def make_request(source, boot_id, uptime_s, wall_time=None, **extra):
    if source not in SOURCES:
        raise ValueError('unknown source %r' % (source,))
    req = {'version': 1, 'source': source, 'boot_id': boot_id,
           'uptime_s': round(float(uptime_s), 3),
           'wall_time': round(float(time.time() if wall_time is None else wall_time), 3)}
    for k, v in extra.items():
        req[k] = v
    return req


def write_request(req, directory=REQUEST_DIR, name=REQUEST_NAME):
    """Atomic write (tmp + rename) so the path unit never sees a half-written file."""
    path = os.path.join(directory, name)
    tmp = path + '.tmp.%d' % os.getpid()
    with open(tmp, 'w') as f:
        json.dump(req, f, sort_keys=True)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    os.rename(tmp, path)
    return path


def read_request(path):
    """Parsed dict, or None if missing; {'_error': ...} if unreadable."""
    try:
        with open(path) as f:
            text = f.read()
    except FileNotFoundError:
        return None
    except OSError as e:
        return {'_error': 'read failed: %s' % e}
    try:
        req = json.loads(text)
    except ValueError as e:
        return {'_error': 'bad json: %s' % e}
    if not isinstance(req, dict):
        return {'_error': 'not an object'}
    return req


def validate_request(req, boot_id, uptime_s, min_uptime_s=30.0, max_age_s=120.0):
    """Return (ok, reason). Rejects stale (previous boot / old), early-boot and malformed requests."""
    if req is None:
        return False, 'no request'
    if '_error' in req:
        return False, req['_error']
    if req.get('source') not in SOURCES:
        return False, 'unknown source %r' % (req.get('source'),)
    if not boot_id or req.get('boot_id') != boot_id:
        return False, 'boot_id mismatch (stale request from an earlier boot)'
    try:
        req_up = float(req.get('uptime_s'))
    except (TypeError, ValueError):
        return False, 'missing uptime_s'
    if req_up > uptime_s + 1.0:
        return False, 'request uptime %.1f s is in the future (now %.1f s)' % (req_up, uptime_s)
    age = uptime_s - req_up
    if age > max_age_s:
        return False, 'request is %.0f s old (max %.0f s)' % (age, max_age_s)
    if req_up < min_uptime_s:
        return False, 'request made %.1f s after boot (< %.0f s): ignored' % (req_up, min_uptime_s)
    return True, 'ok (source=%s, age %.1f s)' % (req.get('source'), age)


def plan_actions(cfg):
    """Ordered host actions for a valid request (pure, for tests and the log)."""
    acts = ['archive_request']
    if cfg_bool(cfg, 'MCU_POWER_CUT'):
        acts.append('arm_mcu_cut')
    acts.append('sync')
    acts.append('log_dry_run' if cfg_bool(cfg, 'DRY_RUN') else 'poweroff')
    return acts


# --------------------------------------------------------------------------- host side
def _log(msg):
    # stdout of a systemd service goes to the journal (SyslogIdentifier=mower-poweroff)
    sys.stdout.write(msg + '\n')
    sys.stdout.flush()


def wait_for_stack(path, req, grace_s, sleep=time.sleep, reader=read_request, clock=time.monotonic):
    """source=kernel: give base_keys up to grace_s to replace the request with its own (it has
    already e-stopped and stopped the mission). Returns the final request."""
    if req is None or req.get('source') != 'kernel' or grace_s <= 0:
        return req
    t_end = clock() + grace_s
    while clock() < t_end:
        sleep(0.25)
        cur = reader(path)
        if cur is not None and cur.get('source') == 'base_keys':
            return cur
    return reader(path) or req


def handle(directory=REQUEST_DIR, cfg=None, run=subprocess.call):
    cfg = load_config() if cfg is None else cfg
    path = os.path.join(directory, REQUEST_NAME)
    req = read_request(path)
    if req is None:
        return 0                               # PathExists re-trigger after our own delete
    req = wait_for_stack(path, req, cfg_float(cfg, 'KERNEL_GRACE_S'))
    ok, why = validate_request(req, read_boot_id(), read_uptime(),
                               cfg_float(cfg, 'MIN_UPTIME_S'), cfg_float(cfg, 'MAX_AGE_S'))
    try:
        os.unlink(path)                        # never act twice on one request
    except OSError:
        pass
    if not ok:
        _log('mower-poweroff: request rejected: %s (%s)' % (why, json.dumps(req, sort_keys=True)))
        return 0
    acts = plan_actions(cfg)
    _log('mower-poweroff: request accepted: %s; actions %s' % (why, acts))
    for a in acts:
        if a == 'archive_request':
            rec = dict(req)
            rec['handled_uptime_s'] = round(read_uptime(), 3)
            rec['dry_run'] = cfg_bool(cfg, 'DRY_RUN')
            try:
                with open(os.path.join(directory, LAST_NAME), 'w') as f:
                    json.dump(rec, f, sort_keys=True)
                    f.write('\n')
            except OSError as e:
                _log('mower-poweroff: archive failed: %s' % e)
        elif a == 'arm_mcu_cut':
            if cfg_bool(cfg, 'DRY_RUN'):
                _log('mower-poweroff: DRY_RUN: would arm %s' % MCU_CUT_MARKER)
                continue
            try:
                os.makedirs(os.path.dirname(MCU_CUT_MARKER), exist_ok=True)
                with open(MCU_CUT_MARKER, 'w') as f:
                    f.write('%s\n' % req.get('source'))
            except OSError as e:
                _log('mower-poweroff: cannot arm MCU power cut: %s' % e)
        elif a == 'sync':
            run(['sync'])
        elif a == 'log_dry_run':
            _log('mower-poweroff: DRY_RUN=1: NOT powering off (set DRY_RUN=0 in %s)' % CONFIG_FILE)
        elif a == 'poweroff':
            _log('mower-poweroff: systemctl poweroff')
            return run(['systemctl', 'poweroff', '--no-block'])
    return 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    directory = REQUEST_DIR
    if '--dir' in argv:
        i = argv.index('--dir')
        directory = argv[i + 1]
        del argv[i:i + 2]
    cmd = argv[0] if argv else 'handle'
    if cmd == 'handle':
        return handle(directory)
    if cmd == 'request':
        source = 'manual'
        if '--source' in argv:
            source = argv[argv.index('--source') + 1]
        if not os.path.isdir(directory):
            _log('mower-poweroff: %s missing (host helper not installed?)' % directory)
            return 1
        p = write_request(make_request(source, read_boot_id(), read_uptime()), directory)
        _log('mower-poweroff: request written to %s (source=%s)' % (p, source))
        return 0
    if cmd == 'frame':
        sys.stdout.write(''.join('\\%03o' % b for b in MCU_POWEROFF_FRAME) + '\n')
        return 0
    sys.stderr.write('usage: mower-poweroff-host [--dir D] handle | request [--source S] | frame\n')
    return 2


if __name__ == '__main__':
    sys.exit(main())
