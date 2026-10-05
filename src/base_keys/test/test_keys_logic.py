"""Tests for the ROS-free button logic (no evdev / rclpy needed)."""
import struct

import pytest

from base_keys import keys_logic as kl

P, L, C, S, M = 25, 38, 46, 31, 50
VENDOR_MAP = {P: kl.KEY_POWER_SHORT, L: kl.KEY_POWER_LONG, C: kl.KEY_WORKING_OR_PAUSE,
              S: kl.KEY_GO_DOCKING, M: kl.KEY_DOCK_AND_PAUSE}

PROC = '''I: Bus=0019 Vendor=0001 Product=0001 Version=0100
N: Name="rk805 pwrkey"
P: Phys=rk805_pwrkey/input0
S: Sysfs=/devices/platform/feb20000.spi/rk805-pwrkey.1.auto/input/input0
U: Uniq=
H: Handlers=kbd event0 cpufreq dmcfreq
B: PROP=0
B: EV=3
B: KEY=10000000000000 0

I: Bus=0000 Vendor=0000 Product=0000 Version=0000
N: Name="key input"
P: Phys=
S: Sysfs=/devices/virtual/input/input5
U: Uniq=
H: Handlers=kbd event5 dmcfreq cpufreq
B: PROP=0
B: EV=3
B: KEY=4404082080000
'''


class FakeSource:
    """Fake evdev source: a scripted list of (t, type, code, value)."""

    def __init__(self, sm):
        self.sm = sm
        self.out = []

    def key(self, t, code, value):
        self.out += [(t, b) for b in self.sm.feed(kl.EV_KEY, code, value, t)]

    def syn(self, t):
        self.out += [(t, b) for b in self.sm.feed(kl.EV_SYN, 0, 0, t)]

    def tick(self, t):
        self.out += [(t, b) for b in self.sm.tick(t)]

    @property
    def bits(self):
        return [b for _, b in self.out]


def make(long_s=0.0, debounce_s=0.0):
    return FakeSource(kl.KeyStateMachine(VENDOR_MAP, power_code=P, long_press_s=long_s,
                                         debounce_s=debounce_s))


# ---------------------------------------------------------------- constants
def test_vendor_constants_match_message():
    assert (kl.KEY_WORKING_OR_PAUSE, kl.KEY_GO_DOCKING, kl.KEY_POWER_SHORT,
            kl.KEY_POWER_LONG, kl.KEY_DOCK_AND_PAUSE) == (1, 2, 4, 8, 16)
    # button.cpp handleKey switch (0x19,0x1f,0x26,0x2e,0x32)
    assert (kl.VENDOR_KEY_POWER, kl.VENDOR_KEY_GO_DOCKING, kl.VENDOR_KEY_POWER_LONG,
            kl.VENDOR_KEY_WORK_PAUSE, kl.VENDOR_KEY_DOCK_AND_PAUSE) == (0x19, 0x1f, 0x26, 0x2e, 0x32)


def test_decode_events_handles_partial_buffer():
    ev = struct.pack(kl.INPUT_EVENT_FMT, 10, 500000, kl.EV_KEY, P, 1)
    syn = struct.pack(kl.INPUT_EVENT_FMT, 10, 500000, kl.EV_SYN, 0, 0)
    events, rest = kl.decode_events(ev + syn + ev[:7])
    assert [(e[1], e[2], e[3]) for e in events] == [(kl.EV_KEY, P, 1), (kl.EV_SYN, 0, 0)]
    assert events[0][0] == pytest.approx(10.5)
    assert rest == ev[:7]
    assert kl.INPUT_EVENT_SIZE == 24


# ---------------------------------------------------------------- discovery
def test_find_device_by_name():
    assert kl.find_device(PROC, 'key input')[0] == '/dev/input/event5'


def test_find_device_by_capabilities_when_name_missing():
    path, why = kl.find_device(PROC, 'nope', (P, C, S))
    assert path == '/dev/input/event5' and 'key input' in why


def test_find_device_none():
    assert kl.find_device(PROC, 'nope', (999,))[0] is None
    assert kl.find_device('', 'key input', (P,))[0] is None


def test_key_bitmap_parse_multiword():
    devs = kl.parse_proc_input_devices(PROC)
    pwr = devs[0]
    assert pwr.has_keys([116])           # KEY_POWER in the high word
    ki = devs[1]
    assert [c for c in range(64) if ki.has_keys([c])] == [19, 25, 31, 38, 46, 50]


# ---------------------------------------------------------------- state machine
def test_short_power_press_vendor_mode_emits_on_press():
    f = make()
    f.key(0.0, P, 1)
    assert f.bits == [kl.KEY_POWER_SHORT]
    f.key(0.0, P, 0)                    # same-tick release (as measured on device)
    assert f.bits == [kl.KEY_POWER_SHORT]
    assert f.sm.held_bits == 0


def test_device_long_press_code_is_power_long():
    f = make(long_s=3.0)
    f.key(0.0, L, 1)
    f.key(0.0, L, 0)
    assert f.bits == [kl.KEY_POWER_LONG]


def test_soft_long_press_short_emits_on_release():
    f = make(long_s=3.0)
    f.key(0.0, P, 1)
    f.tick(1.0)
    assert f.bits == []
    f.key(1.2, P, 0)
    assert f.bits == [kl.KEY_POWER_SHORT]


def test_soft_long_press_fires_while_held_once():
    f = make(long_s=3.0)
    f.key(0.0, P, 1)
    f.tick(2.9)
    f.tick(3.0)
    f.tick(3.5)
    f.key(5.0, P, 0)
    assert f.bits == [kl.KEY_POWER_LONG]
    assert f.out[0][0] == 3.0
    # next press works normally again
    f.key(6.0, P, 1)
    f.key(6.1, P, 0)
    assert f.bits == [kl.KEY_POWER_LONG, kl.KEY_POWER_SHORT]


def test_soft_long_press_release_after_threshold_without_tick():
    f = make(long_s=3.0)
    f.key(0.0, P, 1)
    f.key(4.0, P, 0)
    assert f.bits == [kl.KEY_POWER_LONG]


def test_other_keys_emit_on_press_and_held_bits():
    f = make(long_s=3.0)
    f.key(0.0, C, 1)
    f.key(0.1, S, 1)
    assert f.bits == [kl.KEY_WORKING_OR_PAUSE, kl.KEY_GO_DOCKING]
    assert f.sm.held_bits == kl.KEY_WORKING_OR_PAUSE | kl.KEY_GO_DOCKING
    f.key(0.2, C, 0)
    f.key(0.3, S, 0)
    assert f.sm.held_bits == 0
    f.tick(10.0)
    assert len(f.bits) == 2


def test_combo_key():
    f = make()
    f.key(0.0, M, 1)
    assert f.bits == [kl.KEY_DOCK_AND_PAUSE]


def test_debounce_ignores_bounce_and_its_release():
    f = make(debounce_s=0.05)
    f.key(0.000, C, 1)
    f.key(0.005, C, 0)
    f.key(0.010, C, 1)      # bounce
    f.key(0.015, C, 0)
    f.key(0.200, C, 1)      # real second press
    assert f.bits == [kl.KEY_WORKING_OR_PAUSE, kl.KEY_WORKING_OR_PAUSE]


def test_autorepeat_syn_unknown_ignored():
    f = make()
    f.key(0.0, C, 1)
    f.key(0.3, C, 2)
    f.key(0.4, C, 2)
    f.syn(0.4)
    f.key(0.5, 19, 1)       # KEY_R: exposed by the device, unknown to vendor
    assert f.bits == [kl.KEY_WORKING_OR_PAUSE]
    assert f.sm.unknown_codes == [19]


def test_duplicate_press_without_release_ignored():
    f = make()
    f.key(0.0, S, 1)
    f.key(1.0, S, 1)
    assert f.bits == [kl.KEY_GO_DOCKING]


def test_reset_forgets_held_power():
    f = make(long_s=1.0)
    f.key(0.0, P, 1)
    f.sm.reset()
    f.tick(5.0)
    f.key(5.0, P, 0)
    assert f.bits == []


# ---------------------------------------------------------------- mapping
def kinds(acts):
    return [(a.kind, a.arg) for a in acts]


CFG = kl.ActionConfig()


def test_power_short_clears_estop_and_resets_emergency():
    assert kinds(kl.map_key(kl.KEY_POWER_SHORT, None, CFG)) == [
        ('clear_estop', 0), ('hlc', kl.CMD_RESET_EMERGENCY)]
    cfg = kl.ActionConfig(power_short_reset_emergency=False)
    assert kinds(kl.map_key(kl.KEY_POWER_SHORT, None, cfg)) == [('clear_estop', 0)]
    cfg = kl.ActionConfig(on_power_short=False)
    assert kinds(kl.map_key(kl.KEY_POWER_SHORT, None, cfg)) == [('log', 0)]


@pytest.mark.parametrize('state,expected', [
    (kl.HL_AUTONOMOUS, [('hlc', kl.CMD_STOP)]),
    (kl.HL_MANUAL_MOWING, [('hlc', kl.CMD_STOP)]),
    (kl.HL_IDLE, [('hlc', kl.CMD_START)]),
    (kl.HL_RECORDING, [('log', 0)]),
    (kl.HL_NULL, [('log', 0)]),
    (None, [('log', 0)]),
])
def test_work_pause_toggle(state, expected):
    assert kinds(kl.map_key(kl.KEY_WORKING_OR_PAUSE, state, CFG)) == expected


def test_work_pause_disabled():
    cfg = kl.ActionConfig(on_work_pause=False)
    assert kinds(kl.map_key(kl.KEY_WORKING_OR_PAUSE, kl.HL_IDLE, cfg)) == [('log', 0)]


def test_go_docking_home_and_disable():
    assert kinds(kl.map_key(kl.KEY_GO_DOCKING, kl.HL_AUTONOMOUS, CFG)) == [('hlc', kl.CMD_HOME)]
    cfg = kl.ActionConfig(on_go_docking=False)
    assert kinds(kl.map_key(kl.KEY_GO_DOCKING, kl.HL_IDLE, cfg)) == [('log', 0)]


def test_dock_and_pause_default_noop_optional_home():
    assert kinds(kl.map_key(kl.KEY_DOCK_AND_PAUSE, kl.HL_IDLE, CFG)) == [('log', 0)]
    cfg = kl.ActionConfig(on_dock_and_pause=True)
    assert kinds(kl.map_key(kl.KEY_DOCK_AND_PAUSE, kl.HL_IDLE, cfg)) == [('hlc', kl.CMD_HOME)]


@pytest.mark.parametrize('action,expected', [
    ('log_only', [('log', 0)]),
    ('poweroff_service', [('hlc', kl.CMD_STOP), ('poweroff_service', 0)]),
    ('shutdown', [('hlc', kl.CMD_STOP), ('shutdown_hook', 0)]),
    ('bogus', [('log', 0)]),
])
def test_power_long_actions(action, expected):
    cfg = kl.ActionConfig(power_long_action=action)
    assert kinds(kl.map_key(kl.KEY_POWER_LONG, kl.HL_AUTONOMOUS, cfg)) == expected


def test_default_power_long_is_log_only():
    assert kl.ActionConfig().power_long_action == 'log_only'
