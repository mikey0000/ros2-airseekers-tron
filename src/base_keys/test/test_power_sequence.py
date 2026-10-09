"""Tests for the ROS-free power-off sequencer and its key-path contract (no evdev / rclpy)."""
import pytest

from base_keys import keys_logic as kl
from base_keys.power_sequence import IDLE, REQUESTED, SETTLING, PowerOffSequencer

kinds = lambda steps: [s.kind for s in steps]  # noqa: E731


def seq(**kw):
    return PowerOffSequencer(settle_s=3.0, min_uptime_s=30.0, **kw)


# ---------------------------------------------------------------- guards
def test_min_uptime_guard_blocks():
    s = seq()
    steps = s.on_power_long(100.0, 29.9, True)
    assert kinds(steps) == ['log']
    assert s.state == IDLE and not s.busy
    assert s.tick(200.0) == []


def test_min_uptime_boundary_starts():
    s = seq()
    steps = s.on_power_long(100.0, 30.0, True)
    assert kinds(steps)[0] == 'estop'
    assert s.state == SETTLING


# ---------------------------------------------------------------- happy path
def test_helper_installed_step_order_and_state():
    s = seq()
    steps = s.on_power_long(100.0, 500.0, True)
    assert kinds(steps) == ['estop', 'hlc_stop', 'cutter_off', 'light_poweroff']
    assert s.state == SETTLING
    assert s.busy is True


def test_tick_before_settle_is_empty():
    s = seq()
    s.on_power_long(100.0, 500.0, True)
    assert s.tick(100.0) == []
    assert s.tick(102.99) == []
    assert s.state == SETTLING


def test_tick_at_settle_emits_once():
    s = seq()
    s.on_power_long(100.0, 500.0, True)
    steps = s.tick(103.0)
    assert kinds(steps) == ['light_poweroff', 'write_request']
    assert s.state == REQUESTED
    assert not s.busy
    assert s.tick(104.0) == []
    assert s.tick(1000.0) == []


# ---------------------------------------------------------------- re-entrancy
def test_second_press_while_settling_is_ignored():
    s = seq()
    s.on_power_long(100.0, 500.0, True)
    steps = s.on_power_long(101.0, 501.0, True)
    assert kinds(steps) == ['log']
    assert s.state == SETTLING
    # original timer not restarted
    assert kinds(s.tick(103.0)) == ['light_poweroff', 'write_request']


def test_second_press_after_requested_is_ignored():
    s = seq()
    s.on_power_long(100.0, 500.0, True)
    s.tick(103.0)
    steps = s.on_power_long(110.0, 510.0, True)
    assert kinds(steps) == ['log']
    assert 'estop' not in kinds(steps)
    assert s.state == REQUESTED


# ---------------------------------------------------------------- helper missing
def test_helper_not_installed_safety_only_then_retry():
    s = seq()
    steps = s.on_power_long(100.0, 500.0, False)
    assert kinds(steps) == ['estop', 'hlc_stop', 'cutter_off', 'error']
    assert 'light_poweroff' not in kinds(steps)
    assert s.state == IDLE and not s.busy
    for t in (101.0, 103.0, 500.0):
        assert 'write_request' not in kinds(s.tick(t))
    # a later press can run again (and succeed once the helper exists)
    again = s.on_power_long(600.0, 1000.0, True)
    assert kinds(again)[0] == 'estop'
    assert s.state == SETTLING


def test_request_failed_returns_to_idle_and_allows_retry():
    s = seq()
    s.on_power_long(100.0, 500.0, True)
    s.tick(103.0)
    assert s.state == REQUESTED
    s.request_failed()
    assert s.state == IDLE
    assert s.tick(200.0) == []
    steps = s.on_power_long(300.0, 700.0, True)
    assert kinds(steps) == ['estop', 'hlc_stop', 'cutter_off', 'light_poweroff']
    assert kinds(s.tick(303.0)) == ['light_poweroff', 'write_request']


# ---------------------------------------------------------------- keys_logic integration
@pytest.mark.parametrize('hl', [None, kl.HL_NULL, kl.HL_IDLE, kl.HL_AUTONOMOUS,
                                kl.HL_RECORDING, kl.HL_MANUAL_MOWING])
def test_map_key_sequence(hl):
    acts = kl.map_key(kl.KEY_POWER_LONG, hl, kl.ActionConfig(power_long_action='sequence'))
    assert [a.kind for a in acts] == ['power_sequence']


def test_sequence_in_power_long_actions():
    assert 'sequence' in kl.POWER_LONG_ACTIONS


def test_log_only_still_logs_only():
    acts = kl.map_key(kl.KEY_POWER_LONG, kl.HL_AUTONOMOUS, kl.ActionConfig(power_long_action='log_only'))
    assert [a.kind for a in acts] == ['log']
    assert kl.ActionConfig().power_long_action == 'log_only'


# ---------------------------------------------------------------- kernel module timing model
# Documents what the vendor init_test.ko emits on the "key input" device (docs/buttons.md):
#   short press : KEY_P(25) press + release in the same tick
#   long hold   : KEY_L(38) press + release at 3.0 s while still held, then KEY_R(19)
#                 press + release when the key is released (and sys_close is run)
P, L, R = kl.VENDOR_KEY_POWER, kl.VENDOR_KEY_POWER_LONG, kl.VENDOR_KEY_POWER_LONG_RELEASE
CODE_MAP = {P: kl.KEY_POWER_SHORT, L: kl.KEY_POWER_LONG, 46: kl.KEY_WORKING_OR_PAUSE,
            31: kl.KEY_GO_DOCKING, 50: kl.KEY_DOCK_AND_PAUSE}


def machine():
    return kl.KeyStateMachine(CODE_MAP, power_code=P, long_press_s=3.0, debounce_s=0.05)


def feed_all(sm, events):
    """events: (t, code, value); a tick follows each event time like the node's poll loop."""
    out = []
    for t, code, value in events:
        out += sm.feed(kl.EV_KEY, code, value, t)
        out += sm.tick(t)
    return out


def test_codes():
    assert (P, L, R) == (25, 38, 19)


def test_kernel_short_press_yields_single_short():
    sm = machine()
    out = feed_all(sm, [(10.0, P, 1), (10.0, P, 0)])
    assert out == [kl.KEY_POWER_SHORT]


def test_kernel_long_hold_yields_one_long_no_short():
    sm = machine()
    out = feed_all(sm, [(20.0, L, 1), (20.0, L, 0),       # at 3.0 s while held
                        (23.4, R, 1), (23.4, R, 0)])      # on release
    assert out == [kl.KEY_POWER_LONG]
    assert kl.KEY_POWER_SHORT not in out
    assert sm.unknown_codes == [R]


def test_kernel_release_code_is_unknown_and_silent():
    sm = machine()
    assert feed_all(sm, [(5.0, R, 1), (5.0, R, 0)]) == []
    assert sm.unknown_codes == [19]


def test_kernel_autorepeat_ignored():
    sm = machine()
    out = feed_all(sm, [(10.0, P, 1), (10.5, P, 2), (11.0, P, 2), (11.1, P, 0)])
    assert out == [kl.KEY_POWER_SHORT]
    out = feed_all(machine(), [(20.0, L, 1), (20.0, L, 2), (20.0, L, 0)])
    assert out == [kl.KEY_POWER_LONG]


def test_kernel_long_bounce_within_debounce_yields_one_long():
    sm = machine()
    out = feed_all(sm, [(20.0, L, 1), (20.0, L, 0),
                        (20.02, L, 1), (20.02, L, 0)])     # bounce 20 ms later
    assert out == [kl.KEY_POWER_LONG]
