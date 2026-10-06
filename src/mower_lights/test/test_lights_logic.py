"""ROS-free tests for the status-LED logic and the fake hardware backend."""
import time

import pytest

from mower_lights import led_hw
from mower_lights import lights_logic as ll


def run(pattern, waits=None):
    """Drain a pattern synchronously, collecting the wait steps."""
    out = [] if waits is None else waits
    if pattern is None:
        return out
    for step in pattern:
        out.append(step)
    return out


def px14():
    return ll.Pixels(front_led_num=14, top_led_num=27)


# ------------------------------------------------------------------ enum / tables
def test_enum_values_match_vendor_msg():
    assert (ll.IDLE, ll.TASK_START, ll.POWER_OFF, ll.WARN_SENSOR_TRIGGED) == (1, 2, 7, 19)
    assert (ll.CHARGING, ll.LOW_BATTERY, ll.REMOTE_CONTROL, ll.PAIR_FAILED) == (14, 17, 24, 25)
    assert ll.GENERAL_NORMAL_WHITE == 27 and ll.UNKNOWN == -1
    assert ll.mode_name(19) == 'WarnSensorTrigged'


def test_priority_map_from_vendor_ctor():
    assert ll.mode_priority(ll.POWER_OFF) == 0
    assert ll.mode_priority(ll.WARN_SENSOR_TRIGGED) == 2
    assert ll.mode_priority(ll.IDLE) == 4
    assert ll.mode_priority(ll.REMOTE_CONTROL) == ll.DEFAULT_PRIORITY == 4


def test_color_bytes_wire_order_grb_and_brightness():
    assert ll.color_bytes(ll.RED, 120) == (0, 120, 0)        # byte1 = R
    assert ll.color_bytes(ll.GREEN, 120) == (120, 0, 0)      # byte0 = G
    assert ll.color_bytes(ll.BLUE, 120) == (0, 0, 120)
    assert ll.color_bytes(ll.YELLOW, 120) == (120, 120, 0)
    assert ll.color_bytes(ll.WHITE, 120, brightness=50) == (60, 60, 60)
    assert ll.color_bytes(ll.OFF, 120) == (0, 0, 0)


def test_area_geometry():
    p = px14()
    assert (p.start_pin(ll.AREA_TAIL), p.fill_count(ll.AREA_TAIL)) == (0, 28)
    assert (p.start_pin(ll.AREA_TOP), p.fill_count(ll.AREA_TOP)) == (28, 27)
    assert p.pin_nums(ll.AREA_ALL) == 55
    assert ll.front_led_num_from_sn('1001027304000822') == 14
    assert ll.front_led_num_from_sn('1001027B02000822') == 7
    assert ll.front_led_num_from_sn(None) == 14


# ------------------------------------------------------------------ SPI encoding
def test_encode_frame_matches_libws2812():
    buf = bytearray(180)
    buf[0:3] = bytes([0x80, 0x01, 0x00])
    f = ll.encode_frame(bytes(buf))
    assert len(f) == ll.SPI_FRAME_LEN == 1640
    assert f[:100] == bytes(100) and f[-100:] == bytes(100)
    assert f[100] == 0xFC and f[101:108] == bytes([0xC0] * 7)        # 0x80 MSB first
    assert f[108:115] == bytes([0xC0] * 7) and f[115] == 0xFC         # 0x01
    assert ll.decode_frame(f) == bytes(buf)


# ------------------------------------------------------------------ patterns
def test_idle_breathe_white_both_lanes():
    p = px14()
    pat, last = ll.top_program(p, ll.IDLE, -1)
    waits = run(pat)
    assert last == ll.IDLE
    assert len(waits) == 49 + 48 and all(w.ms == 30 for w in waits)
    # ends on table[21] = 5
    assert set(p.area_pixels(ll.AREA_TOP)) == {(5, 5, 5)}
    # breathe repeats (not 'once'): a second call yields a pattern again
    assert ll.top_program(p, ll.IDLE, ll.IDLE)[0] is not None


def test_warn_sensor_is_constant_red_and_static():
    p = px14()
    clock = lambda: 0.0  # noqa: E731
    run(ll.top_program(p, ll.WARN_SENSOR_TRIGGED, ll.IDLE)[0])
    run(ll.tail_program(p, ll.WARN_SENSOR_TRIGGED, -1, ll.IDLE, clock)[0])
    assert set(p.area_pixels(ll.AREA_TOP)) == {(0, 120, 0)}
    assert set(p.area_pixels(ll.AREA_TAIL)) == {(0, 120, 0)}
    assert ll.summarize_area(p, ll.AREA_TOP) == 'red(120) x27'
    # once rendered, it is not re-run
    assert ll.top_program(p, ll.WARN_SENSOR_TRIGGED, ll.WARN_SENSOR_TRIGGED)[0] is None


def test_task_start_blinks_twice_then_white():
    p = px14()
    waits = run(ll.top_program(p, ll.TASK_START, -1)[0])
    assert [w.ms for w in waits] == [500, 500, 500, 500]
    assert set(p.area_pixels(ll.AREA_TOP)) == {(120, 120, 120)}


def test_charging_green_sweep_dim_then_off():
    p = px14()
    seen = []
    pat = ll.top_program(p, ll.CHARGING, -1)[0]
    for step in pat:
        seen.append(p.get(28 + 13))       # middle LED of the top ring
    assert (60, 0, 0) in seen             # green at DIM=60
    assert set(p.area_pixels(ll.AREA_TOP)) == {(0, 0, 0)}


def test_power_on_hands_over_to_idle():
    p = px14()
    steps = run(ll.top_program(p, ll.POWER_ON, -1)[0])
    assert steps[0] == ll.Wait(500, False)
    assert steps[-1] == ll.SetMainIfStill(ll.POWER_ON, ll.IDLE)
    assert set(p.area_pixels(ll.AREA_TOP)) == {(120, 120, 120)}


def test_remote_control_and_unknown_leave_leds_unchanged():
    p = px14()
    run(ll.top_program(p, ll.WARN_SENSOR_TRIGGED, -1)[0])
    before = p.snapshot()
    assert ll.top_program(p, ll.REMOTE_CONTROL, ll.WARN_SENSOR_TRIGGED) == (None, -1)
    assert ll.top_program(p, ll.TASK_FINISHED, -1) == (None, ll.TASK_FINISHED)
    assert p.snapshot() == before


def test_open_cutter_oneshot_sweeps_red_for_4s():
    p = px14()
    t = [0.0]

    def clock():
        return t[0]

    pat, last = ll.tail_program(p, ll.IDLE, ll.OPEN_CUTTER, ll.IDLE, clock)
    assert last == -1
    passes = 0
    for step in pat:
        if isinstance(step, ll.Wait) and step.interruptible:
            t[0] += step.ms
            if p.get(13) == (0, 120, 0) and p.get(27) == (0, 120, 0):   # first LEDs lit
                passes += 1
    assert t[0] >= 4000
    assert passes >= 4          # ~15 steps x 200 ms per pass over 4 s


# ------------------------------------------------------------------ arbiter
def test_arbiter_priority_and_close_cutter_gate():
    a = ll.ModeArbiter()
    assert a.main_mode == ll.POWER_ON
    a.top_snapshot()                          # top_prio := 0 (PowerOn)
    irq = a.request(ll.IDLE)                  # prio 4 > 0: no preempt, but mode replaced
    assert not irq.top and a.main_mode == ll.IDLE
    a.top_snapshot()                          # top_prio := 4
    irq = a.request(ll.WARN_SENSOR_TRIGGED)   # prio 2 <= 4 -> preempt
    assert irq.top and irq.tail
    assert a.request(ll.CLOSE_CUTTER).tail is False and a.oneshot_mode == -1
    a.cutter_running = True
    assert a.request(ll.CLOSE_CUTTER).tail and a.oneshot_mode == ll.CLOSE_CUTTER
    assert a.tail_snapshot() == (ll.WARN_SENSOR_TRIGGED, ll.CLOSE_CUTTER)
    assert a.oneshot_mode == -1 and a.tail_oneshot_active


# ------------------------------------------------------------------ state mapping
def hl(name, state=1, **kw):
    i = ll.StackInputs(hl_state=state, hl_state_name=name, hl_time=100.0, **kw)
    return ll.stack_to_mode(i, 100.0)


@pytest.mark.parametrize('name,state,kw,mode', [
    ('IDLE', 1, {}, ll.IDLE),
    ('IDLE', 1, {'battery_percent': 12.0}, ll.LOW_BATTERY),
    ('MOWING', 2, {}, ll.TASK_START),
    ('BOUNDARY_PAUSED', 2, {}, ll.TASK_PAUSE),
    ('RETURNING_HOME', 2, {}, ll.DOCKING),
    ('LOW_BATTERY_DOCKING', 2, {}, ll.LOW_BATTERY),
    ('CHARGING', 1, {'hl_is_charging': True, 'battery_percent': 60.0}, ll.CHARGING),
    ('IDLE_DOCKED', 1, {'base_charging': True, 'battery_percent': 100.0}, ll.CHARGE_FINISHED),
    ('MANUAL_MOWING', 4, {}, ll.REMOTE_CONTROL),
    ('RECORDING', 3, {}, ll.REMOTE_CONTROL),
    ('EMERGENCY', 0, {}, ll.WARN_SENSOR_TRIGGED),
    ('IDLE', 1, {'lift': True}, ll.WARN_SENSOR_TRIGGED),
    ('MOWING', 2, {'estop_active': True}, ll.WARN_SENSOR_TRIGGED),
    ('UNDOCK_FAILED', 1, {}, ll.ROBOT_UNUSUAL),
    ('IDLE', 1, {'shutting_down': True}, ll.POWER_OFF),
])
def test_stack_to_mode(name, state, kw, mode):
    assert hl(name, state, **kw) == mode


def test_stale_status_falls_back_to_idle_and_warn_hold():
    i = ll.StackInputs(hl_state=2, hl_state_name='MOWING', hl_time=0.0)
    assert ll.stack_to_mode(i, 100.0) == ll.IDLE
    am = ll.AutoMode()
    am.inputs = ll.StackInputs(hl_state=1, hl_state_name='IDLE', hl_time=10.0, bumper=True)
    assert am.evaluate(10.0) == ll.WARN_SENSOR_TRIGGED
    am.inputs.bumper = False
    am.inputs.hl_time = 11.0
    assert am.evaluate(11.0) is None          # held for warn_hold_s, no re-emit
    am.inputs.hl_time = 12.5
    assert am.evaluate(12.5) == ll.IDLE       # hold expired -> edge to Idle


# ------------------------------------------------------------------ engine + fake hw
def test_engine_with_fake_backend_reaches_idle_then_red():
    be = led_hw.FakeBackend()
    eng = led_hw.LightEngine(be, px14(), initial_mode=ll.POWER_ON, frame_rate_hz=100.0,
                             lane_rate_hz=50.0)
    eng.start()
    try:
        deadline = time.monotonic() + 5.0
        while eng.mode != ll.IDLE and time.monotonic() < deadline:
            time.sleep(0.02)
        assert eng.mode == ll.IDLE                      # PowerOn handed over
        eng.request(ll.WARN_SENSOR_TRIGGED)
        deadline = time.monotonic() + 2.0
        red = (0, 120, 0)
        while time.monotonic() < deadline:
            buf = ll.decode_frame(be.last_frame)
            if all(tuple(buf[3 * k:3 * k + 3]) == red for k in range(55)):
                break
            time.sleep(0.02)
        buf = ll.decode_frame(be.last_frame)
        assert all(tuple(buf[3 * k:3 * k + 3]) == red for k in range(55))
        assert buf[3 * 55:] == bytes(15)                 # LEDs 55..59 never lit
    finally:
        eng.stop()
    assert be.closed and len(be.frames) > 10


def test_spi_transfer_struct_layout():
    s = led_hw.spi_ioc_transfer(0x1000, 1640, 8000000, 8)
    assert len(s) == 32
    assert led_hw.SPI_IOC_MESSAGE_1 == 0x40206B00   # same request code as libws_2812 transfer()


def test_interface_msg_constants_match_logic():
    import os
    import re
    msg = os.path.join(os.path.dirname(__file__), '..', '..', 'mower_lights_interfaces',
                       'msg', 'LightMode.msg')
    if not os.path.exists(msg):
        pytest.skip('interface package not next to mower_lights')
    consts = dict((m.group(1), int(m.group(2))) for m in
                  re.finditer(r'^int8 (\w+) = (-?\d+)', open(msg).read(), re.M))
    assert len(consts) == 29
    for name, value in consts.items():
        assert getattr(ll, name) == value, name


# ------------------------------------------------------------------ CPU-lean engine
def _encode_reference(buf, leds=ll.BUFFER_LEDS):
    """The original per-bit encoder (libws_2812 rgb_to_send_buffer_all)."""
    out = bytearray(ll.SPI_RESET_BYTES)
    for i in range(leds):
        for byte in buf[3 * i:3 * i + 3]:
            for bit in range(7, -1, -1):
                out.append(ll.SPI_BIT_ONE if (byte >> bit) & 1 else ll.SPI_BIT_ZERO)
    out.extend(bytes(ll.SPI_RESET_BYTES))
    return bytes(out)


def test_table_encoder_equals_reference():
    import random
    rnd = random.Random(7)
    for _ in range(50):
        buf = bytes(rnd.randrange(256) for _ in range(180))
        assert ll.encode_frame(buf) == _encode_reference(buf)
        assert ll.decode_frame(ll.encode_frame(buf)) == buf
    assert ll.encode_frame(bytes(range(256))[:180], 20) == _encode_reference(bytes(range(180)), 20)


def test_fill_equals_set_pin():
    a, b = px14(), px14()
    for start, count in ((0, 28), (28, 27), (50, 20), (-3, 5)):
        for i in range(count):
            a.set_pin(start + i, ll.YELLOW, 77)
        b.fill(start, count, ll.YELLOW, 77)
        assert a.snapshot() == b.snapshot()


def _wait_mode(eng, mode, timeout=5.0):
    deadline = time.monotonic() + timeout
    while eng.mode != mode and time.monotonic() < deadline:
        time.sleep(0.02)
    return eng.mode == mode


def test_static_pattern_sends_only_keepalives():
    be = led_hw.FakeBackend()
    eng = led_hw.LightEngine(be, px14(), initial_mode=ll.WARN_SENSOR_TRIGGED,
                             frame_rate_hz=25.0, lane_rate_hz=20.0, keepalive_s=0.5)
    eng.start()
    try:
        time.sleep(0.4)                            # render + send the red frame
        n0 = len(be.frames)
        time.sleep(1.2)
        n1 = len(be.frames)
        red = ll.decode_frame(be.last_frame)
        assert all(tuple(red[3 * k:3 * k + 3]) == (0, 120, 0) for k in range(55))
        assert 1 <= n1 - n0 <= 3                   # keep-alives only (0.5 s), not 25 Hz
        # a static lane is woken by a request (no 20 Hz polling needed)
        eng.request(ll.TASK_PAUSE)                 # yellow, priority 3 > 2: no preempt
        deadline = time.monotonic() + 1.0
        yellow = (120, 120, 0)
        while time.monotonic() < deadline:
            buf = ll.decode_frame(be.last_frame)
            if tuple(buf[0:3]) == yellow and tuple(buf[3 * 30:3 * 30 + 3]) == yellow:
                break
            time.sleep(0.01)
        buf = ll.decode_frame(be.last_frame)
        assert tuple(buf[0:3]) == yellow and tuple(buf[3 * 30:3 * 30 + 3]) == yellow
    finally:
        eng.stop()


def test_idle_breathing_frames_follow_table():
    be = led_hw.FakeBackend()
    eng = led_hw.LightEngine(be, px14(), initial_mode=ll.IDLE, frame_rate_hz=25.0,
                             lane_rate_hz=20.0, keepalive_s=1.0)
    eng.start()
    try:
        time.sleep(1.5)
    finally:
        eng.stop()
    levels = set()
    for f in be.frames[1:]:
        buf = ll.decode_frame(f)
        top, tail = buf[3 * 28:3 * 55], buf[0:3 * 28]
        for area in (top, tail):
            px = {tuple(area[i:i + 3]) for i in range(0, len(area), 3)}
            assert len(px) == 1                   # whole area one white level
            v = px.pop()
            assert v[0] == v[1] == v[2]
            levels.add(v[0])
    assert levels <= set(ll.RESPIRATION_LAMP_TABLE)
    assert len(levels) >= 10                      # it is breathing
    assert 20 <= len(be.frames) <= 1.5 * 25 + 3   # capped at frame_rate_hz


def test_power_on_then_interrupt_preempts_running_pattern():
    be = led_hw.FakeBackend()
    eng = led_hw.LightEngine(be, px14(), initial_mode=ll.POWER_ON, frame_rate_hz=50.0,
                             lane_rate_hz=20.0)
    eng.start()
    try:
        assert _wait_mode(eng, ll.IDLE)
        time.sleep(0.3)                            # breathing (interruptible waits)
        t0 = time.monotonic()
        eng.request(ll.ROBOT_UNUSUAL)              # red flicker, prio 2: preempts Idle
        red = (0, 120, 0)
        while time.monotonic() - t0 < 1.0:
            buf = ll.decode_frame(be.last_frame)
            if tuple(buf[3 * 28:3 * 28 + 3]) == red and tuple(buf[0:3]) == red:
                break
            time.sleep(0.005)
        assert time.monotonic() - t0 < 0.3
    finally:
        eng.stop()
