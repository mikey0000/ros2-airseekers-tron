"""ROS-free status-LED logic for the Airseekers Tron (port of vendor mower_light_sound).

Reconstructed from the vendor ROS 1 node ``mower_light_sound_node``
(``/workspace/src/mower_task/mower_light_sound/src/light_control.cpp``, class
``mower_light_sound::LightManager``) and its ``libws_2812.so`` by Ghidra decompile.
See ``docs/lights.md`` for the evidence.

Hardware model (vendor):
  * one WS2812 chain on ``/dev/spidev3.0``; a 60-LED (180 byte) frame buffer is
    clocked out at 25 Hz (``sendLightCmdThread``);
  * per LED the buffer holds 3 bytes in wire order (b0, b1, b2) = (G, R, B);
  * the chain is split into areas (``getStartPin`` / ``getPinNums``), F = front LED
    count per strip (14, or 7 when SN[7:10] is 'B02'/'203'), T = 27 top LEDs:
        area 0: [0, F)      area 1: [F, 2F)      area 2 ("tail"): [0, 2F) (both strips)
        area 3 ("top"): [2F, 2F+T)               area 4: [0, 2F+T)
  * two animation lanes, each a 20 Hz loop: ``topLightThread`` drives area 3,
    ``tailLightThread`` drives area 2 (+ the cutter one-shots 22/23).

Everything here is pure: patterns are generators that mutate a ``Pixels`` buffer and
yield ``Wait`` / ``SetMainIfStill`` markers which the runner (led_hw.LaneRunner)
executes with interruptible timers, exactly like the vendor ``WakeAbleTimer``.
"""
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterator, List, Optional, Tuple

# ---------------------------------------------------------------------------
# LightMode enum (verbatim from vendor mower_msgs/LightMode.msg)
# ---------------------------------------------------------------------------
UNKNOWN = -1
MAPPING = 0
IDLE = 1
TASK_START = 2
TASK_FINISHED = 3
TASK_PAUSE = 4
TASK_TERMINATION = 5
POWER_ON = 6
POWER_OFF = 7
OTA_ING = 8
OTA_FINISHED = 9
PAIR_ING = 10
PAIR_FINISHED = 11
FINDING = 12
DOCKING = 13
CHARGING = 14
CHARGE_FINISHED = 15
TASK_ING = 16
LOW_BATTERY = 17
MANUAL_INTERVENTION = 18
WARN_SENSOR_TRIGGED = 19
ROBOT_UNUSUAL = 20
SIGNAL_ERROR = 21
OPEN_CUTTER = 22
CLOSE_CUTTER = 23
REMOTE_CONTROL = 24
PAIR_FAILED = 25
GENERAL_ERROR_RED = 26
GENERAL_NORMAL_WHITE = 27

MODE_NAMES = {
    -1: 'Unknown', 0: 'Mapping', 1: 'Idle', 2: 'TaskStart', 3: 'TaskFinished',
    4: 'TaskPause', 5: 'TaskTermination', 6: 'PowerOn', 7: 'PowerOff', 8: 'OTAIng',
    9: 'OTAFinished', 10: 'PairIng', 11: 'PairFinished', 12: 'Finding', 13: 'Docking',
    14: 'Charging', 15: 'ChargeFinished', 16: 'TaskIng', 17: 'LowBattery',
    18: 'ManualIntervention', 19: 'WarnSensorTrigged', 20: 'RobotUnusual',
    21: 'SignalError', 22: 'OpenCutter', 23: 'CloseCutter', 24: 'RemoteControl',
    25: 'PairFailed', 26: 'GeneralErrorRed', 27: 'GeneralNormalWhite',
}


def mode_name(mode: int) -> str:
    return MODE_NAMES.get(int(mode), 'Mode%d' % int(mode))


# Vendor LightManager ctor: std::map<int,int> mode -> priority (lower = stronger).
# Modes not in the map get priority 4 (lightModeService default).
MODE_PRIORITY = {
    6: 0, 7: 0, 22: 0, 23: 0,
    8: 1, 9: 1, 2: 1, 10: 1, 11: 1, 25: 1, 17: 1,
    19: 2, 20: 2,
    3: 3, 4: 3, 5: 3, 12: 3, 13: 3, 14: 3, 15: 3, 16: 3, 18: 3, 21: 3, 0: 3, 26: 3, 27: 3,
    1: 4,
}
DEFAULT_PRIORITY = 4


def mode_priority(mode: int) -> int:
    return MODE_PRIORITY.get(int(mode), DEFAULT_PRIORITY)


# ---------------------------------------------------------------------------
# Colours (setLedPin colour codes) and brightness
# ---------------------------------------------------------------------------
BLUE = 0     # (b0,b1,b2) = (0, 0, v)
RED = 1      # (0, v, 0)
GREEN = 2    # (v, 0, 0)
YELLOW = 3   # (v, v, 0)   (red + green)
WHITE = 4    # (v, v, v)
OFF = 5      # (0, 0, 0)
COLOR_NAMES = {BLUE: 'blue', RED: 'red', GREEN: 'green', YELLOW: 'yellow',
               WHITE: 'white', OFF: 'off'}

FULL = 0x78      # 120: vendor "full" level for every constant/flicker pattern
DIM = 0x3C       # 60: LightManager+0x54, used by the OTA / charging sweeps

# RESPIRATION_LAMP_TABLE (69 bytes, .data @ 0x210010 of mower_light_sound_node)
RESPIRATION_LAMP_TABLE = (
    1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7,
    7, 8, 9, 9, 10, 11, 11, 12, 13, 14, 15, 16, 17, 18, 20, 21, 23, 24, 26, 28, 30, 32,
    34, 37, 39, 42, 45, 48, 52, 56, 60, 64, 68, 73, 79, 84, 90, 97, 104, 111, 119,
)

BUFFER_LEDS = 60          # ws2812_send always clocks 60 LEDs (0x3c)
TOP_LED_NUM = 27          # LightManager+0x2a0
FRONT_LED_NUM = 14        # LightManager+0x29c (default when SN unknown)
FRONT_LED_NUM_SHORT = 7   # SN[7:10] in ('B02', '203')

AREA_A, AREA_B, AREA_TAIL, AREA_TOP, AREA_ALL = 0, 1, 2, 3, 4


def front_led_num_from_sn(sn: Optional[str], default: int = FRONT_LED_NUM) -> int:
    """LightManager ctor: readSn('/meta/sn'); len >= 16 required; SN.substr(7,3)."""
    if not sn or len(sn) < 16:
        return default
    return FRONT_LED_NUM_SHORT if sn[7:10] in ('B02', '203') else FRONT_LED_NUM


def color_bytes(color: int, value: int, brightness: int = 100) -> Tuple[int, int, int]:
    """setLedPin(): v = (int)(value * (brightness / 100.0)); bytes in wire order."""
    v = int(float(value) * (float(brightness) / 100.0)) & 0xFF
    if color == WHITE:
        return (v, v, v)
    if color == BLUE:
        return (0, 0, v)
    if color == RED:
        return (0, v, 0)
    if color == GREEN:
        return (v, 0, 0)
    if color == YELLOW:
        return (v, v, 0)
    if color == OFF:
        return (0, 0, 0)
    return (v, 0, 0)   # vendor default branch


class Pixels:
    """The 180-byte LightManager frame buffer (+0x5c) plus the area geometry."""

    def __init__(self, front_led_num: int = FRONT_LED_NUM, top_led_num: int = TOP_LED_NUM,
                 brightness: int = 100, buffer_leds: int = BUFFER_LEDS):
        self.front = int(front_led_num)
        self.top = int(top_led_num)
        self.brightness = int(brightness)
        self.n = int(buffer_leds)
        self.buf = bytearray(3 * self.n)

    # getStartPin / getPinNums (verbatim)
    def start_pin(self, area: int) -> int:
        if area in (0, 2, 4):
            return 0
        if area == 1:
            return self.front
        if area == 3:
            return self.front * 2
        return 0

    def pin_nums(self, area: int) -> int:
        if area in (0, 1, 2):
            return self.front
        if area == 3:
            return self.top
        if area == 4:
            return self.front * 2 + self.top
        return 0

    def fill_count(self, area: int) -> int:
        """constLight / breathe / flicker double the count for area 2 (both strips)."""
        n = self.pin_nums(area)
        return n * 2 if area == AREA_TAIL else n

    def set_pin(self, idx: int, color: int, value: int) -> None:
        if 0 <= idx < self.n:
            self.buf[3 * idx:3 * idx + 3] = bytes(color_bytes(color, value, self.brightness))

    def fill(self, start: int, count: int, color: int, value: int) -> None:
        """``set_pin(start + i, color, value) for i in range(count)`` in one slice write."""
        a, b = max(0, start), min(self.n, start + count)
        if b > a:
            self.buf[3 * a:3 * b] = bytes(color_bytes(color, value, self.brightness)) * (b - a)

    def get(self, idx: int) -> Tuple[int, int, int]:
        return tuple(self.buf[3 * idx:3 * idx + 3])

    def area_pixels(self, area: int) -> List[Tuple[int, int, int]]:
        s = self.start_pin(area)
        return [self.get(i) for i in range(s, s + self.fill_count(area))]

    def snapshot(self) -> bytes:
        return bytes(self.buf)


def describe_pixel(px: Tuple[int, int, int]) -> str:
    """Inverse of color_bytes for logs/readback (wire order G,R,B)."""
    g, r, b = px
    if g == r == b == 0:
        return 'off'
    if g == r == b:
        return 'white(%d)' % g
    if r and g and not b and r == g:
        return 'yellow(%d)' % r
    if r and not g and not b:
        return 'red(%d)' % r
    if g and not r and not b:
        return 'green(%d)' % g
    if b and not r and not g:
        return 'blue(%d)' % b
    return 'grb(%d,%d,%d)' % px


def summarize_area(pixels: Pixels, area: int) -> str:
    """'red(120) x27' style run-length summary of an area."""
    out = []
    for d in (describe_pixel(p) for p in pixels.area_pixels(area)):
        if out and out[-1][0] == d:
            out[-1][1] += 1
        else:
            out.append([d, 1])
    return ', '.join('%s x%d' % (d, n) for d, n in out)


# ---------------------------------------------------------------------------
# WS2812-over-SPI encoding (libws_2812.so: rgb_to_send_buffer_all / ws2812_send)
# ---------------------------------------------------------------------------
SPI_SPEED_HZ = 8000000    # .data 0x00127a00 -> SPI_IOC_WR_MAX_SPEED_HZ
SPI_MODE = 0
SPI_BITS = 8
SPI_BIT_ONE = 0xFC        # 6/8 high @ 8 MHz = 750 ns  -> WS2812 "1"
SPI_BIT_ZERO = 0xC0       # 2/8 high @ 8 MHz = 250 ns  -> WS2812 "0"
SPI_RESET_BYTES = 100     # leading and trailing zero bytes (100 us low)
SPI_FRAME_LEN = 0x668     # 1640 = 100 + 60*24 + 100


# One 8-byte SPI pattern per colour byte value (MSB first): a frame is 180 table lookups
# instead of 1440 per-bit Python steps.
_ENCODE_TABLE = tuple(bytes(SPI_BIT_ONE if (v >> bit) & 1 else SPI_BIT_ZERO
                            for bit in range(7, -1, -1)) for v in range(256))
_DECODE_TABLE = {enc: v for v, enc in enumerate(_ENCODE_TABLE)}
_RESET = bytes(SPI_RESET_BYTES)


def encode_frame(buf: bytes, leds: int = BUFFER_LEDS) -> bytes:
    """Each colour bit -> one SPI byte, MSB first, bytes in buffer order (G,R,B)."""
    return b''.join((_RESET, b''.join(map(_ENCODE_TABLE.__getitem__, buf[:3 * leds])),
                     _RESET))


def decode_frame(frame: bytes, leds: int = BUFFER_LEDS) -> bytes:
    """Inverse of encode_frame (tests / readback of what was clocked out)."""
    data = bytes(frame[SPI_RESET_BYTES:SPI_RESET_BYTES + 24 * leds])
    out = bytearray()
    for i in range(0, len(data), 8):
        chunk = data[i:i + 8]
        v = _DECODE_TABLE.get(chunk)
        if v is None:
            v = 0
            for b in chunk:
                v = (v << 1) | (1 if b == SPI_BIT_ONE else 0)
        out.append(v)
    return bytes(out)


# ---------------------------------------------------------------------------
# Pattern primitives (generators). ``yield Wait(ms)`` == WakeAbleTimer::wait_for
# (aborts the pattern when the lane is interrupted); Wait(ms, False) == sleep_for.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Wait:
    ms: int
    interruptible: bool = True


@dataclass(frozen=True)
class SetMainIfStill:
    """PowerOn epilogue: main mode := ``to`` (vendor does it unconditionally)."""
    expect: int
    to: int


Step = object
Pattern = Iterator[Step]


def const_light(px: Pixels, area: int, color: int, value: int) -> None:
    px.fill(px.start_pin(area), px.fill_count(area), color, value)


def p_const(px, area, color, value=FULL) -> Pattern:
    const_light(px, area, color, value)
    return
    yield  # pragma: no cover  (makes this a generator)


def p_breathe(px, area, color, step_ms=30) -> Pattern:
    s, n = px.start_pin(area), px.fill_count(area)
    for k in range(0x14, 0x45):              # 20..68 up
        yield Wait(step_ms)
        px.fill(s, n, color, RESPIRATION_LAMP_TABLE[k])
    for k in range(0x44, 0x14, -1):          # 68..21 down
        yield Wait(step_ms)
        px.fill(s, n, color, RESPIRATION_LAMP_TABLE[k])


def p_flicker(px, area, color, step_ms, count, value=FULL) -> Pattern:
    s, n = px.start_pin(area), px.fill_count(area)
    for _ in range(count):
        px.fill(s, n, color, value)
        yield Wait(step_ms)
        px.fill(s, n, OFF, value)
        yield Wait(step_ms)


def p_both_end_to_middle(px, area, color, step_ms, value) -> Pattern:
    s, n = px.start_pin(area), px.pin_nums(area)
    for i in range(n // 2):
        yield Wait(step_ms)
        px.set_pin(s + i, color, value)
        px.set_pin(s + n - 1 - i, color, value)
        if i == n // 2 - 1:
            px.set_pin(s + n // 2, color, value)
    yield Wait(step_ms)


def p_middle_to_both_end(px, area, color, step_ms, value) -> Pattern:
    s, n = px.start_pin(area), px.pin_nums(area)
    for i in range(n // 2):
        yield Wait(step_ms)
        px.set_pin(s + n // 2 - 1 - i, color, value)
        px.set_pin(s + n // 2 + 1 + i, color, value)
        if i == 0:
            px.set_pin(s + n // 2, color, value)
    yield Wait(step_ms)


def _sweep(px, area, color, step_ms, value, duration_ms, down, clock) -> Pattern:
    """topToBottom / bottomToTop: both strips in parallel, then all off; repeat
    until ``duration_ms`` elapsed (0 => one pass)."""
    s, n = px.start_pin(area), px.pin_nums(area)
    t0 = clock()
    while True:
        for i in range(n):
            yield Wait(step_ms)
            if down:
                px.set_pin(s + n - 1 - i, color, value)
                px.set_pin(s + 2 * n - 1 - i, color, value)
            else:
                px.set_pin(s + i, color, value)
                px.set_pin(s + n + i, color, value)
        const_light(px, area, OFF, FULL)
        yield Wait(step_ms)
        if clock() - t0 >= duration_ms:
            return


def p_top_to_bottom(px, area, color, step_ms, value, duration_ms, clock) -> Pattern:
    return _sweep(px, area, color, step_ms, value, duration_ms, True, clock)


def p_bottom_to_top(px, area, color, step_ms, value, duration_ms, clock) -> Pattern:
    return _sweep(px, area, color, step_ms, value, duration_ms, False, clock)


def _chain(*parts) -> Pattern:
    for p in parts:
        if isinstance(p, (Wait, SetMainIfStill)):
            yield p
        elif callable(p):
            p()
        else:
            yield from p


# ---------------------------------------------------------------------------
# Lane programs: topLightControl / tailLightControl
# Each returns (pattern-or-None, new_last). ``last`` is the lane's static
# DAT_00310058 / DAT_0031005c ("last mode rendered", -1 = none).
# ---------------------------------------------------------------------------
NO_CHANGE_MODES = (3, 5, 11, 12, 18)   # handled, but no LED change

# Mode table, shared by both lanes except where noted.
#   once=True : skipped while the lane's last mode == mode (static pattern)
#   once=False: re-run every lane cycle (animation loops)
# (kind, color, step_ms, count/value, once)
MODE_TABLE = {
    0: ('const', WHITE, 0, FULL, True),
    16: ('const', WHITE, 0, FULL, True),
    1: ('breathe', WHITE, 30, 0, False),
    2: ('flicker_then_const', WHITE, 500, 2, True),
    4: ('const', YELLOW, 0, FULL, True),
    9: ('const', WHITE, 0, FULL, True),
    10: ('flicker', BLUE, 500, 1, False),
    13: ('const', GREEN, 0, FULL, True),
    15: ('const', GREEN, 0, FULL, True),
    17: ('breathe', RED, 30, 0, False),
    19: ('const', RED, 0, FULL, True),
    20: ('flicker', RED, 500, 1, False),
    21: ('const', RED, 0, FULL, True),
    25: ('flicker', RED, 500, 2, False),
    26: ('const', RED, 0, FULL, True),
    27: ('const', WHITE, 0, FULL, True),
}


def _table_pattern(px, area, mode) -> Pattern:
    kind, color, step, arg, _ = MODE_TABLE[mode]
    if kind == 'const':
        return p_const(px, area, color, arg)
    if kind == 'breathe':
        return p_breathe(px, area, color, step)
    if kind == 'flicker':
        return p_flicker(px, area, color, step, arg, FULL)
    if kind == 'flicker_then_const':
        return _chain(p_flicker(px, area, color, step, arg, FULL),
                      p_const(px, area, color, FULL))
    raise ValueError(kind)


def top_program(px: Pixels, mode: int, last: int) -> Tuple[Optional[Pattern], int]:
    """topLightControl(): area 3 (top ring)."""
    a = AREA_TOP
    if mode == POWER_ON:
        if last == mode:
            return None, last
        return _chain(p_const(px, a, OFF, FULL), Wait(500, False),
                      p_both_end_to_middle(px, a, WHITE, 30, FULL),
                      SetMainIfStill(POWER_ON, IDLE)), mode
    if mode == POWER_OFF:
        if last == mode:
            return None, last
        return _chain(p_const(px, a, WHITE, FULL), Wait(500, False),
                      p_middle_to_both_end(px, a, OFF, 100, FULL)), mode
    if mode == OTA_ING:
        return _chain(p_const(px, a, OFF, FULL), Wait(500, False),
                      p_middle_to_both_end(px, a, BLUE, 100, DIM)), mode
    if mode == CHARGING:
        return _chain(p_middle_to_both_end(px, a, GREEN, 100, DIM),
                      p_const(px, a, OFF, FULL)), mode
    return _common_program(px, a, mode, last)


def tail_program(px: Pixels, mode: int, oneshot: int, last: int,
                 clock: Callable[[], float]) -> Tuple[Optional[Pattern], int]:
    """tailLightControl(): area 2 (both front strips) + cutter one-shots."""
    a = AREA_TAIL
    if oneshot in (OPEN_CUTTER, CLOSE_CUTTER):
        sweep = p_top_to_bottom if oneshot == OPEN_CUTTER else p_bottom_to_top
        return _chain(p_const(px, a, OFF, FULL), Wait(200, False),
                      sweep(px, a, RED, 200, FULL, 4000, clock)), -1
    if mode == POWER_ON:
        if last == mode:
            return None, last
        return p_top_to_bottom(px, a, WHITE, 150, FULL, 0, clock), mode
    if mode == POWER_OFF:
        if last == mode:
            return None, last
        return p_bottom_to_top(px, a, OFF, 150, FULL, 0, clock), mode
    if mode in (OTA_ING, CHARGING):
        color = BLUE if mode == OTA_ING else GREEN
        return _chain(p_const(px, a, OFF, FULL), Wait(200, False),
                      p_bottom_to_top(px, a, color, 150, DIM, 0, clock)), mode
    return _common_program(px, a, mode, last)


def _common_program(px, area, mode, last):
    if mode in NO_CHANGE_MODES:
        return None, mode
    if mode not in MODE_TABLE:
        return None, -1          # Unknown / 22 / 23 / 24 RemoteControl: no LED change
    once = MODE_TABLE[mode][4]
    if once and last == mode:
        return None, last
    return _table_pattern(px, area, mode), mode


# ---------------------------------------------------------------------------
# Request arbitration (lightModeService + the two lane snapshots)
# ---------------------------------------------------------------------------
@dataclass
class Interrupt:
    top: bool = False
    tail: bool = False


class ModeArbiter:
    """State shared by the service and the two lanes (LightManager members)."""

    def __init__(self, initial_mode: int = POWER_ON):
        self.main_mode = int(initial_mode)     # +0x0c  (LightCtrl ctor: mode 6)
        self.main_prio = 0                      # +0x10
        self.top_prio = 5                       # +0x18
        self.oneshot_mode = -1                  # +0x00
        self.oneshot_prio = 4                   # +0x04
        self.tail_oneshot_active = False        # +0x298
        self.cutter_running = False             # +400 (MowerBaseDevStatus.is_cutting)

    def request(self, mode: int) -> Interrupt:
        mode = int(mode)
        prio = mode_priority(mode)
        if mode in (OPEN_CUTTER, CLOSE_CUTTER):
            if self.cutter_running or mode != CLOSE_CUTTER:
                self.oneshot_prio, self.oneshot_mode = prio, mode
                return Interrupt(tail=True)
            return Interrupt(tail=self.tail_oneshot_active)
        preempt = not (self.main_mode == mode or self.top_prio < prio)
        self.main_prio, self.main_mode = prio, mode
        if preempt:
            return Interrupt(top=True, tail=not self.tail_oneshot_active)
        return Interrupt()

    def top_snapshot(self) -> int:
        self.top_prio = self.main_prio
        return self.main_mode

    def tail_snapshot(self) -> Tuple[int, int]:
        oneshot = self.oneshot_mode
        self.oneshot_mode, self.oneshot_prio = -1, 5
        self.tail_oneshot_active = oneshot != -1
        return self.main_mode, oneshot

    def set_main_if(self, expect: int, to: int) -> bool:
        if self.main_mode != expect:
            return False
        self.main_mode, self.main_prio = to, mode_priority(to)
        return True


# ---------------------------------------------------------------------------
# Stack state -> LightMode (auto_mode)
# ---------------------------------------------------------------------------
MISSION_NAMES = ('PREFLIGHT_CHECK', 'UNDOCKING', 'WAITING_FOR_RTK', 'PLANNING', 'MOWING',
                 'TRANSIT', 'AREA_UNREACHABLE', 'MOWING_COMPLETE')
DOCK_NAMES = ('RETURNING_HOME', 'RAIN_DETECTED_DOCKING', 'COVERAGE_FAILED_DOCKING')
PAUSE_NAMES = ('BOUNDARY_PAUSED', 'RAIN_WAITING')
FAILED_NAMES = ('NAV_TO_DOCK_FAILED', 'UNDOCK_FAILED')
EMERGENCY_NAMES = ('EMERGENCY', 'BOUNDARY_EMERGENCY_STOP')
REMOTE_NAMES = ('MANUAL_MOWING', 'RECORDING')
IDLE_STATE_NAMES = ('IDLE', 'IDLE_DOCKED', 'RECORDING_COMPLETE')

# HighLevelStatus.state codes
HL_NULL, HL_IDLE, HL_AUTONOMOUS, HL_RECORDING, HL_MANUAL_MOWING = 0, 1, 2, 3, 4


@dataclass
class MappingParams:
    low_battery_percent: float = 20.0
    charge_full_percent: float = 100.0
    warn_hold_s: float = 2.0
    status_timeout_s: float = 5.0
    no_status_mode: int = IDLE


@dataclass
class StackInputs:
    hl_state: Optional[int] = None
    hl_state_name: str = ''
    hl_emergency: bool = False
    hl_is_charging: bool = False
    battery_percent: Optional[float] = None
    hl_time: Optional[float] = None
    estop_active: bool = False          # /hardware_bridge/emergency active|latched
    lift: bool = False
    stop: bool = False
    bumper: bool = False
    base_charging: bool = False          # /mower_base/status.is_charging
    last_warn_time: Optional[float] = None
    shutting_down: bool = False


def warn_now(i: StackInputs) -> bool:
    return bool(i.estop_active or i.lift or i.stop or i.bumper or i.hl_emergency
                or i.hl_state_name in EMERGENCY_NAMES)


def stack_to_mode(i: StackInputs, now: float, p: MappingParams = MappingParams()) -> int:
    """Pure mapping of our stack state to a vendor LightMode."""
    if i.shutting_down:
        return POWER_OFF
    if warn_now(i) or (i.last_warn_time is not None and now - i.last_warn_time < p.warn_hold_s):
        return WARN_SENSOR_TRIGGED
    fresh = i.hl_time is not None and (now - i.hl_time) <= p.status_timeout_s
    name = i.hl_state_name if fresh else ''
    state = i.hl_state if fresh else None
    charging = i.hl_is_charging or i.base_charging or name == 'CHARGING'
    batt = i.battery_percent
    if not fresh:
        if charging:
            return _charge_mode(batt, p)
        return p.no_status_mode
    if name in FAILED_NAMES:
        return ROBOT_UNUSUAL
    if name in REMOTE_NAMES or state in (HL_RECORDING, HL_MANUAL_MOWING):
        return REMOTE_CONTROL
    if name == 'LOW_BATTERY_DOCKING':
        return LOW_BATTERY
    if name in DOCK_NAMES:
        return DOCKING
    if name in MISSION_NAMES:
        return TASK_START
    if charging:
        return _charge_mode(batt, p)
    if name in PAUSE_NAMES:
        return TASK_PAUSE
    if name in IDLE_STATE_NAMES or state == HL_IDLE:
        if batt is not None and 0.0 < batt < p.low_battery_percent:
            return LOW_BATTERY
        return IDLE
    if state == HL_AUTONOMOUS:
        return TASK_START
    if state == HL_NULL:
        return WARN_SENSOR_TRIGGED
    return IDLE


def _charge_mode(batt, p):
    if batt is not None and batt >= p.charge_full_percent:
        return CHARGE_FINISHED
    return CHARGING


class AutoMode:
    """Edge-triggered auto mapping: emits a mode only when the mapped mode changes,
    so a manual /light_control request persists until the stack state changes."""

    def __init__(self, params: MappingParams = MappingParams()):
        self.p = params
        self.inputs = StackInputs()
        self.last_emitted: Optional[int] = None

    def note_warn(self, now: float) -> None:
        if warn_now(self.inputs):
            self.inputs.last_warn_time = now

    def evaluate(self, now: float) -> Optional[int]:
        self.note_warn(now)
        mode = stack_to_mode(self.inputs, now, self.p)
        if mode != self.last_emitted:
            self.last_emitted = mode
            return mode
        return None


# Human-readable pattern table (docs / ~/state)
def describe_mode(mode: int) -> Dict[str, str]:
    special = {
        POWER_ON: ('top: off, 0.5 s, white fill both-ends-to-middle 30 ms/step, then -> Idle',
                   'tail: white sweep top-to-bottom 150 ms/step, then off'),
        POWER_OFF: ('top: white, 0.5 s, wipe off middle-to-both-ends 100 ms/step',
                    'tail: wipe off bottom-to-top 150 ms/step'),
        OTA_ING: ('top: loop {off, 0.5 s, blue(60) middle-to-both-ends 100 ms/step}',
                  'tail: loop {off, 0.2 s, blue(60) bottom-to-top 150 ms/step, off}'),
        CHARGING: ('top: loop {green(60) middle-to-both-ends 100 ms/step, off}',
                   'tail: loop {off, 0.2 s, green(60) bottom-to-top 150 ms/step, off}'),
        OPEN_CUTTER: ('top: unchanged', 'tail one-shot: red sweep top-to-bottom 200 ms/step for 4 s'),
        CLOSE_CUTTER: ('top: unchanged',
                       'tail one-shot (only if cutter spinning): red sweep bottom-to-top 200 ms/step for 4 s'),
    }
    if mode in special:
        t, f = special[mode]
        return {'top': t, 'tail': f}
    if mode in NO_CHANGE_MODES or mode not in MODE_TABLE:
        return {'top': 'unchanged', 'tail': 'unchanged'}
    kind, color, step, arg, _ = MODE_TABLE[mode]
    c = COLOR_NAMES[color]
    if kind == 'const':
        d = '%s(120) steady' % c
    elif kind == 'breathe':
        d = '%s breathing (table 1..119, 30 ms/step, ~2.9 s period)' % c
    elif kind == 'flicker':
        d = '%s blink %d ms on/%d ms off (x%d, repeating)' % (c, step, step, arg)
    else:
        d = '%s blink x%d (%d ms), then %s steady' % (c, arg, step, c)
    return {'top': d, 'tail': d}
