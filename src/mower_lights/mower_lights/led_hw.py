"""WS2812 status-LED hardware + the threaded light engine (ROS-free).

Backends:
  * SpiBackend  - /dev/spidev3.0, mode 0, 8 bit, 8 MHz, one SPI_IOC_MESSAGE(1) of
                  1640 bytes per frame (byte-for-byte what libws_2812.so ws2812_send does)
  * FakeBackend - records frames (tests)
  * DryRunBackend - encodes but writes nothing (dry_run:=true)

LightEngine reproduces the vendor LightManager lanes (top / tail, 20 Hz cycle, waits
interruptible like WakeAbleTimer) and its 25 Hz sender, but CPU-lean (RK3588):

* both lanes run as cooperative generators on ONE animation thread that sleeps until the
  next pattern step is due; a lane whose program has nothing to do (static pattern
  already rendered) is not polled at 20 Hz but woken when the mode state changes, so a
  static pattern costs no wakes at all;
* the sender thread writes a frame only when the pixel buffer changed (at most
  ``frame_rate_hz``) plus a ``keepalive_s`` refresh (1 s) of an unchanged strip;
* frames are encoded with a per-byte lookup table and written with one preallocated
  SPI_IOC_MESSAGE ioctl.

The LED output (pixel values and their timing) is the same as the vendor's.
"""
import ctypes
import fcntl
import os
import struct
import threading
import time
from typing import Callable, List, Optional

from . import lights_logic as ll

# <linux/spi/spidev.h>
SPI_IOC_WR_MODE = 0x40016B01
SPI_IOC_WR_MODE32 = 0x40046B05
SPI_IOC_WR_BITS_PER_WORD = 0x40016B03
SPI_IOC_WR_MAX_SPEED_HZ = 0x40046B04
SPI_IOC_MESSAGE_1 = 0x40206B00      # _IOW('k', 0, char[32])  (sizeof spi_ioc_transfer = 32)


def spi_ioc_transfer(tx_addr: int, length: int, speed_hz: int, bits: int,
                     rx_addr: int = 0, delay_usecs: int = 0) -> bytes:
    """struct spi_ioc_transfer {u64 tx_buf, rx_buf; u32 len, speed_hz;
    u16 delay_usecs; u8 bits_per_word, cs_change, tx_nbits, rx_nbits, word_delay_usecs, pad}"""
    return struct.pack('<QQIIHBBBBBB', tx_addr, rx_addr, length, speed_hz,
                       delay_usecs, bits, 0, 0, 0, 0, 0)


class SpiBackend:
    def __init__(self, device: str = '/dev/spidev3.0', speed_hz: int = ll.SPI_SPEED_HZ,
                 mode: int = ll.SPI_MODE, bits: int = ll.SPI_BITS,
                 pre_delay_s: float = 0.005, post_delay_s: float = 0.015):
        self.device, self.speed_hz, self.mode, self.bits = device, int(speed_hz), int(mode), int(bits)
        self.pre_delay_s, self.post_delay_s = pre_delay_s, post_delay_s
        self.fd: Optional[int] = None
        self.last_frame: Optional[bytes] = None
        self.frames = 0
        self.errors = 0
        self._xfer = None

    def open(self) -> None:
        self.fd = os.open(self.device, os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, SPI_IOC_WR_MODE32, struct.pack('<I', self.mode))
        except OSError:
            fcntl.ioctl(self.fd, SPI_IOC_WR_MODE, struct.pack('<B', self.mode))
        fcntl.ioctl(self.fd, SPI_IOC_WR_BITS_PER_WORD, struct.pack('<B', self.bits))
        fcntl.ioctl(self.fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack('<I', self.speed_hz))

    def _transfer(self, length: int):
        """Preallocated tx/rx buffers + spi_ioc_transfer for ``length``-byte frames."""
        if self._xfer is None or self._xfer[0] != length:
            tx = ctypes.create_string_buffer(length)
            rx = ctypes.create_string_buffer(length)   # vendor passes an rx buffer too
            msg = spi_ioc_transfer(ctypes.addressof(tx), length, self.speed_hz, self.bits,
                                   rx_addr=ctypes.addressof(rx))
            self._xfer = (length, tx, rx, msg)
        return self._xfer

    def write(self, frame: bytes) -> None:
        if self.fd is None:
            self.open()
        _n, tx, _rx, msg = self._transfer(len(frame))
        ctypes.memmove(tx, frame, len(frame))
        if self.pre_delay_s:
            time.sleep(self.pre_delay_s)
        try:
            fcntl.ioctl(self.fd, SPI_IOC_MESSAGE_1, msg)
        except OSError:
            self.errors += 1
            raise
        self.last_frame = frame
        self.frames += 1
        if self.post_delay_s:
            time.sleep(self.post_delay_s)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class FakeBackend:
    def __init__(self):
        self.frames: List[bytes] = []
        self.last_frame: Optional[bytes] = None
        self.errors = 0
        self.closed = False

    def open(self) -> None:
        pass

    def write(self, frame: bytes) -> None:
        self.frames.append(frame)
        self.last_frame = frame

    def close(self) -> None:
        self.closed = True


class DryRunBackend(FakeBackend):
    def write(self, frame: bytes) -> None:   # keep only the last frame (bounded memory)
        self.last_frame = frame
        self.frames = [frame]


def read_spi_stats(stats_dir: str) -> dict:
    """Kernel-side readback: /sys/class/spi_master/spiN/statistics counters."""
    out = {}
    for k in ('messages', 'transfers', 'bytes_tx', 'errors'):
        try:
            with open(os.path.join(stats_dir, k)) as f:
                out[k] = int(f.read().strip() or 0)
        except (OSError, ValueError):
            pass
    return out


_NEVER = float('inf')


class Lane:
    """State of one animation lane (topLightThread / tailLightThread)."""

    def __init__(self, name: str, program: Callable, rate_hz: float):
        self.name = name
        self.program = program          # () -> pattern | None (snapshots under state lock)
        self.period = 1.0 / rate_hz
        self.last = -1                  # vendor DAT_00310058 / DAT_0031005c
        self.interrupted = False        # WakeAbleTimer flag (cleared by each snapshot)
        self.pattern = None             # running generator
        self.wait_interruptible = False
        self.wake = 0.0                 # monotonic time of the next action
        self.nxt = None                 # cycle schedule (vendor: nxt += period)
        self.idle_gen = None            # state generation of the last "nothing to do"

    def interrupt(self) -> None:        # kept for API compatibility
        self.interrupted = True


class LightEngine:
    def __init__(self, backend, pixels: ll.Pixels, initial_mode: int = ll.POWER_ON,
                 frame_rate_hz: float = 25.0, lane_rate_hz: float = 20.0,
                 on_error: Optional[Callable[[Exception], None]] = None,
                 keepalive_s: float = 1.0):
        self.backend = backend
        self.pixels = pixels
        self.arbiter = ll.ModeArbiter(initial_mode)
        self.state_lock = threading.Lock()   # LightManager +0x268
        self.pixel_lock = threading.Lock()   # LightManager +0x110
        self.frame_period = 1.0 / frame_rate_hz
        self.keepalive_s = float(keepalive_s)
        self.running = False
        self.on_error = on_error
        self.frames_sent = 0
        self.last_error: Optional[str] = None
        self.top = Lane('top', self._top_prog, lane_rate_hz)
        self.tail = Lane('tail', self._tail_prog, lane_rate_hz)
        self._lanes = (self.top, self.tail)
        self._gen = 0                        # bumped on every arbiter state change
        self._anim_cond = threading.Condition()
        self._anim_kick = False
        self._send_cond = threading.Condition()
        self._dirty = False
        self.animator = threading.Thread(target=self._anim_loop, name='lights-anim',
                                         daemon=True)
        self.sender = threading.Thread(target=self._send_loop, name='lights-send', daemon=True)

    # --- lane programs (snapshot under state lock, like the vendor threads) ---
    def _top_prog(self):
        with self.state_lock:
            mode = self.arbiter.top_snapshot()
            self.top.interrupted = False
            gen = self._gen
        pattern, self.top.last = ll.top_program(self.pixels, mode, self.top.last)
        return pattern, gen

    def _tail_prog(self):
        with self.state_lock:
            mode, oneshot = self.arbiter.tail_snapshot()
            self.tail.interrupted = False
            gen = self._gen
        pattern, self.tail.last = ll.tail_program(self.pixels, mode, oneshot, self.tail.last,
                                                  lambda: time.monotonic() * 1000.0)
        return pattern, gen

    # --- API ---
    def request(self, mode: int) -> ll.Interrupt:
        with self.state_lock:
            irq = self.arbiter.request(mode)
            if irq.top:
                self.top.interrupted = True
            if irq.tail:
                self.tail.interrupted = True
            self._gen += 1
        self._kick()
        return irq

    def set_cutter_running(self, running: bool) -> None:
        with self.state_lock:
            self.arbiter.cutter_running = bool(running)

    @property
    def mode(self) -> int:
        return self.arbiter.main_mode

    def set_brightness(self, pct: int) -> None:
        self.pixels.brightness = max(0, min(100, int(pct)))

    def _write(self, frame: bytes) -> None:
        try:
            self.backend.write(frame)
            self.frames_sent += 1
            self.last_error = None
        except Exception as e:  # noqa: BLE001 - keep animating, report via callback
            self.last_error = str(e)
            if self.on_error:
                self.on_error(e)

    def send_once(self) -> bytes:
        with self.pixel_lock:
            buf = self.pixels.snapshot()
        frame = ll.encode_frame(buf, self.pixels.n)
        self._write(frame)
        return frame

    # --- animation thread ---
    def _kick(self) -> None:
        with self._anim_cond:
            self._anim_kick = True
            self._anim_cond.notify()

    def _mark_dirty(self) -> None:
        with self._send_cond:
            self._dirty = True
            self._send_cond.notify()

    def _report(self, e: Exception) -> None:
        self.last_error = str(e)
        if self.on_error:
            self.on_error(e)

    def _end_cycle(self, lane: Lane, now: float) -> None:
        """Vendor lane loop: ``nxt += period``; start now if the cycle overran."""
        lane.pattern = None
        lane.nxt += lane.period
        if lane.nxt <= now:
            lane.nxt = now
        lane.wake = lane.nxt

    def _advance(self, lane: Lane) -> None:
        """Run pattern segments until the next wait (or the end of the pattern)."""
        changed = False
        try:
            while True:
                with self.pixel_lock:
                    try:
                        step = next(lane.pattern)
                    except StopIteration:
                        step = None
                changed = True
                if step is None:
                    self._end_cycle(lane, time.monotonic())
                    return
                if isinstance(step, ll.Wait):
                    if step.interruptible and (lane.interrupted or not self.running):
                        lane.pattern.close()
                        self._end_cycle(lane, time.monotonic())
                        return
                    lane.wait_interruptible = step.interruptible
                    lane.wake = time.monotonic() + step.ms / 1000.0
                    return
                if isinstance(step, ll.SetMainIfStill):
                    with self.state_lock:
                        if self.arbiter.set_main_if(step.expect, step.to):
                            self._gen += 1
        except Exception as e:  # noqa: BLE001 - a broken pattern must not kill the lights
            self._report(e)
            try:
                lane.pattern.close()
            except Exception:  # noqa: BLE001
                pass
            self._end_cycle(lane, time.monotonic())
        finally:
            if changed:
                self._mark_dirty()

    def _service(self, lane: Lane, now: float) -> None:
        if lane.pattern is not None:
            if lane.wait_interruptible and (lane.interrupted or not self.running):
                lane.pattern.close()
                self._end_cycle(lane, now)
            elif now < lane.wake:
                return
            else:
                self._advance(lane)
                return
        # between cycles
        if lane.idle_gen is not None:
            if lane.idle_gen == self._gen:
                return                      # nothing changed since "nothing to do"
            lane.idle_gen = None
            if lane.nxt < now:
                lane.nxt = lane.wake = now
        if now < lane.wake:
            return
        pattern, gen = lane.program()
        if pattern is None:
            lane.idle_gen = gen
            self._end_cycle(lane, now)
            return
        lane.pattern = pattern
        self._advance(lane)

    def _next_wake(self) -> float:
        t = _NEVER
        for lane in self._lanes:
            if lane.pattern is None and lane.idle_gen is not None and lane.idle_gen == self._gen:
                continue
            t = min(t, lane.wake)
        return t

    def _anim_loop(self) -> None:
        now = time.monotonic()
        for lane in self._lanes:
            lane.nxt = lane.wake = now
        while self.running:
            now = time.monotonic()
            for lane in self._lanes:
                # 1 ms slack: steps of both lanes that fall due together share one wake
                self._service(lane, now + 0.001)
            with self._anim_cond:
                if self._anim_kick:
                    self._anim_kick = False
                    continue
                wait = self._next_wake() - time.monotonic()
                if wait > 0 and self.running:
                    self._anim_cond.wait(None if wait == _NEVER else wait)
                self._anim_kick = False

    # --- sender thread ---
    def _send_loop(self) -> None:
        last_buf = None
        last_send = -_NEVER
        while self.running:
            with self._send_cond:
                while self.running and not self._dirty:
                    if self.keepalive_s > 0:
                        wait = last_send + self.keepalive_s - time.monotonic()
                        if wait <= 0:
                            break
                        self._send_cond.wait(wait)
                    else:
                        self._send_cond.wait()
                self._dirty = False
            if not self.running:
                break
            # at most frame_rate_hz: later changes in the gap coalesce into this frame
            wait = last_send + self.frame_period - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            with self.pixel_lock:
                buf = self.pixels.snapshot()
            now = time.monotonic()
            if buf == last_buf and (self.keepalive_s <= 0 or now - last_send < self.keepalive_s):
                continue
            last_send = now
            self._write(ll.encode_frame(buf, self.pixels.n))
            last_buf = buf

    def start(self) -> None:
        self.backend.open()
        self.running = True
        self._dirty = True               # first frame right away
        self.sender.start()
        self.animator.start()

    def stop(self) -> None:
        self.running = False
        self._kick()
        with self._send_cond:
            self._send_cond.notify()
        for t in (self.animator, self.sender):
            if t.is_alive():
                t.join(timeout=2.0)
        self.backend.close()

    def play_blocking(self, mode: int, seconds: float) -> None:
        """Request ``mode`` and keep animating for ``seconds`` (PowerOff on shutdown)."""
        self.request(mode)
        time.sleep(max(0.0, seconds))
