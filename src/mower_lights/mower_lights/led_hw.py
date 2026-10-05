"""WS2812 status-LED hardware + the threaded light engine (ROS-free).

Backends:
  * SpiBackend  - /dev/spidev3.0, mode 0, 8 bit, 8 MHz, one SPI_IOC_MESSAGE(1) of
                  1640 bytes per frame (byte-for-byte what libws_2812.so ws2812_send does)
  * FakeBackend - records frames (tests)
  * DryRunBackend - encodes but writes nothing (dry_run:=true)

LightEngine reproduces the vendor LightManager threads: a 25 Hz sender, and two
20 Hz animation lanes (top / tail) whose waits are interruptible like WakeAbleTimer.
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

    def open(self) -> None:
        self.fd = os.open(self.device, os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, SPI_IOC_WR_MODE32, struct.pack('<I', self.mode))
        except OSError:
            fcntl.ioctl(self.fd, SPI_IOC_WR_MODE, struct.pack('<B', self.mode))
        fcntl.ioctl(self.fd, SPI_IOC_WR_BITS_PER_WORD, struct.pack('<B', self.bits))
        fcntl.ioctl(self.fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack('<I', self.speed_hz))

    def write(self, frame: bytes) -> None:
        if self.fd is None:
            self.open()
        tx = ctypes.create_string_buffer(frame, len(frame))
        rx = ctypes.create_string_buffer(len(frame))   # vendor passes an rx buffer too
        msg = spi_ioc_transfer(ctypes.addressof(tx), len(frame), self.speed_hz, self.bits,
                               rx_addr=ctypes.addressof(rx))
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


class LaneRunner:
    """One animation lane thread (topLightThread / tailLightThread)."""

    def __init__(self, name: str, engine: 'LightEngine', program: Callable, rate_hz: float):
        self.name = name
        self.engine = engine
        self.program = program        # (engine, last) -> (pattern|None, new_last)
        self.period = 1.0 / rate_hz
        self.stop_evt = threading.Event()   # WakeAbleTimer flag
        self.last = -1
        self.thread = threading.Thread(target=self._loop, name='lights-' + name, daemon=True)

    def interrupt(self) -> None:
        self.stop_evt.set()

    def run_pattern(self, pattern) -> bool:
        """Execute steps; return False if interrupted."""
        eng = self.engine
        for step in pattern:
            if isinstance(step, ll.Wait):
                if step.interruptible:
                    if self.stop_evt.wait(step.ms / 1000.0) or not eng.running:
                        pattern.close()
                        return False
                else:
                    time.sleep(step.ms / 1000.0)
            elif isinstance(step, ll.SetMainIfStill):
                with eng.state_lock:
                    eng.arbiter.set_main_if(step.expect, step.to)
        return True

    def cycle(self) -> None:
        pattern, self.last = self.program(self.engine, self.last)
        if pattern is not None:
            # pixel writes happen inside the generator; serialize them with the sender
            self.run_pattern(_locked(pattern, self.engine.pixel_lock))

    def _loop(self) -> None:
        nxt = time.monotonic()
        while self.engine.running:
            nxt += self.period
            dt = nxt - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                nxt = time.monotonic()
            self.cycle()


def _locked(pattern, lock):
    """Run each generator segment (pixel mutation) under the pixel lock, release
    it while waiting (vendor: lock_guard around setLedPin groups only)."""
    while True:
        with lock:
            try:
                step = next(pattern)
            except StopIteration:
                return
        try:
            yield step
        except GeneratorExit:
            pattern.close()
            raise


class LightEngine:
    def __init__(self, backend, pixels: ll.Pixels, initial_mode: int = ll.POWER_ON,
                 frame_rate_hz: float = 25.0, lane_rate_hz: float = 20.0,
                 on_error: Optional[Callable[[Exception], None]] = None):
        self.backend = backend
        self.pixels = pixels
        self.arbiter = ll.ModeArbiter(initial_mode)
        self.state_lock = threading.Lock()   # LightManager +0x268
        self.pixel_lock = threading.Lock()   # LightManager +0x110
        self.frame_period = 1.0 / frame_rate_hz
        self.running = False
        self.on_error = on_error
        self.frames_sent = 0
        self.last_error: Optional[str] = None
        self.top = LaneRunner('top', self, self._top_prog, lane_rate_hz)
        self.tail = LaneRunner('tail', self, self._tail_prog, lane_rate_hz)
        self.sender = threading.Thread(target=self._send_loop, name='lights-send', daemon=True)

    # --- lane programs (snapshot under state lock, like the vendor threads) ---
    def _top_prog(self, eng, last):
        with self.state_lock:
            mode = self.arbiter.top_snapshot()
            self.top.stop_evt.clear()
        return ll.top_program(self.pixels, mode, last)

    def _tail_prog(self, eng, last):
        with self.state_lock:
            mode, oneshot = self.arbiter.tail_snapshot()
            self.tail.stop_evt.clear()
        return ll.tail_program(self.pixels, mode, oneshot, last,
                               lambda: time.monotonic() * 1000.0)

    # --- API ---
    def request(self, mode: int) -> ll.Interrupt:
        with self.state_lock:
            irq = self.arbiter.request(mode)
            if irq.top:
                self.top.interrupt()
            if irq.tail:
                self.tail.interrupt()
        return irq

    def set_cutter_running(self, running: bool) -> None:
        with self.state_lock:
            self.arbiter.cutter_running = bool(running)

    @property
    def mode(self) -> int:
        return self.arbiter.main_mode

    def set_brightness(self, pct: int) -> None:
        self.pixels.brightness = max(0, min(100, int(pct)))

    def send_once(self) -> bytes:
        with self.pixel_lock:
            frame = ll.encode_frame(self.pixels.snapshot(), self.pixels.n)
        try:
            self.backend.write(frame)
            self.frames_sent += 1
            self.last_error = None
        except Exception as e:  # noqa: BLE001 - keep animating, report via callback
            self.last_error = str(e)
            if self.on_error:
                self.on_error(e)
        return frame

    def _send_loop(self) -> None:
        nxt = time.monotonic()
        while self.running:
            nxt += self.frame_period
            dt = nxt - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                nxt = time.monotonic()
            self.send_once()

    def start(self) -> None:
        self.backend.open()
        self.running = True
        self.sender.start()
        self.tail.thread.start()
        self.top.thread.start()

    def stop(self) -> None:
        self.running = False
        self.top.interrupt()
        self.tail.interrupt()
        for t in (self.top.thread, self.tail.thread, self.sender):
            if t.is_alive():
                t.join(timeout=2.0)
        self.backend.close()

    def play_blocking(self, mode: int, seconds: float) -> None:
        """Request ``mode`` and keep animating for ``seconds`` (PowerOff on shutdown)."""
        self.request(mode)
        time.sleep(max(0.0, seconds))
