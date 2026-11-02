# SPDX-License-Identifier: GPL-3.0-or-later
"""LIS2MDL / IIS2MDC magnetometer readers (Metoak stereo module, i2c-8 address 0x1e).

UNTESTED ON HARDWARE (2026-11-03: the mower was not reachable for the IIO discovery). Two
backends, ``auto`` tries them in this order:

``iio``  a kernel driver exposes the chip as an IIO device (mainline ``st_magn`` names it
         ``lis2mdl`` / ``iis2mdc``; any device with ``in_magn_x_raw`` matches). Channels
         ``in_magn_{x,y,z}_raw`` scaled by ``in_magn_{x,y,z}_scale`` or the shared
         ``in_magn_scale`` (gauss / LSB per the IIO ABI), ``sampling_frequency`` set to the
         nearest of ``sampling_frequency_available``. Polled through sysfs: each raw read
         of an st_sensors device without a buffer is a one-shot conversion (enable, wait
         ``2 / ODR``, read, disable), so ~3 x 20 ms at ODR 100: the achievable rate is
         measured and logged, typically 10-15 Hz.
``i2c``  no IIO device: the registers directly through ``/dev/i2c-<bus>`` (``I2C_SLAVE``,
         not ``I2C_SLAVE_FORCE``: a bound kernel driver makes it fail with EBUSY instead of
         fighting the driver). WHO_AM_I 0x4F = 0x40, CFG_REG_A 0x60 = 0x8C (temperature
         compensation, ODR 100 Hz, continuous), CFG_REG_B 0x61 = 0x02 (offset cancellation),
         CFG_REG_C 0x62 = 0x10 (BDU), output 0x68..0x6D little-endian s16, 1.5 mgauss / LSB.
         Idle mode (CFG_REG_A = 0x03) on close.

Both return raw chip-axis microtesla; the node rotates them into base_link.
"""
from __future__ import annotations

import os
import struct

IIO_ROOT = '/sys/bus/iio/devices'
MAG_NAMES = ('lis2mdl', 'iis2mdc', 'lsm303agr_magn', 'lsm303agr-magn', 'st_magn')

WHO_AM_I = 0x4F
WHO_AM_I_VALUE = 0x40
CFG_REG_A, CFG_REG_B, CFG_REG_C = 0x60, 0x61, 0x62
OUTX_L = 0x68
I2C_SLAVE = 0x0703


def _r(path):
    with open(path) as f:
        return f.read().strip()


def find_iio_mag(root=IIO_ROOT, names=MAG_NAMES):
    """-> 'iio:deviceN' of the first magnetometer (by name, else by an in_magn_x_raw), or None."""
    try:
        entries = sorted(os.listdir(root))
    except OSError:
        return None
    by_chan = None
    for dev in entries:
        base = os.path.join(root, dev)
        try:
            name = _r(os.path.join(base, 'name'))
        except OSError:
            continue
        if any(name.startswith(n) for n in names):
            return dev
        if by_chan is None and os.path.exists(os.path.join(base, 'in_magn_x_raw')):
            by_chan = dev
    return by_chan


def iio_scales(base):
    """Per-axis gauss/LSB: in_magn_<a>_scale, else the shared in_magn_scale."""
    out = []
    for a in 'xyz':
        for name in ('in_magn_%s_scale' % a, 'in_magn_scale'):
            p = os.path.join(base, name)
            if os.path.exists(p):
                out.append(float(_r(p)))
                break
        else:
            raise OSError('no in_magn scale in %s' % base)
    return out


class IioMag:
    backend = 'iio'

    def __init__(self, dev, odr=100.0, root=IIO_ROOT):
        self.dev = dev
        self.base = os.path.join(root, dev)
        self.name = _r(os.path.join(self.base, 'name'))
        avail = os.path.join(self.base, 'sampling_frequency_available')
        sf = os.path.join(self.base, 'sampling_frequency')
        if os.path.exists(avail) and os.path.exists(sf):
            vals = [float(v) for v in _r(avail).split()]
            if vals:
                want = min(vals, key=lambda v: abs(v - float(odr)))
                try:
                    with open(sf, 'w') as f:
                        f.write('%g' % want)
                except OSError:
                    pass
        self.odr = float(_r(sf)) if os.path.exists(sf) else 0.0
        self.scale_ut = [s * 100.0 for s in iio_scales(self.base)]   # gauss -> uT
        self._fds = [os.open(os.path.join(self.base, 'in_magn_%s_raw' % a), os.O_RDONLY)
                     for a in 'xyz']

    def describe(self):
        return 'IIO %s (%s) ODR %g Hz, scale %s uT/LSB' % (
            self.dev, self.name, self.odr, ', '.join('%.4g' % s for s in self.scale_ut))

    def read(self):
        """-> (x, y, z) uT in the chip axes."""
        out = []
        for fd, s in zip(self._fds, self.scale_ut):
            os.lseek(fd, 0, os.SEEK_SET)
            out.append(int(os.read(fd, 32).strip()) * s)
        return tuple(out)

    def close(self):
        for fd in self._fds:
            try:
                os.close(fd)
            except OSError:
                pass
        self._fds = []


def decode_xyz(raw6, ut_per_lsb=0.15):
    """6 output bytes (OUTX_L..OUTZ_H) -> (x, y, z) uT."""
    x, y, z = struct.unpack('<3h', bytes(raw6))
    return x * ut_per_lsb, y * ut_per_lsb, z * ut_per_lsb


class I2cMag:
    backend = 'i2c'

    def __init__(self, bus=8, addr=0x1E, ut_per_lsb=0.15, dev_fmt='/dev/i2c-%d'):
        import fcntl
        self.path = dev_fmt % int(bus)
        self.addr = int(addr)
        self.ut_per_lsb = float(ut_per_lsb)
        self.fd = os.open(self.path, os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, I2C_SLAVE, self.addr)
            who = self._read(WHO_AM_I, 1)[0]
            if who != WHO_AM_I_VALUE:
                raise OSError('WHO_AM_I 0x%02x at %s:0x%02x (want 0x%02x)'
                              % (who, self.path, self.addr, WHO_AM_I_VALUE))
            self._write(CFG_REG_A, 0x8C)
            self._write(CFG_REG_B, 0x02)
            self._write(CFG_REG_C, 0x10)
        except Exception:
            os.close(self.fd)
            raise

    def _write(self, reg, val):
        os.write(self.fd, bytes((reg, val)))

    def _read(self, reg, n):
        os.write(self.fd, bytes((reg,)))
        return os.read(self.fd, n)

    def describe(self):
        return 'i2c %s:0x%02x (LIS2MDL registers, 100 Hz continuous, %.3f uT/LSB)' % (
            self.path, self.addr, self.ut_per_lsb)

    def read(self):
        return decode_xyz(self._read(OUTX_L, 6), self.ut_per_lsb)

    def close(self):
        if self.fd is None:
            return
        try:
            self._write(CFG_REG_A, 0x03)       # idle
        except OSError:
            pass
        try:
            os.close(self.fd)
        except OSError:
            pass
        self.fd = None


def open_mag(backend='auto', i2c_bus=8, i2c_addr=0x1E, odr=100.0, root=IIO_ROOT):
    """-> IioMag | I2cMag, or raise OSError with every backend's reason."""
    errs = []
    if backend in ('auto', 'iio'):
        dev = find_iio_mag(root)
        if dev is not None:
            try:
                return IioMag(dev, odr=odr, root=root)
            except (OSError, ValueError) as exc:
                errs.append('iio %s: %s' % (dev, exc))
        else:
            errs.append('iio: no magnetometer IIO device')
    if backend in ('auto', 'i2c'):
        try:
            return I2cMag(i2c_bus, i2c_addr)
        except (OSError, ImportError) as exc:
            errs.append('i2c-%d:0x%02x: %s' % (int(i2c_bus), int(i2c_addr), exc))
    raise OSError('; '.join(errs) or 'unknown backend %r' % backend)
