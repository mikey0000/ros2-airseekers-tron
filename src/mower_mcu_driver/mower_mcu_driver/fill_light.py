"""Fill light (night-mowing illumination): host PWM + auto (darkness) logic, no ROS.

Vendor reference (``mower_base::FillLight``, mower_base_node, src/sensor/fill_light.cpp,
decompiled in native_decompile/dec/mower_base_node/mower_base_node.c ~246750):

* NOT an MCU command. The light is a host PWM: the class writes
  ``/sys/class/pwm/pwmchip1/pwm0/duty_cycle`` -- ``600000`` (ns) for on, ``0`` for off --
  then reads the file back; ``is_fill_light_on`` = read-back duty > 0 (``atomic<bool>``
  at +0x108, ``getFillLightStatus``). It re-writes the value every 10 s (ros::Timer).
* It never exports the channel nor sets ``period``/``enable``: something else had to do
  that. On this Tron nothing does, and the device tree has only ONE PWM controller
  enabled (``pwm@febe0030`` = PWM11, pin GPIO4_B4, which is ``pwmchip0``), so
  ``pwmchip1`` never exists, the vendor write fails silently, the read-back yields 0 and
  the vendor status always said ``is_fill_light_on: False``.
* Service ``/fill_light_control`` (``mower_msgs/Trigger``, ``data == "on"`` -> on).
  The vendor app config (``FillLightControlReq{enable, brightness /*reserved*/}``,
  ``FillLightAuto``) shows brightness was never implemented.

``PwmFillLight`` reproduces that and adds what the vendor relied on someone else doing
(export, period, enable), all relative to an injectable sysfs root for tests.
"""

import json
import math
import os
import time

VENDOR_CHIP = '/sys/class/pwm/pwmchip1'
VENDOR_ON_DUTY_NS = 600000

MODES = ('off', 'on', 'auto')


# --------------------------------------------------------------------------- solar
def sun_elevation_deg(lat_deg, lon_deg, unix_time):
    """Solar elevation (degrees, refraction ignored) with the NOAA low-precision
    algorithm. Error < ~0.5 deg, plenty for "is it dark". No network, no tz needed."""
    jd = unix_time / 86400.0 + 2440587.5
    n = jd - 2451545.0
    mean_lon = math.radians((280.460 + 0.9856474 * n) % 360.0)
    mean_anom = math.radians((357.528 + 0.9856003 * n) % 360.0)
    ecl_lon = mean_lon + math.radians(1.915) * math.sin(mean_anom) \
        + math.radians(0.020) * math.sin(2 * mean_anom)
    obliq = math.radians(23.439 - 0.0000004 * n)
    ra = math.atan2(math.cos(obliq) * math.sin(ecl_lon), math.cos(ecl_lon))
    dec = math.asin(math.sin(obliq) * math.sin(ecl_lon))
    gmst_h = (18.697374558 + 24.06570982441908 * n) % 24.0
    lst = math.radians(gmst_h * 15.0 + lon_deg)
    ha = lst - ra
    lat = math.radians(lat_deg)
    sin_el = math.sin(lat) * math.sin(dec) + math.cos(lat) * math.cos(dec) * math.cos(ha)
    return math.degrees(math.asin(max(-1.0, min(1.0, sin_el))))


class DarknessDetector:
    """Dark when the sun is below ``dark_below_deg``; light again only above
    ``dark_below_deg + hysteresis_deg``. Unknown position -> never dark (fail off)."""

    def __init__(self, dark_below_deg=-3.0, hysteresis_deg=1.0):
        self.dark_below = float(dark_below_deg)
        self.hyst = float(hysteresis_deg)
        self.dark = False
        self.elevation = None

    def update(self, lat, lon, unix_time):
        if lat is None or lon is None or (lat == 0.0 and lon == 0.0):
            self.elevation = None
            self.dark = False
            return False
        self.elevation = sun_elevation_deg(lat, lon, unix_time)
        if self.dark:
            self.dark = self.elevation < self.dark_below + self.hyst
        else:
            self.dark = self.elevation < self.dark_below
        return self.dark


def want_on(mode, state_name, dark, active_states=('MOWING', 'TRANSIT')):
    """Desired light state. ``auto``: on only while the mission is in an active state
    and it is dark."""
    if mode == 'on':
        return True
    if mode == 'auto':
        return bool(dark) and (state_name or '').upper() in active_states
    return False


def parse_datum_env(text):
    """``(lat, lon)`` from /userdata/ros2/datum.env (DATUM_LAT=/DATUM_LON=) or None."""
    vals = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('export '):
            line = line[7:].strip()
        key, sep, value = line.partition('=')
        if sep:
            try:
                vals[key.strip()] = float(value.strip().strip('"\''))
            except ValueError:
                pass
    if 'DATUM_LAT' in vals and 'DATUM_LON' in vals:
        return vals['DATUM_LAT'], vals['DATUM_LON']
    return None


# --------------------------------------------------------------------------- PWM
def duty_for(brightness_pct, period_ns):
    """Duty (ns) for a brightness in percent, clamped to [0, period]."""
    pct = max(0.0, min(100.0, float(brightness_pct)))
    return int(round(period_ns * pct / 100.0))


class PwmFillLight:
    """sysfs PWM channel ``<chip>/pwm<channel>``. Every method is exception-free and
    returns the read-back state; ``error`` holds the last failure reason."""

    def __init__(self, chip_path=VENDOR_CHIP, channel=0, period_ns=1000000):
        self.chip = chip_path
        self.channel = int(channel)
        self.period_ns = int(period_ns)
        self.error = ''

    @property
    def pwm_dir(self):
        return os.path.join(self.chip, 'pwm%d' % self.channel)

    def available(self):
        return os.path.isdir(self.chip)

    @staticmethod
    def _write(path, value):
        with open(path, 'w') as f:
            f.write(str(value))

    @staticmethod
    def _read_int(path):
        with open(path) as f:
            return int(f.read().strip())

    def _prepare(self):
        if not os.path.isdir(self.pwm_dir):
            self._write(os.path.join(self.chip, 'export'), self.channel)
            for _ in range(20):            # udev may need a moment
                if os.path.isdir(self.pwm_dir):
                    break
                time.sleep(0.01)
        period = self._read_int(os.path.join(self.pwm_dir, 'period'))
        if period != self.period_ns:
            # duty must never exceed period: drop duty first
            self._write(os.path.join(self.pwm_dir, 'duty_cycle'), 0)
            self._write(os.path.join(self.pwm_dir, 'period'), self.period_ns)

    def set(self, on, brightness_pct=60.0):
        """Drive the channel; returns the read-back on/off (False if unavailable)."""
        self.error = ''
        if not self.available():
            self.error = 'PWM controller %s not present (no fill-light channel on this ' \
                         'board/device tree)' % self.chip
            return False
        try:
            self._prepare()
            duty = duty_for(brightness_pct, self.period_ns) if on else 0
            self._write(os.path.join(self.pwm_dir, 'duty_cycle'), duty)
            self._write(os.path.join(self.pwm_dir, 'enable'), 1 if duty > 0 else 0)
        except (OSError, ValueError) as exc:
            self.error = '%s: %s' % (self.pwm_dir, exc)
        return self.read()

    def read(self):
        """Vendor semantics: on == read-back duty_cycle > 0 (and the channel enabled)."""
        try:
            duty = self._read_int(os.path.join(self.pwm_dir, 'duty_cycle'))
            try:
                enabled = self._read_int(os.path.join(self.pwm_dir, 'enable')) == 1
            except (OSError, ValueError):
                enabled = True
            return duty > 0 and enabled
        except (OSError, ValueError):
            return False


def status_json(mode, on, requested, available, error, dark, elevation, state_name,
                brightness, chip):
    return json.dumps({
        'mode': mode, 'on': bool(on), 'requested': bool(requested),
        'available': bool(available), 'error': error or '',
        'dark': bool(dark),
        'sun_elevation_deg': None if elevation is None else round(elevation, 2),
        'mission_state': state_name or '', 'brightness': brightness, 'pwm_chip': chip,
    }, sort_keys=True)
