"""Fill light: solar/auto logic and the sysfs PWM writer (fake sysfs tree)."""
import calendar
import os

from mower_mcu_driver.fill_light import (DarknessDetector, PwmFillLight, duty_for,
                                         parse_datum_env, sun_elevation_deg, want_on)


def ts(y, mo, d, h, mi=0):
    return calendar.timegm((y, mo, d, h, mi, 0))


def test_solar_noon_and_midnight_paris():
    lat, lon = 48.85, 2.35
    noon = sun_elevation_deg(lat, lon, ts(2026, 6, 21, 11, 50))   # ~solar noon UTC
    assert 63.0 < noon < 66.0                                      # 90 - 48.85 + 23.44
    assert sun_elevation_deg(lat, lon, ts(2026, 6, 21, 23, 50)) < -15.0
    winter = sun_elevation_deg(lat, lon, ts(2026, 12, 21, 11, 50))
    assert 16.0 < winter < 19.0


def test_equinox_sunset_near_horizon():
    # Equator, lon 0, equinox: sun sets ~18:07 UTC
    assert abs(sun_elevation_deg(0.0, 0.0, ts(2026, 3, 20, 18, 7))) < 1.5


def test_darkness_hysteresis_and_unknown_position():
    d = DarknessDetector(dark_below_deg=-3.0, hysteresis_deg=1.0)
    assert d.update(None, None, 0) is False and d.elevation is None
    assert d.update(0.0, 0.0, ts(2026, 6, 21, 0)) is False       # 0/0 = unset datum
    assert d.update(48.85, 2.35, ts(2026, 6, 21, 23)) is True
    assert d.update(48.85, 2.35, ts(2026, 6, 21, 12)) is False


def test_hysteresis_band():
    lat, lon = 48.85, 2.35
    t = ts(2026, 6, 21, 19, 0)
    while sun_elevation_deg(lat, lon, t) > -2.5:   # walk into dusk, 30 s steps
        t += 30
    band = t                                       # elevation ~ -2.5 (inside -3..-2)
    d = DarknessDetector(-3.0, 1.0)
    assert d.update(lat, lon, band) is False       # not yet below -3
    while sun_elevation_deg(lat, lon, t) > -3.2:
        t += 30
    assert d.update(lat, lon, t) is True
    assert d.update(lat, lon, band) is True        # stays dark inside the band


def test_want_on():
    assert want_on('on', 'IDLE_DOCKED', False)
    assert not want_on('off', 'MOWING', True)
    assert want_on('auto', 'MOWING', True)
    assert want_on('auto', 'transit', True)
    assert not want_on('auto', 'MOWING', False)
    assert not want_on('auto', 'CHARGING', True)
    assert not want_on('auto', '', True)


def test_parse_datum_env():
    assert parse_datum_env('# x\nDATUM_LAT=48.1\nexport DATUM_LON="2.5"\n') == (48.1, 2.5)
    assert parse_datum_env('DATUM_LAT=abc\n') is None


def test_duty_for():
    assert duty_for(60, 1000000) == 600000      # vendor "on" value
    assert duty_for(150, 1000) == 1000
    assert duty_for(-1, 1000) == 0


def _fake_chip(tmp_path):
    chip = tmp_path / 'pwmchip1'
    chip.mkdir()
    (chip / 'export').write_text('')
    # sysfs export creates pwm0; emulate by pre-creating it on first write
    return chip


def test_pwm_absent_chip(tmp_path):
    p = PwmFillLight(str(tmp_path / 'pwmchip1'))
    assert p.set(True) is False
    assert 'not present' in p.error
    assert p.read() is False


def test_pwm_on_off(tmp_path):
    chip = _fake_chip(tmp_path)
    pwm = chip / 'pwm0'
    pwm.mkdir()
    for n, v in (('period', '0'), ('duty_cycle', '0'), ('enable', '0')):
        (pwm / n).write_text(v)
    p = PwmFillLight(str(chip), 0, 1000000)
    assert p.set(True, 60) is True and p.error == ''
    assert (pwm / 'period').read_text() == '1000000'
    assert (pwm / 'duty_cycle').read_text() == '600000'
    assert (pwm / 'enable').read_text() == '1'
    assert p.set(False) is False
    assert (pwm / 'duty_cycle').read_text() == '0'
    assert (pwm / 'enable').read_text() == '0'


def test_pwm_export_failure_reported(tmp_path):
    chip = _fake_chip(tmp_path)           # export "works" but pwm0 never appears
    p = PwmFillLight(str(chip), 0)
    assert p.set(True) is False
    assert p.error
    assert os.path.exists(str(chip / 'export'))
