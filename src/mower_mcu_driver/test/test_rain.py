"""Rain ADC polarity / debounce (mower_mcu_driver.rain, pure Python)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_mcu_driver.rain import RainDetector  # noqa: E402


def feed(det, raw, t0, t1, dt=0.5):
    t = t0
    while t <= t1 + 1e-9:
        det.update(raw, t)
        t += dt
    return det.triggered


def test_polarity_matches_vendor_band():
    d = RainDetector()
    assert d.classify(4092) == d.DRY        # dry weather reading on the robot
    assert d.classify(4000) == d.WET        # vendor: rain_min <= v <= rain_max is rain
    assert d.classify(2500) == d.WET
    assert d.classify(2000) == d.WET
    assert d.classify(1999) == d.INVALID    # below rain_min: fault, not rain


def test_dry_never_triggers():
    d = RainDetector(window=1)
    assert feed(d, 4092, 0.0, 3600.0) is False
    assert d.value == 4092


def test_wet_debounce_5s():
    d = RainDetector(window=1)
    assert feed(d, 3000, 0.0, 4.5) is False
    assert d.update(3000, 5.0) is True


def test_short_wet_blip_resets():
    d = RainDetector(window=1)
    feed(d, 3000, 0.0, 4.0)
    d.update(4092, 4.5)
    assert feed(d, 3000, 5.0, 9.5) is False
    assert d.update(3000, 10.0) is True


def test_clear_needs_600s_dry():
    d = RainDetector(window=1)
    feed(d, 3000, 0.0, 6.0)
    assert d.triggered
    assert feed(d, 4092, 6.5, 6.5 + 599.0) is True
    assert d.update(4092, 6.5 + 600.0) is False


def test_wet_spell_during_clear_restarts_timer():
    d = RainDetector(window=1)
    feed(d, 3000, 0.0, 6.0)
    feed(d, 4092, 6.5, 300.0)
    d.update(3000, 300.5)
    assert feed(d, 4092, 301.0, 900.0) is True    # < 600 s since the last wet sample
    assert d.update(4092, 901.0) is False


def test_invalid_low_reading_is_not_rain():
    d = RainDetector(window=1)
    assert feed(d, 500, 0.0, 60.0) is False


def test_hysteresis_band_holds_class():
    d = RainDetector(wet_below=3500, dry_above=3800, window=1)
    feed(d, 3000, 0.0, 6.0)
    assert d.triggered
    assert d.classify(3700) == d.WET              # in band: keeps last class (wet)
    d2 = RainDetector(wet_below=3500, dry_above=3800, window=1)
    d2.update(4092, 0.0)
    assert d2.classify(3700) == d2.DRY


def test_window_filter_averages():
    d = RainDetector(window=4)
    for i, raw in enumerate((4092, 4092, 4092, 3000)):
        d.update(raw, i * 0.5)
    assert d.value == 3819
    assert d.triggered is False                   # a single wet sample never triggers
