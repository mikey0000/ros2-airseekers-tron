"""Rain sensor (host ADC) debounce logic -- pure Python, no ROS.

Vendor reference (``dev_health::DevHealthHandler::rainSensorProcess``, mower_base_node,
dev_health_check.cpp:0x250, decompiled in native_decompile/dec/mower_base_node):

* ``/dev/rain`` -> ``/sys/.../saradc/iio:device0/in_voltage4_raw``, 12-bit SARADC (0..4095).
* Read every 100 ms (every 2nd pass of a 50 ms loop), 10-sample moving-window filter.
* ``rain_min``/``rain_max`` (base.yaml: 2000/4000). The sample counts as *wet* only while
  ``rain_min <= value <= rain_max``; above ``rain_max`` is dry (open sensor is pulled up,
  ~4092 in dry weather), below ``rain_min`` is treated as not-rain too (short/fault).
* Triggered once the value has been continuously in the wet band for 1000 ms; any dry
  sample clears it immediately (no hold-off).  The flag overrides ``rain_triggered`` in
  ``mower_base/status`` (``DevStatus::setRainStatus``) and raises notice code 6.

This port keeps the polarity and band, but uses longer, asymmetric debouncing (wet for
``debounce_s`` -> triggered, dry for ``clear_s`` -> cleared) so a single drop or a brief
dry spell does not make the mission resume mowing in drizzle.
"""

import collections


class RainDetector:
    WET, DRY, INVALID = 'wet', 'dry', 'invalid'

    def __init__(self, wet_below=4000, dry_above=4000, valid_min=2000,
                 debounce_s=5.0, clear_s=600.0, window=4):
        self.wet_below = int(wet_below)
        self.dry_above = int(dry_above)
        self.valid_min = int(valid_min)
        self.debounce_s = float(debounce_s)
        self.clear_s = float(clear_s)
        self._win = collections.deque(maxlen=max(1, int(window)))
        self.value = None            # filtered ADC value (int) or None before the 1st read
        self.triggered = False
        self._wet_since = None
        self._dry_since = None
        self._last_class = None

    def classify(self, value):
        """Polarity: dry is HIGH (pulled up), water pulls the ADC down."""
        if value < self.valid_min:
            return self.INVALID
        if value <= self.wet_below:
            return self.WET
        if value > self.dry_above:
            return self.DRY
        return self._last_class or self.DRY      # hysteresis band: keep previous class

    def update(self, raw, now):
        """Feed one raw sample at monotonic time ``now``. Returns ``triggered``."""
        self._win.append(int(raw))
        self.value = int(round(sum(self._win) / len(self._win)))
        cls = self.classify(self.value)
        self._last_class = cls if cls != self.INVALID else self._last_class
        if cls == self.WET:
            self._dry_since = None
            if self._wet_since is None:
                self._wet_since = now
            if not self.triggered and now - self._wet_since >= self.debounce_s:
                self.triggered = True
        else:                                     # DRY or INVALID (vendor: not rain)
            self._wet_since = None
            if self._dry_since is None:
                self._dry_since = now
            if self.triggered and now - self._dry_since >= self.clear_s:
                self.triggered = False
        return self.triggered
