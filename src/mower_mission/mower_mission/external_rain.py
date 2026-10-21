"""External rain verdict (roof weather station / Home Assistant over MQTT).

Upstream MowgliNext contract (59d59e80, docs/MQTT_CONTROL.md): an outside
source publishes yes/no, retained, to ``<mqtt_topic_prefix>/external_rain``.
On our stack the MQTT broker is the GUI backend's embedded one: its MQTT
provider parses the payload with the same word list and calls
``/behavior_tree_node/set_external_rain`` (std_srvs/SetBool). The mission also
takes upstream's ROS topic ``/external_rain`` (std_msgs/Bool, transient_local).

The verdict is OR-ed with the mower's own rain sensors, so it can only add a
reason to stop, never hide rain the mower feels. A wet verdict expires after
``max_age_s`` (0 = never) so a dead automation or broker cannot hold the
mower in RAIN_WAITING forever.
"""

WET_WORDS = frozenset(('yes', 'on', 'true', 'rain', 'wet', '1'))
DRY_WORDS = frozenset(('no', 'off', 'false', 'dry', '0'))


def parse_rain_payload(payload):
    """'yes'/'no' (and on/off, true/false, wet/dry, rain, 1/0; any case, surrounding
    whitespace ignored) -> True / False. Anything else -> None (ignore it)."""
    if isinstance(payload, (bytes, bytearray)):
        try:
            payload = payload.decode('utf-8')
        except UnicodeDecodeError:
            return None
    if not isinstance(payload, str):
        return None
    word = payload.strip().lower()
    if word in WET_WORDS:
        return True
    if word in DRY_WORDS:
        return False
    return None


class ExternalRain:
    """Last external verdict with its arrival time (monotonic seconds)."""

    def __init__(self, enabled=True, max_age_s=7200.0):
        self.enabled = bool(enabled)
        self.max_age_s = max(0.0, float(max_age_s))
        self.value = None       # None = never heard, True = wet, False = dry
        self.stamp = None

    def update(self, wet, now):
        self.value = bool(wet)
        self.stamp = float(now)

    def state(self, now):
        """'disabled' | 'no_data' | 'dry' | 'wet' | 'expired'."""
        if not self.enabled:
            return 'disabled'
        if self.value is None:
            return 'no_data'
        if not self.value:
            return 'dry'
        if self.max_age_s > 0.0 and now - self.stamp > self.max_age_s:
            return 'expired'
        return 'wet'

    def wet(self, now):
        """True only for a fresh wet verdict with the feature enabled."""
        return self.state(now) == 'wet'
