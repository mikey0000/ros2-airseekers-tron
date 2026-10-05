"""Map-origin (GPS datum) helpers for ``/navsat_to_absolute_pose/set_datum``.

ROS-free so the reply format, the mowgli_robot.yaml splice and the datum.env
file can be unit-tested on the host.

Contract (``third_party/mowglinext/gui/web/src/utils/datumGps.ts``): the GUI
splits ``message`` on ``,``, expects exactly two parts and ``parseFloat``s each.
``parseFloat`` stops at the first non-numeric character, so a warning may follow
the longitude as long as it contains no comma.
"""

import math
import os
import re

# sensor_msgs/NavSatStatus.STATUS_NO_FIX
NAVSAT_NO_FIX = -1

# A datum that moves by more than this shifts the stored map-frame areas visibly.
DATUM_SHIFT_WARN_M = 1.0

_EARTH_RADIUS_M = 6371008.8
_NUM_CHARS = set('0123456789.-+eE')


def datum_is_set(lat, lon):
    """0.0/0.0 (or missing / non-finite) is the "unset" placeholder."""
    if lat is None or lon is None:
        return False
    try:
        lat, lon = float(lat), float(lon)
    except (TypeError, ValueError):
        return False
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    return not (lat == 0.0 and lon == 0.0)


def fix_usable(status, lat, lon, age_s, max_age_s):
    """Return ``(ok, reason)`` for a cached NavSatFix.

    ``status`` is ``NavSatFix.status.status`` (None = never received), ``age_s`` the
    seconds since it was received.
    """
    if status is None:
        return False, 'no GPS fix received yet'
    if age_s is None or age_s > max_age_s:
        return False, 'GPS fix is stale (%.1f s old)' % (age_s if age_s is not None else -1.0)
    if int(status) <= NAVSAT_NO_FIX:
        return False, 'GPS has no fix'
    if not datum_is_set(lat, lon) or abs(float(lat)) > 90.0 or abs(float(lon)) > 180.0:
        return False, 'GPS fix has no valid position'
    return True, ''


def distance_m(lat1, lon1, lat2, lon2):
    """Great-circle (haversine) distance in metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2.0 * _EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def format_reply(lat, lon, previous=None, warn_m=DATUM_SHIFT_WARN_M):
    """``"<lat>,<lon>"`` (9 decimals ~ 0.1 mm), plus a comma-free warning when a
    previous datum ``(lat, lon)`` exists and the new one is more than ``warn_m`` away."""
    msg = '%.9f,%.9f' % (lat, lon)
    if previous is not None and datum_is_set(*previous):
        d = distance_m(previous[0], previous[1], lat, lon)
        if d > warn_m:
            msg += (' WARNING: datum moved %.1f m from the previous %.9f/%.9f;'
                    ' areas and the dock pose stored in map coordinates now sit %.1f m'
                    ' off - re-record them or restore the old datum' % (
                        d, previous[0], previous[1], d))
    assert msg.count(',') == 1
    return msg


# ---------------------------------------------------------------------------
# mowgli_robot.yaml (in-place splice, comments and layout kept)
# ---------------------------------------------------------------------------
def splice_scalar(content, key, value):
    """Replace the numeric value of the first INDENTED ``key:`` line (same rule as
    mower_map's robot_yaml splice). Returns ``(content, replaced)``."""
    pos = 0
    while pos < len(content):
        nl = content.find('\n', pos)
        end = len(content) if nl < 0 else nl
        line = content[pos:end]
        stripped = line.lstrip(' \t')
        indent = len(line) - len(stripped)
        if indent > 0 and stripped.startswith(key + ':'):
            c = indent + len(key) + 1
            while c < len(line) and line[c] in ' \t':
                c += 1
            v = c
            while v < len(line) and line[v] in _NUM_CHARS:
                v += 1
            if v > c:
                return content[:pos] + line[:c] + value + line[v:] + content[end:], True
        if nl < 0:
            break
        pos = nl + 1
    return content, False


def splice_datum_yaml(content, lat, lon):
    """Set ``datum_lat``/``datum_lon`` in a mowgli_robot.yaml text. Missing keys are
    added under ``ros__parameters`` when that block exists. Returns ``(content, ok)``."""
    ok = True
    for key, val in (('datum_lat', lat), ('datum_lon', lon)):
        content, done = splice_scalar(content, key, '%.9f' % val)
        if not done:
            m = re.search(r'^([ \t]*)ros__parameters:[ \t]*\n', content, re.M)
            if m is None:
                ok = False
                continue
            indent = m.group(1) + '  '
            insert = '%s%s: %.9f\n' % (indent, key, val)
            content = content[:m.end()] + insert + content[m.end():]
    return content, ok


def read_yaml_datum(content):
    """``(lat, lon)`` from a mowgli_robot.yaml text, or None."""
    vals = {}
    for key in ('datum_lat', 'datum_lon'):
        m = re.search(r'^[ \t]+%s:[ \t]*([-+0-9.eE]+)' % key, content, re.M)
        if m:
            try:
                vals[key] = float(m.group(1))
            except ValueError:
                pass
    if len(vals) != 2:
        return None
    return vals['datum_lat'], vals['datum_lon']


# ---------------------------------------------------------------------------
# datum.env (read by launch/mower.launch.py as the datum_lat/datum_lon defaults)
# ---------------------------------------------------------------------------
def format_datum_env(lat, lon, stamp=''):
    head = '# Written by gui_bridge set_datum%s. Read by launch/mower.launch.py when\n' \
           '# datum_lat/datum_lon are not given on the command line.\n' % (
               (' at ' + stamp) if stamp else '')
    return head + 'DATUM_LAT=%.9f\nDATUM_LON=%.9f\n' % (lat, lon)


def parse_datum_env(content):
    """``(lat, lon)`` from a datum.env text (``KEY=value``, ``#`` comments, optional
    ``export`` and quotes), or None when either key is missing or not a number."""
    vals = {}
    for raw in content.splitlines():
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('export '):
            line = line[len('export '):].strip()
        key, sep, value = line.partition('=')
        if not sep:
            continue
        value = value.strip().strip('"\'')
        try:
            vals[key.strip()] = float(value)
        except ValueError:
            continue
    if 'DATUM_LAT' not in vals or 'DATUM_LON' not in vals:
        return None
    return vals['DATUM_LAT'], vals['DATUM_LON']


def write_text_atomic(path, text):
    """Write via a temp file + rename. Follows a symlink (colcon --symlink-install
    installs config/gui/mowgli_robot.yaml as one) so the real file is updated."""
    real = os.path.realpath(path)
    os.makedirs(os.path.dirname(real) or '.', exist_ok=True)
    tmp = real + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(text)
    try:
        os.chmod(tmp, os.stat(real).st_mode & 0o777)
    except OSError:
        pass
    os.replace(tmp, real)


def read_text(path):
    try:
        with open(path, encoding='utf-8') as f:
            return f.read()
    except OSError:
        return None
