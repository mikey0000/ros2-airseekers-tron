# SPDX-License-Identifier: GPL-3.0-or-later
"""Per-area mowing settings: validation, merge and ``area_settings.yaml`` (no ROS).

Settings are keyed by AREA NAME (areas.dat has no stable area id) and stored
next to ``areas.dat`` in ``maps_dir``::

    # mower_map per-area mowing settings
    version: 1
    defaults:              # overrides of the built-in defaults, every area
      cutter_height_mm: 55
    areas:
      Front lawn:          # only the keys set for this area
        path_mode: spiral
        perimeter_laps: 3

Every key is optional; a missing key falls back to ``defaults``, then to the
built-in :data:`BUILTIN_DEFAULTS`. The effective settings of an area are the
three layers merged (:meth:`AreaSettingsStore.effective`).

Vendor mapping (Tron TaskUnit): ``path_mode`` 0 zigzag, 1 cross, 2 alternate
zigzag, 3 spiral; ``contour_only`` = vendor cut_mode 2; ``perimeter_laps`` =
headland rings; cutter height range 30-90 mm.
"""

import copy
import json
import math
import os

import yaml

DEFAULTS_INDEX = 255
PATH_MODES = ('zigzag', 'cross', 'alternate', 'spiral', 'contour_only')
# Swath visiting order (Fields2Cover route planners): boustrophedon = neighbour
# after neighbour (hairpin turns), snake = skip-row 0,2,4,..,5,3,1 (turns two
# swaths wide), spiral = F2C SpiralOrder over groups of route_spiral_size swaths.
# racetrack = tractor "lands": blocks 0,k,1,k+1,.. with rows >= 2 x min_turn_radius_m
# apart, so every in-block turn is a plain forward U-turn.
ROUTE_ORDERS = ('boustrophedon', 'snake', 'spiral', 'racetrack')
# Swath-to-swath turn: auto = forward loop turn, else reverse-then-curve, else
# pivot; loop = loop turn or pivot; reverse = reverse-curve or pivot; pivot =
# pivot in place (straight connector, the pre-turn behaviour).
TURN_TYPES = ('auto', 'loop', 'reverse', 'pivot')

BUILTIN_DEFAULTS = {
    'cutter_height_mm': 50,
    'perimeter_laps': 2,
    'path_mode': 'zigzag',
    'mow_angle_deg': -1.0,
    'cut_speed_mps': 0.3,
    'swath_overlap_m': 0.02,
    'edge_first': True,
    'repeat': 1,
    'alternate_angle_offset_deg': 90.0,
    # Tron defaults (owner, 2026-10-06): pivots tear the lawn when the caster
    # catches, so race-track (skip-row) order with 0.5 m smooth turns, pivot last resort.
    'route_order': 'racetrack',
    'route_spiral_size': 6,
    'min_turn_radius_m': 0.5,
    'turn_type': 'auto',
}

# key -> (kind, min, max). kind: int | float | bool | enum
SPEC = {
    'cutter_height_mm': ('int', 30, 90),
    'perimeter_laps': ('int', 0, 4),
    'path_mode': ('enum', PATH_MODES, None),
    'mow_angle_deg': ('angle', None, None),        # < 0 -> -1 (auto), else folded to [0, 180)
    'cut_speed_mps': ('float', 0.05, 0.5),
    'swath_overlap_m': ('float', 0.0, 0.1),
    'edge_first': ('bool', None, None),
    'repeat': ('int', 1, 10),
    'alternate_angle_offset_deg': ('float', 0.0, 180.0),
    'route_order': ('enum', ROUTE_ORDERS, None),
    'route_spiral_size': ('int', 2, 20),
    'min_turn_radius_m': ('float', 0.0, 2.0),      # 0 = pivot in place
    'turn_type': ('enum', TURN_TYPES, None),
}

FILE_NAME = 'area_settings.yaml'


def settings_path_for(areas_path):
    return os.path.join(os.path.dirname(os.path.abspath(areas_path)), FILE_NAME)


def validate_value(key, value):
    """Returns (ok, normalised value or error message)."""
    if key not in SPEC:
        return False, 'unknown key %r (known: %s)' % (key, ', '.join(sorted(SPEC)))
    kind, lo, hi = SPEC[key]
    if kind == 'bool':
        if isinstance(value, bool):
            return True, value
        return False, '%s must be true or false' % key
    if kind == 'enum':
        if isinstance(value, str) and value in lo:
            return True, value
        return False, '%s must be one of %s' % (key, ', '.join(lo))
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False, '%s must be a number' % key
    if isinstance(value, float) and not math.isfinite(value):
        return False, '%s must be finite' % key
    if kind == 'int':
        if isinstance(value, float):
            if value != int(value):
                return False, '%s must be an integer' % key
            value = int(value)
        if not lo <= value <= hi:
            return False, '%s must be in %d..%d' % (key, lo, hi)
        return True, int(value)
    if kind == 'angle':
        v = float(value)
        if v < 0.0:
            return True, -1.0
        return True, math.fmod(v, 180.0)
    v = float(value)
    if not lo - 1e-9 <= v <= hi + 1e-9:
        return False, '%s must be in %g..%g' % (key, lo, hi)
    return True, v


def validate(settings):
    """Validate a settings object. ``None`` values are kept (they mean "remove").

    Returns (ok, message, cleaned dict).
    """
    if not isinstance(settings, dict):
        return False, 'settings must be a JSON object', {}
    out = {}
    for k, v in settings.items():
        if v is None:
            if k not in SPEC:
                return False, 'unknown key %r' % k, {}
            out[k] = None
            continue
        ok, val = validate_value(k, v)
        if not ok:
            return False, val, {}
        out[k] = val
    return True, 'ok', out


def parse_json(text):
    """settings_json -> (ok, message, cleaned dict). Empty text = ``{}``."""
    try:
        obj = json.loads(text) if (text or '').strip() else {}
    except ValueError as exc:
        return False, 'invalid JSON: %s' % exc, {}
    return validate(obj)


def _merge_update(stored, update):
    """Merge ``update`` into ``stored``: null removes a key; ``{}`` clears all."""
    if not update:
        return {}
    out = dict(stored)
    for k, v in update.items():
        if v is None:
            out.pop(k, None)
        else:
            out[k] = v
    return out


def polygons_match(a, b, tol=0.01):
    """Same vertex list within ``tol`` (rename detection after clear + re-add)."""
    if len(a) != len(b) or not a:
        return False
    return all(math.hypot(p[0] - q[0], p[1] - q[1]) <= tol for p, q in zip(a, b))


class AreaSettingsStore:
    """Stored per-area settings plus the default overrides."""

    def __init__(self):
        self.defaults = {}      # overrides of BUILTIN_DEFAULTS
        self.areas = {}         # name -> overrides

    # ---- queries ------------------------------------------------------
    def effective_defaults(self):
        out = dict(BUILTIN_DEFAULTS)
        out.update(self.defaults)
        return out

    def effective(self, name):
        out = self.effective_defaults()
        out.update(self.areas.get(name, {}))
        return out

    def snapshot(self, area_names=()):
        """The latched ``~/area_settings`` payload: full defaults, the stored
        (sparse) values per area. Every name in ``area_names`` is listed, with
        ``{}`` when it has no values of its own."""
        areas = {n: {} for n in area_names}
        for n, v in self.areas.items():
            areas[n] = dict(v)
        return {'defaults': self.effective_defaults(), 'areas': areas}

    def snapshot_json(self, area_names=()):
        return json.dumps(self.snapshot(area_names), sort_keys=True)

    # ---- updates ------------------------------------------------------
    def set_defaults(self, update):
        self.defaults = _merge_update(self.defaults, update)

    def set_area(self, name, update):
        merged = _merge_update(self.areas.get(name, {}), update)
        if merged:
            self.areas[name] = merged
        else:
            self.areas.pop(name, None)

    def rename(self, old, new):
        if old == new or old not in self.areas:
            return False
        self.areas[new] = self.areas.pop(old)
        return True

    def drop(self, name):
        return self.areas.pop(name, None) is not None

    def prune(self, names):
        """Drop every entry whose area no longer exists. Returns dropped names."""
        keep = set(names)
        gone = [n for n in self.areas if n not in keep]
        for n in gone:
            del self.areas[n]
        return gone

    def carry_over(self, before, after):
        """Move settings across a replace of the area list.

        ``before`` / ``after`` are lists of (name, polygon). An entry whose
        name vanished moves to a NEW name with the same polygon (a rename).
        Returns [(old, new)].
        """
        after_names = {n for n, _ in after}
        before_names = {n for n, _ in before}
        moved = []
        for old, old_poly in before:
            if old in after_names or old not in self.areas:
                continue
            for new, new_poly in after:
                if new in before_names or new in self.areas:
                    continue
                if polygons_match(old_poly, new_poly):
                    self.rename(old, new)
                    moved.append((old, new))
                    break
        return moved

    # ---- persistence --------------------------------------------------
    def dumps(self):
        doc = {'version': 1, 'defaults': copy.deepcopy(self.defaults),
               'areas': copy.deepcopy(self.areas)}
        return '# mower_map per-area mowing settings (see mower_map/README.md)\n' + \
            yaml.safe_dump(doc, sort_keys=True, allow_unicode=True)

    @classmethod
    def loads(cls, text):
        """Parse; invalid entries are dropped. Returns (store, [warnings])."""
        store, warnings = cls(), []
        try:
            doc = yaml.safe_load(text) or {}
        except yaml.YAMLError as exc:
            return store, ['unreadable: %s' % exc]
        if not isinstance(doc, dict):
            return store, ['not a mapping']
        ok, msg, d = validate(doc.get('defaults') or {})
        if ok:
            store.defaults = {k: v for k, v in d.items() if v is not None}
        else:
            warnings.append('defaults dropped: %s' % msg)
        areas = doc.get('areas') or {}
        if isinstance(areas, dict):
            for name, vals in areas.items():
                ok, msg, d = validate(vals or {})
                if ok:
                    d = {k: v for k, v in d.items() if v is not None}
                    if d:
                        store.areas[str(name)] = d
                else:
                    warnings.append('area %r dropped: %s' % (name, msg))
        return store, warnings


def save_file(path, store):
    tmp = path + '.tmp'
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
    with open(tmp, 'w', encoding='utf-8') as fh:
        fh.write(store.dumps())
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def load_file(path):
    """Returns (store, warnings); a missing file is an empty store."""
    if not os.path.exists(path):
        return AreaSettingsStore(), []
    with open(path, 'r', encoding='utf-8') as fh:
        return AreaSettingsStore.loads(fh.read())
