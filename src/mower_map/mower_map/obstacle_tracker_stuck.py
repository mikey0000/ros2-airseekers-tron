# SPDX-License-Identifier: GPL-3.0-or-later
"""Repeated stuck spots -> pending no-go proposal (stock parity, 2026-11-03).

Stock marks trap spots it cannot leave (IotNotice Stuck 900049). Here every
STUCK_NEEDS_HELP entry and every mission stuck / slip escape incident (``/mission/incident``)
is filed with its map position in a small JSON file that survives restarts and sessions.
When ``hits`` events (default 2) lie within ``radius_m`` of each other, the obstacle tracker
proposes a PENDING no-go disc of that radius for the owner's Accept / Ignore in the GUI; the
events behind it are consumed so one spot is proposed once.

ROS-free; the node owns the file path and the clock (wall time, so sessions line up).
"""

import json
import math
import os
from typing import List, Optional, Tuple


class StuckSpotStore:
    def __init__(self, path: str = '', radius_m: float = 1.0, hits: int = 2,
                 min_gap_s: float = 60.0, max_age_days: float = 90.0, max_events: int = 200):
        self.path = path
        self.radius_m = float(radius_m)
        self.hits = int(hits)
        self.min_gap_s = float(min_gap_s)        # one incident logs several events: dedupe
        self.max_age_s = float(max_age_days) * 86400.0
        self.max_events = int(max_events)
        self.events: List[dict] = []
        self.load()

    # -- persistence ----------------------------------------------------------
    def load(self) -> None:
        if not self.path:
            return
        try:
            with open(self.path) as f:
                data = json.load(f)
            self.events = [{'x': float(e['x']), 'y': float(e['y']), 't': float(e['t']),
                            'kind': str(e.get('kind', ''))} for e in data.get('events', [])]
        except (OSError, ValueError, KeyError, TypeError, AttributeError):
            self.events = []

    def save(self) -> None:
        if not self.path:
            return
        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump({'version': 1, 'events': self.events}, f, indent=1)
        os.replace(tmp, self.path)

    # -- events ---------------------------------------------------------------
    def record(self, x: float, y: float, wall_t: float, kind: str) -> Optional[Tuple[float, float]]:
        """File one event. Returns the proposal centre (mean of the clustered events) when
        this spot now has ``hits`` events within ``radius_m``, else None."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        self.events = [e for e in self.events if wall_t - e['t'] <= self.max_age_s]
        near = [e for e in self.events
                if math.hypot(e['x'] - x, e['y'] - y) <= self.radius_m]
        if any(abs(wall_t - e['t']) < self.min_gap_s for e in near):
            return None                  # the same incident reported twice
        self.events.append({'x': float(x), 'y': float(y), 't': float(wall_t), 'kind': kind})
        self.events = self.events[-self.max_events:]
        near.append(self.events[-1])
        out = None
        if self.hits > 0 and len(near) >= self.hits:
            out = (sum(e['x'] for e in near) / len(near), sum(e['y'] for e in near) / len(near))
            self.events = [e for e in self.events if e not in near]
        self.save()
        return out


def stuck_event_spot(x: float, y: float, now: float, last_incident=None, last_contact=None,
                     skip_kinds=('boundary',), skip_window_s: float = 30.0,
                     contact_window_s: float = 10.0, contact_radius_m: float = 1.0):
    """Where a STUCK_NEEDS_HELP entry at robot pose (x, y) is filed, or None to not file it.

    Review 2026-11-11 (field: a pending 1.0 m keepout 0.11 m inside the line, placed from two
    boundary spot-rule stops whose bumper contact had been ~0.2 m OUTSIDE it): a stop that
    follows a ``skip_kinds`` incident (a boundary excursion: no obstacle) within
    ``skip_window_s`` is no trap spot; one that follows a bumper contact within
    ``contact_window_s`` and ``contact_radius_m`` is filed at the contact point, not at
    base_link. ``last_incident`` = (kind, t); ``last_contact`` = (t, cx, cy)."""
    if last_incident is not None and last_incident[0] in tuple(skip_kinds or ()) and \
            now - float(last_incident[1]) <= float(skip_window_s):
        contact = last_contact is not None and now - float(last_contact[0]) <= \
            float(contact_window_s)
        if not contact:
            return None
    if last_contact is not None and now - float(last_contact[0]) <= float(contact_window_s) \
            and math.hypot(float(last_contact[1]) - x, float(last_contact[2]) - y) <= \
            float(contact_radius_m):
        return float(last_contact[1]), float(last_contact[2])
    return x, y
