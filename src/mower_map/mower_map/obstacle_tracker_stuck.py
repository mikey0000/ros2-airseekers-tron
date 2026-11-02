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
