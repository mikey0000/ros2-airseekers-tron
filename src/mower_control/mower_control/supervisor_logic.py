# SPDX-License-Identifier: GPL-3.0-or-later
"""Node liveness bookkeeping for the supervisor (pure Python, no ROS).

The supervisor polls the ROS graph (``get_node_names_and_namespaces``) at 1 Hz and feeds the
set of fully qualified names to :meth:`LivenessTracker.update`:

* a node is *adopted* once it has been present for ``adopt_after_s`` (short-lived CLI or
  probe nodes never are); ``critical`` nodes are adopted at once;
* an adopted node absent for ``down_after_s`` is DOWN (graph discovery blips shorter than
  that are ignored); it comes back UP when it reappears (launch ``respawn``), and the
  episode counts as a restart;
* a ``critical`` node never seen within ``startup_grace_s`` of the supervisor start is DOWN
  ("never started");
* names matching an ``ignore`` glob are not tracked.
"""

import fnmatch

DEFAULT_IGNORE = ('/_ros2cli_*', '/launch_ros_*', '*/transform_listener_impl_*',
                  '/_NODE_NAME_UNKNOWN_*', '*/_*')


class LivenessTracker:
    def __init__(self, critical=(), ignore=DEFAULT_IGNORE, adopt_after_s=10.0,
                 down_after_s=2.5, startup_grace_s=90.0, now=0.0):
        self.critical = [self._fq(n) for n in critical]
        self.ignore = list(ignore)
        self.adopt_after_s = float(adopt_after_s)
        self.down_after_s = float(down_after_s)
        self.startup_grace_s = float(startup_grace_s)
        self.start = float(now)
        self.nodes = {}       # name -> dict(first, last, adopted, down_since, restarts)
        for name in self.critical:
            self.nodes[name] = self._new(None)
            self.nodes[name]['adopted'] = True

    @staticmethod
    def _fq(name):
        return name if name.startswith('/') else '/' + name

    @staticmethod
    def _new(now):
        return {'first': now, 'last': now, 'adopted': False, 'down_since': None,
                'restarts': 0, 'reason': ''}

    def _ignored(self, name):
        return any(fnmatch.fnmatch(name, p) for p in self.ignore)

    def update(self, now, present):
        """Returns a list of events ``('down', name, reason)`` / ``('up', name, downtime_s)``."""
        events = []
        present = {self._fq(n) for n in present if not self._ignored(self._fq(n))}
        for name in present:
            st = self.nodes.get(name)
            if st is None:
                st = self.nodes[name] = self._new(now)
            if st['first'] is None:
                st['first'] = now
            if st['down_since'] is not None:
                events.append(('up', name, now - st['down_since']))
                st['down_since'] = None
                st['restarts'] += 1
                st['first'] = now
                st['reason'] = ''
            st['last'] = now
            if not st['adopted'] and now - st['first'] >= self.adopt_after_s:
                st['adopted'] = True
        for name, st in self.nodes.items():
            if name in present or st['down_since'] is not None:
                continue
            if st['last'] is None:
                if name in self.critical and now - self.start >= self.startup_grace_s:
                    st['down_since'] = now
                    st['reason'] = 'never started'
                    events.append(('down', name, st['reason']))
                continue
            if st['adopted'] and now - st['last'] >= self.down_after_s:
                st['down_since'] = st['last']
                st['reason'] = 'vanished from the ROS graph'
                events.append(('down', name, st['reason']))
        # forget never-adopted transients
        for name in [n for n, st in self.nodes.items()
                     if not st['adopted'] and n not in present]:
            del self.nodes[name]
        return events

    def down(self):
        return sorted(n for n, st in self.nodes.items() if st['down_since'] is not None)

    def critical_down(self):
        """Critical nodes that were running and vanished (what the mission reacts to).
        A critical node that never started (its launch group disabled, e.g. teleop:=false)
        is reported in ``down`` only, so a reduced bench launch does not latch EMERGENCY."""
        return [n for n in self.down()
                if n in self.critical and self.nodes[n]['reason'] != 'never started']

    def status(self, now):
        nodes = {}
        for name, st in sorted(self.nodes.items()):
            if not st['adopted']:
                continue
            nodes[name] = {
                'alive': st['down_since'] is None and st['last'] is not None,
                'restarts': st['restarts'],
                'down_for_s': None if st['down_since'] is None
                else round(now - st['down_since'], 1),
                'reason': st['reason'],
                'critical': name in self.critical,
            }
        down = self.down()
        return {'ok': not down, 'down': down, 'critical_down': self.critical_down(),
                'restarts': {n: v['restarts'] for n, v in nodes.items() if v['restarts']},
                'nodes': nodes}
