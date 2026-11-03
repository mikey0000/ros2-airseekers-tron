#!/usr/bin/env python3
"""Write a gate run's world: the chosen world plus ``--rtk-loss`` events (sim_gate.sh).

usage: run_world.py <world.yaml> <out.yaml> <container dir of world.yaml> [t0:dur:kind ...]

The copy lives in the run directory, so a relative ``areas_file`` is rewritten to the
container path of the original. Prints ``rtk=1`` when the run world injects RTK loss
(events or multipath) so the gate turns on loc_drift and the localization monitor."""
import os
import sys

import yaml


def main(src, out, cdir, specs):
    with open(src, encoding='utf-8') as fh:
        doc = yaml.safe_load(fh) or {}
    w = doc['world'] if isinstance(doc.get('world'), dict) else doc
    f = w.get('areas_file')
    if f and not os.path.isabs(f):
        w['areas_file'] = os.path.join(cdir, f)
    if specs:
        rtk = dict(w.get('rtk') or {})
        ev = list(rtk.get('events') or [])
        for s in specs:
            p = s.split(':')
            if len(p) not in (2, 3) or (len(p) == 3 and p[2] not in ('float', 'dropout')):
                sys.exit('sim_gate: --rtk-loss %r: want t0:dur[:float|dropout]' % s)
            ev.append({'t0': float(p[0]), 'dur': float(p[1]),
                       'kind': p[2] if len(p) == 3 else 'dropout'})
        rtk['events'] = ev
        w['rtk'] = rtk
    w.setdefault('name', os.path.splitext(os.path.basename(src))[0])
    with open(out, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(doc, fh, sort_keys=False)
    rtk = w.get('rtk') or {}
    print('rtk=%d' % int(bool(rtk.get('events') or rtk.get('multipath'))))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:])
