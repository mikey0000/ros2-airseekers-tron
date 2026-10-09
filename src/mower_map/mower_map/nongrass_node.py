# SPDX-License-Identifier: GPL-3.0-or-later
"""map_server_node glue for the non-grass memory (mower_map.nongrass; 2026-10-09).

A mixin so the map server only gains a few hook calls (init, area changes, terrain_action).

In:  ``nongrass_topic`` (/ai/seg/ground_cells, PointCloud2 in the map frame from
     seg_ros/nongrass_projector). Dropped while charging or lifted.
Out (latched, on the keepout-mask grid like the terrain layers):
     ``~/nongrass_cost``     0..nongrass_cost_max, Nav2 global costmap ``nongrass_layer``
                             (StaticLayer, non-trinary, use_maximum). ALWAYS published (zeros
                             when nothing is confirmed or the memory is off): the Jazzy
                             StaticLayer has no costs until its first map.
     ``~/nongrass_grid``     non-grass probability 0..100, -1 never seen (GUI heat map)
     ``~/nongrass_summary``  JSON {version, stamp, areas: [{area, area_index, enabled,
                             confirmed_cells, confirmed_m2, observed_cells, clusters: [{id,
                             cells, area_m2, x, y, suggest_keepout, hull}]}]}
Actions (``~/terrain_action``, settings_json): ``{"action": "nongrass_keepout" |
"nongrass_dismiss", "cluster_id": n}`` or ``{"action": "nongrass_clear"}``.
Per-area switch: area setting ``avoid_non_grass`` (default true) gates the COST only; the
memory keeps learning so turning it on later is immediate.
"""

import json
import time

from mower_map import area_settings as aset
from mower_map import areas as core
from mower_map import nongrass as ngm
from mower_map import terrain as terr

PARAMS = {
    'nongrass_enabled': True,
    'nongrass_topic': '/ai/seg/ground_cells',
    'nongrass_half_life_days': 14.0,
    'nongrass_confirm_hits': 3.0,       # weighted frames
    'nongrass_confirm_ratio': 0.75,     # hit / (hit + miss)
    'nongrass_confirm_spread_m': 0.5,   # seen as non-grass from viewpoints this far apart
    'nongrass_release_hits': 1.5,
    'nongrass_release_ratio': 0.5,
    'nongrass_cost_max': 75,            # -> costmap 190 (> area_transit_cost 60 -> 151,
                                        # < soft band 90 -> 229): a preference, never lethal
    'nongrass_halo_m': 0.2,
    'nongrass_min_cluster_m2': 0.25,    # smallest keep-out suggestion
    'nongrass_keepout_margin_m': 0.1,
    'nongrass_dismiss_weight': 20.0,
    'nongrass_publish_period_s': 5.0,
    'nongrass_save_period_s': 120.0,
}
SCHEMA_VERSION = 1


class NonGrassMixin:
    """Expects the MapServerNode attributes: p(), store, settings, spec, grid_msg(),
    status, map_frame, terrain_dir(), get_logger(), rebuild(), persist_best_effort()."""

    # ------------------------------------------------------------------ setup
    def nongrass_init(self, sub, lock, latched_qos):
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        self.nongrass = {}                   # area name -> ngm.AreaNonGrass
        self.nongrass_dirty = True
        self._nongrass_stats = {'clouds': 0, 'cells': 0, 'dropped': 0}
        self.nongrass_cost_pub = self.create_publisher(OccupancyGrid, '~/nongrass_cost',
                                                       latched_qos)
        self.nongrass_grid_pub = self.create_publisher(OccupancyGrid, '~/nongrass_grid',
                                                       latched_qos)
        self.nongrass_summary_pub = self.create_publisher(String, '~/nongrass_summary',
                                                          latched_qos)
        if self.p('nongrass_enabled'):
            sub(PointCloud2, self.p('nongrass_topic'), self.on_nongrass_cloud, 5, lock=lock)

    def nongrass_start(self, locked):
        """After load_areas: load the memory, publish once, start the timers."""
        if self.p('nongrass_enabled'):
            self.nongrass_sync_areas()
        self.publish_nongrass()
        if self.p('nongrass_enabled'):
            self.create_timer(max(1.0, float(self.p('nongrass_publish_period_s'))),
                              locked(self.on_nongrass_publish_timer))
            self.create_timer(max(5.0, float(self.p('nongrass_save_period_s'))),
                              locked(self.on_nongrass_save_timer))

    def nongrass_params(self):
        keys = ('half_life_days', 'confirm_hits', 'confirm_ratio', 'confirm_spread_m',
                'release_hits', 'release_ratio', 'cost_max', 'halo_m', 'min_cluster_m2',
                'keepout_margin_m', 'dismiss_weight')
        out = {k: self.p('nongrass_' + k) for k in keys}
        out['resolution'] = self.resolution
        return out

    # ------------------------------------------------------------------ areas
    def nongrass_sync_areas(self):
        """(Re)load the memory of every mowing area whose name or polygon changed."""
        if not self.p('nongrass_enabled') or getattr(self, 'nongrass', None) is None:
            return
        names = {a.name: a for a in self.store.areas if not a.is_navigation}
        d = self.terrain_dir()
        for name in list(self.nongrass):
            if name not in names:
                self._nongrass_save(self.nongrass.pop(name))
        now = time.time()
        for name, area in names.items():
            cur = self.nongrass.get(name)
            if cur is not None and cur.polygon == [tuple(p) for p in area.polygon]:
                continue
            if cur is not None:
                self._nongrass_save(cur)
            m = ngm.AreaNonGrass.load(d, name, area.polygon, self.nongrass_params(), now)
            if getattr(m, 'load_error', None):
                self.get_logger().warn('nongrass %s: unreadable (%s), starting empty'
                                       % (name, m.load_error))
            m.decay(now)
            m.update_confirmed()
            self.nongrass[name] = m
        self.nongrass_dirty = True

    def _nongrass_save(self, m):
        try:
            m.save(self.terrain_dir())
        except OSError as exc:
            self.get_logger().warn('nongrass %s: save failed: %s' % (m.name, exc),
                                   throttle_duration_sec=60.0)

    def nongrass_area_enabled(self, name):
        return bool(self.settings.effective(name).get('avoid_non_grass', True))

    # ------------------------------------------------------------------ input
    def on_nongrass_cloud(self, msg):
        st = self.status
        if st is not None and (getattr(st, 'is_charging', False) or
                               getattr(st, 'lift_triggered', False)):
            self._nongrass_stats['dropped'] += 1
            return
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn('nongrass: cloud in %r, expected %r: dropped'
                                   % (msg.header.frame_id, self.map_frame),
                                   throttle_duration_sec=60.0)
            return
        f = ngm.cloud_fields(msg.fields, msg.data, int(msg.point_step),
                             int(msg.width) * max(1, int(msg.height)))
        if f is None:
            self.get_logger().warn('nongrass: cloud without float32 x y nongrass weight vx vy',
                                   throttle_duration_sec=60.0)
            return
        self.nongrass_observe(f['x'], f['y'], f['nongrass'] > 0.5, f['weight'], f['vx'],
                              f['vy'])

    def nongrass_observe(self, xs, ys, nongrass, samples, vx, vy):
        """Route one frame's cell votes to the area rasters (first raster that holds a
        point wins; a point in no mowing area is ignored)."""
        import numpy as np
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        left = np.ones(xs.shape, bool)
        used = 0
        for m in self.nongrass.values():
            if not left.any():
                break
            _r, _c, ok = m.raster.cells(xs, ys)
            sel = ok & left
            if not sel.any():
                continue
            used += m.observe(xs[sel], ys[sel], np.asarray(nongrass)[sel],
                              np.asarray(samples)[sel], np.asarray(vx)[sel],
                              np.asarray(vy)[sel])
            left &= ~sel
        self._nongrass_stats['clouds'] += 1
        self._nongrass_stats['cells'] += used
        return used

    # ------------------------------------------------------------------ output
    def on_nongrass_publish_timer(self):
        now = time.time()
        changed = False
        for m in self.nongrass.values():
            m.decay(now)
            changed |= m.update_confirmed()
        if changed or self.nongrass_dirty:
            self.publish_nongrass()

    def on_nongrass_save_timer(self):
        for m in self.nongrass.values():
            if m.dirty:
                self._nongrass_save(m)

    def nongrass_summary(self):
        out = {'version': SCHEMA_VERSION, 'stamp': time.time(), 'areas': [],
               'stats': dict(getattr(self, '_nongrass_stats', {}))}
        for idx, a in enumerate(self.store.areas):
            m = self.nongrass.get(a.name) if getattr(self, 'nongrass', None) else None
            if a.is_navigation or m is None:
                continue
            s = m.summary(self.nongrass_area_enabled(a.name))
            s['area_index'] = idx
            out['areas'].append(s)
        return out

    def publish_nongrass(self):
        import numpy as np
        from std_msgs.msg import String
        spec = self.spec
        cost = np.zeros((spec.height, spec.width), np.int16)
        grid = np.full((spec.height, spec.width), -1, np.int16)
        for name, m in (getattr(self, 'nongrass', None) or {}).items():
            terr.stamp_into(cost, (spec.origin_x, spec.origin_y), spec.resolution, m.raster,
                            m.cost(self.nongrass_area_enabled(name)))
            terr.stamp_into(grid, (spec.origin_x, spec.origin_y), spec.resolution, m.raster,
                            m.grid())
        self.nongrass_cost_pub.publish(self.grid_msg(spec, cost))
        self.nongrass_grid_pub.publish(self.grid_msg(spec, grid))
        self.nongrass_summary_pub.publish(String(data=json.dumps(self.nongrass_summary(),
                                                                sort_keys=True)))
        self.nongrass_dirty = False

    # ------------------------------------------------------------------ actions
    def nongrass_action(self, action, cluster_id, area_index):
        """Returns (success, message)."""
        areas = [(i, a) for i, a in enumerate(self.store.areas)
                 if not a.is_navigation and a.name in (getattr(self, 'nongrass', None) or {})
                 and (area_index == aset.DEFAULTS_INDEX or i == area_index)]
        if not areas:
            return False, 'no non-grass memory for area %d' % area_index
        if action == 'nongrass_clear':
            for _, a in areas:
                self.nongrass[a.name].clear()
                self._nongrass_save(self.nongrass[a.name])
            self.publish_nongrass()
            return True, 'non-grass memory cleared'
        if action not in ('nongrass_keepout', 'nongrass_dismiss') or \
                not isinstance(cluster_id, int) or isinstance(cluster_id, bool):
            return False, ('action must be nongrass_keepout|nongrass_dismiss with an int '
                           'cluster_id, or nongrass_clear')
        for i, a in areas:
            m = self.nongrass[a.name]
            cl = next((c for c in m.clusters() if c['id'] == cluster_id), None)
            if cl is None:
                continue
            if action == 'nongrass_keepout':
                ok, msg = self.store.add_obstacle(
                    i, [tuple(p) for p in cl['hull']],
                    'non-grass #%d (%.1f m2)' % (cluster_id, cl['area_m2']), core.SOURCE_DIG)
                if not ok:
                    return False, 'keep-out rejected: ' + msg
                self.rebuild()
                self.persist_best_effort('non-grass keepout')
                msg = 'non-grass cluster %d -> keep-out in area %d' % (cluster_id, i)
            else:
                m.dismiss(cluster_id)
                msg = 'non-grass cluster %d dismissed (lawn)' % cluster_id
            self._nongrass_save(m)
            self.publish_nongrass()
            self.get_logger().info('nongrass: ' + msg)
            return True, msg
        return False, 'no non-grass cluster %s' % cluster_id
