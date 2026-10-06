"""GUI settings -> node parameters at launch time.

The MowgliNext GUI saves its settings to ``config/gui/mowgli_robot.yaml``
(sparse: only keys the operator changed). For the Airseekers Tron profile the
GUI also pushes the *bound* keys live to the running nodes
(third_party/mowglinext/gui/pkg/api/param_bindings.go); this module applies the
same keys at boot, so a value saved in the GUI survives a restart. A key that
is missing from the yaml leaves the package default untouched.

Mechanism, per target node:

* nodes whose launch file takes a ``params_file`` (mission, docking, Nav2):
  the package yaml is loaded, the bound values are deep-merged into it and the
  result is written to a temporary file that replaces ``params_file``;
* ``um960_gps_driver``: an overlay yaml with only the bound values
  (bringup.launch.py ``um960_params_file``);
* the coverage planner: ``tool_width`` becomes the ``cut_width_m`` launch
  argument (swath spacing = cut width - ``swath_overlap_m``);
* ``cmd_vel_slew`` (straight-line trim / deadband / heading hold): the node
  reads these keys from the same yaml itself at startup (its
  ``robot_settings_file`` parameter), so mower.launch.py has nothing to pass
  and skips the target (SELF_LOADED_TARGETS).

Values are coerced to the type the node declared (the GUI writes 30.0 as
``30``; rclpy refuses an int for a double parameter).

Keep BINDINGS in sync with the AirseekersTron table in param_bindings.go
(test/test_robot_settings.py checks it).
"""

import copy
import os
import tempfile

import yaml

# GUI key -> (target, parameter path, type, value map or None)
#   target: a node name (its ros__parameters section), or 'launch' for a
#   launch argument of mower.launch.py. Dotted paths are nested yaml keys
#   (Nav2 plugin parameters).
BINDINGS = {
    'tool_width': ('launch', 'cut_width_m', float, None),
    'mowing_speed': ('controller_server', 'FollowCoveragePath.desired_linear_vel', float, None),
    'transit_speed': ('controller_server', 'FollowPath.desired_linear_vel', float, None),
    'undock_distance': ('behavior_tree_node', 'undock_distance_m', float, None),
    'undock_speed': ('behavior_tree_node', 'undock_speed_mps', float, None),
    'battery_low_percent': ('behavior_tree_node', 'battery_low_percent', float, None),
    'battery_full_percent': ('behavior_tree_node', 'battery_full_percent', float, None),
    'battery_low_action': ('behavior_tree_node', 'battery_low_action', str, None),
    'battery_max_charge_percent': ('behavior_tree_node', 'battery_max_charge_percent', float, None),
    # GUI 0 ignore / 1 dock / 2 dock until dry / 3 pause -> Tron 0 ignore / 1 dock and wait.
    'rain_mode': ('behavior_tree_node', 'rain_mode', int, {0: 0, 1: 1, 2: 1, 3: 1}),
    'rain_delay_minutes': ('behavior_tree_node', 'rain_delay_minutes', float, None),
    'rain_debounce_sec': ('behavior_tree_node', 'rain_debounce_s', float, None),
    'mow_angle_deg': ('behavior_tree_node', 'mow_angle_deg', float, None),
    'dock_approach_distance': ('mower_docking', 'approach_distance', float, None),
    'dock_max_retries': ('mower_docking', 'max_retries', int, None),
    'ntrip_host': ('um960_gps_driver', 'ntrip_host', str, None),
    'ntrip_port': ('um960_gps_driver', 'ntrip_port', int, None),
    'ntrip_mountpoint': ('um960_gps_driver', 'ntrip_mountpoint', str, None),
    'ntrip_user': ('um960_gps_driver', 'ntrip_user', str, None),
    'ntrip_password': ('um960_gps_driver', 'ntrip_password', str, None),
    # NTRIP off means the vendor LoRa base on this robot.
    'ntrip_enabled': ('um960_gps_driver', 'correction_source', str, {True: 'ntrip', False: 'lora'}),
    # Straight-line driving (mower_control/cmd_vel_slew.py SHAPER_PARAMS; defaults neutral).
    'angular_trim_radps': ('cmd_vel_slew', 'angular_trim_radps', float, None),
    'angular_deadband_radps': ('cmd_vel_slew', 'angular_deadband_radps', float, None),
    'heading_hold': ('cmd_vel_slew', 'heading_hold', bool, None),
    'heading_hold_kp': ('cmd_vel_slew', 'heading_hold_kp', float, None),
    'heading_hold_kd': ('cmd_vel_slew', 'heading_hold_kd', float, None),
    'heading_hold_max_radps': ('cmd_vel_slew', 'heading_hold_max_radps', float, None),
}

# Targets whose node reads the GUI yaml itself (no params file to render).
SELF_LOADED_TARGETS = ('cmd_vel_slew',)


def load_settings(path):
    """Flat {key: value} of every ``ros__parameters`` block in the GUI yaml.

    A missing, empty or unreadable file gives {} (package defaults apply).
    """
    if not path:
        return {}
    try:
        with open(path, encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return {}
    flat = {}
    if not isinstance(doc, dict):
        return flat
    for section in doc.values():
        params = section.get('ros__parameters') if isinstance(section, dict) else None
        if isinstance(params, dict):
            flat.update(params)
    return flat


def _coerce(value, kind, mapping):
    """The value as the node expects it; raises ValueError when impossible."""
    if mapping is not None:
        key = value
        if isinstance(value, float) and value.is_integer():
            key = int(value)
        if key not in mapping:
            raise ValueError('%r has no equivalent on this robot' % (value,))
        value = mapping[key]
    if kind is bool:
        if isinstance(value, bool):
            return value
        raise ValueError('%r is not a boolean' % (value,))
    if kind is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError('%r is not a number' % (value,))
        return float(value)
    if kind is int:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or int(value) != value:
            raise ValueError('%r is not an integer' % (value,))
        return int(value)
    if value is None:
        return ''
    return str(value)


def bound_values(settings):
    """({target: {param path: value}}, [warning]) for the keys present in settings."""
    out, warnings = {}, []
    for key, (target, param, kind, mapping) in BINDINGS.items():
        if key not in settings or settings[key] is None:
            continue
        try:
            value = _coerce(settings[key], kind, mapping)
        except ValueError as exc:
            warnings.append('%s: %s; keeping the package default' % (key, exc))
            continue
        out.setdefault(target, {})[param] = value
    return out, warnings


def _nest(params):
    """{'A.b': 1, 'c': 2} -> {'A': {'b': 1}, 'c': 2}."""
    nested = {}
    for path, value in params.items():
        node = nested
        parts = path.split('.')
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = value
    return nested


def _deep_merge(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def overlay_document(node, params):
    """A ROS 2 params document setting params (dotted paths allowed) on node."""
    return {node: {'ros__parameters': _nest(params)}}


def merged_params_document(base_path, node, params):
    """base_path's document with params merged into node's section.

    The node section is matched by name with or without a leading '/', or
    created when absent.
    """
    with open(base_path, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    doc = copy.deepcopy(doc)
    section_key = node
    for candidate in (node, '/' + node):
        if candidate in doc:
            section_key = candidate
            break
    return _deep_merge(doc, {section_key: {'ros__parameters': _nest(params)}})


def write_params_file(doc, out_dir, name):
    path = os.path.join(out_dir, name)
    with open(path, 'w', encoding='utf-8') as f:
        f.write('# generated at launch from the GUI settings (launch/robot_settings.py)\n')
        yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False)
    return path


def make_out_dir():
    return tempfile.mkdtemp(prefix='mower_robot_settings_')


# Local-costmap observation sources that depend on the front stereo depth. det_range is
# marking-only and relies on the stereo raytrace for clearing, so it goes with it.
STEREO_SOURCES = ('stereo', 'det_range')


def without_stereo_sources(doc):
    """Copy of a nav2 params document with the stereo sources removed from the local
    costmap obstacle_layer (``stereo_costmap:=false``: bumper-only obstacle layer)."""
    doc = copy.deepcopy(doc)
    ol = doc['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
    kept = [s for s in str(ol['observation_sources']).split() if s not in STEREO_SOURCES]
    ol['observation_sources'] = ' '.join(kept)
    return doc
