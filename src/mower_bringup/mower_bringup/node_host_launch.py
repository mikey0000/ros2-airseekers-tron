"""Launch side of the Python node hosts (memory diet M1/M2/M6, mower_bringup/node_host.py).

In a launch file, a Python node that may share a process is declared with

    hosted_node('control', package='mower_control', executable='cmd_vel_slew',
                name='cmd_vel_slew', respawn=True, respawn_delay=2.0, output='screen',
                condition=..., parameters=[...], remappings=[...])

instead of ``Node(...)`` (same keyword arguments). The top-level launch (mower.launch.py,
sim.launch.py) brackets everything with ``arm_node_hosts()`` (first) and ``start_node_hosts()``
(last) and declares ``compose_nodes`` (true|false):

* ``compose_nodes:=false``, or a launch file run on its own (not armed): every hosted_node is
  a plain ``launch_ros`` Node, exactly as before.
* ``compose_nodes:=true``: a hosted_node whose condition holds is evaluated like launch_ros
  would (name, namespace, params files, remaps -> the exact ``--ros-args`` argv of its
  standalone process) and registered under its group; ``start_node_hosts()`` then starts one
  ``node_host`` process per group (``node_host_<group>``) with a JSON spec of its members.
  A group with a single active member runs that member standalone (nothing to share).
  ``respawn`` of a host = any member's (``respawn_delay`` the largest); its environment is the
  first member's (e.g. the camera DDS profile of cameras.launch.py's scoped group).

``node_host_standalone:=cameras,perception`` keeps the listed groups one process per node
(e.g. if the camera host cannot hold the frame rates on the mower).

``arguments`` that are all ROS arguments (``['--ros-args', '--log-level', x]``) are hosted.
Nodes that cannot be hosted transparently (not a Python executable, extra non-ROS
``arguments``, a per-node ``env``/``additional_env``/``prefix``) fall back to standalone with
a warning.
"""
import json
import os
import tempfile
import weakref

from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

_REGISTRY = weakref.WeakKeyDictionary()     # LaunchContext -> {group: [member, ...]}
_NOT_HOSTABLE = ('env', 'additional_env', 'prefix', 'exec_name')   # + non-ROS `arguments`
COMPOSE_ARG = 'compose_nodes'


STANDALONE_ARG = 'node_host_standalone'     # comma list of groups kept one process per node


def compose_enabled(context, group=None):
    cfg = context.launch_configurations
    if context not in _REGISTRY or cfg.get(COMPOSE_ARG, 'true').strip().lower() != 'true':
        return False
    standalone = {g.strip() for g in cfg.get(STANDALONE_ARG, '').split(',') if g.strip()}
    return group not in standalone


def _is_python(path):
    try:
        with open(path, 'rb') as f:
            first = f.readline(200)
    except OSError:
        return False
    return first.startswith(b'#!') and b'python' in first


def node_argv(node, context):
    """(executable path, ros argv) launch_ros would run ``node`` with, or (None, reason)."""
    node._perform_substitutions(context)
    name = getattr(node, '_Node__expanded_node_name', Node.UNSPECIFIED_NODE_NAME)
    ros_specific = {}
    if name != Node.UNSPECIFIED_NODE_NAME:
        ros_specific['name'] = '__node:={}'.format(name)
    if node.expanded_node_namespace not in ('', Node.UNSPECIFIED_NODE_NAMESPACE):
        ros_specific['ns'] = '__ns:={}'.format(node.expanded_node_namespace)
    context.extend_locals({'ros_specific_arguments': ros_specific})
    cmd = [perform_substitutions(context, part) for part in node.cmd]
    exe, rest = cmd[0], cmd[1:]
    if not rest or rest[0] != '--ros-args':
        return None, 'has non-ROS arguments %r' % rest[:3]
    if not _is_python(exe):
        return None, '%s is not a Python executable' % exe
    return exe, rest


class HostedNode(Node):
    """A launch_ros Node that runs inside ``node_host_<group>`` when composition is on.

    Still a ``Node`` (introspection, conditions, the standalone path are unchanged)."""

    def __init__(self, group, **node_kwargs):
        super().__init__(**node_kwargs)
        self.group = group
        self.node_kwargs = node_kwargs

    def execute(self, context):
        if not compose_enabled(context, self.group):
            return super().execute(context)
        why = None
        bad = [k for k in _NOT_HOSTABLE if self.node_kwargs.get(k)]
        if bad:
            why = 'uses %s' % ', '.join(bad)
        else:
            exe, argv = node_argv(self, context)
            if exe is None:
                why = argv
        if why is not None:
            kw = {k: v for k, v in self.node_kwargs.items() if k != 'condition'}
            return [LogInfo(msg='node_host: %s/%s runs standalone (%s)' % (
                kw.get('package'), kw.get('executable'), why)), Node(**kw)]
        name = getattr(self, '_Node__expanded_node_name')
        _REGISTRY[context].setdefault(self.group, []).append({
            'name': name,
            'package': self.node_kwargs.get('package'),
            'executable': self.node_kwargs.get('executable'),
            'path': exe, 'ros_args': argv,
            'respawn': bool(self.node_kwargs.get('respawn', False)),
            'respawn_delay': float(self.node_kwargs.get('respawn_delay') or 0.0),
            'output': self.node_kwargs.get('output', 'screen'),
            'environment': dict(context.environment),
        })
        return None


def hosted_node(group, **node_kwargs):
    return HostedNode(group, **node_kwargs)


def _arm(context):
    _REGISTRY[context] = {}
    return None


def arm_node_hosts():
    """First action of a top-level launch that hosts nodes."""
    return OpaqueFunction(function=_arm)


def host_executable():
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix('mower_bringup'), 'lib', 'mower_bringup',
                        'node_host')


def write_spec(group, members, directory=None):
    spec = {'group': group, 'nodes': [
        {k: m[k] for k in ('name', 'package', 'executable', 'path', 'ros_args')}
        for m in members]}
    fd, path = tempfile.mkstemp(prefix='node_host_%s_' % group, suffix='.json', dir=directory)
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        json.dump(spec, f, indent=1)
    return path


def _start(context):
    registry = _REGISTRY.pop(context, None) or {}
    actions = []
    for group, members in sorted(registry.items()):
        if len(members) == 1:      # nothing to share: the member's own standalone argv
            m = members[0]
            actions.append(ExecuteProcess(
                cmd=[m['path']] + m['ros_args'], name=m['name'], output=m['output'],
                env=m['environment'], respawn=m['respawn'], respawn_delay=m['respawn_delay']))
            continue
        actions.append(LogInfo(msg='node_host_%s: %s' % (
            group, ', '.join(m['name'] for m in members))))
        actions.append(ExecuteProcess(
            cmd=[host_executable(), write_spec(group, members)],
            name='node_host_%s' % group, output='screen',
            env=members[0]['environment'],
            respawn=any(m['respawn'] for m in members),
            respawn_delay=max(m['respawn_delay'] for m in members)))
    return actions


def start_node_hosts():
    """Last action of a top-level launch: one node_host process per active group."""
    return OpaqueFunction(function=_start)
