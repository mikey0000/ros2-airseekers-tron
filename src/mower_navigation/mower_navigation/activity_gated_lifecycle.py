#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Activity-gated lifecycle helper for the Nav2 stack.

Watches /mission/activity and asks the *existing* lifecycle manager
(``lifecycle_manager_navigation``) to PAUSE the Nav2 lifecycle nodes when the
robot is idle/docked, and RESUME them when it is mowing/docking/manual.

Why go through ``manage_nodes`` instead of calling each
``<node>/change_state`` directly: Nav2's lifecycle manager owns the bond to
every managed node. A second node that deactivates a server behind the
manager's back tears down that server's bond, so the manager's bond watchdog
reads it as a crash ("CRITICAL FAILURE: SERVER x IS DOWN"), hard-resets the
whole stack and respawns/re-activates it. The two managers then race on the
same transitions and one of them leaves the manager's state UNKNOWN, which the
"Nav2 Health" diagnostic reports as
"An error has occurred during a node state transition".

The manager's own PAUSE/RESUME path deactivates the nodes in reverse order,
drops the bond map and stops the bond timer (PAUSE), then re-activates and
re-creates the bonds and timer (RESUME) -- no false crash detection. It also
moves the manager's own diagnostic to "Managed nodes are inactive" (OK) while
paused.

Why we reconcile against the manager's ``is_active`` service instead of just
tracking the activity string: nav2's ``manage_nodes`` PAUSE/RESUME are *not*
idempotent. ``resume()`` runs ACTIVATE on every node; an already-active node
rejects it, which makes ``resume()`` set the manager's ``managed_nodes_state_``
to UNKNOWN and trips the "Nav2 Health" diagnostic -- exactly the error this
node exists to avoid. A PAUSE issued before autostart has finished has the same
problem (a not-yet-active node rejects DEACTIVATE). So we only ever ask for a
transition the manager can actually complete:

* PAUSE only while the manager reports active (all nodes active), and
* RESUME only if *we* were the one that paused it -- or, after a startup grace
  longer than autostart, if the manager is still not active (recovering from
  this helper itself being respawned while it had the stack paused).

Idle tuning of always-on nodes (heading_aligner) and of the planner's expected
frequency is still done directly with SetParameters; those are parameters, not
lifecycle transitions, so they do not touch the bond.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from nav2_msgs.srv import ManageLifecycleNodes
from std_msgs.msg import String
from std_srvs.srv import Trigger

LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
    'collision_monitor',
]

# Parameters of nodes that stay up even while the Nav2 stack is paused. Setting
# these directly is safe: no lifecycle transition, no bond involvement.
PARAM_NODES = {
    'planner_server': {
        'active': {'expected_planner_frequency': 1.0},
        'inactive': {'expected_planner_frequency': 0.1},
    },
    'heading_aligner': {
        'active': {'imu_batch_rate': 100.0, 'stereo_batch_rate': 50.0},
        'inactive': {'imu_batch_rate': 10.0, 'stereo_batch_rate': 5.0},
    },
}

# Only these explicit modes pause the stack. Everything else (mowing, docking,
# manual, and any unknown / future string) keeps it active, matching
# mower_mission/activity.py: unknown / missing must be treated as active.
INACTIVE_STATES = frozenset(('docked_idle', 'idle', 'charging'))


class ActivityGatedLifecycleManager(Node):
    def __init__(self):
        super().__init__('activity_gated_lifecycle_manager')
        d = self.declare_parameter
        d('lifecycle_nodes', LIFECYCLE_NODES)
        d('lifecycle_manager', 'lifecycle_manager_navigation')
        d('activity_topic', '/mission/activity')
        d('check_interval_s', 2.0)
        # Longer than the manager's autostart; only used to recover from this
        # helper being respawned while it had the stack paused.
        d('startup_grace_s', 25.0)

        self._lifecycle_nodes = list(self.get_parameter('lifecycle_nodes').value)
        self._lifecycle_manager = self.get_parameter('lifecycle_manager').value
        self._activity_topic = self.get_parameter('activity_topic').value
        self._interval = float(self.get_parameter('check_interval_s').value)
        self._startup_grace = float(self.get_parameter('startup_grace_s').value)

        self._manage_client = self.create_client(
            ManageLifecycleNodes, f'/{self._lifecycle_manager}/manage_nodes')
        self._is_active_client = self.create_client(
            Trigger, f'/{self._lifecycle_manager}/is_active')
        self._param_clients = {}
        for node in PARAM_NODES:
            self._param_clients[node] = self.create_client(SetParameters, f'/{node}/set_parameters')

        self._last_activity = None
        # True once *we* successfully PAUSEd the manager. Only then may we
        # RESUME on our own initiative (nav2's resume() is not idempotent).
        self._we_paused = False
        self._grace_resume_tried = False
        self._grace_deadline = None
        self._query_pending = False   # an is_active query is outstanding
        self._transition_pending = False  # a manage_nodes call is outstanding

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._sub = self.create_subscription(String, self._activity_topic,
                                             self._on_activity, qos)

        self._timer = self.create_timer(self._interval, self._tick)
        self.get_logger().info('Activity-gated lifecycle manager started (managing %s): %s' %
                               (self._lifecycle_manager, ', '.join(self._lifecycle_nodes)))

    def _on_activity(self, msg):
        self._last_activity = msg.data
        self.get_logger().info('Activity changed to: %s' % msg.data)

    def _tick(self):
        if self._last_activity is None:
            return

        # Anything that is not an explicitly recognised low-power mode counts as
        # active (mower_mission/activity.py): never pause the stack on an
        # unknown / future activity string.
        want_active = self._last_activity not in INACTIVE_STATES
        self._adjust_parameters(want_active)

        if self._query_pending or self._transition_pending:
            return
        if not self._is_active_client.service_is_ready():
            self.get_logger().warn('%s/is_active not ready; delaying activity gate' %
                                   self._lifecycle_manager)
            return

        self._query_pending = True
        future = self._is_active_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_is_active)

    def _on_is_active(self, future):
        self._query_pending = False
        try:
            manager_active = bool(future.result().success)
        except Exception as e:
            self.get_logger().error('is_active query failed: %s' % e)
            return

        want_active = self._last_activity not in INACTIVE_STATES

        if want_active == manager_active:
            if manager_active:
                # Manager is up and so are we; we no longer own a pause.
                self._we_paused = False
            return

        if not want_active:
            # All nodes are active and we want them idle: PAUSE can complete.
            self._request_state(False)
        elif self._we_paused:
            # We paused it earlier; RESTORE.
            self._request_state(True)
        elif not self._grace_resume_tried:
            # Not active, we did not pause it, and we want it active. At boot
            # this is just autostart still running, so wait the grace before
            # assuming this helper was respawned while the stack was paused.
            self._grace_resume_tried = True
            self._grace_deadline = self.get_clock().now() + Duration(seconds=self._startup_grace)
        elif self._grace_deadline is not None and self.get_clock().now() >= self._grace_deadline:
            self._grace_deadline = None
            self._request_state(True)

    def _request_state(self, want_active):
        if not self._manage_client.service_is_ready():
            self.get_logger().warn('manage_nodes service not ready (%s)' %
                                   self._lifecycle_manager)
            return

        req = ManageLifecycleNodes.Request()
        req.command = (ManageLifecycleNodes.Request.RESUME if want_active
                       else ManageLifecycleNodes.Request.PAUSE)
        self._transition_pending = True
        future = self._manage_client.call_async(req)
        future.add_done_callback(lambda f, wa=want_active: self._on_manage_done(f, wa))

    def _on_manage_done(self, future, want_active):
        self._transition_pending = False
        verb = 'RESUME' if want_active else 'PAUSE'
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error('manage_nodes %s exception: %s' % (verb, e))
            return

        if result and result.success:
            self._we_paused = not want_active
            self.get_logger().info('%s: manage_nodes %s succeeded' %
                                   (self._lifecycle_manager, verb))
        else:
            self.get_logger().warn('%s: manage_nodes %s failed' %
                                   (self._lifecycle_manager, verb))

    def _adjust_parameters(self, want_active):
        for node, configs in PARAM_NODES.items():
            client = self._param_clients.get(node)
            if not client or not client.service_is_ready():
                continue

            config = configs['active'] if want_active else configs['inactive']
            params = []
            for key, value in config.items():
                param = Parameter()
                param.name = key
                param.value = ParameterValue()
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = float(value)
                params.append(param)

            if params:
                req = SetParameters.Request()
                req.parameters = params
                future = self._param_clients[node].call_async(req)
                future.add_done_callback(lambda f, n=node: self._on_params_set(f, n))

    def _on_params_set(self, future, node):
        try:
            result = future.result()
            if result and all(r.successful for r in result.results):
                self.get_logger().info('%s: parameters updated' % node)
            else:
                self.get_logger().warn('%s: parameter update failed: %s' %
                                       (node, [r.reason for r in result.results]))
        except Exception as e:
            self.get_logger().error('%s: parameter update exception: %s' % (node, e))


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = ActivityGatedLifecycleManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
