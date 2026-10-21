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

Bounded waits (2026-10-13 incident): every service call this helper makes has a
deadline. An unanswered ``is_active`` / ``manage_nodes`` / SetParameters future is
dropped (``remove_pending_request``) and retried on a later tick; before, one lost
reply left ``_query_pending`` set forever and the gate went silent for 5.5 h.

Bringup watchdog (``bringup_watchdog_s``, 0 disables): Jazzy's nav2 lifecycle
manager calls ``change_state`` with no timeout. If a server's reply is lost
(rmw_fastrtps answers before the client's response reader is matched -- seen at
boot with ~60 nodes discovering) the manager blocks forever in "Configuring
<node>" and answers none of its services, leaving the stack half configured. Nav2's
startup() refuses nodes that are already configured, so the only clean recovery is
to restart the manager *and* every managed node that is no longer unconfigured
(launch ``respawn`` brings them back unconfigured and autostart runs again). The
watchdog fires only when ``is_active`` has gone *unanswered* (queries sent and timed
out, service discovered) for ``bringup_watchdog_s`` since this helper started, the
manager (re)started or the service reappeared, while we have not paused the stack
ourselves, and at least one managed node is not active. Any answer -- even "not
active" -- resets it, so a slow but progressing autostart never trips it. At most
``bringup_watchdog_max_attempts`` restarts per helper lifetime, with exponential
backoff. See docs/crash_recovery.md.
"""


from __future__ import annotations

import os
import signal
import time

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


# --- pure decision logic (no ROS; unit-tested in test/test_activity_gated_lifecycle.py) ----

def node_now(node):
    """time.monotonic, or the node's ROS clock in seconds under use_sim_time (mower_sim
    publishes /clock, possibly faster than real time). Under sim time it first blocks until
    the first /clock and seeds the node clock with it (no stamp on a zero clock)."""
    if not node.get_parameter('use_sim_time').value:
        return time.monotonic
    from rclpy.time import Time
    from rclpy.wait_for_message import wait_for_message
    from rosgraph_msgs.msg import Clock as ClockMsg
    clock = node.get_clock()
    while not clock.now().nanoseconds and node.context.ok():
        ok, msg = wait_for_message(ClockMsg, node, '/clock', time_to_wait=10.0)
        if ok and (msg.clock.sec or msg.clock.nanosec) and not clock.now().nanoseconds:
            clock.set_ros_time_override(Time.from_msg(msg.clock))
        elif not ok:
            node.get_logger().warn('use_sim_time: waiting for /clock')
    return lambda: clock.now().nanoseconds * 1e-9


class ParamSync:
    """When to (re)send one node's idle/active parameter set.

    Sends only when the wanted state differs from what the node acknowledged, when the
    last send failed or went unanswered for ``ack_timeout_s``, when the node's parameter
    service disappeared (respawn: it restarts from its yaml defaults), or -- quietly --
    every ``resync_s`` (0 disables).
    """

    def __init__(self, ack_timeout_s=5.0, resync_s=60.0):
        self.ack_timeout_s = float(ack_timeout_s)
        self.resync_s = float(resync_s)
        self.applied = None       # state the node last acknowledged (None = unknown)
        self.applied_t = None
        self.pending = None       # (state, sent time)

    def on_unavailable(self):
        self.applied = None

    def expired(self, now):
        """True if the outstanding send timed out (caller drops its future)."""
        if self.pending is None or now - self.pending[1] < self.ack_timeout_s:
            return False
        self.pending = None
        return True

    def should_send(self, want, now):
        if self.pending is not None:
            return False
        if self.applied != want:
            return True
        return self.resync_s > 0 and now - self.applied_t >= self.resync_s

    def sent(self, want, now):
        self.pending = (want, now)

    def acked(self, ok, now):
        """Record the reply; returns True if the node's applied state changed."""
        if self.pending is None:
            return False
        state = self.pending[0]
        self.pending = None
        if not ok:
            self.applied = None
            return False
        changed = self.applied != state
        self.applied = state
        self.applied_t = now
        return changed


class BringupWatchdog:
    """Detects a lifecycle manager that has stopped answering (hung in a transition).

    ``now`` is any monotonic seconds value. The unanswered window starts at construction
    and is restarted by any answer, by the service being undiscovered, by a manager
    process change and by our own recovery attempt.
    """

    def __init__(self, timeout_s=90.0, max_attempts=3, backoff_s=60.0, now=0.0):
        self.timeout_s = float(timeout_s)
        self.max_attempts = int(max_attempts)
        self.backoff_s = float(backoff_s)
        self.attempts = 0
        self._since = now
        self._not_before = now
        self._manager_pid = None

    @property
    def enabled(self):
        return self.timeout_s > 0.0

    def unanswered_for(self, now):
        return now - self._since

    def on_answer(self, now):
        self._since = now

    def on_unavailable(self, now):
        # Manager not discovered: its absence is respawn's business, not a hang. The
        # clock starts when the service is back.
        self._since = now

    def on_manager_pid(self, pid, now):
        if pid is None:
            return
        if self._manager_pid is not None and pid != self._manager_pid:
            self._since = now     # manager restarted: give its autostart a full window
        self._manager_pid = pid

    def due(self, now, we_paused):
        """True when a recovery should be attempted now (after an unanswered query)."""
        if not self.enabled:
            return False
        if we_paused:
            # Our PAUSE leaves nodes inactive on purpose; never restart over it.
            self._since = now
            return False
        if self.attempts >= self.max_attempts or now < self._not_before:
            return False
        return now - self._since >= self.timeout_s

    def exhausted(self):
        return self.attempts >= self.max_attempts

    def defer(self, now):
        """Hung but nothing to recover (all nodes active): look again after a full window."""
        self._since = now

    def record_attempt(self, now):
        self.attempts += 1
        self._since = now
        self._not_before = now + self.backoff_s * (2 ** (self.attempts - 1))


def nodes_to_restart(states):
    """Managed nodes that must be restarted for a clean autostart, or None.

    ``states`` maps node name -> lifecycle state label ('unconfigured', 'inactive',
    'active', ...) or None when get_state went unanswered. Nav2's startup() fails on any
    node that is not unconfigured, so every such node -- and every node we could not
    query -- is restarted. Returns None when every node answered 'active': the stack is
    up and restarting it would only make things worse.
    """
    if states and all(s == 'active' for s in states.values()):
        return None
    return [n for n, s in states.items() if s != 'unconfigured']


def find_node_pids(node_name, proc_root='/proc', exclude=()):
    """PIDs whose command line carries ``__node:=<node_name>`` (launch_ros remap)."""
    token = ('__node:=%s' % node_name).encode()
    pids = []
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return pids
    for entry in entries:
        if not entry.isdigit() or int(entry) in exclude:
            continue
        try:
            with open(os.path.join(proc_root, entry, 'cmdline'), 'rb') as f:
                args = f.read().split(b'\0')
        except OSError:
            continue
        if token in args:
            pids.append(int(entry))
    return sorted(pids)


def kill_pids(pids, sig=signal.SIGTERM, kill=os.kill):
    """Send ``sig`` to each pid; returns the pids it was delivered to."""
    done = []
    for pid in pids:
        try:
            kill(pid, sig)
            done.append(pid)
        except (ProcessLookupError, PermissionError):
            pass
    return done


# --- ROS node ---------------------------------------------------------------------------

def _make_node_class():
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.srv import ManageLifecycleNodes
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class ActivityGatedLifecycleManager(Node):
        def __init__(self):
            super().__init__('activity_gated_lifecycle_manager')
            self._now = node_now(self)
            d = self.declare_parameter
            d('lifecycle_nodes', LIFECYCLE_NODES)
            d('lifecycle_manager', 'lifecycle_manager_navigation')
            d('activity_topic', '/mission/activity')
            d('check_interval_s', 2.0)
            # Longer than the manager's autostart; only used to recover from this
            # helper being respawned while it had the stack paused.
            d('startup_grace_s', 25.0)
            # Deadlines for outstanding calls; on expiry the call is dropped and retried.
            d('query_timeout_s', 10.0)        # is_active
            d('manage_timeout_s', 30.0)       # manage_nodes PAUSE / RESUME
            d('param_timeout_s', 5.0)         # SetParameters
            # Manager unanswered this long (and nodes not all active) -> restart it and
            # every configured managed node. 0 disables. Must exceed a normal autostart.
            d('bringup_watchdog_s', 90.0)
            d('bringup_watchdog_max_attempts', 3)
            d('bringup_watchdog_backoff_s', 60.0)

            gp = lambda n: self.get_parameter(n).value  # noqa: E731
            self._lifecycle_nodes = list(gp('lifecycle_nodes'))
            self._lifecycle_manager = gp('lifecycle_manager')
            self._activity_topic = gp('activity_topic')
            self._interval = float(gp('check_interval_s'))
            self._startup_grace = float(gp('startup_grace_s'))
            self._query_timeout = float(gp('query_timeout_s'))
            self._manage_timeout = float(gp('manage_timeout_s'))
            now = self._now()
            self._watchdog = BringupWatchdog(gp('bringup_watchdog_s'),
                                             gp('bringup_watchdog_max_attempts'),
                                             gp('bringup_watchdog_backoff_s'), now)

            self._manage_client = self.create_client(
                ManageLifecycleNodes, f'/{self._lifecycle_manager}/manage_nodes')
            self._is_active_client = self.create_client(
                Trigger, f'/{self._lifecycle_manager}/is_active')
            self._param_clients = {}
            self._param_sync = {}
            self._param_futures = {}
            for node in PARAM_NODES:
                self._param_clients[node] = self.create_client(
                    SetParameters, f'/{node}/set_parameters')
                self._param_sync[node] = ParamSync(gp('param_timeout_s'))
            self._get_state_clients = {
                n: self.create_client(GetState, f'/{n}/get_state')
                for n in self._lifecycle_nodes}

            self._last_activity = None
            # True once *we* successfully PAUSEd the manager. Only then may we
            # RESUME on our own initiative (nav2's resume() is not idempotent).
            self._we_paused = False
            self._grace_resume_tried = False
            self._grace_deadline = None
            self._query = None        # (future, sent) of the outstanding is_active query
            self._transition = None   # (future, sent, want_active) of a manage_nodes call
            self._probe = None        # (deadline, {node: future}) bringup-watchdog probe
            self._escalate = None     # (when, {pid: node}) SIGKILL leftovers after SIGTERM

            self._Duration = Duration
            self._Parameter, self._ParameterValue = Parameter, ParameterValue
            self._ParameterType, self._SetParameters = ParameterType, SetParameters
            self._GetState, self._Trigger = GetState, Trigger
            self._Manage = ManageLifecycleNodes

            qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self._sub = self.create_subscription(String, self._activity_topic,
                                                 self._on_activity, qos)

            self._timer = self.create_timer(self._interval, self._tick)
            self.get_logger().info(
                'Activity-gated lifecycle manager started (managing %s): %s; bringup '
                'watchdog %s' % (self._lifecycle_manager, ', '.join(self._lifecycle_nodes),
                                 ('%.0f s' % self._watchdog.timeout_s)
                                 if self._watchdog.enabled else 'off'))

        def _on_activity(self, msg):
            self._last_activity = msg.data
            self.get_logger().info('Activity changed to: %s' % msg.data)

        def _want_active(self):
            # Anything that is not an explicitly recognised low-power mode counts as
            # active (mower_mission/activity.py): never pause the stack on an
            # unknown / future activity string.
            return self._last_activity not in INACTIVE_STATES

        # --- tick -------------------------------------------------------------------

        def _tick(self):
            now = self._now()
            self._service_escalation(now)
            if self._probe is not None:
                self._service_probe(now)
                return

            if self._last_activity is not None:
                self._adjust_parameters(self._want_active(), now)

            if self._transition is not None:
                fut, sent, want = self._transition
                if fut.done():
                    self._on_manage_done(fut, want)   # no-op if its callback already ran
                elif now - sent < self._manage_timeout:
                    return
                else:
                    self._manage_client.remove_pending_request(fut)
                    self._transition = None
                    self.get_logger().warn(
                        '%s: manage_nodes %s unanswered after %.0f s; dropped, will '
                        'reconcile again' % (self._lifecycle_manager,
                                             'RESUME' if want else 'PAUSE',
                                             self._manage_timeout),
                        throttle_duration_sec=30.0)
                return

            if self._query is not None:
                fut, sent = self._query
                if fut.done():
                    self._on_is_active(fut)           # no-op if its callback already ran
                    return
                if now - sent < self._query_timeout:
                    return
                self._is_active_client.remove_pending_request(fut)
                self._query = None
                self._on_query_timeout(now)
                if self._probe is not None:
                    return

            if not self._is_active_client.service_is_ready():
                self._watchdog.on_unavailable(now)
                if self._last_activity is not None:
                    self.get_logger().warn('%s/is_active not ready; delaying activity gate'
                                           % self._lifecycle_manager,
                                           throttle_duration_sec=30.0)
                return

            fut = self._is_active_client.call_async(self._Trigger.Request())
            self._query = (fut, now)
            fut.add_done_callback(self._on_is_active)

        def _on_query_timeout(self, now):
            wd = self._watchdog
            pids = find_node_pids(self._lifecycle_manager, exclude=(os.getpid(),))
            wd.on_manager_pid(pids[0] if pids else None, now)
            self.get_logger().warn(
                '%s/is_active unanswered for %.0f s (manager busy or hung in a transition)'
                % (self._lifecycle_manager, wd.unanswered_for(now)),
                throttle_duration_sec=30.0)
            if wd.due(now, self._we_paused):
                self._start_probe(now)
            elif (wd.enabled and wd.exhausted()
                  and wd.unanswered_for(now) >= wd.timeout_s):
                self.get_logger().error(
                    '%s still unresponsive after %d bringup-watchdog restarts; giving up '
                    '(container restart needed)' % (self._lifecycle_manager, wd.attempts),
                    throttle_duration_sec=300.0)

        def _on_is_active(self, future):
            if self._query is None or future is not self._query[0]:
                return    # dropped after its deadline
            self._query = None
            self._watchdog.on_answer(self._now())
            try:
                manager_active = bool(future.result().success)
            except Exception as e:
                self.get_logger().error('is_active query failed: %s' % e)
                return
            if self._last_activity is None:
                return

            want_active = self._want_active()

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
                self._grace_deadline = (self.get_clock().now()
                                        + self._Duration(seconds=self._startup_grace))
            elif (self._grace_deadline is not None
                  and self.get_clock().now() >= self._grace_deadline):
                self._grace_deadline = None
                self._request_state(True)

        # --- manage_nodes -----------------------------------------------------------

        def _request_state(self, want_active):
            if not self._manage_client.service_is_ready():
                self.get_logger().warn('manage_nodes service not ready (%s)' %
                                       self._lifecycle_manager,
                                       throttle_duration_sec=30.0)
                return

            req = self._Manage.Request()
            req.command = (self._Manage.Request.RESUME if want_active
                           else self._Manage.Request.PAUSE)
            future = self._manage_client.call_async(req)
            self._transition = (future, self._now(), want_active)
            future.add_done_callback(lambda f, wa=want_active: self._on_manage_done(f, wa))

        def _on_manage_done(self, future, want_active):
            if self._transition is None or future is not self._transition[0]:
                return    # dropped after its deadline
            self._transition = None
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

        # --- bringup watchdog -------------------------------------------------------

        def _start_probe(self, now):
            futures = {}
            for node, cli in self._get_state_clients.items():
                futures[node] = (cli.call_async(self._GetState.Request())
                                 if cli.service_is_ready() else None)
            self._probe = (now + self._query_timeout, futures)
            self.get_logger().warn(
                '%s unanswered for %.0f s; bringup watchdog probing managed node states'
                % (self._lifecycle_manager, self._watchdog.unanswered_for(now)))

        def _service_probe(self, now):
            deadline, futures = self._probe
            if now < deadline and not all(f is None or f.done() for f in futures.values()):
                return
            self._probe = None
            states = {}
            for node, fut in futures.items():
                state = None
                if fut is not None and fut.done():
                    try:
                        state = fut.result().current_state.label
                    except Exception:
                        state = None
                elif fut is not None:
                    self._get_state_clients[node].remove_pending_request(fut)
                states[node] = state
            restart = nodes_to_restart(states)
            desc = ', '.join('%s=%s' % (n, s or 'no answer') for n, s in states.items())
            if restart is None:
                self._watchdog.defer(now)
                self.get_logger().warn(
                    '%s unresponsive but every managed node is active (%s); not restarting'
                    % (self._lifecycle_manager, desc))
                return
            self._watchdog.record_attempt(now)
            me = (os.getpid(),)
            targets = {}
            for node in [self._lifecycle_manager] + restart:
                for pid in find_node_pids(node, exclude=me):
                    targets[pid] = node
            killed = kill_pids(sorted(targets))
            self.get_logger().error(
                'BRINGUP WATCHDOG (attempt %d/%d): %s has not answered is_active and the '
                'stack is not fully active (%s). Nav2 autostart cannot configure nodes that '
                'are already configured, so restarting the manager and %s via SIGTERM '
                '(launch respawn brings them back unconfigured, autostart reruns). '
                'Signalled pids: %s' % (
                    self._watchdog.attempts, self._watchdog.max_attempts,
                    self._lifecycle_manager, desc, ', '.join(restart) or 'no other node',
                    ', '.join('%d(%s)' % (p, targets[p]) for p in killed) or 'none found'))
            if killed:
                self._escalate = (now + 10.0, {p: targets[p] for p in killed})
            # Old futures are meaningless for the restarted manager.
            for cli, cur in ((self._is_active_client, self._query),
                             (self._manage_client, self._transition)):
                if cur is not None:
                    cli.remove_pending_request(cur[0])
            self._query = None
            self._transition = None

        def _service_escalation(self, now):
            if self._escalate is None or now < self._escalate[0]:
                return
            _, pids = self._escalate
            self._escalate = None
            left = [p for p, node in pids.items()
                    if p in find_node_pids(node, exclude=(os.getpid(),))]
            if left:
                kill_pids(left, signal.SIGKILL)
                self.get_logger().error('bringup watchdog: SIGKILL %s (ignored SIGTERM)' %
                                        ', '.join('%d(%s)' % (p, pids[p]) for p in left))

        # --- idle tuning parameters -------------------------------------------------

        def _adjust_parameters(self, want_active, now):
            for node, configs in PARAM_NODES.items():
                client = self._param_clients[node]
                sync = self._param_sync[node]
                if sync.expired(now):
                    fut = self._param_futures.pop(node, None)
                    if fut is not None:
                        client.remove_pending_request(fut)
                    self.get_logger().warn('%s: parameter update unanswered; will retry'
                                           % node, throttle_duration_sec=60.0)
                if not client.service_is_ready():
                    sync.on_unavailable()
                    continue
                if not sync.should_send(want_active, now):
                    continue

                config = configs['active'] if want_active else configs['inactive']
                req = self._SetParameters.Request()
                for key, value in config.items():
                    param = self._Parameter()
                    param.name = key
                    param.value = self._ParameterValue()
                    param.value.type = self._ParameterType.PARAMETER_DOUBLE
                    param.value.double_value = float(value)
                    req.parameters.append(param)
                future = client.call_async(req)
                sync.sent(want_active, now)
                self._param_futures[node] = future
                future.add_done_callback(lambda f, n=node: self._on_params_set(f, n))

        def _on_params_set(self, future, node):
            if self._param_futures.get(node) is not future:
                return    # dropped after its deadline
            del self._param_futures[node]
            sync = self._param_sync[node]
            state = sync.pending[0] if sync.pending else None
            ok = False
            try:
                result = future.result()
                ok = bool(result) and all(r.successful for r in result.results)
                if not ok:
                    self.get_logger().warn('%s: parameter update failed: %s' % (
                        node, [r.reason for r in result.results] if result else None),
                        throttle_duration_sec=60.0)
            except Exception as e:
                self.get_logger().error('%s: parameter update exception: %s' % (node, e),
                                        throttle_duration_sec=60.0)
            if sync.acked(ok, self._now()):
                self.get_logger().info('%s: parameters set for %s' % (
                    node, 'active' if state else 'idle'))

    return ActivityGatedLifecycleManager


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = _make_node_class()()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
