#!/usr/bin/env python3
"""
cmd_vel_ws_relay - minimal WebSocket relay for /cmd_vel_teleop.

Adapted from MowgliNext (mowgli_bringup/scripts/cmd_vel_ws_relay.py),
licensed GPL-3.0; this derivative is therefore also GPL-3.0.

Accepts TwistStamped-shaped JSON over WebSocket (default port 8766) and
publishes an UNSTAMPED geometry_msgs/Twist on /cmd_vel_teleop (twist_mux 4.x
on Humble consumes Twist).

Wire format (client -> relay, bare JSON frames):
  {"twist": {"linear": {"x": 0.2, "y": 0, "z": 0},
             "angular": {"x": 0, "y": 0, "z": 0.5}}}
Any "header" field is accepted but ignored.

Parameters:
  port (8766), bind ("0.0.0.0"), max_linear (0.5 m/s), max_angular (1.0 rad/s),
  replay_interval (0.05 s), command_lease (0.25 s).

NOTE on bind: the default 0.0.0.0 is deliberate, because the web GUI may run
in another container (network_mode: host) or on another machine. There is no
authentication; restrict access at the network level or set bind:=127.0.0.1.
"""
import asyncio
import threading
from time import monotonic

import rclpy
import websockets
import websockets.exceptions
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from mower_teleop.relay_logic import LeaseTracker, twist_fields_from_json


def twist_from_fields(fields) -> Twist:
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.linear.z = fields['linear']
    msg.angular.x, msg.angular.y, msg.angular.z = fields['angular']
    return msg


class CmdVelRelayNode(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_ws_relay')
        self.port = int(self.declare_parameter('port', 8766).value)
        self.bind = str(self.declare_parameter('bind', '0.0.0.0').value)
        self.max_linear = float(self.declare_parameter('max_linear', 0.5).value)
        self.max_angular = float(self.declare_parameter('max_angular', 1.0).value)
        # Browser timers can be delayed; re-publish the last command at a fixed
        # rate while its short lease is fresh. The lease is bounded so a stale
        # motion command is never kept alive indefinitely. MCU driver timeout
        # and twist_mux teleop timeout are both 0.5 s, so 0.25 s is safe.
        self.replay_interval = float(self.declare_parameter('replay_interval', 0.05).value)
        self.command_lease = float(self.declare_parameter('command_lease', 0.25).value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Twist, '/cmd_vel_teleop', qos)
        self.get_logger().info('cmd_vel_ws_relay: publisher ready on /cmd_vel_teleop')

    def publish(self, msg: Twist) -> None:
        self._pub.publish(msg)

    def publish_json(self, raw) -> Twist:
        msg = twist_from_fields(
            twist_fields_from_json(raw, self.max_linear, self.max_angular))
        self.publish(msg)
        return msg

    def publish_zero(self) -> None:
        self.publish(Twist())


_node: CmdVelRelayNode


async def _ws_handler(websocket, path=None) -> None:
    # `path` is passed by websockets < 10.1 (e.g. 9.1 on Ubuntu Jammy); newer
    # versions call handler(websocket) only.
    addr = websocket.remote_address
    _node.get_logger().info(f'cmd_vel_ws_relay: client connected {addr}')
    latest_msg = None
    lease = LeaseTracker(_node.command_lease)
    try:
        while True:
            try:
                raw = await asyncio.wait_for(websocket.recv(), _node.replay_interval)
                latest_msg = _node.publish_json(raw)
                lease.update(monotonic())
            except asyncio.TimeoutError:
                now = monotonic()
                if latest_msg is not None and lease.should_replay(now):
                    _node.publish(latest_msg)
                elif lease.expired(now):
                    # Stop once on lease expiry; stay quiet until a new frame.
                    _node.publish_zero()
                    latest_msg = None
                    lease.clear()
            except ValueError as exc:
                _node.get_logger().warn(f'cmd_vel_ws_relay: bad message: {exc}')
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        _node.publish_zero()
        _node.get_logger().info(f'cmd_vel_ws_relay: client {addr} disconnected')


async def _serve() -> None:
    # ping_interval=None disables keep-alive pings so the relay does not send
    # periodic frames that could delay publish writes.
    async with websockets.serve(
        _ws_handler,
        _node.bind,
        _node.port,
        ping_interval=None,
        max_size=65536,
    ):
        _node.get_logger().info(
            f'cmd_vel_ws_relay: listening on {_node.bind}:{_node.port}')
        await asyncio.Future()  # run until cancelled


def main() -> None:
    global _node
    rclpy.init()
    _node = CmdVelRelayNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    spin_thread.start()

    try:
        asyncio.run(_serve())
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
