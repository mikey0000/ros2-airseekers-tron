"""Airseekers Tron ``base_ble`` driver — bridges the BLE module (phone app) to ROS 2.

The BLE module is a transparent UART passthrough at ``/dev/ttyS3`` (115200); there is no
Bluetooth stack on Linux (the GATT UUIDs are only visible from the phone). This node reads
the framed stream, dispatches each command id, and either drives the robot or replies over
the same serial line.

Recovered from the stripped ``base_ble_node`` binary + the surviving sources
``src/mower_drivers/base_ble/src/protocol_{read,write}.cc``; command semantics are in
``mower_docs/04-ble-protocol.md`` and ``reference/proto/ble.md``.

Dispatch table (confirmed from ``BleNode::BleNode`` in the binary)::

    0x80 heartbeat        -> echo mower.RTT (number+1, fresh timestamp)
    0x81/0x82 wifi ssid/pwd -> join Wi-Fi (raw bytes)
    0x84 linear  (1 sbyte /100 -> m/s)   0x85 angular (1 sbyte /100 -> rad/s)
    0x87 rtk config {u16 addr, u8 ch}   -> /mower_gps_node/set_lora
    0x88 cert {host[16], token[16]}     -> /api/device/iot-cert
    0x89 get info                        -> reply device info
    0x91 init (mower.init.Config)        -> store region
    0x92 mapping mode (BleMappingMode)   -> /mapping_control
    0x94 get version                     -> reply mower.VersionRsp
    0x95 clear fault                     -> /clear_estop
    0x96 teleop (TeleopMode)             -> enter/exit remote mode + cutter control
    0x98 map pose (MapPose)              -> /localization/GetMapInfo (dock/undock/RTK)
    0x9a loc status                      -> reply mower.RtkStatusRsp (getRobotPose)

Safety notes recovered from the binary and reproduced here:
* Drive velocities are a single signed byte divided by 100 (m/s and rad/s).
* ``cmd_vel`` is only published while in teleop mode (entered via ``0x96`` type START).
* A 1 Hz watchdog stops the blade if no drive command arrives for >8 s while cutting.
* Linear/angular are hard-clamped to 0.8 m/s / 1.0 rad/s.
The authoritative hardware interlocks still live in the C++ ``mower_base`` layer; this
node is a peripheral input bridge.
"""
import threading
import time

import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, Trigger

from mower_interfaces.msg import MotorControl
from mower_interfaces.srv import CutterControl, MappingControl, SetLoRa
from mower_proto import init_pb2, map_pb2, status_pb2, teleop_pb2

from base_ble.ble_protocol import build_frame, extract_frames, parse_frame, CmdId


def _signed_byte(b: int) -> int:
    return b if b < 0x80 else b - 0x100


class BleNode(Node):
    def __init__(self):
        super().__init__('base_ble_node')

        self.declare_parameter('port', '/dev/serial_ble')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('sn_path', '/meta/sn')
        self.declare_parameter('mac_path', '/meta/mac_addr')
        self.declare_parameter('linear_max', 0.8)   # m/s  (recovered clamp)
        self.declare_parameter('angular_max', 1.0)  # rad/s (recovered clamp)
        self.declare_parameter('drive_timeout_ms', 8000)  # cutter watchdog

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.linear_max = self.get_parameter('linear_max').value
        self.angular_max = self.get_parameter('angular_max').value
        self.drive_timeout_ms = self.get_parameter('drive_timeout_ms').value

        # ---- ROS interfaces ----
        # Teleop lane of twist_mux (unstamped Twist, Jazzy); never /cmd_vel directly.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.clear_estop_cli = self.create_client(Empty, '/clear_estop')
        self.cutter_cli = self.create_client(CutterControl, '/cutter_control')
        self.mapping_cli = self.create_client(MappingControl, '/mapping_control')
        self.set_lora_cli = self.create_client(SetLoRa, '/mower_gps_node/set_lora')
        self.init_srv = self.create_service(Trigger, '/base_ble/init', self._init_cb)

        # ---- identity ----
        self.sn = self._read_first_token(self.get_parameter('sn_path').value, fallback='')
        self.mac = self._read_first_token(self.get_parameter('mac_path').value, fallback='')

        # ---- serial + parsing state ----
        self.ser = None
        self._buf = bytearray()
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)

        # ---- drive state ----
        self._linear = 0.0          # m/s
        self._angular = 0.0         # rad/s
        self._teleop = False        # remote-control mode active (0x96 START)
        self._cutter_running = False
        self._last_drive_ms = 0.0
        self._wifi_ssid = None
        self._wifi_pwd = None

        self._connect_serial()
        self._reader.start()

        # 20 Hz cmd_vel publisher (mirrors the binary's 0.05 s Twist timer).
        self._cmd_timer = self.create_timer(1.0 / 20.0, self._publish_cmd_vel)
        # 1 Hz cutter timeout watchdog (mirrors twistModeTimerCallback).
        self._watchdog = self.create_timer(1.0, self._watchdog_cb)

        self.get_logger().info(
            'base_ble up on %s (sn=%s); drive=%s..%s m/s, cutter timeout=%d ms',
            self.port, self.sn, -self.linear_max, self.linear_max, self.drive_timeout_ms)

    # ------------------------------------------------------------------ serial

    @staticmethod
    def _read_first_token(path, fallback):
        try:
            with open(path) as fh:
                return fh.read().strip().splitlines()[0].strip()
        except (OSError, IndexError):
            return fallback

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info('serial %s opened at %d', self.port, self.baud)
        except serial.SerialException as exc:
            self.get_logger().error('failed to open %s: %s', self.port, exc)
            self.ser = None

    def _read_loop(self):
        while self._running:
            if self.ser is None:
                time.sleep(0.5)
                self._connect_serial()
                continue
            try:
                avail = self.ser.in_waiting
                if avail > 0:
                    self._buf.extend(self.ser.read(avail))
            except serial.SerialException:
                self.ser = None
                continue
            self._drain()
            time.sleep(0.005)

    def _drain(self):
        frames, leftover = extract_frames(bytes(self._buf))
        self._buf = bytearray(leftover)
        for frame in frames:
            fields = parse_frame(frame)
            if fields is None:
                self.get_logger().warning('bad frame %s', frame.hex())
                continue
            for fid, data in fields:
                try:
                    self._dispatch(fid, data)
                except Exception as exc:  # never take the node down on a bad frame
                    self.get_logger().error('handler 0x%02x failed: %s', fid, exc)

    # ------------------------------------------------------------------ dispatch

    def _dispatch(self, fid, data):
        if fid == CmdId.HEARTBEAT:
            self._on_heartbeat(data)
        elif fid == CmdId.WIFI_SSID:
            self._wifi_ssid = bytes(data)
            self.get_logger().info('wifi ssid set (%d bytes)', len(data))
        elif fid == CmdId.WIFI_PWD:
            self._wifi_pwd = bytes(data)
            self.get_logger().info('wifi password set; join TODO (NetworkManager/nmcli)')
        elif fid == CmdId.DRIVE_LINEAR and data:
            self._linear = _signed_byte(data[0]) / 100.0
        elif fid == CmdId.DRIVE_ANGULAR and data:
            self._angular = _signed_byte(data[0]) / 100.0
            self._last_drive_ms = time.monotonic()
        elif fid == CmdId.RTK_CONFIG:
            self._on_rtk_config(data)
        elif fid == CmdId.CERT_ACTIVATE:
            self._on_cert(data)
        elif fid == CmdId.GET_INFO:
            self._on_get_info(data)
        elif fid == CmdId.INIT:
            self._on_init(data)
        elif fid == CmdId.MAP_MODE:
            self._on_map_mode(data)
        elif fid == CmdId.GET_VERSION:
            self._on_get_version()
        elif fid == CmdId.CLEAR_FAULT:
            self._on_clear_fault()
        elif fid == CmdId.TELEOP:
            self._on_teleop(data)
        elif fid == CmdId.MAP_POSE:
            self._on_map_pose(data)
        elif fid == CmdId.LOC_STATUS:
            self._on_loc_status()
        else:
            self.get_logger().debug('unhandled id 0x%02x (%d bytes)', fid, len(data))

    # ------------------------------------------------------------------ handlers

    def _send(self, fid, data=b''):
        if self.ser is not None:
            try:
                self.ser.write(build_frame([(fid, data)]))
            except serial.SerialException as exc:
                self.get_logger().error('write failed: %s', exc)

    def _on_heartbeat(self, data):
        rtt = status_pb2.RTT()
        try:
            rtt.ParseFromString(bytes(data))
        except Exception:
            return
        rtt.number += 1
        rtt.timestamp = int(time.time() * 1000)
        self._send(CmdId.HEARTBEAT, rtt.SerializeToString())

    def _on_rtk_config(self, data):
        if len(data) < 3:
            return
        addr = int.from_bytes(data[0:2], 'big')
        channel = data[2]
        if self.set_lora_cli.service_is_ready():
            req = SetLoRa.Request(sn=self.sn, addr=addr, channel=channel, area='')
            self.set_lora_cli.call_async(req)
            self.get_logger().info('set_lora addr=%d channel=%d', addr, channel)
        else:
            self.get_logger().warning('set_lora addr=%d channel=%d: service unavailable',
                                      addr, channel)

    def _on_cert(self, data):
        if len(data) != 32:
            return
        host = data[0:16].split(b'\x00')[0].decode(errors='replace')
        token = data[16:32].split(b'\x00')[0].decode(errors='replace')
        self.get_logger().info('iot cert activation host=%s (POST /api/device/iot-cert TODO)',
                               host)

    def _on_get_info(self, _data):
        # reply: sn, mac, firmware versions (MowerInfoData)
        info = ('sn=%s mac=%s' % (self.sn, self.mac)).encode()
        self._send(CmdId.GET_INFO, info)

    def _on_init(self, data):
        cfg = init_pb2.Config()
        try:
            cfg.ParseFromString(bytes(data))
        except Exception:
            return
        self.get_logger().info('init region=%s', cfg.request.region)

    _MAP_CMD = {
        map_pb2.BleMappingMode.START: 'start',
        map_pb2.BleMappingMode.CANCEL: 'cancel',
        map_pb2.BleMappingMode.SAVE: 'save',
        map_pb2.BleMappingMode.EXTEND: 'extend',
        map_pb2.BleMappingMode.EXPLORE: 'explore',
        map_pb2.BleMappingMode.EXPLORE_SURE: 'explore_sure',
        map_pb2.BleMappingMode.EXPLORE_EXTEND: 'explore_extend',
    }

    def _on_map_mode(self, data):
        mode = map_pb2.BleMappingMode()
        try:
            mode.ParseFromString(bytes(data))
        except Exception:
            return
        command = self._MAP_CMD.get(mode.type)
        if command is None:
            self.get_logger().warning('unknown mapping mode %s', mode.type)
            return
        if self.mapping_cli.service_is_ready():
            req = MappingControl.Request(command=command, map_name=mode.map_name)
            self.mapping_cli.call_async(req)
            self.get_logger().info('mapping command=%s map_name=%s', command, mode.map_name)
        else:
            self.get_logger().warning('mapping command=%s: /mapping_control unavailable',
                                      command)

    def _on_get_version(self):
        v = status_pb2.VersionRsp()
        # chassis/cutter/mower_package/rtk board versions come from the live node; defaults
        # here are placeholders until the real version source is wired.
        v.chassis_board = ''
        v.cutter_board = ''
        v.mower_package = ''
        v.rtk_board = ''
        self._send(CmdId.GET_VERSION, v.SerializeToString())

    def _on_clear_fault(self):
        self.get_logger().info('clear fault -> /clear_estop')
        if self.clear_estop_cli.service_is_ready():
            self.clear_estop_cli.call_async(Empty.Request())
        else:
            self.get_logger().warning('/clear_estop service unavailable')

    def _on_teleop(self, data):
        mode = teleop_pb2.TeleopMode()
        try:
            mode.ParseFromString(bytes(data))
        except Exception:
            return

        if mode.type == teleop_pb2.TeleopMode.START:
            self._teleop = True
            self.get_logger().info('teleop mode START')
        elif mode.type == teleop_pb2.TeleopMode.STOP:
            self._teleop = False
            self._linear = self._angular = 0.0
            self._cutter(enable=False)
            self._publish_cmd_vel()
            self.get_logger().info('teleop mode STOP')

        if mode.cutter_height != 0:
            self._cutter_height(mode.cutter_height)
        if mode.cutter == teleop_pb2.TeleopMode.CUTTER_START:
            self._cutter(enable=True)
        elif mode.cutter == teleop_pb2.TeleopMode.CUTTER_STOP:
            self._cutter(enable=False)

    def _on_map_pose(self, data):
        pose = map_pb2.MapPose()
        try:
            pose.ParseFromString(bytes(data))
        except Exception:
            return
        self.get_logger().info('map pose type=%s (LocatorGetMapInfo client TODO)', pose.type)

    def _on_loc_status(self):
        rsp = status_pb2.RtkStatusRsp()
        # populated from the localization node; placeholders until wired
        rsp.localization_state = 0
        rsp.rtk_status = ''
        self._send(CmdId.LOC_STATUS, rsp.SerializeToString())

    def _init_cb(self, request, response):
        response.success = True
        response.message = 'base_ble ready'
        return response

    # ------------------------------------------------------------------ drive / cutter

    def _publish_cmd_vel(self):
        if not self._teleop:
            return  # publish nothing: twist_mux times the teleop lane out on its own
        msg = Twist()
        msg.linear.x = max(-self.linear_max, min(self.linear_max, self._linear))
        msg.angular.z = max(-self.angular_max, min(self.angular_max, self._angular))
        # deadband (recovered): below 1 mm/s / 1 mrad/s treat as zero
        if abs(msg.linear.x) < 0.001 and abs(msg.angular.z) < 0.001:
            msg.linear.x = msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def _cutter(self, enable):
        req = CutterControl.Request()
        req.cutter = MotorControl(enable=enable, direction=False, speed=100, position=0)
        req.height = MotorControl(enable=False, direction=False, speed=0, position=0)
        if self.cutter_cli.service_is_ready():
            self.cutter_cli.call_async(req)
        else:
            self.get_logger().warning('/logic/cutter_control service unavailable')
        self._cutter_running = enable

    def _cutter_height(self, height):
        req = CutterControl.Request()
        req.cutter = MotorControl(enable=False, direction=False, speed=0, position=0)
        req.height = MotorControl(enable=True, direction=False, speed=100, position=height)
        if self.cutter_cli.service_is_ready():
            self.cutter_cli.call_async(req)
        else:
            self.get_logger().warning('/logic/cutter_control service unavailable')

    def _watchdog_cb(self):
        if not (self._teleop and self._cutter_running):
            return
        if (time.monotonic() - self._last_drive_ms) * 1000.0 > self.drive_timeout_ms:
            self.get_logger().warning('no drive command for %d ms: stopping blade',
                                      self.drive_timeout_ms)
            self._cutter(enable=False)

    # ------------------------------------------------------------------ lifecycle

    def stop(self):
        self._running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = BleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
