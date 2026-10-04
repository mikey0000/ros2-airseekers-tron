"""ROS 2 node wrapping the Unicore UM960 GNSS/RTK receiver.

Publishes
    /fix          sensor_msgs/msg/NavSatFix   (status from NMEA GGA quality + GSA/BESTNAV mode)
    /vel          geometry_msgs/msg/TwistStamped
    /heading      std_msgs/msg/Float32        (degrees, true north, from GPHPR/VTG/RMC)
    /fix_status   std_msgs/msg/String         (human-readable quality/solution summary)
    /nmea         std_msgs/msg/String         (raw ASCII sentences, optional)

Hardware: Unicore UM960 on /dev/serial_rtk (ttyS4, UART4 @ 0xfeb70000), 115200 8N1.
"""

import threading
import time
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String

from .parsers import (
    GSA_FIX_TYPE,
    SOL_COMPUTED,
    NmeaParser,
    Um960StreamSplitter,
    bestnav_position_type_name,
    bestnav_solution_status_name,
    bestnav_velocity_type_name,
    hdop_to_covariance,
    nmea_checksum_ok,
    sigma_to_covariance,
)
from .serial_port import SerialError, SerialPort

# NMEA sentence suffixes that carry a position. feed() returns the full talker id
# ("GNGGA", "GPGGA", ...), so match on the suffix.
FIX_BEARING_SUFFIXES = ("GGA", "RMC")
# Unicore ASCII position logs use the whole id as the talker.
FIX_BEARING_IDS = frozenset({"BESTNAV", "BESTNAVA"})

# Unicore BESTNAV position types that are an integer/fixed solution. Mapped to NMEA
# GGA quality 4; every other position type is reported as quality 1.
FIXED_POS_TYPES = frozenset({1, 10, 11, 12, 17, 19})

# NMEA GGA quality -> sensor_msgs/NavSatStatus.STATUS_*.
STATUS_BY_QUALITY = {
    0: NavSatStatus.STATUS_NO_FIX,
    1: NavSatStatus.STATUS_FIX,
    2: NavSatStatus.STATUS_GBAS_FIX,
    4: NavSatStatus.STATUS_FIX,
    5: NavSatStatus.STATUS_FIX,
    6: NavSatStatus.STATUS_FIX,
}


class Um960Node(Node):
    """Read the UM960 serial stream and publish ROS navigation messages.

    ``**kwargs`` is forwarded to ``rclpy.node.Node``, which lets callers pass
    ``parameter_overrides`` (used by the tests to point the node at a pty).
    """

    def __init__(self, **kwargs) -> None:
        super().__init__("um960_node", **kwargs)

        self.declare_parameter("port", "/dev/serial_rtk")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("vel_topic", "/vel")
        self.declare_parameter("heading_topic", "/heading")
        self.declare_parameter("fix_status_topic", "/fix_status")
        self.declare_parameter("nmea_topic", "/nmea")
        self.declare_parameter("publish_nmea", False)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter("serial_timeout_s", 0.2)
        self.declare_parameter("fix_timeout_s", 2.0)
        self.declare_parameter("sane_lat_limit", 90.0)
        self.declare_parameter("sane_lon_limit", 180.0)
        # Receiver configuration written on (re)connect; empty by default so the
        # driver never fights with a receiver that was already configured.
        self.declare_parameter("config_commands", [])

        self.port_name = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.reconnect_period = float(self.get_parameter("reconnect_period_s").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout_s").value)
        self.fix_timeout = float(self.get_parameter("fix_timeout_s").value)
        self.sane_lat = float(self.get_parameter("sane_lat_limit").value)
        self.sane_lon = float(self.get_parameter("sane_lon_limit").value)
        self.config_commands = [str(c) for c in self.get_parameter("config_commands").value]
        publish_nmea = bool(self.get_parameter("publish_nmea").value)
        self.nmea_topic = str(self.get_parameter("nmea_topic").value)

        fix_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.fix_pub = self.create_publisher(
            NavSatFix, str(self.get_parameter("fix_topic").value), fix_qos
        )
        self.vel_pub = self.create_publisher(
            TwistStamped, str(self.get_parameter("vel_topic").value), fix_qos
        )
        self.heading_pub = self.create_publisher(
            Float32, str(self.get_parameter("heading_topic").value), fix_qos
        )
        self.fix_status_pub = self.create_publisher(
            String, str(self.get_parameter("fix_status_topic").value), fix_qos
        )
        self.nmea_pub = (
            self.create_publisher(String, self.nmea_topic, 10) if publish_nmea else None
        )

        self.nmea = NmeaParser()
        self.splitter = Um960StreamSplitter()
        self._lock = threading.Lock()
        # Set by destroy_node() so the reader thread stops reconnecting instead of
        # holding the device open after the node is gone.
        self._stopping = threading.Event()
        self._port: Optional[SerialPort] = None
        self._connected = False
        self._last_fix_time = 0.0
        self._last_publish = 0.0
        self._unhealthy_streak = 0
        # Cumulative bytes handed to the parsers; handy for diagnostics and tests.
        self._bytes_read = 0

        self.get_logger().info(
            "UM960 driver starting on %s @ %d baud (frame_id=%s)"
            % (self.port_name, self.baud, self.frame_id)
        )
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="um960_serial", daemon=True
        )
        self._reader_thread.start()
        # Publishing happens on the ROS executor thread so QoS/timers behave normally.
        self._timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self._publish_tick)

    # -- serial --------------------------------------------------------------------
    def _open_port(self) -> bool:
        if self._stopping.is_set():
            return False
        port = SerialPort(self.port_name, self.baud, self.serial_timeout)
        try:
            port.open()
        except SerialError as exc:
            self.get_logger().error(
                "Failed to connect to Unicore GPS on port %s with baudrate %d: %s"
                % (self.port_name, self.baud, exc)
            )
            return False
        self._port = port
        self._connected = True
        self.splitter.reset()
        self.get_logger().info(
            "Connected to Unicore GPS on port %s with baudrate %d."
            % (self.port_name, self.baud)
        )
        for command in self.config_commands:
            try:
                port.write_line(command)
                self.get_logger().info("sent receiver config: %s" % command)
            except SerialError as exc:
                self.get_logger().warn("failed to send config %r: %s" % (command, exc))
        return True

    def _close_port(self) -> None:
        if self._port is not None:
            self._port.close()
            self._port = None
        self._connected = False

    def _sleep_before_retry(self) -> None:
        """Wait out the reconnect delay, but wake immediately when shutting down."""
        self._stopping.wait(self.reconnect_period)

    def _reader_loop(self) -> None:
        while rclpy.ok() and not self._stopping.is_set():
            port = self._port
            if port is None or not port.is_open:
                if not self._open_port():
                    self._sleep_before_retry()
                    continue
                port = self._port
            try:
                # Read through the local reference: _close_port() can null out
                # self._port from destroy_node() while this thread is blocked.
                data = port.read()
            except SerialError as exc:  # pragma: no cover - unplugged cable
                if self._port is port:
                    self.get_logger().error("serial read failed: %s" % exc)
                    self._close_port()
                self._sleep_before_retry()
                continue
            if not data:
                continue
            self._consume(data)

    def _consume(self, data: bytes) -> None:
        """Feed raw serial bytes to the mixed ASCII/binary stream splitter."""
        for kind, value in self.splitter.feed(data):
            if kind == "line":
                self._consume_line(value)
            else:
                self._apply_frame(value)
        # Bumped last, once the bytes have actually been parsed, so an observer can
        # treat it as "the reader has caught up with N bytes" rather than "has seen
        # N bytes".
        self._bytes_read += len(data)

    def _apply_frame(self, frame: Dict[str, object]) -> None:
        if frame.get("kind") == "bestnav":
            self._apply_bestnav(frame)
        elif frame.get("kind") == "version":
            with self._lock:
                self.nmea.fix["version"] = frame["version"]

    def _consume_line(self, line: str) -> None:
        if self.nmea_pub is not None:
            self.nmea_pub.publish(String(data=line))
        if not line.startswith("$"):
            return
        if not nmea_checksum_ok(line):
            return
        with self._lock:
            message_id = self.nmea.feed(line)
            if message_id is None:
                return
            position_bearing = message_id in FIX_BEARING_IDS or message_id.endswith(
                FIX_BEARING_SUFFIXES
            )
            # A position-bearing ASCII log keeps /fix alive when no binary BESTNAV
            # is being streamed. Sentences that report "no fix" (GGA quality 0)
            # deliberately do not, so /fix stops instead of repeating a dead one.
            fresh = position_bearing and self.nmea.has_fix
        if fresh:
            self._mark_fix_time()

    # -- fix construction ----------------------------------------------------------
    def _mark_fix_time(self) -> None:
        self._last_fix_time = time.monotonic()

    def _apply_bestnav(self, frame: Dict[str, object]) -> None:
        """Fold a Unicore binary/ASCII BESTNAV solution into the NMEA fix state."""
        if int(frame.get("solution_status", 1)) != SOL_COMPUTED:
            return
        with self._lock:
            fix = self.nmea.fix
            fix["lat"] = frame["latitude"]
            fix["lon"] = frame["longitude"]
            fix["altitude"] = frame["height"]
            fix["lat_sigma"] = frame["lat_sigma"]
            fix["lon_sigma"] = frame["lon_sigma"]
            fix["alt_sigma"] = frame["height_sigma"]
            fix["fix_age"] = frame["diff_age"]
            fix["solution_age"] = frame["solution_age"]
            fix["undulation"] = frame.get("undulation")
            fix["datum_id"] = frame.get("datum_id")
            fix["num_sats"] = frame["num_sats"]
            fix["num_sats_used"] = frame["num_sats_used"]
            fix["station_id"] = frame["station_id"] or fix.get("station_id", "")
            pos_type = int(frame["position_type"])
            fix["position_type"] = bestnav_position_type_name(pos_type)
            fix["solution_status"] = bestnav_solution_status_name(int(frame["solution_status"]))
            fix["velocity_type"] = bestnav_velocity_type_name(int(frame["velocity_type"]))
            fix["horizontal_speed"] = frame["horizontal_speed"]
            fix["vertical_speed"] = frame["vertical_speed"]
            fix["track_ground"] = frame["track_ground"]
            fix["speed_mps"] = frame["horizontal_speed"]
            fix["course_deg"] = frame["track_ground"]
            fix["gga_sentence"] = "BESTNAV"
            # The position type implies the fix quality flag used for NavSatStatus:
            # integer solutions map to GGA quality 4, everything else to 1.
            fix["quality"] = 4 if pos_type in FIXED_POS_TYPES else 1
            fix["quality_label"] = fix["position_type"]
            fix["status"] = "STATUS_FIX"
        self._mark_fix_time()

    def _build_fix(self) -> Optional[NavSatFix]:
        fix = self.nmea.fix
        lat = fix.get("lat")
        lon = fix.get("lon")
        if lat is None or lon is None:
            return None
        if abs(lat) > self.sane_lat or abs(lon) > self.sane_lon:
            self.get_logger().warn("coords not sane (lat=%s lon=%s), skip publish." % (lat, lon))
            return None

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(fix.get("altitude") or 0.0)

        quality = int(fix.get("quality") or 0)
        msg.status.status = STATUS_BY_QUALITY.get(quality, NavSatStatus.STATUS_FIX)
        msg.status.service = (
            NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        )

        lat_sigma = fix.get("lat_sigma")
        lon_sigma = fix.get("lon_sigma")
        alt_sigma = fix.get("alt_sigma")
        if lat_sigma and lon_sigma and alt_sigma:
            cov = sigma_to_covariance(float(lat_sigma), float(lon_sigma), float(alt_sigma))
        else:
            cov = hdop_to_covariance(float(fix.get("hdop") or 0.0), int(fix.get("num_sats") or 0))
        if not any(cov):
            # No usable accuracy information: advertise "no covariance" rather than
            # a fictitious perfect fix.
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            msg.position_covariance = [0.0] * 9
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            msg.position_covariance = cov
        return msg

    def _build_twist(self) -> Optional[TwistStamped]:
        fix = self.nmea.fix
        speed = fix.get("horizontal_speed")
        if speed is None:
            speed = fix.get("speed_mps")
        if speed is None:
            return None
        course = fix.get("track_ground")
        if course is None:
            course = fix.get("course_deg")
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        speed = float(speed)
        # Ground track is published in the GPS frame; sign it so that travelling
        # southward yields a negative body-x velocity (mower convention).
        if course is not None and not (0.0 <= float(course) < 180.0):
            speed = -speed
        msg.twist.linear.x = speed
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = 0.0
        return msg

    def _build_heading(self) -> Optional[Float32]:
        fix = self.nmea.fix
        # GPHPR is the receiver's own heading estimate and wins when present.
        heading = fix.get("heading_deg")
        if heading is None:
            heading = fix.get("track_ground")
            if heading is None:
                heading = fix.get("course_deg")
        if heading is None:
            return None
        # Normalise into [0, 360); both GPHPR and the track angle can arrive negative.
        return Float32(data=float(heading) % 360.0)

    def _build_status(self) -> str:
        fix = self.nmea.fix
        connected = "connected" if self._connected else "disconnected"
        age = time.monotonic() - self._last_fix_time if self._last_fix_time else float("inf")
        parts: List[str] = ["um960=%s" % connected]
        position_type = fix.get("position_type")
        label = fix.get("quality_label")
        if position_type:
            parts.append("solution=%s" % position_type)
        elif label:
            parts.append("quality=%s" % label)
        if fix.get("solution_status"):
            parts.append("sol_status=%s" % fix["solution_status"])
        if fix.get("velocity_type"):
            parts.append("vel_type=%s" % fix["velocity_type"])
        if fix.get("num_sats"):
            used = fix.get("num_sats_used") or fix["num_sats"]
            parts.append("sats=%s/%s" % (used, fix["num_sats"]))
        if fix.get("hdop"):
            parts.append("hdop=%.2f" % float(fix["hdop"]))
        if fix.get("fix_age") is not None:
            parts.append("diff_age=%.1fs" % float(fix["fix_age"]))
        if fix.get("station_id"):
            parts.append("station=%s" % fix["station_id"])
        ref = fix.get("reference_station_status")
        if ref:
            parts.append("ref=%s" % ref)
        if fix.get("fix_type"):
            parts.append("gsa=%s" % GSA_FIX_TYPE.get(int(fix["fix_type"]), "?"))
        if age == float("inf"):
            parts.append("no fix yet")
        else:
            parts.append("age=%.2fs" % age)
        return " ".join(parts)

    def _publish_tick(self) -> None:
        now = time.monotonic()
        if now - self._last_publish < (1.0 / max(self.publish_rate, 1.0)) * 0.9:
            return
        self._last_publish = now
        with self._lock:
            fix_msg = self._build_fix()
            twist_msg = None if fix_msg is None else self._build_twist()
            heading_msg = None if fix_msg is None else self._build_heading()
            status = self._build_status()

        if self._last_fix_time and (now - self._last_fix_time) > self.fix_timeout:
            # Stop publishing the fix, but keep /fix_status alive so an operator can
            # see "the receiver went quiet" rather than an empty graph.
            self._unhealthy_streak += 1
            if self._unhealthy_streak == 1:
                self.get_logger().warn(
                    "no fresh UM960 fix for %.1fs, stopping /fix" % self.fix_timeout
                )
            self.fix_status_pub.publish(String(data="STALE " + status))
            return
        self._unhealthy_streak = 0

        if fix_msg is not None:
            self.fix_pub.publish(fix_msg)
        if twist_msg is not None:
            self.vel_pub.publish(twist_msg)
        if heading_msg is not None:
            self.heading_pub.publish(heading_msg)
        self.fix_status_pub.publish(String(data=status))

    def destroy_node(self) -> bool:
        # Stop the reader thread before closing the port so it cannot reopen the
        # device behind our back, then let it wind down.
        self._stopping.set()
        self._close_port()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = Um960Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
