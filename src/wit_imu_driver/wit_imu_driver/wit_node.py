#!/usr/bin/env python3
import serial
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

import tf2_ros
from geometry_msgs.msg import TransformStamped


class WitImuNode(Node):
    def __init__(self):
        super().__init__('wit_imu_driver')

        # Parameters
        self.declare_parameter('port', '/dev/serial_imu')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calibration_mode', False)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.calibration_mode = self.get_parameter('calibration_mode').get_parameter_value().bool_value

        self.get_logger().info(
            f'WIT IMU Driver starting on {self.port} at {self.rate}Hz, '
            f'frame_id={self.frame_id}, calibration={self.calibration_mode}'
        )

        # Publishers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.imu_pub = self.create_publisher(Imu, '/imu', qos)
        self.temp_pub = self.create_publisher(Float32, '/imu/temperature_c', qos)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # IMU message
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.frame_id
        # orientation covariance (identity-like, unset = zeros)
        self.imu_msg.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.imu_msg.angular_velocity_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.imu_msg.linear_acceleration_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Temperature message
        self.temp_msg = Float32()
        self.temp_msg.data = 0.0

        # Serial connection
        self.ser = None
        self.buffer = bytes()

        # Tracking for multi-packet axes (accel/gyro/mag span 3 packets)
        self.accel = {'x': None, 'y': None, 'z': None}
        self.gyro = {'x': None, 'y': None, 'z': None}
        self.mag = {'x': None, 'y': None, 'z': None}
        self.angle = {'p': None, 'y': None, 'r': None}
        self.packet_count_by_type = {0x51: 0, 0x52: 0, 0x53: 0, 0x54: 0, 0x55: 0}

        # Timer for publishing at specified rate
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(
                self.port,
                115200,
                timeout=0.01
            )
            self.get_logger().info(f'Serial port {self.port} opened at 115200')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            self.ser = None

    def parse_packet(self, data):
        """Parse 0x55 framed packets: (0x55, type, lo, hi, sum)

        Packet format: sync(0x55) | type | lo | hi | sum_check
        Checksum: sum_check = (type + lo + hi) & 0xFF
        """
        while len(data) >= 5:
            # Look for sync byte 0x55
            if data[0] != 0x55:
                data = data[1:]
                continue

            # We have at least 1 byte (sync), need at least 5 total
            if len(data) < 5:
                break

            msg_type = data[1]
            lo = data[2]
            hi = data[3]
            checksum = data[4]

            # Verify checksum: sum of type+lo+hi should match checksum
            computed_sum = (msg_type + lo + hi) & 0xFF
            if computed_sum != checksum:
                # Bad checksum — skip this sync byte and try next
                data = data[1:]
                continue

            # Valid packet! Consume all 5 bytes
            data = data[5:]

            # Process by type
            if msg_type == 0x51:  # Accelerometer (+/-16g)
                # This packet contains one axis; we track 3 per axes
                # JY61P: lo/hi are 16-bit signed little-endian
                value = ((hi << 8) | lo) if ((hi << 8) | lo) < 32768 else ((hi << 8) | lo) - 65536
                # Determine which axis based on packet count
                mod = self.packet_count_by_type[0x51] % 3
                axis = ['x', 'y', 'z'][mod]
                self.accel[axis] = value
                self.packet_count_by_type[0x51] += 1
                if all(self.accel.values()):
                    self.fill_accel_from_tracked()

            elif msg_type == 0x52:  # Gyro (+/-2000 dps)
                value = ((hi << 8) | lo) if ((hi << 8) | lo) < 32768 else ((hi << 8) | lo) - 65536
                mod = self.packet_count_by_type[0x52] % 3
                axis = ['x', 'y', 'z'][mod]
                self.gyro[axis] = value
                self.packet_count_by_type[0x52] += 1
                if all(self.gyro.values()):
                    self.fill_gyro_from_tracked()

            elif msg_type == 0x53:  # Angle
                value = ((hi << 8) | lo) if ((hi << 8) | lo) < 32768 else ((hi << 8) | lo) - 65536
                mod = self.packet_count_by_type[0x53] % 3
                axis = ['p', 'y', 'r'][mod]  # roll, pitch, yaw
                self.angle[axis] = value
                self.packet_count_by_type[0x53] += 1
                if all(self.angle.values()):
                    self.fill_angle_from_tracked()

            elif msg_type == 0x54:  # Magnetometer
                value = ((hi << 8) | lo) if ((hi << 8) | lo) < 32768 else ((hi << 8) | lo) - 65536
                mod = self.packet_count_by_type[0x54] % 3
                axis = ['x', 'y', 'z'][mod]
                self.mag[axis] = value
                self.packet_count_by_type[0x54] += 1

            elif msg_type == 0x55:  # Temperature
                # Temperature: 16-bit value, typical sensitivity ~1 LSB = 0.1°C
                # or raw/100.0. We'll use raw/100.0 convention for this driver.
                temp_raw = ((hi << 8) | lo) if ((hi << 8) | lo) < 32768 else ((hi << 8) | lo) - 65536
                temperature = temp_raw / 100.0
                # Apply calibration offset if enabled
                if self.calibration_mode:
                    temperature -= 25.0  # example calibration offset
                self.temp_msg.data = temperature

        return data

    def fill_accel_from_tracked(self):
        """Fill IMU accel fields from tracked packet data."""
        # Convert raw values to g: +/-16g range
        # Assuming 16-bit values with sensitivity: e.g., 2mg/LSB -> 0.002g/LSB
        # Or more commonly for JY61P: the raw 16-bit maps directly to +/-16g
        # where 32768 = 16g, so 1 LSB = 16/32768 g = 0.00048828125 g
        # But typically the datasheet says the raw value is already in g * 100 or similar.
        # Let's use: value_in_g = raw * (16.0 / 32768.0)
        scale = 16.0 / 32768.0
        self.imu_msg.linear_acceleration.x = self.accel['x'] * scale
        self.imu_msg.linear_acceleration.y = self.accel['y'] * scale
        self.imu_msg.linear_acceleration.z = self.accel['z'] * scale

    def fill_gyro_from_tracked(self):
        """Fill IMU gyro fields from tracked packet data.
        +/-2000 dps range: 1 LSB = 2000/32768 dps"""
        scale = 2000.0 / 32768.0
        # Convert dps to rad/s for ROS2 Imu message
        # 1 dps = pi/180 rad/s
        dps_to_rad_s = math.pi / 180.0
        self.imu_msg.angular_velocity.x = self.gyro['x'] * scale * dps_to_rad_s
        self.imu_msg.angular_velocity.y = self.gyro['y'] * scale * dps_to_rad_s
        self.imu_msg.angular_velocity.z = self.gyro['z'] * scale * dps_to_rad_s

    def fill_angle_from_tracked(self):
        """Store angle data as covariance or as linear acceleration placeholder.
        Angles are roll/pitch/yaw in degrees typically."""
        # For now, just log; angles could be converted to quaternion later
        self.get_logger().debug(
            f'Angles - roll: {self.angle["p"]}°, pitch: {self.angle["y"]}°, yaw: {self.angle["r"]}°'
        )

    def read_serial(self):
        """Read available bytes from serial."""
        if self.ser and self.ser.is_open:
            try:
                available = self.ser.in_waiting
                if available > 0:
                    data = self.ser.read(available)
                    self.buffer += data
                    return True
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                self.connect_serial()
        return False

    def timer_callback(self):
        # Read serial data
        self.read_serial()

        # Parse packets from buffer
        self.buffer = self.parse_packet(self.buffer)

        # Publish IMU and temperature
        self.publish_imu()

    def publish_imu(self):
        # Update header
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = self.frame_id

        # Reset acceleration/gyro if not fully tracked
        # (if we have partial data, publish what we have)
        has_data = (
            all(self.imu_msg.linear_acceleration[i] != 0.0 or True for i in range(3)) or
            self.packet_count_by_type[0x51] > 0
        )

        # Publish IMU message
        self.imu_pub.publish(self.imu_msg)

        # Publish temperature
        self.temp_pub.publish(self.temp_msg)

        # Publish transform
        self.publish_transform()

    def publish_transform(self):
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = self.frame_id
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().warn(f'TF publish error: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = WitImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()