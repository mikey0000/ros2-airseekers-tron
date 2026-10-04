# WIT-Motion JY61P IMU Driver

## Packet Format

The JY61P transmits data in 0x55-framed packets with the structure:

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 |
|--------|--------|--------|--------|--------|
| 0x55 (sync) | type | lo | hi | sum_check |

**Checksum**: `sum_check = (type + lo + hi) & 0xFF`

If checksum doesn't match, skip the sync byte and look for the next valid packet.

## Packet Types

| Type | Description | Range |
|------|-------------|-------|
| `0x51` | Accelerometer (±16g) | 3 axes, 16-bit signed |
| `0x52` | Gyroscope (±2000 dps) | 3 axes, 16-bit signed |
| `0x53` | Angle (Euler angles) | 3 axes |
| `0x54` | Magnetometer | 3 axes |
| `0x55` | Temperature | °C |

## Parameter Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/serial_imu` | Serial port device |
| `rate` | double | `100.0` | Publishing rate in Hz |
| `frame_id` | string | `imu_link` | ROS frame identifier |
| `calibration_mode` | bool | `false` | Enable calibration mode |

## Operation

1. Node opens serial port at 115200 baud
2. Continuously reads packets, validating 0x55 sync and checksum
3. Parses data by type and populates IMU messages
4. Publishes `/imu` (`sensor_msgs/Imu`) at configured rate
5. Publishes `/imu/temperature_c` (`std_msgs/Float32`)
6. Broadcasts TF from `base_link` to `imu_link`

## Notes

- The exact mapping of packet data to IMU fields depends on the JY61P firmware version
- Successive packets of the same type typically provide X, Y, Z axes data
- Temperature sensor requires calibration offset (see `calibration_mode` param)
- If checksum failures exceed 10% of packets, check wiring and baud rate