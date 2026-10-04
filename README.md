# Airseekers Tron — ROS 2 (Humble) port stack

Baseline: MowgliNext (ROS 2 mower stack). Target: runs on the mower via Docker (Humble).

Source assets (read-only, do NOT modify):
- ../ros2_port_handoff/ (README.md first, then 00_docs, 01_mcu_protocol, 03_ros_interfaces, 10_ros_live_snapshot)
- ../mower_docs/ (02-ros2-migration.md, 05-maps-and-http-api.md, 10-hardware.md)

Write all new files under ros2_stack/ only.
Do NOT run anything on the actual mower (192.168.1.105); no apt changes, no flashing, no systemctl.
