"""Airseekers Tron control-support nodes (ROS 2 Jazzy).

MowgliNext-derived patterns: cmd_vel slew limiting with watchdog, wheel-slip
(dig) detection gated on RTK-fixed, and at-rest IMU bias calibration.
"""

__all__ = ['cmd_vel_slew', 'slip_detector', 'imu_cal']
