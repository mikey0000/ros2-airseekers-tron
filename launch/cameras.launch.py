"""Bring up the OA (obstacle-avoidance) + rear cameras via ``v4l2_camera``.

The left/right OA cameras feed ``det_ros`` (YOLOv8) and ``seg_ros`` (PP-LiteSeg); the
rear camera is recording/telemetry only. Devices are udev-symlinked to
``/dev/left_oa_camera``, ``/dev/right_oa_camera``, ``/dev/rear_camera`` (see
``scripts/99-mower-cameras.rules``).

Camera calibration recovered in ``ros2_port_handoff/08_calibration_identity/``; the
``camera_info`` yamls here are the same ``camera_info_manager`` format.

The OA sensors are GalaxyCore GC2093 (see ``docs/perception_vio.md``). Before launching,
run ``scripts/setup_camera_iq.sh`` to install ``gc2093_MY_default.json`` so rkaiq produces
correct colour/exposure.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def oa_cam(name, device, info):
    return Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name=name,
        namespace='',
        parameters=[{
            'video_device': device,
            'image_size': [1920, 1080],
            'pixel_format': 'UYVY',
            'output_encoding': 'bgr8',     # det/seg decode via cv_bridge (bgr8)
            'camera_info_url': f'file:///work/config/cameras/{info}',
            'camera_frame_id': name,
        }],
        output='screen',
    )


def generate_launch_description():
    return LaunchDescription([
        oa_cam('left_oa_camera', '/dev/left_oa_camera', 'left_oa_camera_info.yaml'),
        oa_cam('right_oa_camera', '/dev/right_oa_camera', 'right_oa_camera_info.yaml'),
        oa_cam('rear_camera', '/dev/rear_camera', 'rear_camera_info.yaml'),
    ])
