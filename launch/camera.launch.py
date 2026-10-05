"""Bring up the mower cameras (front stereo + rear UVC) via ``mower_cameras``.

Starts ``mower_cameras/camera_node``: the front Metoak stereo module
(/dev/videoSimor -> video11) is split into ``/vio/left`` and ``/vio/right``
(OpenVINS input) and the rear USB UVC camera (/dev/rear_camera -> video62)
publishes ``/rear_camera``. Defaults mirror
``base_cameras_config/mower_cameras.yaml`` (rear 1920x1080 @ 30 fps) and the
OpenVINS stereo config (640x480 @ 10 Hz); override via
``src/mower_cameras/config/cameras.yaml`` or the parameter dict below.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mower_cameras',
            executable='camera_node',
            name='mower_cameras',
            parameters=[{
                'stereo_device': '/dev/video11',
                'rear_device': '/dev/rear_camera',
                'width': 640,
                'height': 480,
                'fps': 10.0,
                'rear_width': 1920,
                'rear_height': 1080,
                'rear_fps': 30.0,
                'publish_compressed': True,
            }],
            output='screen',
        ),
    ])
