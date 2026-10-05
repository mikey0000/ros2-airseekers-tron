"""OpenCV fallback camera driver (``mower_cameras/camera_node``) — both outputs opt-in.

Canonical producers live elsewhere; this file only starts what you explicitly enable:

* ``enable_rear:=true``  -> ``/rear_camera/image_raw`` (bgr8, MJPEG 1920x1080@30 decoded by
  OpenCV) + ``/rear_camera/camera_info``. ``cameras.launch.py rear_driver:=opencv``
  includes this file with that set; do NOT also run the v4l2_camera rear node
  (``rear_driver:=v4l2``) -- one process per V4L2 device.
* ``enable_stereo:=true`` -> ``/vio/left/image_raw`` + ``/vio/right/image_raw`` from the
  Metoak side-by-side stereo (``stereo_device``, default ``/dev/video11``). Debug fallback
  only: ``stereo_vio_bridge`` (``launch/vio.launch.py``) is the canonical producer of the
  same topics on the same device.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('enable_rear', default_value='false',
                              description='Publish /rear_camera/* from the OpenCV MJPEG path.'),
        DeclareLaunchArgument('enable_stereo', default_value='false',
                              description='Debug: publish /vio/{left,right}/image_raw '
                                          '(never together with stereo_vio_bridge).'),
        DeclareLaunchArgument('rear_device', default_value='/dev/rear_camera'),
        DeclareLaunchArgument('rear_width', default_value='1920'),
        DeclareLaunchArgument('rear_height', default_value='1080'),
        DeclareLaunchArgument('rear_fps', default_value='30.0'),
        DeclareLaunchArgument('stereo_device', default_value='/dev/video11'),
        DeclareLaunchArgument('publish_compressed', default_value='false'),
        DeclareLaunchArgument(
            'rear_camera_info_file',
            default_value=PathJoinSubstitution([FindPackageShare('mower_bringup'), 'config',
                                                'cameras', 'rear_camera_info.yaml'])),
    ]
    node = Node(
        package='mower_cameras',
        executable='camera_node',
        name='mower_cameras',
        output='screen',
        parameters=[{
            'enable_rear': ParameterValue(lc('enable_rear'), value_type=bool),
            'enable_stereo': ParameterValue(lc('enable_stereo'), value_type=bool),
            'rear_device': lc('rear_device'),
            'rear_width': ParameterValue(lc('rear_width'), value_type=int),
            'rear_height': ParameterValue(lc('rear_height'), value_type=int),
            'rear_fps': ParameterValue(lc('rear_fps'), value_type=float),
            'rear_camera_info_file': lc('rear_camera_info_file'),
            'stereo_device': lc('stereo_device'),
            'width': 640,
            'height': 480,
            'fps': 10.0,
            'publish_compressed': ParameterValue(lc('publish_compressed'), value_type=bool),
        }],
    )
    return LaunchDescription(args + [node])
