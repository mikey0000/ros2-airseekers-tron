"""OA (obstacle-avoidance) + rear cameras via ``v4l2_camera``, plus the optional MJPEG feed.

Topics (one namespace per camera, ``v4l2_camera`` publishes ``image_raw`` + ``camera_info``):

=================  =====================  ==============================================
camera             device (udev symlink)  topics
=================  =====================  ==============================================
left_oa_camera     /dev/left_oa_camera    /left_oa_camera/image_raw, /left_oa_camera/camera_info
right_oa_camera    /dev/right_oa_camera   /right_oa_camera/image_raw, /right_oa_camera/camera_info
rear_camera        /dev/rear_camera       /rear_camera/image_raw, /rear_camera/camera_info
=================  =====================  ==============================================

Stock symlinks (``ros2_port_handoff/09_platform/udev-symlinks.txt``):
``left_oa_camera -> video53``, ``right_oa_camera -> video44``, ``rear_camera -> video62``.
The OA cameras are GC2093 MIPI sensors behind rkisp (run ``scripts/setup_camera_iq.sh``
first); vendor config ``base_cameras_config/mower_cameras.yaml``: UYVY 1920x1080 @ 15 fps.
``det_ros`` / ``seg_ros`` consume ``/{left,right}_oa_camera/image_raw``; ``output_encoding``
``bgr8`` makes the driver convert once instead of each NPU node converting UYVY.

Rear camera: the UVC webcam (32e6:9221) streams 1080p@30 only as MJPEG, which
``v4l2_camera`` 0.6 (Humble) cannot decode. ``rear_driver``:

* ``v4l2`` (default): ``v4l2_camera`` with ``rear_pixel_format`` (YUYV) at
  ``rear_width``x``rear_height`` -- UVC YUYV at 1080p is typically only ~5 fps over USB 2;
  check ``v4l2-ctl -d /dev/rear_camera --list-formats-ext`` and lower the size if needed.
* ``opencv``: ``mower_cameras`` (``launch/camera.launch.py``) opens the device as MJPEG
  and publishes the same topics at full rate (more CPU: JPEG decode).
* ``none``: no rear camera.

The Metoak front stereo is NOT here: ``stereo_vio_bridge`` in ``launch/vio.launch.py``
owns it (``/vio/{left,right}/image_raw``).

``web_video_server:=true`` adds an MJPEG HTTP server on ``video_port`` (8080):
``http://<mower>:8080/stream?topic=/rear_camera/image_raw`` (also the source for the
RTSP/WebRTC relay in ``docker/docker-compose.video.yml``). See docs/cameras_and_video.md.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

LC = LaunchConfiguration


def _info_url(name):
    return ParameterValue(
        ['file://', PathJoinSubstitution([LC('camera_info_dir'), f'{name}_info.yaml'])],
        value_type=str)


def _v4l2_cam(name, size_prefix, fmt_arg, condition):
    return Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace=name,                 # -> /<name>/image_raw, /<name>/camera_info
        name=name,
        output='screen',
        condition=condition,
        parameters=[{
            'video_device': LC(f'{name}_device'),
            'image_size': ParameterValue(
                ['[', LC(f'{size_prefix}_width'), ',', LC(f'{size_prefix}_height'), ']'],
                value_type=None),
            'pixel_format': LC(fmt_arg),
            'output_encoding': LC('output_encoding'),
            'camera_info_url': _info_url(name),
            'camera_frame_id': name,
        }],
    )


def generate_launch_description():
    share = FindPackageShare('mower_bringup')
    args = [
        DeclareLaunchArgument('left_oa_camera', default_value='true'),
        DeclareLaunchArgument('right_oa_camera', default_value='true'),
        DeclareLaunchArgument('rear_driver', default_value='v4l2',
                              choices=['v4l2', 'opencv', 'none'],
                              description='Rear camera producer (see module docstring).'),
        DeclareLaunchArgument('left_oa_camera_device', default_value='/dev/left_oa_camera'),
        DeclareLaunchArgument('right_oa_camera_device', default_value='/dev/right_oa_camera'),
        DeclareLaunchArgument('rear_camera_device', default_value='/dev/rear_camera'),
        # OA: recovered vendor config (UYVY 1920x1080, matches config/cameras/*_info.yaml)
        DeclareLaunchArgument('oa_width', default_value='1920'),
        DeclareLaunchArgument('oa_height', default_value='1080'),
        DeclareLaunchArgument('oa_pixel_format', default_value='UYVY'),
        # Rear: calibration is 1920x1080; v4l2_camera needs a raw (YUYV) mode.
        DeclareLaunchArgument('rear_width', default_value='1920'),
        DeclareLaunchArgument('rear_height', default_value='1080'),
        DeclareLaunchArgument('rear_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('output_encoding', default_value='bgr8',
                              description='bgr8 (NPU nodes decode bgr8 for free) | rgb8 | '
                                          'yuv422 (passthrough, cheapest driver CPU).'),
        DeclareLaunchArgument('camera_info_dir',
                              default_value=PathJoinSubstitution([share, 'config', 'cameras']),
                              description='Directory holding <camera>_info.yaml.'),
        DeclareLaunchArgument('web_video_server', default_value='false',
                              description='Start web_video_server (MJPEG over HTTP).'),
        DeclareLaunchArgument('video_port', default_value='8080'),
    ]

    left = _v4l2_cam('left_oa_camera', 'oa', 'oa_pixel_format', IfCondition(LC('left_oa_camera')))
    right = _v4l2_cam('right_oa_camera', 'oa', 'oa_pixel_format',
                      IfCondition(LC('right_oa_camera')))
    rear_v4l2 = _v4l2_cam('rear_camera', 'rear', 'rear_pixel_format',
                          LaunchConfigurationEquals('rear_driver', 'v4l2'))

    rear_opencv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch',
                                                            'camera.launch.py'])),
        launch_arguments={
            'enable_rear': 'true',
            'enable_stereo': 'false',
            'rear_device': LC('rear_camera_device'),
            'rear_width': LC('rear_width'),
            'rear_height': LC('rear_height'),
            'rear_camera_info_file': PathJoinSubstitution(
                [LC('camera_info_dir'), 'rear_camera_info.yaml']),
        }.items(),
        condition=LaunchConfigurationEquals('rear_driver', 'opencv'),
    )

    video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        condition=IfCondition(LC('web_video_server')),
        parameters=[{
            'port': ParameterValue(LC('video_port'), value_type=int),
            'address': '0.0.0.0',
            'server_threads': 2,
            'ros_threads': 2,
            'default_stream_type': 'mjpeg',
        }],
    )

    return LaunchDescription(args + [left, right, rear_v4l2, rear_opencv, video])
