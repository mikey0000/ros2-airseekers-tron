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

OA cameras (verified 2026-10-06): GC2093 -> rkcif -> rkisp ``rkisp_mainpath``. These are
**V4L2 multi-planar** capture nodes; ``v4l2_camera`` 0.6 (Humble) only speaks single-planar
(it lists no formats, requests "0x0 UYVY" -> EINVAL, then "Failed mapping device memory").
``oa_driver:=mower_cameras`` (default) runs ``mower_cameras/v4l2_cam`` (pure-Python mmap
reader, mplane-aware) instead. The media graph is already linked at boot and the host's
``rkaiq_3A.service`` (``/usr/bin/rkaiq_3A_server``) runs AE/AWB; nothing else is needed on
the host (``scripts/setup_cameras_device.sh check`` verifies it). Sensor rate is 30 fps;
``oa_fps`` caps publishing (default 10; vendor: UYVY 1920x1080 @ 15 fps). Frames are only
converted while somebody subscribes (``oa_on_demand``); ``oa_publish_width`` optionally
decimates before the conversion. ``det_ros``/``seg_ros``
consume ``/{left,right}_oa_camera/image_raw`` (bgr8).

Rear camera: UVC webcam (32e6:9221), MJPG only (1920x1080 .. 640x480, 30 fps).
``v4l2_camera`` cannot decode MJPEG. ``rear_driver``:

* ``opencv`` (default; historical name): ``mower_cameras/camera_node``
  (``launch/camera.launch.py``) with a threaded capture (``rear_backend`` v4l2 = pure-Python
  mmap reader, the JPEG is passed through to ``image_raw/compressed``).
* ``v4l2``: ``v4l2_camera`` -- does NOT work on this webcam (no YUYV mode); kept for other
  hardware only.
* ``none``: no rear camera.

Front Metoak stereo (``stereo:=true``, default): ``mower_cameras/stereo_cam`` reads the
side-by-side YUYV 1280x480 frame of ``/dev/video22`` (``/dev/videoIsp``; ``/dev/video11`` is
the hardware-disparity stream of ``stereo_depth``) and publishes ``/vio/left/image_raw`` +
``/vio/right/image_raw`` as a synchronised mono8 pair (640x480 each, both eyes from the same
buffer, same stamp = V4L2 buffer time, ``stereo_fps`` = 10 Hz average) plus
``/vio/right/image_color`` (bgr8, ``stereo_color_fps``, for det_ros); on demand: nothing is
copied unless somebody subscribes. ``stereo_imu:=true`` (default) adds ``mower_cameras/stereo_imu``
(Metoak ICM-40608 via IIO -> ``/stereo_imu/data``, 200 Hz; needs
``scripts/setup_stereo_host.sh`` on the host once per boot). ``stereo_vio_bridge`` (``launch/vio.launch.py``) is the
VIO producer of the same topics on the same device, so the two are mutually exclusive:
pass ``vio:=true`` whenever vio.launch.py runs and stereo_cam is not started.

Front stereo hardware depth (``stereo_depth:=true``): ``mower_cameras/stereo_depth`` ->
``/stereo_depth/points``, noise-filtered and ground-removed (``stereo_ground_fit``, RANSAC
ground plane per frame; ``stereo_persist_frames`` temporal filter); stats on
``/stereo_depth/stats``. Whether Nav2 USES that cloud is ``stereo_costmap`` in
``mower.launch.py`` (it rewrites the nav2 params): ``stereo_costmap:=false`` leaves the
stereo/det_range sources out of the local costmap (bumper-only) while this node keeps
running for det_range / the GUI.

``web_video_server`` (default ``true``) is an MJPEG HTTP server on ``video_port`` (8080):
``http://<mower>:8080/stream?topic=/rear_camera/image_raw`` (also the source for the
RTSP/WebRTC relay in ``docker/docker-compose.video.yml``). The GUI reverse-proxies it as
``:4006/api/cameras/<id>/stream`` and asks for ``quality=50`` at 640x360 and caps delivery
at 5 fps. web_video_server 3.x has no node-level quality/fps parameters (``quality``,
``width``, ``height`` are per-request URL args, default quality 95), so direct
``:8080`` clients should pass them too, plus ``qos_profile=sensor_data`` (the camera
publishers are best-effort; the default reliable subscription gets no frames). See docs/cameras_and_video.md.
"""
import typing

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

LC = LaunchConfiguration
List = typing.List


def _info_url(name):
    return ParameterValue(
        ['file://', PathJoinSubstitution([LC('camera_info_dir'), f'{name}_info.yaml'])],
        value_type=str)


def _oa_cam(name, condition):
    """OA camera via mower_cameras/v4l2_cam (multi-planar capable)."""
    return Node(
        package='mower_cameras',
        executable='v4l2_cam',
        namespace=name,                 # -> /<name>/image_raw, /<name>/camera_info
        name=name,
        output='screen',
        condition=condition,
        parameters=[{
            'video_device': LC(f'{name}_device'),
            'width': ParameterValue(LC('oa_width'), value_type=int),
            'height': ParameterValue(LC('oa_height'), value_type=int),
            'pixel_format': LC('oa_pixel_format'),
            'fps': ParameterValue(LC('oa_fps'), value_type=float),
            'frame_id': name,
            'camera_info_file': PathJoinSubstitution([LC('camera_info_dir'),
                                                      f'{name}_info.yaml']),
            'publish_compressed': ParameterValue(LC('oa_compressed'), value_type=bool),
            'publish_width': ParameterValue(LC('oa_publish_width'), value_type=int),
            'publish_on_demand': ParameterValue(LC('oa_on_demand'), value_type=bool),
        }],
    )


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
            # integer array [w, h] (v4l2_camera rejects strings). NB the "0x0 UYVY" in the
            # v4l2_camera log on the OA nodes is NOT this parameter: it is the empty
            # single-planar G_FMT of an mplane device (see module docstring).
            'image_size': ParameterValue(
                ['[', LC(f'{size_prefix}_width'), ',', LC(f'{size_prefix}_height'), ']'],
                value_type=List[int]),
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
        DeclareLaunchArgument('rear_driver', default_value='opencv',
                              choices=['v4l2', 'opencv', 'none'],
                              description='Rear camera producer (see module docstring).'),
        DeclareLaunchArgument('oa_driver', default_value='mower_cameras',
                              choices=['mower_cameras', 'v4l2_camera'],
                              description='OA producer: mower_cameras/v4l2_cam (works on the '
                                          'rkisp mplane nodes) | v4l2_camera (does not).'),
        DeclareLaunchArgument('oa_fps', default_value='10.0',
                              description='OA publish-rate cap (sensor runs 30 fps; det_ros '
                                          'max_rate_hz is 10).'),
        DeclareLaunchArgument('oa_publish_width', default_value='960',
                              description='OA image_raw width (integer decimation of the '
                                          'capture before colour conversion, camera_info '
                                          'scaled to match; 960 = ~1/4 CPU). 0 = native '
                                          '1920x1080. det_ros accepts any size; '
                                          'obstacle_guard follows camera_info.'),
        DeclareLaunchArgument('oa_on_demand', default_value='true',
                              description='OA: convert/publish only while image_raw, '
                                          'compressed or camera_info has subscribers.'),
        DeclareLaunchArgument('oa_compressed', default_value='false'),
        DeclareLaunchArgument('left_oa_camera_device', default_value='/dev/left_oa_camera'),
        DeclareLaunchArgument('right_oa_camera_device', default_value='/dev/right_oa_camera'),
        DeclareLaunchArgument('rear_camera_device', default_value='/dev/rear_camera'),
        # OA: recovered vendor config (UYVY 1920x1080, matches config/cameras/*_info.yaml)
        DeclareLaunchArgument('oa_width', default_value='1920'),
        DeclareLaunchArgument('oa_height', default_value='1080'),
        DeclareLaunchArgument('oa_pixel_format', default_value='UYVY'),
        # Rear: calibration is 1920x1080 (camera_info is rescaled for 16:9 modes; 4:3 modes
        # such as 640x480 are a crop of the sensor and use rear_camera_info_640x480.yaml,
        # see mower_cameras.camera_node.mode_info_path). 640x480 @ 10 Hz is what docking
        # (ArUco, mower_docking) and the GUI need; it also shows more of the dock vertically
        # than 16:9 and costs ~1/4 of the 720p JPEG decode.
        DeclareLaunchArgument('rear_width', default_value='640'),
        DeclareLaunchArgument('rear_height', default_value='480'),
        DeclareLaunchArgument('rear_fps', default_value='10.0',
                              description='Rear publish-rate cap (webcam runs 30 fps).'),
        DeclareLaunchArgument('rear_compressed', default_value='true',
                              description='Also publish /rear_camera/image_raw/compressed '
                                          '(MJPEG passthrough, ~free).'),
        DeclareLaunchArgument('rear_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('output_encoding', default_value='bgr8',
                              description='bgr8 (NPU nodes decode bgr8 for free) | rgb8 | '
                                          'yuv422 (passthrough, cheapest driver CPU).'),
        DeclareLaunchArgument('camera_info_dir',
                              default_value=PathJoinSubstitution([share, 'config', 'cameras']),
                              description='Directory holding <camera>_info.yaml.'),
        DeclareLaunchArgument(
            'camera_dds_profile',
            default_value=PathJoinSubstitution([share, 'config', 'cameras',
                                                'fastdds_no_shm.xml']),
            description='Fast DDS profile for the camera publishers (UDP-only, no /dev/shm; '
                        'without it 6 MB images arrive at ~1/3 rate). "" = RMW default.'),
        DeclareLaunchArgument('web_video_server', default_value='true',
                              description='Start web_video_server (MJPEG over HTTP; the GUI '
                                          'camera page proxies it via :4006/api/cameras). '
                                          'Idle cost is ~0: it only subscribes/encodes while a '
                                          'client is streaming.'),
        DeclareLaunchArgument('video_port', default_value='8080'),
        DeclareLaunchArgument('stereo', default_value='true',
                              description='Front Metoak stereo via mower_cameras/stereo_cam '
                                          '(/vio/{left,right}/image_raw, bgr8, on demand). '
                                          'Ignored when vio:=true.'),
        DeclareLaunchArgument('vio', default_value='false',
                              description='true = stereo_vio_bridge (launch/vio.launch.py) '
                                          'owns the stereo device: do not start stereo_cam.'),
        DeclareLaunchArgument('stereo_device', default_value='/dev/video22'),
        DeclareLaunchArgument('stereo_width', default_value='1280',
                              description='Side-by-side width (each eye = half).'),
        DeclareLaunchArgument('stereo_height', default_value='480'),
        DeclareLaunchArgument('stereo_fps', default_value='10.0',
                              description='stereo_cam mono8 pair rate (average, from a ~25 Hz '
                                          'source).'),
        DeclareLaunchArgument('stereo_color', default_value='true',
                              description='stereo_cam: also publish <eye>/image_color (bgr8) '
                                          'for det_ros (only while subscribed).'),
        DeclareLaunchArgument('stereo_color_fps', default_value='5.0'),
        DeclareLaunchArgument('stereo_imu', default_value='true',
                              description='mower_cameras/stereo_imu: Metoak ICM-40608 (IIO) -> '
                                          '/stereo_imu/data.'),
        DeclareLaunchArgument('stereo_imu_rate', default_value='200.0'),
        DeclareLaunchArgument('stereo_depth', default_value='true',
                              description='mower_cameras/stereo_depth: Metoak hardware disparity '
                                          '(/dev/video11) -> /stereo_depth/points (Nav2 local '
                                          'costmap obstacle source) + depth image.'),
        DeclareLaunchArgument('stereo_depth_fps', default_value='10.0'),
        DeclareLaunchArgument('stereo_ground_fit', default_value='true',
                              description='stereo_depth: per-frame RANSAC ground plane, points '
                                          '< 0.12 m above it dropped from ~/points.'),
        DeclareLaunchArgument('stereo_persist_frames', default_value='2',
                              description='stereo_depth: voxel must be seen in N consecutive '
                                          'frames to be published.'),
    ]

    def _oa(name):
        def on(driver):
            return IfCondition(PythonExpression(
                ["'", LC(name), "' == 'true' and '", LC('oa_driver'), f"' == '{driver}'"]))
        return [_oa_cam(name, on('mower_cameras')),
                _v4l2_cam(name, 'oa', 'oa_pixel_format', on('v4l2_camera'))]

    oa_nodes = _oa('left_oa_camera') + _oa('right_oa_camera')
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
            'rear_fps': LC('rear_fps'),
            'publish_compressed': LC('rear_compressed'),
            'rear_camera_info_file': PathJoinSubstitution(
                [LC('camera_info_dir'), 'rear_camera_info.yaml']),
        }.items(),
        condition=LaunchConfigurationEquals('rear_driver', 'opencv'),
    )

    stereo = Node(
        package='mower_cameras',
        executable='stereo_cam',
        name='stereo_cam',
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LC('stereo'), "' == 'true' and '", LC('vio'), "' != 'true'"])),
        parameters=[{
            'video_device': LC('stereo_device'),
            'width': ParameterValue(LC('stereo_width'), value_type=int),
            'height': ParameterValue(LC('stereo_height'), value_type=int),
            'pixel_format': 'YUYV',
            'fps': ParameterValue(LC('stereo_fps'), value_type=float),
            'publish_on_demand': True,
            'publish_color': ParameterValue(LC('stereo_color'), value_type=bool),
            'color_fps': ParameterValue(LC('stereo_color_fps'), value_type=float),
            # Right eye camera_info: det_ros runs on /vio/right/image_color and obstacle_guard
            # sizes its danger zone from camera_info.
            'left_camera_info_file': PathJoinSubstitution(
                [LC('camera_info_dir'), 'left_stereo_camera_info.yaml']),
            'right_camera_info_file': PathJoinSubstitution(
                [LC('camera_info_dir'), 'right_stereo_camera_info.yaml']),
        }],
    )

    stereo_imu = Node(
        package='mower_cameras',
        executable='stereo_imu',
        name='stereo_imu',
        output='screen',
        condition=IfCondition(LC('stereo_imu')),
        parameters=[{'rate': ParameterValue(LC('stereo_imu_rate'), value_type=float)}],
    )

    stereo_depth = Node(
        package='mower_cameras',
        executable='stereo_depth',
        name='stereo_depth',
        output='screen',
        condition=IfCondition(LC('stereo_depth')),
        parameters=[{'fps': ParameterValue(LC('stereo_depth_fps'), value_type=float),
                     'ground_plane_fit': ParameterValue(LC('stereo_ground_fit'),
                                                        value_type=bool),
                     'persist_frames': ParameterValue(LC('stereo_persist_frames'),
                                                      value_type=int)}],
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
            # 1: web_video_server creates image_transport subscribers on the HTTP
            # threads and pluginlib's ClassLoader is not thread-safe; with 2 threads,
            # concurrent stream/snapshot requests (the GUI Perception page opens
            # several at once) left it permanently failing with "Unable to load
            # plugin for transport 'image_transport/raw_sub'" (seen 2026-10-06).
            # Streaming itself is asynchronous, one thread serves all viewers.
            'server_threads': 1,
            'ros_threads': 2,
            'default_stream_type': 'mjpeg',
            # Frame re-publish rate for a stalled topic (-1 = never): avoids
            # re-encoding the last image when a camera stops.
            'publish_rate': -1.0,
            'verbose': False,
        }],
    )

    # Scoped: only the camera publishers get the large-SHM Fast DDS profile.
    cameras = GroupAction(scoped=True, forwarding=True, actions=[
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', LC('camera_dds_profile'),
                               condition=IfCondition(PythonExpression(
                                   ["'", LC('camera_dds_profile'), "' != ''"]))),
    ] + oa_nodes + [rear_v4l2, rear_opencv, stereo, stereo_depth])

    return LaunchDescription(args + [cameras, stereo_imu, video])
