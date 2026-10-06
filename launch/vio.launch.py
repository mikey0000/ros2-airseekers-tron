"""Visual-inertial odometry producers: OpenVINS ov_msckf + vio_odom_bridge (docs/vio.md).

    /vio/{left,right}/image_raw  (mower_cameras/stereo_cam, or stereo_vio_bridge capture)
    /imu/data                    (wit_imu_driver, 100 Hz)
        -> ov_msckf/run_subscribe_msckf -> /ov_msckf/odomimu
        -> stereo_vio_bridge/vio_odom_bridge -> /odometry/vio (odom frame, body twist)
        -> mower_localization/vio_gate (launch/nav2.launch.py vio:=true) -> EKF odom2

Included by launch/mower.launch.py only with vio:=true (default false). ov_msckf is NOT in
the mower image yet: build it with scripts/build_openvins_mower.sh (stack stopped).

Arguments:
    capture     stereo_cam (default): images come from mower_cameras/stereo_cam, which
                cameras.launch.py already runs (one device owner, shared with
                stereo_depth / the GUI). bridge: start the legacy stereo_vio_bridge
                capture node instead (mower.launch.py then tells cameras.launch.py
                not to start stereo_cam).
    config_dir  OpenVINS config directory. config/vio_wit (default) = WIT IMU on
                /imu/data; config/vio = Metoak ICM-40608 on /vio/imu (not exposed).
    cov_scale   multiplier on OpenVINS' velocity variance (VIO weight in the EKF).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    stack_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    LC = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('capture', default_value='stereo_cam',
                              description='stereo_cam | bridge (who owns the stereo device).'),
        DeclareLaunchArgument('config_dir',
                              default_value=os.path.join(stack_root, 'config', 'vio_wit'),
                              description='OpenVINS config directory.'),
        DeclareLaunchArgument('cov_scale', default_value='10.0',
                              description='VIO velocity variance multiplier.'),
        Node(
            package='stereo_vio_bridge',
            executable='stereo_vio_bridge',
            name='stereo_vio_bridge',
            output='screen',
            condition=IfCondition(PythonExpression(["'", LC('capture'), "' == 'bridge'"])),
        ),
        Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            name='run_subscribe_msckf',
            namespace='ov_msckf',
            output='screen',
            parameters=[{
                # OpenVINS resolves the kalibr_* siblings relative to this file.
                'config_path': PathJoinSubstitution([LC('config_dir'),
                                                     'estimator_config.yaml']),
                'verbosity': 'WARNING',
                # Never touch /tf: the EKF owns odom -> base_link.
                'publish_global_to_imu_tf': False,
                'publish_calibration_tf': False,
                'use_stereo': True,
                'max_cameras': 2,
                'save_total_state': False,
            }],
        ),
        Node(
            package='stereo_vio_bridge',
            executable='vio_odom_bridge',
            name='vio_odom_bridge',
            output='screen',
            parameters=[{
                'input_topic': '/ov_msckf/odomimu',
                'output_topic': '/odometry/vio',
                'cov_scale': ParameterValue(LC('cov_scale'), value_type=float),
            }],
        ),
    ])
