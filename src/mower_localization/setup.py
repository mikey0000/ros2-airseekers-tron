from setuptools import find_packages, setup

package_name = 'mower_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
        ('share/' + package_name + '/config',
         ['config/ekf.yaml', 'config/ekf_vio.yaml', 'config/navsat.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 port team',
    maintainer_email='dev@todo.todo',
    description=(
        'Slam-less localization for the Airseekers Tron: a GNSS quality gate plus the '
        'robot_localization EKF / navsat_transform_node parameters that fuse wheel '
        'odometry, IMU and RTK fixes into the REP-105 frame tree.'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_gate = mower_localization.gps_gate:main',
            'heading_aligner = mower_localization.heading_aligner:main',
            'vio_gate = mower_localization.vio_gate:main',
        ],
    },
)