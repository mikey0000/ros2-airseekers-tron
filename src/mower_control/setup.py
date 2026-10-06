from setuptools import setup

package_name = 'mower_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael@example.com',
    description='Airseekers Tron control-support nodes for ROS 2 Humble: cmd_vel '
                'slew limiter with watchdog, RTK-gated wheel-slip (dig) detector, '
                'and at-rest IMU bias calibration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_slew = mower_control.cmd_vel_slew:main',
            'slip_detector = mower_control.slip_detector:main',
            'imu_cal = mower_control.imu_cal:main',
            'supervisor = mower_control.supervisor:main',
        ],
    },
)
