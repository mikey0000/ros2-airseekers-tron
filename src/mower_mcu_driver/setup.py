from setuptools import setup

package_name = 'mower_mcu_driver'

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
    description='Airseekers Tron MCU serial driver for ROS 2 Humble '
                '(/dev/serial_mower protocol: battery, odometry, sensors, cmd_vel).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # the driver node
            'mcu_node = mower_mcu_driver.mcu_node:main',
            # synthetic MCU used by the offline socat loopback test
            'fake_mcu = mower_mcu_driver.fake_mcu:main',
        ],
    },
)
