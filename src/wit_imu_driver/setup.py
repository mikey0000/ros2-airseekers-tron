from setuptools import setup

package_name='wit_imu_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/resource_type',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'std_msgs', 'tf2_ros', 'python3-serial'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'wit_node = wit_imu_driver.wit_node:main',
        ],
    },
)