from setuptools import find_packages, setup

package_name = 'base_ble'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ROS 2 port team',
    maintainer_email='dev@todo.todo',
    description='Airseekers Tron BLE-to-ROS 2 bridge.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_ble_node = base_ble.base_ble_node:main',
        ],
    },
)
