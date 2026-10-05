from glob import glob

from setuptools import setup

package_name = 'mower_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael.arthur@gameglass.gg',
    description='WebSocket teleop relay and twist_mux config for the mower.',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_ws_relay = mower_teleop.cmd_vel_ws_relay:main',
        ],
    },
)
