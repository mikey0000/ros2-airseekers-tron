from glob import glob

from setuptools import setup

package_name = 'mower_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael@example.com',
    description='Mission layer (MowgliNext-compatible behavior_tree_node) for the '
                'Airseekers Tron: start/mow/home, recording, manual mowing, resume, '
                'rain/battery/boundary/emergency guards and blade interlocks.',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = mower_mission.mission_node:main',
        ],
    },
)
