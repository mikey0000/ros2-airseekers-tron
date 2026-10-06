from glob import glob

from setuptools import setup

package_name = 'mower_docking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael@example.com',
    description='Docking/undocking action server (rear-camera ArUco, reverse onto the dock).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'docking_server = mower_docking.docking_node:main',
            'print_marker = mower_docking.print_marker:main',
            'marker_check = mower_docking.marker_check:main',
        ],
    },
)
