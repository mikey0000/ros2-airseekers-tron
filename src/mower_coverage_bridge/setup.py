from glob import glob

from setuptools import setup

package_name = 'mower_coverage_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael@example.com',
    description='Serves the MowgliNext /plan_coverage action on top of the '
                'mower_coverage /coverage/plan service.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_action_server = mower_coverage_bridge.coverage_server_node:main',
        ],
    },
)
