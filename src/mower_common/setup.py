from setuptools import setup

package_name = 'mower_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael@example.com',
    description='Shared Python helpers for the Airseekers Tron nodes: the subscription pump, '
                'node clocks, angle helpers, mission motion phases and robot geometry constants.',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
)
