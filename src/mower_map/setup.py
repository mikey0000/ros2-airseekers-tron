from glob import glob

from setuptools import setup

package_name = 'mower_map'

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
    maintainer_email='michael.arthur@gameglass.gg',
    description='Zone/map server (MowgliNext map_server_node port) and vendor map importer.',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_server_node = mower_map.map_server_node:main',
            'import_vendor_geojson = mower_map.import_vendor_geojson:main',
        ],
    },
)
