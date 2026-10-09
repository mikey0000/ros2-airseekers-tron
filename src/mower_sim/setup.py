from glob import glob

from setuptools import setup

package_name = 'mower_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael.arthur@gameglass.gg',
    description='Headless kinematic simulator replacing the Tron hardware drivers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = mower_sim.sim_node:main',
            'write_maps = mower_sim.maps:main',
            'scenario = mower_sim.scenario:main',
        ],
    },
)
