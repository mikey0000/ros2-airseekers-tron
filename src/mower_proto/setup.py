from setuptools import find_packages, setup

package_name = 'mower_proto'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
    ],
    install_requires=['setuptools', 'protobuf'],
    zip_safe=True,
    maintainer='ROS 2 port team',
    maintainer_email='dev@todo.todo',
    description='Generated Python protobuf bindings for the Airseekers Tron mower protocol.',
    license='Apache-2.0',
    entry_points={},
)
