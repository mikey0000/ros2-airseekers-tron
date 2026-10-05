from setuptools import find_packages, setup
import os

package_name = 'det_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
        (os.path.join('share', package_name, 'config'),
         [os.path.join('config', 'det.yaml')]),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='ROS 2 port team',
    maintainer_email='dev@todo.todo',
    description='YOLOv8 detection on the RK3588S NPU.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'det_ros = det_ros.det_ros_node:main',
        ],
    },
)
