from setuptools import find_packages, setup

package_name = 'mower_rknn'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='ROS 2 port team',
    maintainer_email='dev@todo.todo',
    description='Shared RK3588S NPU (rknn-toolkit-lite2) helpers.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
