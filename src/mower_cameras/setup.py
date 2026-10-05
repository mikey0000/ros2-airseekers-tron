from setuptools import find_packages, setup

package_name = 'mower_cameras'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
        ('share/' + package_name + '/config', ['config/cameras.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael@example.com',
    description=('V4L2 camera drivers for the Airseekers Tron: OA rkisp cameras '
                 '(v4l2_cam, multi-planar), '
                 'rear UVC MJPEG (/rear_camera/image_raw, opt-in) and debug Metoak '
                 'stereo split (/vio/{left,right}/image_raw, opt-in).'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = mower_cameras.camera_node:main',
            'v4l2_cam = mower_cameras.v4l2_node:main',
        ],
    },
)
