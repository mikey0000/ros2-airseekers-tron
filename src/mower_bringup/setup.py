"""mower_bringup: installs the stack-level launch/ and config/ trees.

``launch`` and ``config`` in this package are symlinks to ``../../launch`` and
``../../config`` (the ros2_stack root), so the files keep living where the rest
of the stack (docs, scripts, compose bind mount) expects them. The globs below
walk through the symlinks and preserve the directory layout under
``share/mower_bringup/{launch,config}/...``.
"""
import os
from glob import glob

from setuptools import setup

package_name = 'mower_bringup'
here = os.path.dirname(os.path.abspath(__file__))


def tree(root):
    """data_files entries for every file under ``root``, keeping subdirs."""
    entries = {}
    for path in glob(os.path.join(root, '**', '*'), recursive=True):
        if os.path.isdir(path) or '__pycache__' in path:
            continue
        rel_dir = os.path.relpath(os.path.dirname(path), here)
        entries.setdefault(os.path.join('share', package_name, rel_dir), []).append(
            os.path.relpath(path, here))
    return sorted(entries.items())


os.chdir(here)

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + tree('launch') + tree('config'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael@example.com',
    description='Launch-only bringup for the Airseekers Tron ROS 2 stack.',
    license='Apache-2.0',
)
