from setuptools import setup

package_name = 'mower_gui_bridge'

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
    description='Adapter presenting the Tron hardware stack under the MowgliNext GUI '
                'topic/service contract, plus a stub high-level state machine.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_bridge = mower_gui_bridge.gui_bridge_node:main',
        ],
    },
)
