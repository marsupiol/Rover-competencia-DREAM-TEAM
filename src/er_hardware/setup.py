import os
from glob import glob
from setuptools import setup

package_name = 'er_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'requests', 'websocket-client'],
    zip_safe=True,
    maintainer='Earth Rover Team',
    maintainer_email='rover@frodobots.com',
    description='ROS 2 Bridge node for Earth Rovers SDK',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sdk_bridge_node = er_hardware.sdk_bridge_node:main',
        ],
    },
)
