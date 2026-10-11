import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stepper_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaiphop',
    maintainer_email='chaiphop231102@gmail.com',
    description='ROS 2 bridge for the ESP32 + DM556 4-axis stepper controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stepper_bridge_node = stepper_bridge.stepper_bridge_node:main',
        ],
    },
)
