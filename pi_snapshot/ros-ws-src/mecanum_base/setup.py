import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mecanum_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaiphop',
    maintainer_email='chaiphop231102@gmail.com',
    description='Mecanum base driver for the ESP32 esp32_mecanum_base firmware (cmd_vel -> wheel_odom)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_base_node = mecanum_base.mecanum_base_node:main',
            'fake_board = mecanum_base.fake_board:main',
            'odom_covariance_relay = mecanum_base.odom_covariance_relay:main',
            'waypoint_manager = mecanum_base.waypoint_manager:main',
            'reset_pose = mecanum_base.reset_pose:main',
            'reset_pose_server = mecanum_base.reset_pose_server:main',
            'map_walls = mecanum_base.map_walls:main',
            'object_locator = mecanum_base.object_locator:main',
            'tof_node = mecanum_base.tof_node:main',
            'keepout_zones = mecanum_base.keepout_zones:main',
            'goto_point = mecanum_base.goto_point:main',
        ],
    },
)
