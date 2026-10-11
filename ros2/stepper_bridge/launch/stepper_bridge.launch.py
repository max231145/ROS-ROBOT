import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('stepper_bridge'), 'config', 'steppers.yaml')

    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_cfg,
                              description='ไฟล์ yaml ของพารามิเตอร์แกน'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='พอร์ต serial ของ ESP32 (ทับค่าใน yaml)'),

        Node(
            package='stepper_bridge',
            executable='stepper_bridge_node',
            name='stepper_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'port': port}],
        ),
    ])
