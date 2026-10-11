import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_node(context):
    params = [LaunchConfiguration('params_file').perform(context)]
    # ทับค่าใน yaml เฉพาะตอนระบุ port:= เอง ถ้าใส่ {'port': ...} ตลอด ค่าใน yaml จะไม่มีผลเลย
    port = LaunchConfiguration('port').perform(context)
    if port:
        params.append({'port': port})

    return [Node(
        package='mecanum_base',
        executable='mecanum_base_node',
        name='mecanum_base',
        output='screen',
        emulate_tty=True,
        parameters=params,
    )]


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('mecanum_base'), 'config', 'mecanum_base.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_cfg,
                              description='ไฟล์ yaml ของขนาดหุ่นและเพดานความเร็ว'),
        DeclareLaunchArgument('port', default_value='',
                              description='พอร์ต serial ของ ESP32 (เว้นว่าง = ใช้ค่าใน yaml)'),
        OpaqueFunction(function=make_node),
    ])
