import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('stepper_hardware')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')

    # กาง xacro เป็น URDF พร้อมส่งค่าพอร์ตเข้าไปในแท็ก <ros2_control>
    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([pkg, 'urdf', 'stepper_arm.urdf.xacro']), ' ',
            'port:=', port, ' ',
            'baud:=', baud,
        ])
    }

    controllers = os.path.join(
        get_package_share_directory('stepper_hardware'), 'config', 'controllers.yaml')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers],
        output='screen',
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='พอร์ต serial ของ ESP32'),
        DeclareLaunchArgument('baud', default_value='115200',
                              description='ต้องตรงกับ SERIAL_BAUD ในไฟล์ .ino'),

        control_node,
        rsp_node,
        jsb_spawner,
        # รอ joint_state_broadcaster ขึ้นก่อนค่อยปล่อย arm_controller
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
        ),
    ])
