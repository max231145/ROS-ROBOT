#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0
#
# แก้ให้เข้ากับ ROS 2 Jazzy:
#   node_executable= -> executable=
#   node_name=       -> name=
#   node_namespace=  -> namespace=
#   static_transform_publisher: อาร์กิวเมนต์แบบเรียงตำแหน่ง -> แบบมีชื่อ (--x --y ...)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar.yaml'),
        description='Path to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
    )

    # ของเดิมส่ง quaternion 0 0 0 1 = ไม่หมุน จึงเท่ากับ roll/pitch/yaw = 0
    # ตำแหน่ง z=0.02 คือ lidar สูงจาก base_link 2 ซม.
    # 23 ก.ย.: หน้าหุ่น = ทิศที่กล้องหัน -- ทดสอบคนยืนหน้ากล้องแล้ว lidar เห็นที่ 180 องศา
    # และเทียบภาพกล้องกับ scan แล้วเป็นแบบหมุน 180 (ไม่ใช่สะท้อน) จึงตั้ง yaw = pi
    # x=0.17 y=+0.076 (23 ก.ย.): ผู้ใช้วัดห่างขอบหน้า 7 cm และห่างขอบ "ซ้าย" 30 cm แต่รูปจากด้านบนยืนยันว่า
    #   ขอบที่วัด 30 cm คือฝั่งแบตเตอรี่ = ขวาของหุ่น (หันหน้าไปทางกล้อง) lidar อยู่ฝั่งปลั๊กพ่วง = ซ้ายของกลาง
    #   (แผ่นกว้าง 44.8: 30 - 22.4 = 7.6 cm ไปทางซ้าย) · scan ด้านซ้ายโดนที่ชาร์จ/ปลั๊กพ่วงบังตรงกัน
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '--x', '0.17', '--y', '0.076', '--z', '0.02',
            '--roll', '0', '--pitch', '0', '--yaw', '3.14159265',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame',
        ],
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
    ])
