"""Nav2 สำหรับฐาน mecanum -- กดตำแหน่งบนแมพใน Foxglove แล้วหุ่นวิ่งไปเอง

    ros2 launch mecanum_base navigation.launch.py

ต้องมีระบบหลักรันอยู่ก่อน (ros2-slam.service): slam_toolbox (map, map->odom), EKF (odom->base_link),
/scan_fixed และ mecanum_base_node (รับ cmd_vel)

ไม่ใช้ nav2_bringup เพราะลาก Gazebo มาทั้งชุด -- เปิดแค่ node ที่ใช้จริง
ไม่มี map_server / AMCL เพราะ slam_toolbox ทำหน้าที่นั้นอยู่แล้ว

เป้าหมาย: Foxglove แผง 3D ปุ่ม "Publish 2D pose" ส่งเข้า /move_base_simple/goal
  -> waypoint_manager เก็บเป็นรายการจุด (ยังไม่วิ่ง)
  -> เรียก /waypoint_manager/start แล้ววิ่งไปทีละจุด ครบแล้วกลับจุดเริ่มต้น
  bt_navigator ไม่ได้ฟัง /move_base_simple/goal ตรง ๆ แล้ว (คลิกทีเดียวจะไม่วิ่งทันที)

!! ไดรเวอร์มอเตอร์ยังปิดอยู่จนกว่าจะเรียก /mecanum_base/enable true -- Nav2 ส่ง cmd_vel ได้แต่ล้อไม่หมุน !!
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mecanum_base')
    default_params = os.path.join(share, 'config', 'nav2_params.yaml')
    bt_xml = os.path.join(share, 'config', 'nav_to_pose_safe.xml')

    params = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    common = dict(output='screen', respawn=False,
                  arguments=['--ros-args', '--log-level', log_level])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='ไฟล์ yaml ของ Nav2'),
        DeclareLaunchArgument('log_level', default_value='info'),

        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params], remappings=[('cmd_vel', 'cmd_vel_nav')], **common),

        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params], **common),

        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[params], remappings=[('cmd_vel', 'cmd_vel_nav')], **common),

        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params, {'default_nav_to_pose_bt_xml': bt_xml}], **common),

        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             parameters=[params], remappings=[('cmd_vel', 'cmd_vel_nav')], **common),

        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             parameters=[params], **common),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', parameters=[params], **common),

        # ไม่ใช่ lifecycle node -- รอ action navigate_to_pose เองตอนกด start
        # explore_max_radius: 24 ก.ย. ผู้ใช้สั่งขยาย 1 -> 2 m (สายยาวพอ) -> 3 m (ผู้ใช้สั่ง ~16:30) -> 2.5 m (16:40 ปลั๊กหลุดที่ 3 m) ขยายต่อต้องถามผู้ใช้
        Node(package='mecanum_base', executable='waypoint_manager', name='waypoint_manager',
             # reverse_speed: 24 ก.ย. ผู้ใช้ขอเร็วขึ้น 0.08 -> 0.12 m/s (ความเร็วตอนถอยกลับด้วย action backup)
             parameters=[{'explore_max_radius': 2.5, 'reverse_speed': 0.12,
                         # 24 ก.ย. 1.2 -> 0.5: ถอยยาวเข้าใต้โต๊ะ (lidar ไม่เห็นคาน) -- ไกลกว่านี้หมุนแล้วเดินหน้า
                         'reverse_max_dist': 0.5}],
             output='screen', respawn=False),

        # ปุ่ม RESET ใน Foxglove -> /reset_pose/reset (ตำแหน่งตอนนี้เป็นจุด 0 หลังย้ายหุ่นด้วยมือ)
        Node(package='mecanum_base', executable='reset_pose_server', name='reset_pose',
             output='screen', respawn=False),

        # /map -> /map_walls กำแพงไล่สีตามความสูงให้ดูแมพใน Foxglove ง่ายขึ้น (ความสูงเป็นค่าตั้ง ไม่ใช่ของจริง)
        Node(package='mecanum_base', executable='map_walls', name='map_walls',
             output='screen', respawn=False),

        # YOLO (กล้อง) + /scan_nav -> หมุดของบนแมพ /object_markers
        Node(package='mecanum_base', executable='object_locator', name='object_locator',
             output='screen', respawn=True, respawn_delay=2.0),

        # เขตห้ามเข้าที่ผู้ใช้คลิก 4 มุมใน Foxglove (Publish point) -> /keepout_cloud -> keepout_layer
        # เก็บใน ~/maps/keepout_zones.json (24 ก.ย. โต๊ะเหล็กคานต่ำที่ lidar เห็นแค่ขา)
        Node(package='mecanum_base', executable='keepout_zones', name='keepout_zones',
             output='screen', respawn=False),

        # ToF (Waveshare TOF (D)) ทาง /dev/ttyAMA0 -> /tof_range -- ยังไม่ต่อสายก็รันได้ ลองเปิดพอร์ตใหม่ทุก 5 s
        Node(package='mecanum_base', executable='tof_node', name='tof_node',
             output='screen', respawn=True, respawn_delay=2.0),
    ])
