"""rf2o (ไม่ส่ง TF) + odom_covariance_relay + robot_localization EKF

แทน `ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py` ใน start-slam.sh
ค่า rf2o เหมือน launch เดิมของแพ็กเกจทุกตัว ยกเว้น publish_tf: False (EKF เป็นคนส่ง odom -> base_link)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_ekf = os.path.join(
        get_package_share_directory('mecanum_base'), 'config', 'ekf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('ekf_params_file', default_value=default_ekf,
                              description='ไฟล์ yaml ของ robot_localization'),

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                # /scan_rf2o = /scan ที่ตัดใกล้กว่า 0.35 m (scan_resampler) -- driver ส่งถึง 0.12 m ตั้งแต่ 2026-09-23
                # ถ้าใช้ /scan ตรง ๆ rf2o จะเห็นแบต/Pi/ปลั๊กพ่วงบนหุ่นเป็นสิ่งแวดล้อมนิ่ง แล้วนับระยะสั้น
                'laser_scan_topic': '/scan_rf2o',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0}],
            # rf2o วนที่ freq 20 Hz แต่ lidar ออก ~5.8 Hz รอบที่ยังไม่มี scan ใหม่จะ WARN "Waiting for laser_scans...."
            # ~31 บรรทัด/วินาที ลง /tmp/odometry.log (~13 MB/ชม.) เก็บไว้แค่ ERROR ขึ้นไป
            ros_arguments=['--log-level', 'error'],
        ),

        Node(
            package='mecanum_base',
            executable='odom_covariance_relay',
            name='odom_covariance_relay',
            output='screen',
            parameters=[{
                'input_topic': '/odom_rf2o',
                'output_topic': '/odom_rf2o_cov',
                # x y z roll pitch yaw -- ดูที่มาของตัวเลขใน odom_covariance_relay.py
                # x y 3e-2 -> ความเร็ว ~1e-2 หลวมกว่า vx ของ wheel_odom 10 เท่า และล้อส่งถี่กว่า 5 เท่า
                #   = แกนเดินหน้า/เฉข้างเชื่อล้อ ~98% ใช้ rf2o หลัก ๆ แค่ yaw (ล้อยังบอก yaw ไม่ได้)
                # เหตุผล: เทสวิ่งบนพื้น 1 m (2026-09-15) จริง 1.030 m ซ้าย 20 mm
                #   ล้อ 1.002 m / rf2o 0.577 m ขวา 19 mm / EKF เดิม (x y 3e-3) 0.947 m ขวา 30 mm
                #   rf2o นับเดินหน้าได้ครึ่งเดียว น่าจะเพราะวิ่งขนานผนังยาว (ยังไม่ได้พิสูจน์)
                # yaw 1e-2 -> ความเร็ว ~3.4e-3 ล้อ (vyaw 1e-3 x 30 Hz) มีน้ำหนัก ~95%
                #   เทสรอบ 2: rf2o yaw -5.1° แต่มุมหน้า/หลังของฐานเทียบเส้นข้างเท่ากัน = หุ่นไม่ได้หมุน
                'pose_covariance_diagonal': [3.0e-2, 3.0e-2, 1.0e6, 1.0e6, 1.0e6, 1.0e-2],
            }],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('ekf_params_file')],
        ),
    ])
