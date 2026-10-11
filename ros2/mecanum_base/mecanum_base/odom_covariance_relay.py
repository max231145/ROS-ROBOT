#!/usr/bin/env python3
"""
odom_covariance_relay -- ส่งต่อ nav_msgs/Odometry โดยเติมค่า covariance ให้

rf2o_laser_odometry ไม่ใส่ covariance เลย (เป็น 0 ทั้งหมด) ถ้าป้อนเข้า robot_localization ตรง ๆ
EKF จะถือว่าค่าจาก rf2o แม่นสมบูรณ์ แล้ว wheel_odom แทบไม่มีผล node นี้เติม covariance ตามที่ตั้งไว้

  subscribe  input_topic   nav_msgs/Odometry  (ค่าตั้งต้น /odom_rf2o)
  publish    output_topic  nav_msgs/Odometry  (ค่าตั้งต้น /odom_rf2o_cov)

หมายเหตุการตั้งค่า (robot_localization โหมด differential, ros_filter.cpp ขั้น 7e):
  covariance ของความเร็วที่ EKF ใช้ = (covariance ของ pose ครั้งก่อน + ครั้งนี้) * dt
  rf2o ออกตามความถี่ lidar ~5.8 Hz -> dt ~0.17 s  ดังนั้น pose variance c -> velocity variance ~0.34c
"""

import signal

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class OdomCovarianceRelay(Node):

    def __init__(self):
        super().__init__('odom_covariance_relay')
        in_topic = self.declare_parameter('input_topic', '/odom_rf2o').value
        out_topic = self.declare_parameter('output_topic', '/odom_rf2o_cov').value
        # ลำดับ: x y z roll pitch yaw
        self.pose_cov = [float(v) for v in self.declare_parameter(
            'pose_covariance_diagonal', [3.0e-3, 3.0e-3, 1.0e6, 1.0e6, 1.0e6, 1.0e-3]).value]
        self.twist_cov = [float(v) for v in self.declare_parameter(
            'twist_covariance_diagonal', [1.0e-3, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e-3]).value]
        if len(self.pose_cov) != 6 or len(self.twist_cov) != 6:
            raise ValueError('covariance_diagonal ต้องมี 6 ค่า (x y z roll pitch yaw)')

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self.on_odom, 10)
        self.get_logger().info(
            f'{in_topic} -> {out_topic}  pose cov {self.pose_cov}  twist cov {self.twist_cov}')

    def on_odom(self, msg: Odometry):
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        for i in range(6):
            pose_cov[i * 7] = self.pose_cov[i]
            twist_cov[i * 7] = self.twist_cov[i]
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C / timeout -s INT ส่ง SIGINT ให้ทั้ง process group แล้ว launch ส่งซ้ำอีกลูก
        # ลูกที่สองมาตอน Python กำลังปิด -> KeyboardInterrupt Traceback (เหมือนใน mecanum_base_node)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
