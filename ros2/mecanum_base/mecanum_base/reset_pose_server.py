"""reset_pose_server -- service ให้ปุ่ม RESET ใน Foxglove เรียก

    /reset_pose/reset   std_srvs/Trigger

รัน mecanum_base.reset_pose เป็น process แยก (มัน node ของตัวเองและเรียก service ของ
waypoint_manager / slam_toolbox / EKF) แล้วตอบผลกลับ -- ใช้เวลา ~6 วินาที
แยกเป็น node ของตัวเอง ไม่ใส่ใน waypoint_manager เพราะรีเซ็ตต้องเรียก /waypoint_manager/cancel
ถ้าอยู่ใน node เดียวกันที่ใช้ SingleThreadedExecutor จะรอตัวเองจนค้าง
"""
import signal
import subprocess
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


class ResetPoseServer(Node):
    def __init__(self):
        super().__init__('reset_pose')
        self.create_service(Trigger, '~/reset', self.srv_reset)
        self.get_logger().info('พร้อม: เรียก /reset_pose/reset เพื่อให้ตำแหน่งตอนนี้เป็นจุด 0')

    def srv_reset(self, req, res):
        self.get_logger().info('รีเซ็ตตำแหน่ง...')
        try:
            p = subprocess.run([sys.executable, '-m', 'mecanum_base.reset_pose'],
                               capture_output=True, text=True, timeout=45)
        except subprocess.TimeoutExpired:
            res.success, res.message = False, 'รีเซ็ตเกิน 45 วินาที -- ดู log ของ reset_pose'
            self.get_logger().error(res.message)
            return res
        lines = [ln for ln in p.stdout.splitlines() if ln.strip() and 'RTPS_TRANSPORT' not in ln]
        for ln in lines:
            self.get_logger().info(ln)
        res.success = p.returncode == 0
        if res.success:
            pos = next((ln.split('map -> base_link', 1)[1].strip() for ln in lines if 'map -> base_link' in ln), '')
            res.message = f'รีเซ็ตแล้ว ตำแหน่งตอนนี้คือ 0 ({pos}) -- ไดรเวอร์ปิดอยู่'
        else:
            failed = [ln.strip()[3:].strip() for ln in lines if ln.strip().startswith('!!')]
            res.message = 'ไม่ผ่าน: ' + ' / '.join(failed or lines[-2:] or [p.stderr.strip()[-200:]])
            self.get_logger().warn(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ResetPoseServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
