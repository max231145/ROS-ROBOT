"""goto_point -- กดจุดในแมพ Foxglove (Publish point -> /goto_point) แล้วหุ่นเดินไปจุดนั้นเลย ไม่ต้องลากลูกศร

6 ต.ค. ผู้ใช้ขอ: "ทำเป็นจุดให้กดในแมพ Foxglove แล้วให้หุ่นเดินมายังจุดที่กด ไม่ต้องตั้งเป็นลูกศร"
- ใช้ topic แยก /goto_point เพราะ /clicked_point เป็นของ keepout_zones (4 คลิก = 1 เขตห้ามเข้า)
- ทิศตอนถึงจุด = ทิศที่เดินมา (จากหุ่นไปจุด) หุ่นจะไม่หมุนเพิ่มตอนจบ
- กดจุดใหม่ระหว่างวิ่ง = ยกเลิกเป้าเก่าแล้วไปจุดใหม่
- เปิดไดรเวอร์ให้เองตอนเริ่ม ปิดเองเมื่อถึง / ไปไม่ถึง / เกินเวลา / ~/cancel
- เขตห้ามเข้ากับสิ่งกีดขวาง Nav2 จัดการเอง (จุดที่อยู่ในเขตห้ามเข้าจะไปไม่ถึงแล้วหยุด)
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class GotoPoint(Node):
    def __init__(self):
        super().__init__('goto_point')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.timeout_s = float(self.declare_parameter('timeout_s', 120.0).value)
        self.max_dist = float(self.declare_parameter('max_dist', 8.0).value)   # จุดไกลกว่านี้ไม่ไป (กดพลาด)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.enable_cli = self.create_client(SetBool, '/mecanum_base/enable')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '~/marker', 1)
        self.create_subscription(PointStamped, '/goto_point', self.on_point, 10)
        self.create_service(Trigger, '~/cancel', self.srv_cancel)
        self.goal_handle = None
        self.run_id = 0
        self.deadline = None
        self.create_timer(1.0, self.watchdog)
        self.get_logger().info('พร้อม: กดจุดใน Foxglove (Publish point -> /goto_point) แล้วหุ่นจะเดินไป')

    # ---------------- helpers ----------------
    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time()).transform.translation
            return t.x, t.y
        except TransformException:
            return None

    def to_map(self, msg):
        if msg.header.frame_id in ('', self.map_frame):
            return msg.point.x, msg.point.y
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, Time()).transform
        except TransformException as e:
            self.get_logger().warn(f'แปลงจุดจาก {msg.header.frame_id} ไม่ได้: {e}')
            return None
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = msg.point.x, msg.point.y
        return (t.translation.x + x * math.cos(yaw) - y * math.sin(yaw),
                t.translation.y + x * math.sin(yaw) + y * math.cos(yaw))

    def set_driver(self, on):
        if not self.enable_cli.service_is_ready():
            self.get_logger().warn('ไม่เจอ /mecanum_base/enable')
            return
        self.enable_cli.call_async(SetBool.Request(data=on))

    def stop_robot(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())

    def show_marker(self, x, y, color):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'goto_point', 0, Marker.CYLINDER, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.25
        m.scale.z = 0.1
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 0.8
        self.marker_pub.publish(m)

    # ---------------- main flow ----------------
    def on_point(self, msg):
        p = self.to_map(msg)
        robot = self.robot_xy()
        if p is None or robot is None:
            self.get_logger().warn('ยังไม่รู้ตำแหน่งหุ่นหรือจุดที่กด -- ไม่ไป')
            return
        x, y = p
        d = math.hypot(x - robot[0], y - robot[1])
        if d > self.max_dist:
            self.get_logger().warn(f'จุดที่กดห่าง {d:.1f} m เกิน {self.max_dist:.1f} m -- ไม่ไป (กดพลาด?)')
            return
        if not self.nav.server_is_ready():
            self.get_logger().warn('Nav2 ยังไม่พร้อม (เปิด restart_nav2.sh ก่อน) -- ไม่ไป')
            return
        if self.goal_handle is not None:
            self.get_logger().info('มีจุดใหม่ -- ยกเลิกเป้าเก่า')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.run_id += 1
        run = self.run_id
        yaw = math.atan2(y - robot[1], x - robot[0])       # หันตามทิศที่เดินมา ไม่ต้องลากลูกศร
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.set_driver(True)
        self.show_marker(x, y, (0.1, 0.8, 0.1))
        self.deadline = self.get_clock().now().nanoseconds / 1e9 + self.timeout_s
        self.get_logger().info(f'ไปจุด x={x:.2f} y={y:.2f} (ห่าง {d:.1f} m) ไม่เกิน {self.timeout_s:.0f} s')
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.on_accepted(f, run, x, y))

    def on_accepted(self, future, run, x, y):
        h = future.result()
        if run != self.run_id:
            if h is not None and h.accepted:
                h.cancel_goal_async()
            return
        if h is None or not h.accepted:
            self.get_logger().warn('Nav2 ไม่รับเป้านี้')
            self.finish(run, x, y, ok=False)
            return
        self.goal_handle = h
        h.get_result_async().add_done_callback(lambda f: self.on_result(f, run, x, y))

    def on_result(self, future, run, x, y):
        if run != self.run_id:
            return
        status = future.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info('ถึงจุดที่กดแล้ว' if ok else f'ไปไม่ถึงจุดที่กด (status {status})')
        self.finish(run, x, y, ok)

    def finish(self, run, x, y, ok):
        if run != self.run_id:
            return
        self.goal_handle = None
        self.deadline = None
        self.stop_robot()
        self.set_driver(False)
        self.show_marker(x, y, (0.1, 0.8, 0.1) if ok else (0.9, 0.2, 0.1))

    def watchdog(self):
        if self.deadline is not None and self.get_clock().now().nanoseconds / 1e9 > self.deadline:
            self.get_logger().warn(f'เกิน {self.timeout_s:.0f} s -- ยกเลิกแล้วหยุด')
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.run_id += 1
            self.goal_handle = None
            self.deadline = None
            self.stop_robot()
            self.set_driver(False)

    def srv_cancel(self, _req, res):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.run_id += 1
        self.goal_handle = None
        self.deadline = None
        self.stop_robot()
        self.set_driver(False)
        res.success, res.message = True, 'ยกเลิกแล้ว หุ่นหยุด ไดรเวอร์ปิด'
        return res


def main():
    rclpy.init()
    node = GotoPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
