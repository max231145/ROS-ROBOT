"""ตั้งจุดวิ่งตามแมพเดิม (waypoint_manager) แล้วกด START   ใช้: python3 patrol_points.py x,y,yaw_deg ...
24 ก.ย. ผู้ใช้: เดินตามแมพที่เคยเดินให้ลื่น ไม่มุดใต้โต๊ะ -> ใช้จุดที่รอบก่อนไปถึงจริง
ต้องรัน route_test.py ไว้ก่อน (มันเปิดไดรเวอร์ตอนเห็น "เริ่มวิ่ง" และปิดตอนจบ)"""
import math
import sys
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
rclpy.init()
n = Node('patrol_points')
pub = n.create_publisher(PoseStamped, '/move_base_simple/goal', 10)


def call(name):
    c = n.create_client(Trigger, name)
    c.wait_for_service(timeout_sec=5)
    f = c.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    print(name, f.result().message if f.result() else 'no reply')


call('/waypoint_manager/clear')
time.sleep(1.0)
for a in sys.argv[1:]:
    x, y, yd = map(float, a.split(','))
    m = PoseStamped()
    m.header.frame_id = 'map'
    m.pose.position.x, m.pose.position.y = x, y
    m.pose.orientation.z, m.pose.orientation.w = math.sin(math.radians(yd) / 2), math.cos(math.radians(yd) / 2)
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.3)
    time.sleep(0.5)
call('/waypoint_manager/start')
