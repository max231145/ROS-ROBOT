"""enable_driver_checked -- cancel การวิ่ง เช็คว่า /cmd_vel ไม่มีคำสั่งค้าง 3 วิ แล้วค่อยเปิดไดรเวอร์ (ไม่ลบจุด)"""
import time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, SetBool
rclpy.init(); n = Node('pre_enable')
cmds = []
n.create_subscription(Twist, '/cmd_vel', lambda m: cmds.append((m.linear.x, m.angular.z)), 10)
def call(t, name, req=None):
    c = n.create_client(t, name)
    if not c.wait_for_service(timeout_sec=5): return None
    f = c.call_async(req or t.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=10); return f.result()
r = call(Trigger, '/waypoint_manager/cancel'); print('cancel:', r.message if r else 'ไม่มี service')
end = time.time() + 3
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
moving = [c for c in cmds if abs(c[0]) > 1e-3 or abs(c[1]) > 1e-3]
print(f'/cmd_vel 3 วิ: {len(cmds)} ข้อความ  ไม่เป็นศูนย์ {len(moving)}')
if moving:
    print('มีคำสั่งความเร็วค้าง -- ไม่เปิดไดรเวอร์'); raise SystemExit(1)
r = call(SetBool, '/mecanum_base/enable', SetBool.Request(data=True)); print('enable:', r.success if r else None, r.message if r else '')
n.destroy_node(); rclpy.shutdown()
