"""waypoint_manager_test -- เทส preview / วิ่งตามจุด (จุดที่ตำแหน่งหุ่น = สำเร็จทันที) / explore แล้ว cancel  ต้องปิดไดรเวอร์ก่อน"""
import time, rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, SetBool
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node('wm_test'); buf = Buffer(); TransformListener(buf, n)
pub = n.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
def call(t, name, req=None):
    c = n.create_client(t, name)
    if not c.wait_for_service(timeout_sec=5): return f'{name}: ไม่มี service'
    f = c.call_async(req or t.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=15)
    r = f.result(); return f'{name.split("/")[-1]}: {r.success} "{r.message}"'
def wait(s):
    e = time.time() + s
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.1)

print(call(SetBool, '/mecanum_base/enable', SetBool.Request(data=False)))
tf = None; e = time.time() + 10
while tf is None and time.time() < e:
    rclpy.spin_once(n, timeout_sec=0.2)
    try: tf = buf.lookup_transform('map', 'base_link', Time())
    except Exception: pass
t = tf.transform
print('--- 1) preview')
print(call(Trigger, '/waypoint_manager/explore_preview'))
print('--- 2) route regression')
print(call(Trigger, '/waypoint_manager/clear'))
for dx in (0.03, -0.03):
    m = PoseStamped(); m.header.frame_id = 'map'
    m.pose.position.x = t.translation.x + dx; m.pose.position.y = t.translation.y; m.pose.orientation = t.rotation
    pub.publish(m); wait(0.5)
print(call(Trigger, '/waypoint_manager/start'))
wait(8)
print('--- 3) explore แล้ว cancel')
print(call(Trigger, '/waypoint_manager/clear'))
print(call(Trigger, '/waypoint_manager/explore'))
wait(6)
print(call(Trigger, '/waypoint_manager/cancel'))
wait(1)
n.destroy_node(); rclpy.shutdown()
