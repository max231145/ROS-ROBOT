"""circle_test -- วิ่งเป็นวงกลมต่อเนื่อง 1 รอบผ่าน velocity_smoother + collision_monitor (หยุดเองถ้ามีของ)
    python3 circle_test.py [รัศมี=0.7] [ความเร็ว=0.12] [ทิศ=+1 ซ้าย / -1 ขวา]
24 ก.ย. ผู้ใช้: ทดสอบวิ่งเป็นวงกลม (ดูความนุ่มตอนเลี้ยวโค้งต่อเนื่อง ไม่ใช่หยุดทีละจุด)"""
import math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import SetBool
R = float(sys.argv[1]) if len(sys.argv) > 1 else 0.7
V = float(sys.argv[2]) if len(sys.argv) > 2 else 0.12
D = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
rclpy.init(); n = Node('circle_test')
pub = n.create_publisher(Twist, '/cmd_vel_nav', 10)
odo = []
n.create_subscription(Odometry, '/odom_rf2o', lambda m: odo.append((m.pose.pose.position.x, m.pose.pose.position.y)), 10)
en = n.create_client(SetBool, '/mecanum_base/enable')
def enable(on):
    en.wait_for_service(timeout_sec=5); f = en.call_async(SetBool.Request(data=on))
    rclpy.spin_until_future_complete(n, f, timeout_sec=5); print('ไดรเวอร์', 'เปิด' if on else 'ปิด', flush=True)
dur = 2 * math.pi * R / V + 2.0          # +2 s ช่วงเร่ง
try:
    enable(True); time.sleep(0.5)
    print(f'วงกลม รัศมี {R} m ความเร็ว {V} m/s ใช้เวลา {dur:.0f} s', flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        m = Twist(); m.linear.x = V; m.angular.z = D * V / R
        pub.publish(m); rclpy.spin_once(n, timeout_sec=0.05)
finally:
    for _ in range(20):
        pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
    enable(False)
if len(odo) > 10:
    xs = [p[0] for p in odo]; ys = [p[1] for p in odo]
    print(f'ขนาดวงจาก lidar: {max(xs)-min(xs):.2f} x {max(ys)-min(ys):.2f} m (ควรได้ ~{2*R:.2f})  '
          f'จบห่างจุดเริ่ม {math.hypot(odo[-1][0]-odo[0][0], odo[-1][1]-odo[0][1])*100:.0f} cm')
