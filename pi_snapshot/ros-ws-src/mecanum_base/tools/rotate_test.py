"""rotate_test -- หมุนอยู่กับที่ไป-กลับ แล้วดูว่า SLAM เห็นหุ่น "เลื่อน" ไหม เพื่อเช็คตำแหน่ง lidar ใน TF

    python3 rotate_test.py [องศา=90] [rad/s=0.3]

หลักการ: หุ่นหมุนรอบ base_link -- lidar ที่ติดเยื้องจะวิ่งเป็นวงกลม SLAM หาตำแหน่ง lidar ได้ถูก
แล้วลบ offset ใน TF ออกเพื่อได้ base_link ถ้า offset ใน TF ผิดไป dl ตำแหน่ง base_link ที่ SLAM รายงาน
จะเลื่อน (R(th) - I) dl ทั้งที่หุ่นหมุนอยู่กับที่ -- หมุน 90° ผิด 16 cm เห็นเลื่อน ~23 cm
จากค่าที่เลื่อนคำนวณย้อนได้ว่า lidar น่าจะอยู่ตรงไหน (สมมติว่าตัวหุ่นไม่ไถลตอนหมุน)
เปิดไดรเวอร์เองหลังเช็ค /cmd_vel ไม่มีคำสั่งค้าง และปิดทุกกรณีตอนจบ
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
WZ = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3

rclpy.init()
n = Node('rotate_test')
buf = Buffer()
TransformListener(buf, n)
odom, cmds = {}, []
for topic in ('/odometry/filtered', '/odom_rf2o'):
    n.create_subscription(Odometry, topic, lambda m, t=topic: odom.__setitem__(t, m), 10)
n.create_subscription(Twist, '/cmd_vel', cmds.append, 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10)


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tf2d(parent, child):
    t = buf.lookup_transform(parent, child, Time()).transform
    return t.translation.x, t.translation.y, yaw_of(t.rotation)


def poses():
    out = {'SLAM': tf2d('map', 'base_link')}
    for k, m in odom.items():
        p = m.pose.pose
        out[k] = (p.position.x, p.position.y, yaw_of(p.orientation))
    return out


def call_enable(on):
    c = n.create_client(SetBool, '/mecanum_base/enable')
    if not c.wait_for_service(timeout_sec=5):
        return None
    f = c.call_async(SetBool.Request(data=on))
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    return f.result()


def turn(wz, sec):
    tw = Twist()
    tw.angular.z = wz
    end = time.time() + sec
    while time.time() < end:
        pub.publish(tw)
        spin(0.05)
    for _ in range(20):
        pub.publish(Twist())
        spin(0.05)


def report(label, a, b, lx, ly):
    print(f'\n--- {label} ---')
    for k in b:
        if k not in a:
            continue
        dx, dy = b[k][0] - a[k][0], b[k][1] - a[k][1]
        dh = (math.degrees(b[k][2] - a[k][2]) + 180) % 360 - 180
        print(f'{k:>20}: หัน {dh:+6.1f}°  เลื่อน {math.hypot(dx, dy) * 100:5.1f} cm (dx {dx * 100:+.1f} dy {dy * 100:+.1f})')
    # คำนวณย้อน offset จริงของ lidar จาก SLAM: d = R0 (R(th) - I) dl  (สมมติตัวหุ่นไม่ไถล)
    x0, y0, h0 = a['SLAM']
    x1, y1, h1 = b['SLAM']
    th = h1 - h0
    dxm, dym = x1 - x0, y1 - y0
    # เอา d กลับเข้ากรอบหุ่นตอนเริ่ม
    dxb = math.cos(h0) * dxm + math.sin(h0) * dym
    dyb = -math.sin(h0) * dxm + math.cos(h0) * dym
    m00, m01, m10, m11 = math.cos(th) - 1, -math.sin(th), math.sin(th), math.cos(th) - 1
    det = m00 * m11 - m01 * m10
    if abs(det) > 0.2:
        ex = (m11 * dxb - m01 * dyb) / det
        ey = (-m10 * dxb + m00 * dyb) / det
        print(f'ถ้าตัวหุ่นไม่ไถล: TF ผิดไป dx {ex * 100:+.1f} dy {ey * 100:+.1f} cm'
              f' -> lidar น่าจะอยู่ x={lx + ex:+.3f} y={ly + ey:+.3f}')


# static TF บางทีมาช้า (รอบแรก 23 ก.ย. หา laser_frame ไม่เจอใน 3 วิ) -- รอสูงสุด 15 วิ
end = time.time() + 15.0
while True:
    spin(0.5)
    try:
        lx, ly, lyaw = tf2d('base_link', 'laser_frame')
        tf2d('map', 'base_link')
        break
    except Exception:
        if time.time() > end:
            print('ไม่มี TF base_link->laser_frame หรือ map->base_link -- ไม่หมุน')
            raise SystemExit(1)
print(f'TF ตอนนี้: lidar x={lx:+.3f} y={ly:+.3f} yaw={math.degrees(lyaw):+.1f}°')
cmds.clear()
spin(2.0)
if any(abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3 for m in cmds):
    print('/cmd_vel มีคำสั่งค้าง -- ไม่เปิดไดรเวอร์')
    raise SystemExit(1)

t_turn = math.radians(DEG) / WZ
p0 = poses()
try:
    r = call_enable(True)
    print('เปิดไดรเวอร์:', r.success if r else 'ไม่มี service')
    if not r or not r.success:
        raise SystemExit(1)
    spin(1.0)
    print(f'หมุนซ้าย {DEG:.0f}° ({WZ} rad/s {t_turn:.1f} s)', flush=True)
    turn(WZ, t_turn)
    spin(4.0)                   # รอ SLAM จับ scan หลังหยุด
    p1 = poses()
    print(f'หมุนขวากลับ {DEG:.0f}°', flush=True)
    turn(-WZ, t_turn)
    spin(4.0)
    p2 = poses()
finally:
    r = call_enable(False)
    print('ปิดไดรเวอร์:', r.success if r else 'เรียก service ไม่ได้ -- ปิดเองด่วน!')

report(f'หลังหมุนซ้าย {DEG:.0f}°', p0, p1, lx, ly)
report('หลังหมุนกลับ (เทียบตอนเริ่ม)', p0, p2, lx, ly)
n.destroy_node()
rclpy.shutdown()
