"""nudge_test -- สั่งเดินหน้า 10 cm แล้วใช้ lidar ยืนยันว่าหุ่นวิ่งไปทาง +x ของ base_link จริง

    python3 nudge_test.py [ระยะ m=0.10] [ความเร็ว m/s=0.05]

วัดระยะจาก scan ตามทิศ +x / -x ของ base_link (ใช้ TF base_link->laser_frame แปลงมุม)
ก่อนและหลังขยับ: เดินหน้าจริง = ข้างหน้าใกล้ขึ้น ข้างหลังไกลขึ้น
เทียบกับ /wheel_odom, /odom_rf2o, /odometry/filtered ด้วย
เปิดไดรเวอร์เองหลังเช็คว่า /cmd_vel ไม่มีคำสั่งค้าง และปิดไดรเวอร์ทุกกรณีตอนจบ
"""
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

DIST = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
SPEED = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
WIN = math.radians(6)   # ครึ่งความกว้างของช่องที่ใช้วัดระยะ

rclpy.init()
n = Node('nudge_test')
buf = Buffer()
TransformListener(buf, n)
scans, cmds, odom = [], [], {}
n.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
n.create_subscription(Twist, '/cmd_vel', lambda m: cmds.append(m), 10)
for topic in ('/wheel_odom', '/odom_rf2o', '/odometry/filtered'):
    n.create_subscription(Odometry, topic, lambda m, t=topic: odom.__setitem__(t, m), 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10)


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


def laser_yaw():
    t = buf.lookup_transform('base_link', 'laser_frame', Time()).transform
    q = t.rotation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ranges_at(yaw_off, heading):
    """median ระยะในช่อง +-WIN รอบทิศ heading (เทียบ base_link) จาก scan ที่เก็บไว้"""
    vals = []
    for m in scans:
        for i, r in enumerate(m.ranges):
            if not (math.isfinite(r) and m.range_min < r <= m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment + yaw_off   # มุมใน base_link
            d = (a - heading + math.pi) % (2 * math.pi) - math.pi
            if abs(d) <= WIN:
                vals.append(r)
    return (statistics.median(vals), len(vals)) if vals else (float('nan'), 0)


def measure():
    scans.clear()
    spin(2.5)
    return {k: ranges_at(yo, h) for k, h in (('หน้า', 0.0), ('หลัง', math.pi))}


def odom_x():
    return {t: (m.pose.pose.position.x, m.pose.pose.position.y) for t, m in odom.items()}


def call_enable(on):
    c = n.create_client(SetBool, '/mecanum_base/enable')
    if not c.wait_for_service(timeout_sec=5):
        return None
    f = c.call_async(SetBool.Request(data=on))
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    return f.result()


spin(3.0)
yo = laser_yaw()
print(f'TF base_link->laser_frame yaw = {math.degrees(yo):+.1f} deg')
before = measure()
o0 = odom_x()
for k, (r, c) in before.items():
    print(f'ก่อน  {k}: {r:.3f} m ({c} จุด)')
if not before['หน้า'][1] or before['หน้า'][0] < DIST + 0.4:
    print('ข้างหน้าไม่โล่งพอ -- ไม่ขยับ')
    raise SystemExit(1)

cmds.clear()
spin(2.0)
moving = [m for m in cmds if abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3]
if moving:
    print(f'/cmd_vel มีคำสั่งค้าง {len(moving)} ข้อความ -- ไม่เปิดไดรเวอร์')
    raise SystemExit(1)

try:
    r = call_enable(True)
    print('เปิดไดรเวอร์:', r.success if r else 'ไม่มี service', r.message if r else '')
    if not r or not r.success:
        raise SystemExit(1)
    spin(1.5)
    tw = Twist()
    tw.linear.x = SPEED
    t_move = DIST / SPEED
    end = time.time() + t_move
    print(f'เดินหน้า {SPEED} m/s นาน {t_move:.1f} s')
    while time.time() < end:
        pub.publish(tw)
        spin(0.05)
    for _ in range(30):
        pub.publish(Twist())
        spin(0.05)
finally:
    r = call_enable(False)
    print('ปิดไดรเวอร์:', r.success if r else 'เรียก service ไม่ได้ -- ปิดเองด่วน!')

spin(1.0)
after = measure()
o1 = odom_x()
print()
for k in ('หน้า', 'หลัง'):
    (r0, _), (r1, c1) = before[k], after[k]
    print(f'หลัง  {k}: {r1:.3f} m ({c1} จุด)  เปลี่ยน {r1 - r0:+.3f} m')
for t in o1:
    if t in o0:
        dx, dy = o1[t][0] - o0[t][0], o1[t][1] - o0[t][1]
        print(f'{t:>20}: dx {dx:+.3f}  dy {dy:+.3f}')
df = after['หน้า'][0] - before['หน้า'][0]
if df < -DIST / 2:
    print(f'\nสรุป: หุ่นเข้าใกล้ของข้างหน้า {-df * 100:.1f} cm = สั่งเดินหน้าแล้ววิ่งไปทาง +x ถูกทาง')
elif df > DIST / 2:
    print(f'\nสรุป: หุ่นถอยห่างจากข้างหน้า {df * 100:.1f} cm = ล้อพาไปทาง -x กลับทาง!')
else:
    print(f'\nสรุป: ระยะข้างหน้าเปลี่ยนแค่ {df * 100:+.1f} cm -- ไม่ชัด (หุ่นไม่ขยับ หรือเป้าข้างหน้าไม่นิ่ง)')
n.destroy_node()
rclpy.shutdown()
