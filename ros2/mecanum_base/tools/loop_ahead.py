"""loop_ahead -- วางวงวิ่งไป-กลับตามทิศที่หุ่นหันอยู่ ใช้กับ patrol_points.py
    python3 loop_ahead.py [ระยะสูงสุด=2.3] [จำนวนรอบ=3]  -> พิมพ์ "x,y,yaw ..." (ปลายทาง, จุดเริ่ม) x รอบ
ไล่ตามแนวหน้าหุ่นบน global costmap หยุดก่อนเจอของ/ที่ไม่รู้ 0.35 m -- ปลายวงเยื้องข้าง 0.4 m ให้เลี้ยวกลับเป็นโค้ง
24 ก.ย. ผู้ใช้: วางหุ่นหันไปทางตู้สีขาวแล้วให้วิ่งตรงไปแล้ววนกลับ (ตัวเลือกจุดสุ่มพาไปข้างหลัง/ใต้โต๊ะ)
ระยะสูงสุดจำกัดด้วยสายปลั๊ก (หลุดที่ ~2.5 m) -- ขยายต้องถามผู้ใช้"""
import math, sys, time
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
MAXD = min(float(sys.argv[1]) if len(sys.argv) > 1 else 2.3, 2.5)
LAPS = int(sys.argv[2]) if len(sys.argv) > 2 else 3
rclpy.init(); n = Node('loop_ahead'); got = []
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', got.append, q)
buf = Buffer(); TransformListener(buf, n)
t = time.time()
while (not got or not buf.can_transform('map', 'base_link', Time())) and time.time() - t < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
g = got[-1]; i = g.info
tf = buf.lookup_transform('map', 'base_link', Time()).transform
rx, ry = tf.translation.x, tf.translation.y
qq = tf.rotation; yaw = math.atan2(2 * (qq.w * qq.z + qq.x * qq.y), 1 - 2 * (qq.y * qq.y + qq.z * qq.z))
def cost(x, y):
    c = int((x - i.origin.position.x) / i.resolution); r = int((y - i.origin.position.y) / i.resolution)
    return g.data[r * i.width + c] if 0 <= c < i.width and 0 <= r < i.height else -1
def free_along(a):
    d = 0.3; f = 0.0
    while d <= MAXD + 0.35:                     # เดินตามแนว หาของแรก
        v = cost(rx + d * math.cos(a), ry + d * math.sin(a))
        if v < 0 or v >= 50:
            break
        f = d; d += 0.05
    return f
free = free_along(yaw)
if free < 1.0:                                  # ข้างหน้าไม่โล่ง: เลือกทิศที่โล่งไกลสุดแทน (หุ่นหมุนไปเองตอนออกตัว)
    best = max((math.radians(k) for k in range(0, 360, 10)), key=free_along)
    print(f'# ข้างหน้าว่างแค่ {free:.2f} m -- เปลี่ยนไปทิศ {math.degrees(best):.0f}° ที่โล่ง {free_along(best):.2f} m', file=sys.stderr)
    yaw, free = best, free_along(best)
far = min(free - 0.35, MAXD)
if far < 0.6:
    raise SystemExit(f'ข้างหน้าว่างแค่ {free:.2f} m -- วางหุ่นให้หันไปทางที่โล่งกว่านี้')
side = 0.4                                     # ปลายวงเยื้องซ้าย ขากลับเป็นโค้ง ไม่ใช่กลับหลังหันอยู่กับที่
fx = rx + far * math.cos(yaw) - side * math.sin(yaw); fy = ry + far * math.sin(yaw) + side * math.cos(yaw)
back = math.degrees(yaw + math.pi); fwd = math.degrees(yaw)
pts = [f'{fx:.2f},{fy:.2f},{back:.0f}', f'{rx:.2f},{ry:.2f},{fwd:.0f}'] * LAPS
print(f'# หุ่นอยู่ ({rx:.2f},{ry:.2f}) หัน {fwd:.0f}° ข้างหน้าว่าง {free:.2f} m วิ่งไป {far:.2f} m x {LAPS} รอบ', file=sys.stderr)
print(' '.join(pts[:-1]))                      # จุดสุดท้าย = จุดเริ่ม waypoint_manager กลับเองอยู่แล้ว
