"""clear_dirs -- วัดว่าหุ่นขยับได้กี่เมตรใน 8 ทิศก่อนชน costmap (ไม่ขยับหุ่น)  python3 clear_dirs.py"""
import time, math, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node('clear_dirs'); buf = Buffer(); TransformListener(buf, n); got = {}
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: got.setdefault('c', m), q)
tf = None; e = time.time() + 15
while time.time() < e and (tf is None or 'c' not in got):
    rclpy.spin_once(n, timeout_sec=0.2)
    try: tf = buf.lookup_transform('map', 'base_link', Time())
    except Exception: pass
t = tf.transform; q_ = t.rotation
yaw = math.atan2(2*(q_.w*q_.z+q_.x*q_.y), 1-2*(q_.y*q_.y+q_.z*q_.z))
x0, y0 = t.translation.x, t.translation.y
m = got['c']; W, H, r = m.info.width, m.info.height, m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y
def cost(x, y):
    cx, cy = int((x-ox)/r), int((y-oy)/r)
    return m.data[cy*W+cx] if 0 <= cx < W and 0 <= cy < H else -2
print(f'จุดที่หุ่นยืน cost {cost(x0,y0)}')
names = {0: 'หน้า', 45: 'หน้า-ซ้าย', 90: 'ซ้าย', 135: 'หลัง-ซ้าย', 180: 'หลัง', -135: 'หลัง-ขวา', -90: 'ขวา', -45: 'หน้า-ขวา'}
for rel, name in names.items():
    a = yaw + math.radians(rel); free = 0.0
    for i in range(1, 61):
        d = i * 0.05
        v = cost(x0 + d*math.cos(a), y0 + d*math.sin(a))
        if v >= 99 or v < 0: break
        free = d
    print(f'  {name:>9} ({rel:+4d}°): โล่ง {free:.2f} m')
n.destroy_node(); rclpy.shutdown()
