"""scan ตอนนี้ตรงกับแมพไหม: ฉายจุด lidar ลงแมพด้วย TF ปัจจุบัน แล้วลองหมุนเพิ่มทีละ 15° ดูว่ามุมไหนตรงที่สุด"""
import time, math, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node('scan_match'); buf = Buffer(); TransformListener(buf, n); got = {}
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/map', lambda m: got.setdefault('map', m), q)
n.create_subscription(LaserScan, '/scan_fixed', lambda m: got.__setitem__('scan', m), qos_profile_sensor_data)
tf = None; e = time.time() + 15
while time.time() < e and (tf is None or 'map' not in got or 'scan' not in got):
    rclpy.spin_once(n, timeout_sec=0.2)
    try: tf = buf.lookup_transform('map', 'laser_frame', Time())
    except Exception: pass
m, s = got['map'], got['scan']; W, H, r = m.info.width, m.info.height, m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y
occ = set(i for i, v in enumerate(m.data) if v == 100)
def hit(x, y):
    cx, cy = int((x-ox)/r), int((y-oy)/r)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if 0 <= cx+dx < W and 0 <= cy+dy < H and ((cy+dy)*W + cx+dx) in occ: return True
    return False
t = tf.transform; q_ = t.rotation
yaw = math.atan2(2*(q_.w*q_.z+q_.x*q_.y), 1-2*(q_.y*q_.y+q_.z*q_.z)); lx, ly = t.translation.x, t.translation.y
pts = [(rng, s.angle_min + i*s.angle_increment) for i, rng in enumerate(s.ranges) if s.range_min < rng < min(s.range_max, 8.0)]
print(f'จุด lidar ที่ใช้ได้ {len(pts)}  TF ตอนนี้ yaw {math.degrees(yaw):.1f}°')
res = []
for off in range(-180, 180, 15):
    a0 = yaw + math.radians(off)
    h = sum(hit(lx + rng*math.cos(a0+a), ly + rng*math.sin(a0+a)) for rng, a in pts)
    res.append((h/len(pts), off))
for frac, off in sorted(res, reverse=True)[:4]:
    print(f'  ถ้าหมุนเพิ่ม {off:+4d}° : จุด lidar ตรงกับผนังในแมพ {100*frac:.0f}%')
print(f'  ที่มุมปัจจุบัน (+0°): {100*dict((o,f) for f,o in res)[0]:.0f}%')
n.destroy_node(); rclpy.shutdown()
