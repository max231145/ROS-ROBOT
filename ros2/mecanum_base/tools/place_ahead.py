"""วางจุด 1 จุดห่างหน้าหุ่น DIST m หันทางเดิม ถ้าทางโล่ง  (python3 place_ahead.py [dist] [--place])"""
import sys, time, math, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
args = [a for a in sys.argv[1:] if not a.startswith('--')]
DIST = float(args[0]) if args else 1.0
REL = math.radians(float(args[1])) if len(args) > 1 else 0.0   # มุมเทียบหน้าหุ่น (ลบ = ขวา)
PLACE = '--place' in sys.argv
rclpy.init(); n = Node('place_ahead'); buf = Buffer(); TransformListener(buf, n)
got = {}
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: got.setdefault('c', m), q)
pub = n.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
tf = None; e = time.time() + 15
while time.time() < e and (tf is None or 'c' not in got):
    rclpy.spin_once(n, timeout_sec=0.2)
    try: tf = buf.lookup_transform('map', 'base_link', Time())
    except Exception: pass
t = tf.transform; q_ = t.rotation
yaw = math.atan2(2*(q_.w*q_.z+q_.x*q_.y), 1-2*(q_.y*q_.y+q_.z*q_.z))
x0, y0 = t.translation.x, t.translation.y
hd = yaw + REL
gx, gy = x0 + DIST*math.cos(hd), y0 + DIST*math.sin(hd)
m = got['c']; W, H, r = m.info.width, m.info.height, m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y
def cost(x, y):
    cx, cy = int((x-ox)/r), int((y-oy)/r)
    return m.data[cy*W+cx] if 0 <= cx < W and 0 <= cy < H else -2
path = [cost(x0+(gx-x0)*i/40, y0+(gy-y0)*i/40) for i in range(41)]
worst = max(path); blocked = sum(1 for v in path if v >= 99)
print(f'หุ่น x={x0:.2f} y={y0:.2f} หัน {math.degrees(yaw):.0f}°  จุดหมาย x={gx:.2f} y={gy:.2f} (ห่าง {DIST} m มุม {math.degrees(REL):+.0f}°) หันไป {math.degrees(hd):.0f}°')
print(f'costmap ตามแนวตรง: สูงสุด {worst}  ช่องห้ามเข้า {blocked}/41  unknown {path.count(-1)}  ที่จุดหมาย {path[-1]}')
ok = blocked == 0 and 0 <= path[-1] <= 50
print('ทางโล่ง' if ok else 'ทางไม่โล่ง -- ไม่วางจุด')
if ok and PLACE:
    c = n.create_client(Trigger, '/waypoint_manager/clear'); c.wait_for_service(timeout_sec=5)
    f = c.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    p = PoseStamped(); p.header.frame_id = 'map'; p.pose.position.x, p.pose.position.y = gx, gy
    p.pose.orientation.z, p.pose.orientation.w = math.sin(hd/2), math.cos(hd/2)
    e2 = time.time() + 3
    while pub.get_subscription_count() == 0 and time.time() < e2: rclpy.spin_once(n, timeout_sec=0.1)
    pub.publish(p); rclpy.spin_once(n, timeout_sec=0.3); time.sleep(0.3)
    print('วางจุดที่ 1 แล้ว (ลบจุดเก่าทิ้งก่อน)')
n.destroy_node(); rclpy.shutdown()
