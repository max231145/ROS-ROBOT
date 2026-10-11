"""costmap_flicker -- ดู costmap แนวขวาของหุ่นทุก 2 วิ 12 ครั้ง แยกของเคลื่อนไหว (คนเดินผ่าน) กับสัญญาณรบกวน"""
import time, math, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node('flicker'); buf = Buffer(); TransformListener(buf, n); latest = {}
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: latest.__setitem__('c', m), q)
tf = None; e = time.time() + 10
while time.time() < e and (tf is None or 'c' not in latest):
    rclpy.spin_once(n, timeout_sec=0.2)
    try: tf = buf.lookup_transform('map', 'base_link', Time())
    except Exception: pass
t = tf.transform; q_ = t.rotation
yaw = math.atan2(2*(q_.w*q_.z+q_.x*q_.y), 1-2*(q_.y*q_.y+q_.z*q_.z)); x0, y0 = t.translation.x, t.translation.y
hd = yaw - math.pi/2
print(f'ตรวจแนวขวาของหุ่น 1.2 m ทุก 2 วิ 12 ครั้ง (หุ่น x={x0:.2f} y={y0:.2f})')
seen = None
for k in range(12):
    e = time.time() + 2
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.1)
    m = latest['c']
    if seen is not None and m.header.stamp == seen: pass
    seen = m.header.stamp
    W, H, r = m.info.width, m.info.height, m.info.resolution; ox, oy = m.info.origin.position.x, m.info.origin.position.y
    row = ''
    for i in range(1, 25):
        d = i * 0.05; x, y = x0 + d*math.cos(hd), y0 + d*math.sin(hd)
        v = m.data[int((y-oy)/r)*W + int((x-ox)/r)]
        row += '#' if v >= 99 else ('+' if v >= 50 else ('.' if v >= 0 else '?'))
    print(f'  {k*2:2d}s  |{row}|  (ช่องละ 5 cm, ซ้ายสุด=ติดตัวหุ่น ขวาสุด=1.2 m)')
n.destroy_node(); rclpy.shutdown()
