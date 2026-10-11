"""grid_stats -- สัดส่วน ว่าง/ห้ามเข้า/ไม่รู้ ของ /map และ /global_costmap/costmap + นับจุดเดี่ยวในแมพ"""
import sys, collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

TOPICS = ['/map', '/global_costmap/costmap']
got = {}

class N(Node):
    def __init__(self):
        super().__init__('grid_stats')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        for t in TOPICS:
            self.create_subscription(OccupancyGrid, t, lambda m, t=t: got.setdefault(t, m), qos)

rclpy.init(); n = N()
import time; end = time.time() + 20
while time.time() < end and len(got) < len(TOPICS):
    rclpy.spin_once(n, timeout_sec=0.5)

for t in TOPICS:
    m = got.get(t)
    if not m:
        print(f'{t}: ไม่ได้รับ'); continue
    d = m.data; tot = len(d)
    c = collections.Counter(d)
    unk = c.get(-1, 0); free = c.get(0, 0)
    occ = sum(v for k, v in c.items() if k >= 99 or k == 100)
    mid = tot - unk - free - occ
    known = tot - unk
    print(f'{t}: {m.info.width}x{m.info.height} res {m.info.resolution:.3f}  '
          f'unknown {100*unk/tot:.1f}%  free {100*free/tot:.1f}%  lethal/inscribed(>=99) {100*occ/tot:.1f}%  inflated(1-98) {100*mid/tot:.1f}%')
    if known:
        print(f'   ในส่วนที่รู้จัก: free {100*free/known:.1f}%  occupied {100*occ/known:.1f}%  inflated {100*mid/known:.1f}%')
    if t == '/map':
        # นับจุด occupied ที่อยู่โดดเดี่ยว (ไม่มีเพื่อนบ้าน occupied ใน 8 ทิศ)
        W, H = m.info.width, m.info.height
        iso = 0; occ_n = 0
        for y in range(1, H-1):
            row = y*W
            for x in range(1, W-1):
                if d[row+x] == 100:
                    occ_n += 1
                    nb = 0
                    for dy in (-1,0,1):
                        for dx in (-1,0,1):
                            if (dx or dy) and d[(y+dy)*W + x+dx] == 100: nb += 1
                    if nb == 0: iso += 1
        print(f'   จุด occupied ทั้งหมด {occ_n}  โดดเดี่ยว (ไม่มีเพื่อนบ้าน) {iso} = {100*iso/max(occ_n,1):.0f}%')
n.destroy_node(); rclpy.shutdown()
