"""เลือกจุด A B C ที่ขอบพื้นที่ที่หุ่นไปถึงได้ (global costmap) ในรัศมีจากตำแหน่งหุ่น -- ห่างกันมากสุด
    python3 pick_abc.py [รัศมี=2.3]   -> พิมพ์ "x,y,yaw" 3 จุด (yaw หันไปทางจุดถัดไป) ใช้กับ patrol_points.py
24 ก.ย. ผู้ใช้: วิ่ง A->B->C สุดขอบแมพแทนจุดเยอะ ๆ (หยุดทุกจุด = สะดุด ช้า)"""
import math, sys, time
from collections import deque
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
R = float(sys.argv[1]) if len(sys.argv) > 1 else 2.3
rclpy.init(); n = Node('pick_abc'); got = []
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', got.append, q)
buf = Buffer(); TransformListener(buf, n)
t = time.time()
while (not got or not buf.can_transform('map', 'base_link', Time())) and time.time() - t < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
g = got[-1]; i = g.info; res = i.resolution; W, H = i.width, i.height
tr = buf.lookup_transform('map', 'base_link', Time()).transform.translation
rx, ry = tr.x, tr.y
cell = lambda x, y: (int((x - i.origin.position.x) / res), int((y - i.origin.position.y) / res))
xy = lambda c, r: (i.origin.position.x + (c + .5) * res, i.origin.position.y + (r + .5) * res)
ok = lambda c, r: 0 <= c < W and 0 <= r < H and 0 <= g.data[r * W + c] < 30   # ห่างของพอ (ไม่ใช่ขอบ inflation)
c0, r0 = cell(rx, ry)
# หุ่นจอดชิดของ ช่องตัวเองอาจไม่ว่าง (inflation) -- เริ่มจากทุกช่องว่างในรัศมี 0.5 m รอบตัวแทน
seeds = [(c0 + dc, r0 + dr) for dc in range(-10, 11) for dr in range(-10, 11)
         if math.hypot(dc, dr) * res <= 0.5 and ok(c0 + dc, r0 + dr)] or [(c0, r0)]
dist = {s: 0 for s in seeds}; dq = deque(seeds)
while dq:                                            # BFS บนพื้นที่ว่าง = ไปถึงได้จริง
    c, r = dq.popleft()
    for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nc, nr = c + dc, r + dr
        if (nc, nr) in dist or not ok(nc, nr):
            continue
        x, y = xy(nc, nr)
        if math.hypot(x - rx, y - ry) > R:
            continue
        dist[(nc, nr)] = dist[(c, r)] + 1; dq.append((nc, nr))
pts = [xy(c, r) for (c, r) in dist]
if len(pts) < 10:
    raise SystemExit(f'ที่ว่างที่ไปถึงได้น้อยเกินไป ({len(pts)} ช่อง)')
A = max(pts, key=lambda p: math.hypot(p[0] - rx, p[1] - ry))
B = max(pts, key=lambda p: math.hypot(p[0] - A[0], p[1] - A[1]))
C = max(pts, key=lambda p: min(math.hypot(p[0] - A[0], p[1] - A[1]), math.hypot(p[0] - B[0], p[1] - B[1])))
seq = [A, B, C]
out = []
for k, p in enumerate(seq):
    nx = seq[k + 1] if k + 1 < len(seq) else (rx, ry)
    out.append(f'{p[0]:.2f},{p[1]:.2f},{math.degrees(math.atan2(nx[1] - p[1], nx[0] - p[0])):.0f}')
print(f'# หุ่นอยู่ ({rx:.2f},{ry:.2f}) ที่ว่างไปถึงได้ {len(pts)} ช่อง', file=sys.stderr)
print(' '.join(out))
