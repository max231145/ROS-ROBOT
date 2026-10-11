"""cost_at -- ค่า costmap ที่จุดเริ่ม/จุดหมาย และตามแนวตรงระหว่างสองจุด (แก้พิกัดในไฟล์ก่อนใช้)"""
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
got = {}
rclpy.init(); n = Node('cost_at')
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
for t in ('/global_costmap/costmap', '/map'):
    n.create_subscription(OccupancyGrid, t, lambda m, t=t: got.setdefault(t, m), q)
end = time.time() + 15
while time.time() < end and len(got) < 2: rclpy.spin_once(n, timeout_sec=0.3)
pts = {'จุดเริ่ม (1.97,-0.16)': (1.97, -0.16), 'จุดที่ 1 (0.05,0.04)': (0.05, 0.04), 'หุ่นตอนหยุด (2.25,0.06)': (2.25, 0.06)}
for t, m in got.items():
    W, H, r = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    print(f'== {t}')
    for name, (x, y) in pts.items():
        cx, cy = int((x - ox) / r), int((y - oy) / r)
        if not (0 <= cx < W and 0 <= cy < H):
            print(f'  {name}: นอกแมพ'); continue
        v = m.data[cy * W + cx]
        # ค่าสูงสุดในรัศมี 0.25 m รอบจุด
        rr = int(0.25 / r); mx = -2; unk = 0
        for dy in range(-rr, rr + 1):
            for dx in range(-rr, rr + 1):
                xx, yy = cx + dx, cy + dy
                if 0 <= xx < W and 0 <= yy < H:
                    vv = m.data[yy * W + xx]; mx = max(mx, vv); unk += (vv == -1)
        print(f'  {name}: ค่าที่จุด {v}  สูงสุดในรัศมี 25 cm {mx}  unknown {unk} ช่อง')
    # เส้นตรงจากจุดเริ่มไปจุดที่ 1: นับช่องที่ >= 99
    if t == '/global_costmap/costmap':
        (x0, y0), (x1, y1) = pts['จุดเริ่ม (1.97,-0.16)'], pts['จุดที่ 1 (0.05,0.04)']
        steps = 80; blocked = []
        for i in range(steps + 1):
            x = x0 + (x1 - x0) * i / steps; y = y0 + (y1 - y0) * i / steps
            cx, cy = int((x - ox) / r), int((y - oy) / r); v = m.data[cy * W + cx]
            if v >= 99: blocked.append(round(x, 2))
        print(f'  เส้นตรงเริ่ม->จุดที่ 1: ช่องห้ามเข้า {len(blocked)}/{steps+1}  ที่ x = {blocked[:12]}')
n.destroy_node(); rclpy.shutdown()
