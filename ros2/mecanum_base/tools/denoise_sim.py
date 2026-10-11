"""denoise_sim -- จำลองว่าถ้า denoise กรองกลุ่มเล็กกว่า N จุด พื้นที่ห้ามเข้าจะเหลือเท่าไหร่ (ใช้เลือก minimal_group_size)"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

got = {}
class N(Node):
    def __init__(self):
        super().__init__('denoise_sim')
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', lambda m: got.setdefault('m', m), q)
rclpy.init(); n = N(); end = time.time() + 20
while time.time() < end and 'm' not in got: rclpy.spin_once(n, timeout_sec=0.5)
m = got['m']; W, H, res = m.info.width, m.info.height, m.info.resolution
d = list(m.data)
occ = {(i % W, i // W) for i, v in enumerate(d) if v == 100}
known = sum(1 for v in d if v != -1)

def groups(cells):
    seen, out = set(), []
    for c in cells:
        if c in seen: continue
        stack, g = [c], []
        seen.add(c)
        while stack:
            x, y = stack.pop(); g.append((x, y))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (x+dx, y+dy)
                    if nb in cells and nb not in seen:
                        seen.add(nb); stack.append(nb)
        out.append(g)
    return out

G = groups(occ)
sizes = sorted(len(g) for g in G)
print(f'กลุ่มจุด occupied: {len(G)} กลุ่ม  ขนาด 1:{sizes.count(1)} 2:{sizes.count(2)} 3-4:{sum(1 for s in sizes if 3<=s<=4)} 5-9:{sum(1 for s in sizes if 5<=s<=9)} >=10:{sum(1 for s in sizes if s>=10)}  ใหญ่สุด {sizes[-1]}')

def blocked_pct(cells, radius):
    r = int(math.ceil(radius / res))
    disk = [(dx, dy) for dx in range(-r, r+1) for dy in range(-r, r+1) if (dx*dx+dy*dy)*res*res <= radius*radius]
    b = set()
    for x, y in cells:
        for dx, dy in disk:
            xx, yy = x+dx, y+dy
            if 0 <= xx < W and 0 <= yy < H and d[yy*W+xx] != -1:
                b.add((xx, yy))
    return 100 * len(b) / known

print(f'{"กรองกลุ่มเล็กกว่า":<18}{"จุดเหลือ":>9}{"ห้ามเข้า r=0.274":>18}{"ห้ามเข้า r=0.254":>18}')
for mg in (1, 2, 3, 5, 8):
    keep = {c for g in G if len(g) >= mg for c in g}
    print(f'{("ไม่กรอง" if mg==1 else str(mg)+" จุด"):<18}{len(keep):>9}{blocked_pct(keep,0.274):>17.1f}%{blocked_pct(keep,0.254):>17.1f}%')
n.destroy_node(); rclpy.shutdown()
