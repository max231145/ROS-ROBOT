"""
frontier -- หาขอบแมพ (frontier) สำหรับสำรวจอัตโนมัติ  ไม่พึ่ง ROS เทสได้ทุกเครื่อง

frontier = ช่องว่างที่รู้แล้ว (free) ที่ติดกับช่องที่ยังไม่รู้ (unknown) = ขอบของพื้นที่ที่ยังไม่ได้สำรวจ
ส่งหุ่นไปยืนใกล้ขอบ lidar จะเห็นพื้นที่ใหม่ แมพโตขึ้น วนจนไม่เหลือขอบ

ค่าในแมพ (nav_msgs/OccupancyGrid): -1 = ไม่รู้  0 = ว่าง  100 = มีของ
ค่าใน costmap ที่ Nav2 ส่งออกมา: -1 = ไม่รู้  0 = ว่าง  1-98 = ใกล้ของ  99 = ชิดตัวหุ่น  100 = ชน
"""

from collections import deque
import math

import numpy as np

UNKNOWN = -1


class Grid:
    """ห่อ OccupancyGrid ให้แปลง พิกัดโลก <-> ช่อง ได้ง่าย"""

    def __init__(self, data, width, height, resolution, origin_x, origin_y):
        self.a = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.w, self.h, self.res = width, height, resolution
        self.ox, self.oy = origin_x, origin_y

    @classmethod
    def from_msg(cls, msg):
        i = msg.info
        return cls(msg.data, i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y)

    def to_cell(self, x, y):
        return int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def to_world(self, r, c):
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res

    def value(self, x, y, default=UNKNOWN):
        r, c = self.to_cell(x, y)
        if 0 <= r < self.h and 0 <= c < self.w:
            return int(self.a[r, c])
        return default


def find_frontiers(grid, min_size_m=0.3, free_max=10):
    """คืน list ของกลุ่มขอบ: {'cells': [(r,c)...], 'size_m': ความยาวโดยประมาณ, 'centroid': (x,y)}"""
    a = grid.a
    free = (a >= 0) & (a <= free_max)
    unk = a == UNKNOWN
    touch = np.zeros_like(unk)
    touch[1:, :] |= unk[:-1, :]
    touch[:-1, :] |= unk[1:, :]
    touch[:, 1:] |= unk[:, :-1]
    touch[:, :-1] |= unk[:, 1:]
    fr = free & touch

    cells = set(zip(*np.nonzero(fr)))
    seen = set()
    out = []
    min_cells = max(1, int(round(min_size_m / grid.res)))
    for start in cells:
        if start in seen:
            continue
        group = []
        q = deque([start])
        seen.add(start)
        while q:
            r, c = q.popleft()
            group.append((int(r), int(c)))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nb = (r + dr, c + dc)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        q.append(nb)
        if len(group) < min_cells:
            continue
        xs, ys = zip(*(grid.to_world(r, c) for r, c in group))
        out.append({
            'cells': group,
            'size_m': len(group) * grid.res,
            'centroid': (sum(xs) / len(xs), sum(ys) / len(ys)),
        })
    return out


def standoff_point(frontier, grid, costmap, max_cost=40, search_m=0.6):
    """
    หาจุดยืนที่ปลอดภัยใกล้ขอบ: ช่องที่แมพบอกว่าว่าง และ costmap ต่ำกว่า max_cost
    เลือกช่องที่ใกล้จุดกลางของขอบที่สุด (ไม่ยืนบนเส้นขอบพอดี เพราะอีกฝั่งยังไม่รู้ว่ามีอะไร)
    คืน (x, y) หรือ None
    """
    cx, cy = frontier['centroid']
    # ช่องขอบที่ใกล้จุดกลางที่สุด = จุดตั้งต้นของการค้น
    r0, c0 = min(frontier['cells'], key=lambda rc: math.hypot(*(np.subtract(grid.to_world(*rc), (cx, cy)))))
    n = int(search_m / grid.res)
    best, best_d = None, float('inf')
    for dr in range(-n, n + 1):
        for dc in range(-n, n + 1):
            r, c = r0 + dr, c0 + dc
            if not (0 <= r < grid.h and 0 <= c < grid.w):
                continue
            if not (0 <= grid.a[r, c] <= 10):
                continue
            x, y = grid.to_world(r, c)
            cost = costmap.value(x, y) if costmap is not None else 0
            if cost < 0 or cost > max_cost:
                continue
            d = math.hypot(dr, dc) * grid.res
            # อย่ายืนติดขอบพอดี: ต้องห่างช่อง unknown อย่างน้อย 2 ช่อง
            rr0, rr1 = max(0, r - 2), min(grid.h, r + 3)
            cc0, cc1 = max(0, c - 2), min(grid.w, c + 3)
            if (grid.a[rr0:rr1, cc0:cc1] == UNKNOWN).any():
                continue
            if d < best_d:
                best, best_d = (x, y), d
    return best


def choose_target(frontiers, grid, costmap, robot_xy, home_xy, blacklist,
                  max_radius=6.0, blacklist_radius=0.5, w_size=1.0, w_dist=1.0, max_cost=40,
                  size_cap_m=2.0):
    """
    ให้คะแนนแต่ละขอบ: ขอบยาว = ได้พื้นที่ใหม่เยอะ (บวก)  ไกลจากหุ่น = เสียเวลา (ลบ)
    ความยาวนับแค่ไม่เกิน size_cap_m -- แมพจริง 2026-09-17 มีขอบยาว 75 m เพราะลายรัศมีแตกของ lidar
    ถ้าไม่จำกัด หุ่นจะวิ่งไปไกล 4.6 m ทั้งที่มีขอบใกล้ ๆ ให้สำรวจก่อน
    ข้ามขอบที่เคยไปไม่ถึง (blacklist) และที่ไกลจากจุดเริ่มเกิน max_radius
    คืน (target_xy, frontier, score) ของตัวที่ดีที่สุด หรือ None
    """
    best = None
    for f in sorted(frontiers, key=lambda f: -f['size_m']):
        p = standoff_point(f, grid, costmap, max_cost=max_cost)
        if p is None:
            continue
        if math.hypot(p[0] - home_xy[0], p[1] - home_xy[1]) > max_radius:
            continue
        if any(math.hypot(p[0] - bx, p[1] - by) < blacklist_radius for bx, by in blacklist):
            continue
        dist = math.hypot(p[0] - robot_xy[0], p[1] - robot_xy[1])
        if dist < 0.3:
            continue            # ยืนอยู่ตรงนั้นแล้ว lidar ควรเห็นอยู่แล้ว ขอบค้างเพราะมองไม่ถึง
        score = w_size * min(f['size_m'], size_cap_m) - w_dist * dist
        if best is None or score > best[2]:
            best = (p, f, score)
    return best
