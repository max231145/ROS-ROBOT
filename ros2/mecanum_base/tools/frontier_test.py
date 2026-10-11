"""frontier_test -- รัน frontier.py กับแมพจริง พิมพ์ขอบ 10 อันดับแรกและเป้าที่เลือก (sys.path ชี้ /tmp -- แก้ให้ชี้ package ก่อนใช้)"""
import sys, time, math
sys.path.insert(0, '/tmp')
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from frontier import Grid, find_frontiers, choose_target, standoff_point

got = {}
rclpy.init(); n = Node('frontier_test'); buf = Buffer(); TransformListener(buf, n)
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
for t in ('/map', '/global_costmap/costmap'):
    n.create_subscription(OccupancyGrid, t, lambda m, t=t: got.setdefault(t, m), q)
tf = None; end = time.time() + 20
while time.time() < end and (len(got) < 2 or tf is None):
    rclpy.spin_once(n, timeout_sec=0.2)
    try: tf = buf.lookup_transform('map', 'base_link', Time())
    except Exception: pass
rx, ry = tf.transform.translation.x, tf.transform.translation.y
g = Grid.from_msg(got['/map']); cm = Grid.from_msg(got['/global_costmap/costmap'])
t0 = time.time(); fr = find_frontiers(g); t1 = time.time()
print(f'หุ่นอยู่ที่ ({rx:.2f}, {ry:.2f})  แมพ {g.w}x{g.h}  หาขอบใช้ {1000*(t1-t0):.0f} ms')
print(f'ขอบยาวเกิน 30 cm: {len(fr)} กลุ่ม  ยาวรวม {sum(f["size_m"] for f in fr):.1f} m')
rows = []
for f in fr:
    p = standoff_point(f, g, cm)
    d = math.hypot(p[0]-rx, p[1]-ry) if p else None
    rows.append((f['size_m'], f['centroid'], p, d))
for s, c, p, d in sorted(rows, key=lambda r: -r[0])[:10]:
    ps = f'จุดยืน ({p[0]:.2f},{p[1]:.2f}) ห่างหุ่น {d:.2f} m  cost {cm.value(*p)}' if p else 'หาจุดยืนปลอดภัยไม่ได้'
    print(f'  ขอบยาว {s:.2f} m  กลาง ({c[0]:.2f},{c[1]:.2f})  -> {ps}')
best = choose_target(fr, g, cm, (rx, ry), (rx, ry), [])
t2 = time.time()
if best:
    print(f"เลือก: ({best[0][0]:.2f},{best[0][1]:.2f}) คะแนน {best[2]:.2f} ขอบยาว {best[1]['size_m']:.2f} m  (ทั้งหมด {1000*(t2-t0):.0f} ms)")
else:
    print("เลือก: ไม่มี")
n.destroy_node(); rclpy.shutdown()
