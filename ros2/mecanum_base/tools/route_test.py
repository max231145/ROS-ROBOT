"""route_test -- เฝ้าการวิ่งหลายจุดด้วยปุ่ม START (waypoint_manager) แล้ววัดว่ากลับจุดเริ่มจริงแค่ไหนด้วย scan

    python3 route_test.py [WAIT_START_S=900] [MAX_RUN_S=480]
    python3 route_test.py --analyze ไฟล์.npz

ลำดับ: เก็บ scan s0 ตอนหุ่นนิ่ง -> รอเห็น "เริ่มวิ่ง" ใน /tmp/nav2.log (ผู้ใช้กด START ใน Foxglove)
       -> เช็คว่า 3 วิก่อน START /cmd_vel ไม่มีคำสั่งค้าง แล้วเปิดไดรเวอร์ (waypoint_manager รอ 1.5 s ก่อนส่งจุดแรก)
       -> บันทึก map->base_link, odom->base_link, map->odom ทุก 0.2 s -> จบเมื่อ waypoint_manager จบรอบ/ยกเลิก
          หรือครบ MAX_RUN_S (สั่ง cancel เอง) -> ปิดไดรเวอร์ทุกกรณี -> เก็บ scan s1 เทียบ s0
waypoint_manager กลับจุดเริ่มทุกครั้ง: ถ้า SLAM ตามตำแหน่งจริงได้ scan ต้องบอกว่าหุ่นกลับมาใกล้จุดเดิม
และ map->odom ต้องขยับตามที่ EKF คลาด (การกวาดข้างตอนหมุนที่ EKF ไม่เห็น)
"""
import math
import os
import signal
import sys
import time

import numpy as np

sys.path.insert(0, '/home/udru/ros-ws/src/mecanum_base/tools')
import lidar_offset_test as L   # noqa: E402  scan_points, match
import line_test as LT          # noqa: E402  laser_to_base

LOG = '/tmp/nav2.log'
END_WORDS = ('ครบแล้ว', 'ยกเลิกแล้ว', 'หยุดการวิ่ง', 'จบสำรวจ')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rel(p0, p1):
    """ตำแหน่ง p1 ในกรอบหุ่นตอน p0 (หน้า, ข้าง, หัน)"""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(p1[2] - p0[2])


def report(z):
    meta = tuple(float(x) for x in z['meta'])
    P0, P1 = (L.scan_points(z[k], *meta) for k in ('s0', 's1'))
    th, t, rms, frac = L.match(P1, P0)
    d, th = LT.laser_to_base(th, t)
    print(f'จริง (scan):  หน้า {d[0] * 100:+6.1f} cm  ข้าง {d[1] * 100:+6.1f} cm  หัน {math.degrees(th):+6.1f}°'
          f'   [rms {rms * 100:.1f} cm จับคู่ {frac * 100:.0f}%]   (+ข้าง = ฝั่งปลั๊กพ่วง)')
    for key, name in (('map', 'SLAM (map)'), ('odom', 'EKF (odom)')):
        f, s, h = rel(z[key + '0'], z[key + '1'])
        print(f'{name:12s}  หน้า {f * 100:+6.1f} cm  ข้าง {s * 100:+6.1f} cm  หัน {math.degrees(h):+6.1f}°')
    tr = z['track']       # t, map x y yaw, odom x y yaw, map->odom x y yaw
    if len(tr):
        mo = tr[:, 7:10]
        jump = np.hypot(*np.diff(mo[:, :2], axis=0).T) if len(mo) > 1 else np.zeros(1)
        print(f'map->odom (SLAM แก้ EKF): ตอนเริ่ม x={mo[0, 0]:+.3f} y={mo[0, 1]:+.3f} หัน {math.degrees(mo[0, 2]):+.1f}°'
              f'  ตอนจบ x={mo[-1, 0]:+.3f} y={mo[-1, 1]:+.3f} หัน {math.degrees(mo[-1, 2]):+.1f}°'
              f'  กระโดดครั้งใหญ่สุด {jump.max() * 100:.1f} cm  ({int((jump > 0.02).sum())} ครั้ง > 2 cm)')


if len(sys.argv) > 2 and sys.argv[1] == '--analyze':
    report(np.load(sys.argv[2]))
    raise SystemExit(0)

WAIT_START = float(sys.argv[1]) if len(sys.argv) > 1 else 900.0
MAX_RUN = float(sys.argv[2]) if len(sys.argv) > 2 else 480.0

import rclpy                                   # noqa: E402
from geometry_msgs.msg import Twist            # noqa: E402
from rclpy.node import Node                    # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.time import Time                    # noqa: E402
from sensor_msgs.msg import LaserScan          # noqa: E402
from std_srvs.srv import SetBool, Trigger      # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

rclpy.init()
n = Node('route_test')
buf = Buffer()
TransformListener(buf, n)
scans, moving_cmds = [], []
n.create_subscription(LaserScan, '/scan_fixed', scans.append, qos_profile_sensor_data)
n.create_subscription(Twist, '/cmd_vel', lambda m: moving_cmds.append(time.time())
                      if abs(m.linear.x) > 1e-3 or abs(m.linear.y) > 1e-3 or abs(m.angular.z) > 1e-3 else None, 10)


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


def pose(parent, child='base_link'):
    t = buf.lookup_transform(parent, child, Time()).transform
    return t.translation.x, t.translation.y, yaw_of(t.rotation)


def grab(count=10):
    scans.clear()
    while len(scans) < count:
        spin(0.1)
    m = scans[0]
    return np.array([s.ranges[:len(m.ranges)] for s in scans], dtype=float), m


def call(srv_type, name, req):
    c = n.create_client(srv_type, name)
    if not c.wait_for_service(timeout_sec=5):
        return None
    f = c.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    return f.result()


pending = ['']


def new_lines():
    """บรรทัดใหม่ที่ครบบรรทัดใน LOG (บรรทัดที่ launch เขียนยังไม่จบเก็บไว้ต่อรอบหน้า)"""
    chunk = pending[0] + log.read()
    *lines, pending[0] = chunk.split('\n')
    return lines


def enable(on):
    r = call(SetBool, '/mecanum_base/enable', SetBool.Request(data=on))
    return r.success if r else None


class Stop(Exception):
    pass


def on_term(*_):
    raise Stop()


signal.signal(signal.SIGTERM, on_term)

# รอ TF ให้ครบ
end = time.time() + 15
while True:
    spin(0.5)
    try:
        pose('map'), pose('odom')
        break
    except Exception:
        if time.time() > end:
            print('ไม่มี TF map/odom -> base_link -- ยกเลิก')
            raise SystemExit(1)

s0, m0 = grab()
meta = (m0.angle_min, m0.angle_increment, m0.range_min, m0.range_max)
log = open(LOG, encoding='utf-8', errors='replace')
log.seek(0, os.SEEK_END)
print(f'{time.strftime("%H:%M:%S")} เก็บ s0 แล้ว map={tuple(round(v, 3) for v in pose("map"))} -- รอกด START (สูงสุด {WAIT_START:.0f} s)', flush=True)

enabled = False
track = []
out = {}
try:
    # ---- รอ START ----
    t_wait = time.time()
    started = None
    while started is None:
        spin(0.1)
        for line in new_lines():
            if 'waypoint_manager' in line and 'เริ่มวิ่ง' in line:
                started = time.time()
                print(f'{time.strftime("%H:%M:%S")} เห็น START: {line.split("]: ")[-1].strip()}', flush=True)
        if started is None and time.time() - t_wait > WAIT_START:
            print('ไม่เห็น START ภายในเวลา -- จบโดยไม่เปิดไดรเวอร์', flush=True)
            raise SystemExit(0)
    if any(started - 3.0 <= t < started - 0.05 for t in moving_cmds):
        print('/cmd_vel มีคำสั่งค้างก่อน START -- ไม่เปิดไดรเวอร์ ยกเลิกการวิ่ง', flush=True)
        call(Trigger, '/waypoint_manager/cancel', Trigger.Request())
        raise SystemExit(1)
    map0, odom0 = pose('map'), pose('odom')
    enabled = bool(enable(True))
    print(f'เปิดไดรเวอร์: {enabled}  ({time.time() - started:.2f} s หลัง START)', flush=True)
    if not enabled:
        call(Trigger, '/waypoint_manager/cancel', Trigger.Request())
        raise SystemExit(1)

    # ---- วิ่ง ----
    t0 = time.time()
    last = 0.0
    finished = None
    while finished is None:
        spin(0.05)
        now = time.time()
        if now - last >= 0.2:
            last = now
            try:
                mp, op, mo = pose('map'), pose('odom'), pose('map', 'odom')
                track.append((now - t0, *mp, *op, *mo))
            except Exception:
                pass
        for line in new_lines():
            if 'waypoint_manager' in line or 'Failed to make progress' in line or '[ERROR]' in line:
                msg = line.split(']: ')[-1].strip()
                print(f'  {now - t0:6.1f} s  {msg[:150]}', flush=True)
                if 'waypoint_manager' in line and any(w in msg for w in END_WORDS):
                    finished = msg
        if finished is None and now - t0 > MAX_RUN:
            print(f'เกิน {MAX_RUN:.0f} s -- สั่ง cancel', flush=True)
            call(Trigger, '/waypoint_manager/cancel', Trigger.Request())
            finished = 'เกินเวลา'
    spin(2.0)
    enabled = not enable(False)
    print('ปิดไดรเวอร์:', not enabled, flush=True)
    spin(1.0)
    s1, _ = grab()
    map1, odom1 = pose('map'), pose('odom')
    out = dict(s0=s0, s1=s1, meta=np.array(meta), map0=map0, map1=map1, odom0=odom0, odom1=odom1,
               track=np.array(track).reshape(-1, 10))
except Stop:
    print('โดน SIGTERM', flush=True)
finally:
    if enabled:
        r = enable(False)
        print('ปิดไดรเวอร์ (finally):', r if r is not None else 'เรียก service ไม่ได้ -- ปิดเองด่วน!', flush=True)

if out:
    path = time.strftime('/home/udru/route_%H%M%S.npz')
    np.savez(path, **out)
    print('เก็บไว้ที่', path, flush=True)
    report(np.load(path))
n.destroy_node()
rclpy.shutdown()
