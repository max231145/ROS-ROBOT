"""explore_run -- เริ่มสำรวจอัตโนมัติ (waypoint_manager ~/explore) แล้วเฝ้าจนจบ: เปิดไดรเวอร์ให้ ปิดทุกกรณีตอนจบ

    python3 explore_run.py [MAX_S=660]

ลำดับ: เช็ค /cmd_vel ไม่มีคำสั่งค้าง 2 วิ -> เปิดไดรเวอร์ -> เรียก /waypoint_manager/explore
       -> พิมพ์ข้อความของ waypoint_manager จาก /tmp/nav2.log + บันทึกตำแหน่งทุก 0.2 s
       -> จบเมื่อ waypoint_manager จบรอบ ("จบสำรวจ") / ยกเลิก (ปุ่ม STOP ใน Foxglove) / ครบ MAX_S (สั่ง cancel เอง)
       -> ปิดไดรเวอร์ -> สรุป: ห่างจุดเริ่มไกลสุด, หมุนสะสม, กลับถึงจุดเริ่มคลาดเท่าไร (ตาม SLAM)
รัศมีสำรวจตั้งที่ waypoint_manager (explore_max_radius) -- 23 ก.ย.: 1 m เพราะสายแลน/ปลั๊กพ่วงสั้น ขยายต้องถามผู้ใช้
"""
import math
import os
import signal
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

LOG = '/tmp/nav2.log'


def is_end(msg):
    """ข้อความ finish() ของ waypoint_manager เท่านั้น -- ห้ามจับแค่ว่ามีคำ "ยกเลิกแล้ว" อยู่ในประโยค
    (23 ก.ย. รั้วหมุนสะสมพิมพ์ "...ยกเลิกแล้วกลับจุดเริ่ม" สคริปต์ปิดไดรเวอร์ทั้งที่หุ่นยังไม่ถึงบ้าน)"""
    m = msg.strip()
    return m == 'ยกเลิกแล้ว' or m.startswith('จบสำรวจ') or m.startswith('ครบแล้ว') or m.endswith('หยุดการวิ่ง')
MAX_S = float(sys.argv[1]) if len(sys.argv) > 1 else 660.0

rclpy.init()
n = Node('explore_run')
buf = Buffer()
TransformListener(buf, n)
moving = []
n.create_subscription(Twist, '/cmd_vel', lambda m: moving.append(time.time())
                      if abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3 else None, 10)


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose():
    t = buf.lookup_transform('map', 'base_link', Time()).transform
    return t.translation.x, t.translation.y, yaw_of(t.rotation)


def call(srv_type, name, req, timeout=10.0):
    c = n.create_client(srv_type, name)
    if not c.wait_for_service(timeout_sec=5):
        return None
    f = c.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=timeout)
    return f.result()


def enable(on):
    r = call(SetBool, '/mecanum_base/enable', SetBool.Request(data=on))
    return r.success if r else None


class Stop(Exception):
    pass


def on_term(*_):
    raise Stop()


signal.signal(signal.SIGTERM, on_term)

end = time.time() + 15
while True:
    spin(0.5)
    try:
        home = pose()
        break
    except Exception:
        if time.time() > end:
            print('ไม่มี TF map -> base_link -- ยกเลิก')
            raise SystemExit(1)

log = open(LOG, encoding='utf-8', errors='replace')
log.seek(0, os.SEEK_END)
pending = ['']


def new_lines():
    chunk = pending[0] + log.read().replace('\x00', '')
    *lines, pending[0] = chunk.split('\n')
    return lines


moving.clear()
spin(2.0)
if moving:
    print('/cmd_vel มีคำสั่งค้าง -- ไม่เปิดไดรเวอร์')
    raise SystemExit(1)

enabled = False
track = []
try:
    enabled = bool(enable(True))
    print(f'{time.strftime("%H:%M:%S")} เปิดไดรเวอร์: {enabled}', flush=True)
    if not enabled:
        raise SystemExit(1)
    r = call(Trigger, '/waypoint_manager/explore', Trigger.Request())
    print('explore:', r.message if r else 'ไม่มี service', flush=True)
    if not r or not r.success:
        raise SystemExit(1)
    t0 = time.time()
    last = 0.0
    finished = None
    while finished is None:
        spin(0.05)
        now = time.time()
        if now - last >= 0.2:
            last = now
            try:
                track.append((now - t0, *pose()))
            except Exception:
                pass
        for line in new_lines():
            if 'waypoint_manager' in line or 'collision_monitor' in line and 'stop' in line.lower() \
                    or 'Failed to make progress' in line or 'backup' in line.lower():
                msg = line.split(']: ')[-1].strip()
                print(f'  {now - t0:6.1f} s  {msg[:160]}', flush=True)
                if 'waypoint_manager' in line and is_end(msg):
                    finished = msg
        if finished is None and now - t0 > MAX_S:
            print(f'เกิน {MAX_S:.0f} s -- สั่ง cancel', flush=True)
            call(Trigger, '/waypoint_manager/cancel', Trigger.Request())
            finished = 'เกินเวลา'
    spin(1.5)
except Stop:
    print('โดน SIGTERM -- สั่ง cancel', flush=True)
    call(Trigger, '/waypoint_manager/cancel', Trigger.Request())
finally:
    if enabled:
        r = enable(False)
        print('ปิดไดรเวอร์:', r if r is not None else 'เรียก service ไม่ได้ -- ปิดเองด่วน!', flush=True)

if track:
    tr = np.array(track)
    path = time.strftime('/home/udru/explore_%H%M%S.npz')
    np.savez(path, track=tr, home=np.array(home))     # track: t, x, y, yaw (map)
    print('เก็บเส้นทางไว้ที่', path, flush=True)
    d = np.hypot(tr[:, 1] - home[0], tr[:, 2] - home[1])
    yaw = np.unwrap(tr[:, 3])
    print(f'สรุป: ใช้เวลา {tr[-1, 0]:.0f} s  ห่างจุดเริ่มไกลสุด {d.max():.2f} m  '
          f'หมุนสะสม {math.degrees(yaw[-1] - yaw[0]):+.0f}° (มากสุด {math.degrees(np.abs(yaw - yaw[0]).max()):.0f}°)  '
          f'จบห่างจุดเริ่ม {d[-1] * 100:.0f} cm หัน {math.degrees(tr[-1, 3] - home[2]):+.0f}° (ตาม SLAM)', flush=True)
n.destroy_node()
rclpy.shutdown()
