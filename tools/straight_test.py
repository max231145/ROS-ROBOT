#!/usr/bin/env python3
"""ทดสอบวิ่งตรง (ไม่ผ่าน Nav2) แยกว่า "ส่ายซ้ายขวา" มาจากล้อ/สเต็ปหาย หรือจากตัวควบคุม
แต่ละความเร็ว: เดินหน้า DIST m แล้วถอยกลับที่เดิม วัดจาก SLAM (map->base_link) เทียบกับล้อ (/wheel_odom)

    python3 ~/straight_test.py                  # 0.10 0.20 0.30 0.40 m/s
    python3 ~/straight_test.py 0.15 0.25        # เลือกความเร็วเอง

ต้องมีที่โล่งตรงไปข้างหน้า >= 2 m · ยืนพร้อมกด Ctrl+C (หยุดทันที) · ส่งคำสั่งผ่าน collision monitor ถ้า Nav2 เปิดอยู่
"""
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

DIST = 1.5          # m ต่อเที่ยว
ACC = 0.25          # m/s^2 เร่ง/ผ่อน (ตรงกับ velocity_smoother)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Test(Node):
    def __init__(self):
        super().__init__('straight_test')
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.wodom = None
        for t in ('/wheel_odom', '/mecanum_base/wheel_odom'):
            self.create_subscription(Odometry, t, self.on_odom, 20)
        self.pub_safe = self.create_publisher(Twist, 'cmd_vel_smoothed', 10)
        self.pub_raw = self.create_publisher(Twist, '/cmd_vel', 10)
        self.en = self.create_client(SetBool, '/mecanum_base/enable')

    def on_odom(self, m):
        p = m.pose.pose
        self.wodom = (p.position.x, p.position.y, yaw_of(p.orientation), m.twist.twist.linear.x, m.twist.twist.angular.z)

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform
        except TransformException:
            return None
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))


def main():
    speeds = [float(a) for a in sys.argv[1:]] or [0.10, 0.20, 0.30, 0.40]
    rclpy.init()
    n = Test()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    t_end = time.monotonic() + 15.0      # รอ TF + ค้นหา node (ครั้งแรกอาจใช้หลายวินาที)
    while time.monotonic() < t_end and (n.pose() is None or n.pub_safe.get_subscription_count() == 0):
        time.sleep(0.2)
    via_cm = n.pub_safe.get_subscription_count() > 0
    pub = n.pub_safe if via_cm else n.pub_raw
    print(f'ส่งคำสั่งทาง {"cmd_vel_smoothed (ผ่าน collision monitor)" if via_cm else "/cmd_vel ตรง (ไม่มี collision monitor!)"}')
    if n.pose() is None:
        print('ไม่ได้ TF map->base_link ใน 15 s -- SLAM ไม่ทำงาน?')
        rclpy.shutdown()
        return
    if n.wodom is None:
        print('ไม่ได้ /wheel_odom -- จะวัดจาก SLAM อย่างเดียว')
    if n.en.wait_for_service(timeout_sec=3.0):
        n.en.call_async(SetBool.Request(data=True))
        time.sleep(0.5)
    rows = []

    def stop():
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.02)

    def run(v):
        p0, w0 = n.pose(), n.wodom
        h0 = p0[2]
        ux, uy = math.cos(h0), math.sin(h0)
        cur, t_start, yaws, lat = 0.0, time.monotonic(), [], []
        last = time.monotonic()
        while True:
            p = n.pose()
            d = (p[0] - p0[0]) * ux + (p[1] - p0[1]) * uy
            remain = DIST - abs(d)
            if remain <= 0.01 or time.monotonic() - t_start > DIST / abs(v) * 3 + 5:
                break
            now = time.monotonic()
            dt, last = now - last, now
            target = math.copysign(min(abs(v), math.sqrt(2 * ACC * max(0.0, remain))), v)
            cur = min(cur + ACC * dt, target) if target > cur else max(cur - ACC * dt, target)
            if abs(cur) < 0.02:
                cur = math.copysign(0.02, v)
            t = Twist()
            t.linear.x = cur
            pub.publish(t)
            yaws.append(wrap(p[2] - h0))
            lat.append(-(p[0] - p0[0]) * uy + (p[1] - p0[1]) * ux)
            time.sleep(0.05)
        stop()
        time.sleep(0.8)
        p1, w1 = n.pose(), n.wodom
        slam_d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        r = {'v': v, 'slam_d': slam_d, 'yaw': math.degrees(wrap(p1[2] - h0)),
             'lat': (-(p1[0] - p0[0]) * uy + (p1[1] - p0[1]) * ux) * 100,
             'osc': math.degrees((max(yaws) - min(yaws)) if yaws else 0.0),
             'latmax': max((abs(x) for x in lat), default=0.0) * 100,
             'time': time.monotonic() - t_start}
        if w0 and w1:
            r['wheel_d'] = math.hypot(w1[0] - w0[0], w1[1] - w0[1])
            r['wheel_yaw'] = math.degrees(wrap(w1[2] - w0[2]))
        return r

    try:
        for v in speeds:
            for sgn, name in ((1, 'เดินหน้า'), (-1, 'ถอยกลับ')):
                print(f'\n>> {v:.2f} m/s {name} {DIST} m ...', flush=True)
                r = run(sgn * v)
                r['dir'] = name
                rows.append(r)
                wd = (f" · ล้อนับ {r['wheel_d']:.2f} m ({(r['wheel_d'] / r['slam_d'] - 1) * 100:+.0f}%) ล้อว่าหัน {r['wheel_yaw']:+.1f}°"
                      if 'wheel_d' in r and r['slam_d'] > 0.05 else '')
                print(f"   SLAM: ไป {r['slam_d']:.2f} m · หันเปลี่ยน {r['yaw']:+.1f}° · เบี่ยงข้าง {r['lat']:+.0f} cm "
                      f"(มากสุด {r['latmax']:.0f}) · ส่าย {r['osc']:.1f}°{wd}", flush=True)
                time.sleep(1.0)
    except KeyboardInterrupt:
        print('\nหยุด (Ctrl+C)')
    finally:
        stop()
    if not rows:
        rclpy.shutdown()
        return
    print('\n==== สรุป (บวก = เบี้ยวซ้าย / ลบ = เบี้ยวขวา) ====')
    print(' ความเร็ว ทิศ       | หันเปลี่ยน | เบี่ยงข้าง | ส่าย | ล้อนับต่างจาก SLAM | ล้อคิดว่าหัน')
    for r in rows:
        wd = (f"{(r['wheel_d'] / r['slam_d'] - 1) * 100:+5.0f}%     | {r['wheel_yaw']:+5.1f}°"
              if 'wheel_d' in r and r['slam_d'] > 0.05 else '   -       |   -')
        print(f" {r['v']:+.2f}  {r['dir']:8s} | {r['yaw']:+6.1f}°   | {r['lat']:+5.0f} cm  | {r['osc']:4.1f}° | {wd}")
    print('\nอ่านผล: หันเปลี่ยนไปทางเดียวกันทุกรอบ + ล้อคิดว่าไม่หัน = ล้อสองฝั่งหมุนจริงไม่เท่ากัน/ลื่น/สเต็ปหาย'
          '\n        ยิ่งเร็วยิ่งเบี้ยวมาก = สเต็ปหายตอนเร็ว · ส่ายมากแต่หันสุทธิน้อย = ไม่ใช่ล้อ (ไปดูตัวควบคุม)')


if __name__ == '__main__':
    main()
