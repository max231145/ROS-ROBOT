"""motion_test -- ทดสอบความนุ่มของการเคลื่อนที่พื้นฐานผ่าน velocity_smoother (เหมือนตอน Nav2 สั่ง)
    python3 motion_test.py
ลำดับ: เปิดไดรเวอร์ -> เดินหน้า 0.15 m/s 3 s -> หยุด -> ถอย 3 s -> หยุด -> หมุนซ้าย 0.45 rad/s 3.5 s -> หยุด
       -> หมุนขวา 3.5 s -> หยุด -> ปิดไดรเวอร์ (ปิดทุกกรณี)
สั่งเข้า /cmd_vel_nav (ขาเข้า velocity_smoother) -> smoother -> collision_monitor -> /cmd_vel -> ฐาน
บันทึก /cmd_vel ที่ออกจริง + ความเร็วจาก rf2o (lidar) ทุก 0.05 s -> สรุปการกระตุก (ความเร่งสูงสุดที่วัดได้) ต่อช่วง
24 ก.ย. ผู้ใช้: เทสเดินหน้า/ถอย/หมุนรอบตัวให้เนียน (เดินข้างทำไม่ได้: ไดรเวอร์ 2 ตัว ล้อหน้า-หลังฝั่งเดียวกันผูกกัน)"""
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import SetBool

STEPS = [('เดินหน้า', 0.15, 0.0, 3.0), ('หยุด', 0, 0, 2.0), ('ถอยหลัง', -0.15, 0.0, 3.0), ('หยุด', 0, 0, 2.0),
         ('หมุนซ้าย', 0.0, 0.45, 3.5), ('หยุด', 0, 0, 2.0), ('หมุนขวา', 0.0, -0.45, 3.5), ('หยุด', 0, 0, 2.5)]


class T(Node):
    def __init__(self):
        super().__init__('motion_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.out = (0.0, 0.0)
        self.odo = None
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'out', (m.linear.x, m.angular.z)), 10)
        self.create_subscription(Odometry, '/odom_rf2o', self.on_odo, 10)
        self.en = self.create_client(SetBool, '/mecanum_base/enable')

    def on_odo(self, m):
        q = m.pose.pose.orientation
        self.odo = (time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def enable(self, on):
        self.en.wait_for_service(timeout_sec=5)
        f = self.en.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, f, timeout_sec=5)
        print('ไดรเวอร์', 'เปิด' if on else 'ปิด', f.result().success if f.result() else '?', flush=True)


rclpy.init()
n = T()
rows = []
try:
    n.enable(True)
    time.sleep(0.5)
    for name, vx, wz, dur in STEPS:
        t0 = time.monotonic()
        print(f'{name} {dur:.1f} s', flush=True)
        while time.monotonic() - t0 < dur:
            m = Twist()
            m.linear.x, m.angular.z = float(vx), float(wz)
            n.pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.05)
            rows.append((name, time.monotonic(), *n.out, *(n.odo or (0, 0, 0, 0))[1:]))
finally:
    for _ in range(5):
        n.pub.publish(Twist())
        time.sleep(0.05)
    n.enable(False)
# สรุป: ความเร็วจริงจาก rf2o (diff ทุก ~0.2 s) และความเร่งสูงสุดของคำสั่งที่ออก
print('ช่วง            คำสั่งสูงสุด v/w   ความเร่งคำสั่งสูงสุด   rf2o ระยะ/มุม')
for name, *_ in STEPS:
    seg = [r for r in rows if r[0] == name]
    if not seg:
        continue
    acc = max((abs(b[2] - a[2]) + abs(b[3] - a[3])) / max(b[1] - a[1], 1e-3) for a, b in zip(seg, seg[1:])) if len(seg) > 1 else 0
    d = math.hypot(seg[-1][4] - seg[0][4], seg[-1][5] - seg[0][5])
    dyaw = math.degrees(math.atan2(math.sin(seg[-1][6] - seg[0][6]), math.cos(seg[-1][6] - seg[0][6])))
    print(f'{name:10s} v{max(abs(r[2]) for r in seg):.2f} w{max(abs(r[3]) for r in seg):.2f}   {acc:5.2f}/s   {d * 100:5.1f} cm {dyaw:+6.1f}°')
