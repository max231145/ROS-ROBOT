"""
kinematics -- สมการล้อ mecanum 4 ล้อ แยกไว้ไม่พึ่ง ROS จะได้เทสบนเครื่องไหนก็ได้

ลำดับล้อ (ต้องตรงกับเฟิร์มแวร์): 0 = หน้าซ้าย  1 = หน้าขวา  2 = หลังซ้าย  3 = หลังขวา
แกนตาม REP-103: x ไปข้างหน้า  y ไปทางซ้าย  wz ทวนเข็มนาฬิกาเป็นบวก
ล้อหมุนเป็นบวก = ล้อพาหุ่นไปข้างหน้า

    w_fl = (vx - vy - k*wz) / r
    w_fr = (vx + vy + k*wz) / r
    w_rl = (vx + vy - k*wz) / r
    w_rr = (vx - vy + k*wz) / r

    k = ครึ่งระยะล้อหน้า-หลัง + ครึ่งระยะล้อซ้าย-ขวา

สมการชุดนี้ใช้ได้เมื่อลูกกลิ้งของล้อติดตั้งถูกทิศ ถ้าสั่งเดินข้างแล้วหุ่นหมุนตัวแทน
แปลว่าล้อสลับฝั่งกันอยู่ ต้องสลับล้อซ้ายกับขวาที่ตัวหุ่น แก้ในซอฟต์แวร์ไม่ได้
"""

import math

WHEEL_COUNT = 4
WHEEL_NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')


class MecanumKinematics:

    def __init__(self, wheel_radius: float, wheel_base: float, track_width: float):
        """
        wheel_radius : รัศมีล้อ (m)
        wheel_base   : ระยะระหว่างแกนล้อหน้ากับแกนล้อหลัง (m)
        track_width  : ระยะระหว่างกึ่งกลางหน้าล้อซ้ายกับขวา (m)
        """
        if wheel_radius <= 0.0 or wheel_base <= 0.0 or track_width <= 0.0:
            raise ValueError('wheel_radius, wheel_base, track_width ต้องมากกว่า 0')
        self.r = wheel_radius
        self.k = (wheel_base + track_width) / 2.0

    def inverse(self, vx: float, vy: float, wz: float):
        """ความเร็วตัวหุ่น (m/s, m/s, rad/s) -> ความเร็วเชิงมุมของล้อ (rad/s) 4 ค่า"""
        kw = self.k * wz
        return (
            (vx - vy - kw) / self.r,
            (vx + vy + kw) / self.r,
            (vx + vy - kw) / self.r,
            (vx - vy + kw) / self.r,
        )

    def forward(self, w):
        """
        ความเร็วเชิงมุมล้อ 4 ค่า (rad/s) -> (vx, vy, wz)
        ใส่เป็นมุมที่ล้อหมุนไป (rad) ก็ได้ จะได้ระยะที่หุ่นเคลื่อน (dx, dy, dth) ในกรอบตัวหุ่นแทน
        """
        fl, fr, rl, rr = w
        q = self.r / 4.0
        return (
            q * (fl + fr + rl + rr),
            q * (-fl + fr + rl - rr),
            q * (-fl + fr - rl + rr) / self.k,
        )


def limit_wheel_rates(rates, max_rate: float):
    """
    ถ้าล้อไหนเกินเพดาน ย่อทุกล้อด้วยสัดส่วนเดียวกัน
    ถ้า clamp ทีละล้อ ทิศการเคลื่อนที่ของหุ่นจะเพี้ยน เช่นสั่งเดินเฉียงแล้วกลายเป็นเดินตรง
    """
    peak = max(abs(v) for v in rates)
    if max_rate <= 0.0 or peak <= max_rate:
        return list(rates)
    s = max_rate / peak
    return [v * s for v in rates]


def step_delta(new: int, old: int) -> int:
    """ส่วนต่างตัวนับ int32 ของบอร์ด เผื่อกรณีตัวนับล้นวนรอบ"""
    d = (new - old) & 0xFFFFFFFF
    return d - 0x100000000 if d >= 0x80000000 else d


def integrate_pose(x: float, y: float, th: float, dx: float, dy: float, dth: float):
    """
    บวกระยะที่เคลื่อนในกรอบตัวหุ่น (dx, dy, dth) เข้ากับ pose ในกรอบ odom
    หมุนด้วยมุมกึ่งกลางช่วง ไม่ใช่มุมต้นช่วง ไม่งั้นเดินโค้งนาน ๆ แล้ว pose ลอยออกนอกวง
    """
    mid = th + dth / 2.0
    c, s = math.cos(mid), math.sin(mid)
    x += dx * c - dy * s
    y += dx * s + dy * c
    th = math.atan2(math.sin(th + dth), math.cos(th + dth))
    return x, y, th
