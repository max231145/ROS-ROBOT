"""เทส kinematics ไม่ต้องมี ROS: python -m pytest test/  หรือ  python test/test_kinematics.py"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mecanum_base.kinematics import (  # noqa: E402
    MecanumKinematics, integrate_pose, limit_wheel_rates, step_delta)

KIN = MecanumKinematics(wheel_radius=0.05, wheel_base=0.20, track_width=0.24)


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_forward_is_inverse_of_inverse():
    for vx, vy, wz in [(0.3, 0, 0), (0, 0.2, 0), (0, 0, 1.0), (0.1, -0.25, 0.7), (-0.2, 0.1, -2.0)]:
        assert close(KIN.forward(KIN.inverse(vx, vy, wz)), (vx, vy, wz))


def test_straight_all_wheels_same():
    w = KIN.inverse(0.1, 0.0, 0.0)
    assert close(w, [2.0] * 4)                      # 0.1 m/s / r 0.05 = 2 rad/s


def test_strafe_left_pattern():
    fl, fr, rl, rr = KIN.inverse(0.0, 0.1, 0.0)     # y บวก = ไปทางซ้าย
    assert fl < 0 and rr < 0 and fr > 0 and rl > 0
    assert close([abs(fl), fr, rl, abs(rr)], [2.0] * 4)


def test_rotate_ccw_left_back_right_forward():
    fl, fr, rl, rr = KIN.inverse(0.0, 0.0, 1.0)
    assert fl < 0 and rl < 0 and fr > 0 and rr > 0
    assert abs(fr - (0.22 / 0.05)) < 1e-9           # k = (0.20 + 0.24) / 2


def test_limit_keeps_direction():
    rates = [4000.0, -1000.0, 1000.0, -4000.0]
    out = limit_wheel_rates(rates, 2000.0)
    assert max(abs(v) for v in out) == 2000.0
    assert close([v / out[0] for v in out], [v / rates[0] for v in rates])
    assert limit_wheel_rates([10.0, -5.0, 0.0, 1.0], 2000.0) == [10.0, -5.0, 0.0, 1.0]


def test_step_delta_wraps():
    assert step_delta(105, 100) == 5
    assert step_delta(-3, 2) == -5
    assert step_delta(-2147483648, 2147483647) == 1           # ล้นจากบวกสุดไปลบสุด
    assert step_delta(2147483647, -2147483648) == -1


def test_integrate_full_circle_returns_home():
    # หมุนรอบวงกลมด้วย vx คงที่ + wz คงที่ ครบรอบต้องกลับที่เดิม
    x = y = th = 0.0
    n = 2000
    vx, wz = 0.3, 2.0 * math.pi / 10.0                        # วงละ 10 s
    dt = 10.0 / n
    for _ in range(n):
        x, y, th = integrate_pose(x, y, th, vx * dt, 0.0, wz * dt)
    assert abs(x) < 1e-6 and abs(y) < 1e-6 and abs(th) < 1e-9


def test_integrate_strafe_in_rotated_frame():
    # หุ่นหันไปทาง +y ของ odom (90 องศา) แล้วเดินข้างไปทางซ้าย = เคลื่อนไปทาง -x ของ odom
    x, y, th = integrate_pose(0.0, 0.0, math.pi / 2, 0.0, 1.0, 0.0)
    assert close((x, y, th), (-1.0, 0.0, math.pi / 2))


def test_rejects_bad_geometry():
    for args in [(0.0, 0.2, 0.2), (0.05, -0.1, 0.2), (0.05, 0.2, 0.0)]:
        try:
            MecanumKinematics(*args)
        except ValueError:
            continue
        raise AssertionError(f'ควร error: {args}')


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for t in tests:
        t()
        print(f'ok  {t.__name__}')
    print(f'ผ่านทั้งหมด {len(tests)} เทส')
