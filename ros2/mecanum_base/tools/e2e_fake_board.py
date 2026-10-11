"""เทสทั้งระบบ: รันผ่าน tools/e2e_fake_board.sh บน Pi (ต้องมี node + fake_board รันอยู่แล้ว)"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger

FAILS = []


def check(name, cond, detail):
    print(f"{'PASS' if cond else 'FAIL'}  {name}: {detail}", flush=True)
    if not cond:
        FAILS.append(name)


class T(Node):
    def __init__(self):
        super().__init__('mb_e2e')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom = None
        self.n = 0
        self.create_subscription(Odometry, 'wheel_odom', self.cb, 20)
        self.reset = self.create_client(Trigger, 'mecanum_base/reset_odom')
        self.estop = self.create_client(Trigger, 'mecanum_base/estop')
        self.clear = self.create_client(Trigger, 'mecanum_base/clear_fault')
        self.enable = self.create_client(SetBool, 'mecanum_base/enable')

    def cb(self, m):
        self.odom = m
        self.n += 1


def tw(vx=0.0, vy=0.0, wz=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
    return t


def run(node, sec, twist=None, once=False):
    end = time.monotonic() + sec
    last = 0.0
    sent = False
    while time.monotonic() < end:
        now = time.monotonic()
        if twist is not None and now - last >= 0.05 and not (once and sent):
            node.pub.publish(twist)
            last, sent = now, True
        rclpy.spin_once(node, timeout_sec=0.01)


def st(node):
    o = node.odom
    p, q, v = o.pose.pose.position, o.pose.pose.orientation, o.twist.twist
    return p.x, p.y, 2.0 * math.atan2(q.z, q.w), v.linear.x, v.linear.y, v.angular.z


def call(node, cli, req):
    assert cli.wait_for_service(timeout_sec=5.0), 'service not available'
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    return fut.result()


def fmt(s):
    return 'x=%.4f y=%.4f yaw=%.4f vx=%.3f vy=%.3f wz=%.3f' % s


def main():
    rclpy.init()
    n = T()

    t0 = time.monotonic()
    while n.n == 0 and time.monotonic() - t0 < 15.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    check('odom arrives', n.n > 0, f'{n.n} msgs after {time.monotonic() - t0:.1f}s')
    if n.n == 0:
        sys.exit(1)
    c0 = n.n
    run(n, 2.0)
    check('odom rate ~30 Hz', 50 <= n.n - c0 <= 70, f'{n.n - c0} msgs in 2 s')

    def leg(name, twist, sec, expect, tol_pos, idx_vel, want_vel):
        call(n, n.reset, Trigger.Request())
        run(n, 0.3)
        run(n, sec / 2, twist)
        mid = st(n)
        run(n, sec / 2, twist)
        run(n, 1.2, tw())                 # explicit zero -> ramp down
        end = st(n)
        pos_ok = all(abs(a - b) <= tol_pos for a, b in zip(end[:3], expect))
        check(f'{name} pose', pos_ok, f'{fmt(end)}  expect x,y,yaw={expect}')
        check(f'{name} twist mid-run', abs(mid[idx_vel] - want_vel) < 0.01 * max(1, abs(want_vel) * 10),
              f'{fmt(mid)}')
        check(f'{name} stopped', all(abs(v) < 1e-3 for v in end[3:]), f'twist={end[3:]}')

    if 'rotate_repeat' in sys.argv:
        # yaw error source check: timing jitter scatters both ways, a scale bug is always the same sign
        for i in range(5):
            leg(f'rotate #{i + 1}', tw(wz=0.5), 2.0, (0.0, 0.0, 1.0), 0.15, 5, 0.5)
        for i in range(3):
            leg(f'forward #{i + 1}', tw(vx=0.1), 2.0, (0.20, 0.0, 0.0), 0.03, 3, 0.1)
        print(f'\n{len(FAILS)} FAIL(s): {FAILS}' if FAILS else '\nALL PASS', flush=True)
        rclpy.shutdown()
        sys.exit(1 if FAILS else 0)

    # 0.1 m/s * 2 s = 0.20 m (ramp up/down cancel); yaw 0.5 rad/s * 2 s = 1.0 rad
    leg('forward', tw(vx=0.1), 2.0, (0.20, 0.0, 0.0), 0.03, 3, 0.1)
    leg('strafe left', tw(vy=0.1), 2.0, (0.0, 0.20, 0.0), 0.03, 4, 0.1)
    leg('rotate ccw', tw(wz=0.5), 2.0, (0.0, 0.0, 1.0), 0.15, 5, 0.5)

    # clamp: vx 1.0 -> max_vx 0.3
    call(n, n.reset, Trigger.Request())
    run(n, 1.5, tw(vx=1.0))
    s = st(n)
    check('clamp max_vx', abs(s[3] - 0.3) < 0.005, fmt(s))
    run(n, 1.5, tw())

    # wheel-rate limit keeps direction: vx .3 vy .3 wz 1 -> FR 4175 step/s > 3200 -> scale 0.7664
    run(n, 2.0, tw(vx=0.3, vy=0.3, wz=1.0))
    s = st(n)
    scale = s[3] / 0.3
    check('wheel limit scales uniformly',
          abs(s[4] / 0.3 - scale) < 0.01 and abs(s[5] / 1.0 - scale) < 0.01 and 0.74 < scale < 0.79,
          f'{fmt(s)}  scale={scale:.3f}')
    run(n, 2.0, tw())

    # cmd_vel timeout: single message, node must stop by itself after 0.5 s
    call(n, n.reset, Trigger.Request())
    run(n, 0.3)
    run(n, 2.5, tw(vx=0.2), once=True)
    s = st(n)
    check('cmd_timeout stops robot', all(abs(v) < 1e-3 for v in s[3:]) and 0.04 < s[0] < 0.16, fmt(s))

    # estop blocks motion until clear + enable
    r = call(n, n.estop, Trigger.Request())
    run(n, 0.5)
    call(n, n.reset, Trigger.Request())
    run(n, 1.5, tw(vx=0.1))
    s = st(n)
    check('estop blocks cmd_vel', abs(s[0]) < 1e-6 and abs(s[3]) < 1e-6, f'{r.message} | {fmt(s)}')
    call(n, n.clear, Trigger.Request())
    run(n, 1.0, tw(vx=0.1))
    s = st(n)
    check('after clear_fault still disabled', abs(s[3]) < 1e-6, fmt(s))
    call(n, n.enable, SetBool.Request(data=True))
    run(n, 1.5, tw(vx=0.1))
    s = st(n)
    check('enable resumes motion', abs(s[3] - 0.1) < 0.005, fmt(s))
    run(n, 1.2, tw())

    print(f'\n{len(FAILS)} FAIL(s): {FAILS}' if FAILS else '\nALL PASS', flush=True)
    rclpy.shutdown()
    sys.exit(1 if FAILS else 0)


if __name__ == '__main__':
    main()
