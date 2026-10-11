"""line_test -- เดินหน้า DIST m แล้วถอยกลับจุดเดิม วัดของจริงด้วย scan (ไม่พึ่ง SLAM/ล้อ)

    python3 line_test.py [DIST=1.0] [SPEED=0.1] [PAUSE=8]
    python3 line_test.py --analyze ไฟล์.npz

ลำดับ: เปิดไดรเวอร์ -> เก็บ scan s0 -> เดินหน้า 0.10 m เช็คด้วย lidar ว่าไปข้างหน้าจริงและไม่หมุน
       (หลังสลับล้อ 23 ก.ย. -- ถ้าใส่ผิดหุ่นอาจหมุน/ไถลข้าง จะหยุดทันที) -> เดินหน้าที่เหลือ -> พัก PAUSE วิ
       (เก็บ s1 ให้ผู้ใช้ดูตลับเมตร) -> ถอยหลัง DIST m -> เก็บ s2 -> ปิดไดรเวอร์
วัดการเคลื่อนที่ของหุ่นจาก scan: จับคู่ scan กับ s0 (ค้นรอบค่าที่ล้อนับ แล้ว ICP) แปลงเป็นกรอบหุ่นด้วย TF lidar
"""
import math
import sys
import time

import numpy as np

sys.path.insert(0, '/home/udru/ros-ws/src/mecanum_base/tools')
import lidar_offset_test as L   # noqa: E402  scan_points, icp, rot

LIDAR = np.array([0.17, 0.076])   # TF base_link -> laser_frame (yaw = pi)


def seeded_match(src, dst, th0, t0, span_deg=15.0, span_t=0.35, res=0.05, half=12.0):
    """ค้นรอบค่าเริ่ม (มุม +-span_deg, เลื่อน +-span_t) ด้วยตาราง 5 cm แล้ว ICP"""
    N = int(2 * half / res)
    grid = np.zeros((N, N), bool)
    ij = ((dst + half) / res).astype(int)
    ok = ((ij >= 0) & (ij < N)).all(axis=1)
    grid[ij[ok, 0], ij[ok, 1]] = True
    g = grid.copy()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            g |= np.roll(np.roll(grid, dx, 0), dy, 1)
    ts = np.arange(-span_t, span_t + 1e-9, 0.03)
    TT = np.array([(t0[0] + a, t0[1] + b) for a in ts for b in ts])
    best = (-1.0, th0, t0)
    for th in th0 + np.radians(np.arange(-span_deg, span_deg + 1e-9, 0.5)):
        q = (src @ L.rot(th).T)[None, :, :] + TT[:, None, :]
        ij = np.clip(((q + half) / res).astype(int), 0, N - 1)
        score = g[ij[..., 0], ij[..., 1]].mean(axis=1)
        k = int(score.argmax())
        if score[k] > best[0]:
            best = (float(score[k]), float(th), TT[k])
    R, t, rms, frac = L.icp(src, dst, L.rot(best[1]), np.array(best[2]))
    return math.atan2(R[1, 0], R[0, 0]), t, rms, frac


def laser_to_base(th, t):
    """การเคลื่อนที่ของเฟรม lidar (ในกรอบ lidar ตอนเริ่ม) -> การเคลื่อนที่ของ base_link (ในกรอบหุ่นตอนเริ่ม)
    lidar: p_B = l + R(pi) p_L  ->  base เลื่อน d = (I - R(th)) l + R(pi) t = (I - R) l - t"""
    return (np.eye(2) - L.rot(th)) @ LIDAR - t, th


def base_to_laser_seed(d, th):
    return (np.eye(2) - L.rot(th)) @ LIDAR - d


def pts(ranges, meta):
    return L.scan_points(ranges, *meta)


def describe(label, d, th, rms, frac):
    print(f'{label}: lidar วัดได้ เดินหน้า {d[0] * 100:+6.1f} cm  ข้าง {d[1] * 100:+5.1f} cm (+ = ฝั่งปลั๊กพ่วง)'
          f'  หัน {math.degrees(th):+5.1f}°   [rms {rms * 100:.1f} cm จับคู่ {frac * 100:.0f}%]', flush=True)


def analyze(path):
    z = np.load(path)
    meta = tuple(float(x) for x in z['meta'])
    P0 = pts(z['s0'], meta)
    for key, wkey in (('s1', 'w1'), ('s2', 'w2')):
        w = z[wkey] - z['w0']                      # (dx, dy, dyaw) ของล้อ ในกรอบ odom (เริ่มที่ 0)
        th, t, rms, frac = seeded_match(pts(z[key], meta), P0, float(w[2]), base_to_laser_seed(w[:2], float(w[2])))
        d, th = laser_to_base(th, t)
        describe('ปลายทาง 1 m' if key == 's1' else 'กลับจุดเริ่ม', d, th, rms, frac)
        print(f'{"":14s}ล้อนับ     เดินหน้า {w[0] * 100:+6.1f} cm  ข้าง {w[1] * 100:+5.1f} cm  หัน {math.degrees(w[2]):+5.1f}°')


def capture(dist, speed, pause):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import SetBool

    rclpy.init()
    n = Node('line_test')
    scans, cmds, st = [], [], {}
    n.create_subscription(LaserScan, '/scan_fixed', scans.append, qos_profile_sensor_data)
    n.create_subscription(Twist, '/cmd_vel', cmds.append, 10)
    n.create_subscription(Odometry, '/wheel_odom', lambda m: st.__setitem__('w', m), 10)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    def wheel():
        p = st['w'].pose.pose
        q = p.orientation
        return np.array([p.position.x, p.position.y,
                         math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))])

    def grab(count=10):
        scans.clear()
        while len(scans) < count:
            spin(0.1)
        m = scans[0]
        return np.array([s.ranges[:len(m.ranges)] for s in scans], dtype=float), m

    def call_enable(on):
        c = n.create_client(SetBool, '/mecanum_base/enable')
        if not c.wait_for_service(timeout_sec=5):
            return None
        f = c.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(n, f, timeout_sec=10)
        return f.result()

    def drive(vx, sec):
        tw = Twist()
        tw.linear.x = vx
        end = time.time() + sec
        while time.time() < end:
            pub.publish(tw)
            spin(0.05)
        for _ in range(20):
            pub.publish(Twist())
            spin(0.05)

    spin(3.0)
    if 'w' not in st:
        print('ไม่มี /wheel_odom -- ไม่วิ่ง')
        raise SystemExit(1)
    cmds.clear()
    spin(2.0)
    if any(abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3 for m in cmds):
        print('/cmd_vel มีคำสั่งค้าง -- ไม่เปิดไดรเวอร์')
        raise SystemExit(1)

    s0, m0 = grab()
    meta = (m0.angle_min, m0.angle_increment, m0.range_min, m0.range_max)
    P0 = pts(s0, meta)
    # ทางข้างหน้า: จุด lidar ในแนววิ่ง (กรอบหุ่น) x 0.27..dist+0.4 |y| < 0.32
    pb = LIDAR + (-P0)            # p_B = l - p_L
    blk = pb[(pb[:, 0] > 0.27) & (pb[:, 0] < dist + 0.4) & (np.abs(pb[:, 1]) < 0.32)]
    if len(blk):
        print(f'ทางข้างหน้าไม่โล่ง: มีจุด {len(blk)} จุด ใกล้สุด {blk[:, 0].min():.2f} m -- ไม่วิ่ง')
        raise SystemExit(1)
    w0 = wheel()
    out = {}
    try:
        r = call_enable(True)
        print('เปิดไดรเวอร์:', r.success if r else 'ไม่มี service', flush=True)
        if not r or not r.success:
            raise SystemExit(1)
        spin(1.0)
        # ขั้นเช็ค 10 cm
        drive(speed, 0.10 / speed)
        spin(1.5)
        sc, _ = grab(6)
        wc = wheel() - w0
        th, t, rms, frac = seeded_match(pts(sc, meta), P0, float(wc[2]), base_to_laser_seed(wc[:2], float(wc[2])))
        d, th = laser_to_base(th, t)
        describe('เช็ค 10 cm', d, th, rms, frac)
        if not (0.05 < d[0] < 0.16 and abs(d[1]) < 0.05 and abs(math.degrees(th)) < 5):
            print('ไม่ผ่าน: หุ่นไม่ได้เดินหน้าตรงตามสั่ง (หลังสลับล้อ?) -- หยุดเทส')
            raise SystemExit(1)
        drive(speed, (dist - 0.10) / speed)
        spin(1.5)
        s1, _ = grab()
        w1 = wheel()
        print(f'ถึง {dist:.1f} m แล้ว พัก {pause:.0f} วิ', flush=True)
        spin(pause)
        drive(-speed, dist / speed)
        spin(1.5)
        s2, _ = grab()
        w2 = wheel()
        out = dict(s0=s0, s1=s1, s2=s2, w0=w0, w1=w1, w2=w2, meta=np.array(meta))
    finally:
        r = call_enable(False)
        print('ปิดไดรเวอร์:', r.success if r else 'เรียก service ไม่ได้ -- ปิดเองด่วน!', flush=True)
        n.destroy_node()
        rclpy.shutdown()
    path = time.strftime('/home/udru/line_test_%H%M%S.npz')
    np.savez(path, **out)
    print('เก็บ scan ไว้ที่', path, flush=True)
    return path


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == '--analyze':
        analyze(sys.argv[2])
    else:
        a = [float(x) for x in sys.argv[1:]]
        p = capture(a[0] if a else 1.0, a[1] if len(a) > 1 else 0.1, a[2] if len(a) > 2 else 8.0)
        analyze(p)
