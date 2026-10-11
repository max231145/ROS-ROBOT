"""lidar_offset_test -- หาตำแหน่ง lidar บนหุ่นจริง ๆ จาก scan ก่อน-หลังหมุนอยู่กับที่ (ไม่พึ่ง SLAM)

    python3 lidar_offset_test.py [องศา=90] [rad/s=0.3]     หมุนซ้าย-เก็บ scan-หมุนกลับ แล้ววิเคราะห์
    python3 lidar_offset_test.py --analyze ไฟล์.npz         วิเคราะห์ไฟล์ที่เก็บไว้ซ้ำ (ไม่ขยับหุ่น)

ทำไมไม่ใช้ SLAM: slam_toolbox รับ scan ใหม่เฉพาะตอนหุ่นเคลื่อนที่ >= minimum_travel_distance
หมุนอยู่กับที่ไม่นับ (23 ก.ย. หมุน 90° สองรอบ SLAM = EKF เป๊ะ กราฟมี node เดียว)

หลักการ: จับคู่ scan หลังหมุนกับก่อนหมุน (ค้นทุกมุมด้วยตาราง แล้ว ICP) ได้ p_L0 = R(th) p_L1 + t
- th = มุมที่หุ่นหมุนจริง (เทียบกับที่ล้อนับ)
- จุดหมุนในกรอบ lidar c = (I - R)^-1 t -> ในกรอบหุ่น = l - c (lidar yaw = pi)
  ถ้าหุ่นหมุนรอบกลางตัวจริง จุดหมุนจะอยู่ ~(0, 0) และใช้หาตำแหน่ง lidar ได้ (l = c)
เก็บ scan ตอนหยุดนิ่ง 3 ช่วง: ก่อนหมุน / หลังหมุนซ้าย / หลังหมุนกลับ (ช่วงสุดท้ายไว้เช็คว่า ICP กลับมาที่ศูนย์)
23 ก.ย.: สั่ง 90° หมุนจริง 45° ทั้งที่ 0.3 และ 0.15 rad/s จุดหมุนอยู่หลังกลางหุ่น ~45 cm
"""
import math
import sys
import time

import warnings

import numpy as np

warnings.filterwarnings("ignore", "All-NaN slice")

ODOM_TOPICS = ['/wheel_odom', '/odom_rf2o', '/odom_rf2o_fixed', '/odometry/filtered']   # _fixed = rf2o ตัวเทสบน /scan_fixed
LASER_YAW = math.pi      # lidar ติดกลับด้าน (23 ก.ย.) -- สูตรข้างบนใช้ R(pi) = -I


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def scan_points(ranges, amin, ainc, rmin, rmax):
    """median ต่อ beam จากหลาย scan -> จุดในเฟรม lidar"""
    r = np.nanmedian(np.where((ranges > rmin) & (ranges < rmax), ranges, np.nan), axis=0)
    a = amin + ainc * np.arange(r.size)
    ok = np.isfinite(r)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def icp(src, dst, R, t, iters=60):
    """point-to-point ICP 2D (Kabsch) -- คืน R, t, rms, สัดส่วนจุดที่จับคู่ได้"""
    for i in range(iters):
        gate = max(0.08, 0.5 * (1 - i / 30))
        p = src @ R.T + t
        d2 = ((p[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2)
        j = d2.argmin(axis=1)
        dmin = np.sqrt(d2[np.arange(len(p)), j])
        m = dmin < gate
        if m.sum() < 20:
            break
        a, b = src[m], dst[j[m]]
        ca, cb = a.mean(axis=0), b.mean(axis=0)
        U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
        Rn = (U @ Vt).T
        if np.linalg.det(Rn) < 0:
            Vt[1] *= -1
            Rn = (U @ Vt).T
        R, t = Rn, cb - ca @ Rn.T
    p = src @ R.T + t
    d = np.sqrt(((p[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2).min(axis=1))
    m = d < 0.08
    return R, t, float(np.sqrt((d[m] ** 2).mean())) if m.any() else 9.9, float(m.mean())


def raster_search(src, dst, res=0.05, half=12.0):
    """ค้นทุกมุม -180..180 (ทีละ 1°) + เลื่อน +-0.6 m ด้วยตารางช่อง 5 cm ของ dst -- เร็ว (~5 วิ) ไม่พึ่งค่าจากล้อ
    (23 ก.ย. ล้อบอก 90° แต่หมุนจริง 45° ถ้าค้นแค่รอบ ๆ ค่าล้อจะไม่เจอเลย)"""
    N = int(2 * half / res)
    grid = np.zeros((N, N), bool)
    ij = ((dst + half) / res).astype(int)
    ok = ((ij >= 0) & (ij < N)).all(axis=1)
    grid[ij[ok, 0], ij[ok, 1]] = True
    g = grid.copy()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            g |= np.roll(np.roll(grid, dx, 0), dy, 1)
    ts = np.arange(-0.6, 0.61, 0.04)
    TT = np.array([(a, b) for a in ts for b in ts])
    best = (-1.0, 0.0, np.zeros(2))
    for th in np.radians(np.arange(-180, 180, 1.0)):
        q = (src @ rot(th).T)[None, :, :] + TT[:, None, :]
        ij = np.clip(((q + half) / res).astype(int), 0, N - 1)
        score = g[ij[..., 0], ij[..., 1]].mean(axis=1)
        k = int(score.argmax())
        if score[k] > best[0]:
            best = (float(score[k]), float(th), TT[k])
    return best


def match(src, dst):
    """ค้นหยาบทั้งวงแล้ว ICP ละเอียด -> (หัน rad, t, rms, สัดส่วนจับคู่)"""
    score, th0, t0 = raster_search(src, dst)
    R, t, rms, frac = icp(src, dst, rot(th0), t0)
    return math.atan2(R[1, 0], R[0, 0]), t, rms, frac


def analyze(path, lidar_xy=(0.17, 0.076)):
    d = np.load(path)
    amin, ainc, rmin, rmax = (float(x) for x in d['meta'])
    P = {k: scan_points(d[k], amin, ainc, rmin, rmax) for k in ('s0', 's1', 's2')}
    wheel = d['wheel_yaw']          # yaw ของล้อ (odom) ตอนเก็บแต่ละช่วง
    th_w = float(wheel[1] - wheel[0])
    print(f'จุดต่อช่วง: {[len(P[k]) for k in ("s0", "s1", "s2")]}  ล้อบอกหมุน {math.degrees(th_w):+.1f}°')
    th, t, rms, frac = match(P['s1'], P['s0'])
    print(f'lidar เห็นหมุน {math.degrees(th):+.2f}° (= {th / th_w:.2f} เท่าของล้อ)  t = ({t[0] * 100:+.1f}, {t[1] * 100:+.1f}) cm'
          f'  rms {rms * 100:.1f} cm  จับคู่ได้ {frac * 100:.0f}%')
    M = np.eye(2) - rot(th)
    if abs(np.linalg.det(M)) > 0.1:
        cL = np.linalg.solve(M, t)                  # จุดหมุนในกรอบ lidar ตอนเริ่ม
        l = np.array(lidar_xy)
        cB = l - cL                                 # lidar yaw = pi: p_B = l - p_L
        print(f'==> จุดหมุนจริง (กรอบหุ่น ใช้ lidar ที่ {lidar_xy}): x = {cB[0]:+.3f}  y = {cB[1]:+.3f} m'
              f'  (หมุนรอบกลางหุ่นจริงจะได้ ~0, 0)')
        print(f'    ถ้าเชื่อว่าหมุนรอบกลางหุ่น lidar จะต้องอยู่ห่างกลาง {math.hypot(*cL) * 100:.1f} cm')
    if 'odo' in d:
        for k, name in enumerate(d['odo_names']):
            o = d['odo'][:, k, :]
            if np.isnan(o).any():
                continue
            a1 = math.degrees((o[1, 2] - o[0, 2] + math.pi) % (2 * math.pi) - math.pi)
            a2 = math.degrees((o[2, 2] - o[0, 2] + math.pi) % (2 * math.pi) - math.pi)
            print(f'    {str(name):>20}: นับหมุนไป {a1:+7.1f}° ({a1 / math.degrees(th):4.2f} เท่าของจริง)'
                  f'  หมุนกลับแล้วเหลือ {a2:+6.1f}°')
    th2, t2, rms2, frac2 = match(P['s2'], P['s0'])
    print(f'เช็ค: หมุนกลับแล้วเทียบตอนเริ่ม หัน {math.degrees(th2):+.2f}°'
          f'  t = ({t2[0] * 100:+.1f}, {t2[1] * 100:+.1f}) cm  rms {rms2 * 100:.1f} cm (ควรใกล้ 0)')


def capture(deg, wz):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import SetBool

    rclpy.init()
    n = Node('lidar_offset_test')
    scans, cmds, st = [], [], {}
    n.create_subscription(LaserScan, '/scan_fixed', scans.append, qos_profile_sensor_data)
    n.create_subscription(Twist, '/cmd_vel', cmds.append, 10)
    n.create_subscription(Odometry, '/wheel_odom', lambda m: st.__setitem__('w', m), 10)
    for topic in ODOM_TOPICS:
        n.create_subscription(Odometry, topic, lambda m, t=topic: st.__setitem__(t, m), 10)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def odo_all():
        # (x, y, yaw) ของทุกแหล่ง -- nan ถ้าไม่มี topic นั้น
        out = []
        for t in ODOM_TOPICS:
            if t not in st:
                out.append((np.nan, np.nan, np.nan))
                continue
            p = st[t].pose.pose
            q = p.orientation
            out.append((p.position.x, p.position.y,
                        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        return np.array(out)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    def wheel_yaw():
        q = st['w'].pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def grab(count=12):
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

    def turn(w, sec):
        tw = Twist()
        tw.angular.z = w
        end = time.time() + sec
        while time.time() < end:
            pub.publish(tw)
            spin(0.05)
        for _ in range(20):
            pub.publish(Twist())
            spin(0.05)

    spin(3.0)
    if 'w' not in st:
        print('ไม่มี /wheel_odom -- ไม่หมุน')
        raise SystemExit(1)
    cmds.clear()
    spin(2.0)
    if any(abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3 for m in cmds):
        print('/cmd_vel มีคำสั่งค้าง -- ไม่เปิดไดรเวอร์')
        raise SystemExit(1)

    t_turn = math.radians(deg) / wz
    s0, m0 = grab()
    y = [wheel_yaw()]
    odo = [odo_all()]
    try:
        r = call_enable(True)
        print('เปิดไดรเวอร์:', r.success if r else 'ไม่มี service', flush=True)
        if not r or not r.success:
            raise SystemExit(1)
        spin(1.0)
        print(f'หมุนซ้าย {deg:.0f}°', flush=True)
        turn(wz, t_turn)
        spin(2.0)
        s1, _ = grab()
        y.append(wheel_yaw())
        odo.append(odo_all())
        print(f'หมุนขวากลับ {deg:.0f}°', flush=True)
        turn(-wz, t_turn)
        spin(2.0)
        s2, _ = grab()
        y.append(wheel_yaw())
        odo.append(odo_all())
    finally:
        r = call_enable(False)
        print('ปิดไดรเวอร์:', r.success if r else 'เรียก service ไม่ได้ -- ปิดเองด่วน!', flush=True)
    path = time.strftime('/home/udru/lidar_offset_%H%M%S.npz')
    np.savez(path, s0=s0, s1=s1, s2=s2, wheel_yaw=np.array(y), odo=np.array(odo), odo_names=np.array(ODOM_TOPICS),
             meta=np.array([m0.angle_min, m0.angle_increment, m0.range_min, m0.range_max]))
    print('เก็บ scan ไว้ที่', path, flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return path


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == '--analyze':
        analyze(sys.argv[2])
    else:
        args = [a for a in sys.argv[1:] if not a.startswith('--')]
        p = capture(float(args[0]) if args else 90.0, float(args[1]) if len(args) > 1 else 0.3)
        analyze(p)
