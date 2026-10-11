"""nav_rotate_test -- ให้ Nav2 หมุนแบบปิดวง: หัน +DEG ที่จุดเดิม แล้วกลับทิศ/ตำแหน่งเดิม วัดผลจริงด้วย scan

    python3 nav_rotate_test.py [DEG=90]

ต่างจาก lidar_offset_test.py ที่สั่งล้อตามเวลาแบบไม่ดูผล -- ตัวนี้ Nav2 ดูทิศจาก EKF (ตอนหมุนอยู่กับที่เชื่อ rf2o
ตั้งแต่ 23 ก.ย.) แล้วหมุนจนถึงเป้าเอง ถ้าทิศใน EKF ถูก หุ่นต้องกลับมาทิศเดิมแม้ล้อไถล
เปิดไดรเวอร์เอง (เช็ค /cmd_vel ไม่มีคำสั่งค้าง) ปิดทุกกรณีตอนจบ บันทึก scan ไว้ที่ ~/nav_rotate_HHMMSS.npz
"""
import math
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, '/home/udru/ros-ws/src/mecanum_base/tools')
import lidar_offset_test as L   # noqa: E402  scan_points, match, rot
import line_test as LT          # noqa: E402  laser_to_base

DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0

rclpy.init()
n = Node('nav_rotate_test')
buf = Buffer()
TransformListener(buf, n)
scans, cmds, st = [], [], {}
n.create_subscription(LaserScan, '/scan_fixed', scans.append, qos_profile_sensor_data)
n.create_subscription(Twist, '/cmd_vel', cmds.append, 10)
for topic in ('/wheel_odom', '/odom_rf2o', '/odometry/filtered'):
    n.create_subscription(Odometry, topic, lambda m, t=topic: st.__setitem__(t, m), 10)
ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def map_pose():
    t = buf.lookup_transform('map', 'base_link', Time()).transform
    return t.translation.x, t.translation.y, yaw_of(t.rotation)


def grab(count=10):
    scans.clear()
    while len(scans) < count:
        spin(0.1)
    m = scans[0]
    return np.array([s.ranges[:len(m.ranges)] for s in scans], dtype=float), m


def odo():
    return {k: math.degrees(yaw_of(m.pose.pose.orientation)) for k, m in st.items()}


def call_enable(on):
    c = n.create_client(SetBool, '/mecanum_base/enable')
    if not c.wait_for_service(timeout_sec=5):
        return None
    f = c.call_async(SetBool.Request(data=on))
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    return f.result()


def goto(x, y, yaw, timeout=30.0):
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = 'map'
    g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x, g.pose.pose.position.y = x, y
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    fut = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    gh = fut.result()
    if not gh or not gh.accepted:
        return 'Nav2 ไม่รับเป้า', 0.0
    res = gh.get_result_async()
    t0 = time.time()
    while not res.done() and time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
    if not res.done():
        gh.cancel_goal_async()
        spin(2.0)
        return 'เกินเวลา -- ยกเลิก', time.time() - t0
    s = res.result().status
    return {GoalStatus.STATUS_SUCCEEDED: 'ถึงเป้า', GoalStatus.STATUS_ABORTED: 'ล้ม (aborted)',
            GoalStatus.STATUS_CANCELED: 'ถูกยกเลิก'}.get(s, str(s)), time.time() - t0


# รอ TF/odom/scan ให้ครบ
end = time.time() + 15
while True:
    spin(0.5)
    try:
        x0, y0, h0 = map_pose()
        if len(st) == 3:
            break
    except Exception:
        pass
    if time.time() > end:
        print('ไม่มี TF map->base_link หรือ odom -- ยกเลิก')
        raise SystemExit(1)
if not ac.wait_for_server(timeout_sec=10):
    print('ไม่มี action navigate_to_pose -- Nav2 ไม่ได้เปิด?')
    raise SystemExit(1)
cmds.clear()
spin(2.0)
if any(abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3 for m in cmds):
    print('/cmd_vel มีคำสั่งค้าง -- ไม่เปิดไดรเวอร์')
    raise SystemExit(1)

s0, m0 = grab()
meta = (m0.angle_min, m0.angle_increment, m0.range_min, m0.range_max)
o0 = odo()
print(f'เริ่ม: map x={x0:+.3f} y={y0:+.3f} หัน {math.degrees(h0):+.1f}°', flush=True)
out = {}
try:
    r = call_enable(True)
    print('เปิดไดรเวอร์:', r.success if r else 'ไม่มี service', flush=True)
    if not r or not r.success:
        raise SystemExit(1)
    spin(1.0)
    stat, dt = goto(x0, y0, h0 + math.radians(DEG))
    spin(2.0)
    s1, _ = grab()
    o1, p1 = odo(), map_pose()
    print(f'ขาไป (หัน {DEG:+.0f}°): {stat} {dt:.1f} s   map ตอนนี้ x={p1[0]:+.3f} y={p1[1]:+.3f} หัน {math.degrees(p1[2]):+.1f}°', flush=True)
    stat, dt = goto(x0, y0, h0)
    spin(2.0)
    s2, _ = grab()
    o2, p2 = odo(), map_pose()
    print(f'ขากลับ: {stat} {dt:.1f} s   map ตอนนี้ x={p2[0]:+.3f} y={p2[1]:+.3f} หัน {math.degrees(p2[2]):+.1f}°', flush=True)
    out = dict(s0=s0, s1=s1, s2=s2, meta=np.array(meta))
finally:
    r = call_enable(False)
    print('ปิดไดรเวอร์:', r.success if r else 'เรียก service ไม่ได้ -- ปิดเองด่วน!', flush=True)

path = time.strftime('/home/udru/nav_rotate_%H%M%S.npz')
np.savez(path, **out)
print('เก็บ scan ไว้ที่', path, flush=True)
P0, P1, P2 = (L.scan_points(out[k], *meta) for k in ('s0', 's1', 's2'))
for label, P, o in (('หลังหันไป', P1, o1), ('หลังกลับ', P2, o2)):
    th, t, rms, frac = L.match(P, P0)
    d, th = LT.laser_to_base(th, t)
    ek = (o['/odometry/filtered'] - o0['/odometry/filtered'] + 180) % 360 - 180
    wh = (o['/wheel_odom'] - o0['/wheel_odom'] + 180) % 360 - 180
    print(f'{label}: จริง (scan) หัน {math.degrees(th):+6.1f}°  เลื่อน {np.hypot(*d) * 100:5.1f} cm'
          f' (หน้า {d[0] * 100:+.1f} ข้าง {d[1] * 100:+.1f})  [rms {rms * 100:.1f} cm {frac * 100:.0f}%]'
          f'  | EKF {ek:+6.1f}°  ล้อ {wh:+6.1f}°', flush=True)
n.destroy_node()
rclpy.shutdown()
