"""nav_goal_test -- ส่งเป้าให้ Nav2 ตรงข้างหน้า DIST m แล้วหยุดรอที่เป้า (ไม่กลับจุดเริ่ม) ไว้วัดระยะจริงด้วยตลับเมตร

    python3 nav_goal_test.py [DIST m=1.0]

ต้องเปิดไดรเวอร์ไว้ก่อน (enable_driver_checked.py) · บันทึกตำแหน่งทุก 0.5 s แล้วสรุประยะ/เฉ/มุม
จาก TF map->base_link (SLAM), /odometry/filtered (EKF), /wheel_odom, /odom_rf2o
"""
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

DIST = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0

rclpy.init()
n = Node('nav_goal_test')
buf = Buffer()
TransformListener(buf, n)
odom = {}
for topic in ('/odometry/filtered', '/wheel_odom', '/odom_rf2o'):
    n.create_subscription(Odometry, topic, lambda m, t=topic: odom.__setitem__(t, m), 10)
ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def poses():
    out = {}
    try:
        t = buf.lookup_transform('map', 'base_link', Time()).transform
        out['SLAM map'] = (t.translation.x, t.translation.y, yaw_of(t.rotation))
    except Exception:
        pass
    for k, m in odom.items():
        p = m.pose.pose
        out[k] = (p.position.x, p.position.y, yaw_of(p.orientation))
    return out


spin(3.0)
p0 = poses()
if 'SLAM map' not in p0:
    print('ไม่มี TF map->base_link -- ยกเลิก')
    raise SystemExit(1)
x0, y0, h0 = p0['SLAM map']
gx, gy = x0 + DIST * math.cos(h0), y0 + DIST * math.sin(h0)
print(f'{time.strftime("%H:%M:%S")} เริ่ม x={x0:.3f} y={y0:.3f} หัน {math.degrees(h0):.1f}°  เป้า x={gx:.3f} y={gy:.3f}')

if not ac.wait_for_server(timeout_sec=10):
    print('ไม่มี action navigate_to_pose -- Nav2 ไม่ได้เปิด?')
    raise SystemExit(1)
g = NavigateToPose.Goal()
g.pose = PoseStamped()
g.pose.header.frame_id = 'map'
g.pose.header.stamp = n.get_clock().now().to_msg()
g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(h0 / 2), math.cos(h0 / 2)
fut = ac.send_goal_async(g)
rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
gh = fut.result()
if not gh or not gh.accepted:
    print('Nav2 ไม่รับเป้า')
    raise SystemExit(1)
res = gh.get_result_async()
t0 = time.time()
last = 0.0
while not res.done() and time.time() - t0 < 120:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last >= 0.5:
        last = time.time()
        p = poses().get('SLAM map')
        if p:
            # ระยะตามแนวหน้าเดิม / เฉข้าง เทียบจุดเริ่ม
            dx, dy = p[0] - x0, p[1] - y0
            along = dx * math.cos(h0) + dy * math.sin(h0)
            side = -dx * math.sin(h0) + dy * math.cos(h0)
            print(f'  {time.time() - t0:5.1f}s  ไป {along:+.3f}  ข้าง {side:+.3f}  หัน {math.degrees(p[2] - h0):+.1f}°', flush=True)
if not res.done():
    print('เกิน 120 s -- ยกเลิกเป้า')
    gh.cancel_goal_async()
    spin(2.0)
    status = 'timeout'
else:
    status = {GoalStatus.STATUS_SUCCEEDED: 'ถึงเป้า', GoalStatus.STATUS_ABORTED: 'ล้ม (aborted)',
              GoalStatus.STATUS_CANCELED: 'ถูกยกเลิก'}.get(res.result().status, str(res.result().status))
print(f'{time.strftime("%H:%M:%S")} ผล Nav2: {status} ใช้เวลา {time.time() - t0:.1f} s')
spin(2.0)
p1 = poses()
print('\n=== ระยะที่แต่ละแหล่งนับได้ (เทียบแนวหน้าตอนเริ่ม) ===')
for k in p1:
    if k not in p0:
        continue
    ax, ay, ah = p0[k]
    bx, by, bh = p1[k]
    dx, dy = bx - ax, by - ay
    along = dx * math.cos(ah) + dy * math.sin(ah)
    side = -dx * math.sin(ah) + dy * math.cos(ah)
    dh = (math.degrees(bh - ah) + 180) % 360 - 180
    print(f'{k:>20}: ไป {along:+.3f} m  เฉ {side:+.3f} m (+ = ซ้าย)  หัน {dh:+.1f}°')
n.destroy_node()
rclpy.shutdown()
