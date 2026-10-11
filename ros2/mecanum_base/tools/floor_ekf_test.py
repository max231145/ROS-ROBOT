"""วิ่งบนพื้นกับระบบจริง: เปิดไดรเวอร์ ล็อกล้อ HOLD s แล้ววิ่ง TARGET m ตาม wheel_odom เทียบ ล้อ / rf2o / EKF / SLAM + lidar วัดระยะก่อน-หลัง
รันผ่าน floor_ekf_run.sh (ปิดไดรเวอร์ซ้ำให้แม้สคริปต์นี้พัง) ต้องมี ros2-slam.service รันอยู่ ยกล้อหรือเตรียมพื้นที่ก่อน"""
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

TARGET, VX, HOLD, SETTLE = 1.0, 0.05, 40.0, 3.0
STOP_EARLY = 0.0025


def clock():
    return time.strftime('%H:%M:%S')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FloorEkfTest(Node):
    def __init__(self):
        super().__init__('floor_ekf_test')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.last = {}
        for name, topic in (('wheel', '/wheel_odom'), ('rf2o', '/odom_rf2o'), ('ekf', '/odometry/filtered')):
            self.create_subscription(Odometry, topic, self.saver(name), 10)
        self.enable_cli = self.create_client(SetBool, '/mecanum_base/enable')
        self.scans = None                       # list collecting /scan_fixed while recording
        self.create_subscription(LaserScan, '/scan_fixed', self.on_scan, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_scan(self, msg):
        if self.scans is not None:
            self.scans.append(msg)

    def saver(self, name):
        def cb(msg):
            self.last[name] = msg
        return cb


rclpy.init()
n = FloorEkfTest()
signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))


def spin_for(sec, twist=None):
    end = time.monotonic() + sec
    last = 0.0
    while time.monotonic() < end:
        if twist is not None and time.monotonic() - last >= 0.05:
            n.pub.publish(twist)
            last = time.monotonic()
        rclpy.spin_once(n, timeout_sec=0.01)


def set_enable(flag):
    if not n.enable_cli.wait_for_service(timeout_sec=5.0):
        return False
    fut = n.enable_cli.call_async(SetBool.Request(data=flag))
    end = time.monotonic() + 5.0
    while not fut.done() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    return fut.done() and fut.result().success


def record_scan(sec):
    """per-beam median range while parked (lidar ruler)"""
    n.scans = []
    spin_for(sec, Twist())
    got, n.scans = n.scans, None
    if not got:
        return None, None
    N = len(got[-1].ranges)
    got = [g for g in got if len(g.ranges) == N]
    med = []
    for i in range(N):
        v = sorted(g.ranges[i] for g in got if math.isfinite(g.ranges[i]) and g.ranges[i] > 0)
        med.append(v[len(v) // 2] if v else float('nan'))
    return got[-1], med


def ruler_report(meta, a, b):
    if meta is None or a is None or b is None or len(a) != len(b):
        print('lidar ruler: no scans', flush=True)
        return
    print('lidar ruler (range change per sector; target straight ahead should show about -distance):', flush=True)
    for center in (0, 90, 180, -90):
        d = []
        for i in range(len(a)):
            ang = math.degrees(meta.angle_min + i * meta.angle_increment)
            diff = (ang - center + 180.0) % 360.0 - 180.0
            if abs(diff) <= 5.0 and math.isfinite(a[i]) and math.isfinite(b[i]):
                d.append((b[i] - a[i], a[i]))
        if d:
            ds = sorted(x for x, _ in d)
            r0 = sorted(y for _, y in d)
            print(f'  laser {center:+4d} deg +-5: {len(d):2d} beams  start range {r0[len(r0)//2]:.3f} m  change median {ds[len(ds)//2]:+.3f} m', flush=True)
        else:
            print(f'  laser {center:+4d} deg +-5: no valid beams', flush=True)


def poses():
    out = {}
    for k in ('wheel', 'rf2o', 'ekf'):
        p = n.last[k].pose.pose
        out[k] = (p.position.x, p.position.y, yaw_of(p.orientation))
    try:
        t = n.tf_buffer.lookup_transform('map', 'base_link', Time())
        out['slam'] = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
    except Exception as e:
        out['slam'] = None
        print(f'{clock()} map->base_link lookup failed: {e}', flush=True)
    return out


try:
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline and (len(n.last) < 3 or n.pub.get_subscription_count() == 0):
        rclpy.spin_once(n, timeout_sec=0.1)
    if len(n.last) < 3 or n.pub.get_subscription_count() == 0:
        print(f'{clock()} FAIL: inputs not ready: have {sorted(n.last)} cmd_vel subs {n.pub.get_subscription_count()}', flush=True)
        sys.exit(1)
    if not set_enable(True):
        print(f'{clock()} FAIL: enable service call failed', flush=True)
        sys.exit(1)
    move_at = time.strftime('%H:%M:%S', time.localtime(time.time() + HOLD))
    print(f'{clock()} driver ON, holding {HOLD:.0f} s -> move starts at {move_at}', flush=True)
    spin_for(max(0.0, HOLD - 2.0), Twist())
    scan_meta, scan0 = record_scan(2.0)

    p0 = poses()
    go = Twist()
    go.linear.x = VX
    print(f'{clock()} MOVE start', flush=True)
    t0 = time.monotonic()
    while True:
        spin_for(0.02, go)
        if n.last['wheel'].pose.pose.position.x - p0['wheel'][0] >= TARGET - STOP_EARLY:
            break
        if time.monotonic() - t0 > TARGET / VX * 2.0:
            print(f'{clock()} move time limit hit', flush=True)
            break
    spin_for(SETTLE, Twist())
    p1 = poses()
    _, scan1 = record_scan(2.0)
    print(f'{clock()} STOPPED after {time.monotonic() - t0 - SETTLE:.1f} s of motion', flush=True)
    print(f'{"source":6s} {"forward[m]":>10s} {"lateral[m]":>10s} {"yaw[deg]":>8s} {"distance":>8s}', flush=True)
    for k in ('wheel', 'rf2o', 'ekf', 'slam'):
        if p0.get(k) is None or p1.get(k) is None:
            print(f'{k:6s} n/a', flush=True)
            continue
        x0, y0, h0 = p0[k]
        x1, y1, h1 = p1[k]
        dx, dy = x1 - x0, y1 - y0
        fwd = dx * math.cos(h0) + dy * math.sin(h0)          # along the start heading of that source
        lat = -dx * math.sin(h0) + dy * math.cos(h0)          # + = left
        dyaw = math.degrees(math.atan2(math.sin(h1 - h0), math.cos(h1 - h0)))
        print(f'{k:6s} {fwd:10.4f} {lat:10.4f} {dyaw:8.2f} {math.hypot(dx, dy):8.4f}', flush=True)
    ruler_report(scan_meta, scan0, scan1)
except KeyboardInterrupt:
    print(f'{clock()} interrupted', flush=True)
finally:
    try:
        n.pub.publish(Twist())
        print(f'{clock()} disable driver: {set_enable(False)}', flush=True)
    except Exception as e:
        print(f'{clock()} disable from python failed ({e!r}); wrapper will retry', flush=True)
