#!/usr/bin/env python3
"""เฝ้าดูการทำงานของหุ่นระหว่างทดสอบภารกิจ A -> B -> A แล้วสรุปจุดที่แปลก
อ่านอย่างเดียว ไม่สั่งหุ่น -- เปิดทิ้งไว้ระหว่างทดสอบ กด Ctrl+C เมื่อเสร็จ แล้วจะได้รายงาน

    python3 ~/mission_monitor.py                 # จุดชื่อ A กับ B
    python3 ~/mission_monitor.py จุด1 จุด2       # ใช้ชื่อจุดอื่น

ไฟล์ผลลัพธ์: ~/mission_reports/<เวลา>.txt (รายงาน) และ .csv (ข้อมูลทุก 0.1 s)
"""
import csv
import json
import math
import os
import sys
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:          # Nav2 รุ่นที่ไม่มี msg นี้
    CollisionMonitorState = None

MAPS = os.path.expanduser('~/maps')
OUT = os.path.expanduser('~/mission_reports')
MISSION_LOG = '/tmp/admin_web_mission.log'
FOOT = (0.27, -0.24, 0.244)      # ขอบหน้า, ขอบหลัง, ครึ่งกว้าง (ตรง nav2_params)
CM_ACTION = {0: 'ไม่ทำอะไร', 1: 'หยุด', 2: 'ชะลอ', 3: 'เข้าใกล้', 4: 'จำกัดความเร็ว'}
WATCH_NODES = ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server',
               'collision_monitor', 'velocity_smoother', 'slam_toolbox', 'admin_web')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def foot_clearance(x, y):
    """ระยะจากจุด (กรอบหุ่น) ถึงขอบตัวหุ่น (ติดลบ = อยู่บนตัวหุ่น)"""
    dx = max(FOOT[1] - x, 0.0, x - FOOT[0])
    dy = max(abs(y) - FOOT[2], 0.0)
    if dx == 0 and dy == 0:
        return -min(FOOT[0] - x, x - FOOT[1], FOOT[2] - abs(y))
    return math.hypot(dx, dy)


def load_places():
    try:
        name = open(os.path.join(MAPS, 'active_map.txt'), encoding='utf-8').read().strip()
    except OSError:
        name = ''
    for f in (os.path.join(MAPS, f'places_{name}.json'), os.path.join(MAPS, 'places.json')):
        try:
            return {p['name']: p for p in json.load(open(f, encoding='utf-8')).get('places', [])}, f
        except (OSError, ValueError):
            continue
    return {}, None


class Monitor(Node):
    def __init__(self):
        super().__init__('mission_monitor')
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.cmd = {'nav': (0.0, 0.0, 0.0), 'smooth': (0.0, 0.0, 0.0), 'out': (0.0, 0.0, 0.0)}   # vx, wz, เวลา
        self.cm = ('-', '')
        self.clear = (None, '')          # ระยะใกล้สุดจากขอบหุ่นถึงจุด lidar, ทิศ
        self.plan_n, self.plan_t = 0, 0.0
        self.events = []                 # (t, ข้อความ)
        self.warn_count = {}
        self.rows = []
        self.pose = None
        self.t0 = time.monotonic()
        for topic, key in (('/cmd_vel_nav', 'nav'), ('/cmd_vel_smoothed', 'smooth'), ('/cmd_vel', 'out')):
            self.create_subscription(Twist, topic, lambda m, k=key: self.on_cmd(k, m), 10)
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState, '/collision_monitor_state', self.on_cm, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(LaserScan, '/scan_nav', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.log_pos = os.path.getsize(MISSION_LOG) if os.path.exists(MISSION_LOG) else 0
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.read_mission_log)

    def now(self):
        return time.monotonic() - self.t0

    def event(self, text, show=True):
        self.events.append((self.now(), text))
        if show:
            print(f'[{self.now():6.1f}s] {text}', flush=True)

    def on_cmd(self, key, m):
        self.cmd[key] = (m.linear.x, m.angular.z, self.now())

    def on_cm(self, m):
        st = (CM_ACTION.get(m.action_type, str(m.action_type)), m.polygon_name)
        if st != self.cm:
            if m.action_type in (1, 2, 3, 4) or self.cm[0] in ('หยุด', 'ชะลอ'):
                self.event(f'collision monitor: {st[0]} {st[1]}')
            self.cm = st

    def on_plan(self, m):
        self.plan_n, self.plan_t = len(m.poses), self.now()

    def on_scan(self, s):
        try:
            t = self.tf.lookup_transform('base_link', s.header.frame_id, Time()).transform
        except TransformException:
            return
        yaw, tx, ty = yaw_of(t.rotation), t.translation.x, t.translation.y
        c, sn = math.cos(yaw), math.sin(yaw)
        best = None
        for k, r in enumerate(s.ranges):
            if not (s.range_min <= r <= min(s.range_max, 2.0)):
                continue
            a = s.angle_min + k * s.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            x, y = tx + lx * c - ly * sn, ty + lx * sn + ly * c
            d = foot_clearance(x, y)
            if best is None or d < best[0]:
                best = (d, x, y)
        if best:
            d, x, y = best
            side = ('หน้า' if x > FOOT[0] else 'หลัง' if x < FOOT[1] else '') + \
                   ('ซ้าย' if y > FOOT[2] else 'ขวา' if y < -FOOT[2] else '')
            self.clear = (d, side or 'บนตัว')

    def on_log(self, m):
        if m.level < 30 or not any(n in m.name for n in WATCH_NODES):
            return
        key = m.name.split('.')[-1] + ': ' + m.msg[:90]
        n = self.warn_count.get(key, 0) + 1
        self.warn_count[key] = n
        if n == 1 or n % 20 == 0:      # ข้อความซ้ำๆ พิมพ์แค่ครั้งแรกและทุก 20 ครั้ง
            self.event(f'{"ERROR" if m.level >= 40 else "WARN"} {key}' + (f' (ครั้งที่ {n})' if n > 1 else ''))

    def read_mission_log(self):
        try:
            with open(MISSION_LOG, encoding='utf-8') as fp:
                fp.seek(self.log_pos)
                for line in fp.read().splitlines():
                    if line.strip():
                        self.event('ภารกิจ> ' + line.strip())
                self.log_pos = fp.tell()
        except OSError:
            pass

    def tick(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform
        except TransformException:
            return
        x, y, yaw = t.translation.x, t.translation.y, yaw_of(t.rotation)
        now = self.now()
        if self.pose and math.hypot(x - self.pose[0], y - self.pose[1]) > 0.15:
            self.event(f'ตำแหน่งกระโดด {math.hypot(x - self.pose[0], y - self.pose[1]) * 100:.0f} cm ใน 0.1 s '
                       f'(SLAM แก้ตำแหน่ง?)')
        self.pose = (x, y, yaw)
        fresh = lambda k: now - self.cmd[k][2] < 0.5
        row = {'t': round(now, 2), 'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(math.degrees(yaw), 1)}
        for k in ('nav', 'smooth', 'out'):
            row[f'{k}_vx'] = round(self.cmd[k][0], 3) if fresh(k) else 0.0
            row[f'{k}_wz'] = round(self.cmd[k][1], 3) if fresh(k) else 0.0
        row['cm'] = self.cm[0] + (' ' + self.cm[1] if self.cm[1] else '')
        row['clear_cm'] = round(self.clear[0] * 100, 1) if self.clear[0] is not None else ''
        row['clear_side'] = self.clear[1]
        row['plan_pts'] = self.plan_n if now - self.plan_t < 3 else 0
        self.rows.append(row)


def analyse(mon, places, a_name, b_name):
    rows, out = mon.rows, []
    if len(rows) < 10:
        return ['ข้อมูลน้อยเกินไป (ไม่ได้ TF map->base_link?) -- เช็กว่า SLAM ทำงาน']
    dur = rows[-1]['t'] - rows[0]['t']
    dist = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(rows, rows[1:]))
    turn = sum(abs(wrap(math.radians(b['yaw_deg'] - a['yaw_deg']))) for a, b in zip(rows, rows[1:]))
    out.append(f'เวลาทั้งหมด {dur:.0f} s · วิ่งไป {dist:.2f} m · หมุนรวม {math.degrees(turn):.0f}°')

    A, B = places.get(a_name), places.get(b_name)
    if A and B:
        ax, ay, bx, by = A['x'], A['y'], B['x'], B['y']
        L = math.hypot(bx - ax, by - ay)
        out.append(f'จุด {a_name} ({ax:.2f},{ay:.2f}) -> {b_name} ({bx:.2f},{by:.2f}) ห่างกันตรงๆ {L:.2f} m')
        if L > 0.05:
            ux, uy = (bx - ax) / L, (by - ay) / L
            xt = [abs((r['x'] - ax) * uy - (r['y'] - ay) * ux) for r in rows]
            out.append(f'ออกนอกเส้นตรง A-B: เฉลี่ย {sum(xt) / len(xt) * 100:.0f} cm · มากสุด {max(xt) * 100:.0f} cm'
                       + ('  <-- แปลก: ออกนอกเส้นเกิน 30 cm' if max(xt) > 0.30 else ''))
            if dist > 0:
                out.append(f'ระยะที่วิ่ง / ระยะตรง (ไป-กลับ = {2 * L:.2f} m): {dist / (2 * L):.2f} เท่า'
                           + ('  <-- แปลก: อ้อมหรือวนมาก' if dist > 3 * L else ''))
        dA = min(math.hypot(r['x'] - ax, r['y'] - ay) for r in rows)
        dB = min(math.hypot(r['x'] - bx, r['y'] - by) for r in rows)
        out.append(f'เข้าใกล้ {a_name} สุด {dA * 100:.0f} cm · เข้าใกล้ {b_name} สุด {dB * 100:.0f} cm'
                   + ('  <-- ไปไม่ถึง (เกิน 25 cm)' if max(dA, dB) > 0.25 else ''))
    else:
        out.append(f'ไม่เจอจุด "{a_name}" หรือ "{b_name}" ในไฟล์จุด -- ข้ามการวัดเส้นตรง (มีจุด: {", ".join(places) or "-"})')

    # ช่วงที่สั่งขยับแต่หุ่นไม่ขยับ / ตัวควบคุมสั่งแต่ collision monitor ตัด
    def spans(cond, min_s):
        res, start = [], None
        for r in rows:
            if cond(r):
                start = r['t'] if start is None else start
            else:
                if start is not None and r['t'] - start >= min_s:
                    res.append((start, r['t']))
                start = None
        if start is not None and rows[-1]['t'] - start >= min_s:
            res.append((start, rows[-1]['t']))
        return res

    moving_cmd = lambda r: abs(r['out_vx']) > 0.01 or abs(r['out_wz']) > 0.05
    win = {}
    for i, r in enumerate(rows):          # หุ่นขยับจริงไหมใน 1 s ถัดไป
        j = min(len(rows) - 1, i + 10)
        win[r['t']] = (math.hypot(rows[j]['x'] - r['x'], rows[j]['y'] - r['y']),
                       abs(wrap(math.radians(rows[j]['yaw_deg'] - r['yaw_deg']))))
    stuck = spans(lambda r: moving_cmd(r) and win[r['t']][0] < 0.01 and win[r['t']][1] < 0.03, 2.0)
    for s, e in stuck:
        out.append(f'  <-- แปลก: {s:.0f}-{e:.0f} s สั่งล้อหมุนแต่หุ่นไม่ขยับ (ล้อฟรี/ไดรเวอร์ปิด/ติดคาน?)')
    cut = spans(lambda r: (abs(r['nav_vx']) > 0.01 or abs(r['nav_wz']) > 0.05) and not moving_cmd(r), 1.0)
    for s, e in cut:
        cms = {r['cm'] for r in rows if s <= r['t'] <= e}
        out.append(f'  <-- {s:.0f}-{e:.0f} s ตัวควบคุมสั่งขยับ แต่คำสั่งไม่ถึงล้อ (collision monitor: {", ".join(cms)})')
    spin = spans(lambda r: abs(r['nav_vx']) < 0.01 and abs(r['nav_wz']) > 0.05, 4.0)
    for s, e in spin:
        out.append(f'  <-- {s:.0f}-{e:.0f} s Nav2 สั่งหมุนอยู่กับที่นาน {e - s:.0f} s (หมุนหาทาง/ท่าช่วยเหลือ)')
    idle = spans(lambda r: not moving_cmd(r) and r['nav_vx'] == 0 and r['nav_wz'] == 0, 5.0)
    for s, e in idle:
        out.append(f'  ช่วงนิ่ง {s:.0f}-{e:.0f} s (ไม่มีใครสั่ง -- รอคำสั่ง/วางทางไม่ได้?)')
    rev = sum(1 for a, b in zip(rows, rows[1:]) if a['out_vx'] * b['out_vx'] < 0)
    if rev:
        out.append(f'สลับเดินหน้า/ถอย {rev} ครั้ง' + ('  <-- แปลก: ส่ายไปมา' if rev > 10 else ''))

    cl = [r['clear_cm'] for r in rows if r['clear_cm'] != '']
    if cl:
        k = min(range(len(cl)), key=lambda i: cl[i])
        side = [r['clear_side'] for r in rows if r['clear_cm'] != ''][k]
        out.append(f'ของใกล้ตัวหุ่นที่สุด {cl[k]:.0f} cm ทาง{side}' + ('  <-- เฉียดมาก' if cl[k] < 5 else ''))
    replans = len({r['t'] for r in rows if r['plan_pts']})
    out.append(f'มีเส้นทางจาก planner อยู่ {replans / 10:.0f} s จาก {dur:.0f} s')

    if mon.warn_count:
        out.append('\nคำเตือนจาก Nav2 (จำนวนครั้ง):')
        for k, n in sorted(mon.warn_count.items(), key=lambda kv: -kv[1])[:12]:
            out.append(f'  {n:4d} × {k}')
    return out


def main():
    a_name = sys.argv[1] if len(sys.argv) > 1 else 'A'
    b_name = sys.argv[2] if len(sys.argv) > 2 else 'B'
    rclpy.init()
    mon = Monitor()
    places, pf = load_places()
    print(f'เฝ้าดูอยู่... (จุด {a_name}/{b_name} จาก {pf}) ทดสอบได้เลย เสร็จแล้วกด Ctrl+C', flush=True)
    try:
        rclpy.spin(mon)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    places, _ = load_places()            # เผื่อบันทึกจุดระหว่างเฝ้า
    os.makedirs(OUT, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    rep = analyse(mon, places, a_name, b_name)
    with open(os.path.join(OUT, stamp + '.csv'), 'w', newline='', encoding='utf-8') as fp:
        if mon.rows:
            w = csv.DictWriter(fp, fieldnames=list(mon.rows[0]))
            w.writeheader()
            w.writerows(mon.rows)
    text = '\n'.join(['==== สรุป ===='] + rep + ['\n==== เหตุการณ์ ===='] +
                     [f'[{t:6.1f}s] {e}' for t, e in mon.events])
    with open(os.path.join(OUT, stamp + '.txt'), 'w', encoding='utf-8') as fp:
        fp.write(text + '\n')
    print('\n' + '\n'.join(['==== สรุป ===='] + rep))
    print(f'\nบันทึกรายงานไว้ที่ {OUT}/{stamp}.txt และ .csv')
    try:
        mon.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
