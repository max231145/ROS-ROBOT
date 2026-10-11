#!/usr/bin/env python3
"""ตรวจว่าจุด lidar จุดไหนอยู่ในกรอบหยุดตอนหมุน (rotation polygon ของ collision_monitor)
อ่านอย่างเดียว ไม่สั่งหุ่น -- รัน ~5 วินาทีแล้วสรุป

    python3 ~/scan_check.py            # ใช้ /scan_nav (ตัวที่ collision monitor ใช้)
    python3 ~/scan_check.py /scan      # เทียบกับ scan ดิบ
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

# ตรงกับ nav2_params.yaml
FOOT = (0.27, -0.24, 0.244)        # หน้า, หลัง, ครึ่งกว้าง
ROT = (0.34, -0.31, 0.314)         # กรอบ rotation / stopped
MIN_POINTS = 4
SECONDS = 5.0


def side(x, y):
    """ทิศของจุดเทียบกับตัวหุ่น และระยะจากขอบตัวหุ่น (cm)"""
    dx = x - FOOT[0] if x > FOOT[0] else (FOOT[1] - x if x < FOOT[1] else 0.0)
    dy = abs(y) - FOOT[2] if abs(y) > FOOT[2] else 0.0
    if dx == 0 and dy == 0:
        return 'บนตัวหุ่น', 0.0
    name = []
    if x > FOOT[0]:
        name.append('หน้า')
    elif x < FOOT[1]:
        name.append('หลัง')
    if abs(y) > FOOT[2]:
        name.append('ซ้าย' if y > 0 else 'ขวา')
    return '-'.join(name), math.hypot(dx, dy) * 100


class Check(Node):
    def __init__(self, topic):
        super().__init__('scan_check')
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.topic = topic
        self.scans = 0
        self.hits = {}        # (มุม lidar ปัดเป็นองศา) -> [ครั้งที่เจอ, x, y, ระยะ lidar]
        self.over = 0         # จำนวนรอบที่จุดในกรอบ >= MIN_POINTS (= collision monitor สั่งหยุด)
        self.create_subscription(LaserScan, topic, self.cb, qos_profile_sensor_data)

    def cb(self, s):
        try:
            t = self.tf.lookup_transform('base_link', s.header.frame_id, Time()).transform
        except TransformException as e:
            self.get_logger().warn(f'ยังไม่ได้ TF base_link<-{s.header.frame_id}: {e}', throttle_duration_sec=2)
            return
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty = t.translation.x, t.translation.y
        self.scans += 1
        n = 0
        seen = set()
        for k, r in enumerate(s.ranges):
            if not (s.range_min <= r <= s.range_max) or math.isinf(r) or math.isnan(r):
                continue
            a = s.angle_min + k * s.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            x = tx + lx * math.cos(yaw) - ly * math.sin(yaw)
            y = ty + lx * math.sin(yaw) + ly * math.cos(yaw)
            if ROT[1] <= x <= ROT[0] and abs(y) <= ROT[2]:
                n += 1
                key = round(math.degrees(a))
                h = self.hits.setdefault(key, [0, x, y, r])
                if key not in seen:          # นับครั้งเดียวต่อ scan ต่อองศา
                    seen.add(key)
                    h[0] += 1
                h[1], h[2], h[3] = x, y, r
        if n >= MIN_POINTS:
            self.over += 1


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/scan_nav'
    rclpy.init()
    node = Check(topic)
    print(f'อ่าน {topic} {SECONDS:.0f} วินาที... (หุ่นอยู่นิ่งๆ)')
    end = time.monotonic() + SECONDS
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.scans:
        print('ไม่ได้รับ scan เลย -- เช็กว่า SLAM/lidar ทำงาน และตั้ง ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST แล้ว')
        return
    print(f'ได้ {node.scans} scan · scan ที่มีจุดในกรอบหมุน >= {MIN_POINTS} จุด (= สั่งหยุด): '
          f'{node.over}/{node.scans} ({100 * node.over / node.scans:.0f}%)')
    if not node.hits:
        print('ไม่มีจุดในกรอบหมุนเลย -- collision monitor ไม่น่าจะเป็นตัวสั่งหยุดตอนนี้')
        return
    print('\nจุดที่อยู่ในกรอบ (เรียงตามจำนวนครั้งที่เจอ):')
    print(' มุม lidar | เจอกี่ % ของ scan | ทิศ | ห่างขอบตัวหุ่น | x,y จากกลางหุ่น | ระยะจาก lidar')
    for key, (cnt, x, y, r) in sorted(node.hits.items(), key=lambda kv: -kv[1][0])[:25]:
        name, d = side(x, y)
        print(f' {key:5d}°   | {100 * cnt / node.scans:4.0f}%  | {name:10s} | {d:5.1f} cm | '
              f'({x:+.2f}, {y:+.2f}) | {r:.2f} m')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
