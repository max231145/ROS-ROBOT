#!/usr/bin/env python3
"""
object_locator -- ปักหมุดของที่ YOLO เห็น (chair, person, ...) ลงแมพ: กล้องบอก "อะไร/ทิศไหน" lidar บอก "ไกลเท่าไร"

  subscribe  /detections/summary   std_msgs/String (JSON จาก ~/yolo_node.py: class conf box=[x1,y1,x2,y2] พิกเซล)
             /scan_nav             sensor_msgs/LaserScan (ตัดจุดบนตัวหุ่นแล้ว)
  publish    /object_markers       visualization_msgs/MarkerArray  หมุด + ชื่อ (ภาษาอังกฤษ -- Foxglove 3D ไม่แสดงวรรณยุกต์ไทย)
             /object_cloud         sensor_msgs/PointCloud2 (base_link) วงกลมรอบของแต่ละชิ้น -> camera_layer ใน costmap
                                   ให้ Nav2 หลบของที่ lidar มองไม่เห็น (เก้าอี้ที่ lidar ลอดใต้เบาะ) -- ไม่รวม person
                                   (lidar เห็นขาคนอยู่แล้ว)
             /object_clear_cloud   sensor_msgs/PointCloud2 (base_link) ปลายลำแสงทั่วมุมกล้องที่ clear_range
                                   -> camera_layer ลบเครื่องหมายเก่าในส่วนที่กล้องมองเห็นอยู่ (ของย้าย/ค่าประมาณเก่า)
                                   ส่วนที่อยู่นอกมุมกล้องยังจำไว้
             /objects              std_msgs/String  JSON รายการของที่เจอ {class, x, y, source, hits}
  service    ~/clear               std_srvs/Trigger  ลบหมุดทั้งหมด (เรียกจาก reset_pose ตอนล้างแมพ)

หาตำแหน่ง
  1. ขอบซ้าย/ขวาของกรอบ -> มุมจากกล้อง (เลนส์แบบเส้นตรง fx จาก hfov_deg)
  2. จุด lidar ที่อยู่ในช่วงมุมนั้น (+- angle_margin_deg) -> กลุ่มที่ใกล้สุด = ของชิ้นนั้น -> ตำแหน่งจาก lidar (แม่น)
  3. ไม่มีจุด lidar (ของสูง/ต่ำกว่าระนาบ lidar): ใช้ขอบล่างกรอบที่แตะพื้น + ความสูงกล้อง (กล้องมองตรง) -> ระยะโดยประมาณ
     ป้ายจะมี "~" นำหน้า
  4. แปลงเป็นพิกัด map ด้วย TF แล้วรวมกับหมุดชนิดเดียวกันที่ห่างไม่เกิน merge_dist

!! ภาพ 640x480 เห็นแคบกว่าสเปก 102° มาก: ฉายจุด lidar ลงภาพ (23 ก.ย.) วงกบประตู/เสาตรงขอบจริงที่ hfov ~65°
   (85° เยื้องเข้ากลางภาพชัด) -- น่าจะเป็นโหมดเซนเซอร์ที่ครอปกลาง · ปรับต่อด้วย log "เทียบมุมกล้องกับ lidar" !!
!! ระยะจากขอบล่างกรอบไวต่อเส้นขอบฟ้ามาก (ขอบล่างขยับ 20 px ระยะเปลี่ยน ~40%) -> camera_pitch_deg ต้องเทียบกับ
   ระยะจริงที่วัดด้วยตลับเมตร และใช้ค่ากลางหลายครั้งแทนค่าล่าสุด !!
"""

import json
import math
import os
import signal
import zlib

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def class_color(name):
    h = (zlib.crc32(name.encode()) % 360) / 360.0
    i, f = int(h * 6), h * 6 - int(h * 6)
    q, t = 1 - f, f
    # ต้องเป็น float -- ColorRGBA รับ int แล้ว rclpy assert ล้มทั้ง process (เจอ 23 ก.ย.)
    return tuple(float(v) for v in [(1, t, 0), (q, 1, 0), (0, 1, t), (0, q, 1), (t, 0, 1), (1, 0, q)][i % 6])


class ObjectLocator(Node):

    def __init__(self):
        super().__init__('object_locator')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.map_frame = p('map_frame', 'map')
        self.base_frame = p('base_frame', 'base_link')
        self.W = int(p('image_width', 640))
        self.H = int(p('image_height', 480))
        self.hfov = math.radians(float(p('hfov_deg', 65.0)))
        self.cam_x = float(p('camera_x', 0.22))       # m จากกลางหุ่น (ติดหน้าหุ่นตรงกลาง)
        self.cam_y = float(p('camera_y', 0.0))
        self.cam_z = float(p('camera_height', 0.23))  # ผู้ใช้วัด 23 ก.ย. มองตรง
        self.min_conf = float(p('min_conf', 0.4))
        self.margin = math.radians(float(p('angle_margin_deg', 3.0)))
        self.gap = float(p('cluster_gap', 0.15))
        self.max_range = float(p('max_range', 4.0))
        self.merge_dist = float(p('merge_dist', 0.5))
        self.merge_dist_ground = float(p('merge_dist_ground', 0.9))   # ค่าจากขอบล่างกรอบแกว่งมากกว่า
        # + = ก้มลง · ผู้ใช้บอก "มองตรง" แต่เทียบภาพกับ lidar (23 ก.ย.) ตีนเสาขาว (lidar 1.78 m) และราวบันได
        # แตะพื้นสูงกว่าที่กล้องมองตรงควรเห็น ~60 px -> ก้มราว 6.5°
        self.pitch = math.radians(float(p('camera_pitch_deg', 6.5)))
        self.person_ttl = float(p('person_ttl', 5.0))   # คนเดินได้ -- หมุดหายถ้าไม่เห็นเกินนี้
        self.min_hits = int(p('min_hits', 3))           # เห็นซ้ำกี่ครั้งถึงปักหมุด (กันเจอผิดครั้งเดียว)
        self.no_costmap = set(p('costmap_exclude', ['person']))
        self.radius = {'chair': 0.25, 'dining table': 0.4, 'couch': 0.5, 'bed': 0.6, 'bench': 0.4}
        self.default_radius = float(p('default_radius', 0.2))
        self.clear_range = float(p('clear_range', 3.0))     # ลำแสงล้างใน costmap ยาวจากกล้อง (m)
        self.forget_after = float(p('forget_after', 10.0))  # ของที่อยู่ในมุมกล้องแต่ไม่เห็นเกินนี้ = ถูกย้ายไปแล้ว
        # หมุดปลอม (24 ก.ย. เจอ bench/skateboard/cat/airplane ที่ไม่มีจริง ขวางทางกลับบ้าน)
        # 1. lidar มองทะลุ: หมุดที่ปักจากจุด lidar ถ้าลำแสงทิศนั้นยาวเลยหมุดไป see_through_margin (หรือไม่มีจุดสะท้อน)
        #    ติดกัน see_through_count รอบ (1 Hz) = ตรงนั้นว่างแล้ว ลบหมุด
        # 2. ชนิดที่ไม่มีทางอยู่ในห้องนี้ ไม่ปักเลย
        self.see_margin = float(p('see_through_margin', 0.3))
        self.see_count = int(p('see_through_count', 3))
        self.see_max = float(p('see_through_max_dist', 3.0))
        self.ignore = set(p('ignore_classes', [
            'airplane', 'train', 'bus', 'truck', 'boat', 'car', 'motorcycle', 'skateboard', 'surfboard',
            'skis', 'snowboard', 'kite', 'frisbee', 'sports ball', 'baseball bat', 'baseball glove',
            'tennis racket', 'giraffe', 'elephant', 'zebra', 'bear', 'horse', 'cow', 'sheep', 'bird',
            'traffic light', 'fire hydrant', 'stop sign', 'parking meter']))
        self.fx = (self.W / 2.0) / math.tan(self.hfov / 2.0)
        self.cx, self.cy = self.W / 2.0, self.H / 2.0
        self.horizon = self.cy - self.fx * math.tan(self.pitch)      # แถวพิกเซลของเส้นขอบฟ้า (ระยะไกลสุด)

        self.scan = None
        self.laser_tf = None
        self.tracks = []          # dict: cls x y hits last source
        # จำของข้ามรอบ (24 ก.ย. ผู้ใช้: เก็บของที่กล้องเห็นลงแมพแยก รอบหน้าจะได้ไม่ชนของที่เคยเห็นแล้ว)
        # เก็บหมุดที่ยืนยันแล้ว (hits >= min_hits) ยกเว้นคน ทุก save_every_s วิ โหลดกลับตอนเปิด node
        # หมุดที่โหลดมายังโดนลบได้ตามปกติ (lidar มองทะลุ / อยู่ในมุมกล้องแต่ไม่เห็น) -- ของถูกย้ายก็หายเอง
        # !! พิกัดผูกกับแมพ SLAM รอบนั้น เหมือน keepout_zones -- restart แล้วตรงเฉพาะถ้าหุ่นเริ่มที่เดิมทิศเดิม !!
        self.obj_file = os.path.expanduser(p('objects_file', '~/maps/objects.json'))
        self.save_every = float(p('save_every_s', 5.0))
        self.last_save = 0.0
        self.last_saved_json = None
        self.load_objects()
        self.calib = []           # (tan มุมจาก lidar, พิกเซลกลางกรอบ) ไว้เทียบ hfov
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, '/detections/summary', self.on_detections, 10)
        self.create_subscription(LaserScan, '/scan_nav', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        self.obj_pub = self.create_publisher(String, '/objects', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/object_cloud', 10)
        self.clear_pub = self.create_publisher(PointCloud2, '/object_clear_cloud', 10)
        self.create_service(Trigger, '~/clear', self.srv_clear)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(
            f'พร้อม: /detections/summary + /scan_nav -> /object_markers  hfov {math.degrees(self.hfov):.0f}° '
            f'กล้อง x={self.cam_x:.2f} สูง {self.cam_z:.2f} m')

    # ------------------------------------------------------------------
    def bearing(self, u):
        """พิกเซลแนวนอน -> มุมจากแกนหน้ากล้อง (rad, + = ซ้ายตาม ROS)"""
        return -math.atan((u - self.cx) / self.fx)

    def scan_points(self):
        """จุด lidar ล่าสุดในเฟรม base_link (N x 2) หรือ None"""
        m = self.scan
        if m is None:
            return None
        if self.laser_tf is None:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, m.header.frame_id, Time()).transform
            except TransformException:
                return None
            self.laser_tf = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        lx, ly, lyaw = self.laser_tf
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + m.angle_increment * np.arange(r.size)
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        return np.stack([lx + r[ok] * np.cos(a[ok] + lyaw), ly + r[ok] * np.sin(a[ok] + lyaw)], axis=1)

    def locate(self, box, pts):
        """(x, y ในเฟรม base_link, 'lidar'|'ground') หรือ None"""
        u1, v1, u2, v2 = box
        a_left, a_right = self.bearing(u1), self.bearing(u2)
        ground = self.ground_estimate(box)
        if pts is not None and len(pts):
            c = pts - np.array([self.cam_x, self.cam_y])
            ang = np.arctan2(c[:, 1], c[:, 0])
            rng = np.hypot(c[:, 0], c[:, 1])
            sel = (c[:, 0] > 0) & (ang <= a_left + self.margin) & (ang >= a_right - self.margin) \
                & (rng > 0.15) & (rng < self.max_range)
            if sel.sum() >= 2:
                rs = rng[sel]
                near = rs <= rs.min() + self.gap
                # จุด lidar ไกลกว่าที่ขอบล่างกรอบบอกมาก = lidar ลอดใต้/ข้างของชิ้นนี้ไปเจอผนังข้างหลัง -> ไม่ใช้
                behind = ground is not None and rs.min() > 1.5 * math.hypot(
                    ground[0] - self.cam_x, ground[1] - self.cam_y) + 0.3
                if near.sum() >= 2 and not behind:
                    xy = np.median(pts[sel][near], axis=0)
                    cang = math.atan2(xy[1] - self.cam_y, xy[0] - self.cam_x)
                    self.calib.append((math.tan(cang), (u1 + u2) / 2.0))
                    return float(xy[0]), float(xy[1]), 'lidar'
        return ground

    def ground_estimate(self, box):
        """ขอบล่างกรอบแตะพื้น + กล้องสูง cam_z มองตรง -> (x, y, 'ground') ในเฟรม base_link หรือ None
        ขอบล่างต้องต่ำกว่าเส้นขอบฟ้าและไม่ชนขอบล่างภาพ (ชน = ของใกล้จนล้นภาพ ระยะเชื่อไม่ได้)"""
        u1, _v1, u2, v2 = box
        if not (self.horizon + 5 < v2 < self.H - 3):
            return None
        # มุมก้มของแนวมองไปจุดแตะพื้น = pitch + มุมใต้แกนกล้อง (พิกเซลเป็นสี่เหลี่ยมจัตุรัส fy = fx)
        dip = self.pitch + math.atan((v2 - self.cy) / self.fx)
        if dip <= 0.01:
            return None
        fwd = self.cam_z / math.tan(dip)
        if fwd >= self.max_range:
            return None
        side = fwd * math.tan(self.bearing((u1 + u2) / 2.0))
        return self.cam_x + fwd, self.cam_y + side, 'ground'

    def on_detections(self, msg):
        try:
            dets = json.loads(msg.data).get('detections', [])
        except ValueError:
            return
        dets = [d for d in dets if d.get('conf', 0) >= self.min_conf and d.get('class') not in self.ignore]
        if not dets:
            return
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time()).transform
        except TransformException:
            return
        bx, by, byaw = t.translation.x, t.translation.y, yaw_of(t.rotation)
        c, s = math.cos(byaw), math.sin(byaw)
        pts = self.scan_points()
        now = self.get_clock().now().nanoseconds * 1e-9
        for d in dets:
            hit = self.locate(d['box'], pts)
            if hit is None:
                continue
            x, y, src = hit
            if src == 'ground':
                # ขอบล่างกรอบ = ขา/ฐานด้านที่ใกล้กล้องสุด -> กลางของอยู่ลึกเข้าไปอีกราวรัศมีตามแนวมอง
                r = self.radius.get(d['class'], self.default_radius)
                dx, dy = x - self.cam_x, y - self.cam_y
                k = 1.0 + r / max(math.hypot(dx, dy), 0.1)
                x, y = self.cam_x + dx * k, self.cam_y + dy * k
            mx, my = bx + c * x - s * y, by + s * x + c * y
            self.update_track(d['class'], mx, my, src, now)
        if len(self.calib) >= 30:
            self.report_calib()

    def update_track(self, cls, x, y, src, now):
        best, bd = None, (self.merge_dist_ground if src == 'ground' else self.merge_dist)
        for tr in self.tracks:
            if tr['cls'] == cls:
                dd = math.hypot(tr['x'] - x, tr['y'] - y)
                if dd < bd:
                    best, bd = tr, dd
        if best is None:
            self.tracks.append(dict(cls=cls, x=x, y=y, hits=1, last=now, source=src, hist=[(x, y)]))
            return
        # ตำแหน่งจาก lidar เชื่อมากกว่าจากขอบล่างกรอบ -- ไม่ให้ค่าประมาณลากหมุดที่ lidar ยืนยันแล้ว
        if best['source'] == 'lidar' and src == 'ground':
            best['last'] = now
            return
        if src != best['source']:
            best['hist'] = []
        # ค่ากลาง 30 ครั้งล่าสุด (~4 วิ ที่ YOLO 7.5 Hz) -- ทนกรอบที่กระตุกบางเฟรม
        best['hist'] = (best['hist'] + [(x, y)])[-30:]
        best['x'], best['y'] = (float(v) for v in np.median(np.array(best['hist']), axis=0))
        best['hits'] += 1
        best['last'] = now
        best['source'] = src

    def report_calib(self):
        """เทียบพิกเซลกลางกรอบกับมุมจาก lidar: u = cx - fx*tan(มุม) -> fx ที่เข้ากับข้อมูลที่สุด -> hfov"""
        a = np.array(self.calib)
        self.calib = []
        tn, du = a[:, 0], a[:, 1] - self.cx
        if np.ptp(tn) < 0.2:
            return      # ของอยู่มุมเดียว ประมาณ fx ไม่ได้
        fx = -float(np.dot(tn, du) / np.dot(tn, tn))
        if fx <= 0:
            return
        res = du + fx * tn
        hfov = math.degrees(2 * math.atan(self.cx / fx))
        self.get_logger().info(
            f'เทียบมุมกล้องกับ lidar ({len(a)} ครั้ง): hfov ที่เข้ากับข้อมูล {hfov:.1f}° (ตั้งไว้ '
            f'{math.degrees(self.hfov):.1f}°) คลาดเฉลี่ย {np.abs(res).mean():.0f} px')

    # ------------------------------------------------------------------
    def load_objects(self):
        try:
            with open(self.obj_file) as f:
                data = json.load(f)
        except FileNotFoundError:
            return
        except ValueError as e:
            self.get_logger().error(f'อ่าน {self.obj_file} ไม่ได้ ({e}) -- เริ่มแบบไม่มีหมุด')
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for o in data.get('objects', []):
            self.tracks.append(dict(cls=o['cls'], x=o['x'], y=o['y'], hits=max(int(o.get('hits', 0)), self.min_hits),
                                    last=now, source=o.get('source', 'lidar'), hist=[(o['x'], o['y'])]))
        self.get_logger().info(f'โหลดหมุดของที่เคยเห็น {len(self.tracks)} ชิ้นจาก {self.obj_file}')

    def save_objects(self, now):
        if now - self.last_save < self.save_every:
            return
        self.last_save = now
        objs = [dict(cls=tr['cls'], x=round(tr['x'], 3), y=round(tr['y'], 3), hits=tr['hits'],
                     source=tr.get('source', 'lidar'))
                for tr in self.tracks if tr['hits'] >= self.min_hits and tr['cls'] != 'person']
        text = json.dumps({'frame': self.map_frame, 'objects': objs}, ensure_ascii=False, indent=1)
        if text == self.last_saved_json:
            return
        try:
            os.makedirs(os.path.dirname(self.obj_file), exist_ok=True)
            tmp = self.obj_file + '.tmp'
            with open(tmp, 'w') as f:
                f.write(text)
            os.replace(tmp, self.obj_file)
            self.last_saved_json = text
        except OSError as e:
            self.get_logger().error(f'เซฟ {self.obj_file} ไม่ได้: {e}')

    def see_through(self):
        """นับรอบที่ lidar มองทะลุตำแหน่งหมุด (เฉพาะหมุดที่ปักจากจุด lidar) -- ครบ see_count ลบทิ้ง"""
        m = self.scan
        if m is None or self.laser_tf is None or not self.tracks:
            return
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.map_frame, Time()).transform
        except TransformException:
            return
        c, s_ = math.cos(yaw_of(t.rotation)), math.sin(yaw_of(t.rotation))
        lx, ly, lyaw = self.laser_tf
        n = len(m.ranges)
        keep = []
        for tr in self.tracks:
            if tr.get('source') != 'lidar':
                keep.append(tr)
                continue
            bx = t.translation.x + c * tr['x'] - s_ * tr['y']
            by = t.translation.y + s_ * tr['x'] + c * tr['y']
            dx, dy = bx - lx, by - ly
            d = math.hypot(dx, dy)
            if d > self.see_max or d < 0.15:
                keep.append(tr)
                continue
            a = math.atan2(dy, dx) - lyaw
            a = math.atan2(math.sin(a), math.cos(a))
            i = int(round((a - m.angle_min) / m.angle_increment))
            if not 0 <= i < n:
                keep.append(tr)
                continue
            # ดูลำแสงข้างเคียงด้วย (หมุดมีขนาด) -- ทะลุทุกเส้นถึงนับ
            rs = [m.ranges[(i + k) % n] for k in (-2, -1, 0, 1, 2)]
            through = all((not math.isfinite(r)) or r > d + self.see_margin for r in rs)
            hit = any(math.isfinite(r) and abs(r - d) < 0.2 for r in rs)
            if hit:
                tr['through'] = 0
            elif through:
                tr['through'] = tr.get('through', 0) + 1
            if tr.get('through', 0) >= self.see_count:
                self.get_logger().info(f'ลบหมุด {tr["cls"]} x={tr["x"]:.2f} y={tr["y"]:.2f} -- lidar มองทะลุ')
                continue
            keep.append(tr)
        self.tracks = keep

    def publish(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.see_through()
        self.save_objects(now)
        self.tracks = [tr for tr in self.tracks
                       if not (tr['cls'] == 'person' and now - tr['last'] > self.person_ttl)
                       and not (now - tr['last'] > self.forget_after and self.in_view(tr))]
        shown = [tr for tr in self.tracks if tr['hits'] >= self.min_hits]
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, tr in enumerate(shown):
            col = class_color(tr['cls'])
            pin = Marker()
            pin.header.frame_id = self.map_frame
            pin.ns, pin.id, pin.type, pin.action = 'objects', 2 * i, Marker.CYLINDER, Marker.ADD
            pin.pose.position.x, pin.pose.position.y, pin.pose.position.z = tr['x'], tr['y'], 0.15
            pin.pose.orientation.w = 1.0
            pin.scale.x = pin.scale.y = 0.15
            pin.scale.z = 0.30
            pin.color.r, pin.color.g, pin.color.b = col
            pin.color.a = 0.9 if tr['source'] == 'lidar' else 0.45
            label = Marker()
            label.header.frame_id = self.map_frame
            label.ns, label.id, label.type, label.action = 'objects', 2 * i + 1, Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = tr['x'], tr['y'], 0.45
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = ('' if tr['source'] == 'lidar' else '~') + tr['cls']
            arr.markers += [pin, label]
        self.marker_pub.publish(arr)
        out = String()
        out.data = json.dumps([{k: (round(v, 2) if isinstance(v, float) else v) for k, v in tr.items()
                                if k not in ('last', 'hist')} for tr in shown])
        self.obj_pub.publish(out)
        self.publish_cloud([tr for tr in shown if tr['cls'] not in self.no_costmap])

    def to_base(self):
        """(tx, ty, cos, sin) ที่แปลงพิกัด map -> base_link หรือ None"""
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.map_frame, Time())
        except TransformException:
            return None
        yaw = yaw_of(t.transform.rotation)
        return t, (t.transform.translation.x, t.transform.translation.y, math.cos(yaw), math.sin(yaw))

    def in_view(self, tr):
        """ของชิ้นนี้อยู่ในมุมกล้องและระยะที่ควรเห็นไหม (ไม่เห็นเกิน forget_after = ถูกย้ายไปแล้ว)"""
        got = self.to_base()
        if got is None:
            return False
        tx, ty, c, s = got[1]
        x = tx + c * tr['x'] - s * tr['y'] - self.cam_x
        y = ty + s * tr['x'] + c * tr['y'] - self.cam_y
        return 0.4 < x < self.clear_range and abs(math.atan2(y, x)) < self.hfov / 2 - math.radians(5)

    def publish_cloud(self, tracks):
        """วงกลมรัศมีตามชนิดของ (ตาราง 5 cm) ในเฟรม base_link -- costmap ต้องการต้นทางเซนเซอร์ใกล้หุ่น
        (ถ้าส่งในเฟรม map ระยะ obstacle_max_range จะนับจากจุด 0 ของแมพ)"""
        got = self.to_base()
        if got is None:
            return
        t, (tx, ty, c, s) = got
        pts = []
        for tr in tracks:
            r = self.radius.get(tr['cls'], self.default_radius)
            for dx in np.arange(-r, r + 1e-6, 0.05):
                for dy in np.arange(-r, r + 1e-6, 0.05):
                    if dx * dx + dy * dy <= r * r:
                        mx, my = tr['x'] + dx, tr['y'] + dy
                        pts.append((tx + c * mx - s * my, ty + s * mx + c * my, 0.1))
        # header ของ TF = เฟรม base_link + เวลาของ TF ล่าสุด (ใช้เวลาปัจจุบัน costmap จะหา TF ล่วงหน้าไม่เจอแล้วทิ้ง)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(t.header, pts))
        # ปลายลำแสงล้าง ทั่วมุมกล้อง (เว้นขอบภาพ 2°) -- costmap ลบเครื่องหมายเก่าตลอดแนวก่อนทำเครื่องหมายของปัจจุบัน
        half = self.hfov / 2 - math.radians(2)
        ends = [(self.cam_x + self.clear_range * math.cos(a), self.cam_y + self.clear_range * math.sin(a), 0.1)
                for a in np.arange(-half, half + 1e-6, math.radians(1.0))]
        self.clear_pub.publish(point_cloud2.create_cloud_xyz32(t.header, ends))

    def srv_clear(self, _req, res):
        n = len(self.tracks)
        self.tracks = []
        self.publish()
        res.success = True
        res.message = f'ลบหมุดของ {n} ชิ้น'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
