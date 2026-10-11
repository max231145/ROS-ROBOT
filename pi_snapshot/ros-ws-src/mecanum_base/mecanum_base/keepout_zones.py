"""keepout_zones -- เขตห้ามเข้าที่ผู้ใช้คลิก 4 มุมบนแมพใน Foxglove

    Foxglove แผง 3D ปุ่ม "Publish point" -> /clicked_point  คลิกครบ 4 มุม = ได้เขตหนึ่ง (เก็บลงไฟล์ทันที)
    /keepout_cloud   PointCloud2 (base_link) จุดเต็มพื้นที่ทุกเขต -> keepout_layer ใน costmap (Nav2 ไม่วางทางผ่าน)
    ~/markers        MarkerArray กรอบเขต (แดง) + มุมที่คลิกค้างอยู่ (เหลือง)
    ~/undo           ลบเขตล่าสุด   ~/clear_pending  ทิ้งมุมที่คลิกค้าง   ~/clear_all  ลบทุกเขต

ทำไม (2026-09-24): โต๊ะเหล็กคานขวางต่ำ lidar เห็นแค่ขา คานอยู่สูงกว่าระนาบ lidar YOLO ไม่รู้จัก
-> แมพว่างระหว่างขา หุ่นลอดใต้โต๊ะแล้วชนทุกรอบ ผู้ใช้เลือกวิธี "คลิก 4 มุม"

!! พิกัดเก็บในเฟรม map ของ SLAM รอบปัจจุบัน -- restart ros2-slam แมพเริ่มใหม่ที่ตำแหน่งหุ่นตอนนั้น
   เขตจะตรงที่เดิมก็ต่อเมื่อหุ่นเริ่มจากจุดเดิม หันทิศเดิม (แก้ถาวรเมื่อโหลดแมพเดิมกลับมาได้) !!
"""
import json
import math
import os

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def inside(x, y, poly):
    """จุดอยู่ในรูปหลายเหลี่ยมไหม (ray casting)"""
    c = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            c = not c
        j = i
    return c


def order_ccw(pts):
    """เรียงมุมรอบจุดกลาง -- คลิกมุมไม่ตามลำดับก็ไม่เป็นรูปผีเสื้อ"""
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


class KeepoutZones(Node):
    def __init__(self):
        super().__init__('keepout_zones')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.path = os.path.expanduser(self.declare_parameter('file', '~/maps/keepout_zones.json').value)
        self.corners = int(self.declare_parameter('corners', 4).value)
        self.step = float(self.declare_parameter('fill_step', 0.05).value)     # เท่าความละเอียด costmap
        self.zones = self.load()
        self.pending = []
        self.fill = []            # จุดเต็มพื้นที่ทุกเขต (map) คำนวณครั้งเดียวตอนเขตเปลี่ยน
        self.refill()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cloud_pub = self.create_publisher(PointCloud2, '/keepout_cloud', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', latched)
        self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)
        self.create_service(Trigger, '~/undo', self.srv_undo)
        self.create_service(Trigger, '~/clear_pending', self.srv_clear_pending)
        self.create_service(Trigger, '~/clear_all', self.srv_clear_all)
        self.create_timer(0.5, self.publish_cloud)
        self.publish_markers()
        self.get_logger().info(f'พร้อม: เขตห้ามเข้า {len(self.zones)} เขตจาก {self.path} -- '
                               f'คลิก "Publish point" {self.corners} มุมเพื่อเพิ่มเขต')

    # ------------------------------------------------------------------
    def load(self):
        try:
            with open(self.path) as f:
                return [[tuple(p) for p in z] for z in json.load(f)['zones']]
        except FileNotFoundError:
            return []
        except (ValueError, KeyError) as e:
            self.get_logger().error(f'อ่าน {self.path} ไม่ได้ ({e}) -- เริ่มแบบไม่มีเขต')
            return []

    def save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump({'frame': self.map_frame, 'zones': self.zones}, f, indent=1)
        os.replace(tmp, self.path)

    def refill(self):
        self.fill = []
        for poly in self.zones:
            xs, ys = [p[0] for p in poly], [p[1] for p in poly]
            x = min(xs)
            while x <= max(xs) + 1e-6:
                y = min(ys)
                while y <= max(ys) + 1e-6:
                    if inside(x, y, poly):
                        self.fill.append((x, y))
                    y += self.step
                x += self.step
            self.fill += [tuple(p) for p in poly]      # มุมเองด้วย เขตแคบก็ยังได้จุด

    # ------------------------------------------------------------------
    def on_click(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, Time())
            except TransformException as e:
                self.get_logger().error(f'แปลงจุดที่คลิกเข้า {self.map_frame} ไม่ได้: {e}')
                return
            t = tf.transform
            c, s = math.cos(yaw_of(t.rotation)), math.sin(yaw_of(t.rotation))
            x = t.translation.x + c * msg.point.x - s * msg.point.y
            y = t.translation.y + s * msg.point.x + c * msg.point.y
        else:
            x, y = msg.point.x, msg.point.y
        self.pending.append((round(x, 3), round(y, 3)))
        self.get_logger().info(f'มุมที่ {len(self.pending)}/{self.corners}: x={x:.2f} y={y:.2f}')
        if len(self.pending) >= self.corners:
            poly = order_ccw(self.pending)
            self.zones.append(poly)
            self.pending = []
            self.save()
            self.refill()
            self.get_logger().info(f'เพิ่มเขตห้ามเข้าที่ {len(self.zones)} แล้ว เก็บลง {self.path}')
        self.publish_markers()

    def srv_undo(self, _req, res):
        if self.pending:
            self.pending = []
            res.message = 'ทิ้งมุมที่คลิกค้างแล้ว'
        elif self.zones:
            self.zones.pop()
            self.save()
            self.refill()
            res.message = f'ลบเขตล่าสุดแล้ว เหลือ {len(self.zones)} เขต'
        else:
            res.message = 'ไม่มีเขตให้ลบ'
        res.success = True
        self.publish_markers()
        self.get_logger().info(res.message)
        return res

    def srv_clear_pending(self, _req, res):
        self.pending = []
        self.publish_markers()
        res.success, res.message = True, 'ทิ้งมุมที่คลิกค้างแล้ว'
        return res

    def srv_clear_all(self, _req, res):
        self.zones, self.pending = [], []
        self.save()
        self.refill()
        self.publish_markers()
        res.success, res.message = True, 'ลบเขตห้ามเข้าทั้งหมดแล้ว'
        self.get_logger().info(res.message)
        return res

    # ------------------------------------------------------------------
    def publish_cloud(self):
        """ส่งใน base_link: costmap ตัดจุดตามระยะจาก origin ของ frame ใน cloud (ส่งใน map แล้วไกลจาก 0,0 จะถูกทิ้ง)"""
        if not self.fill:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.map_frame, Time())
        except TransformException:
            return
        t = tf.transform
        c, s = math.cos(yaw_of(t.rotation)), math.sin(yaw_of(t.rotation))
        pts = [(t.translation.x + c * x - s * y, t.translation.y + s * x + c * y, 0.1) for x, y in self.fill]
        header = tf.header
        header.frame_id = self.base_frame
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))

    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        mid = 1
        for i, poly in enumerate(self.zones):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns, m.id, m.type, m.action = 'keepout', mid, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.04
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 1.0
            m.points = [Point(x=x, y=y, z=0.06) for x, y in poly + [poly[0]]]
            arr.markers.append(m)
            mid += 1
            cx = sum(p[0] for p in poly) / len(poly)
            cy = sum(p[1] for p in poly) / len(poly)
            t = Marker()
            t.header.frame_id = self.map_frame
            t.ns, t.id, t.type, t.action = 'keepout', mid, Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = cx, cy, 0.3
            t.pose.orientation.w = 1.0
            t.scale.z = 0.15
            t.color.r, t.color.g, t.color.b, t.color.a = 0.9, 0.1, 0.1, 1.0
            t.text = f'ห้ามเข้า {i + 1}'
            arr.markers.append(t)
            mid += 1
        for k, (x, y) in enumerate(self.pending):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns, m.id, m.type, m.action = 'pending', mid, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.08
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.0, 1.0
            arr.markers.append(m)
            mid += 1
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = KeepoutZones()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
