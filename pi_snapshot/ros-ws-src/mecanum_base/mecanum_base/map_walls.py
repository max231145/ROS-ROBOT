#!/usr/bin/env python3
"""
map_walls -- ยกช่องที่เป็นสิ่งกีดขวางใน /map ขึ้นเป็นกำแพงสูงไล่สีตามความสูง ให้ดูใน Foxglove ง่ายขึ้น

  subscribe  /map         nav_msgs/OccupancyGrid  (slam_toolbox)
  publish    /map_walls   visualization_msgs/MarkerArray  (latched -- เปิด Foxglove ทีหลังก็เห็น)

!! ความสูงเป็นค่าที่ตั้งเอง ไม่ใช่ของจริง -- lidar เห็นแค่ระนาบเดียว (ความสูงของ lidar)
   ของที่อยู่สูง/ต่ำกว่าระนาบนั้น (ขอบโต๊ะ เบาะเก้าอี้) ไม่อยู่ในแมพ !!

พารามิเตอร์
  wall_height     0.40   m   ความสูงกำแพงที่วาด
  occupied_min    65         ค่าใน /map ตั้งแต่เท่านี้ถือว่าเป็นสิ่งกีดขวาง (slam_toolbox ใช้ 100)
  min_neighbors   1          ตัดช่องเดี่ยวที่รอบ 8 ช่องมีสิ่งกีดขวางน้อยกว่านี้ (จุดรบกวน/คนเดินผ่าน) 0 = ไม่ตัด
  min_period      2.0    s   ส่งใหม่ไม่ถี่กว่านี้ (slam ส่งแมพทุก map_update_interval)
"""

import colorsys
import signal

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def layer_color(frac):
    """0 = ล่างสุด (น้ำเงิน) -> 1 = บนสุด (แดง) แบบรูป RViz/OctoMap"""
    r, g, b = colorsys.hsv_to_rgb(0.66 * (1.0 - frac), 1.0, 1.0)
    return ColorRGBA(r=r, g=g, b=b, a=1.0)


class MapWalls(Node):

    def __init__(self):
        super().__init__('map_walls')
        self.height = float(self.declare_parameter('wall_height', 0.40).value)
        self.occ_min = int(self.declare_parameter('occupied_min', 65).value)
        self.min_nb = int(self.declare_parameter('min_neighbors', 1).value)
        self.min_period = float(self.declare_parameter('min_period', 2.0).value)
        grid_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, '/map_walls', grid_qos)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, grid_qos)
        self.pending = None
        self.last_sent = 0.0
        self.create_timer(0.5, self.flush)
        self.get_logger().info(f'/map -> /map_walls กำแพงสูง {self.height:.2f} m')

    def on_map(self, msg):
        self.pending = msg

    def flush(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.pending is None or now - self.last_sent < self.min_period:
            return
        msg, self.pending = self.pending, None
        self.last_sent = now
        self.pub.publish(self.build(msg))

    def build(self, msg):
        info = msg.info
        a = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        occ = a >= self.occ_min
        if self.min_nb > 0:
            p = np.pad(occ, 1).astype(np.int8)
            nb = sum(p[1 + dr:1 + dr + occ.shape[0], 1 + dc:1 + dc + occ.shape[1]]
                     for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc)
            occ &= nb >= self.min_nb
        rows, cols = np.nonzero(occ)
        res = info.resolution
        # แมพของ slam_toolbox ไม่หมุน -- ใช้ origin x,y ตรง ๆ
        xs = info.origin.position.x + (cols + 0.5) * res
        ys = info.origin.position.y + (rows + 0.5) * res
        cells = [Point(x=float(x), y=float(y)) for x, y in zip(xs, ys)]

        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        layers = max(1, int(round(self.height / res)))
        for k in range(layers):
            m = Marker()
            m.header = msg.header
            m.ns = 'walls'
            m.id = k
            m.type = Marker.CUBE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.pose.position.z = (k + 0.5) * res
            m.scale.x = m.scale.y = m.scale.z = res
            m.color = layer_color(k / max(1, layers - 1))
            m.points = cells
            arr.markers.append(m)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = MapWalls()
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
