#!/usr/bin/env python3
"""
waypoint_manager -- ให้หุ่นวิ่งตามจุดที่กดบนแมพ หรือสำรวจแมพเอง แล้วกลับจุดเริ่มต้นทุกครั้ง

วิธีใช้ (Foxglove):
  วิ่งตามจุด
    1. แผง 3D ปุ่ม Publish โหมด "Publish 2D pose" คลิกบนแมพ = เพิ่มจุด (ยังไม่วิ่ง)
    2. ~/start  -> จำตำแหน่งหุ่นตอนนั้นเป็นจุดเริ่มต้น วิ่งไปจุด 1, 2, ... ครบแล้วกลับจุดเริ่ม
    3. ~/undo ลบจุดล่าสุด   ~/clear ลบทุกจุด
  สำรวจแมพ
    ~/explore -> จำจุดเริ่ม แล้ววนหา "ขอบแมพ" (ที่ว่างที่ติดกับที่ยังไม่รู้) ที่ใกล้และคุ้มที่สุด
                 ไปยืนตรงนั้นให้ lidar เห็นพื้นที่ใหม่ จนไม่เหลือขอบ / ครบเวลา / ครบจำนวน แล้วกลับจุดเริ่ม
    ~/explore_preview -> คำนวณอย่างเดียว แสดงขอบกับเป้าหมายถัดไปบนแมพ หุ่นไม่ขยับ
  ทั้งสองโหมด
    ~/cancel หยุดทันที

  subscribe  goal_topic                 geometry_msgs/PoseStamped  (ค่าตั้งต้น /move_base_simple/goal)
             /map  /global_costmap/costmap   nav_msgs/OccupancyGrid  (ใช้ตอนสำรวจ)
  publish    ~/markers                  visualization_msgs/MarkerArray
  service    ~/start ~/explore ~/explore_preview ~/cancel ~/clear ~/undo   std_srvs/Trigger
  action     navigate_to_pose           nav2_msgs/NavigateToPose  (bt_navigator)

พฤติกรรม:
  * ส่งเป้าหมายทีละจุดด้วย NavigateToPose (BT nav_to_pose_safe.xml)
  * จุดไหนไปไม่ถึง: โหมดวิ่งตามจุด = ข้ามไปจุดถัดไป  โหมดสำรวจ = จำไว้ไม่ไปซ้ำ แล้วหาขอบอื่น
  * ก่อนออกวิ่งล้าง costmap ทิ้ง (คน/ของที่เคยยืนใกล้หุ่นค้างอยู่) แล้วรอ 1.5 s
  * ระหว่างวิ่ง คลิกเพิ่มจุดไม่ได้
  * ไม่รับจุดที่ตัวหุ่นจะทับสิ่งกีดขวางใน global costmap / ชิดกำแพงเกิน / ทับตัวหุ่นเอง -- แสดงกากบาทแดงแทน
  * จุดในที่ยังไม่มีในแมพ (สีเทา) รับ แต่ขึ้นป้ายส้ม -- Nav2 วางเส้นทางผ่านที่ไม่รู้จักได้ (allow_unknown)
    (23 ก.ย. เคยไม่รับ: ช่องประตูที่ lidar ลอดเกิน max_laser_range เป็นสีเทา ผู้ใช้วางจุดไม่ได้จนงง)
  * สีหมุด: ฟ้า = รอ  เหลือง = กำลังไป  เขียว = ถึงแล้ว  แดง = ไปไม่ถึง  ส้ม = จุดเริ่มต้น
           ม่วงจุดเล็ก = ขอบแมพ  ม่วงใหญ่ = เป้าหมายสำรวจ  แดงเล็ก = ขอบที่ไปไม่ถึง
  * สำรวจ (23 ก.ย.: สายแลน/ปลั๊กพ่วงยาวแค่ ~1 m -- ขยายรัศมีต้องถามผู้ใช้ก่อนทุกครั้ง):
      - เป้าอยู่ในรัศมี explore_max_radius - explore_goal_margin จากจุดเริ่ม
      - รั้วเสมือน: กลางหุ่นห่างจุดเริ่มเกิน explore_max_radius = ยกเลิกเป้าแล้วกลับจุดเริ่มทันที
      - หมุนสะสมเกิน explore_max_turn_deg (สายพันตัวหุ่น) = ยกเลิกแล้วกลับจุดเริ่ม

!! ไม่ได้เปิดไดรเวอร์มอเตอร์ให้ ต้องเรียก /mecanum_base/enable true เอง (tools/explore_run.py ทำให้) !!
"""

import copy
import json
import os
import math
import random
import signal
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import BackUp, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mecanum_base.frontier import UNKNOWN, Grid, choose_target, find_frontiers

PENDING, ACTIVE, DONE, FAILED = 'pending', 'active', 'done', 'failed'
COLORS = {                      # r g b
    PENDING: (0.2, 0.6, 1.0),
    ACTIVE: (1.0, 0.85, 0.1),
    DONE: (0.2, 0.85, 0.3),
    FAILED: (0.95, 0.2, 0.2),
}
HOME_COLOR = (1.0, 0.5, 0.0)
FRONTIER_COLOR = (0.7, 0.3, 1.0)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def seg_dist(px, py, ax, ay, bx, by):
    """ระยะจากจุด p ถึงเส้นตรง a-b"""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def pose_xy_yaw(frame, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class WaypointManager(Node):

    def __init__(self):
        super().__init__('waypoint_manager')
        goal_topic = self.declare_parameter('goal_topic', '/move_base_simple/goal').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.max_waypoints = int(self.declare_parameter('max_waypoints', 30).value)
        self.return_home = bool(self.declare_parameter('return_home', True).value)
        # สำรวจ -- ค่าตั้งต้นระวังไว้ก่อน (lidar มีโซนบอด ~10 cm รอบตัวหุ่น)
        # รัศมี 1 m: สายแลน/ปลั๊กพ่วงยาวแค่ ~1 m (2026-09-23) -- ขยายต้องถามผู้ใช้ก่อนทุกครั้ง
        self.ex_radius = float(self.declare_parameter('explore_max_radius', 1.0).value)
        self.ex_goal_margin = float(self.declare_parameter('explore_goal_margin', 0.2).value)
        self.ex_max_turn = float(self.declare_parameter('explore_max_turn_deg', 330.0).value)
        # เป้าอยู่ข้างหลัง (หันเกิน reverse_min_angle) และไม่ไกล: ถอยตรง ๆ ด้วย BackUp ก่อน แทนกลับหลังหัน
        # (23 ก.ย. ผู้ใช้: "หุ่นหมุนเป็นวงกลม เดินหน้าไม่เท่าไร" -- ขากลับเคยกลับหลังหัน 180° สองรอบ)
        self.rev_min_angle = math.radians(float(self.declare_parameter('reverse_min_angle_deg', 135.0).value))
        self.rev_max_dist = float(self.declare_parameter('reverse_max_dist', 1.2).value)
        self.rev_speed = float(self.declare_parameter('reverse_speed', 0.08).value)
        # ขอบแมพในรัศมีหมดแล้ว: เดินสุ่มต่อ -- ออกไปจุดว่าง (เดินหน้า) แล้วถอยกลับจุดเริ่ม สลับกันจนครบเวลา
        # ทิศของจุดไม่เกิน +-wander_max_angle จากทิศตอนเริ่ม หมุนสะสมจะไม่บาน (สายไม่พัน)
        self.wander_s = float(self.declare_parameter('explore_wander_s', 180.0).value)
        self.wander_max_angle = math.radians(float(self.declare_parameter('wander_max_angle_deg', 90.0).value))
        self.wander_min_dist = float(self.declare_parameter('wander_min_dist', 0.35).value)
        # จุดถัดไปห่างจากทิศที่หุ่นหันอยู่ไม่เกินนี้ (หาไม่ได้ค่อยปล่อย) -- ไม่ต้องหมุนตัวเยอะก่อนออกเดิน
        self.wander_turn_max = math.radians(float(self.declare_parameter('wander_turn_max_deg', 60.0).value))
        self.unwind_after = math.radians(float(self.declare_parameter('unwind_after_deg', 90.0).value))
        self.backup_retry_s = float(self.declare_parameter('backup_retry_s', 2.0).value)
        self.ex_duration = float(self.declare_parameter('explore_max_duration', 600.0).value)
        self.ex_max_goals = int(self.declare_parameter('explore_max_goals', 20).value)
        self.ex_min_size = float(self.declare_parameter('frontier_min_size', 0.5).value)
        self.ex_max_cost = int(self.declare_parameter('frontier_max_cost', 40).value)
        # ขนาดหุ่นตรงกับ nav2_params.yaml -- ใช้เช็คว่าจุดที่กดวางหุ่นได้ไหม (ไม่ทับสิ่งกีดขวางใน global costmap)
        fp = json.loads(self.declare_parameter(
            'footprint', '[[0.27, 0.244], [0.27, -0.244], [-0.24, -0.244], [-0.24, 0.244]]').value)
        xs, ys = [v[0] for v in fp], [v[1] for v in fp]
        self.fp_samples = [(x, y) for x in np.arange(min(xs), max(xs) + 1e-6, 0.05)
                           for y in np.arange(min(ys), max(ys) + 1e-6, 0.05)]
        self.rejected = None      # (x, y, เหตุผล) จุดล่าสุดที่ไม่รับ -- แสดงเป็นกากบาทแดงบนแมพ
        self.notices = {}         # index จุด -> ป้ายเตือนสีส้ม (รับจุดแล้วแต่ควรรู้ไว้)
        self.min_goal_dist = float(self.declare_parameter('min_goal_dist', 0.30).value)
        # ล้อติด (ชนสายไฟ/ของเตี้ยที่ lidar ไม่เห็น) -- 2026-09-24 ผู้ใช้ขอ "ชนแล้วไปต่อไม่ได้ให้ถอยแล้วไปทางอื่น"
        # สเต็ปเปอร์ open loop: ล้อ/EKF นับว่าไปแล้วทั้งที่ตัวไม่ไป จึงวัดการขยับจริงจาก rf2o (lidar)
        # สั่งเดินหน้า >= stall_min_cmd ตลอด stall_window_s แต่ rf2o ขยับไม่ถึง stall_min_ratio ของที่สั่ง = ติด
        # -> ยกเลิกเป้า ถอย stall_backup_dist จำจุดหน้าหุ่นไว้ (สีส้มบนแมพ) แล้วเลือกเป้าที่ทางไม่ผ่านจุดนั้น
        self.stall_window = float(self.declare_parameter('stall_window_s', 1.5).value)
        self.stall_min_cmd = float(self.declare_parameter('stall_min_cmd', 0.05).value)
        self.stall_min_ratio = float(self.declare_parameter('stall_min_ratio', 0.25).value)
        self.stall_backup = float(self.declare_parameter('stall_backup_dist', 0.35).value)
        self.stall_avoid_r = float(self.declare_parameter('stall_avoid_radius', 0.35).value)
        self.stall_min_turn = math.radians(float(self.declare_parameter('stall_min_turn_deg', 20.0).value))
        self.stall_veer = math.radians(float(self.declare_parameter('stall_veer_deg', 25.0).value))
        # 24 ก.ย. ผู้ใช้: ตอนเลี้ยว ล้อฝั่งที่ไม่หมุนลูกกลิ้งไม่ลื่น สะดุดข้ามสายไม่ได้ -> ถอยให้พ้นสาย (0.35 m) ก่อนเลี้ยว
        # และปักจุดติดลง costmap (stuck_layer ผ่าน /stuck_cloud) Nav2 จะวางทางอ้อม ไม่ไปหมุนตัวทับสายซ้ำ
        # จุดติดเป็นเส้นขวางทางเดิน (สายวางขวาง) ยาว 2 * stuck_mark_half_width
        self.stuck_half = float(self.declare_parameter('stuck_mark_half_width', 0.25).value)
        self.stuck_marks = []     # (x, y, yaw) ใน map -- ทิศหุ่นตอนติด ใช้วางเส้นขวาง
        # จำจุดที่ล้อติดข้ามรอบ (24 ก.ย. ผู้ใช้: "จดจำทางที่เดินติด รอบต่อไปจะได้หลบ")
        # เก็บ ~/maps/stuck_marks.json ทันทีที่ติด โหลดกลับตอนเปิด node -- ลบด้วย ~/clear_stuck
        # พิกัดผูกกับแมพ SLAM เดียวกับ keepout_zones/objects (โหลดแมพ room_main ตอนเปิด SLAM)
        self.stuck_file = os.path.expanduser(self.declare_parameter('stuck_file', '~/maps/stuck_marks.json').value)
        self.load_stuck()
        self.stall_points = [(m[0], m[1]) for m in self.stuck_marks]    # (x, y) ใน map -- จุดหน้าหุ่นตอนติด (รวมที่โหลดมา)
        self.cmd_hist = deque()   # (monotonic, vx ที่ส่งเข้าฐานจริง /cmd_vel)
        self.rf2o_hist = deque()  # (monotonic, x, y) จาก /odom_rf2o
        self.cur_goal = None      # (name, on_done) ของเป้า NavigateToPose ที่กำลังวิ่ง

        # รายการจุดที่ผู้ใช้กด
        self.waypoints = []
        self.status = []
        # สถานะการวิ่ง
        self.mode = None          # None | 'route' | 'explore'
        self.running = False
        self.cancel_requested = False
        self.run_id = 0           # กันผลของรอบเก่าที่มาถึงช้า ไปปนกับรอบใหม่
        self.goal_handle = None
        self.home = None
        self.home_status = PENDING
        self.step = 0             # route: index จุดถัดไป
        self.results = {DONE: 0, FAILED: 0}
        # สำรวจ
        self.ex_start = 0.0
        self.ex_goals = 0
        self.ex_blacklist = []
        self.ex_target = None
        self.ex_frontiers = []
        self.ex_cum_yaw = 0.0     # หมุนสะสม (rad) ตั้งแต่เริ่มสำรวจ -- กันสายพันตัวหุ่น
        self.ex_last_yaw = None
        self.ex_wander_until = None   # monotonic ที่เลิกเดินสุ่ม (None = ยังสำรวจขอบอยู่)
        self.geo_tripped = False  # รั้ว/หมุนสะสมสั่งกลับบ้านไปแล้ว ไม่ต้องสั่งซ้ำ
        self.map_msg = None
        self.cost_msg = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.backup = ActionClient(self, BackUp, 'backup')        # behavior_server
        self.spin = ActionClient(self, Spin, 'spin')              # behavior_server -- หมุนทางคลายการหมุนสะสม
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'),
            self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'),
        ]

        self.create_subscription(PoseStamped, goal_topic, self.on_click, 10)
        grid_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map_msg', m), grid_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'cost_msg', m), grid_qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', latched)
        self.create_service(Trigger, '~/start', self.srv_start)
        self.create_service(Trigger, '~/explore', self.srv_explore)
        self.create_service(Trigger, '~/explore_preview', self.srv_explore_preview)
        self.create_service(Trigger, '~/cancel', self.srv_cancel)
        self.create_service(Trigger, '~/clear', self.srv_clear)
        self.create_service(Trigger, '~/undo', self.srv_undo)
        self.create_service(Trigger, '~/clear_stuck', self.srv_clear_stuck)
        self.stuck_pub = self.create_publisher(PointCloud2, '/stuck_cloud', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Odometry, '/odom_rf2o', self.on_rf2o, 10)
        self.create_timer(0.2, self.guard_tick)

        self.publish_markers()
        self.get_logger().info(
            f'พร้อม: คลิก "Publish 2D pose" บนแมพเพื่อเพิ่มจุด ({goal_topic}) '
            f'แล้วเรียก ~/start  หรือเรียก ~/explore เพื่อสำรวจแมพเอง')

    # ------------------------------------------------------------------
    # รับจุดจาก Foxglove
    # ------------------------------------------------------------------
    def on_click(self, msg):
        if self.running:
            self.get_logger().warn('กำลังวิ่งอยู่ ไม่รับจุดเพิ่ม -- เรียก cancel ก่อน')
            return
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'จุดที่กดอยู่ในเฟรม "{msg.header.frame_id}" ไม่ใช่ "{self.map_frame}" -- '
                f'ตั้ง Display frame ของแผง 3D เป็น {self.map_frame} แล้วกดใหม่')
            return
        if len(self.waypoints) >= self.max_waypoints:
            self.get_logger().warn(f'จุดเต็มแล้ว ({self.max_waypoints}) -- clear หรือ undo ก่อน')
            return
        why = self.blocked_reason(msg.pose)
        p = msg.pose.position
        self.rejected = (p.x, p.y, why[1]) if why else None
        if why:
            self.get_logger().warn(f'ไม่รับจุด x={p.x:.2f} y={p.y:.2f}: {why[0]}')
            self.publish_markers()
            return
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose = msg.pose
        if self.map_msg is not None and Grid.from_msg(self.map_msg).value(p.x, p.y) == UNKNOWN:
            self.notices[len(self.waypoints)] = 'UNKNOWN AREA'
            self.get_logger().warn(f'จุด x={p.x:.2f} y={p.y:.2f} อยู่ในที่ยังไม่มีในแมพ -- รับไว้ Nav2 จะลองวางเส้นทางผ่าน')
        self.waypoints.append(pose)
        self.status = [PENDING] * len(self.waypoints)
        p = pose.pose.position
        self.get_logger().info(
            f'เพิ่มจุดที่ {len(self.waypoints)}: x={p.x:.2f} y={p.y:.2f} '
            f'หัน {math.degrees(yaw_of(pose.pose.orientation)):.0f} องศา')
        self.publish_markers()

    def blocked_reason(self, pose):
        """(เหตุผลภาษาไทยลง log, ป้ายภาษาอังกฤษบนแมพ) หรือ None -- ยังไม่มี costmap/แมพ ปล่อยให้ Nav2 ตัดสินเอง
        ป้ายเป็นอังกฤษเพราะตัวหนังสือ 3D ของ Foxglove ไม่แสดงวรรณยุกต์/สระบนล่างของไทย"""
        p = pose.position
        # จุดแรกคลิกโดนตัวหุ่นเอง = น่าจะพลาด -- เป้าชิดหุ่นทำให้หุ่นหันกลับหลังไปเดินไม่กี่ cm (23 ก.ย.)
        robot = self.robot_pose()
        if not self.waypoints and robot is not None and math.hypot(
                p.x - robot.pose.position.x, p.y - robot.pose.position.y) < self.min_goal_dist:
            return f'ทับตัวหุ่นเอง (ห่างไม่ถึง {self.min_goal_dist:.2f} m)', 'ON ROBOT'
        if self.cost_msg is None:
            return None
        g = Grid.from_msg(self.cost_msg)
        yaw = yaw_of(pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        # costmap แบบ OccupancyGrid: 100 = สิ่งกีดขวาง, 99 = ชิดจนตัวหุ่น (วงใน) แตะ
        for fx, fy in self.fp_samples:
            if g.value(p.x + c * fx - s * fy, p.y + s * fx + c * fy, default=0) >= 100:
                return 'ตัวหุ่นทับกำแพง/สิ่งกีดขวาง', 'HITS WALL'
        if g.value(p.x, p.y, default=0) >= 99:
            return 'ชิดกำแพงเกินไป', 'TOO CLOSE TO WALL'
        return None

    # ------------------------------------------------------------------
    # service
    # ------------------------------------------------------------------
    def _precheck(self, res):
        """ตรวจก่อนออกวิ่ง คืน PoseStamped ของหุ่น หรือ None (พร้อมเขียน res.message)"""
        if self.running:
            res.message = 'กำลังวิ่งอยู่แล้ว -- cancel ก่อน'
            return None
        if not self.nav.server_is_ready():
            res.message = 'Nav2 ยังไม่พร้อม (ไม่มี action navigate_to_pose) -- เปิด navigation.launch.py ก่อน'
            return None
        home = self.robot_pose()
        if home is None:
            res.message = f'หาตำแหน่งหุ่นไม่ได้ (TF {self.map_frame} -> {self.base_frame})'
        return home

    def _begin(self, mode, home, first_step):
        self.mode = mode
        self.home = home
        self.home_status = PENDING
        self.running = True
        self.cancel_requested = False
        self.geo_tripped = False
        self.run_id += 1
        self.results = {DONE: 0, FAILED: 0}
        for c in self.clear_clients:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())
        self.publish_markers()
        # รอ costmap อัปเดตหลังล้างก่อนส่งจุดแรก (global costmap update 1 Hz)
        run = self.run_id
        timer = None

        def fire():
            timer.cancel()
            self.destroy_timer(timer)
            if run == self.run_id and self.running and not self.cancel_requested:
                first_step()
        timer = self.create_timer(1.5, fire)

    def srv_start(self, _req, res):
        res.success = False
        if not self.waypoints:
            res.message = 'ยังไม่มีจุด -- คลิก Publish 2D pose บนแมพก่อน'
            return res
        home = self._precheck(res)
        if home is None:
            return res
        self.status = [PENDING] * len(self.waypoints)
        self.step = 0
        hp = home.pose.position
        self.get_logger().info(
            f'เริ่มวิ่ง {len(self.waypoints)} จุด'
            + (f' แล้วกลับจุดเริ่มต้น x={hp.x:.2f} y={hp.y:.2f}' if self.return_home else ''))
        self._begin('route', home, self.route_next)
        res.success = True
        res.message = f'เริ่มวิ่ง {len(self.waypoints)} จุด' + (' แล้วกลับจุดเริ่มต้น' if self.return_home else '')
        return res

    def srv_explore(self, _req, res):
        res.success = False
        if self.map_msg is None:
            res.message = 'ยังไม่ได้รับ /map'
            return res
        home = self._precheck(res)
        if home is None:
            return res
        self.ex_start = time.monotonic()
        self.ex_goals = 0
        self.ex_blacklist = [(m[0], m[1]) for m in self.stuck_marks]   # ที่เคยติด = ไม่ตั้งเป้าตรงนั้น
        self.ex_target = None
        self.ex_cum_yaw = 0.0
        self.ex_last_yaw = None
        self.ex_wander_until = None
        hp = home.pose.position
        self.get_logger().info(
            f'เริ่มสำรวจจาก x={hp.x:.2f} y={hp.y:.2f}  รัศมีไม่เกิน {self.ex_radius:.1f} m '
            f'ไม่เกิน {self.ex_duration / 60:.0f} นาที / {self.ex_max_goals} เป้าหมาย')
        self._begin('explore', home, self.explore_next)
        res.success = True
        res.message = (f'เริ่มสำรวจ รัศมี {self.ex_radius:.1f} m ไม่เกิน {self.ex_duration / 60:.0f} นาที '
                       f'แล้วกลับจุดเริ่มต้น')
        return res

    def srv_explore_preview(self, _req, res):
        pose = self.robot_pose()
        if pose is None or self.map_msg is None:
            res.success = False
            res.message = 'ยังไม่มีแมพหรือหาตำแหน่งหุ่นไม่ได้'
            return res
        center = self.home if (self.running and self.home is not None) else pose
        best, n = self.compute_frontier(pose, center)
        res.success = True
        if best is None:
            res.message = f'เจอขอบแมพ {n} กลุ่ม แต่ไม่มีจุดที่ไปยืนได้ปลอดภัยในรัศมี {self.ex_radius:.1f} m'
        else:
            (x, y), _f, _s = best
            d = math.hypot(x - pose.pose.position.x, y - pose.pose.position.y)
            res.message = f'เจอขอบแมพ {n} กลุ่ม  เป้าหมายถัดไป x={x:.2f} y={y:.2f} ห่าง {d:.1f} m (หุ่นไม่ขยับ)'
        self.get_logger().info(res.message)
        return res

    def srv_cancel(self, _req, res):
        if not self.running:
            res.success = True
            res.message = 'ไม่ได้วิ่งอยู่'
            return res
        self.cancel_requested = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.get_logger().warn('ยกเลิกการวิ่ง')
        self.finish('ยกเลิกแล้ว')
        res.success = True
        res.message = 'ยกเลิกแล้ว หุ่นหยุด'
        return res

    def srv_clear(self, _req, res):
        if self.running:
            res.success = False
            res.message = 'กำลังวิ่งอยู่ -- cancel ก่อน'
            return res
        n = len(self.waypoints)
        self.waypoints, self.status = [], []
        self.home, self.ex_frontiers, self.ex_target, self.ex_blacklist = None, [], None, []
        self.rejected = None
        self.notices = {}
        self.publish_markers()
        self.get_logger().info(f'ลบจุดทั้งหมด ({n} จุด)')
        res.success = True
        res.message = f'ลบจุดทั้งหมดแล้ว ({n} จุด)'
        return res

    def srv_undo(self, _req, res):
        if self.running:
            res.success = False
            res.message = 'กำลังวิ่งอยู่ -- cancel ก่อน'
            return res
        if not self.waypoints:
            res.success = False
            res.message = 'ไม่มีจุดให้ลบ'
            return res
        self.waypoints.pop()
        self.notices.pop(len(self.waypoints), None)
        self.status = [PENDING] * len(self.waypoints)
        self.publish_markers()
        res.success = True
        res.message = f'ลบจุดล่าสุดแล้ว เหลือ {len(self.waypoints)} จุด'
        self.get_logger().info(res.message)
        return res

    # ------------------------------------------------------------------
    # โหมดวิ่งตามจุด
    # ------------------------------------------------------------------
    def route_next(self):
        if self.step >= len(self.waypoints):
            self.go_home(f'ครบ {len(self.waypoints)} จุด')
            return
        i = self.step
        self.set_status(i, ACTIVE)
        self.send_goal(f'จุดที่ {i + 1} ({i + 1}/{len(self.waypoints)})', self.waypoints[i],
                       lambda ok: self.route_done(i, ok))

    def route_done(self, i, ok):
        self.set_status(i, DONE if ok else FAILED)
        if not ok:
            self.get_logger().warn(f'จุดที่ {i + 1}: ไปไม่ถึง -- ข้ามไปจุดถัดไป')
        self.step += 1
        self.route_next()

    # ------------------------------------------------------------------
    # โหมดสำรวจ
    # ------------------------------------------------------------------
    def compute_frontier(self, robot, center):
        grid = Grid.from_msg(self.map_msg)
        cost = Grid.from_msg(self.cost_msg) if self.cost_msg is not None else None
        frontiers = find_frontiers(grid, min_size_m=self.ex_min_size)
        rp, cp = robot.pose.position, center.pose.position
        best = choose_target(frontiers, grid, cost, (rp.x, rp.y), (cp.x, cp.y), self.ex_blacklist,
                             max_radius=self.ex_radius - self.ex_goal_margin, max_cost=self.ex_max_cost)
        # เก็บไว้วาดบนแมพ: เฉพาะขอบในรัศมี เรียงใหญ่ก่อน ไม่เกิน 40 กลุ่ม
        near = [f for f in frontiers
                if math.hypot(f['centroid'][0] - cp.x, f['centroid'][1] - cp.y) <= self.ex_radius + 1.0]
        self.ex_frontiers = [f['centroid'] for f in sorted(near, key=lambda f: -f['size_m'])[:40]]
        self.ex_target = best[0] if best else None
        self.publish_markers()
        return best, len(frontiers)

    def explore_next(self):
        elapsed = time.monotonic() - self.ex_start
        if elapsed > self.ex_duration:
            self.go_home(f'ครบเวลาสำรวจ {self.ex_duration / 60:.0f} นาที')
            return
        if self.ex_goals >= self.ex_max_goals:
            self.go_home(f'ครบ {self.ex_max_goals} เป้าหมายสำรวจ')
            return
        robot = self.robot_pose()
        if robot is None:
            self.go_home('หาตำแหน่งหุ่นไม่ได้ระหว่างสำรวจ')
            return
        best, n = self.compute_frontier(robot, self.home)
        if best is None:
            self.wander_next(robot, n)
            return
        (x, y), f, score = best
        rp = robot.pose.position
        yaw = math.atan2(y - rp.y, x - rp.x)          # หันหน้าไปทางขอบ กล้องจะได้เห็นด้วย
        self.ex_goals += 1
        self.get_logger().info(
            f'สำรวจเป้าหมายที่ {self.ex_goals}: x={x:.2f} y={y:.2f} ห่าง {math.hypot(x - rp.x, y - rp.y):.1f} m '
            f'ขอบยาว {f["size_m"]:.1f} m (เหลือขอบ {n} กลุ่ม)')
        # ขาออกเดินหน้าเสมอ (ถอยออกไปแล้วต้องหมุนกลับตัวที่ปลายทาง เสียการหมุนฟรี -- เจอ 23 ก.ย.)
        self.drive_to(f'สำรวจเป้าหมายที่ {self.ex_goals}', pose_xy_yaw(self.map_frame, x, y, yaw),
                      lambda ok: self.explore_done((x, y), ok), allow_reverse=False)

    def wander_next(self, robot, n_frontiers):
        """ขอบแมพในรัศมีหมดแล้ว: เดินสุ่ม -- ออกไปจุดว่าง (เดินหน้า) แล้วถอยกลับจุดเริ่ม สลับกันจนครบ explore_wander_s"""
        now = time.monotonic()
        if self.ex_wander_until is None:
            if self.wander_s <= 0:
                self.go_home(f'สำรวจครบในรัศมี {self.ex_radius:.1f} m แล้ว (ขอบที่เหลือ {n_frontiers} กลุ่มไปยืนไม่ได้/ไกลเกิน)')
                return
            self.ex_wander_until = now + self.wander_s
            self.get_logger().info(f'ขอบแมพในรัศมีหมดแล้ว (ขอบที่เหลือ {n_frontiers} กลุ่มไปยืนไม่ได้/ไกลเกิน) '
                                   f'-- เดินสุ่มต่อ {self.wander_s:.0f} s')
        if now > self.ex_wander_until:
            self.go_home(f'ครบเวลาเดินสุ่ม {self.wander_s:.0f} s')
            return
        rp, hp = robot.pose.position, self.home.pose.position
        if math.hypot(rp.x - hp.x, rp.y - hp.y) > 0.2:
            # กลับจุดเริ่มก่อน (ปกติคือถอย) หันทิศที่หันอยู่ ไม่หมุนเพิ่ม
            back = pose_xy_yaw(self.map_frame, hp.x, hp.y, yaw_of(robot.pose.orientation))
            self.drive_to('จุดเริ่มต้น (พัก)', back, lambda ok: self.explore_next())
            return
        target = self.pick_wander(robot)
        if target is None:
            self.go_home('หาจุดว่างให้เดินสุ่มในรัศมีไม่ได้')
            return
        self.ex_goals += 1
        p = target.pose.position
        self.ex_target = (p.x, p.y)
        self.publish_markers()
        self.get_logger().info(f'เดินสุ่มเป้าหมายที่ {self.ex_goals}: x={p.x:.2f} y={p.y:.2f} '
                               f'ห่างจุดเริ่ม {math.hypot(p.x - hp.x, p.y - hp.y):.2f} m')
        self.drive_to(f'เดินสุ่มเป้าหมายที่ {self.ex_goals}', target, lambda ok: self.explore_next(),
                      allow_reverse=False)

    def pick_wander(self, robot):
        """จุดว่างสุ่มในรัศมี (explore_max_radius - goal_margin) จากจุดเริ่ม ทิศไม่เกิน +-wander_max_angle
        จากทิศตอนเริ่ม ห่างหุ่นอย่างน้อย wander_min_dist เป็นที่ว่างที่รู้แล้วใน /map ค่า costmap ไม่เกิน
        frontier_max_cost และตัวหุ่นไม่ทับสิ่งกีดขวาง -- หันหน้าออกจากจุดเริ่ม (ขากลับจะได้ถอยตรง)"""
        hp, rp = self.home.pose.position, robot.pose.position
        h_yaw = yaw_of(self.home.pose.orientation)
        r_yaw = yaw_of(robot.pose.orientation)
        rmax = self.ex_radius - self.ex_goal_margin
        grid = Grid.from_msg(self.map_msg) if self.map_msg is not None else None
        cost = Grid.from_msg(self.cost_msg) if self.cost_msg is not None else None
        for i in range(400):
            a = h_yaw + random.uniform(-self.wander_max_angle, self.wander_max_angle)
            turn = abs(math.atan2(math.sin(a - r_yaw), math.cos(a - r_yaw)))
            if i < 300 and turn > self.wander_turn_max:
                continue                  # 300 รอบแรกเลือกเฉพาะทิศที่ไม่ต้องหมุนตัวเยอะ
            r = random.uniform(self.wander_min_dist, rmax)
            x, y = hp.x + r * math.cos(a), hp.y + r * math.sin(a)
            if math.hypot(x - rp.x, y - rp.y) < self.wander_min_dist:
                continue
            if any(seg_dist(sx, sy, rp.x, rp.y, x, y) < self.stall_avoid_r for sx, sy in self.stall_points):
                continue                  # ทางตรงไปจุดนี้ผ่านจุดที่ล้อเคยติด
            # คลายการหมุนสะสม (24 ก.ย. หลายรอบจบเพราะหมุนสะสมเกิน 330° -- หมุนทางเดียวตลอด)
            # หมุนสะสมเกิน unwind_after_deg แล้ว 300 รอบแรกเลือกเฉพาะจุดที่ต้องหมุนทางตรงข้าม (หรือแทบไม่ต้องหมุน)
            if i < 300 and abs(self.ex_cum_yaw) > self.unwind_after:
                b = math.atan2(y - rp.y, x - rp.x) - r_yaw
                b = math.atan2(math.sin(b), math.cos(b))
                if abs(b) > math.radians(15) and (b > 0) == (self.ex_cum_yaw > 0):
                    continue
            if grid is not None and grid.value(x, y) != 0:
                continue
            if cost is not None and cost.value(x, y, default=100) > self.ex_max_cost:
                continue
            pose = pose_xy_yaw(self.map_frame, x, y, a)
            if self.blocked_reason(pose.pose):
                continue
            return pose
        return None

    # ------------------------------------------------------------------
    # ไปยังเป้า: ถ้าเป้าอยู่ข้างหลังให้ถอยตรงก่อน
    # ------------------------------------------------------------------
    def drive_to(self, name, pose, on_done, allow_reverse=True):
        """เป้าอยู่ข้างหลัง (มุมจากหน้าหุ่น >= reverse_min_angle) และไม่ไกลเกิน reverse_max_dist:
        ถอยตรงตามแนวหลังหุ่นด้วย BackUp ก่อน แล้ว NavigateToPose เก็บส่วนที่เหลือ (ปกติเหลือน้อย ไม่ต้องกลับหลังหัน)
        ถอยไม่ได้ (ข้างหลังมีของ) รอ backup_retry_s แล้วลองอีกครั้ง ยังไม่ได้ค่อยไปแบบปกติ (หมุนตัวเดินหน้า)"""
        robot = self.robot_pose()
        if allow_reverse and robot is not None and self.backup.server_is_ready():
            rp, tp = robot.pose.position, pose.pose.position
            dx, dy = tp.x - rp.x, tp.y - rp.y
            d = math.hypot(dx, dy)
            rel = math.atan2(dy, dx) - yaw_of(robot.pose.orientation)
            rel = abs(math.atan2(math.sin(rel), math.cos(rel)))
            if 0.15 < d <= self.rev_max_dist and rel >= self.rev_min_angle:
                dist = -d * math.cos(rel)

                def after_retry(ok):
                    if ok:
                        self.send_goal(name, pose, on_done)
                    else:
                        self.veer(name, lambda: self.send_goal(name, self.facing_arrival(pose), on_done))

                def after_first(ok):
                    if ok:
                        self.send_goal(name, pose, on_done)
                    else:
                        self.later(self.backup_retry_s, lambda: self.retry_backup(name, pose, after_retry))
                self.send_backup(name, dist, after_first)
                return
        self.turn_then_goal(name, pose, on_done, robot)

    def veer(self, name, then):
        """เดินหน้า/ถอยไม่ได้: เบี่ยงตัวออกซ้าย 45° ถ้าไม่ได้ลองขวา 45° แล้วค่อยทำต่อ (then)
        24 ก.ย. ผู้ใช้: "ถ้าถอยหรือเดินหน้าไม่ได้ให้เบี่ยงออกซ้ายขวา" -- Spin ของ behavior_server เช็คการชนเอง"""
        if not self.running or not self.spin.server_is_ready():
            then()
            return
        run = self.run_id

        def attempt(angle, fallback):
            self.get_logger().info(f'{name}: เดินหน้า/ถอยไม่ได้ -- เบี่ยง {math.degrees(angle):+.0f}°')
            goal = Spin.Goal()
            goal.target_yaw = float(angle)
            goal.time_allowance = Duration(sec=10)

            def on_resp(f):
                h = f.result()
                if run != self.run_id or not self.running or self.cancel_requested:
                    if h is not None and h.accepted:
                        h.cancel_goal_async()
                    return
                if h is None or not h.accepted:
                    fallback()
                    return
                self.goal_handle = h
                h.get_result_async().add_done_callback(on_result)

            def on_result(f):
                if run != self.run_id or not self.running or self.cancel_requested:
                    return
                self.goal_handle = None
                if f.result().status == GoalStatus.STATUS_SUCCEEDED:
                    then()
                else:
                    fallback()
            self.spin.send_goal_async(goal).add_done_callback(on_resp)

        # ซ้ายก่อนถ้าหมุนสะสมไปทางขวา (คลายการหมุน) ไม่งั้นขวาก่อน
        first = math.radians(45) if self.ex_cum_yaw <= 0 else -math.radians(45)
        attempt(first, lambda: attempt(-first, then))

    def turn_then_goal(self, name, pose, on_done, robot):
        """เป้าอยู่ข้างหลังไกล ๆ (ต้องกลับหลังหัน): หมุนเองด้วย Spin ไปทางที่คลายการหมุนสะสมก่อน แล้วค่อย NavigateToPose
        (24 ก.ย. รอบ 3 m จบเพราะหมุนสะสม 330° -- ขากลับจุดพักกลับหลังหัน 180° ไปทางเดียวกับที่หมุนมาตลอด
        Nav2 เลือกทางสั้นสุดเอง ที่ ~180° เลือกทางไหนก็ได้) หมุนไม่สำเร็จ (มีของใกล้) ก็ไปแบบปกติ"""
        if (robot is None or not self.running or not self.spin.server_is_ready()
                or abs(self.ex_cum_yaw) < math.radians(45)):
            self.send_goal(name, pose, on_done)
            return
        rp, tp = robot.pose.position, pose.pose.position
        if math.hypot(tp.x - rp.x, tp.y - rp.y) < 0.3:
            self.send_goal(name, pose, on_done)
            return
        rel = math.atan2(tp.y - rp.y, tp.x - rp.x) - yaw_of(robot.pose.orientation)
        rel = math.atan2(math.sin(rel), math.cos(rel))
        if abs(rel) < math.radians(100):
            self.send_goal(name, pose, on_done)
            return
        if (rel > 0) == (self.ex_cum_yaw > 0):          # ทางสั้นหมุนซ้ำทางเดิม -> อ้อมอีกทาง
            rel -= math.copysign(2 * math.pi, rel)
        self.get_logger().info(f'-> {name}: หมุน {math.degrees(rel):+.0f}° ก่อนออกเดิน '
                               f'(คลายการหมุนสะสม {math.degrees(self.ex_cum_yaw):+.0f}°)')
        goal = Spin.Goal()
        goal.target_yaw = float(rel)
        goal.time_allowance = Duration(sec=int(abs(rel) / 0.3 + 10))
        run = self.run_id

        def on_resp(f):
            h = f.result()
            if run != self.run_id or not self.running or self.cancel_requested:
                if h is not None and h.accepted:
                    h.cancel_goal_async()
                return
            if h is None or not h.accepted:
                self.send_goal(name, pose, on_done)
                return
            self.goal_handle = h
            h.get_result_async().add_done_callback(on_result)

        def on_result(f):
            if run != self.run_id or not self.running or self.cancel_requested:
                return
            self.goal_handle = None
            if f.result().status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f'{name}: หมุนไม่ครบ (มีของใกล้?) -- ไปแบบปกติ')
            self.send_goal(name, pose, on_done)
        self.spin.send_goal_async(goal).add_done_callback(on_resp)

    def retry_backup(self, name, pose, then):
        """ถอยรอบสองจากตำแหน่งปัจจุบัน (รอบแรกอาจถอยไปได้บางส่วนแล้ว)"""
        robot = self.robot_pose()
        if robot is None:
            then(False)
            return
        rp, tp = robot.pose.position, pose.pose.position
        rel = math.atan2(tp.y - rp.y, tp.x - rp.x) - yaw_of(robot.pose.orientation)
        rel = abs(math.atan2(math.sin(rel), math.cos(rel)))
        d = math.hypot(tp.x - rp.x, tp.y - rp.y)
        if d <= 0.15 or rel < self.rev_min_angle:
            then(True)
            return
        self.get_logger().info(f'{name}: ลองถอยอีกครั้ง')
        self.send_backup(name, -d * math.cos(rel), then)

    def facing_arrival(self, pose):
        """ขากลับจุดพักที่ถอยไม่ได้: ให้ถึงแล้วหันตามทิศที่เดินมา ไม่ต้องหมุนกลับทิศเดิมอีกรอบ
        (จุดเริ่มต้นตอนจบยังหันทิศเดิมตอนเริ่ม -- คลายสายที่หมุนสะสม)"""
        if not self.running or self.home is None or pose is self.home:
            return pose
        robot = self.robot_pose()
        if robot is None:
            return pose
        rp, tp = robot.pose.position, pose.pose.position
        return pose_xy_yaw(self.map_frame, tp.x, tp.y, math.atan2(tp.y - rp.y, tp.x - rp.x))

    def later(self, delay, fn):
        """เรียก fn ครั้งเดียวหลัง delay วิ (ทิ้งถ้ารอบวิ่งเปลี่ยนไปแล้ว)"""
        run = self.run_id
        timer = None

        def fire():
            timer.cancel()
            self.destroy_timer(timer)
            if run == self.run_id and self.running and not self.cancel_requested:
                fn()
        timer = self.create_timer(delay, fire)

    def send_backup(self, name, dist, then, why='เป้าอยู่ข้างหลัง ถอยตรง'):
        if self.cancel_requested or not self.running:
            return
        goal = BackUp.Goal()
        goal.target.x = float(dist)                      # ระยะถอย (บวก) -- behavior_server เช็คการชนเอง
        goal.speed = float(self.rev_speed)
        goal.time_allowance = Duration(sec=int(dist / self.rev_speed * 2.0 + 5.0))
        self.get_logger().info(f'-> {name}: {why} {dist:.2f} m ก่อน (ไม่กลับหลังหัน)')
        run = self.run_id
        self.backup.send_goal_async(goal).add_done_callback(
            lambda f: self.on_backup_response(f, run, name, then))

    def on_backup_response(self, future, run, name, then):
        try:
            handle = future.result()
        except Exception:                               # noqa: BLE001
            handle = None
        if run != self.run_id or self.cancel_requested or not self.running:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'{name}: behavior_server ไม่รับคำสั่งถอย -- ไปแบบปกติ')
            then(False)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_backup_result(f, run, name, then))

    def on_backup_result(self, future, run, name, then):
        if run != self.run_id or self.cancel_requested or not self.running:
            return
        self.goal_handle = None
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'{name}: ถอยไม่ครบ (status {status}) ข้างหลังอาจมีสิ่งกีดขวาง -- ไปแบบปกติ')
        then(status == GoalStatus.STATUS_SUCCEEDED)

    def explore_done(self, xy, ok):
        if not ok:
            self.ex_blacklist.append(xy)
            self.get_logger().warn(f'ไปขอบ x={xy[0]:.2f} y={xy[1]:.2f} ไม่ถึง -- จำไว้ไม่ไปซ้ำ')
        self.explore_next()

    def guard_tick(self):
        """ระหว่างสำรวจ: รั้วรัศมีจากจุดเริ่ม + หมุนสะสม -- เกินแล้วยกเลิกเป้าและกลับจุดเริ่มทันที
        (ทุกโหมด: เช็คล้อติดก่อน)"""
        self.check_stall()
        self.publish_stuck()
        if not self.running or self.mode != 'explore' or self.geo_tripped or self.home is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return
        t = tf.transform
        hp = self.home.pose.position
        d = math.hypot(t.translation.x - hp.x, t.translation.y - hp.y)
        yaw = yaw_of(t.rotation)
        if self.ex_last_yaw is not None:
            dy = yaw - self.ex_last_yaw
            self.ex_cum_yaw += math.atan2(math.sin(dy), math.cos(dy))
        self.ex_last_yaw = yaw
        if d > self.ex_radius:
            self.abort_to_home(f'หุ่นออกนอกรัศมี {self.ex_radius:.1f} m (ห่างจุดเริ่ม {d:.2f} m) '
                               f'-- เลิกเป้านี้ กลับจุดเริ่ม')
        elif abs(math.degrees(self.ex_cum_yaw)) > self.ex_max_turn:
            self.abort_to_home(f'หมุนสะสม {math.degrees(self.ex_cum_yaw):.0f} องศา เกิน {self.ex_max_turn:.0f} '
                               f'-- สายอาจพันตัวหุ่น เลิกเป้านี้ กลับจุดเริ่ม')

    # ------------------------------------------------------------------
    # ล้อติด: สั่งเดินหน้าแต่ lidar บอกว่าตัวไม่ไป
    # ------------------------------------------------------------------
    def on_cmd_vel(self, msg):
        now = time.monotonic()
        self.cmd_hist.append((now, msg.linear.x, msg.angular.z))
        while self.cmd_hist and self.cmd_hist[0][0] < now - 5.0:
            self.cmd_hist.popleft()

    def on_rf2o(self, msg):
        now = time.monotonic()
        p = msg.pose.pose.position
        self.rf2o_hist.append((now, p.x, p.y, yaw_of(msg.pose.pose.orientation)))
        while self.rf2o_hist and self.rf2o_hist[0][0] < now - 5.0:
            self.rf2o_hist.popleft()

    def load_stuck(self):
        try:
            with open(self.stuck_file) as f:
                self.stuck_marks = [tuple(m) for m in json.load(f)['marks']]
        except FileNotFoundError:
            return
        except (ValueError, KeyError) as e:
            self.get_logger().error(f'อ่าน {self.stuck_file} ไม่ได้ ({e})')
            return
        self.stall_points = [(m[0], m[1]) for m in self.stuck_marks]
        self.get_logger().info(f'โหลดจุดที่ล้อเคยติด {len(self.stuck_marks)} จุดจาก {self.stuck_file}')

    def save_stuck(self):
        try:
            os.makedirs(os.path.dirname(self.stuck_file), exist_ok=True)
            tmp = self.stuck_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'frame': self.map_frame,
                           'marks': [[round(v, 3) for v in m] for m in self.stuck_marks]}, f)
            os.replace(tmp, self.stuck_file)
        except OSError as e:
            self.get_logger().error(f'เซฟ {self.stuck_file} ไม่ได้: {e}')

    def srv_clear_stuck(self, _req, res):
        n = len(self.stuck_marks)
        self.stuck_marks, self.stall_points = [], []
        self.save_stuck()
        self.publish_markers()
        res.success, res.message = True, f'ลบจุดที่ล้อเคยติด {n} จุดแล้ว'
        return res

    def publish_stuck(self):
        """จุดติดเป็นเส้นขวางทิศที่หุ่นวิ่งตอนติด ส่งเป็น cloud ใน base_link (costmap ตัดตามระยะจาก origin ของ frame)"""
        if not self.stuck_marks:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.map_frame, Time())
        except TransformException:
            return
        t = tf.transform
        c, s_ = math.cos(yaw_of(t.rotation)), math.sin(yaw_of(t.rotation))
        pts = []
        for mx, my, yaw in self.stuck_marks:
            px, py = -math.sin(yaw), math.cos(yaw)          # ตั้งฉากกับทิศวิ่ง
            for k in np.arange(-self.stuck_half, self.stuck_half + 1e-6, 0.05):
                x, y = mx + k * px, my + k * py
                pts.append((t.translation.x + c * x - s_ * y, t.translation.y + s_ * x + c * y, 0.1))
        header = tf.header
        header.frame_id = self.base_frame
        self.stuck_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))

    def check_stall(self):
        """ล้อติด 3 แบบ (24 ก.ย. ผู้ใช้: ล้อข้างเดียวติดสาย อีก 3 ล้อยังหมุน หุ่นพยายามไปต่อแล้วหยุด)
        วัดจาก rf2o (lidar) เทียบกับคำสั่งที่ส่งเข้าฐานในช่วง stall_window_s:
          1. สั่งเดินหน้าแต่ตัวไม่ไป (ไม่ถึง stall_min_ratio)
          2. สั่งหมุนแต่ตัวไม่หมุน (ไม่ถึง stall_min_ratio)
          3. สั่งเดินหน้าแต่ตัวเบี้ยวเกินที่สั่ง > stall_veer_deg = ล้อข้างเดียวติด ตัวเลยหมุนรอบล้อนั้น"""
        if not self.running or self.goal_handle is None or self.cur_goal is None:
            return
        now = time.monotonic()
        t0 = now - self.stall_window
        cmds = [c for c in self.cmd_hist if c[0] >= t0]
        if len(cmds) < 5 or cmds[0][0] > t0 + 0.3:
            return
        odo = [o for o in self.rf2o_hist if o[0] >= t0]
        if len(odo) < 3 or odo[-1][0] - odo[0][0] < self.stall_window * 0.6:
            return
        nxt = [c[0] for c in cmds[1:]] + [now]
        exp_lin = sum(max(c[1], 0.0) * (t2 - c[0]) for c, t2 in zip(cmds, nxt))
        exp_yaw = sum(c[2] * (t2 - c[0]) for c, t2 in zip(cmds, nxt))
        moved = math.hypot(odo[-1][1] - odo[0][1], odo[-1][2] - odo[0][2])
        dyaw = 0.0
        for a, b in zip(odo, odo[1:]):
            d = b[3] - a[3]
            dyaw += math.atan2(math.sin(d), math.cos(d))
        min_lin = self.stall_min_cmd * self.stall_window * 0.6
        if exp_lin >= min_lin and moved < self.stall_min_ratio * exp_lin:
            self.on_stall(f'สั่งเดินหน้า {exp_lin * 100:.0f} cm แต่ lidar วัดได้ขยับ {moved * 100:.0f} cm', True)
        elif abs(exp_yaw) >= self.stall_min_turn and abs(dyaw) < self.stall_min_ratio * abs(exp_yaw):
            self.on_stall(f'สั่งหมุน {math.degrees(exp_yaw):.0f}° แต่ lidar วัดได้หมุน {math.degrees(dyaw):.0f}°', False)
        elif exp_lin >= min_lin and abs(dyaw - exp_yaw) > self.stall_veer:
            self.on_stall(f'สั่งเดินหน้า {exp_lin * 100:.0f} cm หมุน {math.degrees(exp_yaw):.0f}° แต่ตัวหมุนไป '
                          f'{math.degrees(dyaw):.0f}° (ล้อข้างเดียวติด?)', True)

    def on_stall(self, why, mark):
        name, on_done = self.cur_goal
        robot = self.robot_pose()
        if robot is not None and mark:
            p, yaw = robot.pose.position, yaw_of(robot.pose.orientation)
            sp = (p.x + 0.35 * math.cos(yaw), p.y + 0.35 * math.sin(yaw))   # ขอบหน้าหุ่น 0.27 m + เผื่อ
            self.stall_points.append(sp)
            self.stuck_marks.append((sp[0], sp[1], yaw))
            self.save_stuck()
            if self.mode == 'explore':
                self.ex_blacklist.append(sp)
        self.get_logger().warn(
            f'{name}: ล้อติด -- {why} ถอย {self.stall_backup:.2f} m แล้วไปทางอื่น')
        handle = self.goal_handle
        self.goal_handle = None
        self.cur_goal = None
        handle.cancel_goal_async()
        self.run_id += 1          # ทิ้งผล CANCELED ของเป้าเดิม (ไม่งั้น on_result หยุดทั้งรอบ)
        self.cmd_hist.clear()
        self.rf2o_hist.clear()
        if not name.startswith('จุดเริ่มต้น'):
            self.results[FAILED] += 1
        self.publish_markers()
        self.later(0.5, lambda: self.send_backup(
            name, self.stall_backup,
            lambda ok: on_done(False) if ok else self.veer(name, lambda: on_done(False)),
            why='ล้อติด ถอยออกมาก่อน'))

    def abort_to_home(self, reason):
        """ยกเลิกเป้าที่กำลังไปแล้วกลับจุดเริ่ม -- ใช้ run_id ใหม่ ผลของเป้าเดิม (ถูกยกเลิก) จะถูกทิ้ง
        ไม่งั้น on_result เห็น CANCELED แล้วหยุดทั้งรอบโดยไม่กลับบ้าน"""
        self.geo_tripped = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.run_id += 1
        self.ex_target = None
        self.get_logger().warn(reason)
        self.go_home(reason)

    # ------------------------------------------------------------------
    # กลับบ้าน / จบ
    # ------------------------------------------------------------------
    def go_home(self, reason):
        self.get_logger().info(reason)
        if not (self.return_home and self.home is not None):
            self.finish(self.summary(reason))
            return
        self.drive_to('จุดเริ่มต้น', self.home, lambda ok: self.home_done(reason, ok))

    def home_done(self, reason, ok):
        self.home_status = DONE if ok else FAILED
        if not ok:
            self.get_logger().error('กลับจุดเริ่มต้นไม่ถึง -- หุ่นหยุดอยู่กับที่')
        self.finish(self.summary(reason))

    def summary(self, reason):
        home = {DONE: 'กลับถึงจุดเริ่ม', FAILED: 'กลับจุดเริ่มไม่ถึง'}.get(self.home_status, '')
        if self.mode == 'explore':
            return (f'จบสำรวจ: {reason} -- ไปถึง {self.results[DONE]} เป้าหมาย '
                    f'ไม่ถึง {self.results[FAILED]} -- {home}')
        return f'ครบแล้ว สำเร็จ {self.results[DONE]}/{self.results[DONE] + self.results[FAILED]} จุด -- {home}'

    def finish(self, summary):
        self.running = False
        self.goal_handle = None
        self.publish_markers()
        self.get_logger().info(summary)

    # ------------------------------------------------------------------
    # ส่งเป้าหมายให้ Nav2 (ใช้ร่วมทุกโหมด)
    # ------------------------------------------------------------------
    def send_goal(self, name, pose, on_done):
        if self.cancel_requested or not self.running:
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = pose.header.frame_id
        goal.pose.header.stamp = Time().to_msg()      # 0 = ใช้ TF ล่าสุด
        goal.pose.pose = pose.pose
        p = pose.pose.position
        self.get_logger().info(f'-> {name} x={p.x:.2f} y={p.y:.2f}')
        run = self.run_id
        self.nav.send_goal_async(goal).add_done_callback(
            lambda f: self.on_goal_response(f, run, name, on_done))

    def on_goal_response(self, future, run, name, on_done):
        try:
            handle = future.result()
        except Exception:                               # noqa: BLE001
            handle = None
        if run != self.run_id or self.cancel_requested or not self.running:
            # ยกเลิกไปแล้วระหว่างรอ Nav2 ตอบ -- ถ้า Nav2 รับเป้าหมายไปแล้วให้ยกเลิกทิ้งด้วย
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{name}: Nav2 ไม่รับเป้าหมาย')
            self.results[FAILED] += 1
            on_done(False)
            return
        self.goal_handle = handle
        self.cur_goal = (name, on_done)
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, run, name, on_done))

    def on_result(self, future, run, name, on_done):
        if run != self.run_id or self.cancel_requested or not self.running:
            return
        self.goal_handle = None
        self.cur_goal = None
        status = future.result().status
        if status == GoalStatus.STATUS_CANCELED:
            # มีคนยกเลิกจากที่อื่น (เช่นส่งเป้าหมายใหม่ทับ) -- หยุดทั้งรอบ
            self.finish(f'{name}: ถูกยกเลิกจากภายนอก หยุดการวิ่ง')
            return
        ok = status == GoalStatus.STATUS_SUCCEEDED
        if not name.startswith('จุดเริ่มต้น'):
            self.results[DONE if ok else FAILED] += 1
        if ok:
            self.get_logger().info(f'ถึง {name} แล้ว')
        else:
            self.get_logger().warn(f'{name}: ไปไม่ถึง (status {status})')
        on_done(ok)

    # ------------------------------------------------------------------
    def set_status(self, idx, st):
        if idx < len(self.status):
            self.status[idx] = st
        self.publish_markers()

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().error(f'TF {self.map_frame}->{self.base_frame}: {e}')
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.orientation = tf.transform.rotation
        return pose

    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        ids = iter(range(1, 10000))

        def base(ns, mtype):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns = ns
            m.id = next(ids)
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.a = 1.0
            return m

        def rgb(m, c):
            m.color.r, m.color.g, m.color.b = c

        def dot(ns, x, y, size, color, z=0.05, mtype=Marker.CYLINDER):
            m = base(ns, mtype)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.scale.x = m.scale.y = size
            m.scale.z = 0.1 if mtype == Marker.CYLINDER else size
            rgb(m, color)
            arr.markers.append(m)

        def text(ns, x, y, s, color, size=0.22):
            m = base(ns, Marker.TEXT_VIEW_FACING)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.35
            m.scale.z = size
            m.text = s
            rgb(m, color)
            arr.markers.append(m)

        # จุดที่ผู้ใช้กด + เส้นเชื่อม
        line = base('waypoints', Marker.LINE_STRIP)
        line.scale.x = 0.03
        rgb(line, (0.6, 0.6, 0.6))
        line.color.a = 0.8
        show_route = self.mode != 'explore' or not self.running
        if show_route and self.home is not None and self.return_home and self.mode == 'route':
            line.points.append(Point(x=self.home.pose.position.x, y=self.home.pose.position.y, z=0.05))
        for i, wp in enumerate(self.waypoints if show_route else []):
            st = self.status[i] if i < len(self.status) else PENDING
            p = wp.pose.position
            dot('waypoints', p.x, p.y, 0.18, COLORS[st])
            arrow = base('waypoints', Marker.ARROW)     # ทิศที่หุ่นจะหันตอนถึง
            arrow.pose = copy.deepcopy(wp.pose)         # อย่าแก้ z ของจุดจริง
            arrow.pose.position.z = 0.1
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.3, 0.04, 0.04
            rgb(arrow, COLORS[st])
            arr.markers.append(arrow)
            text('waypoints', p.x, p.y, str(i + 1), (1.0, 1.0, 1.0))
            if i in self.notices and st == PENDING:
                m = base('notices', Marker.TEXT_VIEW_FACING)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = p.x, p.y, 0.55
                m.scale.z = 0.10
                m.text = self.notices[i]
                rgb(m, (1.0, 0.55, 0.0))
                arr.markers.append(m)
            line.points.append(Point(x=p.x, y=p.y, z=0.05))
        if show_route and self.home is not None and self.return_home and self.mode == 'route':
            line.points.append(Point(x=self.home.pose.position.x, y=self.home.pose.position.y, z=0.05))
        if len(line.points) >= 2:
            arr.markers.append(line)

        # จุดล่าสุดที่ไม่รับ: กากบาทแดง + เหตุผล (หายเมื่อกดจุดที่วางได้ หรือ clear)
        if self.rejected is not None:
            x, y, why = self.rejected
            cross = base('rejected', Marker.LINE_LIST)
            cross.scale.x = 0.05
            rgb(cross, COLORS[FAILED])
            for d in (0.15, -0.15):
                cross.points += [Point(x=x - 0.15, y=y - d, z=0.08), Point(x=x + 0.15, y=y + d, z=0.08)]
            arr.markers.append(cross)
            text('rejected', x, y, f'X {why}', COLORS[FAILED], 0.12)

        # สำรวจ
        for x, y in self.ex_frontiers:
            dot('frontiers', x, y, 0.10, FRONTIER_COLOR, mtype=Marker.SPHERE)
        for x, y in self.ex_blacklist:
            dot('frontiers', x, y, 0.14, COLORS[FAILED], mtype=Marker.SPHERE)
        for x, y in self.stall_points:                  # จุดที่ล้อเคยติด (สายไฟ/ของเตี้ย)
            dot('stall', x, y, 0.2, (1.0, 0.5, 0.0))
            text('stall', x, y, 'STUCK', (1.0, 0.5, 0.0), 0.12)
        if self.ex_target is not None:
            dot('frontiers', self.ex_target[0], self.ex_target[1], 0.28, FRONTIER_COLOR)
            text('frontiers', self.ex_target[0], self.ex_target[1], 'EXPLORE', FRONTIER_COLOR, 0.18)

        # จุดเริ่มต้น
        if self.home is not None and self.return_home:
            hp = self.home.pose.position
            dot('home', hp.x, hp.y, 0.16, COLORS[DONE] if self.home_status == DONE else HOME_COLOR,
                z=0.08, mtype=Marker.CUBE)
            text('home', hp.x, hp.y, 'START', HOME_COLOR, 0.2)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)      # SingleThreadedExecutor -- ดู memory rclpy-multithreaded-executor-jitter
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
