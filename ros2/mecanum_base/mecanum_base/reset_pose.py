"""reset_pose -- ให้ตำแหน่งที่หุ่นยืนอยู่ตอนนี้เป็นจุด 0 ใหม่ ไม่ต้องรีสตาร์ทระบบ (ไม่ต้อง sudo)

    ros2 run mecanum_base reset_pose        (หรือ bash ~/reset_pose.sh)
    ปุ่ม RESET ใน Foxglove -> service /reset_pose/reset (reset_pose_server รันไฟล์นี้ให้)

ใช้หลังย้าย/หมุนหุ่นด้วยมือตอนระบบรันอยู่ -- ถ้าไม่รีเซ็ต SLAM จะเชื่อตำแหน่งเก่า
แล้ว scan match ผิด (23 ก.ย. หุ่นเลี้ยวขวาเอง 45° เพราะเรื่องนี้)

ลำดับ: ยกเลิกการวิ่ง -> ปิดไดรเวอร์ -> wheel_odom = 0 -> EKF = 0 -> ล้างแมพ SLAM เริ่มใหม่
       -> ล้าง costmap -> ลบจุดที่กดไว้ -> เช็คว่า odom/map ของหุ่นอยู่ที่ 0 จริง
ไดรเวอร์จะปิดอยู่หลังรีเซ็ต ถ้าจะวิ่งต้องเปิดใหม่
exit code 0 = ผ่านทุกขั้น, 1 = มีขั้นไม่ผ่าน (บรรทัดที่ขึ้นต้น "!!")
"""
import math
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from rclpy.time import Time
from robot_localization.srv import SetPose
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener


class Resetter:
    def __init__(self):
        self.n = Node('reset_pose_run')
        self.buf = Buffer()
        TransformListener(self.buf, self.n)
        self.bad = []
        self.slam_restarted = None

    def call(self, srv_type, name, req=None, timeout=10.0):
        c = self.n.create_client(srv_type, name)
        if not c.wait_for_service(timeout_sec=5.0):
            return None
        f = c.call_async(req if req is not None else srv_type.Request())
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=timeout)
        return f.result()

    def step(self, label, ok, detail=''):
        print(f'  {"OK " if ok else "!! "} {label}{("  " + detail) if detail else ""}', flush=True)
        if not ok:
            self.bad.append(label)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def pose(self, parent, newer_than=None):
        """None ถ้ายังไม่มี TF หรือ TF เก่ากว่า newer_than (Time) -- กันอ่าน map->odom ค้างจากก่อนรีเซ็ต"""
        try:
            tf = self.buf.lookup_transform(parent, 'base_link', Time())
        except Exception:
            return None
        if newer_than is not None and Time.from_msg(tf.header.stamp) < newer_than:
            return None
        t = tf.transform
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, math.degrees(yaw)

    def slam_state(self):
        r = self.call(GetState, '/slam_toolbox/get_state')
        return r.current_state.id if r else None

    def restart_slam(self):
        """ล้างแมพด้วย lifecycle deactivate -> cleanup -> configure (launch ของ slam_toolbox activate ให้เอง)

        ไม่ใช้ /slam_toolbox/reset: 2.8.5 ล้างแมพแต่ไม่ลงทะเบียน lidar ใหม่ แล้วไม่รับ scan อีกเลย
        (23 ก.ย. หมุน 90° แล้วกราฟยังมี 1 node แมพค้าง 802 ช่อง) -- cleanup ทำ Unregistering/Registering sensor ครบ
        """
        st = self.slam_state()
        if st is None:
            return False, 'ไม่มี /slam_toolbox/get_state'
        steps = []
        if st == State.PRIMARY_STATE_ACTIVE:
            steps.append((Transition.TRANSITION_DEACTIVATE, 'deactivate'))
        if st in (State.PRIMARY_STATE_ACTIVE, State.PRIMARY_STATE_INACTIVE):
            steps.append((Transition.TRANSITION_CLEANUP, 'cleanup'))
        steps.append((Transition.TRANSITION_CONFIGURE, 'configure'))
        for tid, name in steps:
            req = ChangeState.Request()
            req.transition.id = tid
            r = self.call(ChangeState, '/slam_toolbox/change_state', req, timeout=20.0)
            if not (r and r.success):
                return False, f'{name} ไม่ผ่าน'
        # รอ launch สั่ง activate เอง ถ้าไม่มาใน 5 วิ สั่งเอง
        end = time.time() + 5.0
        while time.time() < end and self.slam_state() != State.PRIMARY_STATE_ACTIVE:
            self.spin(0.3)
        if self.slam_state() != State.PRIMARY_STATE_ACTIVE:
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            r = self.call(ChangeState, '/slam_toolbox/change_state', req, timeout=20.0)
            if not (r and r.success):
                return False, 'activate ไม่ผ่าน'
        self.slam_restarted = self.n.get_clock().now()
        return True, ''

    def run(self):
        print('รีเซ็ตตำแหน่งหุ่น -- ตำแหน่งตอนนี้จะเป็นจุด 0', flush=True)

        r = self.call(Trigger, '/waypoint_manager/cancel')
        self.step('ยกเลิกการวิ่งของ waypoint_manager', r is not None,
                  r.message if r else 'ไม่มี service (Nav2 ไม่ได้เปิด?)')
        # goal_id ศูนย์ + stamp ศูนย์ = ยกเลิกทุกเป้าของ navigate_to_pose (เช่นจาก nav_goal_test.py)
        r = self.call(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.step('ยกเลิกเป้า Nav2 ทั้งหมด', r is not None,
                  f'ยกเลิก {len(r.goals_canceling)} เป้า' if r else 'ไม่มี action')

        r = self.call(SetBool, '/mecanum_base/enable', SetBool.Request(data=False))
        self.step('ปิดไดรเวอร์', bool(r and r.success), r.message if r else 'ไม่มี service')
        self.spin(0.5)

        r = self.call(Trigger, '/mecanum_base/reset_odom')
        self.step('wheel_odom = 0', bool(r and r.success), r.message if r else 'ไม่มี service')

        req = SetPose.Request()
        req.pose = PoseWithCovarianceStamped()
        req.pose.header.frame_id = 'odom'
        req.pose.header.stamp = self.n.get_clock().now().to_msg()
        req.pose.pose.pose.orientation.w = 1.0
        for i in (0, 7, 35):          # x, y, yaw -- ความไม่แน่นอนเล็ก ๆ
            req.pose.pose.covariance[i] = 1e-4
        r = self.call(SetPose, '/set_pose', req)
        self.step('EKF = 0', r is not None, '' if r is not None else 'ไม่มี service /set_pose')
        self.spin(1.0)

        ok, detail = self.restart_slam()
        self.step('ล้างแมพ SLAM เริ่มใหม่', ok, detail)

        for cm in ('global_costmap/clear_entirely_global_costmap',
                   'local_costmap/clear_entirely_local_costmap'):
            r = self.call(ClearEntireCostmap, '/' + cm)
            self.step('ล้าง ' + cm.split('/')[0], r is not None, '' if r is not None else 'ไม่มี service')

        r = self.call(Trigger, '/waypoint_manager/clear')
        self.step('ลบจุดที่กดไว้', r is not None, r.message if r else '')

        # หมุดของที่กล้องเห็นอิงแมพเก่า -- ล้างทิ้งพร้อมแมพ
        r = self.call(Trigger, '/object_locator/clear')
        self.step('ลบหมุดของจากกล้อง', r is not None, r.message if r else 'ไม่มี service')

        # รอ SLAM รับ scan แรกของแมพใหม่แล้วส่ง map->odom ชุดใหม่
        end = time.time() + 8.0
        while time.time() < end and self.pose('map', self.slam_restarted) is None:
            self.spin(0.3)
        self.spin(1.0)
        for parent in ('odom', 'map'):
            p = self.pose(parent, self.slam_restarted)
            if p is None:
                self.step(f'{parent} -> base_link', False, 'ยังไม่มี TF')
                continue
            ok = math.hypot(p[0], p[1]) < 0.05 and abs(p[2]) < 3.0
            self.step(f'{parent} -> base_link', ok, f'x={p[0]:+.3f} y={p[1]:+.3f} หัน {p[2]:+.1f}°')

        print()
        if self.bad:
            print('มีขั้นที่ไม่ผ่าน: ' + ', '.join(self.bad))
            print('ถ้า map ยังไม่เป็น 0 ลองรอ 5 วิแล้วรันซ้ำ -- ถ้ายังไม่ได้ ใช้ sudo systemctl restart ros2-slam')
        else:
            print('เสร็จ -- ตำแหน่งตอนนี้คือ 0 แล้ว (ไดรเวอร์ปิดอยู่ ถ้าจะวิ่งต้องเปิดใหม่)')
        return not self.bad


def main():
    rclpy.init()
    r = Resetter()
    try:
        ok = r.run()
    finally:
        r.n.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
