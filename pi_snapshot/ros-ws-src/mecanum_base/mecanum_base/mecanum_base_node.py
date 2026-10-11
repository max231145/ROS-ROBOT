#!/usr/bin/env python3
"""
mecanum_base -- node ขับฐานล้อ mecanum ผ่านเฟิร์มแวร์ esp32_mecanum_base

  subscribe  cmd_vel      geometry_msgs/Twist  (หรือ TwistStamped ถ้า use_stamped_cmd_vel)
                          ใช้ linear.x, linear.y, angular.z
  publish    wheel_odom   nav_msgs/Odometry    odometry จาก step ที่บอร์ดปล่อยออกไปจริง
             TF odom -> base_link              เฉพาะเมื่อ publish_tf = true
                                               (ตอนนี้ rf2o เป็นเจ้าของ TF ตัวนี้อยู่ ห้ามเปิดซ้อน)

  service    ~/enable       std_srvs/SetBool   เปิด/ปิดไดรเวอร์
             ~/estop        std_srvs/Trigger   หยุดทันที + ตัด ENA
             ~/clear_fault  std_srvs/Trigger
             ~/reset_odom   std_srvs/Trigger   รีเซ็ต pose ของ wheel_odom เป็น 0

ความปลอดภัย: ไม่มี cmd_vel ใหม่เกิน cmd_timeout -> node สั่งศูนย์
             node ตาย / USB หลุด -> เฟิร์มแวร์ไม่ได้รับ V เกิน 300 ms แล้วชะลอหยุดเอง
"""

import math
import signal
import threading
import time

import rclpy
from rclpy.node import Node
# ใช้ SingleThreadedExecutor เท่านั้น: วัดบน Pi 5 แล้ว MultiThreadedExecutor ทำ timer 30 Hz สะดุดถึง 300 ms
# (p99 222-252 ms, หายไป 25% ของจังหวะ) เกิน CMD_TIMEOUT_MS ของเฟิร์มแวร์ หุ่นจะเบรกกระตุกเป็นพัก ๆ
# single-thread นิ่งที่ 33.5 ms  callback ทุกตัวใน node นี้สั้น ไม่มีตัวไหนต้องรอ
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

import serial

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from mecanum_base.kinematics import (
    WHEEL_COUNT, MecanumKinematics, integrate_pose, limit_wheel_rates, step_delta)

DYNAMIC = ParameterDescriptor(dynamic_typing=True)

# บิต sys ต้องตรงกับ SYS_* ในไฟล์ .ino
SYS_ENABLED = 0x01
SYS_FAULT = 0x02
SYS_TIMEOUT = 0x04

# W <ms> <sys> <p0..p3> <v0..v3>
STATUS_FIELDS = 3 + 2 * WHEEL_COUNT

HANDSHAKE_TIMEOUT = 5.0     # s  ESP32 รีเซ็ตตอนเปิดพอร์ต บูต ~1 s แล้วเริ่มส่ง W
RETRY_WRONG_PORT = 10.0     # s  พอร์ตไม่ใช่บอร์ดนี้ รอนาน ๆ ก่อนลองใหม่ กันไปแย่งข้อมูลอุปกรณ์อื่น
TICK_GAP_WARN = 0.15        # s  timer ส่ง V ห่างเกินนี้ให้เตือน (เฟิร์มแวร์เบรกเองที่ 0.3 s)


class MecanumBase(Node):

    def __init__(self):
        super().__init__('mecanum_base')

        self.port = self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58EA045139-if00').value
        self.baud = int(self.num('baud', 115200))
        self.status_hz = int(self.num('status_hz', 30))
        self.cmd_rate = self.num('cmd_rate', 30.0)
        self.cmd_timeout = self.num('cmd_timeout', 0.5)
        use_stamped = self.declare_parameter('use_stamped_cmd_vel', False).value
        self.auto_enable = self.declare_parameter('auto_enable', False).value

        wheel_radius = self.num('wheel_radius', 0.05)
        wheel_base = self.num('wheel_base', 0.20)
        track_width = self.num('track_width', 0.20)
        steps_per_rev = self.num('steps_per_rev', 200)
        microsteps = self.num('microsteps', 8)
        gear_ratio = self.num('gear_ratio', 1.0)
        invert = list(self.declare_parameter('invert_wheels', [False] * WHEEL_COUNT).value)
        # 2026-10-11 ตัวคูณรัศมีล้อจริงรายล้อ (FL FR RL RR): ล้อที่วิ่งได้ระยะน้อยกว่า -> ค่า < 1 -> สั่งหมุนเร็วขึ้นชดเชย
        #   ได้จาก tools/straight_test.py (หุ่นเบี้ยวทั้งที่ล้อนับว่าไม่หัน)
        self.wheel_scale = [float(v) for v in self.declare_parameter('wheel_scale', [1.0] * WHEEL_COUNT).value]

        self.max_vx = self.num('max_vx', 0.3)
        self.max_vy = self.num('max_vy', 0.3)
        self.max_wz = self.num('max_wz', 1.0)
        self.max_wheel_rate = self.num('max_wheel_rate', 3200)
        self.wheel_accel = self.num('wheel_accel', 2000)

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.publish_tf = self.declare_parameter('publish_tf', False).value
        self.pose_cov = list(self.declare_parameter(
            'pose_covariance_diagonal', [0.01, 0.01, 1e6, 1e6, 1e6, 0.05]).value)
        self.twist_cov = list(self.declare_parameter(
            'twist_covariance_diagonal', [0.01, 0.02, 1e6, 1e6, 1e6, 0.05]).value)
        # หมุนอยู่กับที่ ล้อ mecanum ไถลจนนับมุมผิด (23 ก.ย. สั่ง 90° ได้จริง 45°/63°/93°/116° แล้วแต่รอบ)
        # ช่วงนั้นส่ง vyaw covariance ค่านี้แทน ให้ EKF ไปเชื่อ yaw จาก rf2o -- ค่าตั้งต้น = ไม่เปลี่ยนพฤติกรรม
        self.spin_vyaw_cov = self.num('spin_vyaw_covariance', self.twist_cov[5])
        # หมุนอยู่กับที่หุ่นยังกวาดออกข้าง ~20-35 cm (หมุนรอบข้างหนึ่ง) แต่ล้อบอก vx vy = 0 -> ให้เชื่อ rf2o เหมือนกัน
        self.spin_lin_cov = self.num('spin_linear_covariance', self.twist_cov[0])
        self.spin_max_vx = self.num('spin_max_vx', 0.03)      # m/s  เดินหน้าช้ากว่านี้ = ถือว่าหมุนอยู่กับที่
        self.spin_min_wz = self.num('spin_min_wz', 0.05)      # rad/s หมุนเร็วกว่านี้ถึงนับ

        if len(invert) != WHEEL_COUNT:
            raise ValueError(f'invert_wheels ต้องมี {WHEEL_COUNT} ค่า (FL FR RL RR)')
        if len(self.pose_cov) != 6 or len(self.twist_cov) != 6:
            raise ValueError('covariance_diagonal ต้องมี 6 ค่า (x y z roll pitch yaw)')

        self.kin = MecanumKinematics(wheel_radius, wheel_base, track_width)
        self.steps_per_rad = float(steps_per_rev) * float(microsteps) * float(gear_ratio) / (2.0 * math.pi)
        self.sign = [-1 if inv else 1 for inv in invert]
        if len(self.wheel_scale) != WHEEL_COUNT or min(self.wheel_scale) <= 0.5:
            raise ValueError(f'wheel_scale ต้องมี {WHEEL_COUNT} ค่า (FL FR RL RR) และมากกว่า 0.5')

        # ---- สถานะ ----
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_stamp = 0.0              # time.monotonic() ของ cmd_vel ล่าสุด
        self.cmd_stale = True             # ใช้ log ครั้งเดียวตอน cmd_vel ขาดหาย
        self.x = self.y = self.th = 0.0
        self.last_pos = None
        self.last_ms = None
        self.last_rx = 0.0                # time.monotonic() ของบรรทัด W ล่าสุด
        self.last_tick = None             # ข้าม tick แรก ช่วงเปิด node ไม่ใช่การสะดุด
        self.max_tick_gap = 0.0
        self.sys_flags = None

        self.ser = None
        self.ser_lock = threading.Lock()
        self.board_ok = False             # ได้รับบรรทัด W ที่ถูกรูปแบบแล้ว = ใช่บอร์ดนี้จริง
        self.running = True

        self.pub_odom = self.create_publisher(Odometry, 'wheel_odom', 20)
        # อัลตราโซนิกหน้า/หลังจาก ESP32 (บรรทัด "U <หน้า mm> <หลัง mm>", -1 = ไม่มีเสียงสะท้อน) -- 2026-09-25
        self.pub_us = [self.create_publisher(Range, 'ultrasonic/front', 10),
                       self.create_publisher(Range, 'ultrasonic/rear', 10)]
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        if use_stamped:
            self.create_subscription(TwistStamped, 'cmd_vel',
                                     lambda m: self.on_cmd_vel(m.twist), 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.create_service(SetBool, '~/enable', self.srv_enable)
        self.create_service(Trigger, '~/estop', self.srv_estop)
        self.create_service(Trigger, '~/clear_fault', self.srv_clear)
        self.create_service(Trigger, '~/reset_odom', self.srv_reset_odom)

        m_per_step = wheel_radius / self.steps_per_rad
        self.get_logger().info(
            f'ล้อ r={wheel_radius} m  ฐานล้อ {wheel_base} m  ระยะซ้าย-ขวา {track_width} m  '
            f'{self.steps_per_rad * 2 * math.pi:.0f} step/รอบ ({m_per_step * 1000:.3f} mm/step)  '
            f'publish_tf={self.publish_tf}')

        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        self.create_timer(1.0 / max(1.0, self.cmd_rate), self.tick_cmd)

    def num(self, name: str, default) -> float:
        """param ตัวเลขที่ yaml จะเขียน 3200 หรือ 3200.0 ก็ได้ (ปกติ rclpy จะ error ถ้าชนิดไม่ตรง default)"""
        return float(self.declare_parameter(name, default, DYNAMIC).value)

    # ------------------------------------------------------------------ serial
    def open_serial(self) -> bool:
        try:
            # บอร์ด ESP32 ที่ใช้ (ชิป CH9102) ถูกกดขา EN ค้างถ้า DTR กับ RTS ติดทั้งคู่
            # ซึ่งเป็นค่าตั้งต้นของ pyserial ตอนเปิดพอร์ต -> บอร์ดเงียบตลอด (เจอจริงบน Windows)
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout = self.port, self.baud, 0.2
            ser.dtr = False
            ser.rts = False
            ser.open()
        except serial.SerialException as e:
            self.get_logger().error(f'เปิด {self.port} ไม่ได้: {e}', throttle_duration_sec=30.0)
            return False
        with self.ser_lock:
            self.ser = ser
        self.board_ok = False
        self.last_pos = None
        self.get_logger().info(f'เปิด {self.port} @ {self.baud} แล้ว รอสถานะจากบอร์ด')
        return True

    def close_serial(self):
        with self.ser_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
        self.board_ok = False

    def send(self, line: str) -> bool:
        """ส่งได้ก็ต่อเมื่อยืนยันแล้วว่าเป็นบอร์ดนี้ ไม่งั้นอาจยิงคำสั่งใส่ lidar ที่เสียบพอร์ตผิด"""
        with self.ser_lock:
            if self.ser is None or not self.board_ok:
                return False
            try:
                self.ser.write((line + '\n').encode('ascii'))
                return True
            except serial.SerialException as e:
                self.get_logger().error(f'เขียนพอร์ตไม่ได้: {e}')
        self.close_serial()
        return False

    def push_config(self):
        """บอร์ดไม่เก็บค่าข้ามการรีบูต ต้องยัดใหม่ทุกครั้งที่ต่อติด"""
        accel = max(1, int(self.wheel_accel))
        self.send(f'A {max(1, int(self.max_wheel_rate))} {accel}')
        self.send(f'F {int(self.status_hz)}')
        if self.auto_enable:
            self.send('E 1')

    def rx_loop(self):
        buf = b''
        opened_at = 0.0
        while self.running:
            if self.ser is None:
                if not self.open_serial():
                    time.sleep(2.0)
                    continue
                buf = b''
                opened_at = time.monotonic()

            try:
                # อ่านเท่าที่มีอยู่ (รออย่างน้อย 1 byte) ห้ามใช้ read(256): pyserial จะรอจนครบ 256 byte
                # หรือหมด timeout ได้ W ทีละ ~5 บรรทัดทุก ~170 ms -> wheel_odom ออกเป็นก้อน stamp แทบเท่ากัน
                # EKF (robot_localization) เลยเดินเวลาแค่ ~5 ครั้ง/วินาที ทั้งที่เฉลี่ยยัง 30 Hz
                chunk = self.ser.read(max(1, self.ser.in_waiting))
            except (serial.SerialException, OSError, TypeError, AttributeError) as e:
                if not self.running:
                    break                      # shutdown() ปิดพอร์ตระหว่างอ่าน ไม่ใช่ error
                self.get_logger().error(f'อ่านพอร์ตไม่ได้: {e}')
                self.close_serial()
                time.sleep(1.0)
                continue

            if chunk:
                buf += chunk
                while b'\n' in buf:
                    raw, buf = buf.split(b'\n', 1)
                    try:
                        self.handle_line(raw.decode('ascii', 'ignore').strip())
                    except Exception as e:
                        # ห้ามปล่อยให้ thread นี้ตาย ไม่งั้น wheel_odom หยุดเงียบ ๆ ทั้งที่ยังขับได้
                        if not self.running or not rclpy.ok():
                            return             # Ctrl+C: rclpy ปิด context ก่อน shutdown() จะทำงาน
                        self.get_logger().error(f'จัดการข้อมูลจากบอร์ดไม่ได้: {e!r}',
                                                throttle_duration_sec=5.0)
                if len(buf) > 4096:
                    buf = b''                  # ไม่มีขึ้นบรรทัดเลย = ข้อมูล binary ไม่ใช่บอร์ดนี้

            if not self.board_ok and time.monotonic() - opened_at > HANDSHAKE_TIMEOUT:
                self.get_logger().error(
                    f'{self.port} ไม่ส่งสถานะ W มาใน {HANDSHAKE_TIMEOUT:.0f} s -- '
                    f'พอร์ตผิด (อาจเป็น lidar) หรือยังไม่ได้แฟลช esp32_mecanum_base '
                    f'จะลองใหม่ใน {RETRY_WRONG_PORT:.0f} s')
                self.close_serial()
                time.sleep(RETRY_WRONG_PORT)

    def handle_line(self, line: str):
        if not line:
            return
        if line.startswith('#'):
            self.get_logger().info(f'esp32: {line[1:].strip()}')
            return
        if line.startswith('ERR'):
            self.get_logger().warning(f'esp32: {line}')
            return
        if line.startswith('U '):
            f = line.split()
            if len(f) == 3:
                stamp = self.get_clock().now().to_msg()
                for k, (pub, frame) in enumerate(zip(self.pub_us, ('us_front', 'us_rear'))):
                    try:
                        mm = int(f[1 + k])
                    except ValueError:
                        continue
                    m = Range()
                    m.header.stamp = stamp
                    m.header.frame_id = frame
                    m.radiation_type = Range.ULTRASOUND
                    m.field_of_view = 0.26          # ~15°
                    m.min_range, m.max_range = 0.02, 4.0
                    m.range = float('inf') if mm < 0 else mm / 1000.0
                    pub.publish(m)
            return
        if not line.startswith('W '):
            return

        f = line.split()
        if len(f) != STATUS_FIELDS:
            return
        try:
            vals = [int(v) for v in f[1:]]
        except ValueError:
            return
        ms, sysf = vals[0], vals[1]
        pos = vals[2:2 + WHEEL_COUNT]
        vel = vals[2 + WHEEL_COUNT:]
        self.last_rx = time.monotonic()

        if not self.board_ok:
            self.board_ok = True
            self.get_logger().info('ต่อกับบอร์ด esp32_mecanum_base แล้ว')
            self.push_config()
        elif self.last_ms is not None and ms < self.last_ms:
            # millis() ย้อนกลับ = บอร์ดรีบูต (ไฟตกตอนมอเตอร์กินกระแส / กดปุ่ม EN)
            self.get_logger().warning('บอร์ดรีบูต -- ยัดค่าใหม่ ตัวนับ step เริ่มจาก 0')
            self.last_pos = None
            self.push_config()

        self.report_sys_change(sysf)
        self.update_odom(ms, pos, vel)

    def report_sys_change(self, sysf: int):
        prev = self.sys_flags
        self.sys_flags = sysf
        if prev is None:
            prev = ~sysf
        log = self.get_logger()
        if (sysf ^ prev) & SYS_ENABLED:
            log.info('ไดรเวอร์เปิดแล้ว' if sysf & SYS_ENABLED else 'ไดรเวอร์ปิดอยู่')
        if (sysf ^ prev) & SYS_FAULT and sysf & SYS_FAULT:
            log.error('บอร์ดอยู่ในสถานะ FAULT (E-stop) -- เรียก ~/clear_fault แล้ว ~/enable true')

    # ---------------------------------------------------------------- odometry
    def update_odom(self, ms: int, pos, vel):
        if self.last_pos is None:
            self.last_pos, self.last_ms = pos, ms
            return

        # มุมที่แต่ละล้อหมุนไป (rad) -> ระยะที่หุ่นเคลื่อนในกรอบตัวเอง
        d_ang = [self.sign[i] * step_delta(pos[i], self.last_pos[i]) / self.steps_per_rad * self.wheel_scale[i]
                 for i in range(WHEEL_COUNT)]
        dx, dy, dth = self.kin.forward(d_ang)
        self.last_pos, self.last_ms = pos, ms

        # ความเร็วใช้ค่าที่บอร์ดกำลังปั่นอยู่ ไม่หารระยะด้วยเวลา เพราะ millis() ละเอียดแค่ 1 ms
        w = [self.sign[i] * vel[i] / self.steps_per_rad * self.wheel_scale[i] for i in range(WHEEL_COUNT)]
        vx, vy, wz = self.kin.forward(w)

        with self.lock:
            self.x, self.y, self.th = integrate_pose(self.x, self.y, self.th, dx, dy, dth)
            x, y, th = self.x, self.y, self.th

        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(th / 2.0), math.cos(th / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        for i in range(6):
            odom.pose.covariance[i * 7] = float(self.pose_cov[i])
            odom.twist.covariance[i * 7] = float(self.twist_cov[i])
        if abs(wz) > self.spin_min_wz and abs(vx) < self.spin_max_vx:
            odom.twist.covariance[35] = float(self.spin_vyaw_cov)
            odom.twist.covariance[0] = float(max(self.spin_lin_cov, self.twist_cov[0]))
            odom.twist.covariance[7] = float(max(self.spin_lin_cov, self.twist_cov[1]))
        self.pub_odom.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------ cmd_vel
    def on_cmd_vel(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            self.get_logger().warning('cmd_vel มีค่า NaN/inf -- ทิ้ง', throttle_duration_sec=2.0)
            return
        vx = min(max(vx, -self.max_vx), self.max_vx)
        vy = min(max(vy, -self.max_vy), self.max_vy)
        wz = min(max(wz, -self.max_wz), self.max_wz)
        with self.lock:
            self.cmd = (vx, vy, wz)
            self.cmd_stamp = time.monotonic()

    def tick_cmd(self):
        """ส่ง V ทุกคาบแม้ค่าเป็นศูนย์ เป็น heartbeat ให้เฟิร์มแวร์ไปในตัว"""
        now = time.monotonic()
        gap = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        self.max_tick_gap = max(self.max_tick_gap, gap)
        if self.board_ok and gap > TICK_GAP_WARN:
            self.get_logger().warning(
                f'timer ส่งคำสั่งสะดุด {gap * 1000:.0f} ms (ควรเป็น {1000 / self.cmd_rate:.0f} ms) '
                f'-- เกิน 300 ms บอร์ดจะเบรกเองชั่วคราว', throttle_duration_sec=2.0)

        with self.lock:
            vx, vy, wz = self.cmd
            age = now - self.cmd_stamp

        if age > self.cmd_timeout:
            if not self.cmd_stale and any((vx, vy, wz)):
                self.get_logger().warning(f'ไม่มี cmd_vel ใหม่เกิน {self.cmd_timeout} s -- สั่งหยุด')
            self.cmd_stale = True
            vx = vy = wz = 0.0
        else:
            self.cmd_stale = False

        rates = [self.sign[i] * w * self.steps_per_rad / self.wheel_scale[i]
                 for i, w in enumerate(self.kin.inverse(vx, vy, wz))]
        rates = limit_wheel_rates(rates, self.max_wheel_rate)
        self.send('V ' + ' '.join(str(int(round(r))) for r in rates))

        rx_age = time.monotonic() - self.last_rx
        if self.board_ok and rx_age > 1.0:
            self.get_logger().warning(f'บอร์ดเงียบไป {rx_age:.1f} s -- wheel_odom ไม่อัปเดต',
                                      throttle_duration_sec=5.0)

    # --------------------------------------------------------------- services
    def srv_enable(self, req, res):
        res.success = self.send('E 1' if req.data else 'E 0')
        if res.success:
            res.message = 'เปิดไดรเวอร์แล้ว' if req.data else 'ปิดไดรเวอร์แล้ว'
        else:
            res.message = 'ส่งคำสั่งไม่ได้ -- ยังไม่ได้ต่อกับบอร์ด'
        return res

    def srv_estop(self, req, res):
        res.success = self.send('X')
        res.message = ('E-stop: หยุดทันทีและตัด ENA แล้ว ต้องเรียก clear_fault แล้ว enable ก่อนใช้ต่อ'
                       if res.success else 'ส่งคำสั่งไม่ได้ -- ยังไม่ได้ต่อกับบอร์ด')
        return res

    def srv_clear(self, req, res):
        res.success = self.send('C')
        res.message = ('เคลียร์ fault แล้ว ไดรเวอร์ยังปิดอยู่ เรียก ~/enable true เพื่อใช้ต่อ'
                       if res.success else 'ส่งคำสั่งไม่ได้ -- ยังไม่ได้ต่อกับบอร์ด')
        return res

    def srv_reset_odom(self, req, res):
        with self.lock:
            self.x = self.y = self.th = 0.0
        res.success = True
        res.message = 'รีเซ็ต pose ของ wheel_odom เป็น 0 แล้ว'
        return res

    # ----------------------------------------------------------------- cleanup
    def shutdown(self):
        self.running = False
        print(f'[mecanum_base] timer ส่งคำสั่งห่างกันนานสุด {self.max_tick_gap * 1000:.0f} ms', flush=True)
        try:
            # ชะลอหยุด ไม่ตัด ENA หุ่นจะได้ไม่ไหลตอนจอดบนทางลาด
            with self.ser_lock:
                if self.ser is not None and self.board_ok:
                    self.ser.write(b'S\n')
                    self.ser.flush()           # รอให้ S ออกจาก buffer จริงก่อนปิดพอร์ต
        except Exception:
            pass    # พอร์ตหลุดไปแล้ว เฟิร์มแวร์จะเบรกเองเมื่อไม่มี V เกิน 300 ms
        finally:
            self.close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumBase()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C ใน terminal (หรือ timeout -s INT) ส่ง SIGINT ให้ทั้ง process group แล้ว launch ส่งซ้ำอีกลูก
        # ลูกที่สองมาตอนกำลังเก็บกวาด -> KeyboardInterrupt กลาง destroy_node  ช่วงนี้ไม่รับ SIGINT แล้ว
        # (SIGTERM จาก launch/systemd ยังฆ่าได้ตามปกติ ถ้าเก็บกวาดค้าง)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
