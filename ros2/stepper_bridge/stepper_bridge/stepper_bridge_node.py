#!/usr/bin/env python3
"""
stepper_bridge -- สะพานระหว่าง ROS 2 กับเฟิร์มแวร์ esp32_dm556_stepper

หน้าที่หลักคือแปลงหน่วย: ฝั่ง ROS คิดเป็น rad (หรือ m ถ้าเป็นแกนเชิงเส้น)
ฝั่ง ESP32 คิดเป็น step ล้วน ๆ  node นี้แปลงให้ทั้งสองทาง

  subscribe  ~/command      sensor_msgs/JointState   (ใช้ field position, จับคู่ด้วย name)
  publish    /joint_states  sensor_msgs/JointState   (position + velocity ของจริงจากบอร์ด)
  publish    ~/flags        std_msgs/UInt8MultiArray (บิตสถานะต่อแกน ดู AF_* ในเฟิร์มแวร์)

  service    ~/enable       std_srvs/SetBool   เปิด/ปิดไฟเลี้ยงมอเตอร์
             ~/home         std_srvs/Trigger   home ทุกแกนที่เปิดใช้ ทีละแกน
             ~/stop         std_srvs/Trigger   เบรกแบบมี ramp
             ~/estop        std_srvs/Trigger   หยุดทันที + ตัด ENA
             ~/clear_fault  std_srvs/Trigger
             ~/zero         std_srvs/Trigger   เซ็ตตำแหน่งปัจจุบันเป็น 0 ทุกแกน
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import serial

from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import SetBool, Trigger

AXIS_COUNT = 4

# บิตสถานะต่อแกน ต้องตรงกับ AF_* ในไฟล์ .ino
AF_MOVING = 0x01
AF_HOMED = 0x02
AF_LIMIT = 0x04
AF_FAULT = 0x08
AF_HOMING = 0x10


class AxisCfg:
    """ค่าคงที่ของแกนหนึ่ง ๆ พร้อมตัวแปลงหน่วย step <-> rad/m"""

    def __init__(self, node: Node, idx: int):
        p = f'axis{idx}.'
        self.idx = idx
        self.name = node.declare_parameter(p + 'name', f'joint{idx}').value
        self.enabled = node.declare_parameter(p + 'enabled', True).value

        steps_per_rev = node.declare_parameter(p + 'steps_per_rev', 200).value
        microsteps = node.declare_parameter(p + 'microsteps', 8).value
        gear_ratio = node.declare_parameter(p + 'gear_ratio', 1.0).value
        # หน่วยของ joint ต่อการหมุน "ด้านออก" 1 รอบ
        #   แกนหมุน  -> 2*pi (rad)
        #   แกนเลื่อน -> ระยะ lead ของสกรู หน่วยเมตร เช่น 0.005 = สกรู 5 mm/รอบ
        units_per_rev = node.declare_parameter(p + 'units_per_rev', 2.0 * math.pi).value

        self.invert = node.declare_parameter(p + 'invert', False).value
        # offset: ค่า joint (rad/m) ที่ตรงกับตำแหน่ง step = 0 (คือจุด home)
        self.offset = node.declare_parameter(p + 'offset', 0.0).value

        self.max_vel = node.declare_parameter(p + 'max_vel', 3.0).value       # unit/s
        self.max_accel = node.declare_parameter(p + 'max_accel', 6.0).value   # unit/s^2
        self.limit_lo = node.declare_parameter(p + 'limit_lo', 0.0).value
        self.limit_hi = node.declare_parameter(p + 'limit_hi', 0.0).value     # lo==hi = ปิด soft limit
        self.home_dir = node.declare_parameter(p + 'home_dir', -1).value

        steps_per_output_rev = float(steps_per_rev) * float(microsteps) * float(gear_ratio)
        if units_per_rev == 0.0:
            raise ValueError(f'axis{idx}: units_per_rev เป็น 0 ไม่ได้')
        self.steps_per_unit = steps_per_output_rev / float(units_per_rev)
        if self.invert:
            self.steps_per_unit = -self.steps_per_unit

    def to_steps(self, value: float) -> int:
        return int(round((value - self.offset) * self.steps_per_unit))

    def to_units(self, steps: float) -> float:
        return steps / self.steps_per_unit + self.offset

    def rate_to_steps(self, rate: float) -> int:
        """ความเร็ว/ความเร่ง ไม่มีเรื่อง offset และต้องเป็นบวกเสมอ"""
        return max(1, int(round(abs(rate * self.steps_per_unit))))


class StepperBridge(Node):

    def __init__(self):
        super().__init__('stepper_bridge')

        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.status_hz = self.declare_parameter('status_hz', 20).value
        self.frame_id = self.declare_parameter('frame_id', '').value
        self.auto_enable = self.declare_parameter('auto_enable', False).value

        self.axes = [AxisCfg(self, i) for i in range(AXIS_COUNT)]
        self.active = [a for a in self.axes if a.enabled]
        self.by_name = {a.name: a for a in self.active}

        # สถานะล่าสุดที่อ่านได้จากบอร์ด
        self.lock = threading.Lock()
        self.pos_steps = [0] * AXIS_COUNT
        self.vel_steps = [0] * AXIS_COUNT
        self.flags = [0] * AXIS_COUNT
        self.sys_flags = 0
        self.last_rx = 0.0
        # เพดานความเร็วที่ยัดลงบอร์ดไว้ล่าสุด (step/s) ใช้กันส่งคำสั่ง V ซ้ำโดยไม่จำเป็น
        self.cur_vmax = [0] * AXIS_COUNT

        self.ser = None
        self.ser_lock = threading.Lock()
        self.running = True

        cbg = ReentrantCallbackGroup()

        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_flags = self.create_publisher(UInt8MultiArray, '~/flags', 10)
        self.create_subscription(JointState, '~/command', self.on_command, 10,
                                 callback_group=cbg)

        self.create_service(SetBool, '~/enable', self.srv_enable, callback_group=cbg)
        self.create_service(Trigger, '~/home', self.srv_home, callback_group=cbg)
        self.create_service(Trigger, '~/stop', self.srv_stop, callback_group=cbg)
        self.create_service(Trigger, '~/estop', self.srv_estop, callback_group=cbg)
        self.create_service(Trigger, '~/clear_fault', self.srv_clear, callback_group=cbg)
        self.create_service(Trigger, '~/zero', self.srv_zero, callback_group=cbg)

        self.open_serial()

        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # heartbeat กัน HOST_TIMEOUT_MS ของเฟิร์มแวร์เบรกมอเตอร์ทิ้งกลางทาง
        self.create_timer(0.1, self.tick_heartbeat, callback_group=cbg)
        self.create_timer(1.0 / max(1, self.status_hz), self.tick_publish, callback_group=cbg)

    # ------------------------------------------------------------------ serial
    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except serial.SerialException as e:
            self.get_logger().error(f'เปิด {self.port} ไม่ได้: {e}')
            self.ser = None
            return

        # ESP32 รีเซ็ตตอนเปิดพอร์ต (DTR) ต้องรอบูตก่อนค่อยยิงคำสั่ง
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.get_logger().info(f'ต่อกับ {self.port} @ {self.baud} แล้ว')
        self.push_config()

    def send(self, line: str):
        with self.ser_lock:
            if self.ser is None:
                return False
            try:
                self.ser.write((line + '\n').encode('ascii'))
                return True
            except serial.SerialException as e:
                self.get_logger().error(f'เขียนพอร์ตไม่ได้: {e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return False

    def set_vmax(self, a: AxisCfg, vel_units: float):
        """ตั้งเพดานความเร็วของแกน ส่งลงบอร์ดเฉพาะตอนค่าเปลี่ยนจริง"""
        steps = a.rate_to_steps(vel_units)
        if steps == self.cur_vmax[a.idx]:
            return
        if self.send(f'V {a.idx} {steps} {a.rate_to_steps(a.max_accel)}'):
            self.cur_vmax[a.idx] = steps

    def push_config(self):
        """ยัดค่า vmax/accel/soft limit ลงบอร์ด (บอร์ดไม่เก็บค่าข้ามการรีบูต)"""
        self.send(f'F {self.status_hz}')
        for a in self.active:
            self.cur_vmax[a.idx] = 0            # บังคับให้ส่งใหม่
            self.set_vmax(a, a.max_vel)
            if a.limit_lo != a.limit_hi:
                s1 = a.to_steps(a.limit_lo)
                s2 = a.to_steps(a.limit_hi)
                self.send(f'L {a.idx} {min(s1, s2)} {max(s1, s2)}')
        if self.auto_enable:
            self.send('E 1')

    def rx_loop(self):
        buf = b''
        while self.running:
            if self.ser is None:
                time.sleep(1.0)
                self.open_serial()
                continue
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f'อ่านพอร์ตไม่ได้: {e}')
                self.ser = None
                continue
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                self.handle_line(raw.decode('ascii', 'ignore').strip())

    def handle_line(self, line: str):
        if not line:
            return
        if line.startswith('#'):
            # ข้อความ log ของบอร์ด รวมถึงขยะตอนบูตของ ESP32 ที่ไม่ขึ้นต้นด้วย S
            self.get_logger().info(f'esp32: {line[1:].strip()}')
            return
        if line.startswith('ERR'):
            self.get_logger().warning(f'esp32: {line}')
            return
        if line.startswith('OK'):
            return
        if not line.startswith('S '):
            return

        f = line.split()
        # S <ms> <sys> + 3 ค่าต่อแกน
        if len(f) != 3 + 3 * AXIS_COUNT:
            return
        try:
            sysf = int(f[2])
            pos, vel, fl = [], [], []
            for i in range(AXIS_COUNT):
                pos.append(int(f[3 + i * 3]))
                vel.append(int(f[4 + i * 3]))
                fl.append(int(f[5 + i * 3]))
        except ValueError:
            return

        with self.lock:
            self.sys_flags = sysf
            self.pos_steps = pos
            self.vel_steps = vel
            self.flags = fl
            self.last_rx = time.time()

    # ----------------------------------------------------------------- ROS I/O
    def on_command(self, msg: JointState):
        if not msg.name:
            self.get_logger().warning('command ไม่มี field name -- ต้องระบุชื่อ joint')
            return

        targets = ['*'] * AXIS_COUNT
        for i, name in enumerate(msg.name):
            a = self.by_name.get(name)
            if a is None:
                continue
            if i >= len(msg.position):
                continue
            val = msg.position[i]
            if a.limit_lo != a.limit_hi:
                val = min(max(val, min(a.limit_lo, a.limit_hi)), max(a.limit_lo, a.limit_hi))
            targets[a.idx] = str(a.to_steps(val))

            # ถ้าส่ง velocity มาด้วยให้ใช้เป็นเพดานความเร็วของ move นี้
            # ถ้าไม่ส่งมา ต้องคืนกลับเป็น max_vel ไม่งั้นค่าที่เคยลดไว้จะค้างไปตลอด
            if i < len(msg.velocity) and msg.velocity[i] > 0.0:
                self.set_vmax(a, min(abs(msg.velocity[i]), a.max_vel))
            else:
                self.set_vmax(a, a.max_vel)

        if all(t == '*' for t in targets):
            self.get_logger().warning(f'ไม่มีชื่อ joint ที่รู้จักใน command: {list(msg.name)}')
            return
        self.send('MA ' + ' '.join(targets))

    def tick_heartbeat(self):
        self.send('P')
        with self.lock:
            age = time.time() - self.last_rx
        if self.last_rx > 0.0 and age > 1.0:
            self.get_logger().warning(f'ไม่ได้รับ status จากบอร์ดมา {age:.1f} s',
                                      throttle_duration_sec=5.0)

    def tick_publish(self):
        with self.lock:
            pos = list(self.pos_steps)
            vel = list(self.vel_steps)
            fl = list(self.flags)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = self.frame_id
        for a in self.active:
            js.name.append(a.name)
            js.position.append(a.to_units(pos[a.idx]))
            js.velocity.append(vel[a.idx] / a.steps_per_unit)
        self.pub_js.publish(js)

        m = UInt8MultiArray()
        m.data = [fl[a.idx] for a in self.active]
        self.pub_flags.publish(m)

    # --------------------------------------------------------------- services
    def srv_enable(self, req, res):
        ok = self.send('E 1' if req.data else 'E 0')
        res.success = ok
        if ok:
            res.message = ('เปิด' if req.data else 'ปิด') + 'ไฟเลี้ยงมอเตอร์แล้ว'
        else:
            res.message = 'ส่งคำสั่งไม่ได้ -- ยังไม่ได้ต่อกับบอร์ด'
        return res

    def srv_home(self, req, res):
        """home ทีละแกน รอให้แกนก่อนหน้าเสร็จก่อน (กันไฟตกเพราะขยับพร้อมกัน 4 ตัว)"""
        if self.ser is None:
            res.success = False
            res.message = 'ยังไม่ได้ต่อกับบอร์ด'
            return res

        self.send('E 1')
        for a in self.active:
            self.send(f'H {a.idx} {a.home_dir}')
            deadline = time.time() + 35.0
            time.sleep(0.3)
            while time.time() < deadline:
                with self.lock:
                    f = self.flags[a.idx]
                if f & AF_FAULT:
                    res.success = False
                    res.message = f'{a.name} home ไม่สำเร็จ'
                    return res
                if (f & AF_HOMED) and not (f & AF_HOMING):
                    break
                time.sleep(0.05)
            else:
                res.success = False
                res.message = f'{a.name} home ไม่ทันเวลา'
                return res

        self.push_config()   # home เสร็จแล้ว soft limit ในบอร์ดถูกล้าง ต้องยัดกลับ
        res.success = True
        res.message = 'home ครบทุกแกนแล้ว'
        return res

    def srv_stop(self, req, res):
        res.success = self.send('S')
        res.message = 'เบรกทุกแกน'
        return res

    def srv_estop(self, req, res):
        res.success = self.send('X')
        res.message = 'E-stop: หยุดทันทีและตัด ENA แล้ว ต้องเรียก clear_fault ก่อนใช้งานต่อ'
        return res

    def srv_clear(self, req, res):
        res.success = self.send('C')
        res.message = 'เคลียร์ fault แล้ว'
        return res

    def srv_zero(self, req, res):
        ok = True
        for a in self.active:
            ok = self.send(f'Z {a.idx} 0') and ok
        res.success = ok
        res.message = 'เซ็ตตำแหน่งปัจจุบันเป็น 0 ทุกแกนแล้ว'
        return res

    # ----------------------------------------------------------------- cleanup
    def shutdown(self):
        self.running = False
        self.send('S')
        time.sleep(0.1)
        with self.ser_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = StepperBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
