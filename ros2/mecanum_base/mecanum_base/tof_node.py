#!/usr/bin/env python3
"""
tof_node -- อ่านเซนเซอร์วัดระยะ Waveshare TOF Laser Range Sensor (D) (GY-TOF10M) ทาง UART ของ Pi
แล้วส่ง sensor_msgs/Range ให้ costmap (RangeSensorLayer) และ collision_monitor (source แบบ range)

  ต่อสาย (Pi 5 หัว 40 ขา): GND->6  VIN->4 (5V)  RC->8 (GPIO14 TX)  DT->10 (GPIO15 RX)  PS/I-O ปล่อยลอย (= UART)
  เปิดพอร์ต: dtoverlay=uart0-pi5 ใน /boot/firmware/config.txt แล้วรีบูต -> /dev/ttyAMA0

  publish  /tof_range   sensor_msgs/Range (frame tof_link)
           TF คงที่ base_link -> tof_link จากพารามิเตอร์ tof_x tof_y tof_z tof_pitch_deg (ลบ = เงยขึ้น)

โปรโตคอล NLink (TOFSense ของ Nooploop ที่ Waveshare ใช้) เฟรม 16 byte:
  [0]=0x57 [1]=0x00 [2]=reserved [3]=id [4:8]=system_time ms (uint32 LE)
  [8:11]=dis (int24 LE, mm) [11]=dis_status [12:14]=signal_strength (uint16 LE) [14]=reserved [15]=checksum (ผลรวม [0:15] & 0xFF)
baud ค่าตั้งต้นของรุ่นนี้ยังไม่ยืนยัน -> ลองหลายค่าจนเจอเฟรมที่ checksum ถูก
!! ยังไม่ได้เทสกับตัวจริง -- ตอนต่อครั้งแรกดู log "เฟรมแรก" ว่า dis/status สมเหตุสมผลไหม !!
"""

import math
import signal
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from tf2_ros import StaticTransformBroadcaster

FRAME_LEN = 16
HEADER = (0x57, 0x00)


def parse_frames(buf):
    """ตัดเฟรมที่ถูกต้องออกจาก bytearray -> (list ของ (dis_mm, status, strength), ส่วนที่เหลือ)"""
    out = []
    i = 0
    while i + FRAME_LEN <= len(buf):
        if buf[i] != HEADER[0] or buf[i + 1] != HEADER[1]:
            i += 1
            continue
        f = buf[i:i + FRAME_LEN]
        if sum(f[:15]) & 0xFF != f[15]:
            i += 1
            continue
        dis = int.from_bytes(f[8:11], 'little', signed=True)
        out.append((dis, f[11], int.from_bytes(f[12:14], 'little')))
        i += FRAME_LEN
    return out, buf[i:]


class TofNode(Node):

    def __init__(self):
        super().__init__('tof_node')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.port = p('port', '/dev/ttyAMA0')
        self.bauds = [int(b) for b in p('bauds', [921600, 115200, 460800, 230400, 256000])]
        self.frame_id = p('frame_id', 'tof_link')
        self.fov = math.radians(float(p('fov_deg', 5.0)))
        self.min_range = float(p('min_range', 0.03))
        self.max_range = float(p('max_range', 3.0))     # ไกลกว่านี้ = "ไม่เจอ" (costmap ล้างตามลำแสง)
        self.valid_status = [int(s) for s in p('valid_status', [0])]
        self.rate_hz = float(p('publish_hz', 20.0))
        tf = TransformStamped()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = self.frame_id
        # ค่าประมาณก่อนผู้ใช้วัด: หน้าหุ่นตรงกลาง สูง 25 cm เงย 12 องศา
        tf.transform.translation.x = float(p('tof_x', 0.25))
        tf.transform.translation.y = float(p('tof_y', 0.0))
        tf.transform.translation.z = float(p('tof_z', 0.25))
        pitch = math.radians(float(p('tof_pitch_deg', -12.0)))
        tf.transform.rotation.y = math.sin(pitch / 2.0)
        tf.transform.rotation.w = math.cos(pitch / 2.0)
        tf.header.stamp = self.get_clock().now().to_msg()
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(tf)

        self.pub = self.create_publisher(Range, '/tof_range', qos_profile_sensor_data)
        self.ser = None
        self.buf = bytearray()
        self.last_pub = 0.0
        self.frames = 0
        self.bad_status = 0
        self.warned_port = False
        self.create_timer(0.01, self.poll)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'รออ่าน ToF ที่ {self.port} (ลอง baud {self.bauds})')

    def open_port(self):
        for baud in self.bauds:
            try:
                s = serial.Serial(self.port, baud, timeout=0)
            except (serial.SerialException, OSError) as e:
                if not self.warned_port:
                    self.get_logger().warn(f'เปิด {self.port} ไม่ได้: {e} -- ต่อสาย/เปิด uart0 แล้วหรือยัง (จะลองใหม่ทุก 5 s)')
                    self.warned_port = True
                return
            end = time.monotonic() + 0.6
            data = bytearray()
            while time.monotonic() < end:
                data += s.read(512)
                time.sleep(0.02)
            frames, _ = parse_frames(data)
            if frames:
                self.ser = s
                self.get_logger().info(
                    f'เจอ ToF ที่ baud {baud}: เฟรมแรก dis={frames[0][0]} mm status={frames[0][1]} '
                    f'strength={frames[0][2]} ({len(frames)} เฟรมใน 0.6 s)')
                return
            s.close()
            self.get_logger().debug(f'baud {baud}: ได้ {len(data)} byte ไม่มีเฟรมที่ถูก')
        self.get_logger().warn(f'{self.port}: ไม่เจอเฟรม NLink ที่ baud ไหนเลย (ไม่ได้ต่อ? RC/DT สลับกัน? ขา PS ต้องปล่อยลอย) '
                               f'-- ลองต่อไปเรื่อย ๆ', throttle_duration_sec=60.0)

    def poll(self):
        if self.ser is None:
            now = time.monotonic()
            if now - getattr(self, '_last_try', 0.0) < 5.0:
                return
            self._last_try = now
            self.open_port()
            return
        try:
            self.buf += self.ser.read(1024)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'อ่าน ToF ไม่ได้: {e} -- จะเปิดใหม่')
            self.ser.close()
            self.ser = None
            return
        frames, self.buf = parse_frames(self.buf)
        if not frames:
            return
        self.frames += len(frames)
        dis, status, _strength = frames[-1]
        now = time.monotonic()
        if now - self.last_pub < 1.0 / self.rate_hz:
            return
        self.last_pub = now
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        r = dis / 1000.0
        if status not in self.valid_status or r <= 0.0:
            self.bad_status += 1
            r = float('inf')                      # ไม่เจออะไร / ค่าเชื่อไม่ได้
        msg.range = min(max(r, self.min_range), self.max_range) if math.isfinite(r) else self.max_range
        self.last_range = r
        self.pub.publish(msg)

    def report(self):
        if self.ser is not None:
            r = getattr(self, 'last_range', float('nan'))
            self.get_logger().info(f'ToF {self.frames / 5.0:.0f} เฟรม/s  ล่าสุด {r:.3f} m  status ไม่ผ่าน {self.bad_status}',
                                   throttle_duration_sec=30.0)
            self.frames = 0
            self.bad_status = 0


def main(args=None):
    rclpy.init(args=args)
    node = TofNode()
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
