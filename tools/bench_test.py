#!/usr/bin/env python3
"""
ทดสอบบอร์ด esp32_dm556_stepper บนโต๊ะ โดยไม่ต้องมี ROS

ตัวอย่าง:
    python3 bench_test.py --port /dev/ttyUSB0 --axis 0 --steps 3200
    python3 bench_test.py --port COM5 --axis 1 --steps 800 --vmax 1000 --repeat 5
    python3 bench_test.py --port /dev/ttyUSB0 --axis 0 --home
    python3 bench_test.py --port /dev/ttyUSB0 --monitor        # ดู status อย่างเดียว
"""

import argparse
import sys
import threading
import time

import serial

AF = {0x01: 'moving', 0x02: 'homed', 0x04: 'limit', 0x08: 'fault', 0x10: 'homing'}


def decode_flags(f):
    on = [name for bit, name in AF.items() if f & bit]
    return ','.join(on) if on else '-'


class Board:
    def __init__(self, port, baud=115200, verbose=False):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(2.0)                 # ESP32 รีเซ็ตตอนเปิดพอร์ต ต้องรอบูต
        self.ser.reset_input_buffer()
        self.verbose = verbose
        self.pos = [0] * 4
        self.vel = [0] * 4
        self.flags = [0] * 4
        self.sys = 0
        self.running = True
        threading.Thread(target=self._rx, daemon=True).start()

    def _rx(self):
        buf = b''
        while self.running:
            try:
                chunk = self.ser.read(256)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                self._line(raw.decode('ascii', 'ignore').strip())

    def _line(self, line):
        if not line:
            return
        if line.startswith('S '):
            f = line.split()
            if len(f) == 15:
                try:
                    self.sys = int(f[2])
                    for i in range(4):
                        self.pos[i] = int(f[3 + i * 3])
                        self.vel[i] = int(f[4 + i * 3])
                        self.flags[i] = int(f[5 + i * 3])
                except ValueError:
                    pass
            if self.verbose:
                print(line)
            return
        print(f'  <- {line}')

    def send(self, line):
        if self.verbose:
            print(f'  -> {line}')
        self.ser.write((line + '\n').encode('ascii'))
        time.sleep(0.02)

    def heartbeat_forever(self):
        def beat():
            while self.running:
                try:
                    self.send('P')
                except serial.SerialException:
                    return
                time.sleep(0.1)
        threading.Thread(target=beat, daemon=True).start()

    def wait_idle(self, axis, timeout=60.0):
        deadline = time.time() + timeout
        time.sleep(0.2)
        while time.time() < deadline:
            if not (self.flags[axis] & 0x01):
                return True
            time.sleep(0.05)
        return False

    def close(self):
        self.running = False
        try:
            self.send('S')
            time.sleep(0.1)
            self.ser.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True)
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--axis', type=int, default=0, choices=range(4))
    ap.add_argument('--steps', type=int, default=1600, help='ระยะที่จะขยับ (step)')
    ap.add_argument('--vmax', type=int, default=2000, help='step/s')
    ap.add_argument('--accel', type=int, default=4000, help='step/s^2')
    ap.add_argument('--repeat', type=int, default=1, help='ไป-กลับกี่รอบ')
    ap.add_argument('--home', action='store_true', help='สั่ง home แกนนี้แทนการขยับ')
    ap.add_argument('--monitor', action='store_true', help='ดู status อย่างเดียว ไม่สั่งอะไร')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    b = Board(args.port, args.baud, args.verbose)
    b.heartbeat_forever()

    try:
        if args.monitor:
            print('อ่าน status อย่างเดียว กด Ctrl-C เพื่อออก')
            while True:
                time.sleep(0.5)
                print(f'sys={b.sys}  ' + '  '.join(
                    f'A{i}: {b.pos[i]:>8} step {b.vel[i]:>7} st/s [{decode_flags(b.flags[i])}]'
                    for i in range(4)))

        b.send('C')
        b.send('E 1')
        print('จ่ายไฟมอเตอร์แล้ว -- ลองหมุนเพลาด้วยมือ ต้องล็อกแน่น')
        print('ถ้ายังหมุนฟรี แปลว่าขั้ว ENA กลับ ให้แก้ ENABLE_ACTIVE_LOW ใน .ino')
        time.sleep(2.0)

        if args.home:
            print(f'home แกน {args.axis} ...')
            b.send(f'H {args.axis} -1')
            deadline = time.time() + 40
            while time.time() < deadline:
                f = b.flags[args.axis]
                if f & 0x08:
                    print('home ไม่สำเร็จ (fault)')
                    break
                if (f & 0x02) and not (f & 0x10):
                    print(f'home เสร็จ ตำแหน่ง = {b.pos[args.axis]}')
                    break
                time.sleep(0.1)
            else:
                print('home ไม่ทันเวลา')
        else:
            b.send(f'V {args.axis} {args.vmax} {args.accel}')
            start = b.pos[args.axis]
            for n in range(args.repeat):
                for target in (start + args.steps, start):
                    print(f'  รอบ {n + 1}: ไป {target} ...', end=' ', flush=True)
                    t0 = time.time()
                    b.send(f'M {args.axis} {target}')
                    ok = b.wait_idle(args.axis)
                    err = b.pos[args.axis] - target
                    print(f'ถึงที่ {b.pos[args.axis]} (คลาด {err} step) '
                          f'ใช้เวลา {time.time() - t0:.2f} s'
                          + ('' if ok else '  << ไม่หยุดในเวลาที่รอ'))
                    if b.flags[args.axis] & 0x08:
                        print('  แกนนี้เข้าสถานะ fault -- น่าจะชนลิมิตสวิตช์')
                        break

    except KeyboardInterrupt:
        print('\nหยุด')
    finally:
        b.send('S')
        time.sleep(0.3)
        b.send('E 0')
        b.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
