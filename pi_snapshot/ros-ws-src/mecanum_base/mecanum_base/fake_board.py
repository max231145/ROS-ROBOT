#!/usr/bin/env python3
"""
fake_board -- จำลองเฟิร์มแวร์ esp32_mecanum_base บน pseudo-terminal (Linux เท่านั้น)
ใช้ทดสอบ node mecanum_base / Nav2 ได้ทั้งระบบโดยไม่ต้องเสียบ ESP32 หรือยกหุ่นขึ้นลอย

  ros2 run mecanum_base fake_board --link /tmp/fake_mecanum
  ros2 launch mecanum_base mecanum_base.launch.py port:=/tmp/fake_mecanum

พฤติกรรมเลียนแบบเฟิร์มแวร์: คำสั่ง V/E/S/X/C/A/F/Z/Q, ramp แบบซิงค์ 4 ล้อ,
ชะลอหยุดเมื่อไม่มี V เกิน 300 ms, บรรทัดสถานะ W
"""

import argparse
import math
import os
import pty
import select
import signal
import sys
import time
import tty

WHEEL_COUNT = 4
RATE_MAX = 10000.0
CMD_TIMEOUT = 0.3
SYS_ENABLED, SYS_FAULT, SYS_TIMEOUT = 0x01, 0x02, 0x04


class FakeBoard:

    def __init__(self):
        self.t0 = time.monotonic()
        self.posf = [0.0] * WHEEL_COUNT
        self.v = [0.0] * WHEEL_COUNT
        self.target = [0.0] * WHEEL_COUNT
        self.vmax = 4000.0
        self.accel = 2000.0
        self.status_hz = 20
        self.enabled = False
        self.fault = False
        self.timed_out = True
        self.last_cmd = self.t0
        self.max_gap = 0.0                # ช่วงห่างของคำสั่ง V ที่นานที่สุด (s) ไว้ดูว่า node สะดุดไหม
        self.slow_gaps = 0                # จำนวนครั้งที่ V ห่างเกิน 100 ms

    def hard_stop(self):
        self.v = [0.0] * WHEEL_COUNT
        self.target = [0.0] * WHEEL_COUNT

    def handle(self, line: str):
        """คืนบรรทัดตอบกลับ หรือ None"""
        f = line.split()
        if not f:
            return None
        cmd, args = f[0], f[1:]
        try:
            nums = [int(a) for a in args]
        except ValueError:
            return 'ERR arg'

        if cmd == 'V':
            if len(nums) != WHEEL_COUNT:
                return 'ERR arg'
            now = time.monotonic()
            if self.last_cmd != self.t0:        # node ส่ง V ต่อเนื่องแม้หยุดนิ่ง ช่วงห่าง = node สะดุด
                gap = now - self.last_cmd
                self.max_gap = max(self.max_gap, gap)
                self.slow_gaps += gap > 0.1
            self.last_cmd = now
            self.timed_out = False
            if self.fault or not self.enabled:
                return None
            peak = max(abs(n) for n in nums)
            cap = min(self.vmax, RATE_MAX)
            s = cap / peak if peak > cap else 1.0
            self.target = [n * s for n in nums]
            return None
        if cmd == 'E':
            if len(nums) != 1:
                return 'ERR arg'
            if nums[0] and self.fault:
                return 'ERR fault'
            self.hard_stop()
            self.enabled = bool(nums[0])
        elif cmd == 'S':
            self.target = [0.0] * WHEEL_COUNT
        elif cmd == 'X':
            self.hard_stop()
            self.fault = True
            self.enabled = False
            return '# ESTOP host'
        elif cmd == 'C':
            self.fault = False
            self.hard_stop()
        elif cmd == 'A':
            if len(nums) != 2 or nums[0] <= 0 or nums[1] <= 0:
                return 'ERR arg'
            self.vmax = min(float(nums[0]), RATE_MAX)
            self.accel = float(nums[1])
        elif cmd == 'F':
            if len(nums) != 1 or not 0 <= nums[0] <= 200:
                return 'ERR arg'
            self.status_hz = nums[0]
        elif cmd == 'Z':
            self.posf = [0.0] * WHEEL_COUNT
        elif cmd == 'Q':
            return self.status()
        else:
            return 'ERR cmd'
        return f'OK {cmd}'

    def step(self, dt: float):
        """คืนข้อความ log แบบเดียวกับเฟิร์มแวร์ หรือ None"""
        msg = None
        if not self.timed_out and time.monotonic() - self.last_cmd > CMD_TIMEOUT:
            self.timed_out = True
            if any(self.target) or any(self.v):
                msg = '# cmd timeout, braking'
            self.target = [0.0] * WHEEL_COUNT
        if self.fault or not self.enabled:
            self.hard_stop()
            return msg

        max_dv = max(abs(t - v) for t, v in zip(self.target, self.v))
        limit = self.accel * dt
        s = limit / max_dv if max_dv > limit else 1.0
        for i in range(WHEEL_COUNT):
            self.v[i] = self.target[i] if s >= 1.0 else self.v[i] + (self.target[i] - self.v[i]) * s
            if abs(self.v[i]) >= 1.0:
                self.posf[i] += self.v[i] * dt
        return msg

    def status(self) -> str:
        sys_flags = ((SYS_ENABLED if self.enabled else 0) | (SYS_FAULT if self.fault else 0) |
                     (SYS_TIMEOUT if self.timed_out else 0))
        ms = int((time.monotonic() - self.t0) * 1000.0) & 0xFFFFFFFF
        pos = ' '.join(str(math.floor(p)) for p in self.posf)
        vel = ' '.join(str(int(v)) for v in self.v)
        return f'W {ms} {sys_flags} {pos} {vel}'


def main():
    ap = argparse.ArgumentParser(description='จำลองบอร์ด esp32_mecanum_base บน pty')
    ap.add_argument('--link', default='', help='สร้าง symlink ชี้ไปที่ pty เช่น /tmp/fake_mecanum')
    ap.add_argument('--verbose', action='store_true', help='พิมพ์คำสั่งที่ได้รับ (ยกเว้น V)')
    args, _ = ap.parse_known_args()      # ros2 run อาจแถม --ros-args มา

    # ปิดด้วย kill / launch ก็ต้องลบ symlink ด้วย และถ้าถูกรันเป็น background job
    # bash จะตั้ง SIGINT เป็น ignore ไว้ Ctrl+C / kill -INT จะไม่มีผลถ้าไม่ตั้งกลับ
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    if args.link:
        if os.path.islink(args.link):
            os.unlink(args.link)
        os.symlink(path, args.link)
    print(f'fake esp32_mecanum_base อยู่ที่ {path}' + (f' ({args.link})' if args.link else ''),
          flush=True)

    def write(line: str):
        try:
            os.write(master, (line + '\n').encode('ascii'))
        except BlockingIOError:
            pass                          # ไม่มีใครเปิดพอร์ตอ่าน buffer เต็ม ทิ้งไป

    board = FakeBoard()
    write('# esp32_mecanum_base ready wheels=4')
    buf = b''
    last = last_status = time.monotonic()
    max_loop = 0.0                        # loop ของตัวจำลองเองค้างนานสุด แยกว่าใครสะดุด node หรือตัวนี้
    try:
        while True:
            ready, _, _ = select.select([master], [], [], 0.001)
            if ready:
                try:
                    buf += os.read(master, 4096)
                except (BlockingIOError, OSError):
                    pass
                while b'\n' in buf:
                    raw, buf = buf.split(b'\n', 1)
                    line = raw.decode('ascii', 'ignore').strip()
                    if args.verbose and line and not line.startswith('V'):
                        print(f'<- {line}', flush=True)
                    reply = board.handle(line)
                    if reply:
                        write(reply)

            now = time.monotonic()
            max_loop = max(max_loop, now - last)
            msg = board.step(now - last)
            last = now
            if msg:
                write(msg)
                if args.verbose:
                    print(f'{msg[2:]} (V ห่างสุดที่เคยเห็น {board.max_gap * 1000:.0f} ms)', flush=True)
            if board.status_hz and now - last_status >= 1.0 / board.status_hz:
                last_status = now
                write(board.status())
    except KeyboardInterrupt:
        pass
    finally:
        if args.verbose:
            print(f'คำสั่ง V ห่างกันนานสุด {board.max_gap * 1000:.0f} ms  เกิน 100 ms {board.slow_gaps} ครั้ง '
                  f'(เฟิร์มแวร์เบรกเองที่ {CMD_TIMEOUT * 1000:.0f} ms)  '
                  f'loop ตัวจำลองค้างนานสุด {max_loop * 1000:.0f} ms', flush=True)
        if args.link and os.path.islink(args.link):
            os.unlink(args.link)


if __name__ == '__main__':
    main()
