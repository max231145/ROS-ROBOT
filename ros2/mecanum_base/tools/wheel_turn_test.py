"""wheel_turn_test -- เทสล้อหมุนสุทธิ สั่งบอร์ดตรง ไม่ผ่าน ROS  รันได้ทั้งโน้ตบุ๊กและ Pi

    python wheel_turn_test.py                # หาบอร์ด CH9102 เอง
    python wheel_turn_test.py COM4           # ระบุพอร์ตเอง
    python wheel_turn_test.py --ch 0         # เฉพาะช่องซ้าย (ค่าตั้งต้น = ทั้งสองช่อง)
    python wheel_turn_test.py --rate 50 --dur 30

Windows ถ้าไม่มี pyserial ใช้  %USERPROFILE%\.platformio\penv\Scripts\python.exe
แปะเทปที่ล้อก่อนรัน -- ตัวนับ step ของบอร์ดมองไม่เห็นว่าเพลาหมุนจริงหรือไม่
"""
import argparse, sys, time

import serial
from serial.tools import list_ports

PI_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58EA045139-if00'
BOARD_VID, BOARD_SERIAL = 0x1A86, '58EA045139'


def find_port():
    for p in list_ports.comports():
        if p.serial_number == BOARD_SERIAL or p.vid == BOARD_VID:
            return p.device
    import os
    if os.path.exists(PI_PORT):
        return PI_PORT
    listed = ', '.join(f'{p.device} ({p.description})' for p in list_ports.comports())
    sys.exit(f'หาบอร์ดไม่เจอ พอร์ตที่มี: {listed or "ไม่มีเลย"}\nระบุพอร์ตเองได้ เช่น COM4')


ap = argparse.ArgumentParser()
ap.add_argument('port', nargs='?')
ap.add_argument('--ch', type=int, choices=(0, 1), help='ทดสอบช่องเดียว (0=ซ้าย 1=ขวา)')
ap.add_argument('--rate', type=int, default=50, help='step/s (ค่าตั้งต้น 50 = ช้ามาก เห็นการคืบ)')
ap.add_argument('--dur', type=float, default=30.0, help='วินาทีต่อช่วง')
args = ap.parse_args()

port = args.port or find_port()
s = serial.Serial()
s.port, s.baudrate, s.timeout = port, 115200, 0.02
s.dtr = False       # CH9102: pyserial ค่าตั้งต้นกดขา EN ค้าง ดู memory esp32-ch9102-dtr-rts-reset
s.rts = False
s.open()

t0 = time.time()
buf = b''
w = None


def say(m):
    print(f'{time.time() - t0:6.1f}  {m}', flush=True)


def pump():
    global buf, w
    buf += s.read(4096)
    while b'\n' in buf:
        raw, buf = buf.split(b'\n', 1)
        line = raw.decode('utf-8', 'replace').strip()
        tok = line.split()
        if len(tok) == 11 and tok[0] == 'W':
            w = (int(tok[2]), [int(x) for x in tok[3:7]])
        elif line:
            say(f'board: {line}')


def hold(v, dur):
    end = time.time() + dur
    while time.time() < end:
        s.write(('V ' + ' '.join(str(x) for x in v) + '\n').encode())
        pump()
        time.sleep(0.05)


def wheels(sign):
    r = args.rate * sign
    if args.ch == 0:
        return (r, 0, 0, 0)
    if args.ch == 1:
        return (0, r, 0, 0)
    return (r, r, 0, 0)


try:
    say(f'ต่อบอร์ดที่ {port}')
    end = time.time() + 3
    while time.time() < end:
        pump()
    s.write(b'F 10\nA 3200 2000\nE 1\n')
    end = time.time() + 1
    while time.time() < end:
        pump()
    if w is None:
        sys.exit('ไม่ได้รับสถานะ W -- เฟิร์มแวร์บนบอร์ดไม่ใช่ esp32_mecanum_base')
    say(f'เปิดไดรเวอร์ sys=0x{w[0]:02x}')

    say(f'== 1) เดินหน้า {args.dur:.0f} วิ -- เทปควรคืบไป {args.rate * args.dur / 1600:.2f} รอบล้อ')
    hold(wheels(+1), args.dur)
    say(f'   step={w[1]}')
    say('== พัก 5 วิ -- จำตำแหน่งเทป')
    hold((0, 0, 0, 0), 5.0)
    say(f'== 2) ถอยหลัง {args.dur:.0f} วิ -- เทปควรกลับมาที่เดิม')
    hold(wheels(-1), args.dur)
    say(f'   step={w[1]}  (ควรใกล้ 0)')
finally:
    s.write(b'V 0 0 0 0\nE 0\n')
    time.sleep(0.3)
    pump()
    say('ปิดไดรเวอร์แล้ว -- จบ')
    s.close()
