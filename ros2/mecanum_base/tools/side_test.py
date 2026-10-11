"""side_test -- เทสต่อ DM556 สองตัว (ซ้าย/ขวา) ยกล้อลอย สั่งบอร์ดตรง ไม่ผ่าน node ไม่ต้องใช้ ROS

รันได้ทั้งบนโน้ตบุ๊ก (เสียบ ESP32 เข้า USB โน้ตบุ๊ก) และบน Pi (ต้องปิด mecanum_base_node ก่อน พอร์ตใช้ร่วมกันไม่ได้)
    python side_test.py --dry          # ไม่ต่อบอร์ด ดูลำดับขั้นก่อน (เร่งเวลา 5 เท่า)
    python side_test.py                # หาบอร์ด ESP32 (ชิป CH9102) เอง
    python side_test.py COM5           # ระบุพอร์ตเอง
    python side_test.py --start 2      # เริ่มที่ขั้น 2
Windows ถ้าไม่มี pyserial ใช้ Python ของ PlatformIO:  %USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe side_test.py
Ctrl+C ได้ทุกเมื่อ จะสั่ง V 0 + E 0 ปิดไดรเวอร์ให้เสมอ   log: <temp>/side_test.log

"หมุนไปหน้า" = ผิวล้อด้านบนวิ่งไปทางหัวหุ่น (ถ้าวางบนพื้นจะพาหุ่นไปข้างหน้า)
ผิดแบบไหนแก้อะไร: tools/sim_two_drivers.py หรือหน้าจำลอง
"""
import argparse
import os
import signal
import sys
import tempfile
import time

PI_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58EA045139-if00'
BOARD_VID, BOARD_SERIAL = 0x1A86, '58EA045139'
RATE = 400          # step/s = 1/4 รอบ/s มองทันแต่เห็นชัด
SPIN = 5.0          # s ต่อขั้น
PAUSE = 8.0         # s พักให้จดผลก่อนขั้นถัดไป
LOCK = 18.0         # s ขั้นล็อกล้อ ให้เวลาลองหมุนล้อด้วยมือครบ 4 ล้อ
SYS_ENABLED, SYS_FAULT = 0x01, 0x02
PLAN = [  # (ขั้น, คำสั่ง V ช่อง 0-3, สิ่งที่ต้องเห็น)
    ('1 ล็อกล้อ', (0, 0, 0, 0),
     'ลองหมุนล้อด้วยมือทีละล้อ ต้องฝืนทั้ง 4 ล้อ (ข้างไหนหมุนฟรี = ENA/ไฟ 24V ข้างนั้น)'),
    ('2 ช่อง 0 อย่างเดียว', (RATE, 0, 0, 0),
     'หน้าซ้าย + หลังซ้าย หมุนไปหน้าทั้งคู่ · ข้างขวานิ่ง'),
    ('3 ช่อง 1 อย่างเดียว', (0, RATE, 0, 0),
     'หน้าขวา + หลังขวา หมุนไปหน้าทั้งคู่ · ข้างซ้ายนิ่ง'),
    ('4 เดินหน้า', (RATE, RATE, RATE, RATE),
     'ทั้ง 4 ล้อหมุนไปหน้า ความเร็วเท่ากัน'),
    ('5 หมุนซ้าย', (-RATE, RATE, -RATE, RATE),
     'ข้างซ้ายหมุนถอย · ข้างขวาหมุนไปหน้า'),
]
DIAGNOSE = """
ผลไม่ตรง ดูตรงนี้ (ภาพเคลื่อนไหวของแต่ละกรณีอยู่ในหน้าจำลอง):
  ขั้น 2 ข้างขวาหมุนแทน ขั้น 3 ข้างซ้ายหมุนแทน  -> สลับข้าง: สลับขั้วต่อ PUL/DIR ระหว่างสองไดรเวอร์ (config แก้ไม่ได้)
  ขั้น 2 ซ้ายหมุนถอยทั้งคู่                    -> invert_wheels ช่อง 0 และ 2 เป็น true
  ขั้น 3 ขวาหมุนถอยทั้งคู่                     -> invert_wheels ช่อง 1 และ 3 เป็น true
  ข้างเดียวกันหมุนสวนกันเอง                     -> สลับคอยล์ A+/A- ของมอเตอร์ตัวที่หมุนถอย (config แก้ไม่ได้)
  ข้างหนึ่งช้ากว่าอีกข้างในขั้น 4                -> DIP microstep สองไดรเวอร์ไม่ตรงกัน
  ข้างไหนไม่หมุนเลย แต่ตัวนับช่องนั้นขึ้น          -> ไฟ 24V / ALM / สาย PUL-DIR ข้างนั้น
"""


class FakeBoard:
    """--dry: ทำตัวเหมือนเฟิร์มแวร์พอให้เห็นลำดับขั้นและตัวนับ ไม่มีมอเตอร์หมุนจริง"""

    def __init__(self):
        self.t0 = time.monotonic()
        self.out = b'# esp32_mecanum_base ready wheels=4 (dry run)\n'
        self.v, self.pos, self.en, self.last = [0] * 4, [0.0] * 4, False, time.monotonic()

    def write(self, data):
        for line in data.decode('ascii').splitlines():
            tok = line.split()
            if tok[0] == 'E':
                self.en = tok[1] == '1'
            elif tok[0] == 'V':
                self.v = [int(x) for x in tok[1:5]] if self.en else [0] * 4

    def read(self, _n):
        now = time.monotonic()
        dt, self.last = (now - self.last) * SPEEDUP, now
        self.pos = [p + v * dt for p, v in zip(self.pos, self.v)]
        ms = int((now - self.t0) * 1000)
        pos = ' '.join(str(int(p)) for p in self.pos)
        vel = ' '.join(str(v) for v in self.v)
        out, self.out = self.out + f'W {ms} {int(self.en)} {pos} {vel}\n'.encode(), b''
        return out

    def close(self):
        pass


def find_port():
    if os.path.exists(PI_PORT):
        return PI_PORT
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for want_serial in (True, False):
        for p in ports:
            if p.vid == BOARD_VID and (not want_serial or p.serial_number == BOARD_SERIAL):
                return p.device
    listed = ', '.join(f'{p.device} ({p.description})' for p in ports) or 'ไม่มีเลย'
    sys.exit(f'หาบอร์ด ESP32 (CH9102) ไม่เจอ -- เสียบ USB แล้วหรือยัง? พอร์ตที่เห็น: {listed}\n'
             f'ระบุเองได้: python side_test.py COM5')


ap = argparse.ArgumentParser(description='เทสต่อ DM556 ซ้าย/ขวา ทีละข้าง (ยกล้อลอยก่อน)')
ap.add_argument('port', nargs='?', help='พอร์ตบอร์ด เช่น COM5 หรือ /dev/ttyACM0 (ไม่ใส่ = หาเอง)')
ap.add_argument('--start', type=int, default=1, choices=range(1, len(PLAN) + 1), help='เริ่มที่ขั้นไหน')
ap.add_argument('--dry', action='store_true', help='ไม่ต่อบอร์ด ดูลำดับขั้นอย่างเดียว')
args = ap.parse_args()
SPEEDUP = 5.0 if args.dry else 1.0

log = open(os.path.join(tempfile.gettempdir(), 'side_test.log'), 'w', buffering=1, encoding='utf-8')
stop = False


def on_signal(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, on_signal)
signal.signal(signal.SIGTERM, on_signal)

T0 = time.monotonic()
buf = b''
w_last = None       # (sys, [pos 4 ช่อง])
ready = False


def say(msg=''):
    line = f'{time.strftime("%H:%M:%S")} {time.monotonic() - T0:6.1f}  {msg}' if msg else ''
    print(line, flush=True)
    log.write(line + '\n')


if args.dry:
    ser = FakeBoard()
    say('โหมด --dry ไม่ได้ต่อบอร์ด (เวลาเร็วขึ้น 5 เท่า)')
else:
    try:
        import serial
    except ImportError:
        sys.exit('ไม่มี pyserial -- pip install pyserial หรือใช้ Python ของ PlatformIO (ดูหัวไฟล์)')
    port = args.port or find_port()
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, 115200, 0.02
    ser.dtr = False     # CH9102: ค่าตั้งต้นของ pyserial กดขา EN ของ ESP32 ค้าง บอร์ดเงียบ
    ser.rts = False
    try:
        ser.open()
    except serial.SerialException as e:
        sys.exit(f'เปิด {port} ไม่ได้ ({e}) -- บน Pi ต้องปิด mecanum_base_node ก่อน')
    say(f'ต่อบอร์ดที่ {port}')


def pump():
    global buf, w_last, ready
    buf += ser.read(4096)
    while b'\n' in buf:
        raw, buf = buf.split(b'\n', 1)
        line = raw.decode('ascii', 'replace').strip()
        tok = line.split()
        if len(tok) == 11 and tok[0] == 'W':
            try:
                w_last = (int(tok[2]), [int(x) for x in tok[3:7]])
            except ValueError:
                pass
        elif line:
            ready = ready or 'ready' in line
            log.write(f'board: {line}\n')


def send(line):
    ser.write((line + '\n').encode('ascii'))


def wait(dur):
    end = time.monotonic() + dur / SPEEDUP
    while time.monotonic() < end and not stop:
        pump()
        time.sleep(0.02)


def hold(v, dur):
    """ส่ง V ซ้ำทุก 50 ms (บอร์ดเบรกเองถ้าเงียบเกิน 300 ms)"""
    end = time.monotonic() + dur / SPEEDUP
    while time.monotonic() < end and not stop:
        send('V ' + ' '.join(str(x) for x in v))
        pump()
        time.sleep(0.05)


def counts():
    return list(w_last[1]) if w_last else None


wait(4.0 if not args.dry else 0.5)
if not ready and w_last is None:
    say('บอร์ดยังไม่ส่งอะไรมา (ถ้าบอร์ดบูตไปก่อนแล้วไม่เป็นไร จะลองสั่งต่อ)')

try:
    send('F 10')
    send('A 3200 2000')
    send('E 1')
    wait(0.5)
    if w_last is None:
        say('ไม่ได้รับสถานะ W จากบอร์ดเลย -- พอร์ตผิดหรือเฟิร์มแวร์ไม่ใช่ esp32_mecanum_base หยุดเทส')
        sys.exit(1)
    if not w_last[0] & SYS_ENABLED or w_last[0] & SYS_FAULT:
        why = ('มี ESTOP ค้างอยู่ในบอร์ด (fault) -- เช็คว่าปลอดภัยแล้วกดปุ่ม EN ที่ ESP32 แล้วรันใหม่'
               if w_last[0] & SYS_FAULT else 'บอร์ดไม่ยอมเปิดไดรเวอร์')
        say(f'{why}  (sys={w_last[0]}) หยุดเทส')
        sys.exit(1)
    say('เปิดไดรเวอร์แล้ว -- ล้อต้องยกลอยจากพื้นทั้ง 4 ล้อ')
    for name, v, expect in PLAN[args.start - 1:]:
        if stop:
            break
        say()
        say(f'== ขั้น {name}   (V {" ".join(map(str, v))})')
        say(f'   ต้องเห็น: {expect}')
        for n in (3, 2, 1):
            if stop:
                break
            say(f'   เริ่มใน {n}')
            hold((0, 0, 0, 0), 1.0)
        before = counts()
        hold(v, LOCK if v == (0, 0, 0, 0) else SPIN)
        hold((0, 0, 0, 0), 0.6)
        after = counts()
        moved = '  '.join(f'ช่อง{i} {after[i] - before[i]:+d}' for i in range(4))
        say(f'   จบขั้น -- ตัวนับ step บอร์ด: {moved}')
        if name != PLAN[-1][0] and not stop:
            say(f'   พัก {PAUSE:.0f} s จดผล')
            hold((0, 0, 0, 0), PAUSE)
    if not stop:
        say(DIAGNOSE)
finally:
    send('V 0 0 0 0')
    wait(1.0)
    send('E 0')
    wait(0.3)
    say('ปิดไดรเวอร์แล้ว' + (' (หยุดกลางคัน)' if stop else ''))
    ser.close()
