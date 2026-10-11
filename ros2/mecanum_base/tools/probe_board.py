"""เช็คโปรโตคอลกับบอร์ดจริงโดยไม่เปิดไดรเวอร์ (ไม่ส่ง E 1 = ไม่มีพัลส์ มอเตอร์ไม่หมุน)  python probe_board.py COM4 หรือ /dev/ttyUSB1"""
import sys
import time

import serial

PORT = sys.argv[1]
FAILS = []


def check(name, cond, detail=''):
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}", flush=True)
    if not cond:
        FAILS.append(name)


# this board (CH9102) holds EN low while DTR+RTS are asserted, which is pyserial's default on open
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = PORT, 115200, 0.05
ser.dtr = False
ser.rts = False
ser.open()
ser.rts = True          # clean reset pulse so the boot banner check is deterministic
time.sleep(0.1)
ser.rts = False


def read_for(sec):
    lines, buf, end = [], b'', time.monotonic() + sec
    while time.monotonic() < end:
        buf += ser.read(512)
        while b'\n' in buf:
            raw, buf = buf.split(b'\n', 1)
            lines.append((time.monotonic(), raw.decode('ascii', 'replace').strip()))
    return lines


def status(lines):
    return [l for _, l in lines if l.startswith('W ')]


def last_w(lines):
    w = status(lines)
    return [int(x) for x in w[-1].split()[1:]] if w else None


def send(line):
    ser.write((line + '\n').encode('ascii'))


boot = read_for(2.5)
texts = [l for _, l in boot]
check('boot banner', any('esp32_mecanum_base ready wheels=4' in l for l in texts),
      next((l for l in texts if 'ready' in l), f'first lines: {texts[:3]}'))
w = status(boot)


def board_rate(wlines):
    ms = [int(l.split()[1]) for l in wlines]
    return (len(ms) - 1) * 1000.0 / (ms[-1] - ms[0]) if len(ms) > 2 else 0.0


# host receive times are bunched by the Windows USB serial driver -> use the board's own ms stamps
check('W at default 20 Hz (board clock)', 19.0 <= board_rate(w) <= 21.0, f'{board_rate(w):.2f} Hz from {len(w)} lines')
fields = w[-1].split() if w else []
check('W has 11 fields', len(fields) == 11, w[-1] if w else '')
s = last_w(boot)
check('boot state: disabled, no fault, timeout bit set', s and s[1] == 0x04, f'sys={s[1] if s else None}')

send('F 30')
lines = read_for(2.0)
check('F 30 -> OK', any(l == 'OK F' for _, l in lines))
rate = board_rate(status(lines)[2:])   # first lines may still be at 20 Hz
check('status rate 30 Hz (board clock)', 29.0 <= rate <= 31.0, f'{rate:.2f} Hz')
ms = [int(l.split()[1]) for _, l in lines if l.startswith('W ')]
check('board ms increasing', all(b > a for a, b in zip(ms, ms[1:])), f'{ms[0]}..{ms[-1]}')

send('A 3200 2000')
check('A -> OK', any(l == 'OK A' for _, l in read_for(0.3)))

# V while disabled: must be ignored (no motion), must clear timeout bit, then timeout bit returns after 300 ms
# stream off + Q right after V -> the only W line is guaranteed to be generated after V was handled
send('F 0')
read_for(0.5)
send('V 500 -500 500 -500')
send('Q')
s = last_w(read_for(0.3))
check('V while disabled: no velocity, no steps', s and s[2:] == [0] * 8, f'W={s}')
check('V clears timeout bit', s and not (s[1] & 0x04), f'sys={s[1] if s else None}')
time.sleep(0.5)
send('Q')
s = last_w(read_for(0.3))
check('timeout bit back after 300 ms (fixed bug)', s and (s[1] & 0x04), f'sys={s[1] if s else None}')
send('F 30')
read_for(0.3)

send('Q')
lines = read_for(0.2)
check('Q -> immediate W', len(status(lines)) >= 1)

send('X')
lines = read_for(0.3)
texts = [l for _, l in lines]
s = last_w(lines)
check('X -> ESTOP message + fault bit', any('ESTOP host' in l for l in texts) and s and (s[1] & 0x02),
      f'sys={s[1] if s else None}')
send('E 1')
lines = read_for(0.3)
s = last_w(lines)
check('E 1 refused while fault (driver stays off)', any(l == 'ERR fault' for _, l in lines) and s and not (s[1] & 0x01),
      f'sys={s[1] if s else None}')
send('C')
lines = read_for(0.3)
s = last_w(lines)
check('C clears fault, still disabled', any(l == 'OK C' for _, l in lines) and s and not (s[1] & 0x03),
      f'sys={s[1] if s else None}')

for bad, want in [('V 1 2 3', 'ERR arg'), ('V 1 2 3 x', 'ERR arg'), ('A 0 5', 'ERR arg'), ('F 999', 'ERR arg'),
                  ('HELLO', 'ERR cmd'), ('V ' + '9' * 120, 'ERR long'),
                  ('9' * 100 + ' X', 'ERR long')]:          # tail of an over-long line must not run as a command
    send(bad)
    lines = read_for(0.3)
    got = [l for _, l in lines if l.startswith('ERR') or 'ESTOP' in l]
    check(f'reject {bad[:14]!r}...', got == [want], f'got {got}')
s = last_w(lines) if last_w(lines) else None
send('Q')
s = last_w(read_for(0.3))
check('over-long line tail " X" did not trigger estop', s and not (s[1] & 0x02), f'sys={s[1] if s else None}')

send('Z')
check('Z -> OK', any(l == 'OK Z' for _, l in read_for(0.3)))

send('F 0')
lines = read_for(0.8)
check('F 0 stops status stream', len(status(lines[3:])) == 0, f'{len(status(lines))} W lines')
send('F 20')
read_for(0.2)

ser.close()
print(f'\n{len(FAILS)} FAIL(s): {FAILS}' if FAILS else '\nALL PASS', flush=True)
sys.exit(1 if FAILS else 0)
